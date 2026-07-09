from __future__ import annotations

from dataclasses import dataclass, field

from model.geom_utils import project_point_on_edge, tangent_along_edge
from model.rail_network import RailNetwork
from model.vec3 import Vec3


@dataclass
class SnapResult:
    """吸附系统的输出结果。

    切线相关字段说明：
    - `tangent`: 单一最佳切线方向（路径吸附时按 reference_pos 选；点吸附端点时唯一）
    - `tangent_candidates`: 该位置所有合法切线方向集合（用于 BUILD_ACTIVE 中
      根据 M2 动态选择最合适的方向）。空列表表示无切线约束（Case 1）。
    - `edge_direction`: 路径吸附到直边时的边方向向量（归一化）；用于 Case 2T
      （§10.5）算法输入。弧边或点吸附时 = None。
    """
    snapped: bool                                       # 是否发生了吸附
    position: Vec3                                      # 吸附后的世界坐标（无吸附时 = 原始光标位置）
    tangent: Vec3 | None = None                         # 该位置的最佳切线方向（无切线约束时 None）
    tangent_candidates: list[Vec3] = field(default_factory=list)  # 所有候选切线（按方向不同各算一个）
    snapped_node_id: int | None = None                  # 吸附到的 Node ID（点吸附时）
    snapped_edge_id: int | None = None                  # 吸附到的 Edge ID（路径吸附时）
    snapped_edge_t: float | None = None                 # 路径吸附时沿边的参数 t ∈ [0,1]
    edge_direction: Vec3 | None = None                  # 路径吸附到直边时的边方向（归一化），弧边/点吸附=None


class PointSnapProvider:
    """点吸附提供器：吸附到既有节点"""

    def __init__(self, threshold: float = 0.3) -> None:
        self.threshold = threshold
        self.enabled = True

    def snap(self, world_pos: Vec3, network: RailNetwork) -> SnapResult | None:
        """尝试吸附到最近的节点，返回 SnapResult 或 None（无命中）"""
        if not self.enabled:
            return None

        best_node_id: int | None = None
        best_dist = float('inf')

        # 走空间索引：只扫描邻近节点（O(k) vs O(N) 全扫描）
        candidates = network.nearby_node_ids(world_pos, self.threshold)
        for node_id in candidates:
            node = network.nodes[node_id]
            d = node.position.distance_to(world_pos)
            if d < self.threshold and d < best_dist:
                best_dist = d
                best_node_id = node_id

        if best_node_id is None:
            return None

        node = network.nodes[best_node_id]
        candidates = self._tangent_candidates_at_node(node, network)
        # 端点（仅 1 条候选）时 best 等于唯一项；道岔多候选时 best 留 None，
        # 由后续 commit 逻辑根据 M2 选择
        best = candidates[0] if len(candidates) == 1 else None

        return SnapResult(
            snapped=True,
            position=node.position,
            tangent=best,
            tangent_candidates=candidates,
            snapped_node_id=best_node_id,
        )

    def _tangent_candidates_at_node(self, node, network: RailNetwork) -> list[Vec3]:
        """节点处的所有候选切线方向。

        - 孤立节点（count==0）：返回 []，无切线约束
        - 端点（count==1）：返回 [t]，唯一方向
        - 道岔/交叉（count>=2）：返回 [t1, t2, ...]，每条相邻边一个外向切线
        """
        count = node.connection_count()
        if count == 0:
            return []

        candidates: list[Vec3] = []
        for edge_id in node.incident_edge_ids:
            edge = network.edges[edge_id]
            t = self._tangent_at_node_for_edge(node, edge, network)
            candidates.append(t)
        return candidates

    def _tangent_at_node_for_edge(self, node, edge, network: RailNetwork) -> Vec3:
        """计算节点处"沿这条边、远离对面 other 节点"的切线方向。

        语义：T1 表示"以 M1 为起点继续延伸的方向"。新建段在 M1 处的切线必须
        与既有边在 M1 处切向连续，且朝外延伸（不折返），所以方向是"远离 other"。
        """
        other_id = edge.node_b_id if edge.node_a_id == node.node_id else edge.node_a_id
        other = network.nodes[other_id]

        if not edge.is_arc:
            # 直线：M1 处沿边、远离 other 的方向 = (node - other).normalize()
            return (node.position - other.position).normalize()

        # 圆弧：M1 处的切线方向 = arc_normal × (node - center)
        # 调整符号使其远离 other（夹角与 (node - other) 同向）
        center = edge.arc_center
        to_node = node.position - center
        tangent = edge.arc_normal.cross(to_node)

        outward_dir = (node.position - other.position).normalize()
        if tangent.dot(outward_dir) < 0:
            tangent = tangent * -1.0

        return tangent.normalize()


class GridSnapProvider:
    """格点吸附提供器:吸附到网格交点,粒度随缩放档位变化。

    优先级:点 > 格点 > 路径(见 SnapSystem)。格点无切线约束。
    """

    def __init__(self, pixel_scale: float = 40.0) -> None:
        self.enabled = True
        self.pixel_scale = pixel_scale  # 由上层每帧同步

    def snap(self, world_pos: Vec3, network: RailNetwork) -> SnapResult | None:
        """尝试吸附到最近的网格交点。threshold 隐含在档位粒度中(无需显式阈值)。"""
        if not self.enabled or self.pixel_scale <= 0.0:
            return None

        # 当前网格档位(米) — 复用 view/grid.py 的档位算法
        from view.grid import _pick_minor_spacing
        spacing = _pick_minor_spacing(self.pixel_scale)

        # 四舍五入到最近的网格交点
        gx = round(world_pos.x / spacing) * spacing
        gy = round(world_pos.y / spacing) * spacing
        grid_point = Vec3(gx, gy, 0.0)

        # 屏幕像素距离判定:若光标距网格点 > SNAP_THRESHOLD_PX,不吸附
        # (避免远距离误吸,与点/路径吸附的像素阈值对齐)
        SNAP_THRESHOLD_PX = 12.0
        world_dist = world_pos.distance_to(grid_point)
        screen_dist_px = world_dist * self.pixel_scale
        if screen_dist_px > SNAP_THRESHOLD_PX:
            return None

        return SnapResult(
            snapped=True,
            position=grid_point,
            tangent=None,  # 格点无切线约束
            tangent_candidates=[],
        )


class ParallelSnapProvider:
    """平行吸附提供器:生成既有轨道的平行参考点(Simple/Complex Case)。

    Simple Case:每个节点沿切线垂线在固定间距 R 处生成参考点,携带父节点切线。
    Complex Case:在 (R-δ, R+δ) 区间投射线段,与其它边求交点作为参考点。
    """

    def __init__(self, spacing: float = 5.0) -> None:
        self.enabled = False  # 默认关闭,由上层键盘切换
        self.spacing = spacing  # 平行间距(米),固定 5m
        self.pixel_scale = 40.0  # 由上层每帧同步
        # 缓存的参考点列表:(position, tangent, parent_node_id)
        # 全部为 Simple Case,Complex 改为 lazy 按需计算
        self._reference_points: list[tuple[Vec3, Vec3, int]] = []

    def update_reference_points(self, network: RailNetwork) -> None:
        """重新生成所有平行参考点(Simple Case only)。

        所有节点统一生成固定间距参考点,Complex Case 改为吸附时 lazy 计算。
        """
        self._reference_points.clear()

        for node_id, node in network.nodes.items():
            if node.connection_count() == 0:
                continue  # 孤立节点无切线,跳过

            # 获取该节点的所有切线候选
            tangents = self._get_node_tangents(node, network)

            for tangent in tangents:
                # 两侧各生成固定间距参考点(Simple Case)
                perp = self._perp_xy(tangent)
                for side in (1.0, -1.0):
                    ref_pos = node.position + perp * (side * self.spacing)
                    self._reference_points.append((ref_pos, tangent, node_id))

    def snap(self, world_pos: Vec3, network: RailNetwork) -> SnapResult | None:
        """尝试吸附到最近的平行参考点,并执行 lazy Complex 后处理。

        1. 找到最近的 Simple 参考点(屏幕像素阈值)
        2. 后处理:检测参考点是否在既有边上(容差内) → Complex Case
        3. 返回修正后的结果(位置+切线)
        """
        if not self.enabled or not self._reference_points:
            return None

        SNAP_THRESHOLD_PX = 12.0
        best_dist = float('inf')
        best_ref: tuple[Vec3, Vec3, int] | None = None

        for ref_pos, ref_tangent, parent_id in self._reference_points:
            world_dist = world_pos.distance_to(ref_pos)
            screen_dist_px = world_dist * self.pixel_scale
            if screen_dist_px < SNAP_THRESHOLD_PX and world_dist < best_dist:
                best_dist = world_dist
                best_ref = (ref_pos, ref_tangent, parent_id)

        if best_ref is None:
            return None

        ref_pos, ref_tangent, parent_id = best_ref

        # Lazy Complex 后处理:检测参考点是否在既有边上
        complex_result = self._try_lazy_complex(ref_pos, parent_id, network)
        if complex_result is not None:
            # Complex Case 命中:用边上的点和边切线
            complex_pos, complex_tangent, edge_id, edge_t = complex_result
            return SnapResult(
                snapped=True,
                position=complex_pos,
                tangent=complex_tangent,
                tangent_candidates=[complex_tangent, complex_tangent * -1.0],
                snapped_node_id=None,  # 不是节点,是边上的点
                snapped_edge_id=edge_id,  # 吸附到边上
                snapped_edge_t=edge_t,  # 边上的参数位置
                edge_direction=complex_tangent,  # 边方向
            )

        # Simple Case:保持原参考点
        return SnapResult(
            snapped=True,
            position=ref_pos,
            tangent=ref_tangent,
            tangent_candidates=[ref_tangent, ref_tangent * -1.0],
        )

    def _try_lazy_complex(
        self, ref_pos: Vec3, parent_node_id: int, network: RailNetwork
    ) -> tuple[Vec3, Vec3, int, float] | None:
        """Lazy Complex Case:检测参考点是否在既有边上。

        返回 (边上最近点, 边切线, edge_id, t) 或 None(无匹配)。
        仅检测道岔(度数≥3)的参考点,且距离 < δ 容差。
        """
        parent_node = network.nodes.get(parent_node_id)
        if parent_node is None or parent_node.connection_count() < 3:
            return None  # 非道岔,跳过 Complex

        delta = self.spacing * 0.2  # δ = 1m 容差
        from model.geom_utils import closest_point_on_edge, edge_aabb

        parent_edges = parent_node.incident_edge_ids
        candidates: list[tuple[Vec3, Vec3, int, float, float]] = []  # (最近点, 边切线, edge_id, t, 距离)

        # 走空间索引：只扫描参考点邻近的边（O(k) vs O(N) 全扫描）
        # 查询半径 = spacing + delta + 余量，覆盖可能命中的边
        nearby_edge_ids = network.nearby_edge_ids(ref_pos, self.spacing + delta + 1.0)
        for edge_id in nearby_edge_ids:
            if edge_id in parent_edges:
                continue  # 排除父节点关联边

            edge = network.edges[edge_id]
            node_a = network.nodes[edge.node_a_id]
            node_b = network.nodes[edge.node_b_id]

            # 精确最近点（索引已做粗筛，不再需要手工 AABB 判定）
            closest_pt, dist = closest_point_on_edge(ref_pos, edge, node_a, node_b)

            if dist < delta:
                # 计算边切线和参数 t
                from model.geom_utils import tangent_along_edge, project_on_segment
                t_closest, _proj, _d = project_on_segment(closest_pt, node_a.position, node_b.position)
                edge_tangent = tangent_along_edge(edge, node_a, node_b, t=t_closest)
                if edge_tangent.length() > 1e-9:
                    candidates.append((closest_pt, edge_tangent.normalize(), edge_id, t_closest, dist))

        # 取最近的候选
        if candidates:
            candidates.sort(key=lambda x: x[4])
            closest_pt, edge_tangent, edge_id, t_closest, _dist = candidates[0]
            return closest_pt, edge_tangent, edge_id, t_closest

        return None

    def _get_node_tangents(self, node: Node, network: RailNetwork) -> list[Vec3]:
        """获取节点的所有出射切线(每条关联边一个)。"""
        from model.geom_utils import tangent_along_edge

        tangents = []
        for edge_id in node.incident_edge_ids:
            edge = network.edges.get(edge_id)
            if edge is None:
                continue

            node_a = network.nodes[edge.node_a_id]
            node_b = network.nodes[edge.node_b_id]

            # 判断当前节点是 A 还是 B
            if edge.node_a_id == node.node_id:
                # 节点是 A 端,切线从 A 指向 B
                tangent = tangent_along_edge(edge, node_a, node_b, t=0.0)
            else:
                # 节点是 B 端,切线从 B 指向 A(反向)
                tangent = tangent_along_edge(edge, node_a, node_b, t=1.0) * -1.0

            if tangent.length() > 1e-9:
                tangents.append(tangent.normalize())

        return tangents

    @staticmethod
    def _perp_xy(v: Vec3) -> Vec3:
        """XY 平面垂直向量(逆时针 90°)"""
        return Vec3(-v.y, v.x, 0.0)


class SnapSystem:
    """吸附系统：管理多个吸附提供器，按优先级返回结果"""

    def __init__(self) -> None:
        self.point_snap = PointSnapProvider(threshold=0.3)
        self.grid_snap = GridSnapProvider(pixel_scale=40.0)
        self.path_snap = PathSnapProvider(threshold=0.3)
        self.parallel_snap = ParallelSnapProvider(spacing=5.0)

    def any_enabled(self) -> bool:
        """是否有至少一个 Provider 启用"""
        return (
            self.point_snap.enabled
            or self.grid_snap.enabled
            or self.path_snap.enabled
            or self.parallel_snap.enabled
        )

    def snap(
        self,
        world_pos: Vec3,
        network: RailNetwork,
        reference_pos: Vec3 | None = None,
        world_threshold: float | None = None,
    ) -> SnapResult:
        """按优先级依次尝试吸附。

        reference_pos：用于路径吸附时选择切线正反方向（一般传 BUILD_ACTIVE 的 M1）。
        BUILD_IDLE 状态传 None，此时路径吸附不产生切线。

        world_threshold：本次吸附使用的世界阈值（米）。由上层按 camera.scale 从
        屏幕像素阈值换算而来，使吸附半径以"屏幕视觉距离"为基准、随缩放跟随。
        为 None 时沿用各 Provider 的默认阈值。
        """
        if world_threshold is not None:
            self.point_snap.threshold = world_threshold
            self.path_snap.threshold = world_threshold

        # 优先级 1: 点吸附
        result = self.point_snap.snap(world_pos, network)
        if result is not None:
            return result

        # 优先级 2: 格点吸附
        result = self.grid_snap.snap(world_pos, network)
        if result is not None:
            return result

        # 优先级 3: 平行吸附
        result = self.parallel_snap.snap(world_pos, network)
        if result is not None:
            return result

        # 优先级 4: 路径吸附
        result = self.path_snap.snap(world_pos, network, reference_pos)
        if result is not None:
            return result

        # 无吸附：返回原始位置
        return SnapResult(snapped=False, position=world_pos)


class PathSnapProvider:
    """路径吸附提供器：吸附到既有边的最近投影点"""

    def __init__(self, threshold: float = 0.3) -> None:
        self.threshold = threshold
        self.enabled = True

    def snap(
        self,
        world_pos: Vec3,
        network: RailNetwork,
        reference_pos: Vec3 | None,
    ) -> SnapResult | None:
        """尝试吸附到最近的边。reference_pos 不为 None 时计算切线方向。"""
        if not self.enabled:
            return None

        best_edge_id: int | None = None
        best_t = 0.0
        best_pos = Vec3()
        best_dist = float('inf')

        # 走空间索引：只扫描邻近边（O(k) vs O(N) 全扫描）
        candidates = network.nearby_edge_ids(world_pos, self.threshold)
        for edge_id in candidates:
            edge = network.edges[edge_id]
            node_a = network.nodes[edge.node_a_id]
            node_b = network.nodes[edge.node_b_id]
            t, proj_pos, dist = project_point_on_edge(world_pos, edge, node_a, node_b)
            if dist < self.threshold and dist < best_dist:
                best_dist = dist
                best_edge_id = edge_id
                best_t = t
                best_pos = proj_pos

        if best_edge_id is None:
            return None

        edge = network.edges[best_edge_id]
        node_a = network.nodes[edge.node_a_id]
        node_b = network.nodes[edge.node_b_id]

        # 路径上一点的两个候选切线（forward / reverse）
        forward = tangent_along_edge(edge, node_a, node_b, best_t)
        reverse = forward * -1.0
        candidates = [forward, reverse]

        # 最佳切线选择：仅在 BUILD_ACTIVE（reference_pos 非 None）时计算
        best: Vec3 | None = None
        if reference_pos is not None:
            cursor_dir = world_pos - reference_pos
            if cursor_dir.length() > 1e-9:
                cursor_dir = cursor_dir.normalize()
                best = forward if forward.dot(cursor_dir) >= reverse.dot(cursor_dir) else reverse
            else:
                best = forward  # cursor 与 reference 重合，任取

        # edge_direction：仅直边有意义（用于 Case 2T §10.5）
        edge_dir = None
        if not edge.is_arc:
            edge_dir = (node_b.position - node_a.position).normalize()

        return SnapResult(
            snapped=True,
            position=best_pos,
            tangent=best,
            tangent_candidates=candidates,
            snapped_edge_id=best_edge_id,
            snapped_edge_t=best_t,
            edge_direction=edge_dir,
        )
