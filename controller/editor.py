from __future__ import annotations

from enum import Enum, auto

from model.geom_utils import (
    can_merge_arcs,
    can_merge_straight,
    is_biarc_collinear_straight,
    is_t1_consistent_with_target,
    merged_arc_b_point,
    project_along_direction,
    solve_biarc,
    solve_case2_arc,
)
from model.rail_network import RailNetwork
from model.vec3 import Vec3
from controller.snap import SnapSystem, SnapResult
from controller.build_plan import ConstructionPlan, PreviewGeometry

SNAP_THRESHOLD = 0.3
WARNING_THRESHOLD = 0.5
MAX_ARC_RADIUS = 500.0  # 弧半径超过此值时退化为沿 T1 的直线（Q3）


class EditMode(Enum):
    """EditMode 顶层模式 = IDLE / BUILD / DELETE.

    DELETE 操作单位是 Edge，自动清理孤立 Node。
    """
    IDLE = auto()     # 空闲：仅视图操作
    BUILD = auto()    # 建造轨道
    DELETE = auto()   # 删除轨道（边为单位）


class BuildState(Enum):
    """BUILD 模式的子状态"""
    IDLE = auto()     # 等待第一次点击
    ACTIVE = auto()   # 已选定 M1，等待 M2 确认


class Editor:
    def __init__(self, network: RailNetwork) -> None:
        self.network = network
        self.mode: EditMode = EditMode.IDLE
        self.snap_system = SnapSystem()

        # BUILD 模式状态
        self.build_state: BuildState = BuildState.IDLE
        self.build_m1: Vec3 | None = None              # M1 世界坐标
        self.build_m1_node_id: int | None = None       # M1 吸附的节点 ID（None 表示空白）
        self.build_t1_candidates: list[Vec3] = []      # M1 处的所有候选切线（空 = 无切线约束 / Case 1）

        # 悬停状态（用于 DELETE 和视觉反馈）
        self.hovered_node_id: int | None = None
        self.hovered_edge_id: int | None = None

        # 预览（BUILD_ACTIVE 时有效）
        self.preview: PreviewGeometry | None = None

        # 警告状态（BUILD_IDLE 吸附关闭时邻近元素警告）
        self.show_warning: bool = False

        # 修饰键标志：由 GameLoop 每帧同步
        # force_straight=True 时强制走 Case 1 直线（即便 T1 有候选）。临时调试用，参见 docs §10.2
        self.force_straight: bool = False

    def update_hover(self, world_pos: Vec3) -> None:
        """每帧更新：处理鼠标悬停 + 预览 + 警告"""
        # 吸附检测
        snap = self._snap(world_pos)

        # 更新悬停状态（用于 DELETE 模式和高亮）
        self.hovered_node_id = snap.snapped_node_id
        if self.mode == EditMode.DELETE:
            # DELETE 模式：操作单位是边。点吸附到节点时不算 hover edge，
            # 否则用路径吸附结果作为 hover edge。
            if snap.snapped_node_id is not None:
                self.hovered_edge_id = None
            elif snap.snapped_edge_id is not None:
                self.hovered_edge_id = snap.snapped_edge_id
            else:
                self.hovered_edge_id = None
        else:
            self.hovered_edge_id = None

        # BUILD_ACTIVE: 更新预览
        if self.mode == EditMode.BUILD and self.build_state == BuildState.ACTIVE:
            self._update_preview(snap)

        # BUILD_IDLE: 检查警告（所有吸附 Provider 关闭 + 邻近元素）
        if self.mode == EditMode.BUILD and self.build_state == BuildState.IDLE:
            if not self.snap_system.any_enabled() and self._has_nearby_element(world_pos):
                self.show_warning = True
            else:
                self.show_warning = False
        else:
            self.show_warning = False

    def handle_click(self, world_pos: Vec3) -> None:
        """处理左键点击"""
        if self.mode == EditMode.IDLE:
            return  # IDLE 模式下左键无效

        snap = self._snap(world_pos)

        if self.mode == EditMode.BUILD:
            if self.build_state == BuildState.IDLE:
                self._start_build(world_pos, snap)
            else:  # BuildState.ACTIVE
                self._commit_build(world_pos, snap)

        elif self.mode == EditMode.DELETE:
            self._delete()

    def _snap(self, world_pos: Vec3) -> SnapResult:
        """统一的吸附入口：BUILD_ACTIVE 时传入 M1 作为切线方向参考。"""
        reference = (
            self.build_m1
            if (self.mode == EditMode.BUILD and self.build_state == BuildState.ACTIVE)
            else None
        )
        return self.snap_system.snap(world_pos, self.network, reference)

    def handle_cancel(self) -> None:
        """处理 Esc 键"""
        if self.mode == EditMode.BUILD and self.build_state == BuildState.ACTIVE:
            # 取消当前建造
            self._reset_build_active()
        else:
            # 回到 IDLE 模式
            self.set_mode(EditMode.IDLE)

    def set_mode(self, mode: EditMode) -> None:
        """切换顶层模式"""
        self.mode = mode
        # 重置 BUILD 状态
        self._reset_build_active()
        self.show_warning = False

    def _reset_build_active(self) -> None:
        """清空 BUILD_ACTIVE 状态，回到 BUILD_IDLE。"""
        self.build_state = BuildState.IDLE
        self.build_m1 = None
        self.build_m1_node_id = None
        self.build_t1_candidates = []
        self.preview = None

    # ===== BUILD 逻辑 =====

    def _start_build(self, world_pos: Vec3, snap: SnapResult) -> None:
        """开始建造：记录 M1，进入 BUILD_ACTIVE。

        拒绝条件：所有吸附 Provider 关闭 且 附近有既有元素。
        """
        # 1. 建造前检查
        if not self.snap_system.any_enabled() and self._has_nearby_element(world_pos):
            self.show_warning = True
            return  # 拒绝；保持 BUILD_IDLE

        # 2. 确定 M1
        if snap.snapped:
            self.build_m1 = snap.position
            self.build_m1_node_id = snap.snapped_node_id
            self.build_t1_candidates = list(snap.tangent_candidates)
        else:
            self.build_m1 = world_pos
            self.build_m1_node_id = None
            self.build_t1_candidates = []

        # 3. 进入 BUILD_ACTIVE
        self.build_state = BuildState.ACTIVE
        self.preview = None  # 预览在 update_hover 中每帧更新

    def _commit_build(self, world_pos: Vec3, snap: SnapResult) -> None:
        """提交建造：从 M1 到 M2 建造轨道。

        所有拒绝一律保持 BUILD_ACTIVE 状态，由用户重新选 M2 或 Esc 取消。
        """
        # 1. 确定 M2
        if snap.snapped:
            m2 = snap.position
            m2_node_id = snap.snapped_node_id
            t2_candidates = list(snap.tangent_candidates)
        else:
            m2 = world_pos
            m2_node_id = None
            t2_candidates = []

        # 2. 计算建造计划
        plan = self._compute_plan(
            m1=self.build_m1,
            m2=m2,
            t1_candidates=self.build_t1_candidates,
            t2_candidates=t2_candidates,
            m1_node_id=self.build_m1_node_id,
            m2_node_id=m2_node_id,
        )

        if not plan.valid:
            return  # 拒绝；保持 BUILD_ACTIVE

        # 3. TODO: 截断（M2 在既有边中间）
        # 4. 应用计划到网络
        self._apply_plan(plan)

        # 5. 回到 BUILD_IDLE
        self._reset_build_active()

    def _compute_plan(
        self,
        m1: Vec3,
        m2: Vec3,
        t1_candidates: list[Vec3],
        t2_candidates: list[Vec3],
        m1_node_id: int | None,
        m2_node_id: int | None,
    ) -> ConstructionPlan:
        """计算建造计划。

        分支：
        - 无 T1 候选 → Case 1（直接连 M1→M2）
        - 有 T1 候选无 T2 候选 → 选最佳 T1，求 Case 2 单弧（半径过大或 force_straight 退化为沿 T1 直线）
        - T1 + T2 都有候选 → 选各自最佳（按 (M2-M1) / (M1-M2) 内积），尝试 Case 3 Biarc
            - 共线退化 fast-path → Case 1
            - force_straight=True → 仍走 T1 退化直线（Case 3 是后续约束，强制直线优先级更高）
            - 其他失败 → 直接拒绝（按用户决议；不向 Case 2 降级）
        - Q2/Q5 拒绝条件适用于所有有 T1 的分支（含强制直线），保证既有轨道方向连续
        """
        # M1 == M2 → 拒绝
        if m1.distance_to(m2) < 1e-6:
            return ConstructionPlan(case=1, m1=m1, m2=m2, valid=False)

        # Case 1：无切线约束（自由直线）
        if not t1_candidates:
            return ConstructionPlan(
                case=1,
                m1=m1,
                m2=m2,
                node_a_id=m1_node_id,
                node_b_id=m2_node_id,
                edge_geometry=[],
                valid=True,
            )

        # 有 T1 候选：先选最佳
        d = m2 - m1
        d_len = d.length()
        if d_len < 1e-9:
            return ConstructionPlan(case=2, m1=m1, m2=m2, valid=False)
        d_hat = d * (1.0 / d_len)

        best_t1: Vec3 | None = None
        best_dot = -float("inf")
        for cand in t1_candidates:
            cn = cand.normalize()
            score = cn.dot(d_hat)
            if score > best_dot:
                best_dot = score
                best_t1 = cn

        if best_t1 is None or best_dot < 1e-6:
            # Q5: 所有候选与 d 钝角（或垂直）→ 拒绝
            return ConstructionPlan(case=2, m1=m1, m2=m2, valid=False)

        # Q2: T1 反向（M2 在 T1 背后）→ 拒绝
        if not is_t1_consistent_with_target(best_t1, m1, m2):
            return ConstructionPlan(case=2, m1=m1, m2=m2, valid=False)

        # 强制直线：跳过弧 / Biarc，直接沿 T1 退化（优先级高于 Case 3）
        if self.force_straight:
            return self._degenerate_to_straight(m1, best_t1, m2, m1_node_id)

        if t2_candidates:
            # Case 3 候选路径：枚举所有 T2 候选，挑能解出且 R 最大者
            plan_3 = self._try_case3_biarc(
                m1, best_t1, m2, t2_candidates, m1_node_id, m2_node_id
            )
            if plan_3 is not None:
                return plan_3
            # 失败 → 直接拒绝（按用户决议：不向 Case 2 降级）
            return ConstructionPlan(case=3, m1=m1, m2=m2, valid=False)

        # 无 T2 → 走 Case 2 单弧
        result = solve_case2_arc(m1, best_t1, m2)
        if result is None:
            # 弧不可解（M2 几乎在 T1 延长线上）→ 退化为沿 T1 的直线
            return self._degenerate_to_straight(m1, best_t1, m2, m1_node_id)

        center, b_point, arc_normal, radius = result

        # Q3: 半径过大 → 退化为沿 T1 的直线
        if radius > MAX_ARC_RADIUS:
            return self._degenerate_to_straight(m1, best_t1, m2, m1_node_id)

        return ConstructionPlan(
            case=2,
            m1=m1,
            m2=m2,
            node_a_id=m1_node_id,
            node_b_id=m2_node_id,
            edge_geometry=[b_point],  # 弧用 B 点表示
            valid=True,
        )

    def _try_case3_biarc(
        self,
        m1: Vec3,
        t1: Vec3,
        m2: Vec3,
        t2_candidates: list[Vec3],
        m1_node_id: int | None,
        m2_node_id: int | None,
    ) -> ConstructionPlan | None:
        """尝试 Case 3 Biarc。枚举所有 T2 候选，每个候选求 biarc 解，
        返回 R 最大（曲率最小）的合法 Plan，或 None（全部不可解 / 半径超限）。

        T1 和 T2 候选的语义都是"远离自身节点的另一端，朝外延伸"。
        biarc 数学约定 T2 是"沿 M_mid → M2 进入 M2 的方向"，所以传入
        solve_biarc 时取 -t2。共线退化 fast-path 同样用取负后的 t2 判定。
        """
        best_plan: ConstructionPlan | None = None
        best_r = -1.0

        for t2_raw in t2_candidates:
            t2_for_biarc = t2_raw.normalize() * -1.0

            # Fast-path: T1 与 -T2 同向 且 d∥T1 → Case 1 直线（最高优先级）
            if is_biarc_collinear_straight(t1, t2_for_biarc, m1, m2):
                return ConstructionPlan(
                    case=1,
                    m1=m1,
                    m2=m2,
                    node_a_id=m1_node_id,
                    node_b_id=m2_node_id,
                    edge_geometry=[],
                    valid=True,
                )

            result = solve_biarc(m1, t1, m2, t2_for_biarc)
            if result is None:
                continue
            m_mid, b1, b2, an1, an2, radius = result

            if radius > MAX_ARC_RADIUS:
                continue

            if radius > best_r:
                best_r = radius
                best_plan = ConstructionPlan(
                    case=3,
                    m1=m1,
                    m2=m2,
                    node_a_id=m1_node_id,
                    node_b_id=m2_node_id,
                    edge_geometry=[],
                    biarc_mid=m_mid,
                    biarc_geom_1=[b1],
                    biarc_geom_2=[b2],
                    valid=True,
                )

        return best_plan

    def _degenerate_to_straight(
        self,
        m1: Vec3,
        t1: Vec3,
        m2: Vec3,
        m1_node_id: int | None,
    ) -> ConstructionPlan:
        """Case 2 退化为沿 T1 的直线：终点是 M2 在 (M1, T1) 上的投影。

        如此保证起点切线连续，避免 M1 处出现折角。
        end_node 一律新建（不复用 m2 处的吸附节点，因为目标点已偏移）。
        """
        m2_eff = project_along_direction(m1, t1, m2)
        if m1.distance_to(m2_eff) < 1e-6:
            return ConstructionPlan(case=1, m1=m1, m2=m2_eff, valid=False)
        return ConstructionPlan(
            case=1,
            m1=m1,
            m2=m2_eff,
            node_a_id=m1_node_id,
            node_b_id=None,  # 退化情形：终点是新位置，不复用 M2 的节点
            edge_geometry=[],
            valid=True,
        )

    def _apply_plan(self, plan: ConstructionPlan) -> None:
        """将建造计划应用到网络"""
        # 获取或创建端点节点
        if plan.node_a_id is not None:
            node_a = self.network.nodes[plan.node_a_id]
        else:
            node_a = self.network.add_node(plan.m1)

        if plan.node_b_id is not None:
            node_b = self.network.nodes[plan.node_b_id]
        else:
            node_b = self.network.add_node(plan.m2)

        # 避免自环
        if node_a.node_id == node_b.node_id:
            return

        if plan.case == 3:
            # Biarc 方案 A：两条 Edge + 中间 Node
            if plan.biarc_mid is None or plan.biarc_geom_1 is None or plan.biarc_geom_2 is None:
                return
            mid_node = self.network.add_node(plan.biarc_mid)
            try:
                self.network.add_edge(node_a, mid_node, plan.biarc_geom_1)
                self.network.add_edge(mid_node, node_b, plan.biarc_geom_2)
            except ValueError:
                # 数学解算失败兜底：清理已添加的资源（保守处理）
                self.network.remove_node(mid_node.node_id)
            return

        # Case 1 / Case 2：单条 Edge
        self.network.add_edge(node_a, node_b, plan.edge_geometry)

    def _update_preview(self, snap: SnapResult) -> None:
        """更新 BUILD_ACTIVE 的预览几何"""
        if self.build_m1 is None:
            self.preview = None
            return

        m2 = snap.position
        t2_candidates = list(snap.tangent_candidates) if snap.snapped else []

        # 计算预览计划（和提交时相同逻辑）
        plan = self._compute_plan(
            m1=self.build_m1,
            m2=m2,
            t1_candidates=self.build_t1_candidates,
            t2_candidates=t2_candidates,
            m1_node_id=self.build_m1_node_id,
            m2_node_id=snap.snapped_node_id,
        )

        self.preview = PreviewGeometry(
            m1=plan.m1,
            m2=plan.m2,
            case=plan.case,
            edge_geometry=plan.edge_geometry,
            valid=plan.valid,
            biarc_mid=plan.biarc_mid,
            biarc_geom_1=plan.biarc_geom_1,
            biarc_geom_2=plan.biarc_geom_2,
        )

    # ===== DELETE 逻辑 =====

    def _delete(self) -> None:
        """DELETE 模式点击：

        - 命中节点（仅 connection_count == 2）→ 尝试合并两侧边为一条新边；不可合并则静默忽略
        - 命中边 → 删除该边，并连带清理变成孤立的 Node
        - 否则 → 无操作

        悬停优先级（在 update_hover 中已固定）：节点 > 边。
        底线：操作完成后没有孤立节点、没有无头边。
        """
        # 命中节点（仅尝试合并 connection==2 的中间节点）
        if self.hovered_node_id is not None:
            self._try_merge_at_node(self.hovered_node_id)
            return

        # 命中边
        if self.hovered_edge_id is None:
            return

        edge = self.network.edges.get(self.hovered_edge_id)
        if edge is None:
            self.hovered_edge_id = None
            return

        node_a_id = edge.node_a_id
        node_b_id = edge.node_b_id

        self.network.remove_edge(self.hovered_edge_id)
        self.hovered_edge_id = None

        for nid in (node_a_id, node_b_id):
            node = self.network.nodes.get(nid)
            if node is not None and node.connection_count() == 0:
                self.network.remove_node(nid)

    def _try_merge_at_node(self, node_id: int) -> None:
        """尝试合并 connection_count == 2 节点两侧的边。

        失败条件（任一满足都静默忽略，不修改网络）：
        - 节点不存在 / 连接数不为 2
        - 两条边一直一弧（混合）
        - 两条直线不共线（夹角 > 阈值）
        - 两条弧不同心 / 不同半径 / 不同向（normal 反向）/ 切线不连续
        - 合并后的弧 B 点求解失败
        """
        node = self.network.nodes.get(node_id)
        if node is None or node.connection_count() != 2:
            return

        edge_ids = list(node.incident_edge_ids)
        e1 = self.network.edges[edge_ids[0]]
        e2 = self.network.edges[edge_ids[1]]

        # 找出两侧的"远端"节点（不是当前 node 的那一端）
        other_a_id = e1.node_a_id if e1.node_a_id != node_id else e1.node_b_id
        other_b_id = e2.node_a_id if e2.node_a_id != node_id else e2.node_b_id

        if other_a_id == other_b_id:
            # 两条边连接相同的远端 → 这是个"双重边/环"，合并后会自环，拒绝
            return

        a_pos = self.network.nodes[other_a_id].position
        b_pos = self.network.nodes[other_b_id].position
        mid_pos = node.position

        # 类型一致性检查
        if e1.is_arc != e2.is_arc:
            return  # 一直一弧不可合并

        new_geometry: list[Vec3]
        if not e1.is_arc:
            # 两条直线
            if not can_merge_straight(a_pos, mid_pos, b_pos):
                return
            new_geometry = []
        else:
            # 两条弧
            if not can_merge_arcs(e1, e2, mid_pos, a_pos, b_pos):
                return
            new_b = merged_arc_b_point(e1, e2, a_pos, b_pos)
            if new_b is None:
                return
            new_geometry = [new_b]

        # 删除中间节点 + 两条原边，再以原远端节点和新几何建一条边
        # 顺序：先删边、再删节点（避免连带删除把远端也清理掉），再 add_edge
        self.network.remove_edge(e1.edge_id)
        self.network.remove_edge(e2.edge_id)
        self.network.remove_node(node_id)
        self.hovered_node_id = None
        self.hovered_edge_id = None

        try:
            self.network.add_edge(
                self.network.nodes[other_a_id],
                self.network.nodes[other_b_id],
                new_geometry,
            )
        except ValueError:
            # add_edge 在弧几何无效时抛出（理论上前面检查后不应发生）。静默吞掉。
            return

    # ===== 辅助函数 =====

    def _has_nearby_element(self, pos: Vec3) -> bool:
        """检查位置附近是否有节点或边（用于警告）"""
        if self.network.node_id_at(pos, WARNING_THRESHOLD) is not None:
            return True
        if self.network.edge_id_at(pos, WARNING_THRESHOLD) is not None:
            return True
        return False
