from __future__ import annotations

from dataclasses import dataclass

from model.geom_utils import project_point_on_edge, tangent_along_edge
from model.rail_network import RailNetwork
from model.vec3 import Vec3


@dataclass
class SnapResult:
    """吸附系统的输出结果"""
    snapped: bool                        # 是否发生了吸附
    position: Vec3                       # 吸附后的世界坐标（无吸附时 = 原始光标位置）
    tangent: Vec3 | None = None          # 该位置的切线方向（有切线约束时非 None）
    snapped_node_id: int | None = None   # 吸附到的 Node ID（点吸附时）
    snapped_edge_id: int | None = None   # 吸附到的 Edge ID（路径吸附时）
    snapped_edge_t: float | None = None  # 路径吸附时沿边的参数 t ∈ [0,1]


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

        for node_id, node in network.nodes.items():
            d = node.position.distance_to(world_pos)
            if d < self.threshold and d < best_dist:
                best_dist = d
                best_node_id = node_id

        if best_node_id is None:
            return None

        node = network.nodes[best_node_id]
        tangent = self._tangent_at_node(node, network)

        return SnapResult(
            snapped=True,
            position=node.position,
            tangent=tangent,
            snapped_node_id=best_node_id,
        )

    def _tangent_at_node(self, node, network: RailNetwork) -> Vec3 | None:
        """计算节点处的切线方向（仅 connection_count == 1 时有效）"""
        count = node.connection_count()

        if count == 0:
            return None  # 孤立节点，无切线

        if count == 1:
            # 端点：取唯一连接边的外向切线
            edge_id = next(iter(node.incident_edge_ids))
            edge = network.edges[edge_id]
            return self._tangent_at_node_for_edge(node, edge, network)

        # count >= 2: 暂不处理（道岔/交叉），返回 None
        # TODO: Step 2/3 中处理多连接节点的切线选择
        return None

    def _tangent_at_node_for_edge(self, node, edge, network: RailNetwork) -> Vec3:
        """计算节点在特定边上的外向切线方向"""
        other_id = edge.node_b_id if edge.node_a_id == node.node_id else edge.node_a_id
        other = network.nodes[other_id]

        if not edge.is_arc:
            # 直线：方向指向远处
            return (other.position - node.position).normalize()

        # 圆弧：切线方向 = arc_normal × (node.position - arc_center)
        center = edge.arc_center
        to_node = node.position - center
        tangent = edge.arc_normal.cross(to_node)

        # 确保方向指向远处（与 other 方向夹角 < 90°）
        outward_dir = (other.position - node.position).normalize()
        if tangent.dot(outward_dir) < 0:
            tangent = tangent * -1.0

        return tangent.normalize()


class SnapSystem:
    """吸附系统：管理多个吸附提供器，按优先级返回结果"""

    def __init__(self) -> None:
        self.point_snap = PointSnapProvider(threshold=0.3)
        self.path_snap = PathSnapProvider(threshold=0.3)
        # TODO: 后续添加 ParallelPointProvider, ParallelPathProvider

    def any_enabled(self) -> bool:
        """是否有至少一个 Provider 启用"""
        return self.point_snap.enabled or self.path_snap.enabled

    def snap(
        self,
        world_pos: Vec3,
        network: RailNetwork,
        reference_pos: Vec3 | None = None,
    ) -> SnapResult:
        """按优先级依次尝试吸附。

        reference_pos：用于路径吸附时选择切线正反方向（一般传 BUILD_ACTIVE 的 M1）。
        BUILD_IDLE 状态传 None，此时路径吸附不产生切线。
        """
        # 优先级 1: 点吸附
        result = self.point_snap.snap(world_pos, network)
        if result is not None:
            return result

        # 优先级 2: 路径吸附
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

        for edge_id, edge in network.edges.items():
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

        # 切线方向选择：仅在 BUILD_ACTIVE（reference_pos 非 None）时计算
        tangent: Vec3 | None = None
        if reference_pos is not None:
            forward = tangent_along_edge(edge, node_a, node_b, best_t)
            reverse = forward * -1.0
            # 取与 (world_pos - reference_pos) 夹角较小的方向
            cursor_dir = world_pos - reference_pos
            if cursor_dir.length() > 1e-9:
                cursor_dir = cursor_dir.normalize()
                if forward.dot(cursor_dir) >= reverse.dot(cursor_dir):
                    tangent = forward
                else:
                    tangent = reverse
            else:
                tangent = forward  # cursor 与 reference 重合，任取

        return SnapResult(
            snapped=True,
            position=best_pos,
            tangent=tangent,
            snapped_edge_id=best_edge_id,
            snapped_edge_t=best_t,
        )
