from __future__ import annotations

from dataclasses import dataclass

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
        # TODO: 后续添加 PathSnapProvider, ParallelPointProvider, ParallelPathProvider

    def snap(self, world_pos: Vec3, network: RailNetwork) -> SnapResult:
        """按优先级依次尝试吸附，返回最终结果（总是返回，未吸附时 snapped=False）"""
        # 优先级 1: 点吸附
        result = self.point_snap.snap(world_pos, network)
        if result is not None:
            return result

        # TODO: 优先级 2: 路径吸附

        # 无吸附：返回原始位置
        return SnapResult(snapped=False, position=world_pos)
