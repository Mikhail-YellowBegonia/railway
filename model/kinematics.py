from __future__ import annotations

import math
from dataclasses import dataclass

from model.rail_network import RailNetwork
from model.pathfinding import Path, DirectedEdge, head_node, tail_node
from model.vec3 import Vec3


@dataclass
class Pose:
    """位姿：位置 + 朝向（运动学层输出）。"""
    position: Vec3
    heading: Vec3  # 单位化，指向前进方向


class PathKinematics:
    """Path 的弧长参数化 forward kinematics（质点模型）。

    给定 Path（有向边序列）和标量弧长 s（米），输出位姿 (position, heading)。
    - 弧长 s ∈ [0, total_length]，越界自动 clamp
    - heading 单位化，沿有向边的前进方向
    - 跨 Edge 边界连续（相切设计保证）
    """

    def __init__(self, network: RailNetwork, path: Path) -> None:
        self.network = network
        self.path = path
        # 累积弧长表：segments[i] = (有向边, 该段起点累积弧长, 该段长度)
        self._segments: list[tuple[DirectedEdge, float, float]] = []
        self._total_length = 0.0
        self._build_segments()

    def _build_segments(self) -> None:
        """构建弧长累积表：逐段累加每条有向边的长度。"""
        s_accum = 0.0
        for directed in self.path.edges:
            edge_id, direction = directed
            edge = self.network.edges[edge_id]
            length = edge.length
            self._segments.append((directed, s_accum, length))
            s_accum += length
        self._total_length = s_accum

    @property
    def total_length(self) -> float:
        """路径总弧长（米）。"""
        return self._total_length

    def pose_at(self, s: float) -> Pose:
        """查询弧长 s 处的位姿。s 越界自动 clamp 到 [0, total_length]。"""
        s = max(0.0, min(s, self._total_length))

        if self._total_length == 0.0:
            # 空路径或零长路径：返回起点位置 + 默认朝向
            if self.path.edges:
                edge_id, direction = self.path.edges[0]
                edge = self.network.edges[edge_id]
                node_id = tail_node(self.network, (edge_id, direction))
                pos = self.network.nodes[node_id].position
                return Pose(position=pos, heading=Vec3(1.0, 0.0, 0.0))
            return Pose(position=Vec3(0, 0, 0), heading=Vec3(1.0, 0.0, 0.0))

        # 二分查找定位 s 落在哪个 segment
        idx = self._locate_segment(s)
        directed, s_start, seg_length = self._segments[idx]
        s_local = s - s_start  # 该段内的局部弧长

        # 边内插值
        edge_id, direction = directed
        edge = self.network.edges[edge_id]
        node_a = self.network.nodes[edge.node_a_id]
        node_b = self.network.nodes[edge.node_b_id]

        # t ∈ [0,1] 是 Edge 自身的参数（node_a → node_b）
        # direction 决定正/反向遍历
        if direction > 0:
            # 沿 node_a → node_b
            t = s_local / seg_length if seg_length > 0 else 0.0
            t = max(0.0, min(t, 1.0))
        else:
            # 反向 node_b → node_a
            t = 1.0 - (s_local / seg_length if seg_length > 0 else 0.0)
            t = max(0.0, min(t, 1.0))

        # 位置与切线插值
        pos = self._interpolate_position(edge, node_a, node_b, t)
        tangent = self._interpolate_tangent(edge, node_a, node_b, t)

        # heading 按 direction 决定正/反向
        heading = tangent if direction > 0 else tangent * -1.0
        if heading.length() < 1e-9:
            heading = Vec3(1.0, 0.0, 0.0)  # fallback
        else:
            heading = heading.normalize()

        return Pose(position=pos, heading=heading)

    def _locate_segment(self, s: float) -> int:
        """二分查找：定位弧长 s 落在哪个 segment（返回索引）。"""
        if not self._segments:
            return 0
        # 线性扫描（段数不多，二分意义不大；后续可优化）
        for i, (directed, s_start, seg_length) in enumerate(self._segments):
            if s < s_start + seg_length:
                return i
        return len(self._segments) - 1  # s 在最后一段或越界

    def edge_at(self, s: float) -> tuple[int, float]:
        """返回弧长 s 处所在的 Edge ID 和边内参数 t ∈ [0, 1]。

        t 的方向与有向边一致（dir=+1 时 t=0 为 node_a，dir=-1 时 t=0 为 node_b）。
        用于从当前位置发起新一次寻路。
        """
        s = max(0.0, min(s, self._total_length))
        idx = self._locate_segment(s)
        directed, s_start, seg_length = self._segments[idx]
        edge_id, direction = directed
        t_local = (s - s_start) / seg_length if seg_length > 0 else 0.0
        t_local = max(0.0, min(t_local, 1.0))
        # 转换为 Edge 自身的 t（node_a → node_b 方向）
        t_edge = t_local if direction > 0 else (1.0 - t_local)
        return edge_id, t_edge

    def directed_edge_at(self, s: float) -> tuple[int, float, int]:
        """原子返回弧长处的 ``(edge_id, t, direction)``。"""
        s = max(0.0, min(s, self._total_length))
        idx = self._locate_segment(s)
        directed, s_start, seg_length = self._segments[idx]
        edge_id, direction = directed
        t_local = (s - s_start) / seg_length if seg_length > 0 else 0.0
        t_local = max(0.0, min(t_local, 1.0))
        t_edge = t_local if direction > 0 else (1.0 - t_local)
        return edge_id, t_edge, direction

    def sub_path(self, s_tail: float, s_head: float) -> tuple[Path, float]:
        """提取 [s_tail, s_head] 范围的子路径。

        用于列车停放时截取覆盖车身的最小 Path。

        返回:
            (sub_path, initial_offset)
            - sub_path: 覆盖 [s_tail, s_head] 的有向边子序列
            - initial_offset: s_tail 在子路径第一段内的局部弧长偏移（米）
              调用方用 RigidWagonKinematics(initial_offset=initial_offset) 保持车头位置
        """
        if not self._segments:
            return Path(edges=[], total_cost=0.0), 0.0

        s_tail = max(0.0, s_tail)
        s_head = min(s_head, self._total_length)

        # 找到 s_tail 所在段的索引
        tail_idx = self._locate_segment(s_tail)
        head_idx = self._locate_segment(s_head)

        sub_edges = [seg[0] for seg in self._segments[tail_idx: head_idx + 1]]
        sub_cost = sum(seg[2] for seg in self._segments[tail_idx: head_idx + 1])

        # initial_offset = s_tail 在首段内的局部偏移
        _, seg_start, _ = self._segments[tail_idx]
        initial_offset = s_tail - seg_start

        return Path(edges=sub_edges, total_cost=sub_cost), initial_offset

    def _interpolate_position(self, edge, node_a, node_b, t: float) -> Vec3:
        """边上参数 t ∈ [0,1] 处的位置插值（直线线性 / 圆弧反解角度）。"""
        if not edge.is_arc:
            # 直线：线性插值
            return node_a.position + (node_b.position - node_a.position) * t

        # 圆弧：按弧长比例反解角度
        angle_at_t = edge.arc_angle_rad * t
        from model.geom_utils import rotate_around_axis
        rotated_dir = rotate_around_axis(edge.arc_start_dir, edge.arc_normal, angle_at_t)
        return edge.arc_center + rotated_dir * edge.arc_radius

    def _interpolate_tangent(self, edge, node_a, node_b, t: float) -> Vec3:
        """边上参数 t ∈ [0,1] 处的切线方向（forward，node_a → node_b）。"""
        from model.geom_utils import tangent_along_edge
        return tangent_along_edge(edge, node_a, node_b, t)
