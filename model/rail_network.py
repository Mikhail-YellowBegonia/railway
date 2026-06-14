from __future__ import annotations

import math
from dataclasses import dataclass, field

from model.vec3 import Vec3


@dataclass
class Node:
    node_id: int
    position: Vec3
    incident_edge_ids: set[int] = field(default_factory=set)

    def connection_count(self) -> int:
        return len(self.incident_edge_ids)


@dataclass
class Edge:
    edge_id: int
    node_a_id: int
    node_b_id: int
    geometry: list[Vec3] = field(default_factory=list)
    length: float = 0.0
    is_arc: bool = False
    arc_center: Vec3 | None = None
    arc_radius: float = 0.0
    arc_angle_rad: float = 0.0
    arc_start_dir: Vec3 | None = None
    arc_normal: Vec3 | None = None

    def sample_arc_points(self, segments: int = 20) -> list[Vec3]:
        if not self.is_arc or self.arc_center is None or self.arc_start_dir is None or self.arc_normal is None:
            return []

        center = self.arc_center
        radius = self.arc_radius
        angle = self.arc_angle_rad
        start_dir = self.arc_start_dir
        normal = self.arc_normal

        points: list[Vec3] = []
        for i in range(segments + 1):
            t = i / segments
            a = angle * t
            p = self._rotate_around_axis(start_dir, normal, a)
            points.append(center + p * radius)
        return points

    @staticmethod
    def _rotate_around_axis(v: Vec3, axis: Vec3, angle: float) -> Vec3:
        if axis.length_squared() == 0.0:
            return v
        k = axis.normalize()
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        return v * cos_a + k.cross(v) * sin_a + k * (k.dot(v)) * (1.0 - cos_a)


class RailNetwork:
    def __init__(self) -> None:
        self.nodes: dict[int, Node] = {}
        self.edges: dict[int, Edge] = {}
        self._next_node_id: int = 0
        self._next_edge_id: int = 0
        self._connectivity: dict[int, list[set[int]]] = {}

    def add_node(self, position: Vec3) -> Node:
        nid = self._next_node_id
        self._next_node_id += 1
        node = Node(node_id=nid, position=position)
        self.nodes[nid] = node
        return node

    def add_edge(self, node_a: Node, node_b: Node, geometry: list[Vec3] | None = None) -> Edge:
        eid = self._next_edge_id
        self._next_edge_id += 1

        if geometry is None:
            geometry = []

        edge = Edge(
            edge_id=eid,
            node_a_id=node_a.node_id,
            node_b_id=node_b.node_id,
            geometry=geometry,
        )

        if len(geometry) == 1:
            edge.is_arc = True
            arc_result = _compute_arc(node_a.position, geometry[0], node_b.position)
            if arc_result is None:
                raise ValueError("invalid arc definition")
            center, radius, angle, normal = arc_result
            edge.arc_center = center
            edge.arc_radius = radius
            edge.arc_angle_rad = angle
            edge.arc_start_dir = (node_a.position - center).normalize()
            edge.arc_normal = normal
            edge.length = radius * abs(angle)
        else:
            all_points = [node_a.position, *geometry, node_b.position]
            length = 0.0
            for i in range(len(all_points) - 1):
                length += all_points[i].distance_to(all_points[i + 1])
            edge.length = length

        self.edges[eid] = edge

        node_a.incident_edge_ids.add(eid)
        node_b.incident_edge_ids.add(eid)

        self._rebuild_connectivity_for(node_a.node_id)
        self._rebuild_connectivity_for(node_b.node_id)

        return edge

    def adjacent_edges_at(self, node_id: int, from_edge_id: int) -> set[int]:
        groups = self._connectivity.get(node_id, [])
        for group in groups:
            if from_edge_id in group:
                return group - {from_edge_id}
        target_node = self.nodes[node_id]
        return target_node.incident_edge_ids - {from_edge_id}

    def _rebuild_connectivity_for(self, node_id: int) -> None:
        node = self.nodes[node_id]
        count = node.connection_count()
        edge_ids = list(node.incident_edge_ids)

        if count <= 1:
            self._connectivity[node_id] = []
        elif count == 2:
            if len(edge_ids) == 2:
                self._connectivity[node_id] = [{edge_ids[0], edge_ids[1]}]
            else:
                self._connectivity[node_id] = []
        else:
            self._connectivity[node_id] = [set(edge_ids)]

    def connectivity_for(self, node_id: int) -> list[set[int]]:
        return self._connectivity.get(node_id, [])

    def node_id_at(self, position: Vec3, epsilon: float = 0.01) -> int | None:
        for node in self.nodes.values():
            if node.position.distance_to(position) < epsilon:
                return node.node_id
        return None

    def remove_edge(self, edge_id: int) -> None:
        edge = self.edges.pop(edge_id, None)
        if edge is None:
            return

        node_a = self.nodes.get(edge.node_a_id)
        node_b = self.nodes.get(edge.node_b_id)
        if node_a:
            node_a.incident_edge_ids.discard(edge_id)
            self._rebuild_connectivity_for(node_a.node_id)
        if node_b:
            node_b.incident_edge_ids.discard(edge_id)
            self._rebuild_connectivity_for(node_b.node_id)

    def remove_node(self, node_id: int) -> None:
        node = self.nodes.get(node_id)
        if node is None:
            return
        to_remove = list(node.incident_edge_ids)
        for eid in to_remove:
            self.remove_edge(eid)
        self.nodes.pop(node_id, None)
        self._connectivity.pop(node_id, None)

    def split_edge_at(
        self, edge_id: int, t: float
    ) -> int | None:
        """在参数 t 处把一条 Edge 拆成两段，返回新插入的中间节点 ID。

        - 直线：均匀线性分割，新两段 geometry=[]
        - 圆弧：保持原圆心/半径/方向，按切线交点反算每段新 B 点
        - 无效输入返回 None（边不存在 / t 越界 / 几何不可解）
        """
        if t <= 1e-6 or t >= 1.0 - 1e-6:
            return None
        edge = self.edges.get(edge_id)
        if edge is None:
            return None

        node_a = self.nodes[edge.node_a_id]
        node_b = self.nodes[edge.node_b_id]

        from model.geom_utils import (
            project_on_arc,
            project_on_segment,
            split_arc_b_points,
            rotate_around_axis,
        )

        # 求截断点位置
        if edge.is_arc:
            angle_at_t = edge.arc_angle_rad * t
            rotated_dir = rotate_around_axis(
                edge.arc_start_dir, edge.arc_normal, angle_at_t
            )
            split_pos = edge.arc_center + rotated_dir * edge.arc_radius
            b_points = split_arc_b_points(edge, node_a, node_b, split_pos)
            if b_points is None:
                return None
            geom_1 = [b_points[0]]
            geom_2 = [b_points[1]]
        else:
            split_pos = node_a.position + (node_b.position - node_a.position) * t
            geom_1 = []
            geom_2 = []

        # 创建新中间节点
        new_node = self.add_node(split_pos)

        # 保留两端节点的引用，删除原边
        a_id = node_a.node_id
        b_id = node_b.node_id
        self.remove_edge(edge_id)

        # 建两段新边（注意按原方向）
        try:
            self.add_edge(self.nodes[a_id], new_node, geom_1)
            self.add_edge(new_node, self.nodes[b_id], geom_2)
        except ValueError:
            # 极少见的几何不可建（理论上 split_arc_b_points 通过后不应出现）
            self.remove_node(new_node.node_id)
            return None

        return new_node.node_id

    def edge_id_at(self, position: Vec3, epsilon: float = 0.5) -> int | None:
        for edge in self.edges.values():
            node_a = self.nodes[edge.node_a_id]
            node_b = self.nodes[edge.node_b_id]
            d = _point_to_segment_distance(
                position, node_a.position, node_b.position
            )
            if d < epsilon:
                return edge.edge_id
        return None


def _point_to_segment_distance(p: Vec3, a: Vec3, b: Vec3) -> float:
    ab = b - a
    ab_len_sq = ab.length_squared()
    if ab_len_sq == 0.0:
        return p.distance_to(a)
    t = max(0.0, min(1.0, (p - a).dot(ab) / ab_len_sq))
    projection = a + ab * t
    return p.distance_to(projection)


def _compute_arc(a: Vec3, b: Vec3, c: Vec3) -> tuple[Vec3, float, float, Vec3] | None:
    ca = b - a
    cb = b - c
    la = ca.length()
    lc = cb.length()
    if la == 0.0 or lc == 0.0:
        return None

    n = ca.cross(cb)
    nl = n.length()
    if nl < 1e-9:
        return None

    n = n * (1.0 / nl)

    pa = n.cross(ca)
    pa = pa * (1.0 / la)
    pc = n.cross(cb)
    pc = pc * (1.0 / lc)

    cross_pa_pc = pa.cross(pc)
    cpa_pc_len = cross_pa_pc.length()
    if cpa_pc_len < 1e-9:
        return None

    t = (c - a).cross(pc).dot(n) / cross_pa_pc.dot(n)

    center = a + pa * t
    radius = (a - center).length()

    oa = a - center
    oc = c - center
    cos_angle = oa.dot(oc) / (radius * radius)
    cos_angle = max(-1.0, min(1.0, cos_angle))
    sin_angle = oa.cross(oc).dot(n) / (radius * radius)
    angle = math.atan2(sin_angle, cos_angle)

    return center, radius, angle, n
