from __future__ import annotations

import math
from dataclasses import dataclass, field

from model.vec3 import Vec3
from model.spatial_index import SpatialIndex

# 转向许可判据：列车只能前进——到达节点的行进方向与离开节点的行进方向
# 夹角必须 < 90°（d_in · d_out > TURN_ALLOW_DOT_MIN，取 0）。
# 这样交叉渡线 (crossover) 的缓角分股（~30°，dot≈+0.85）许可，而近 180°
# 的发卡掉头（dot≈-0.85）被排除。
#
# 注：早先用 cos(150°)≈-0.866 作阈值，基于"编辑器只产生 0°/180° 切线"的
# 假设。但交叉渡线的 4 联通点存在真实分股角，148° 的发卡弯（dot=-0.847）
# 会通过 -0.847 >= -0.866 而被误判为许可（表现为寻路在渡线口发卡掉头）。
# 改为前进半平面（90°）判据后彻底修复；90° 相对真实道岔角（<15°）已极宽松。
TURN_ALLOW_DOT_MIN = 0.0


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
        # 空间索引（旁挂，见 docs/editor.md §11）
        self._spatial_index = SpatialIndex()

    def add_node(self, position: Vec3) -> Node:
        nid = self._next_node_id
        self._next_node_id += 1
        node = Node(node_id=nid, position=position)
        self.nodes[nid] = node
        # 插桩：同步到空间索引
        self._spatial_index.insert_node(nid, position)
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

        # 插桩：同步到空间索引
        self._spatial_index.insert_edge(eid, edge, node_a, node_b)

        return edge

    def adjacent_edges_at(self, node_id: int, from_edge_id: int) -> set[int]:
        groups = self._connectivity.get(node_id, [])
        for group in groups:
            if from_edge_id in group:
                return group - {from_edge_id}
        target_node = self.nodes[node_id]
        return target_node.incident_edge_ids - {from_edge_id}

    def _edge_dir_at_node(self, edge: Edge, node_id: int, *, leaving: bool) -> Vec3 | None:
        """边在给定端点处的行进切线方向（几何推断转向许可用）。

        - leaving=True：从 node_id 出发、沿边离开的方向（指出节点）。
        - leaving=False：到达 node_id 时的行进方向（指向节点）。

        node_id 必须是 edge 的某个端点，否则返回 None。
        """
        from model.geom_utils import tangent_along_edge

        node_a = self.nodes.get(edge.node_a_id)
        node_b = self.nodes.get(edge.node_b_id)
        if node_a is None or node_b is None:
            return None

        # tangent_along_edge 给的是 node_a -> node_b 的 forward 切线。
        if node_id == edge.node_a_id:
            # node_a 处：forward 即离开方向；到达方向为其反向。
            fwd = tangent_along_edge(edge, node_a, node_b, 0.0)
            return fwd if leaving else fwd * -1.0
        elif node_id == edge.node_b_id:
            # node_b 处：forward 指向节点，即到达方向；离开为其反向。
            fwd = tangent_along_edge(edge, node_a, node_b, 1.0)
            return fwd * -1.0 if leaving else fwd
        return None

    def leaving_direction_at(self, node_id: int, edge_id: int) -> Vec3 | None:
        """从 node_id 出发、沿 edge_id 离开该节点的切线方向（归一化）。

        edge_id 必须是 node_id 的关联边，否则返回 None。
        """
        edge = self.edges.get(edge_id)
        if edge is None:
            return None
        return self._edge_dir_at_node(edge, node_id, leaving=True)

    def resolve_directed_edge_by_click(
        self, node_id: int, click_world_pos: Vec3
    ) -> int | None:
        """按点击方向，在 node_id 的关联边中选出最匹配的一条（返回 edge_id）。

        规则：枚举 incident_edge_ids，取"离开该节点方向"与"点击方向"
        点积最大的那条边。用于信号槛位选择——一个节点最多有
        len(incident_edge_ids) 个独立信号槛位（每个槛位 = 一个
        DirectedEdge，见 model/signal.py），需要确定性地从点击位置消解
        出玩家想要操作哪一个。纯几何计算，不依赖 view/controller。

        退化兜底：click_dir 长度不足以定向时（典型场景：Edge 中途点击
        触发 split_edge_at 后，新节点的位置精确等于点击坐标本身，此时
        click_dir 恒为零——不是概率性边界情况，是这个交互流程下必然
        发生的常规路径），不返回 None，而是确定性选择"该节点作为
        node_a 的那条边"（对应原 edge 的 node_a→node_b 正方向，dir=+1）。
        这样退化时仍有确定结果，玩家想要另一方向只需点击时稍微偏离
        节点、朝目标边方向靠一点，点积路径会自然生效，不需要额外的
        交互手势。

        node_id 不存在或无关联边时返回 None。
        """
        node = self.nodes.get(node_id)
        if node is None or not node.incident_edge_ids:
            return None

        click_dir = click_world_pos - node.position
        if click_dir.length() < 1e-6:
            for eid in sorted(node.incident_edge_ids):
                if self.edges[eid].node_a_id == node_id:
                    return eid
            return min(node.incident_edge_ids)
        click_dir = click_dir.normalize()

        best_edge_id = None
        best_dot = -1.0
        for eid in node.incident_edge_ids:
            leaving_dir = self.leaving_direction_at(node_id, eid)
            if leaving_dir is None:
                continue
            dot = leaving_dir.dot(click_dir)
            if dot > best_dot:
                best_dot = dot
                best_edge_id = eid
        return best_edge_id

    def simple_segment_from_endpoint(
        self, endpoint_node_id: int
    ) -> tuple[float, int | None, list[int]]:
        """从 endpoint（连接数=1 的节点）沿连接数=2 的节点链走，直到遇到
        turnout（连接数>=3）或另一个 endpoint，得到这段 simple_segment。

        simple_segment：不含任何分歧点的一段轨道，一端是 endpoint，另一端
        是 turnout（或另一个 endpoint，此时整段孤立、没有 turnout）。

        提升为 RailNetwork 方法（原为 pathfinding.py 里的自由函数）：
        这是纯图遍历，不依赖寻路概念，原本只服务折返（判断死端 simple
        segment 是否容纳车身），现在是通用基础设施——OpenTTD Path Signal
        的"安全停车位置"判定（见 docs/train_control.md 调研记录）同样
        需要"车身是否会跨在道岔上"这个信息，是同一个几何问题，理应共用
        同一份实现，不是各自重新遍历一遍图。

        参数:
            endpoint_node_id: 起点节点 ID，必须 connection_count() == 1，
                              否则返回 (0.0, None, [])

        返回:
            (total_length, turnout_node_id, edge_ids)
            - total_length: 这段轨道的总弧长（米）
            - turnout_node_id: 终止于的 turnout 节点 ID；若终止于另一个
              endpoint（孤立段，两端都是死端），则为 None
            - edge_ids: 途经的所有 edge_id（按遍历顺序，供调用方需要时使用）
        """
        start_node = self.nodes.get(endpoint_node_id)
        if start_node is None or start_node.connection_count() != 1:
            return 0.0, None, []

        total_length = 0.0
        edge_ids: list[int] = []
        prev_edge_id: int | None = None
        current_node_id = endpoint_node_id

        while True:
            node = self.nodes[current_node_id]
            remaining = node.incident_edge_ids - ({prev_edge_id} if prev_edge_id is not None else set())
            if not remaining:
                # connection_count()==1 且已经是走进来的那条边：说明current_node是死端且已到达
                break
            next_edge_id = next(iter(remaining))
            edge = self.edges[next_edge_id]
            total_length += edge.length
            edge_ids.append(next_edge_id)

            next_node_id = edge.node_b_id if edge.node_a_id == current_node_id else edge.node_a_id
            next_node = self.nodes[next_node_id]
            count = next_node.connection_count()

            if count >= 3:
                return total_length, next_node_id, edge_ids
            if count == 1:
                # 另一端也是 endpoint：孤立段，没有 turnout
                return total_length, None, edge_ids
            # count == 2：继续沿链走
            prev_edge_id = next_edge_id
            current_node_id = next_node_id

        return total_length, None, edge_ids

    def turn_allowed(self, node_id: int, from_edge_id: int, to_edge_id: int) -> bool:
        """经节点 node_id 从 from_edge 转到 to_edge 是否几何许可。

        判据：**前进半平面**——到达节点的行进方向 d_in 与离开节点的行进方向
        d_out 夹角**严格 < 90°**（`d_in · d_out > TURN_ALLOW_DOT_MIN`，常量取 0）。
        这样直通（0°）与缓分股（~32°）许可，正交（90°）、发卡弯（~148°）、
        掉头（180°）全部排除。纯几何推断，无需道岔配置。
        （初版用 cos(150°) 阈值，交叉渡线的 4 联通点会误判 148° 发卡弯为许可；
        改为前进半平面后彻底修复——本 docstring 此前漏改，2026-09-10 补正。）

        约束：to_edge 必须在 from_edge 经该节点的拓扑邻接集内，否则不许可。
        原路返回（from == to）视为掉头，不许可。
        """
        if from_edge_id == to_edge_id:
            return False
        if to_edge_id not in self.adjacent_edges_at(node_id, from_edge_id):
            return False

        from_edge = self.edges.get(from_edge_id)
        to_edge = self.edges.get(to_edge_id)
        if from_edge is None or to_edge is None:
            return False

        d_in = self._edge_dir_at_node(from_edge, node_id, leaving=False)
        d_out = self._edge_dir_at_node(to_edge, node_id, leaving=True)
        if d_in is None or d_out is None:
            return False

        # 严格大于：dot=0（正交 90°）也排除，铁路不存在直角转向。
        return d_in.dot(d_out) > TURN_ALLOW_DOT_MIN

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

        # 插桩：先从空间索引删除
        self._spatial_index.remove_edge(edge_id)

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
        # 插桩：从空间索引删除 Node
        self._spatial_index.remove_node(node_id, node.position)

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

    def nearby_node_ids(self, pos: Vec3, radius: float) -> set[int]:
        """返回距 pos 世界距离可能 <= radius 的 Node ID 候选集。

        候选集可能略多（瓦片邻域粒度），调用方需按实际距离精算筛选。
        实现见 model/spatial_index.py（设计依据 docs/editor.md §11）。
        """
        return self._spatial_index.nearby_node_ids(pos, radius)

    def nearby_edge_ids(self, pos: Vec3, radius: float) -> set[int]:
        """返回距 pos 世界距离可能 <= radius 的 Edge ID 候选集。

        候选集可能略多（瓦片邻域粒度），调用方需按实际距离精算筛选。
        实现见 model/spatial_index.py（设计依据 docs/editor.md §11）。
        """
        return self._spatial_index.nearby_edge_ids(pos, radius)


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
