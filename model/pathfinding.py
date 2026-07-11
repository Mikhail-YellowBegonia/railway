from __future__ import annotations

import copy
import heapq
from dataclasses import dataclass
from typing import Callable

from model.rail_network import Edge, RailNetwork

# 有向边：(edge_id, dir)，dir=+1 沿 node_a->node_b，dir=-1 反向。
DirectedEdge = tuple[int, int]

# 代价 / 可通行钩子（信号层以后往这里注入约束）。
CostFn = Callable[[Edge, int], float]
PassableFn = Callable[[Edge, int], bool]


def _default_cost(edge: Edge, direction: int) -> float:
    return edge.length


def _default_passable(edge: Edge, direction: int) -> bool:
    return True


@dataclass
class Path:
    """一条有向边序列构成的路径（寻路输出 / 运动学输入的契约）。

    edges 里每个元素是 (edge_id, dir)，按行进顺序排列。相邻两条有向边
    在共享节点处满足 turn_allowed。total_cost 为累计代价（默认即总弧长）。

    折返标记：edges 中可出现连续两条 dir 相反的同 edge_id，表示在终端节点
    处折返（反向行驶）。PathKinematics 天然支持，无需特殊处理。
    """

    edges: list[DirectedEdge]
    total_cost: float

    def __bool__(self) -> bool:
        return bool(self.edges)


def head_node(network: RailNetwork, directed: DirectedEdge) -> int:
    """有向边前进到达的节点：dir=+1 -> node_b，dir=-1 -> node_a。"""
    edge_id, direction = directed
    edge = network.edges[edge_id]
    return edge.node_b_id if direction > 0 else edge.node_a_id


def tail_node(network: RailNetwork, directed: DirectedEdge) -> int:
    """有向边出发的节点：dir=+1 -> node_a，dir=-1 -> node_b。"""
    edge_id, direction = directed
    edge = network.edges[edge_id]
    return edge.node_a_id if direction > 0 else edge.node_b_id


def _directed_from(network: RailNetwork, edge_id: int, entry_node_id: int) -> DirectedEdge:
    """从 entry_node_id 进入 edge_id 时的有向边定向。

    从 node_a 进入则沿 node_a->node_b（dir=+1），否则 dir=-1。
    """
    edge = network.edges[edge_id]
    return (edge_id, 1) if entry_node_id == edge.node_a_id else (edge_id, -1)


# 换向惩罚（秒等效弧长，加在折返代价上）
REVERSAL_PENALTY = 200.0


def neighbors(
    network: RailNetwork,
    directed: DirectedEdge,
    passable_fn: PassableFn,
    allow_reversal: bool = False,
) -> list[tuple[DirectedEdge, float]]:
    """当前有向边在其 head 节点处的所有合法后继有向边及附加代价。

    合法 = turn_allowed（几何转向许可）且 passable_fn 通过。
    allow_reversal=True 时，在连接数=1 的终端节点处额外注入反向同 Edge
    作为折返选项，附加 REVERSAL_PENALTY 代价。

    返回: list of (DirectedEdge, extra_cost)，extra_cost 通常为 0，折返时为惩罚值。
    """
    edge_id, direction = directed
    node_id = head_node(network, directed)

    result: list[tuple[DirectedEdge, float]] = []
    for to_edge_id in network.adjacent_edges_at(node_id, edge_id):
        if not network.turn_allowed(node_id, edge_id, to_edge_id):
            continue
        nxt = _directed_from(network, to_edge_id, node_id)
        if not passable_fn(network.edges[nxt[0]], nxt[1]):
            continue
        result.append((nxt, 0.0))

    # 折返：终端节点（连接数=1）时允许沿原边反向出发
    if allow_reversal and network.nodes[node_id].connection_count() == 1:
        reversed_dir = (edge_id, -direction)
        if passable_fn(network.edges[edge_id], -direction):
            result.append((reversed_dir, REVERSAL_PENALTY))

    return result


def find_path(
    network: RailNetwork,
    start: DirectedEdge,
    goal_node_id: int,
    *,
    cost_fn: CostFn = _default_cost,
    passable_fn: PassableFn = _default_passable,
    allow_reversal: bool = False,
) -> Path | None:
    """edge-based Dijkstra：从有向边 start 出发，到达 goal_node_id。

    搜索状态是有向边（edge_id, dir），邻接由 turn_allowed 决定。到达条件是
    某条有向边的 head 节点 == goal_node_id（即该边已完整走到目标节点）。
    allow_reversal=True 时支持在终端节点处折返。
    返回 Path（有向边序列 + 总代价），不可达返回 None。
    """
    start_edge = network.edges.get(start[0])
    if start_edge is None:
        return None
    if not passable_fn(start_edge, start[1]):
        return None

    start_cost = cost_fn(start_edge, start[1])

    dist: dict[DirectedEdge, float] = {start: start_cost}
    prev: dict[DirectedEdge, DirectedEdge | None] = {start: None}
    pq: list[tuple[float, DirectedEdge]] = [(start_cost, start)]

    goal: DirectedEdge | None = None
    while pq:
        d, cur = heapq.heappop(pq)
        if d > dist.get(cur, float("inf")):
            continue
        if head_node(network, cur) == goal_node_id:
            goal = cur
            break
        for nxt, extra in neighbors(network, cur, passable_fn, allow_reversal):
            nd = d + cost_fn(network.edges[nxt[0]], nxt[1]) + extra
            if nd < dist.get(nxt, float("inf")):
                dist[nxt] = nd
                prev[nxt] = cur
                heapq.heappush(pq, (nd, nxt))

    if goal is None:
        return None

    # 回溯
    chain: list[DirectedEdge] = []
    node: DirectedEdge | None = goal
    while node is not None:
        chain.append(node)
        node = prev[node]
    chain.reverse()
    return Path(edges=chain, total_cost=dist[goal])


def find_path_between_nodes(
    network: RailNetwork,
    start_node_id: int,
    goal_node_id: int,
    *,
    cost_fn: CostFn = _default_cost,
    passable_fn: PassableFn = _default_passable,
    allow_reversal: bool = False,
) -> Path | None:
    """在两个节点间寻路（交互测试的自然入口）。

    从 start_node_id 枚举所有出发有向边（离开该节点的方向），分别寻路到
    goal_node_id，取总代价最小者。起点终点相同或不可达返回 None。
    allow_reversal=True 时支持在终端节点处折返。
    """
    if start_node_id == goal_node_id:
        return None
    start_node = network.nodes.get(start_node_id)
    if start_node is None:
        return None

    best: Path | None = None
    for edge_id in start_node.incident_edge_ids:
        directed = _directed_from(network, edge_id, start_node_id)
        p = find_path(
            network, directed, goal_node_id,
            cost_fn=cost_fn, passable_fn=passable_fn,
            allow_reversal=allow_reversal,
        )
        if p is not None and (best is None or p.total_cost < best.total_cost):
            best = p
    return best


def find_path_from_point(
    network: RailNetwork,
    start_edge_id: int,
    start_t: float,
    goal_edge_id: int,
    goal_t: float,
    *,
    cost_fn: CostFn = _default_cost,
    passable_fn: PassableFn = _default_passable,
    allow_reversal: bool = False,
) -> tuple[Path, float, float] | None:
    """从 Edge 途中的点寻路到另一 Edge 途中的点。

    不修改原网络；在内存中临时分割目标 Edge，寻路完成后丢弃临时副本。

    返回:
        (path, start_offset, end_offset) 或 None（不可达）

        - start_offset: 列车在起始 Edge 上已走过的弧长（= start_t × edge.length）
        - end_offset: 终止 Edge 末尾需截去的弧长（= (1-goal_t) × edge.length）

        调用方使用方式：
            kin = RigidWagonKinematics(network, path, consist,
                                       initial_offset=start_offset,
                                       end_offset=end_offset)
    """
    start_edge = network.edges.get(start_edge_id)
    goal_edge = network.edges.get(goal_edge_id)
    if start_edge is None or goal_edge is None:
        return None

    # 计算起点偏移（弧长）
    start_offset = start_t * start_edge.length

    # 起点所在 Edge 的出发节点：列车朝向决定从哪端出发
    # 约定 t < 0.5 时从 node_a 出发，否则从 node_b 出发（当前测试无方向信息，用最近端）
    start_node_id = start_edge.node_a_id if start_t < 0.5 else start_edge.node_b_id

    # 目标 Edge 与起始 Edge 相同时的特殊处理
    if start_edge_id == goal_edge_id:
        if start_t <= goal_t:
            # 直接向前，不需要绕路
            directed = _directed_from(network, start_edge_id, start_edge.node_a_id)
            end_offset = (1.0 - goal_t) * goal_edge.length
            path = Path(edges=[directed], total_cost=goal_edge.length)
            return path, start_offset, end_offset
        # 否则需要绕一圈，继续走正常寻路

    # 在临时网络副本中分割目标 Edge，插入虚拟节点
    tmp_network = copy.deepcopy(network)
    virtual_node_id = tmp_network.split_edge_at(goal_edge_id, goal_t)
    if virtual_node_id is None:
        # 分割失败（t 极端或几何不可解），降级到最近端节点
        virtual_node_id = goal_edge.node_a_id if goal_t < 0.5 else goal_edge.node_b_id

    # 在临时网络上寻路
    path = find_path_between_nodes(
        tmp_network, start_node_id, virtual_node_id,
        cost_fn=cost_fn, passable_fn=passable_fn,
        allow_reversal=allow_reversal,
    )
    if path is None:
        return None

    # 将临时网络路径映射回原网络的 edge_id（虚拟分割产生了新 edge_id，需要还原）
    # split_edge_at 保留了原 edge_id（前半段），新增了一个新 edge_id（后半段）
    # 路径末段的 edge_id 可能是原 edge 或新 edge，但 end_offset 已经编码在目标位置
    end_offset = (1.0 - goal_t) * goal_edge.length

    # 路径里的 edge_id 若属于新增虚拟边，需映射回原 edge_id
    # 原 edge 被分成两段：前段保留原 id，后段是新 id（tmp_network 中 max edge_id）
    # 对于 path 里出现新 id 的情况，将其还原为原 goal_edge_id
    new_edge_ids = set(tmp_network.edges.keys()) - set(network.edges.keys())
    restored_edges: list[DirectedEdge] = []
    for eid, d in path.edges:
        if eid in new_edge_ids:
            # 这是后半段虚拟边，还原为原 edge_id（方向不变）
            restored_edges.append((goal_edge_id, d))
        else:
            restored_edges.append((eid, d))
    path = Path(edges=restored_edges, total_cost=path.total_cost)

    return path, start_offset, end_offset

