from __future__ import annotations

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


def neighbors(
    network: RailNetwork,
    directed: DirectedEdge,
    passable_fn: PassableFn,
) -> list[DirectedEdge]:
    """当前有向边在其 head 节点处的所有合法后继有向边。

    合法 = turn_allowed（几何转向许可）且 passable_fn 通过。
    """
    edge_id, _ = directed
    node_id = head_node(network, directed)

    result: list[DirectedEdge] = []
    for to_edge_id in network.adjacent_edges_at(node_id, edge_id):
        if not network.turn_allowed(node_id, edge_id, to_edge_id):
            continue
        nxt = _directed_from(network, to_edge_id, node_id)
        if not passable_fn(network.edges[nxt[0]], nxt[1]):
            continue
        result.append(nxt)
    return result


def find_path(
    network: RailNetwork,
    start: DirectedEdge,
    goal_node_id: int,
    *,
    cost_fn: CostFn = _default_cost,
    passable_fn: PassableFn = _default_passable,
) -> Path | None:
    """edge-based Dijkstra：从有向边 start 出发，到达 goal_node_id。

    搜索状态是有向边（edge_id, dir），邻接由 turn_allowed 决定。到达条件是
    某条有向边的 head 节点 == goal_node_id（即该边已完整走到目标节点）。
    返回 Path（有向边序列 + 总代价），不可达返回 None。
    """
    start_edge = network.edges.get(start[0])
    if start_edge is None:
        return None
    if not passable_fn(start_edge, start[1]):
        return None

    # 起点即到达：start 的 head 就是目标。
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
        for nxt in neighbors(network, cur, passable_fn):
            nd = d + cost_fn(network.edges[nxt[0]], nxt[1])
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
) -> Path | None:
    """在两个节点间寻路（交互测试的自然入口）。

    从 start_node_id 枚举所有出发有向边（离开该节点的方向），分别寻路到
    goal_node_id，取总代价最小者。起点终点相同或不可达返回 None。
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
        )
        if p is not None and (best is None or p.total_cost < best.total_cost):
            best = p
    return best

