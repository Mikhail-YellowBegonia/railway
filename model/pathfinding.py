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

    # 折返：allow_reversal=True 时允许在任意节点沿原边反向出发
    # （不限于终端节点，支持复杂场景下的中途折返）
    if allow_reversal:
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
    start_direction: int = 1,
    cost_fn: CostFn = _default_cost,
    passable_fn: PassableFn = _default_passable,
    allow_reversal: bool = False,
    debug: bool = False,
) -> tuple[Path, float, float] | None:
    """从 Edge 途中的点寻路到另一 Edge 途中的点。

    不修改原网络；在内存中临时分割目标 Edge，寻路完成后丢弃临时副本。

    参数:
        start_direction: 列车当前朝向（+1 = node_a → node_b，-1 = 反向）
                        决定从哪端节点出发，禁止反向折返

    返回:
        (path, start_offset, end_offset) 或 None（不可达）

        - start_offset: 列车在起始 Edge 上已走过的弧长（= start_t × edge.length）
        - end_offset: 终止 Edge 末尾需截去的弧长（= (1-goal_t) × edge.length）

        调用方使用方式：
            kin = RigidWagonKinematics(network, path, consist,
                                       initial_offset=start_offset,
                                       end_offset=end_offset)
    """
    if debug:
        print(f"\n[寻路开始] edge {start_edge_id} t={start_t:.2f} dir={start_direction} → edge {goal_edge_id} t={goal_t:.2f}")
        print(f"[寻路参数] allow_reversal={allow_reversal}")

    start_edge = network.edges.get(start_edge_id)
    goal_edge = network.edges.get(goal_edge_id)
    if start_edge is None or goal_edge is None:
        if debug:
            print(f"[寻路失败] edge 不存在")
        return None

    # 计算起点偏移（弧长）
    start_offset = start_t * start_edge.length

    # 起点节点：列车当前所在节点（用于端点附近的 fallback）
    # 注意：这里不是"朝向的节点"，而是"当前位置最近的节点"
    if start_t < 0.5:
        start_node_id = start_edge.node_a_id
    else:
        start_node_id = start_edge.node_b_id

    # 目标 Edge 与起始 Edge 相同时的特殊处理
    if start_edge_id == goal_edge_id:
        if debug:
            print(f"[寻路] 同 Edge：start_t={start_t:.2f}, goal_t={goal_t:.2f}, dir={start_direction}, allow_reversal={allow_reversal}")
        # 检查是否需要折返（目标在反方向）
        if start_direction > 0 and start_t <= goal_t:
            # 正方向，目标在前方
            if debug:
                print(f"[寻路] 同 Edge 正方向，目标在前方，直接返回")
            directed = _directed_from(network, start_edge_id, start_edge.node_a_id)
            end_offset = (1.0 - goal_t) * goal_edge.length
            path = Path(edges=[directed], total_cost=goal_edge.length)
            return path, start_offset, end_offset
        elif start_direction < 0 and start_t >= goal_t:
            # 负方向，目标在前方
            if debug:
                print(f"[寻路] 同 Edge 负方向，目标在前方，直接返回")
            directed = _directed_from(network, start_edge_id, start_edge.node_b_id)
            end_offset = goal_t * goal_edge.length
            path = Path(edges=[directed], total_cost=goal_edge.length)
            return path, start_offset, end_offset
        # 否则目标在反方向，不允许折返，返回 None
        if not allow_reversal:
            if debug:
                print(f"[寻路] 同 Edge 目标在反方向，allow_reversal=False，返回 None")
            return None
        # allow_reversal=True 时才尝试绕路
        if debug:
            print(f"[寻路] 同 Edge 目标在反方向，allow_reversal=True，尝试绕路")

    # 在临时网络副本中分割起始和目标 Edge
    tmp_network = copy.deepcopy(network)
    original_edge_ids_before_split = set(tmp_network.edges.keys())

    # 分割起始 edge（如果不在端点）
    start_directed: DirectedEdge | None = None
    if 0.01 < start_t < 0.99:
        start_virtual_node = tmp_network.split_edge_at(start_edge_id, start_t)
        if start_virtual_node is not None:
            # 找到虚拟节点出发、沿 start_direction 方向的边
            vnode = tmp_network.nodes[start_virtual_node]
            for eid in vnode.incident_edge_ids:
                edge = tmp_network.edges[eid]
                if start_direction > 0 and edge.node_a_id == start_virtual_node:
                    # 正方向：vnode → node_b（原 edge 的 node_b）
                    start_directed = (eid, 1)
                    break
                elif start_direction < 0 and edge.node_b_id == start_virtual_node:
                    # 负方向：vnode → node_a（原 edge 的 node_a）
                    start_directed = (eid, -1)
                    break

    # 如果没有分割（端点附近）或分割失败，从端点出发
    if start_directed is None:
        # 根据 start_t 和 start_direction 确定起始有向边
        if start_t < 0.5:
            # 靠近 node_a
            if start_direction > 0:
                # 正方向：node_a → node_b
                start_directed = (start_edge_id, 1)
            else:
                # 负方向：需要从 node_a 继续向前（沿反向）
                # 但这需要找到 node_a 的其他邻接边，暂时用当前边反向
                start_directed = (start_edge_id, -1)
        else:
            # 靠近 node_b
            if start_direction > 0:
                # 正方向：需要从 node_b 继续向前
                # 这种情况下应该从 node_b 的邻接边出发，但简化为当前边正向
                start_directed = (start_edge_id, 1)
            else:
                # 负方向：node_b → node_a
                start_directed = (start_edge_id, -1)

    # 记录起始edge分割后产生的新边
    new_edges_from_start = set(tmp_network.edges.keys()) - original_edge_ids_before_split

    # 分割目标 edge
    virtual_node_id = tmp_network.split_edge_at(goal_edge_id, goal_t)
    if virtual_node_id is None:
        # 分割失败（t 极端或几何不可解），降级到最近端节点
        virtual_node_id = goal_edge.node_a_id if goal_t < 0.5 else goal_edge.node_b_id

    # 记录目标edge分割后产生的新边
    new_edges_from_goal = set(tmp_network.edges.keys()) - original_edge_ids_before_split - new_edges_from_start

    # 在临时网络上寻路：从有向边开始，到虚拟节点
    if debug:
        print(f"[寻路] 调用 find_path：start_directed={start_directed}, goal_node={virtual_node_id}")
    path = find_path(
        tmp_network, start_directed, virtual_node_id,
        cost_fn=cost_fn, passable_fn=passable_fn,
        allow_reversal=allow_reversal,
    )
    if path is None:
        if debug:
            print(f"[寻路失败] find_path 返回 None")
        return None

    # 将临时网络路径映射回原网络的 edge_id
    end_offset = (1.0 - goal_t) * goal_edge.length

    restored_edges: list[DirectedEdge] = []
    for eid, d in path.edges:
        if eid in new_edges_from_start:
            # 起始edge的虚拟边，还原为 start_edge_id
            restored_edges.append((start_edge_id, d))
        elif eid in new_edges_from_goal:
            # 目标edge的虚拟边，还原为 goal_edge_id
            restored_edges.append((goal_edge_id, d))
        else:
            restored_edges.append((eid, d))
    path = Path(edges=restored_edges, total_cost=path.total_cost)

    if debug:
        print(f"[寻路成功] 路径长度 {len(path.edges)} 段，总代价 {path.total_cost:.1f}")
        print(f"[寻路路径] {[(eid, '+' if d > 0 else '-') for eid, d in path.edges]}")

    return path, start_offset, end_offset

