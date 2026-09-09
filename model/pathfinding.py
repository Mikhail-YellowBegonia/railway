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
    consist_length: float = 0.0,
) -> list[tuple[DirectedEdge, float]]:
    """当前有向边在其 head 节点处的所有合法后继有向边及附加代价。

    合法 = turn_allowed（几何转向许可）且 passable_fn 通过。

    折返约束（既有规定）：列车能且仅能在 endpoint（connection_count()==1）
    处折返，不允许在中间节点/turnout 折返——这不只是简化，是刚体列车的
    物理限制：折返需要车身整体原地翻转，只有死端才有意义，中间节点折返
    在几何上无法定义"车身占用哪一侧"。

    进一步地，即使在 endpoint，也要求该 endpoint 的 simple_segment（到最近
    turnout 之间的无分歧路段）总长 >= consist_length，否则车身放不下这段
    死端，折返会导致车尾越过 turnout（引发"人"字形道岔连通性歧义，寻路时
    不知道列车从哪一撇进入，见折返设计讨论）。simple_segment 长度不足时
    直接不插入折返边，寻路层面就排除这种不可行方案。

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

    # 折返：仅在 endpoint（connection_count()==1）且 simple_segment 长度
    # 足够容纳车身时插入，不再允许中间节点折返。
    if allow_reversal and network.nodes[node_id].connection_count() == 1:
        seg_length, _turnout, _edges = network.simple_segment_from_endpoint(node_id)
        if seg_length >= consist_length:
            reversed_dir = (edge_id, -direction)
            if passable_fn(network.edges[edge_id], -direction):
                overflow = max(0.0, seg_length - consist_length)
                result.append((reversed_dir, REVERSAL_PENALTY + overflow))

    return result


def find_path(
    network: RailNetwork,
    start: DirectedEdge,
    goal_node_id: int,
    *,
    cost_fn: CostFn = _default_cost,
    passable_fn: PassableFn = _default_passable,
    allow_reversal: bool = False,
    goal_directed: DirectedEdge | None = None,
    consist_length: float = 0.0,
) -> Path | None:
    """edge-based Dijkstra：从有向边 start 出发，到达 goal_node_id。

    搜索状态是有向边（edge_id, dir），邻接由 turn_allowed 决定。

    到达条件（二选一）：
    - goal_directed 为 None（默认）：任意有向边的 head 节点 == goal_node_id
      即该边以任意方向走到目标节点即可（无方向约束，行为不变）
    - goal_directed 给定：必须精确走到这条有向边（消除到达方向歧义，
      解决"目标节点有多条汇入边、方向不同代表不同物理终点"的问题）

    allow_reversal=True 时支持在终端节点处折返。
    返回 Path（有向边序列 + 总代价），不可达返回 None。
    """
    if passable_fn is None:
        passable_fn = _default_passable  # 全通（调车/拓扑直查入口可传 None）
    start_edge = network.edges.get(start[0])
    if start_edge is None:
        return None
    if not passable_fn(start_edge, start[1]):
        return None

    start_cost = cost_fn(start_edge, start[1])

    def _is_goal(cur: DirectedEdge) -> bool:
        if goal_directed is not None:
            return cur == goal_directed
        return head_node(network, cur) == goal_node_id

    # start 本身就等于目标有向边时：不能在第 0 步就"trivially"判定命中
    # （那只是"还没离开起点"，不是真正到达）。必须先离开 start，找一条
    # 真正绕回 start 的路径，再把 start 拼回路径最前面。
    # 场景：start_edge_id == goal_edge_id 且方向相同、但目标点在"身后"
    # （已用真实网络验证：不加这个特判，Dijkstra 会把"原地不动"错误地
    # 当成到达，返回一条长度为 0 的假路径，而不是绕路/折返或不可达）。
    #
    # 实现上完全不把 start 放进 dist/prev（不是先放再删）——已用真实网络
    # 复现过死循环：只要 start 曾经作为 key 出现在 dist 里、又被删掉腾
    # 空位，图中所有指向 start 的边（折返场景下必然存在）会把 start 当
    # 成"新发现的普通节点"重新塞回去，形成 prev 环，回溯永远走不出来。
    # 这里改成 start 从头到尾都不是搜索状态空间的一员，只是搜索种子生成
    # 时用一次的输入，回溯结果里也不会天然包含它，最后手动拼到最前面。
    force_leave = goal_directed is not None and start == goal_directed

    dist: dict[DirectedEdge, float] = {}
    prev: dict[DirectedEdge, DirectedEdge | None] = {}
    pq: list[tuple[float, DirectedEdge]] = []

    if force_leave:
        for nxt, extra in neighbors(network, start, passable_fn, allow_reversal, consist_length):
            nd = start_cost + cost_fn(network.edges[nxt[0]], nxt[1]) + extra
            if nd < dist.get(nxt, float("inf")):
                dist[nxt] = nd
                prev[nxt] = None
                heapq.heappush(pq, (nd, nxt))
    else:
        dist[start] = start_cost
        prev[start] = None
        pq.append((start_cost, start))

    goal: DirectedEdge | None = None
    while pq:
        d, cur = heapq.heappop(pq)
        if d > dist.get(cur, float("inf")):
            continue
        if _is_goal(cur):
            goal = cur
            break
        for nxt, extra in neighbors(network, cur, passable_fn, allow_reversal, consist_length):
            nd = d + cost_fn(network.edges[nxt[0]], nxt[1]) + extra
            if nd < dist.get(nxt, float("inf")):
                dist[nxt] = nd
                prev[nxt] = cur
                heapq.heappush(pq, (nd, nxt))

    if goal is None:
        return None

    # 回溯：prev[node] 链一直走到某个 node 的 prev 是 None（该 node 是
    # 搜索种子）。force_leave 场景下种子是 start 的所有后继，它们的
    # prev 被设为 None 作为链的终点，start 本身从未出现在 dist/prev
    # 里，回溯结果天然不含 start，最后统一拼到最前面即可。
    chain: list[DirectedEdge] = []
    node = goal
    while node is not None:
        chain.append(node)
        node = prev[node]
    chain.reverse()
    if force_leave:
        chain.insert(0, start)
    # 注意：force_leave 分支的种子 nd 已经把 start_cost 算进去了
    # （nd = start_cost + edge_cost + extra），dist[goal] 本身就是
    # 含 start 这段的总代价，不能再加一次 start_cost。
    return Path(edges=chain, total_cost=dist[goal])


def find_path_between_nodes(
    network: RailNetwork,
    start_node_id: int,
    goal_node_id: int,
    *,
    cost_fn: CostFn = _default_cost,
    passable_fn: PassableFn = _default_passable,
    allow_reversal: bool = False,
    consist_length: float = 0.0,
) -> Path | None:
    """在两个节点间寻路（交互测试的自然入口）。

    从 start_node_id 枚举所有出发有向边（离开该节点的方向），分别寻路到
    goal_node_id，取总代价最小者。起点终点相同或不可达返回 None。
    allow_reversal=True 时支持在 endpoint 处折返（需 simple_segment 长度
    >= consist_length，默认 0 表示不限制，兼容旧的调试用法）。
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
            consist_length=consist_length,
        )
        if p is not None and (best is None or p.total_cost < best.total_cost):
            best = p
    return best


def find_path_from_point(
    network: RailNetwork,
    start_edge_id: int,
    start_t: float,
    start_direction: int,
    goal_edge_id: int,
    goal_t: float,
    goal_direction: int,
    *,
    cost_fn: CostFn = _default_cost,
    passable_fn: PassableFn = _default_passable,
    allow_reversal: bool = False,
    consist_length: float = 0.0,
    debug: bool = False,
) -> tuple[Path, float, float] | None:
    """从 Edge 途中的点寻路到另一 Edge 途中的点。

    起点/终点都不分割网络，不产生任何碎节点/碎边：Dijkstra 只关心走哪个
    方向、到哪个节点，不关心站在边上具体哪个点；start_offset/end_offset
    各自独立算出，作为标量修正抵消"整条边"和"精确点"之间的差值（跟喂给
    RigidWagonKinematics 的 initial_offset/end_offset 是同一套语义）。

    早前版本曾对终点做"提交时分割"处理（commit 参数），是为了绕开虚拟
    子边还原产生的重复边 bug；但持续分割会在网络里累积碎节点/碎边，需要
    额外 GC 机制清理，属于治标不治本。既然搜索层面本来就能接受"整条边"
    （起点从 Step 3 收尾起就是这么处理的），终点也没必要分割——两端一致，
    网络从寻路阶段起永远不被修改，垃圾从源头上不会产生。commit 参数已
    移除，调用方不再需要"先探路再提交"的两阶段流程，直接一次调用即可。

    start_direction / goal_direction 均为必填（+1 = node_a → node_b，-1 = 反向），
    不再允许隐式猜测方向。理由：目标点若无方向约束，Dijkstra 可能从虚拟节点的
    任一侧到达，而 end_offset 的计算依赖到达方向——方向不定则 end_offset 可能
    算错（表现为列车停止位置轻微偏移的 glitch）。goal_direction 精确锁定寻路
    要到达的有向边，end_offset 据此计算，不再假设固定方向。

    consist_length: 列车总长（米），决定折返在哪些 endpoint 可行——仅
    simple_segment 长度 >= consist_length 的 endpoint 才允许折返（既有
    规定：只能在 endpoint 折返，且需容纳车身，否则车尾会越过 turnout，
    引发"人"字形道岔连通性歧义）。默认 0.0 = 不限制（旧行为，调试用）。

    返回:
        (path, start_offset, end_offset) 或 None（不可达）

        - start_offset: 列车在起始 Edge 上已走过的弧长（按有向边的尾端点算：
          direction=+1 时 = start_t × edge.length，direction=-1 时 =
          (1 - start_t) × edge.length；start_t 恒为 node_a→node_b 参数）
        - end_offset: 终止 Edge 末尾需截去的弧长（按 goal_direction 计算）

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

    # 计算起点偏移（弧长）：已沿有向边走过的弧长。start_t 是 node_a→node_b
    # 参数，方向 -1 时列车从 node_b 出发，已走过 (1 - start_t) × length。
    # 早前这里无条件用 start_t × length，方向 -1 时会把"剩余弧长"当成
    # "已走过弧长"，导致 remaining_to_goal 算错（同 Edge 逆向场景实测算成 0）。
    start_offset = (
        start_t * start_edge.length
        if start_direction > 0
        else (1.0 - start_t) * start_edge.length
    )

    # 目标 Edge 与起始 Edge 相同、且到达方向也相同时的直接处理
    # （方向不同 = 需要经过网络绕路或折返才能反向进入同一条边，落到下方通用分支处理）
    if start_edge_id == goal_edge_id and start_direction == goal_direction:
        if debug:
            print(f"[寻路] 同 Edge 同方向：start_t={start_t:.2f}, goal_t={goal_t:.2f}, dir={start_direction}")
        if start_direction > 0 and start_t <= goal_t:
            directed = _directed_from(network, start_edge_id, start_edge.node_a_id)
            end_offset = (1.0 - goal_t) * goal_edge.length
            path = Path(edges=[directed], total_cost=goal_edge.length)
            return path, start_offset, end_offset
        elif start_direction < 0 and start_t >= goal_t:
            directed = _directed_from(network, start_edge_id, start_edge.node_b_id)
            end_offset = goal_t * goal_edge.length
            path = Path(edges=[directed], total_cost=goal_edge.length)
            return path, start_offset, end_offset
        # 目标在"身后"（同方向但 t 已经过了）：需要绕路或折返，落到通用分支
        if debug:
            print(f"[寻路] 同 Edge 同方向但目标在身后，尝试绕路/折返")

    # 起点/终点都不分割，统一按"整条边+方向"处理——edge-based Dijkstra 只
    # 关心走哪个方向、到哪个节点，不关心站在边上具体哪个点。start_offset /
    # end_offset 各自独立算出，作为标量修正抵消"整条边"和"精确点"之间的
    # 差值（跟 initial_offset/end_offset 喂给 RigidWagonKinematics 的方式
    # 完全一致），不需要 split_edge_at 真正切开网络。
    #
    # 之所以不分割：起点边正是列车 occupancy.occupied 里记录的那条边，
    # 分割它会在还原虚拟子边时产生"同一 edge_id 重复出现"的错误路径
    # （已复现：路径需要连续走完两段虚拟子边时，映射回同一原始 edge_id，
    # 车厢位移计算出的距离变成真实值两倍）。终点边此前虽然改成了"提交时
    # 永久分割"（commit 机制）来规避这个问题，但那样会持续在网络里累积
    # 碎节点/碎边，且需要额外的 GC 机制清理，属于治标不治本。既然搜索
    # 层面本来就能接受"整条边"，终点也没必要分割——两端保持一致，网络
    # 从寻路阶段起就永远不被修改，垃圾从源头上不会产生。
    start_directed = (start_edge_id, start_direction)
    goal_directed = (goal_edge_id, goal_direction)
    end_offset = (1.0 - goal_t) * goal_edge.length if goal_direction > 0 else goal_t * goal_edge.length

    if debug:
        print(f"[寻路] 调用 find_path：start_directed={start_directed}, goal_directed={goal_directed}")
    path = find_path(
        network, start_directed, head_node(network, goal_directed),
        cost_fn=cost_fn, passable_fn=passable_fn,
        allow_reversal=allow_reversal,
        goal_directed=goal_directed,
        consist_length=consist_length,
    )
    if path is None:
        if debug:
            print(f"[寻路失败] find_path 返回 None")
        return None

    if debug:
        print(f"[寻路成功] 路径长度 {len(path.edges)} 段，总代价 {path.total_cost:.1f}")
        print(f"[寻路路径] {[(eid, '+' if d > 0 else '-') for eid, d in path.edges]}")

    return path, start_offset, end_offset

