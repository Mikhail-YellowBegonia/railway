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


def simple_segment_from_endpoint(
    network: RailNetwork, endpoint_node_id: int
) -> tuple[float, int | None, list[int]]:
    """从 endpoint（连接数=1 的节点）沿连接数=2 的节点链走，直到遇到
    turnout（连接数>=3）或另一个 endpoint，得到这段 simple_segment。

    simple_segment：不含任何分歧点的一段轨道，一端是 endpoint，另一端是
    turnout（或另一个 endpoint，此时整段孤立、没有 turnout）。折返是否
    允许取决于这段总长是否 >= 列车长度（见 pathfinding.neighbors）。

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
    start_node = network.nodes.get(endpoint_node_id)
    if start_node is None or start_node.connection_count() != 1:
        return 0.0, None, []

    total_length = 0.0
    edge_ids: list[int] = []
    prev_edge_id: int | None = None
    current_node_id = endpoint_node_id

    while True:
        node = network.nodes[current_node_id]
        remaining = node.incident_edge_ids - ({prev_edge_id} if prev_edge_id is not None else set())
        if not remaining:
            # connection_count()==1 且已经是走进来的那条边：说明current_node是死端且已到达
            break
        next_edge_id = next(iter(remaining))
        edge = network.edges[next_edge_id]
        total_length += edge.length
        edge_ids.append(next_edge_id)

        next_node_id = edge.node_b_id if edge.node_a_id == current_node_id else edge.node_a_id
        next_node = network.nodes[next_node_id]
        count = next_node.connection_count()

        if count >= 3:
            return total_length, next_node_id, edge_ids
        if count == 1:
            # 另一端也是 endpoint：孤立段，没有 turnout
            return total_length, None, edge_ids
        # count == 2：继续沿链走
        prev_edge_id = next_edge_id
        current_node_id = next_node_id


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
        seg_length, _turnout, _edges = simple_segment_from_endpoint(network, node_id)
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
    start_edge = network.edges.get(start[0])
    if start_edge is None:
        return None
    if not passable_fn(start_edge, start[1]):
        return None

    start_cost = cost_fn(start_edge, start[1])

    dist: dict[DirectedEdge, float] = {start: start_cost}
    prev: dict[DirectedEdge, DirectedEdge | None] = {start: None}
    pq: list[tuple[float, DirectedEdge]] = [(start_cost, start)]

    def _is_goal(cur: DirectedEdge) -> bool:
        if goal_directed is not None:
            return cur == goal_directed
        return head_node(network, cur) == goal_node_id

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
    commit: bool = False,
    debug: bool = False,
) -> tuple[Path, float, float] | None:
    """从 Edge 途中的点寻路到另一 Edge 途中的点。

    起点不分割：Dijkstra 只关心走哪个方向、到哪个节点，不关心站在边上
    具体哪个点；start_offset 单独修正这段精度。这是有意的设计（而非疏漏）
    ——起点边正是列车 occupancy.occupied 里记录的那条边，分割它会在还原
    虚拟子边时产生"同一 edge_id 重复出现"的错误路径（已复现：路径需要
    连续走完两段虚拟子边时，映射回同一原始 edge_id，车厢位移计算出的
    距离变成真实值两倍）。

    终点分割由 commit 控制：
    - commit=False（默认，探路模式）：在临时网络副本上分割，仅用于比较
      不同 goal_direction 的代价，不应采用返回的 path.edges 去下达指令
    - commit=True（正式提交，玩家已确定要走这条路）：直接在传入的 network
      上永久分割 goal_edge，返回的 path.edges 全部是真实 edge_id，可以
      放心用于 assign_route。分割点即为路径终点，end_offset 恒为 0

    调用方典型用法：先两次 commit=False 调用比较 +1/-1 哪个方向代价更低，
    对选中的方向再发起一次 commit=True 调用取得可下达的干净路径。

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

        - start_offset: 列车在起始 Edge 上已走过的弧长（= start_t × edge.length）
        - end_offset: 终止 Edge 末尾需截去的弧长（commit=True 时恒为 0）

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

    # 起点不分割，统一按"整条边+方向"出发——edge-based Dijkstra 只关心走哪个
    # 方向、到哪个节点，不关心站在边上具体哪个点，start_offset 已经单独修正
    # 了这段精度（total_cost 会包含整条 start_edge，调用方用 total_cost -
    # start_offset - end_offset 抵消，结果等价）。
    #
    # 之所以不分割起点：起点边此刻正是列车 occupancy.occupied 里记录的
    # 那条边，若在临时网络里分割它，映射回原 edge_id 会在"路径需要连续
    # 走完两段虚拟子边"时把它们压扁成两条重复的完整边（已复现：一段完整
    # 穿越 + 后续绕路回来又走后半段，还原后变成同一个 edge_id 出现两次，
    # 车厢位移计算出的距离变成真实值两倍，即用户报告的跳变）。终点边没有
    # 这个问题（不属于当前占用范围），仍然分割，只是分割现在改为在提交时
    # 永久生效（见下方 commit 分支），不再需要虚拟子边还原逻辑。
    start_directed = (start_edge_id, start_direction)

    # 同 edge_id 场景（start_edge_id == goal_edge_id 但方向不同，需绕路/折返）：
    # 起点不再分割，goal_edge_id 与 start_edge_id 相同时该边仍完整存在；
    # Dijkstra 从整条 start_edge 出发本来就无法在同一条边内部反向掉头
    # （那需要折返，是终端节点的行为），落到通用搜索也会得到正确的
    # 绕路/折返结果，不需要再特殊拒绝。

    # 终点 edge 的分割：commit=False（默认，探路模式）在临时副本上分割，
    # 只用来算 total_cost 这个数字（调用方在两个 goal_direction 间比较
    # 代价，不会真的采用这次返回的 path.edges）；commit=True（正式提交，
    # 玩家已经选定要走这条路）直接在传入的真实 network 上永久分割，
    # 分割点本身就是新节点端点，返回的 path.edges 全是真实 edge_id，
    # 不需要任何"虚拟子边还原"逻辑——这正是消除重复插入 bug 的关键：
    # 之前还原逻辑把虚拟子边压扁回原 edge_id 时，会在"路径连续走完两段
    # 虚拟子边"的场景下产生同一 edge_id 重复出现的错误路径。
    work_network = network if commit else copy.deepcopy(network)
    virtual_node_id = work_network.split_edge_at(goal_edge_id, goal_t)

    if virtual_node_id is None:
        # 分割失败（t 极端，边未被拆分）：原边仍完整存在，直接钉死目标
        goal_directed = (goal_edge_id, goal_direction)
        end_offset = (1.0 - goal_t) * goal_edge.length if goal_direction > 0 else goal_t * goal_edge.length
    else:
        vnode = work_network.nodes[virtual_node_id]
        goal_directed = None
        for eid in vnode.incident_edge_ids:
            edge = work_network.edges[eid]
            if goal_direction > 0 and edge.node_b_id == virtual_node_id:
                goal_directed = (eid, 1)
                break
            elif goal_direction < 0 and edge.node_a_id == virtual_node_id:
                goal_directed = (eid, -1)
                break
        if goal_directed is None:
            if debug:
                print(f"[寻路失败] 无法按 goal_direction={goal_direction} 定位目标有向边")
            return None
        # 分割后目标边就是虚拟子边本身，t=0 处即终点，end_offset 恒为 0
        end_offset = 0.0

    if debug:
        print(f"[寻路] 调用 find_path：start_directed={start_directed}, goal_directed={goal_directed}, commit={commit}")
    path = find_path(
        work_network, start_directed, head_node(work_network, goal_directed),
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

