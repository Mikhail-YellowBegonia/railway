"""锚点解析器（计划层 P2：把"条目 + 路径限定"解析成一条 `route`）。

规格：`docs/plan_layer_roadmap.md` §4.1（Q22-3 硬约束 / Q22-5 控制点 /
Q21-2 锚点是"条目的路径限定"）。这是纯模型解析器：确定性条目由编辑器调用并冻结，
声明式条目可由调度器在激活/刷新时调用并生成运行期解析快照；解析器自身不修改列车。

**一次解析做什么**

1. 把一个条目（`PlanItem`）的目标 + **有序锚点序列**拆成若干"段"：
   `起点 → 锚点₁ → 锚点₂ → … → 终点`；
2. 每段用现成的 `model.pathfinding.find_path` 求路（**不切边、不改图**——
   锚点一律是图上已有的节点）；
3. 锚点处要求"**进得来、出得去**"：把"经过节点 N"表达成**到达 N 的有向边**，
   并枚举该节点的**合法转向对** `(到达边 → 离开边)`（`turn_allowed`）——
   **控制点**就是"离开边被钉死"的那些转向对（`Anchor.exit_edge_id`，
   等价于现实里的进路 / 道岔定反位）；
4. **逐锚点做 DP**（每个锚点保留"到达边→离开边"组合的最优代价，顺序合并）：
   相邻两段各自独立求完再拼，会在锚点处拼成一个**非法转向**（例如原路折回），
   DP 正是为了排除它；
5. **失败即失败**（Q22-3 硬约束）：任何一段不可达或锚点无法通行 ⇒ 整个条目解析
   失败，**绝不自动改走别的路**；失败原因随结果返回，供 GUI 显示（Q23-3）。

**产出契约与现有链路完全一致**（⇒ 执行层零改动）：`ResolvedPath` 给
`edges` / `total_cost` / `start_offset` / `end_offset`，`remaining_to_goal`
即 `total_cost - start_offset - end_offset`；P3 只再加"减去停车提前量
`stop_before_m`"（连挂驶向用）并做首边匹配（`route = edges[1:]`，与
`GameLoop._apply_route_result` 现成逻辑同构）。

**无锚点时与现有寻路同解**：锚点为空 ⇒ 直接委托 `find_path_from_point`，
行为与右键寻路一字不差（`tests/test_plan_path.py` 有等价断言）。
"""

from __future__ import annotations

from dataclasses import dataclass

from model.pathfinding import (
    DirectedEdge,
    Path,
    find_path,
    find_path_from_point,
    head_node,
    tail_node,
)
from model.plan import Anchor, FixedRoute, Goal, PlanCommand, PlanItem
from model.rail_network import RailNetwork


@dataclass(frozen=True)
class PathStart:
    """解析起点：车头所在的有向边位置（与 `find_path_from_point` 同形）。"""

    edge_id: int
    t: float
    direction: int


@dataclass(frozen=True)
class ResolvedPath:
    """解析结果：一条完整的有向边序列 + 偏移修正。"""

    edges: tuple[DirectedEdge, ...]
    total_cost: float
    start_offset: float
    end_offset: float

    @property
    def remaining_to_goal(self) -> float:
        """车头前方到终点的剩余弧长（米）。P3 再减去停车提前量。"""
        return max(0.0, self.total_cost - self.start_offset - self.end_offset)

    def used_edge_ids(self) -> tuple[int, ...]:
        """路线用到的边 id（按顺序，跨越折返点时会重复出现）。"""
        return tuple(edge_id for edge_id, _direction in self.edges)

    def touches_node(self, network: RailNetwork, node_id: int) -> bool:
        """路线是否触及某节点（作为任意一条边的头或尾）。"""
        for directed in self.edges:
            if tail_node(network, directed) == node_id or head_node(network, directed) == node_id:
                return True
        return False

    def passes_through(self, network: RailNetwork, node_id: int) -> bool:
        """路线是否**经过**某节点：它是某条边的头、且是下一条边的尾。

        这正是锚点约束成立的判据——起点自身只作为"尾"出现，不算经过。
        """
        for i in range(len(self.edges) - 1):
            if (
                head_node(network, self.edges[i]) == node_id
                and tail_node(network, self.edges[i + 1]) == node_id
            ):
                return True
        return False

    def freeze(self) -> FixedRoute:
        """冻结编辑期解析结果，供 P3a 确认计划条目使用。

        此方法是 P2 到 P3a 的唯一数据转换点。返回值不携带网络或寻路器引用；P3b
        只读取它，不能回调 `resolve_plan_item()` 或 Dijkstra。
        """
        return FixedRoute(
            edges=self.edges,
            start_offset=self.start_offset,
            end_offset=self.end_offset,
        )


@dataclass(frozen=True)
class PlanResolution:
    """解析结果 + 失败原因（供 Q23-3 的"可见 + 带原因"直接用）。"""

    path: ResolvedPath | None = None
    failure: str = ""
    is_no_path: bool = False  # 等待类条目：本就不产生路径（不是失败）

    @property
    def ok(self) -> bool:
        return self.path is not None


def resolve_plan_item(
    network: RailNetwork,
    start: PathStart,
    item: PlanItem,
    *,
    passable_fn=None,
    allow_reversal: bool = False,
    consist_length: float = 0.0,
    couple_target: Goal | None = None,
) -> PlanResolution:
    """解析一个计划条目。

    - `passable_fn`：透传给寻路（信号层注入；`None` = 全通）。
    - `couple_target`：连挂类条目的**编辑期目标点**。调用方可从兼容 `TrainRef`
      或 `CoupleSelector` 创建时点击的端头位置得到“边 + t + 到达方向”；确认后只冻结
      路径，运行期由 P7c selector 重新解析并锁定具体端头。缺失即解析失败。
    """
    problems = item.validate(network)
    if problems:
        return PlanResolution(failure="条目非法：" + problems[0])

    if item.command in (
        PlanCommand.WAIT_COUPLE, PlanCommand.DECOUPLE, PlanCommand.REVERSE,
    ):
        # 等待类条目不产生路径（内容是"原地等待"，由事件推进；Q13 / roadmap §2.3）
        return PlanResolution(is_no_path=True)

    if item.command is PlanCommand.GOTO_COUPLE:
        if couple_target is None:
            return PlanResolution(
                failure="连挂条目的目标钩位尚未解析（需要运行时给出 couple_target）"
            )
        goal: Goal | None = couple_target
    else:
        goal = item.goal

    if goal is None:
        return PlanResolution(failure="条目缺少终点")
    start_edge = network.edges.get(start.edge_id)
    if start_edge is None:
        return PlanResolution(failure=f"起点边 {start.edge_id} 不存在")

    goal_directed: DirectedEdge = (goal[0], goal[2])
    end_offset = _end_offset(network, goal)
    start_offset = (
        start.t * start_edge.length
        if start.direction > 0
        else (1.0 - start.t) * start_edge.length
    )
    start_directed: DirectedEdge = (start.edge_id, start.direction)

    # ── 无锚点：直接委托现有入口，保证与右键寻路一字不差 ──────────────
    if not item.anchors:
        result = find_path_from_point(
            network,
            start.edge_id,
            start.t,
            start.direction,
            goal[0],
            goal[1],
            goal[2],
            passable_fn=passable_fn,
            allow_reversal=allow_reversal,
            consist_length=consist_length,
        )
        if result is None:
            return PlanResolution(failure="不可达：起点到终点没有可行路径")
        path, s_off, e_off = result
        return PlanResolution(
            path=ResolvedPath(
                edges=tuple(path.edges),
                total_cost=path.total_cost,
                start_offset=s_off,
                end_offset=e_off,
            )
        )

    # ── 有锚点：逐锚点枚举合法转向对 + DP ───────────────────────────────
    stages: list[list[tuple[DirectedEdge, DirectedEdge]]] = []
    for i, anchor in enumerate(item.anchors):
        pairs = _through_pairs(network, anchor)
        if not pairs:
            return PlanResolution(
                failure=(
                    f"锚点 {i}（节点 {anchor.node_id}）无法通行：该处没有任何合法的"
                    f"「到达边 → 离开边」组合"
                )
            )
        stages.append(pairs)

    # DP 状态：key = 离开边（该锚点走完后的有向边状态），value = (累计代价, 边序列)
    best: dict[DirectedEdge, tuple[float, tuple[DirectedEdge, ...]]] = {}
    for arrive, depart in stages[0]:
        leg = _leg(
            network,
            start_directed,
            arrive,
            passable_fn=passable_fn,
            allow_reversal=allow_reversal,
            consist_length=consist_length,
        )
        if leg is None:
            continue
        previous = best.get(depart)
        if previous is None or leg.total_cost < previous[0]:
            best[depart] = (leg.total_cost, tuple(leg.edges))

    if not best:
        return PlanResolution(
            failure=(
                f"不可达：起点 → 锚点 0（节点 {item.anchors[0].node_id}）没有可行路径"
            )
        )

    for i in range(1, len(stages)):
        next_best: dict[DirectedEdge, tuple[float, tuple[DirectedEdge, ...]]] = {}
        for prev_depart, (cost_so_far, edges_so_far) in best.items():
            for arrive, depart in stages[i]:
                leg = _leg(
                    network,
                    prev_depart,
                    arrive,
                    passable_fn=passable_fn,
                    allow_reversal=allow_reversal,
                    consist_length=consist_length,
                )
                if leg is None:
                    continue
                # 拼接：上一段以"到达边"结束、本段以"离开边"开始，二者必为不同的边
                # （`_through_pairs` 已排除"原路折回"）⇒ 直接首尾相接，无需去重；
                # 每一条边的代价恰好计入一次 ⇒ total_cost = 路径各边弧长之和。
                cost = cost_so_far + leg.total_cost
                joined = edges_so_far + tuple(leg.edges)
                previous = next_best.get(depart)
                if previous is None or cost < previous[0]:
                    next_best[depart] = (cost, joined)
        if not next_best:
            return PlanResolution(
                failure=(
                    f"不可达：锚点 {i - 1}（节点 {item.anchors[i - 1].node_id}）→ "
                    f"锚点 {i}（节点 {item.anchors[i].node_id}）没有可行路径"
                )
            )
        best = next_best

    # ── 最后一段：最后一个锚点 → 终点 ──────────────────────────────────
    final: tuple[float, tuple[DirectedEdge, ...]] | None = None
    for depart, (cost_so_far, edges_so_far) in best.items():
        leg = _leg(
            network,
            depart,
            goal_directed,
            passable_fn=passable_fn,
            allow_reversal=allow_reversal,
            consist_length=consist_length,
        )
        if leg is None:
            continue
        cost = cost_so_far + leg.total_cost
        joined = edges_so_far + tuple(leg.edges)
        if final is None or cost < final[0]:
            final = (cost, joined)
    if final is None:
        last_index = len(item.anchors) - 1
        last = item.anchors[-1]
        return PlanResolution(
            failure=f"不可达：锚点 {last_index}（节点 {last.node_id}）→ 终点没有可行路径"
        )

    total_cost, edges = final
    return PlanResolution(
        path=ResolvedPath(
            edges=edges,
            total_cost=total_cost,
            start_offset=start_offset,
            end_offset=end_offset,
        )
    )


# ── 内部工具 ─────────────────────────────────────────────────────────────

def _leg(
    network: RailNetwork,
    start_directed: DirectedEdge,
    goal_directed: DirectedEdge,
    *,
    passable_fn,
    allow_reversal: bool,
    consist_length: float,
) -> Path | None:
    """一段寻路：从一个有向边状态走到"到达某条有向边"。

    **同边同向的特例**：`start_directed == goal_directed` 时直接返回"就这一条边"。
    本模块里段的终点语义是"**到达该有向边的 head 节点**"（锚点），或"停在末段的
    目标点上"（末段的起点恒为某个锚点的离开边，即 t=0、目标点必在正前方）
    ⇒ 列车**已经在这条边上朝目标走**，这一段不需要任何额外位移。
    ⚠ 不能直接交给 `find_path`：它为了避免"原地不动被当成到达"专门有
    `force_leave` 分支（起点等于目标有向边时必须先离开再绕回来），那会把
    "锚点就在眼前"错判成"必须绕一大圈"甚至判不可达（`tests/test_plan_path.py`
    ③ 正是这个用例：控制点落在起点节点上）。
    """
    if start_directed == goal_directed:
        edge = network.edges.get(start_directed[0])
        if edge is None:
            return None
        return Path(edges=[start_directed], total_cost=edge.length)
    return find_path(
        network,
        start_directed,
        head_node(network, goal_directed),
        passable_fn=passable_fn,
        allow_reversal=allow_reversal,
        goal_directed=goal_directed,
        consist_length=consist_length,
    )


def _arrival_directed(
    network: RailNetwork, edge_id: int, node_id: int
) -> DirectedEdge | None:
    """**到达** node_id 的有向边（head == node_id）。"""
    edge = network.edges.get(edge_id)
    if edge is None or node_id not in (edge.node_a_id, edge.node_b_id):
        return None
    return (edge_id, 1) if edge.node_b_id == node_id else (edge_id, -1)


def _departure_directed(
    network: RailNetwork, edge_id: int, node_id: int
) -> DirectedEdge | None:
    """**离开** node_id 的有向边（tail == node_id）。"""
    edge = network.edges.get(edge_id)
    if edge is None or node_id not in (edge.node_a_id, edge.node_b_id):
        return None
    return (edge_id, 1) if edge.node_a_id == node_id else (edge_id, -1)


def _through_pairs(
    network: RailNetwork, anchor: Anchor
) -> list[tuple[DirectedEdge, DirectedEdge]]:
    """锚点处所有合法的 `(到达边 → 离开边)` 组合。

    控制点（`anchor.exit_edge_id` 给定）只保留"离开边 == 该边"的组合；
    普通锚点保留全部合法转向。空列表 ⇒ 该锚点**无法通行**
    （死端、急折节点，或指定的出口没法从任何方向转进去）。
    """
    node = network.nodes.get(anchor.node_id)
    if node is None:
        return []
    pairs: list[tuple[DirectedEdge, DirectedEdge]] = []
    for arrive_edge_id in sorted(node.incident_edge_ids):
        arrive = _arrival_directed(network, arrive_edge_id, anchor.node_id)
        if arrive is None:
            continue
        for depart_edge_id in sorted(node.incident_edge_ids):
            if depart_edge_id == arrive_edge_id:
                continue  # 原路折回不是合法转向
            if anchor.exit_edge_id is not None and depart_edge_id != anchor.exit_edge_id:
                continue
            if not network.turn_allowed(anchor.node_id, arrive_edge_id, depart_edge_id):
                continue
            depart = _departure_directed(network, depart_edge_id, anchor.node_id)
            if depart is None:
                continue
            pairs.append((arrive, depart))
    return pairs


def _end_offset(network: RailNetwork, goal: Goal) -> float:
    """终点边末尾需截去的弧长（与 `find_path_from_point` 同一公式）。"""
    edge = network.edges.get(goal[0])
    if edge is None:
        return 0.0
    return (1.0 - goal[1]) * edge.length if goal[2] > 0 else goal[1] * edge.length
