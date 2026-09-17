"""P4 路网拓扑变更的纯模型协调工具。

RailNetwork 只维护几何，不知道列车、信号或计划。这里集中定义"哪些引用必须保护"和
"切边后如何机械改写"，由 GameLoop 在真正调用 split_edge_at 前后组成一次操作；
Editor 因而不依赖计划/列车/信号。
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Iterable

from model.plan import FixedRoute, Plan, PlanItem


DirectedEdge = tuple[int, int]


@dataclass(frozen=True)
class EdgeSplit:
    """一次已提交切边的稳定描述。"""

    old_edge_id: int
    t: float
    old_node_a_id: int
    old_node_b_id: int
    first_edge_id: int       # 原 node_a → 新节点
    second_edge_id: int      # 新节点 → 原 node_b

    def replace_directed(self, directed: DirectedEdge) -> tuple[DirectedEdge, ...]:
        edge_id, direction = directed
        if edge_id != self.old_edge_id:
            return (directed,)
        if direction > 0:
            return ((self.first_edge_id, 1), (self.second_edge_id, 1))
        return ((self.second_edge_id, -1), (self.first_edge_id, -1))

    def rewrite_fixed_route(self, route: FixedRoute) -> FixedRoute:
        edges = tuple(
            child
            for directed in route.edges
            for child in self.replace_directed(directed)
        )
        return replace(route, edges=edges)

    def rewrite_plan_item(self, item: PlanItem) -> PlanItem:
        """返回改写后的不可变条目；没有命中时原对象原样返回。"""
        fixed_route = item.fixed_route
        goal = item.goal
        anchors = item.anchors
        changed = False

        if fixed_route is not None and self.old_edge_id in fixed_route.used_edge_ids():
            fixed_route = self.rewrite_fixed_route(fixed_route)
            changed = True

        if goal is not None and goal[0] == self.old_edge_id:
            _edge_id, old_t, direction = goal
            if old_t < self.t:
                goal = (self.first_edge_id, old_t / self.t, direction)
            else:
                goal = (self.second_edge_id, (old_t - self.t) / (1.0 - self.t), direction)
            changed = True

        rewritten_anchors = tuple(
            replace(
                anchor,
                exit_edge_id=(
                    self.first_edge_id if anchor.node_id == self.old_node_a_id
                    else self.second_edge_id if anchor.node_id == self.old_node_b_id
                    else anchor.exit_edge_id
                ),
            ) if anchor.exit_edge_id == self.old_edge_id else anchor
            for anchor in anchors
        )
        changed = changed or rewritten_anchors != anchors

        return item if not changed else replace(
            item, fixed_route=fixed_route, goal=goal, anchors=rewritten_anchors,
        )


def fixed_routes(items: Iterable[PlanItem]) -> tuple[FixedRoute, ...]:
    return tuple(item.fixed_route for item in items if item.fixed_route is not None)


def routes_conflict_with_signal(items: Iterable[PlanItem], signal: DirectedEdge) -> bool:
    return any(route.conflicts_with_signal(signal) for route in fixed_routes(items))


def plans_conflict_with_signal(plans: Iterable[Plan], signal: DirectedEdge) -> bool:
    """计划条目及计划级循环接缝是否会被该信号从背面封死。"""
    for plan in plans:
        if routes_conflict_with_signal(plan.items, signal):
            return True
        if plan.loop_route is not None and plan.loop_route.conflicts_with_signal(signal):
            return True
    return False
