"""P5 最小计划编辑叠加态。"""

from __future__ import annotations

from dataclasses import dataclass, replace

from model.plan import Anchor, Plan, PlanItem
from model.plan_path import PathStart, PlanResolution, resolve_plan_item
from model.rail_network import RailNetwork
from model.train_entity import TrainEntity
from model.wagon import Wagon


@dataclass
class PlanDraft:
    anchors: tuple[Anchor, ...] = ()
    goal: tuple[int, float, int] | None = None
    resolution: PlanResolution | None = None

    @property
    def failure(self) -> str:
        return self.resolution.failure if self.resolution is not None else ""


class PlanEditor:
    """把点击序列转换成冻结 ``goto`` 条目；不接触运行期调度器。"""

    def __init__(self, network: RailNetwork, passable_fn=None) -> None:
        self.network = network
        self.passable_fn = passable_fn
        self.active = False
        self.train: TrainEntity | None = None
        self.owner: Wagon | None = None
        self.draft = PlanDraft()

    def enter(self, train: TrainEntity) -> tuple[bool, str]:
        if not train.is_parked():
            return False, "计划编辑拒绝：列车必须完全停放"
        controls = train.state.consist.control_cars()
        if not controls:
            return False, "计划编辑拒绝：编组没有控制车"
        owner = min(controls, key=lambda wagon: (-wagon.priority, wagon.wagon_id))
        self.active = True
        self.train = train
        self.owner = owner
        self.draft = PlanDraft()
        return True, f"计划编辑开启：控制车 {owner.wagon_id[:8]}"

    def cancel(self) -> None:
        self.active = False
        self.train = None
        self.owner = None
        self.draft = PlanDraft()

    def click_node(self, node_id: int) -> str:
        if not self.active or node_id not in self.network.nodes:
            return "计划编辑错误：节点不存在或编辑态未开启"
        self.draft = PlanDraft(anchors=self.draft.anchors + (Anchor(node_id),))
        return f"计划锚点 {len(self.draft.anchors)}：node {node_id}"

    def click_edge(self, edge_id: int, t: float) -> str:
        if not self.active or self.train is None:
            return "计划编辑错误：编辑态未开启"
        edge = self.network.edges.get(edge_id)
        if edge is None:
            return f"计划编辑错误：edge {edge_id} 不存在"

        if self.draft.anchors:
            last = self.draft.anchors[-1]
            node = self.network.nodes.get(last.node_id)
            if (last.exit_edge_id is None and node is not None
                    and node.connection_count() >= 3 and edge_id in node.incident_edge_ids):
                anchors = self.draft.anchors[:-1] + (replace(last, exit_edge_id=edge_id),)
                self.draft = PlanDraft(anchors=anchors)
                return f"计划控制点：node {last.node_id} 从 edge {edge_id} 离开"

        best: PlanResolution | None = None
        best_goal = None
        start = self._construction_start()
        for direction in (1, -1):
            goal = (edge_id, max(0.0, min(1.0, t)), direction)
            item = PlanItem.goto(goal, anchors=self.draft.anchors)
            result = resolve_plan_item(
                self.network,
                start,
                item,
                passable_fn=self.passable_fn,
                allow_reversal=True,
                consist_length=self.train.state.consist.total_length,
            )
            if result.ok and (best is None or not best.ok
                              or result.path.remaining_to_goal < best.path.remaining_to_goal):
                best, best_goal = result, goal
            elif best is None:
                best = result
        self.draft = PlanDraft(self.draft.anchors, best_goal, best)
        if best is None or not best.ok:
            reason = best.failure if best is not None else "没有候选方向"
            return f"计划解析失败：{reason}"
        return f"计划终点：edge {edge_id} t={t:.2f}，Enter 确认"

    def backspace(self) -> str:
        if self.draft.goal is not None:
            self.draft = PlanDraft(anchors=self.draft.anchors)
            return "已撤销计划终点"
        if self.draft.anchors:
            self.draft = PlanDraft(anchors=self.draft.anchors[:-1])
            return "已撤销最后一个锚点"
        return "计划草稿已为空"

    def confirm(self) -> tuple[bool, str]:
        result = self.draft.resolution
        if self.owner is None or self.draft.goal is None or result is None or result.path is None:
            reason = self.draft.failure or "尚未设置可达终点"
            return False, f"计划确认失败：{reason}"
        item = PlanItem.goto(
            self.draft.goal,
            anchors=self.draft.anchors,
            fixed_route=result.path.freeze(),
        )
        if self.owner.plan is None:
            self.owner.plan = Plan()
        self.owner.plan.append(item)
        count = len(self.owner.plan)
        self.draft = PlanDraft()
        return True, f"计划已冻结并追加为第 {count} 条"

    def _construction_start(self) -> PathStart:
        assert self.train is not None
        if self.owner is not None and self.owner.plan is not None and self.owner.plan.items:
            previous = self.owner.plan.items[-1]
            if previous.goal is not None:
                return PathStart(*previous.goal)
        edge_id, t = self.train.current_edge_and_t()
        return PathStart(edge_id, t, self.train.current_direction())
