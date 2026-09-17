"""P5 最小计划编辑叠加态。"""

from __future__ import annotations

from dataclasses import dataclass, replace

from model.plan import END_REAR, Anchor, Plan, PlanCommand, PlanItem, TrainRef
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
        owner = train.state.consist.control_winner()
        assert owner is not None
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

    def finalize_cycle(self) -> tuple[bool, str]:
        """确认普通计划闭环，或确认含连挂条目的一次性事件链。"""
        if self.owner is None or self.train is None:
            return False, "计划闭环失败：编辑态未开启"
        plan = self.owner.plan
        if plan is None or not plan.items:
            return True, "计划编辑关闭：空计划"
        # 重新确认前先使旧接缝失效；失败时不得继续保留可能已过期的路线。
        plan.loop_route = None
        plan.has_wrapped = False
        if any(item.command is not PlanCommand.GOTO for item in plan.items):
            plan.requires_closed_cycle = False
            plan.repeat = False
            return True, "计划已确认为一次性连挂事件链：执行完毕后停车，不回绕"
        plan.requires_closed_cycle = True
        plan.repeat = True
        first = plan.items[0]
        last = plan.items[-1]
        if first.goal is None or last.goal is None:
            return False, "计划闭环失败：首条或末条缺少固定终点"
        result = resolve_plan_item(
            self.network,
            PathStart(*last.goal),
            PlanItem.goto(first.goal, anchors=first.anchors),
            passable_fn=self.passable_fn,
            allow_reversal=True,
            consist_length=self.train.state.consist.total_length,
        )
        if result.path is None:
            return False, f"计划闭环失败：{result.failure}"
        plan.loop_route = result.path.freeze()
        problems = plan.validate_cycle(self.network)
        if problems:
            plan.loop_route = None
            return False, f"计划闭环失败：{'；'.join(problems)}"
        return True, "计划闭环已冻结：末目标将沿固定路线返回第一目标"

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
            self.owner.plan = Plan(requires_closed_cycle=True)
        self.owner.plan.append(item)
        count = len(self.owner.plan)
        self.draft = PlanDraft()
        return True, f"计划已冻结并追加为第 {count} 条"

    def append_wait_couple(self) -> tuple[bool, str]:
        if self.owner is None:
            return False, "计划编辑错误：编辑态未开启"
        if self.owner.plan is None:
            self.owner.plan = Plan(requires_closed_cycle=True)
        self.owner.plan.repeat = False
        self.owner.plan.requires_closed_cycle = False
        self.owner.plan.append(PlanItem.wait_couple())
        self.draft = PlanDraft()
        return True, f"计划已追加等待连挂为第 {len(self.owner.plan)} 条"

    def append_goto_couple(
        self, target_train: TrainEntity, target_end: str,
    ) -> tuple[bool, str]:
        if self.owner is None or self.train is None:
            return False, "计划编辑错误：编辑态未开启"
        if target_train is self.train:
            return False, "计划编辑错误：连挂目标不能是本编组"
        if target_end != "tail":
            return False, "计划编辑错误：当前版本只支持驶向目标车尾"
        if not target_train.is_parked():
            return False, "计划编辑错误：连挂目标列车必须停稳"

        from controller.coupling import end_coupler_pos
        _position, edge_id, t = end_coupler_pos(target_train, "tail")
        target_wagon = target_train.state.consist.wagons[-1]
        item = PlanItem.goto_couple(
            TrainRef(target_wagon.wagon_id, END_REAR),
            anchors=self.draft.anchors,
        )
        result = resolve_plan_item(
            self.network,
            self._construction_start(),
            item,
            passable_fn=self.passable_fn,
            allow_reversal=True,
            consist_length=self.train.state.consist.total_length,
            couple_target=(edge_id, t, target_train.current_direction()),
        )
        if result.path is None:
            self.draft = PlanDraft(self.draft.anchors, resolution=result)
            return False, f"计划解析失败：{result.failure}"
        frozen = PlanItem.goto_couple(
            item.train_ref,
            anchors=item.anchors,
            fixed_route=result.path.freeze(),
        )
        if self.owner.plan is None:
            self.owner.plan = Plan(requires_closed_cycle=True)
        self.owner.plan.repeat = False
        self.owner.plan.requires_closed_cycle = False
        self.owner.plan.append(frozen)
        self.draft = PlanDraft()
        return True, f"计划已冻结前往连挂并追加为第 {len(self.owner.plan)} 条"

    def _construction_start(self) -> PathStart:
        assert self.train is not None
        if self.owner is not None and self.owner.plan is not None and self.owner.plan.items:
            previous = self.owner.plan.items[-1]
            if previous.goal is not None:
                return PathStart(*previous.goal)
            if previous.fixed_route is not None:
                edge_id, direction = previous.fixed_route.edges[-1]
                edge_length = self.network.edges[edge_id].length
                t = (1.0 - previous.fixed_route.end_offset / edge_length
                     if direction > 0 else previous.fixed_route.end_offset / edge_length)
                return PathStart(edge_id, t, direction)
        edge_id, t = self.train.current_edge_and_t()
        return PathStart(edge_id, t, self.train.current_direction())
