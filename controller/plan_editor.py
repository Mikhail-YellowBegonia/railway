"""P5 最小计划编辑叠加态。"""

from __future__ import annotations

from dataclasses import dataclass, replace

from model.plan import Anchor, Plan, PlanCommand, PlanItem
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
        """确认纯行车计划闭环，或启用可重复的固定场景调车计划。"""
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
            plan.repeat = True
            return True, "固定场景调车计划已确认为循环执行"
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

    def remove_current(self) -> tuple[bool, str]:
        """删除当前条目；仅编辑态调用，退出时由既有确认流程重建闭环。"""
        if self.owner is None or self.owner.plan is None or not self.owner.plan.items:
            return False, "计划编辑错误：没有可删除的计划条目"
        plan = self.owner.plan
        index = plan.pointer if plan.pointer < len(plan.items) else len(plan.items) - 1
        removed = plan.remove_at(index)
        self.draft = PlanDraft()
        return True, f"已删除第 {index + 1} 条计划：{removed.label}"

    def clear_plan(self) -> tuple[bool, str]:
        """清空胜出控制车的计划；只清计划，不改变车厢控制权或运行状态。"""
        if self.owner is None:
            return False, "计划编辑错误：编辑态未开启"
        self.owner.plan = None
        self.draft = PlanDraft()
        return True, "已清空当前控制车的计划"

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
        self.owner.plan.requires_closed_cycle = False
        self.owner.plan.append(PlanItem.wait_couple())
        self.draft = PlanDraft()
        return True, f"计划已追加等待连挂为第 {len(self.owner.plan)} 条"

    def append_goto_couple(
        self, target_train: TrainEntity, target_end: str,
    ) -> tuple[bool, str]:
        if self.owner is None or self.train is None:
            return False, "计划编辑错误：编辑态未开启"
        if not target_train.is_parked():
            return False, "计划编辑错误：连挂目标列车必须停稳"

        from controller.coupling import end_coupler_pos
        end = "head" if target_end == "head" else "tail"
        _position, edge_id, t = end_coupler_pos(target_train, end)
        return self.append_goto_couple_edge(edge_id, target_t=t)

    def append_goto_couple_edge(
        self, edge_id: int, *, target_t: float = 0.5,
        direction: int | None = None,
    ) -> tuple[bool, str]:
        """按 edge 创建声明式连挂入口。

        计划编辑时不要求 edge 上当前已经存在一列“外部目标车”。这使得
        headshunt 可以在目标仍属于本编组、端头仍受 PLAY 手动连挂保护时，
        预先写下未来的 ``CoupleSelector(edge, direction)``。

        ``target_t`` 只用于编辑期冻结接近路线；运行期候选端头会在同一 edge
        上重新解析。未指定 direction 时尝试两个方向并选择可达且代价较低者。
        """
        if self.owner is None or self.train is None:
            return False, "计划编辑错误：编辑态未开启"
        edge = self.network.edges.get(edge_id)
        if edge is None:
            return False, f"计划编辑错误：edge {edge_id} 不存在"
        target_t = max(0.0, min(1.0, float(target_t)))
        directions = (direction,) if direction in (1, -1) else (1, -1)
        result = None
        selected_direction = None
        for target_direction in directions:
            item = PlanItem.goto_couple(
                anchors=self.draft.anchors,
                edge_id=edge_id,
                direction=target_direction,
            )
            candidate = resolve_plan_item(
                self.network,
                self._construction_start(),
                item,
                passable_fn=self.passable_fn,
                # P7c 调车计划必须用显式 reverse 条目管理每次折返；selector
                # 接近路线不得偷偷依赖寻路器的死端自动换向。
                allow_reversal=False,
                consist_length=self.train.state.consist.total_length,
                couple_target=(edge_id, target_t, target_direction),
            )
            if (candidate.path is not None
                    and (result is None or result.path is None
                         or candidate.path.remaining_to_goal < result.path.remaining_to_goal)):
                result = candidate
                selected_direction = target_direction
            elif result is None:
                result = candidate
        assert result is not None
        if result.path is None:
            self.draft = PlanDraft(self.draft.anchors, resolution=result)
            return False, f"计划解析失败：{result.failure}"
        frozen = PlanItem.goto_couple(
            anchors=self.draft.anchors,
            fixed_route=result.path.freeze(),
            edge_id=edge_id,
            direction=(selected_direction
                       if selected_direction in (1, -1)
                       else result.path.edges[-1][1]),
        )
        if self.owner.plan is None:
            self.owner.plan = Plan(requires_closed_cycle=True)
        self.owner.plan.requires_closed_cycle = False
        self.owner.plan.append(frozen)
        self.draft = PlanDraft()
        return True, (
            f"计划已冻结驶入 edge {edge_id} dir {frozen.couple_selector.direction:+d} 的"
            "声明式连挂 selector，"
            f"并追加为第 {len(self.owner.plan)} 条"
        )

    def append_goto_couple_poi(
        self, poi_id: str, *, direction: int | None = None,
    ) -> tuple[bool, str]:
        """Append declarative POI intent without freezing a unique edge/path."""
        if self.owner is None or self.train is None:
            return False, "计划编辑错误：编辑态未开启"
        item = PlanItem.goto_couple(
            anchors=self.draft.anchors,
            poi_id=poi_id,
            direction=direction,
        )
        problems = item.validate(self.network, require_fixed_route=True)
        if problems:
            return False, f"计划编辑错误：{'；'.join(problems)}"
        if self.owner.plan is None:
            self.owner.plan = Plan(requires_closed_cycle=True)
        self.owner.plan.requires_closed_cycle = False
        self.owner.plan.append(item)
        self.draft = PlanDraft()
        return True, (
            f"计划已追加声明式 POI 连挂 selector {poi_id[:8]}，"
            "将在激活或目标失效时解析当前车钩与路径"
        )

    def append_decouple(self, after: int) -> tuple[bool, str]:
        if self.owner is None or self.train is None:
            return False, "计划编辑错误：编辑态未开启"
        if not (1 <= after < len(self.train.state.consist.wagons)):
            return False, f"计划编辑错误：无法从车头后第 {after} 位解挂"
        if self.owner.plan is None:
            self.owner.plan = Plan(requires_closed_cycle=False)
        self.owner.plan.requires_closed_cycle = False
        self.owner.plan.append(PlanItem.decouple(after))
        return True, f"计划已追加从车头后第 {after} 位解挂"

    def append_reverse(self) -> tuple[bool, str]:
        if self.owner is None:
            return False, "计划编辑错误：编辑态未开启"
        if self.owner.plan is None:
            self.owner.plan = Plan(requires_closed_cycle=False)
        self.owner.plan.requires_closed_cycle = False
        self.owner.plan.append(PlanItem.reverse())
        return True, f"计划已追加折返为第 {len(self.owner.plan)} 条"

    def _construction_start(self) -> PathStart:
        assert self.train is not None
        edge_id, t = self.train.current_edge_and_t()
        start = PathStart(edge_id, t, self.train.current_direction())
        if self.owner is None or self.owner.plan is None:
            return start
        for item in self.owner.plan.items:
            if item.goal is not None:
                start = PathStart(*item.goal)
            elif item.fixed_route is not None:
                route_edge_id, direction = item.fixed_route.edges[-1]
                edge_length = self.network.edges[route_edge_id].length
                route_t = (1.0 - item.fixed_route.end_offset / edge_length
                           if direction > 0 else item.fixed_route.end_offset / edge_length)
                start = PathStart(route_edge_id, route_t, direction)
            if item.command is PlanCommand.REVERSE:
                start = PathStart(start.edge_id, start.t, -start.direction)
        return start
