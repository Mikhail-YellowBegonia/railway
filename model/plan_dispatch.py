"""将控制车计划解析/投影为既有列车运行指令。

计划条目保存长期意图；每次激活或运行租约失效时，调度器都从当前车头和当前
目标重新调用路径解析器，生成只存在于运行期的解析快照。快照最终只向列车
下达一次 ``assign_route``，之后仍由既有闭塞调度器推进、等待和恢复。
"""

from __future__ import annotations

from dataclasses import dataclass

from model.plan import (
    END_FRONT, END_REAR, FixedRoute, Goal, PathPolicy, Plan, PlanCommand, PlanItem,
)
from model.plan_path import PathStart, resolve_plan_item
from model.rail_network import RailNetwork
from model.train_controller import BrakingController
from model.train_entity import TrainEntity
from model.train_control import TrainControlAuthority
from model.wagon import Wagon


PLAN_CRUISE_SPEED = 12.0
POSITION_EPSILON_M = 0.1


@dataclass
class PlanExecution:
    """编组运行期派生的计划激活令牌，不属于车厢域数据。"""

    wagon_id: str
    plan_id: int
    item_id: int
    pointer: int
    route_signature: tuple
    projected_goal: Goal | None
    target_wagon_id: str | None = None
    target_end: int | None = None
    # Runtime resolution snapshot. For declarative selectors this is rebuilt
    # when the current target/path becomes invalid; it is never plan data.
    resolved_route: FixedRoute | None = None


@dataclass(frozen=True)
class DecoupleAction:
    train: TrainEntity
    winner: Wagon
    plan: Plan
    item: PlanItem
    after: int


class PlanDispatcher:
    """控制车遴选、动态/固定路径生产、路线投影和到达事件步进。"""

    def __init__(
        self, network: RailNetwork, pois=None, stations=None, passable_fn=None,
    ) -> None:
        self.network = network
        self.pois = pois
        self.stations = stations
        self.passable_fn = passable_fn
        self._decouple_actions: list[DecoupleAction] = []
        self._pending_resolved_route: FixedRoute | None = None

    def take_decouple_actions(self) -> list[DecoupleAction]:
        actions, self._decouple_actions = self._decouple_actions, []
        return actions

    def set_paused(self, train: TrainEntity, paused: bool) -> None:
        """暂停/继续该列车的计划自动驾驶，不移动计划指针。"""
        train.plan_paused = paused
        self._decouple_actions = [
            action for action in self._decouple_actions if action.train is not train
        ]
        self._clear_execution(train)
        if paused:
            train.emergency_stop()
            train.control_authority = TrainControlAuthority.PLAN_PAUSED
            train.plan_status = "计划已暂停（空格继续）"
        else:
            train.control_authority = TrainControlAuthority.NONE
            train.plan_status = ""

    def tick(self, train: TrainEntity, trains: list[TrainEntity] | None = None) -> None:
        """在既有信号调度前运行一次；只在首次激活时写入列车指令。"""
        incoming_claim = self._incoming_claim(train, trains)
        if incoming_claim is not None:
            if not train.is_parked():
                train.emergency_stop()
            self._clear_execution(train)
            wagon_id, target_end = incoming_claim
            end_label = "前端" if target_end == END_FRONT else "后端"
            self._report_once(
                train, f"连挂认领：车厢 {wagon_id[:8]} {end_label} 已锁定，目标列车保持停放",
            )
            return
        if getattr(train, "plan_paused", False):
            if not train.is_parked():
                train.emergency_stop()
            self._clear_execution(train)
            train.control_authority = TrainControlAuthority.PLAN_PAUSED
            self._report_once(train, "计划已暂停（空格继续）")
            return
        winner = self._winner(train)
        execution = getattr(train, "plan_execution", None)

        if winner is None:
            if execution is not None:
                self._clear_execution(train)
            return
        if winner.plan is None:
            self._clear_execution(train)
            if any(wagon.plan is not None for wagon in train.state.consist.control_cars()):
                self._report_once(
                    train,
                    f"计划错误：胜出控制车 {winner.wagon_id[:8]} 没有计划，列车不执行",
                )
            return

        plan = winner.plan
        cycle_problems = plan.validate_cycle(self.network)
        if cycle_problems:
            if not train.is_parked():
                train.emergency_stop()
            self._clear_execution(train)
            self._report_once(train, f"计划不可执行：{'；'.join(cycle_problems)}")
            return
        item = plan.current()
        if item is None:
            if not train.is_parked():
                train.emergency_stop()
            self._clear_execution(train)
            if plan.is_complete:
                self._report_once(train, "计划已完成：一次性事件链已消费完毕，列车停车")
            return

        if execution is not None and self._same_activation(execution, winner, plan, item):
            if item.command is PlanCommand.GOTO_COUPLE:
                outcome = self._couple_goal(train, item, trains, execution=execution)
                expected_goal = outcome[3]
                target_id = outcome[4]
                target_end = outcome[5]
                if (outcome[0] != "ready" or expected_goal != execution.projected_goal
                        or target_id != execution.target_wagon_id
                        or target_end != execution.target_end):
                    train.emergency_stop()
                    self._clear_execution(train)
                    if (item.couple_selector is not None
                            and item.couple_selector.is_declarative):
                        self._report_once(train, f"计划等待：{outcome[1] or '声明式目标需要刷新'}")
                        return
                    plan.advance()
                    reason = outcome[1] or "锁定连挂目标失效"
                    self._report_once(train, f"计划跳过：{reason}")
                    return
                if self._pending_resolved_route is not None:
                    refreshed = self._pending_resolved_route
                    refreshed_signature = self._route_signature(refreshed, expected_goal)
                    if refreshed_signature != execution.route_signature:
                        from controller.coupling import head_hook_offset
                        goal_edge_id, goal_t, goal_direction = expected_goal
                        goal_edge_length = self.network.edges[goal_edge_id].length
                        runtime_end_offset = goal_edge_length * (
                            (1.0 - goal_t) if goal_direction > 0 else goal_t
                        )
                        projection = self._project_route_start(
                            train, refreshed,
                            end_offset=runtime_end_offset + head_hook_offset(train),
                        )
                        if projection is None:
                            self._clear_execution(train)
                            self._report_once(
                                train, "计划等待：当前车头无法投影到动态连挂路径",
                            )
                            return
                        route, remaining = projection
                        train._stop_before_m = head_hook_offset(train)
                        train.assign_route(route, remaining, expected_goal)
                        train.couple_approach_partner = outcome[2]
                        execution.resolved_route = refreshed
                        execution.projected_goal = expected_goal
                        execution.route_signature = refreshed_signature
            # 普通右键/K 调车允许覆盖计划投影出的运行指令。目标一旦不同，旧令牌
            # 立即失效；不在行驶中抢回 route，待人工指令结束后再按起点契约判定。
            if train.state.goal != execution.projected_goal:
                self._clear_execution(train)
                self._report_once(train, "计划被临时行车指令覆盖，等待重新对齐")
                return
            # 计划自动驾驶拥有本条指令期间的巡航速度。GUI 的手动巡航值可能仍为
            # 0，若这里只在首次激活时设速，下一帧会被写回 0 并在目标前停成
            # parked；旧逻辑随后因激活令牌仍有效而永久静默等待。
            if item.command in (PlanCommand.GOTO, PlanCommand.GOTO_COUPLE):
                train.v_target = max(train.v_target, PLAN_CRUISE_SPEED)
            # 新激活的极短路线可能在首次物理帧内就完成，因而不在 GameLoop
            # 的 moving_before 集合里；这里补偿该停车事件，仍只推进一次。
            if train.is_parked() and item.command is PlanCommand.GOTO:
                self._clear_execution(train)
                if self._at_goal(train, item):
                    plan.advance()
                    return
                # 未到目标却丢失 route/controller：从当前车头重新消费同一冻结
                # 路线后缀。这里只做投影，不调用 P2/Dijkstra；失败由 _activate
                # 的起点契约明确报告，不能继续保持无原因的 parked 状态。
                self._activate(train, winner, plan, trains)
                return
            active_route = execution.resolved_route
            execution.route_signature = self._route_signature(
                active_route, execution.projected_goal,
            )
            return

        self._clear_execution(train)
        self._activate(train, winner, plan, trains)

    def on_arrival(self, train: TrainEntity) -> None:
        """仅由真正到达冻结目标的停车事件推进一次 ``goto`` 指针。"""
        execution = getattr(train, "plan_execution", None)
        if execution is None:
            return
        winner = self._winner(train)
        if winner is None or winner.plan is None:
            self._clear_execution(train)
            return
        plan = winner.plan
        item = plan.current()
        if (not self._same_activation(execution, winner, plan, item)
                or item is None or item.command is not PlanCommand.GOTO
                or not self._at_goal(train, item)):
            return
        plan.advance()
        self._clear_execution(train)

    def on_coupled(
        self,
        new_train: TrainEntity,
        participant_wagon_groups: tuple[set[str], set[str]],
        expected_target_wagon_id: str | None = None,
    ) -> None:
        """合并实体产生后，只推进新控制车当前的连挂事件条目。"""
        winner = self._winner(new_train)
        self._clear_execution(new_train)
        if winner is None or winner.plan is None:
            return
        item = winner.plan.current()
        if item is None:
            return
        if item.command is PlanCommand.GOTO_COUPLE:
            winner_group = next(
                (group for group in participant_wagon_groups if winner.wagon_id in group),
                set(),
            )
            # 固定-edge 命令不永久引用某节车厢；连挂事件只需来自合并前另一编组。
            target_group = next(
                (group for group in participant_wagon_groups if group is not winner_group), set(),
            )
            legacy_target_mismatch = (
                item.train_ref is not None
                and item.train_ref.wagon_id not in target_group
            )
            fixed_edge_target_mismatch = (
                item.couple_selector is not None
                and expected_target_wagon_id is not None
                and expected_target_wagon_id not in target_group
            )
            if not target_group or legacy_target_mismatch or fixed_edge_target_mismatch:
                self._report_once(new_train, "计划错误：连挂事件与当前目标不匹配，未推进")
                return
            winner.plan.advance()
            new_train.plan_status = ""
        elif item.command is PlanCommand.WAIT_COUPLE:
            winner.plan.advance()
            new_train.plan_status = ""

    @staticmethod
    def _winner(train: TrainEntity) -> Wagon | None:
        """在全部控制车中稳定遴选；胜出者无计划时不得降级执行次优者。"""
        return train.state.consist.control_winner()

    def _activate(
        self,
        train: TrainEntity,
        winner: Wagon,
        plan: Plan,
        trains: list[TrainEntity] | None,
    ) -> None:
        # 永久失效按 Q23-2 跳过；每次 tick 至多环扫一圈，杜绝全坏计划死循环。
        for _ in range(len(plan)):
            item = plan.current()
            if item is None:
                return
            # 新计划默认动态解析；旧条目若携带 FixedRoute 仍走兼容分支。
            problems = item.validate(self.network)
            if problems:
                self._report_once(train, f"计划跳过：{'；'.join(problems)}")
                plan.advance()
                continue
            if item.command is PlanCommand.WAIT_COUPLE:
                if not train.is_parked():
                    train.emergency_stop()
                train.couple_approach_partner = None
                self._report_once(train, "计划等待连挂")
                return
            if item.command is PlanCommand.REVERSE:
                if not train.is_parked():
                    self._report_once(train, "计划等待当前运行指令结束")
                    return
                reverse_ok = train.reverse_in_place()
                if not reverse_ok:
                    self._report_once(train, "计划等待：当前车身不能在同一 simple_segment 内折返")
                    return
                plan.advance()
                self._clear_execution(train)
                train.plan_status = ""
                return
            if item.command is PlanCommand.DECOUPLE:
                if not train.is_parked():
                    self._report_once(train, "计划等待当前运行指令结束")
                    return
                after = item.decouple_after or 0
                if after >= len(train.state.consist.wagons):
                    self._report_once(
                        train, f"计划等待：当前编组不足以从车头后第 {after} 位解挂",
                    )
                    return
                train.plan_execution = PlanExecution(
                    wagon_id=winner.wagon_id,
                    plan_id=id(plan), item_id=id(item), pointer=plan.pointer,
                    route_signature=(), projected_goal=None,
                )
                action = DecoupleAction(train, winner, plan, item, after)
                if action not in self._decouple_actions:
                    self._decouple_actions.append(action)
                return
            if item.command is PlanCommand.GOTO_COUPLE:
                self._pending_resolved_route = None
                outcome = self._couple_goal(train, item, trains)
                if outcome[0] != "ready":
                    if not train.is_parked():
                        train.emergency_stop()
                    if (item.couple_selector is not None
                            and item.couple_selector.is_declarative):
                        # Declarative selectors wait for a resolver/refresh pass;
                        # they are not permanently invalid merely because no
                        # current concrete candidate has been materialized yet.
                        self._report_once(train, f"计划等待：{outcome[1]}")
                        return
                    self._report_once(train, f"计划跳过：{outcome[1]}")
                    plan.advance()
                    continue
                target_train, goal, target_id, target_end = (
                    outcome[2], outcome[3], outcome[4], outcome[5]
                )
                if not train.is_parked():
                    self._report_once(train, "计划等待当前运行指令结束")
                    return
                active_route = (
                    item.fixed_route or self._pending_resolved_route
                    if item.effective_path_policy is PathPolicy.FIXED
                    else self._pending_resolved_route
                )
                if active_route is None:
                    self._report_once(train, "计划错误：当前连挂无法生成动态路径")
                    return
                from controller.coupling import head_hook_offset
                stop_before = head_hook_offset(train)
                goal_edge_id, goal_t, goal_direction = goal
                goal_edge_length = self.network.edges[goal_edge_id].length
                runtime_end_offset = (
                    goal_edge_length * ((1.0 - goal_t) if goal_direction > 0 else goal_t)
                )
                projection = self._project_route_start(
                    train,
                    active_route,
                    end_offset=runtime_end_offset + stop_before,
                )
                if projection is None:
                    self._report_once(train, self._route_start_error(train, active_route))
                    return
                route, remaining = projection
                train._stop_before_m = stop_before
                train.assign_route(route, remaining, goal)
                train.couple_approach_partner = target_train
                train.v_target = max(train.v_target, PLAN_CRUISE_SPEED)
                train.plan_execution = PlanExecution(
                    wagon_id=winner.wagon_id,
                    plan_id=id(plan),
                    item_id=id(item),
                    pointer=plan.pointer,
                    route_signature=self._route_signature(active_route, goal),
                    projected_goal=goal,
                    target_wagon_id=target_id,
                    target_end=target_end,
                    resolved_route=(active_route if item.effective_path_policy is PathPolicy.DYNAMIC else None),
                )
                train.plan_status = ""
                return
            if item.command is not PlanCommand.GOTO or item.goal is None:
                self._report_once(train, "计划条目不可执行")
                return
            if not train.is_parked():
                self._report_once(train, "计划等待当前运行指令结束")
                return
            active_route = (
                self._active_route(plan, item)
                if item.effective_path_policy is PathPolicy.FIXED
                else self._resolve_dynamic_goto(
                    train, item,
                    allow_reversal=not self._previous_item_is_reverse(plan),
                )
            )
            if active_route is None:
                self._report_once(
                    train,
                    "计划错误：当前条目无法生成可行路径",
                )
                return
            route_problems = active_route.validate(self.network, goal=item.goal)
            if route_problems:
                self._report_once(
                    train, f"计划错误：循环接缝失效：{'；'.join(route_problems)}",
                )
                return
            projection = self._project_route_start(train, active_route)
            if projection is None:
                self._report_once(train, self._route_start_error(train, active_route))
                return
            route, remaining = projection
            train._stop_before_m = 0.0
            train.assign_route(route, remaining, item.goal)
            train.v_target = max(train.v_target, PLAN_CRUISE_SPEED)
            train.plan_execution = PlanExecution(
                wagon_id=winner.wagon_id,
                plan_id=id(plan),
                item_id=id(item),
                pointer=plan.pointer,
                route_signature=self._route_signature(active_route, item.goal),
                projected_goal=item.goal,
                resolved_route=(active_route if item.effective_path_policy is PathPolicy.DYNAMIC else None),
            )
            train.control_authority = TrainControlAuthority.PLAN
            train.plan_status = ""
            return
        self._clear_execution(train)
        last_error = getattr(train, "plan_status", "")
        suffix = f"；最后错误：{last_error}" if last_error else ""
        self._report_once(train, f"计划整圈均执行失败，列车停车等待{suffix}")

    def _resolve_dynamic_goto(
        self,
        train: TrainEntity,
        item: PlanItem,
        *,
        allow_reversal: bool = True,
    ) -> FixedRoute | None:
        """按当前车头位置生成一次动态路径快照，不写回计划。"""
        if item.goal is None:
            return None
        start_edge, start_t, start_direction = train.current_directed_edge_and_t()
        resolution = resolve_plan_item(
            self.network,
            PathStart(start_edge, start_t, start_direction),
            item,
            passable_fn=self.passable_fn,
            # A scheduled reverse is an explicit plan item.  In that case do
            # not let the pathfinder silently add an ``edge,+/-`` reversal
            # marker: the physical layer consumes it by reversing the whole
            # consist, so a second implicit marker can make the same edge
            # appear to be traversed twice.  Keep legacy implicit reversal for
            # goto items without an explicit preceding reverse.
            allow_reversal=allow_reversal,
            consist_length=train.state.consist.total_length,
        )
        return resolution.path.freeze() if resolution.path is not None else None

    @staticmethod
    def _previous_item_is_reverse(plan: Plan) -> bool:
        """Whether the current item follows an explicit plan reversal."""
        if not plan.items:
            return False
        index = plan.pointer - 1
        if index < 0:
            if not plan.repeat:
                return False
            index = len(plan.items) - 1
        return plan.items[index].command is PlanCommand.REVERSE

    def _couple_goal(
        self,
        train: TrainEntity,
        item: PlanItem,
        trains: list[TrainEntity] | None,
        *,
        execution: PlanExecution | None = None,
    ) -> tuple[str, str, TrainEntity | None, Goal | None, str | None, int | None]:
        ref = item.train_ref
        if trains is None:
            return "temporary", "当前列车表不可用，无法解析连挂目标", None, None, None, None
        if item.couple_selector is not None:
            if execution is not None:
                return self._locked_couple_goal(train, item, trains, execution)
            if item.couple_selector.is_declarative:
                return self._couple_goal_in_scope(train, item, trains)
            return self._couple_goal_on_edge(train, item, trains)
        if ref is None:
            return "permanent", "前往连挂缺少目标", None, None, None, None
        target_train = next(
            (candidate for candidate in trains
             if any(wagon.wagon_id == ref.wagon_id
                    for wagon in candidate.state.consist.wagons)),
            None,
        )
        if target_train is None:
            return "permanent", f"连挂目标车厢 {ref.wagon_id[:8]} 已不存在", None, None, None, None
        if target_train is train:
            return "temporary", "连挂目标已在本编组内", None, None, None, None
        if ref.end == END_FRONT:
            return "temporary", "旧式车厢目标不支持前端", None, None, None, None
        if ref.end != END_REAR:
            return "permanent", f"连挂目标端头非法：{ref.end}", None, None, None, None
        if target_train.state.consist.wagons[-1].wagon_id != ref.wagon_id:
            return "temporary", "目标车厢的指定后端当前未暴露", None, None, None, None
        if not target_train.is_parked():
            return "temporary", "连挂目标列车尚未停稳", None, None, None, None
        from controller.coupling import end_coupler_pos
        _position, edge_id, t = end_coupler_pos(target_train, "tail")
        direction = target_train.current_direction()
        start_edge, start_t, start_direction = train.current_directed_edge_and_t()
        resolution = resolve_plan_item(
            self.network,
            PathStart(start_edge, start_t, start_direction),
            item,
            passable_fn=self.passable_fn,
            allow_reversal=False,
            consist_length=train.state.consist.total_length,
            couple_target=(edge_id, t, direction),
        )
        if resolution.path is None:
            return "temporary", f"动态连挂路径不可达：{resolution.failure}", None, None, None, None
        self._pending_resolved_route = resolution.path.freeze()
        edge_length = self.network.edges[edge_id].length
        runtime_end_offset = edge_length * ((1.0 - t) if direction > 0 else t)
        return "ready", "", target_train, (edge_id, t, direction), ref.wagon_id, ref.end

    def _selector_edge_ids(self, item: PlanItem) -> tuple[int, ...]:
        """Expand POI/Station intent to a stable set of current physical edges."""
        selector = item.couple_selector
        if selector is None:
            return ()
        if selector.edge_id is not None:
            return (selector.edge_id,)
        if selector.poi_id is not None:
            poi = self.pois.get(selector.poi_id) if self.pois is not None else None
            if poi is None or getattr(poi.member_kind, "value", None) != "edge":
                return ()
            return tuple(poi.member_ids)
        station = (
            self.stations.get(selector.station_id)
            if self.stations is not None and selector.station_id is not None else None
        )
        if station is None or self.pois is None:
            return ()
        edges: list[int] = []
        for platform_id in station.platform_ids:
            platform = self.pois.get(platform_id)
            if platform is None or getattr(platform.member_kind, "value", None) != "edge":
                continue
            edges.extend(platform.member_ids)
        return tuple(dict.fromkeys(edges))

    def _couple_goal_in_scope(
        self, train: TrainEntity, item: PlanItem, trains: list[TrainEntity],
    ) -> tuple[str, str, TrainEntity | None, Goal | None, str | None, int | None]:
        """Resolve declarative scope atomically to target hook + current route snapshot."""
        selector = item.couple_selector
        edge_ids = self._selector_edge_ids(item)
        if selector is None or not edge_ids:
            return "temporary", "POI/Station 当前没有可解析的 edge", None, None, None, None
        from controller.coupling import end_coupler_pos, head_hook_offset
        own_ids = {wagon.wagon_id for wagon in train.state.consist.wagons}
        claimed = self._claimed_couplers(trains, exclude_train=train)
        start_edge, start_t, start_direction = train.current_directed_edge_and_t()
        start = PathStart(start_edge, start_t, start_direction)
        candidates = []
        allowed_directions = (
            (selector.direction,) if selector.direction in (1, -1) else (1, -1)
        )
        for target in trains:
            if target is train or not target.is_parked():
                continue
            for end, end_value, end_rank, wagon in (
                ("head", END_FRONT, 0, target.state.consist.wagons[0]),
                ("tail", END_REAR, 1, target.state.consist.wagons[-1]),
            ):
                _pos, edge_id, t = end_coupler_pos(target, end)
                claim = (wagon.wagon_id, end_value)
                if edge_id not in edge_ids or wagon.wagon_id in own_ids or claim in claimed:
                    continue
                for direction in allowed_directions:
                    draft = PlanItem.goto_couple(
                        anchors=item.anchors,
                        edge_id=edge_id,
                        direction=direction,
                    )
                    resolution = resolve_plan_item(
                        self.network, start, draft,
                        passable_fn=self.passable_fn,
                        allow_reversal=False,
                        consist_length=train.state.consist.total_length,
                        couple_target=(edge_id, t, direction),
                    )
                    if resolution.path is None:
                        continue
                    route = resolution.path.freeze()
                    edge = self.network.edges[edge_id]
                    progress = t if direction > 0 else 1.0 - t
                    end_offset = edge.length * (1.0 - progress) + head_hook_offset(train)
                    if self._project_route_start(train, route, end_offset=end_offset) is None:
                        continue
                    candidates.append((
                        resolution.path.remaining_to_goal,
                        progress,
                        wagon.wagon_id,
                        end_rank,
                        target,
                        t,
                        direction,
                        route,
                    ))
        if not candidates:
            return "temporary", "POI/Station 范围内没有未认领且可达的停放端头", None, None, None, None
        (_distance, _progress, wagon_id, end_rank, target, t, direction, route) = min(
            candidates, key=lambda value: value[:4],
        )
        self._pending_resolved_route = route
        target_end = END_FRONT if end_rank == 0 else END_REAR
        return "ready", "", target, (route.edges[-1][0], t, direction), wagon_id, target_end

    def _couple_goal_on_edge(
        self, train: TrainEntity, item: PlanItem, trains: list[TrainEntity],
    ) -> tuple[str, str, TrainEntity | None, Goal | None, str | None, int | None]:
        selector = item.couple_selector
        if selector is not None and selector.is_declarative:
            # Architecture boundary: POI/Station selectors are plan intent. Their
            # resolver will produce a current concrete edge, target hook and route
            # snapshot; they must not fall through to the legacy edge-only code.
            scope = (
                f"POI {selector.poi_id}" if selector.poi_id is not None
                else f"Station {selector.station_id}"
            )
            return "temporary", f"声明式 selector {scope} 尚未生成当前解析快照", None, None, None, None
        edge_id = selector.edge_id if selector is not None else None
        if edge_id is None:
            return "permanent", "连挂 selector 缺少 edge", None, None, None, None
        edge = self.network.edges.get(edge_id)
        if edge is None:
            return "permanent", f"连挂 selector edge {edge_id} 不存在", None, None, None, None
        from controller.coupling import end_coupler_pos, head_hook_offset
        candidates: list[tuple[float, str, int, TrainEntity, str, float, FixedRoute]] = []
        own_ids = {wagon.wagon_id for wagon in train.state.consist.wagons}
        claimed = self._claimed_couplers(trains, exclude_train=train)
        for target in trains:
            if target is train or not target.is_parked():
                continue
            for end, end_value, end_rank, wagon in (
                ("head", END_FRONT, 0, target.state.consist.wagons[0]),
                ("tail", END_REAR, 1, target.state.consist.wagons[-1]),
            ):
                _pos, candidate_edge_id, t = end_coupler_pos(target, end)
                claim = (wagon.wagon_id, end_value)
                if (candidate_edge_id != edge_id or wagon.wagon_id in own_ids
                        or claim in claimed):
                    continue
                directions = (
                    (selector.direction,) if selector.direction in (1, -1)
                    else (1, -1)
                )
                for direction in directions:
                    route = self._resolve_dynamic_couple_route(
                        train, item, edge_id, t, direction,
                    )
                    if route is None:
                        continue
                    progress = t if direction > 0 else 1.0 - t
                    candidates.append((
                        route.remaining_to_goal(self.network),
                        wagon.wagon_id, end_rank, target, end, t, route,
                    ))
        if not candidates:
            return (
                "temporary", f"edge {edge_id} 上没有未被认领且可动态到达的停放端头",
                None, None, None, None,
            )
        _distance, wagon_id, end_rank, target, _end, t, route = min(
            candidates, key=lambda value: (value[0], value[1], value[2]),
        )
        self._pending_resolved_route = route
        direction = route.edges[-1][1]
        target_end = END_FRONT if end_rank == 0 else END_REAR
        return "ready", "", target, (edge_id, t, direction), wagon_id, target_end

    def _resolve_dynamic_couple_route(
        self, train: TrainEntity, item: PlanItem, edge_id: int,
        target_t: float, direction: int,
    ) -> FixedRoute | None:
        """Resolve a coupling approach from the train's current live head.

        The returned ``FixedRoute`` is only a runtime resolution snapshot.  It
        is intentionally stored on ``PlanExecution``/``_pending_resolved_route``
        and never on the plan item.
        """
        start_edge, start_t, start_direction = train.current_directed_edge_and_t()
        draft = PlanItem.goto_couple(
            anchors=item.anchors,
            edge_id=edge_id,
            direction=direction,
        )
        resolution = resolve_plan_item(
            self.network,
            PathStart(start_edge, start_t, start_direction),
            draft,
            passable_fn=self.passable_fn,
            allow_reversal=False,
            consist_length=train.state.consist.total_length,
            couple_target=(edge_id, target_t, direction),
        )
        return resolution.path.freeze() if resolution.path is not None else None

    def _locked_couple_goal(
        self,
        train: TrainEntity,
        item: PlanItem,
        trains: list[TrainEntity],
        execution: PlanExecution,
    ) -> tuple[str, str, TrainEntity | None, Goal | None, str | None, int | None]:
        """只验证已认领端头；不得重新运行 selector 或偷换候选。"""
        wagon_id = execution.target_wagon_id
        target_end = execution.target_end
        selector = item.couple_selector
        if wagon_id is None or target_end not in (END_FRONT, END_REAR):
            return "permanent", "连挂执行令牌缺少锁定端头", None, None, None, None
        target = next(
            (candidate for candidate in trains
             if any(wagon.wagon_id == wagon_id for wagon in candidate.state.consist.wagons)),
            None,
        )
        if target is None:
            return "temporary", f"锁定连挂目标车厢 {wagon_id[:8]} 已不存在", None, None, None, None
        if target is train:
            return "temporary", "锁定连挂目标已进入本编组", None, None, None, None
        endpoint = (
            target.state.consist.wagons[0] if target_end == END_FRONT
            else target.state.consist.wagons[-1]
        )
        if endpoint.wagon_id != wagon_id:
            return "temporary", "锁定连挂端头已不再暴露", None, None, None, None
        if not target.is_parked():
            return "temporary", "锁定连挂目标列车已开始移动", None, None, None, None
        from controller.coupling import end_coupler_pos, head_hook_offset
        end = "head" if target_end == END_FRONT else "tail"
        _pos, edge_id, t = end_coupler_pos(target, end)
        allowed_edges = self._selector_edge_ids(item)
        if selector is not None and edge_id not in allowed_edges:
            return "temporary", "锁定连挂端头已离开 selector 范围", None, None, None, None
        directions = (
            (selector.direction,) if selector is not None and selector.direction in (1, -1)
            else ((target.current_direction(),) if selector is None else (1, -1))
        )
        route = None
        for direction_candidate in directions:
            route = self._resolve_dynamic_couple_route(
                train, item, edge_id, t, direction_candidate,
            )
            if route is not None:
                break
        if route is None:
            return "temporary", "锁定连挂端头当前不可动态到达", None, None, None, None
        self._pending_resolved_route = route
        direction = route.edges[-1][1]
        return "ready", "", target, (edge_id, t, direction), wagon_id, target_end

    @staticmethod
    def _claimed_couplers(
        trains: list[TrainEntity], *, exclude_train: TrainEntity,
    ) -> set[tuple[str, int]]:
        claims: set[tuple[str, int]] = set()
        for candidate in trains:
            if candidate is exclude_train:
                continue
            execution = getattr(candidate, "plan_execution", None)
            if (execution is not None and execution.target_wagon_id is not None
                    and execution.target_end in (END_FRONT, END_REAR)):
                claims.add((execution.target_wagon_id, execution.target_end))
        return claims

    @staticmethod
    def _incoming_claim(
        train: TrainEntity, trains: list[TrainEntity] | None,
    ) -> tuple[str, int] | None:
        """若本编组端头已被其它计划认领，返回稳定的首个 claim。"""
        if trains is None:
            return None
        own_ids = {wagon.wagon_id for wagon in train.state.consist.wagons}
        claims: list[tuple[str, int]] = []
        for candidate in trains:
            if candidate is train:
                continue
            execution = getattr(candidate, "plan_execution", None)
            if (execution is not None and execution.target_wagon_id in own_ids
                    and execution.target_end in (END_FRONT, END_REAR)):
                claims.append((execution.target_wagon_id, execution.target_end))
        return min(claims) if claims else None

    def _project_route_start(
        self,
        train: TrainEntity,
        route: FixedRoute,
        *,
        end_offset: float | None = None,
    ) -> tuple[list[tuple[int, int]], float] | None:
        """把车头重定位到冻结路径中的合法后缀，不移动列车或修改路网。"""
        if not route.edges:
            return None
        edge_id, t, direction = train.current_directed_edge_and_t()
        edge_length = self.network.edges[edge_id].length
        actual_offset = t * edge_length if direction > 0 else (1.0 - t) * edge_length
        current = (edge_id, direction)
        effective_end_offset = route.end_offset if end_offset is None else end_offset
        edge_lengths = [self.network.edges[eid].length for eid, _ in route.edges]
        candidates: list[tuple[float, int, bool]] = []

        # 当前车头已经位于冻结路径某次出现的有向 edge 上：该 edge 属 occupied，
        # 下发其后的后缀。重复出现时最终选到目标剩余距离最短的合法 occurrence。
        for index, directed in enumerate(route.edges):
            if directed != current:
                continue
            remaining = (
                edge_lengths[index] - actual_offset
                + sum(edge_lengths[index + 1:])
                - effective_end_offset
            )
            if remaining >= -POSITION_EPSILON_M:
                candidates.append((max(0.0, remaining), index, True))

        from model.pathfinding import head_node, tail_node
        boundary_node = head_node(self.network, current)
        remaining_to_boundary = edge_length - actual_offset
        if remaining_to_boundary <= POSITION_EPSILON_M:
            for index, following in enumerate(route.edges):
                if boundary_node != tail_node(self.network, following):
                    continue
                if current[0] == following[0] and current[1] == -following[1]:
                    if self.network.nodes[boundary_node].connection_count() != 1:
                        continue
                    segment_length, _turnout, _edges = \
                        self.network.simple_segment_from_endpoint(boundary_node)
                    if segment_length < train.state.consist.total_length:
                        continue
                elif not self.network.turn_allowed(
                    boundary_node, current[0], following[0],
                ):
                    continue
                remaining = (
                    remaining_to_boundary
                    + sum(edge_lengths[index:])
                    - effective_end_offset
                )
                if remaining >= -POSITION_EPSILON_M:
                    candidates.append((max(0.0, remaining), index, False))

        if not candidates:
            return None
        remaining, index, current_is_on_route = min(candidates, key=lambda item: item[0])
        suffix_start = index + 1 if current_is_on_route else index
        return list(route.edges[suffix_start:]), remaining

    def _route_start_error(self, train: TrainEntity, route: FixedRoute) -> str:
        edge_id, t, direction = train.current_directed_edge_and_t()
        actual_length = self.network.edges[edge_id].length
        actual_offset = t * actual_length if direction > 0 else (1.0 - t) * actual_length
        expected_edge, expected_direction = route.edges[0]
        return (
            "计划等待：动态路径无法从当前车头投影（固定路线可消费位置检查已移除）"
            f"（实际 edge {edge_id} dir {direction:+d} offset {actual_offset:.2f}m；"
            f"路线首项 edge {expected_edge} dir {expected_direction:+d} "
            f"建表 offset {route.start_offset:.2f}m）"
        )

    @staticmethod
    def _active_route(plan: Plan, item: PlanItem) -> FixedRoute | None:
        """Legacy fixed-route compatibility only; new plans never use it."""
        if plan.pointer == 0 and plan.has_wrapped and plan.loop_route is not None:
            return plan.loop_route
        return item.fixed_route

    def _at_goal(self, train: TrainEntity, item: PlanItem) -> bool:
        if item.goal is None:
            return False
        edge_id, t, direction = train.current_directed_edge_and_t()
        goal_edge_id, goal_t, goal_direction = item.goal
        return (
            edge_id == goal_edge_id
            and direction == goal_direction
            and abs(t - goal_t) * self.network.edges[edge_id].length <= POSITION_EPSILON_M
        )

    @staticmethod
    def _route_signature(route: FixedRoute | None, goal: Goal | None) -> tuple:
        return (route.edges, route.start_offset, route.end_offset, goal) if route else ()

    @staticmethod
    def _same_activation(execution: PlanExecution, winner: Wagon, plan: Plan, item: PlanItem | None) -> bool:
        return (
            item is not None
            and execution.wagon_id == winner.wagon_id
            and execution.plan_id == id(plan)
            and execution.item_id == id(item)
            and execution.pointer == plan.pointer
        )

    @staticmethod
    def _clear_execution(train: TrainEntity) -> None:
        had_plan_execution = train.plan_execution is not None
        train.plan_execution = None
        if had_plan_execution:
            train.couple_approach_partner = None

    @staticmethod
    def _report_once(train: TrainEntity, message: str) -> None:
        if getattr(train, "plan_status", "") != message:
            print(message)
            train.plan_status = message
