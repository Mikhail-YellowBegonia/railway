"""P3b：将控制车的冻结计划投影为既有列车运行指令。

本模块只读取 ``FixedRoute``，从不导入或调用编辑期的 P2/Dijkstra。它不替代
``model.dispatch`` 的信号预约：这里只在条目首次激活时下达一次 ``assign_route``，
之后仍由既有闭塞调度器推进、等待和恢复。
"""

from __future__ import annotations

from dataclasses import dataclass

from model.plan import END_FRONT, END_REAR, FixedRoute, Goal, Plan, PlanCommand, PlanItem
from model.rail_network import RailNetwork
from model.train_controller import BrakingController
from model.train_entity import TrainEntity
from model.wagon import Wagon


PLAN_CRUISE_SPEED = 12.0
POSITION_EPSILON_M = 0.1
COUPLE_TARGET_DRIFT_M = 5.0


@dataclass
class PlanExecution:
    """编组运行期派生的计划激活令牌，不属于车厢域数据。"""

    wagon_id: str
    plan_id: int
    item_id: int
    pointer: int
    route_signature: tuple
    projected_goal: Goal | None


class PlanDispatcher:
    """控制车遴选、固定路线投影和到达事件步进。"""

    def __init__(self, network: RailNetwork) -> None:
        self.network = network

    def tick(self, train: TrainEntity, trains: list[TrainEntity] | None = None) -> None:
        """在既有信号调度前运行一次；只在首次激活时写入列车指令。"""
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
        item = plan.current()
        if item is None:
            if not train.is_parked():
                train.emergency_stop()
            self._clear_execution(train)
            return

        if execution is not None and self._same_activation(execution, winner, plan, item):
            if item.command is PlanCommand.GOTO_COUPLE:
                outcome = self._couple_goal(train, item, trains)
                expected_goal = outcome[3]
                if outcome[0] != "ready" or expected_goal != execution.projected_goal:
                    train.emergency_stop()
                    self._clear_execution(train)
                    if outcome[0] == "permanent":
                        plan.advance()
                        self._report_once(train, f"计划跳过：{outcome[1]}")
                    else:
                        reason = outcome[1] or "连挂目标在执行中移动，需重新确认固定接近路线"
                        self._report_once(train, f"计划等待：{reason}")
                    return
            # 普通右键/K 调车允许覆盖计划投影出的运行指令。目标一旦不同，旧令牌
            # 立即失效；不在行驶中抢回 route，待人工指令结束后再按起点契约判定。
            if train.state.goal != execution.projected_goal:
                self._clear_execution(train)
                self._report_once(train, "计划被临时行车指令覆盖，等待重新对齐")
                return
            # 新激活的极短路线可能在首次物理帧内就完成，因而不在 GameLoop
            # 的 moving_before 集合里；这里补偿该停车事件，仍只推进一次。
            if train.is_parked() and item.command is PlanCommand.GOTO and self._at_goal(train, item):
                plan.advance()
                self._clear_execution(train)
                return
            # P4 可机械改写 FixedRoute；已有运行 route 已在同一事务中被改写。
            # 这里只同步令牌，不能从当前车头重新下单或重新寻路。
            execution.route_signature = self._route_signature(item)
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
            target_id = item.train_ref.wagon_id if item.train_ref is not None else ""
            winner_group = next(
                (group for group in participant_wagon_groups if winner.wagon_id in group),
                set(),
            )
            target_group = next(
                (group for group in participant_wagon_groups if target_id in group),
                set(),
            )
            if not target_group or target_group is winner_group:
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
        candidates = train.state.consist.control_cars()
        return min(candidates, key=lambda wagon: (-wagon.priority, wagon.wagon_id), default=None)

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
            problems = item.validate(self.network, require_fixed_route=True)
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
            if item.command is PlanCommand.GOTO_COUPLE:
                outcome = self._couple_goal(train, item, trains)
                if outcome[0] == "permanent":
                    self._report_once(train, f"计划跳过：{outcome[1]}")
                    plan.advance()
                    continue
                if outcome[0] == "temporary":
                    if not train.is_parked():
                        train.emergency_stop()
                    self._report_once(train, f"计划等待：{outcome[1]}")
                    return
                target_train, goal = outcome[2], outcome[3]
                if not train.is_parked():
                    self._report_once(train, "计划等待当前运行指令结束")
                    return
                if item.fixed_route is None:
                    self._report_once(train, "计划错误：前往连挂缺少固定路径")
                    return
                projection = self._project_route_start(train, item.fixed_route)
                if projection is None:
                    self._report_once(train, "计划等待：车头不在固定路线起点，拒绝重新寻路")
                    return
                from controller.coupling import head_hook_offset
                route, start_adjustment = projection
                stop_before = head_hook_offset(train)
                remaining = (
                    self._remaining_to_couple(item.fixed_route, goal, stop_before)
                    + start_adjustment
                )
                train._stop_before_m = stop_before
                train.assign_route(route, remaining, goal)
                train.couple_approach_partner = target_train
                train.v_target = max(train.v_target, PLAN_CRUISE_SPEED)
                train.plan_execution = PlanExecution(
                    wagon_id=winner.wagon_id,
                    plan_id=id(plan),
                    item_id=id(item),
                    pointer=plan.pointer,
                    route_signature=self._route_signature(item),
                    projected_goal=goal,
                )
                train.plan_status = ""
                return
            if item.command is not PlanCommand.GOTO or item.fixed_route is None or item.goal is None:
                self._report_once(train, "计划条目不可执行")
                return
            if not train.is_parked():
                self._report_once(train, "计划等待当前运行指令结束")
                return
            projection = self._project_route_start(train, item.fixed_route)
            if projection is None:
                self._report_once(train, "计划等待：车头不在固定路线起点，拒绝重新寻路")
                return
            route, start_adjustment = projection
            train._stop_before_m = 0.0
            remaining = item.fixed_route.remaining_to_goal(self.network) + start_adjustment
            train.assign_route(route, max(0.0, remaining), item.goal)
            train.v_target = max(train.v_target, PLAN_CRUISE_SPEED)
            train.plan_execution = PlanExecution(
                wagon_id=winner.wagon_id,
                plan_id=id(plan),
                item_id=id(item),
                pointer=plan.pointer,
                route_signature=self._route_signature(item),
                projected_goal=item.goal,
            )
            train.plan_status = ""
            return
        self._clear_execution(train)
        self._report_once(train, "计划整圈均永久失效，列车停车等待")

    def _couple_goal(
        self,
        train: TrainEntity,
        item: PlanItem,
        trains: list[TrainEntity] | None,
    ) -> tuple[str, str, TrainEntity | None, Goal | None]:
        ref = item.train_ref
        if ref is None:
            return "permanent", "前往连挂缺少目标", None, None
        if trains is None:
            return "temporary", "当前列车表不可用，无法解析连挂目标", None, None
        target_train = next(
            (candidate for candidate in trains
             if any(wagon.wagon_id == ref.wagon_id
                    for wagon in candidate.state.consist.wagons)),
            None,
        )
        if target_train is None:
            return "permanent", f"连挂目标车厢 {ref.wagon_id[:8]} 已不存在", None, None
        if target_train is train:
            return "temporary", "连挂目标已在本编组内", None, None
        if ref.end == END_FRONT:
            return "temporary", "当前版本不支持计划驶向目标前端（无倒车连挂）", None, None
        if ref.end != END_REAR:
            return "permanent", f"连挂目标端头非法：{ref.end}", None, None
        if target_train.state.consist.wagons[-1].wagon_id != ref.wagon_id:
            return "temporary", "目标车厢的指定后端当前未暴露", None, None
        if not target_train.is_parked():
            return "temporary", "连挂目标列车尚未停稳", None, None
        if item.fixed_route is None:
            return "permanent", "前往连挂尚未冻结固定路径", None, None

        from controller.coupling import end_coupler_pos
        _position, edge_id, t = end_coupler_pos(target_train, "tail")
        direction = target_train.current_direction()
        if item.fixed_route.edges[-1] != (edge_id, direction):
            return "temporary", "目标后钩已离开固定路线末边或改变方向", None, None
        edge_length = self.network.edges[edge_id].length
        runtime_end_offset = edge_length * ((1.0 - t) if direction > 0 else t)
        if abs(runtime_end_offset - item.fixed_route.end_offset) > COUPLE_TARGET_DRIFT_M:
            return "temporary", "目标后钩偏离冻结位置超过 5 米", None, None
        return "ready", "", target_train, (edge_id, t, direction)

    def _remaining_to_couple(
        self, route: FixedRoute, goal: Goal, stop_before: float,
    ) -> float:
        edge_id, t, direction = goal
        edge_length = self.network.edges[edge_id].length
        runtime_end_offset = edge_length * ((1.0 - t) if direction > 0 else t)
        total = sum(self.network.edges[current].length for current, _ in route.edges)
        return max(0.0, total - route.start_offset - runtime_end_offset - stop_before)

    def _project_route_start(
        self, train: TrainEntity, route: FixedRoute,
    ) -> tuple[list[tuple[int, int]], float] | None:
        """把厘米级停车误差投影到冻结路径起点，不移动列车或修改路网。"""
        if not route.edges:
            return None
        edge_id, t = train.current_edge_and_t()
        direction = train.current_direction()
        edge_length = self.network.edges[edge_id].length
        actual_offset = t * edge_length if direction > 0 else (1.0 - t) * edge_length
        current = (edge_id, direction)
        first = route.edges[0]

        if current == first:
            adjustment = route.start_offset - actual_offset
            if abs(adjustment) <= POSITION_EPSILON_M:
                return list(route.edges[1:]), adjustment
            return None

        from model.pathfinding import head_node, tail_node
        boundary_node = head_node(self.network, current)
        if boundary_node != tail_node(self.network, first):
            return None
        remaining_to_boundary = edge_length - actual_offset
        adjustment = remaining_to_boundary + route.start_offset
        if adjustment > POSITION_EPSILON_M:
            return None

        if current[0] == first[0] and current[1] == -first[1]:
            if self.network.nodes[boundary_node].connection_count() != 1:
                return None
            segment_length, _turnout, _edges = \
                self.network.simple_segment_from_endpoint(boundary_node)
            if segment_length < train.state.consist.total_length:
                return None
        elif not self.network.turn_allowed(boundary_node, current[0], first[0]):
            return None
        return list(route.edges), adjustment

    def _at_goal(self, train: TrainEntity, item: PlanItem) -> bool:
        if item.goal is None:
            return False
        edge_id, t = train.current_edge_and_t()
        goal_edge_id, goal_t, goal_direction = item.goal
        return (
            edge_id == goal_edge_id
            and train.current_direction() == goal_direction
            and abs(t - goal_t) * self.network.edges[edge_id].length <= POSITION_EPSILON_M
        )

    @staticmethod
    def _route_signature(item: PlanItem) -> tuple:
        route = item.fixed_route
        return (route.edges, route.start_offset, route.end_offset, item.goal) if route else ()

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
