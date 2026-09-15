"""P3b：将控制车的冻结计划投影为既有列车运行指令。

本模块只读取 ``FixedRoute``，从不导入或调用编辑期的 P2/Dijkstra。它不替代
``model.dispatch`` 的信号预约：这里只在条目首次激活时下达一次 ``assign_route``，
之后仍由既有闭塞调度器推进、等待和恢复。
"""

from __future__ import annotations

from dataclasses import dataclass

from model.plan import FixedRoute, Plan, PlanCommand, PlanItem
from model.rail_network import RailNetwork
from model.train_controller import BrakingController
from model.train_entity import TrainEntity
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


class PlanDispatcher:
    """控制车遴选、固定路线投影和到达事件步进。"""

    def __init__(self, network: RailNetwork) -> None:
        self.network = network

    def tick(self, train: TrainEntity) -> None:
        """在既有信号调度前运行一次；只在首次激活时写入列车指令。"""
        winner = self._winner(train)
        execution = getattr(train, "plan_execution", None)

        if winner is None or winner.plan is None:
            if execution is not None:
                self._clear_execution(train)
            return

        plan = winner.plan
        item = plan.current()
        if item is None:
            if not train.is_parked():
                train.emergency_stop()
            self._clear_execution(train)
            return

        if execution is not None and self._same_activation(execution, winner, plan, item):
            # 普通右键/K 调车允许覆盖计划投影出的运行指令。目标一旦不同，旧令牌
            # 立即失效；不在行驶中抢回 route，待人工指令结束后再按起点契约判定。
            if train.state.goal != item.goal:
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
        self._activate(train, winner, plan)

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

    @staticmethod
    def _winner(train: TrainEntity) -> Wagon | None:
        """只在持有计划的控制车中稳定遴选，避免无计划控制车劫持人工指令。"""
        candidates = [
            wagon for wagon in train.state.consist.control_cars()
            if wagon.plan is not None
        ]
        return min(candidates, key=lambda wagon: (-wagon.priority, wagon.wagon_id), default=None)

    def _activate(self, train: TrainEntity, winner: Wagon, plan: Plan) -> None:
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
                self._report_once(train, "计划等待连挂")
                return
            if item.command is PlanCommand.GOTO_COUPLE:
                self._report_once(train, "计划前往连挂等待 P7 事件接线")
                return
            if item.command is not PlanCommand.GOTO or item.fixed_route is None or item.goal is None:
                self._report_once(train, "计划条目不可执行")
                return
            if not train.is_parked():
                self._report_once(train, "计划等待当前运行指令结束")
                return
            if not self._at_route_start(train, item.fixed_route):
                self._report_once(train, "计划等待：车头不在固定路线起点，拒绝重新寻路")
                return
            route = list(item.fixed_route.edges[1:])
            train._stop_before_m = 0.0
            train.assign_route(route, item.fixed_route.remaining_to_goal(self.network), item.goal)
            train.v_target = max(train.v_target, PLAN_CRUISE_SPEED)
            train.plan_execution = PlanExecution(
                wagon_id=winner.wagon_id,
                plan_id=id(plan),
                item_id=id(item),
                pointer=plan.pointer,
                route_signature=self._route_signature(item),
            )
            train.plan_status = ""
            return
        self._clear_execution(train)
        self._report_once(train, "计划整圈均永久失效，列车停车等待")

    def _at_route_start(self, train: TrainEntity, route: FixedRoute) -> bool:
        if not route.edges:
            return False
        edge_id, t = train.current_edge_and_t()
        direction = train.current_direction()
        if (edge_id, direction) != route.edges[0]:
            return False
        edge_length = self.network.edges[edge_id].length
        expected_t = (route.start_offset / edge_length if direction > 0
                      else 1.0 - route.start_offset / edge_length)
        return abs(t - expected_t) * edge_length <= POSITION_EPSILON_M

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
        train.plan_execution = None

    @staticmethod
    def _report_once(train: TrainEntity, message: str) -> None:
        if getattr(train, "plan_status", "") != message:
            print(message)
            train.plan_status = message
