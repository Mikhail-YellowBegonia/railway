from __future__ import annotations

from dataclasses import dataclass, field

from model.pathfinding import DirectedEdge, Path
from model.wagon import Consist
from model.occupancy import (
    OccupancyState, advance_occupied_path, occupied_as_path,
    reverse_occupancy, truncate_to_segment,
)
from model.rigid_kinematics import RigidWagonKinematics
from model.train_physics import TrainPhysics
from model.train_controller import BrakingController
from model.rail_network import RailNetwork


@dataclass
class TrainState:
    """列车持久化状态（纯数据，几何优先）。

    三段式设计（Step 1 重构）：
    - occupancy: 车身占用轨迹（滑动窗口，权威几何状态，唯一真源）
    - route: 待走的有向边（寻路结果未消费部分），为空 = 无指令，随占用推进逐边消耗
    - remaining_to_goal: 距离本次指令终点的剩余弧长（米），与 occupancy 的边界无关，
      纯粹按 delta_s 递减；供 BrakingController 判断制动/停车，不依赖 occupancy 内部坐标
    - goal: 本次指令的最终目标 (edge_id, t, direction)，为空 = 无指令。
      Step 3 part2：route 中若含中途折返点，寻路阶段按"点模型"铺设的剩余
      路径无法直接拼接刚体列车折返后的新车头位置（折返需要额外走完"车身
      长度"才能重新抵达折返节点，寻路结果没有算这段），所以折返触发时不
      复用旧 route，而是从新车头位置对 goal 重新寻路。

    寻路只应读取车头位置 (edge_id, t, direction)，不应引用 occupancy 或 route 本身。
    """
    occupancy: OccupancyState
    remaining_to_goal: float
    v: float        # 当前速度（m/s），0 = 停放
    consist: Consist
    goal: tuple[int, float, int] | None = None

    @property
    def s(self) -> float:
        """车头在 occupancy 路径上的绝对弧长（只读，向后兼容旧引用语义）。

        注意：这是相对 occupancy.occupied_offset 的坐标（= kinematics.get_all_wagon_poses
        等方法期望的 front_bogie_s 参数）。若要直接对 self.kinematics._path_kin 取值
        （绕过 initial_offset 自动加成），必须用 abs_s，不能直接用这个属性。
        """
        return self.occupancy.s

    @property
    def abs_s(self) -> float:
        """车头在底层 _path_kin 坐标系下的绝对弧长（= occupancy.s + occupied_offset）。

        直接调用 self.kinematics._path_kin.* 时必须用这个值，不能用 occupancy.s——
        后者是相对 occupied_offset 的坐标，kinematics 高层方法（get_all_wagon_poses
        等）会自动加上 initial_offset，但 _path_kin 是底层对象，不会自动加。
        """
        return self.occupancy.s + self.occupancy.occupied_offset


class TrainEntity:
    """游戏内列车对象（持久存在，不因路径结束而删除）。

    几何逻辑高于物理逻辑：
    - emergency_stop() 可随时强制归零速度，忽略物理层
    - controller 为 None 时列车停放，不接受物理更新

    状态机：
        停放（controller=None）→ 收到指令（assign_route）→ 行驶 → 到达 → 停放
    """

    def __init__(
        self,
        state: TrainState,
        network: RailNetwork,
        physics: TrainPhysics,
    ) -> None:
        self.state = state
        self.network = network
        self.physics = physics
        self.controller: BrakingController | None = None
        self.last_a: float = 0.0        # 上一帧加速度（HUD 显示用）
        self.v_target: float = 0.0      # 玩家设定的巡航速度

        self.kinematics = self._build_kinematics()

    def _build_kinematics(self) -> RigidWagonKinematics:
        """从 state.occupancy 重建运动学对象（occupied 边列表 → Path）。"""
        path = occupied_as_path(self.state.occupancy, self.network)
        return RigidWagonKinematics(
            self.network, path, self.state.consist,
            initial_offset=self.state.occupancy.occupied_offset,
        )

    # ------------------------------------------------------------------
    # 调度接口
    # ------------------------------------------------------------------

    def assign_route(
        self,
        route: list[DirectedEdge],
        remaining_to_goal: float,
        goal: tuple[int, float, int] | None = None,
    ) -> None:
        """分配新的行驶指令：只设置 route，不改动 occupancy（车身位置/历史不受影响）。

        route: 从当前车头位置出发、尚待走的有向边序列（不含车头当前所在边）。
        remaining_to_goal: 从车头当前位置到指令终点的剩余弧长（米）。
        goal: 本次指令的最终目标 (edge_id, t, direction)，供中途折返时重新
              寻路使用；不提供则折返触发时直接放弃剩余 route（安全兜底）。
        """
        self.state.occupancy.route = list(route)
        self.state.remaining_to_goal = remaining_to_goal
        self.state.goal = goal
        self.controller = BrakingController(self.physics, self.state.consist)
        self.controller.reset()

    def emergency_stop(self) -> None:
        """调度层强制停车：忽略物理，直接归零速度，清空待走指令。

        occupancy 不需要额外裁剪——它本身就是滑动窗口，持续保持车身覆盖。
        """
        self.state.v = 0.0
        self.last_a = 0.0
        self.controller = None
        self.state.occupancy.route = []
        self.state.remaining_to_goal = 0.0

    def hard_stop(self) -> None:
        """接口占位（Step 3/4 信号系统用，当前无调用点）：无视当前速度和
        制动曲线，下一帧强制速度不连续地归零。

        与 emergency_stop 的区别：emergency_stop 是调度层主动取消指令的
        正常操作（此刻车速可能仍在合理范围内减速）；hard_stop 对应的是
        "信号灯突然变红，但列车已经越过安全制动距离，物理减速曲线来不及
        让车停在信号前"这种异常场景（JGRPP 的 realistic braking 称为
        "无法安全制动"的兜底）——本质是承认物理约束已经无法满足，用
        牺牲连续性的方式保住"不闯红灯"这条更高优先级的不变量。

        现在只占位不实现：唯一可能触发它的场景（信号强制停车）在
        Step 3/4 才存在，没有真实调用方就没法验证"什么阈值算来不及"，
        提前写实现等于裸测。触发时应打印可见警告（不能静默吞掉）——
        否则玩家会把它误当成 bug，而不是"游戏在保护你"，这是 JGRPP
        社区反馈里反复出现的抱怨。
        """
        raise NotImplementedError("hard_stop: Step 3/4 信号系统实现，当前无调用点")

    def reverse_in_place(self) -> None:
        """原地折返：车头车尾互换，车身占用的物理边集合不变。

        只允许停车时调用（折返不是物理倒车，是逻辑方向翻转，要求先静止）。
        调用后 route 清空、remaining_to_goal 归零——旧指令基于旧车头方向，
        折返后必须重新寻路才有意义，不会自动继续。

        车厢本身没有物理旋转（车厢没有转身，只是重新定义哪端朝前），所以
        必须同步反转 Consist（reversed_consist），否则 get_all_wagon_poses
        仍按旧的 wagons[0] 链式求解，会在几何上出现车厢错位/重叠——这正是
        最初报告的"转向架反弹"bug 的根源。
        """
        if not self.is_parked():
            raise RuntimeError("reverse_in_place: 只能在停车状态下折返")

        # 折返前先截断掉 occupied 中不属于当前 simple_segment 的前缀
        # （turnout 背面的边）——否则反转会把这些边也带上，车头折返后
        # 会沿着错误方向走上原路径（Step3 死循环 bug 的真正成因）。
        from model.pathfinding import head_node
        endpoint_node_id = head_node(self.network, self.state.occupancy.occupied[-1])
        _seg_len, _turnout, segment_edge_ids = self.network.simple_segment_from_endpoint(
            endpoint_node_id,
        )
        self.state.occupancy = truncate_to_segment(
            self.state.occupancy, set(segment_edge_ids), self.network,
        )
        self.kinematics = self._build_kinematics()

        real_tail_bogie_abs_s = self.kinematics.real_tail_bogie_abs_s(self.state.s)
        self.state.occupancy = reverse_occupancy(
            self.network, self.state.occupancy,
            real_tail_bogie_abs_s, self.state.consist.total_length,
        )
        self.state.consist = self.state.consist.reversed_consist()
        self.state.remaining_to_goal = 0.0
        self.kinematics = self._build_kinematics()

    # ------------------------------------------------------------------
    # 每帧更新
    # ------------------------------------------------------------------

    def update(self, dt: float, v_target: float) -> None:
        """推进一帧物理状态。停放中（controller=None）时跳过。

        参数:
            dt: 时间步长（秒）
            v_target: 玩家当前设定的巡航速度（m/s）
        """
        if self.controller is None or dt <= 0:
            return

        self.v_target = v_target

        # BrakingController 只需要“剩余距离”这个标量，与 occupancy 边界无关
        throttle, brake = self.controller.update(
            self.state.v,
            0.0,
            self.state.remaining_to_goal,
            v_target,
            dt,
        )

        self.last_a = self.physics.compute_acceleration(
            self.state.v, throttle, brake, self.state.consist
        )

        self.state.v = max(0.0, self.state.v + self.last_a * dt)
        delta_s = self.state.v * dt

        consist_length = self.state.consist.total_length
        new_occ, reached_end, needs_reversal = advance_occupied_path(
            self.network, self.state.occupancy, delta_s, consist_length,
        )
        self.state.occupancy = new_occ
        self.state.remaining_to_goal = max(0.0, self.state.remaining_to_goal - delta_s)
        self.kinematics = self._build_kinematics()

        if needs_reversal:
            self._do_auto_reversal()
            return

        # 到达终点：停放，保留几何状态
        if reached_end or self.controller.stopped:
            self.emergency_stop()

    def _do_auto_reversal(self) -> None:
        """行驶到 route 中的折返点：停车、原地翻转、直接消费剩余 route 继续。

        早前弯路（已废弃，不要重犯）：曾经尝试折返后重新对 state.goal 调用
        一次寻路，理由是"寻路把列车当点，折返后车头退回车身长度，和寻路
        结果的死端节点不是同一个位置，直接拼接会几何断裂"。这个诊断本身
        没错，但"重新寻路"是错误的修复方向——寻路算法没有记忆，每次都从
        当前局部拓扑重新决策，遇到"口袋型"死胡同会反复判定"这里该折返"，
        导致折返→重新寻路→又判定折返的无限振荡（已实测复现：两个死端间
        每 6 秒振荡一次，永远到不了终点）。

        真正的修复是 Step3 阶段 C：reverse_in_place() 在反转前用
        truncate_to_segment() 把 occupied 精确截断到 simple_segment 范围
        （丢弃 turnout 背面的边）。只要这个截断是对的，折返后的新车头就
        必然落在 simple_segment 内部，而 route 剩余部分（advance_occupied_path
        已经弹出了折返标记边）本来就是 Dijkstra 一次性算好的、从 turnout
        出发的正确路径——不需要重算，直接接上即可，这样"重新寻路"带来的
        振荡风险完全消失（寻路只发生一次，折返只是几何操作）。

        remaining_to_goal 保留：reverse_in_place() 作为通用原语会把它清零
        （孤立调用时旧目标已失效），但这里终点没变，要手动恢复已递减过的
        剩余距离。
        """
        remaining_route = list(self.state.occupancy.route)
        remaining_to_goal = self.state.remaining_to_goal
        goal = self.state.goal
        self.state.v = 0.0
        self.last_a = 0.0
        self.controller = None  # reverse_in_place 要求 is_parked()
        self.reverse_in_place()
        self.state.occupancy.route = remaining_route
        self.state.remaining_to_goal = remaining_to_goal
        self.state.goal = goal
        self.controller = BrakingController(self.physics, self.state.consist)
        self.controller.reset()

    # ------------------------------------------------------------------
    # 几何查询
    # ------------------------------------------------------------------

    def is_parked(self) -> bool:
        return self.controller is None

    def is_moving(self) -> bool:
        return self.controller is not None

    def current_edge_and_t(self) -> tuple[int, float]:
        """返回车头当前所在的 Edge ID 和边内参数 t ∈ [0, 1]。

        用于从当前位置发起新一次寻路（find_path_from_point 的输入）。
        """
        return self.kinematics._path_kin.edge_at(self.state.abs_s)

    def current_direction(self) -> int:
        """返回列车当前行驶方向（+1 或 -1）：occupied 最后一条边的方向。"""
        if not self.state.occupancy.occupied:
            return 1
        return self.state.occupancy.occupied[-1][1]

    def head_directed_edge(self) -> DirectedEdge:
        """车头当前所在的有向边（occupied 最后一条）。"""
        return self.state.occupancy.occupied[-1]

    # ------------------------------------------------------------------
    # Couple / Decouple
    # ------------------------------------------------------------------

    def decouple_at(self, wagon_idx: int) -> tuple["TrainEntity", "TrainEntity"]:
        """在第 wagon_idx 节车厢后解挂，返回 (前段, 后段) 两个停放实体。

        当前阶段：强制两段都停车（v=0，route 清空）。
        wagon_idx: 0-indexed，前段保留 wagons[0..wagon_idx]，后段 wagons[wagon_idx+1..]。

        前提：只能在本车已停车时调用——运行中的车身覆盖是连续物理约束，
        切成两段会让"车身中段速度不为零"这种物理无意义状态成立。之前
        只在 GameLoop 调用前检查过一次，这里补一道数据层断言，不依赖
        调用方自觉（未来调度 AI/批处理脚本等新入口不会绕过这条不变量）。
        """
        if not self.is_parked():
            raise RuntimeError("decouple_at: 只能在停车状态下解挂")

        wagons = self.state.consist.wagons
        if wagon_idx < 0 or wagon_idx >= len(wagons) - 1:
            raise ValueError(f"decouple_at: wagon_idx={wagon_idx} 越界，编组共 {len(wagons)} 节")

        from model.wagon import solve_rear_bogie_s
        path_kin = self.kinematics._path_kin
        abs_s_head = self.state.abs_s

        bogie_s: list[tuple[float, float]] = []
        current_s = abs_s_head
        for i, wagon in enumerate(wagons):
            front_s = current_s
            rear_s = solve_rear_bogie_s(front_s, wagon.bogie_spacing, path_kin)
            bogie_s.append((front_s, rear_s))
            if i < len(wagons) - 1:
                next_wagon = wagons[i + 1]
                gap = (wagon.coupler_2_pos - wagon.bogies[1].pos) + \
                      (next_wagon.bogies[0].pos - next_wagon.coupler_1_pos)
                current_s = solve_rear_bogie_s(rear_s, gap, path_kin)

        rear_head_s = bogie_s[wagon_idx + 1][0]
        front_consist, rear_consist = self.state.consist.split_at(wagon_idx)

        # 前段：sub_path 从车尾到当前车头
        s_tail_front = max(0.0, abs_s_head - sum(w.length for w in wagons[:wagon_idx + 1]))
        front_path, front_offset = path_kin.sub_path(s_tail_front, abs_s_head)
        front_occ = OccupancyState(
            occupied=list(front_path.edges),
            occupied_offset=front_offset,
            s=abs_s_head - s_tail_front,
            route=[],
        )
        front_state = TrainState(
            occupancy=front_occ, remaining_to_goal=0.0, v=0.0, consist=front_consist,
        )
        front_entity = TrainEntity(front_state, self.network, self.physics)

        # 后段：sub_path 从后段车尾到后段车头
        rear_wagons = wagons[wagon_idx + 1:]
        rear_tail_s = max(0.0, rear_head_s - sum(w.length for w in rear_wagons))
        rear_path, rear_offset = path_kin.sub_path(rear_tail_s, rear_head_s)
        rear_occ = OccupancyState(
            occupied=list(rear_path.edges),
            occupied_offset=rear_offset,
            s=rear_head_s - rear_tail_s,
            route=[],
        )
        rear_state = TrainState(
            occupancy=rear_occ, remaining_to_goal=0.0, v=0.0, consist=rear_consist,
        )
        rear_entity = TrainEntity(rear_state, self.network, self.physics)

        return front_entity, rear_entity

    def couple_with(self, rear: "TrainEntity") -> "TrainEntity":
        """将 rear 连挂到本列车车尾，返回合并后的新停放实体。

        前提：两车都必须已停车——同 decouple_at，数据层断言不依赖调用方
        自觉。几何对齐条件（车钩距离/朝向）仍由 GameLoop 层的 _try_couple
        负责检查，那部分是"游戏规则"而非"物理不变量"，留在 controller。

        调用方负责：
        - 从 trains 列表移除 self 和 rear，加入返回的新实体
        """
        if not self.is_parked() or not rear.is_parked():
            raise RuntimeError("couple_with: 两列车都必须先停车")

        new_consist = self.state.consist.merged_with(rear.state.consist)

        self_path_kin = self.kinematics._path_kin
        abs_s_head = self.state.abs_s
        combined_length = new_consist.total_length
        s_tail_new = max(0.0, abs_s_head - combined_length)
        new_path, new_initial_offset = self_path_kin.sub_path(s_tail_new, abs_s_head)

        new_occ = OccupancyState(
            occupied=list(new_path.edges),
            occupied_offset=new_initial_offset,
            s=abs_s_head - s_tail_new,
            route=[],
        )
        new_state = TrainState(
            occupancy=new_occ, remaining_to_goal=0.0, v=0.0, consist=new_consist,
        )
        new_entity = TrainEntity(new_state, self.network, self.physics)
        return new_entity
