from __future__ import annotations

from dataclasses import dataclass, field

from model.pathfinding import DirectedEdge, Path
from model.wagon import Consist
from model.occupancy import OccupancyState, advance_occupied_path, occupied_as_path
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

    寻路只应读取车头位置 (edge_id, t, direction)，不应引用 occupancy 或 route 本身。
    """
    occupancy: OccupancyState
    remaining_to_goal: float
    v: float        # 当前速度（m/s），0 = 停放
    consist: Consist

    @property
    def s(self) -> float:
        """车头在 occupancy 路径上的绝对弧长（只读，向后兼容旧引用语义）。"""
        return self.occupancy.s


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

    def assign_route(self, route: list[DirectedEdge], remaining_to_goal: float) -> None:
        """分配新的行驶指令：只设置 route，不改动 occupancy（车身位置/历史不受影响）。

        route: 从当前车头位置出发、尚待走的有向边序列（不含车头当前所在边）。
        remaining_to_goal: 从车头当前位置到指令终点的剩余弧长（米）。
        """
        self.state.occupancy.route = list(route)
        self.state.remaining_to_goal = remaining_to_goal
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
        new_occ, reached_end = advance_occupied_path(
            self.network, self.state.occupancy, delta_s, consist_length,
        )
        self.state.occupancy = new_occ
        self.state.remaining_to_goal = max(0.0, self.state.remaining_to_goal - delta_s)
        self.kinematics = self._build_kinematics()

        # 到达终点：停放，保留几何状态
        if reached_end or self.controller.stopped:
            self.emergency_stop()

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
        return self.kinematics._path_kin.edge_at(self.state.occupancy.s)

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
        """
        wagons = self.state.consist.wagons
        if wagon_idx < 0 or wagon_idx >= len(wagons) - 1:
            raise ValueError(f"decouple_at: wagon_idx={wagon_idx} 越界，编组共 {len(wagons)} 节")

        from model.wagon import solve_rear_bogie_s
        path_kin = self.kinematics._path_kin
        abs_s_head = self.state.occupancy.s

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

        # 前段：sub_path 从车尾到当前车头
        s_tail_front = max(0.0, abs_s_head - sum(w.length for w in wagons[:wagon_idx + 1]))
        front_path, front_offset = path_kin.sub_path(s_tail_front, abs_s_head)
        front_consist = Consist(wagons=wagons[:wagon_idx + 1])
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
        rear_consist = Consist(wagons=rear_wagons)
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

        调用方负责：
        - 确认两车已停车且满足几何条件（couple 条件检查在 GameLoop 层）
        - 从 trains 列表移除 self 和 rear，加入返回的新实体
        """
        new_wagons = self.state.consist.wagons + rear.state.consist.wagons
        new_consist = Consist(wagons=new_wagons)

        self_path_kin = self.kinematics._path_kin
        abs_s_head = self.state.occupancy.s
        combined_length = sum(w.length for w in new_wagons)
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
