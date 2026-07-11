from __future__ import annotations

from dataclasses import dataclass, field

from model.pathfinding import Path
from model.wagon import Consist
from model.rigid_kinematics import RigidWagonKinematics
from model.train_physics import TrainPhysics
from model.train_controller import BrakingController
from model.rail_network import RailNetwork


@dataclass
class TrainState:
    """列车持久化状态（纯数据，几何优先）。

    path + s 唯一确定列车在轨道上的几何位置，包括跨道岔情况。
    v 为速度，停放时为 0。
    """
    path: Path
    s: float        # 首节前转向架在 path 上的弧长（米）
    v: float        # 当前速度（m/s），0 = 停放
    consist: Consist


class TrainEntity:
    """游戏内列车对象（持久存在，不因路径结束而删除）。

    几何逻辑高于物理逻辑：
    - emergency_stop() 可随时强制归零速度，忽略物理层
    - controller 为 None 时列车停放，不接受物理更新

    状态机：
        停放（controller=None）→ 收到指令 → 行驶 → 到达 → 停放
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

        # 从 state 派生的运动学对象
        self.kinematics = RigidWagonKinematics(
            network, state.path, state.consist
        )

    # ------------------------------------------------------------------
    # 调度接口
    # ------------------------------------------------------------------

    def assign_path(
        self,
        path: Path,
        start_offset: float = 0.0,
        end_offset: float = 0.0,
    ) -> None:
        """分配新行驶路径，重建运动学对象，启动 BrakingController。

        start_offset / end_offset 来自 find_path_from_point 的返回值。
        列车从新路径的起点（initial_offset 处）出发，state.s 初始化为 0。
        """
        self.state.path = path
        self.state.s = 0.0  # 可行驶区间起点
        self.kinematics = RigidWagonKinematics(
            self.network, path, self.state.consist,
            initial_offset=start_offset,
            end_offset=end_offset,
        )
        self.controller = BrakingController(self.physics, self.state.consist)
        self.controller.reset()

    def emergency_stop(self) -> None:
        """调度层强制停车：忽略物理，直接归零速度，清除控制器。

        截取覆盖车身的最小 Path 作为停放 Path，保持几何位置连续。
        """
        self.state.v = 0.0
        self.last_a = 0.0
        self.controller = None
        self._trim_to_parking_path()

    def _trim_to_parking_path(self) -> None:
        """截取覆盖车身的最小 Path，更新 state.path / state.s / kinematics。

        停放 Path 覆盖范围：[s_head - consist.total_length - margin, s_head]
        margin = 25m（最长单节车厢估计值），防止边界误差导致车厢悬空。
        """
        MARGIN = 25.0
        # s_head：车头在底层（未裁剪）path_kin 上的绝对弧长
        s_head = self.kinematics.initial_offset + self.state.s
        s_tail = max(0.0, s_head - self.state.consist.total_length - MARGIN)

        base_kin = self.kinematics._path_kin
        park_path, initial_offset = base_kin.sub_path(s_tail, s_head)

        if not park_path.edges:
            return

        self.state.path = park_path
        # seg_start = s_tail - initial_offset（sub_path 第一段在 base_kin 上的起点）
        seg_start = s_tail - initial_offset
        self.state.s = (s_head - seg_start) - initial_offset
        self.kinematics = RigidWagonKinematics(
            self.network, park_path, self.state.consist,
            initial_offset=initial_offset,
        )

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

        throttle, brake = self.controller.update(
            self.state.v,
            self.state.s,
            self.kinematics.total_length,
            v_target,
            dt,
        )

        self.last_a = self.physics.compute_acceleration(
            self.state.v, throttle, brake, self.state.consist
        )

        self.state.v = max(0.0, self.state.v + self.last_a * dt)
        self.state.s = max(
            0.0,
            min(self.state.s + self.state.v * dt, self.kinematics.total_length),
        )

        # 到达终点：停放，保留几何状态
        if self.controller.stopped:
            self.emergency_stop()

    # ------------------------------------------------------------------
    # 几何查询（委托给 kinematics）
    # ------------------------------------------------------------------

    def is_parked(self) -> bool:
        return self.controller is None

    def is_moving(self) -> bool:
        return self.controller is not None

    def current_edge_and_t(self) -> tuple[int, float]:
        """返回车头当前所在的 Edge ID 和边内参数 t ∈ [0, 1]。

        用于从当前位置发起新一次寻路（find_path_from_point 的输入）。
        """
        abs_s = self.kinematics.initial_offset + self.state.s
        return self.kinematics._path_kin.edge_at(abs_s)

    def current_direction(self) -> int:
        """返回列车当前行驶方向（+1 或 -1）。

        从车头当前所在的有向边推断方向。
        """
        if not self.state.path.edges:
            return 1  # 默认正方向

        # 找到车头当前所在的 edge
        abs_s = self.kinematics.initial_offset + self.state.s
        current_edge_id, _ = self.kinematics._path_kin.edge_at(abs_s)

        # 在 path 中查找对应的有向边
        for eid, direction in self.state.path.edges:
            if eid == current_edge_id:
                return direction

        # 降级：返回第一条边的方向（理论上不应该到这里）
        return self.state.path.edges[0][1]

    def tail_coverage_path(self) -> tuple[Path, float, float]:
        """返回覆盖车尾到车头的路径片段（用于路径拼接）。

        返回:
            (tail_path, initial_offset_tail, s_head_in_tail_path)
            - tail_path: 从车尾位置开始到车头位置的子路径（包含完整 edge）
            - initial_offset_tail: 车尾在 tail_path 首段的局部偏移（米）
            - s_head_in_tail_path: 车头在 tail_path 上的绝对弧长（米，= s_head - s_tail）

        用于换路径时保持车尾连续性。
        """
        s_head = self.kinematics.initial_offset + self.state.s
        s_tail = max(0.0, s_head - self.state.consist.total_length)
        tail_path, initial_offset_tail = self.kinematics._path_kin.sub_path(s_tail, s_head)
        s_head_in_tail_path = s_head - s_tail
        return tail_path, initial_offset_tail, s_head_in_tail_path
