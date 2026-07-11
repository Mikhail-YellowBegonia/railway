from __future__ import annotations


class SimpleSpeedController:
    """PID 速度控制器：将目标速度转换为 (throttle, brake)。

    调用方设定 v_target，每帧调用 update() 获取控制输出。
    物理层接口不变，仍接受 throttle/brake。

    Anti-windup：积分项限幅，防止长时间偏差累积导致过冲。

    参数:
        kp: 比例增益（默认 0.3）
        ki: 积分增益（默认 0.02）
        kd: 微分增益（默认 0.05）
        integral_limit: 积分项绝对值上限（anti-windup）
    """

    def __init__(
        self,
        kp: float = 0.3,
        ki: float = 0.02,
        kd: float = 0.05,
        integral_limit: float = 10.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit

        self._integral: float = 0.0
        self._prev_error: float = 0.0

    def update(self, v_target: float, v: float, dt: float) -> tuple[float, float]:
        """计算控制输出。

        参数:
            v_target: 目标速度（m/s），应 >= 0
            v: 当前速度（m/s）
            dt: 时间步长（秒）

        返回:
            (throttle, brake)，均 ∈ [0, 1]
        """
        if dt <= 0:
            return 0.0, 0.0

        error = v_target - v

        self._integral += error * dt
        # anti-windup：积分项限幅
        self._integral = max(-self.integral_limit, min(self._integral, self.integral_limit))

        derivative = (error - self._prev_error) / dt
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative

        # output > 0 → 加速，output < 0 → 制动
        throttle = max(0.0, min(output, 1.0))
        brake = max(0.0, min(-output, 1.0))
        return throttle, brake

    def reset(self) -> None:
        """重置积分项和微分项（切换目标速度时调用，避免积分残留）。"""
        self._integral = 0.0
        self._prev_error = 0.0


class BrakingController:
    """制动曲线停车控制器：在路径终点前自动触发全制动。

    职责分离：
    - 巡航阶段：委托 SimpleSpeedController（PID）
    - 停车阶段：当剩余距离 ≤ 制动所需距离时，覆盖 PID 输出全制动

    制动所需距离公式：d_stop = v² / (2 × |a_brake|)
    a_brake 从物理层查询（throttle=0, brake=1），确保与实际制动力一致。
    加入安全余量系数 safety_margin（默认 1.3）防止过冲。

    使用方式：
        ctrl = BrakingController(physics, consist)
        # 每帧：
        throttle, brake = ctrl.update(v, s, total_length, v_cruise, dt)
        # 查询是否已停车：
        if ctrl.stopped: ...
    """

    STOP_SPEED = 0.3   # m/s，低于此速度视为停车

    def __init__(
        self,
        physics,           # TrainPhysics
        consist,           # Consist
        safety_margin: float = 1.3,
        cruise_controller: SimpleSpeedController | None = None,
    ) -> None:
        self.physics = physics
        self.consist = consist
        self.safety_margin = safety_margin
        self._pid = cruise_controller or SimpleSpeedController()
        self.stopped = False

    def update(
        self,
        v: float,
        s: float,
        total_length: float,
        v_cruise: float,
        dt: float,
    ) -> tuple[float, float]:
        """计算控制输出，自动在终点前切换到制动曲线。

        参数:
            v: 当前速度（m/s）
            s: 当前弧长（米）
            total_length: 路径可行驶总长（米）
            v_cruise: 巡航目标速度（m/s），由玩家设定
            dt: 时间步长（秒）

        返回:
            (throttle, brake)，均 ∈ [0, 1]
        """
        remaining = total_length - s

        # 查询当前速度下的最大制动减速度（全制动，无牵引）
        a_brake = abs(self.physics.compute_acceleration(v, 0.0, 1.0, self.consist))
        if a_brake < 1e-6:
            a_brake = 1e-6  # 防止除零

        # 制动所需距离（含安全余量）
        d_stop = (v * v) / (2.0 * a_brake) * self.safety_margin

        if remaining <= 0.0 or (v <= self.STOP_SPEED and remaining < 1.0):
            # 已到达终点
            self.stopped = True
            return 0.0, 1.0

        if remaining <= d_stop:
            # 进入制动区：覆盖 PID，全制动
            # 重置 PID 积分，避免恢复巡航时有残留
            self._pid.reset()
            self.stopped = False
            return 0.0, 1.0

        # 巡航区：PID 控制
        self.stopped = False
        return self._pid.update(v_cruise, v, dt)

    def reset(self) -> None:
        """重置控制器状态（新路径开始时调用）。"""
        self._pid.reset()
        self.stopped = False
