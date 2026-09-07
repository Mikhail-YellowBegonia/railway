from __future__ import annotations

import math


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
    """制动曲线停车控制器：连续制动曲线，在路径终点前平滑减速到停车。

    职责分离：
    - 巡航阶段：委托 SimpleSpeedController（PID）追 v_cruise
    - 制动阶段：目标速度不是固定值，是随剩余距离连续衰减的曲线
      `v_brake_target = sqrt(2·a_brake·remaining/margin)`——这是真实
      ATO（列车自动驾驶）制动曲线的标准做法：速度平滑趋近于 0，不是
      "全力刹车到某个低速再匀速爬行"的两段式手感。
    - 制动阶段用硬控制（bang-bang），不用 PID：v > 目标曲线就全力刹车，
      v < 目标曲线（且未达巡航速度）就全力加速，两者都不成立则空转。
      不能像巡航阶段那样把这条曲线交给 PID 追——PID 的增益是为追一个
      基本恒定的目标调的，目标曲线本身在快速衰减时 PID 的响应滞后会
      让车速跟不上曲线，实测直接冲过终点（v_final 还有 6+ m/s 时
      remaining 已经归零）。制动/加速两侧都可能发生这个问题：车速
      低于目标曲线时也要能主动加速追上去，否则曲线单调下降、车速
      一旦被外部因素压低就再也追不回来。

    制动所需距离公式：d_stop = v² / (2 × |a_brake|)
    a_brake 从物理层查询（throttle=0, brake=1），确保与实际制动力一致。
    加入安全余量系数 safety_margin（默认 1.3）防止过冲——目标曲线的
    v_brake_target 用同一个 margin 缩放（sqrt 形式），margin 越大，
    同样的 remaining 对应的目标速度越低，减速越保守。

    使用方式：
        ctrl = BrakingController(physics, consist)
        # 每帧：
        throttle, brake = ctrl.update(v, s, total_length, v_cruise, dt)
        # 查询是否已停车：
        if ctrl.stopped: ...
    """

    STOP_EPSILON = 0.05  # m，判定"到点"的距离容差

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
        self._braking = False
        # 制动区闩锁：一旦进入就不再退出（除非 stopped/reset），只在
        # "是否已经进入制动曲线阶段"这件事上用，不影响曲线内部的速度
        # 判断——曲线本身连续，不存在旧版"remaining<=d_stop"双向翻转
        # 震荡的问题（那是固定 d_stop 阈值才有的缺陷，见下方历史记录），
        # 但闩锁仍保留：一旦目标曲线开始生效就不应该被"remaining 反超
        # d_stop"打断退回巡航——曲线本身单调朝 0 逼近，进入后没有回头
        # 的语义。
        #
        # 历史记录（2026-08~2026-09，两轮被推翻的方案，供理解设计演进）：
        # 1. 最初用固定阈值 remaining<=d_stop 判断"是否制动"，margin>1
        #    时该判据在全力制动过程中双向翻转（d_stop 比 remaining
        #    掉得更快），导致 throttle/brake 持续震荡（实测 v_cruise=
        #    15m/s 时震荡 161 次）。加了 _braking 闩锁解决。
        # 2. 闩锁后又用"v<=STOP_SPEED 就直接判定到达"，但 safety_margin
        #    高估的制动距离在高速时是一段不小的绝对距离，车会在剩余
        #    一大段路时就已经降到阈值速度——曾经改成"降到 STOP_SPEED
        #    后转入 CRAWL_SPEED=1.8km/h 匀速爬行"，但爬行距离本身随
        #    速度平方增长（20m/s 巡航时爬行段能到 22m，要爬 44 秒），
        #    体感就是"过早减速+缓慢爬行"，用户实测报告确认。
        # 现在的连续曲线方案把上述两个问题一次性消除：不存在固定阈值
        # 可以双向翻转，也不存在"先砸到固定低速再匀速爬"的两段式手感。

    def update(
        self,
        v: float,
        s: float,
        total_length: float,
        v_cruise: float,
        dt: float,
    ) -> tuple[float, float]:
        """计算控制输出，自动在终点前切换到连续制动曲线。

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

        if remaining <= self.STOP_EPSILON:
            self.stopped = True
            self._braking = False
            return 0.0, 1.0

        # 查询当前速度下的最大制动减速度（全制动，无牵引）
        a_brake = abs(self.physics.compute_acceleration(v, 0.0, 1.0, self.consist))
        if a_brake < 1e-6:
            a_brake = 1e-6  # 防止除零

        v_brake_target = math.sqrt(
            max(0.0, 2.0 * a_brake * remaining / self.safety_margin)
        )

        if not self._braking and v >= v_brake_target:
            self._braking = True

        if self._braking:
            self.stopped = False
            self._pid.reset()  # 保持巡航 PID 积分干净，恢复巡航时无残留
            if v > v_brake_target:
                return 0.0, 1.0  # 全力制动，追曲线
            if v < v_brake_target and v < v_cruise:
                return 1.0, 0.0  # 曲线本身也允许再加速一点（还没到巡航速度）
            return 0.0, 0.0  # 恰好贴着曲线，空转

        # 巡航区：PID 控制
        self.stopped = False
        return self._pid.update(v_cruise, v, dt)

    def reset(self) -> None:
        """重置控制器状态（新路径开始时调用）。"""
        self._pid.reset()
        self.stopped = False
        self._braking = False
