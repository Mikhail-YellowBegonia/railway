from __future__ import annotations

from abc import ABC, abstractmethod


class TrainPhysics(ABC):
    """列车物理层抽象接口（无状态，纯函数式）。

    职责：根据当前状态（速度、轨道元数据）和控制输入，计算瞬时加速度。
    状态演化（v 和 s 的积分）由调用方（运动学层或状态管理器）负责。
    """

    @abstractmethod
    def compute_acceleration(
        self,
        v: float,
        throttle: float,
        brake: float,
        track_metadata: dict | None = None,
    ) -> float:
        """计算瞬时加速度（m/s²）。

        参数:
            v: 当前速度（m/s）
            throttle: 油门/功率手柄（∈ [0, 1]，0=怠速，1=全开）
            brake: 制动（∈ [0, 1]，0=无制动，1=全制动）
            track_metadata: 轨道元数据（可选），如 {'curve_radius': float, 'grade': float}
                用于计算曲线/坡度附加阻力（SimplePhysics 忽略，真实物理模型使用）

        返回:
            加速度（m/s²），可正（加速）可负（减速）
        """
        pass


class SimplePhysics(TrainPhysics):
    """玩具级物理模型（无状态，极简）。

    忽略牵引曲线、黏着、阻力、质量，只为让列车"能动、能停"。
    加速度 = throttle × a_max - brake × b_max，速度限制由调用方处理。

    参数:
        a_max: 最大加速度（m/s²），默认 1.0
        b_max: 最大制动减速度（m/s²），默认 2.0
        v_max: 最高速度（m/s），默认 30（~108 km/h），用于调用方 clamp
    """

    def __init__(
        self,
        a_max: float = 1.0,
        b_max: float = 2.0,
        v_max: float = 30.0,
    ) -> None:
        self.a_max = a_max
        self.b_max = b_max
        self.v_max = v_max

    def compute_acceleration(
        self,
        v: float,
        throttle: float,
        brake: float,
        track_metadata: dict | None = None,
    ) -> float:
        # Clamp 输入
        throttle = max(0.0, min(throttle, 1.0))
        brake = max(0.0, min(brake, 1.0))

        # 简单线性模型
        a = throttle * self.a_max - brake * self.b_max

        # 速度上限：超过 v_max 时强制不加速（调用方也会 clamp v，这是双保险）
        if v >= self.v_max and a > 0:
            a = 0.0

        return a

