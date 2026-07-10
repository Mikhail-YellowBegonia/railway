from __future__ import annotations

from abc import ABC, abstractmethod


class TrainPhysics(ABC):
    """列车物理层抽象接口（可插拔实现）。

    所有物理模型共享此接口，支持从玩具级到真实级的渐进扩展。
    调用方（GameLoop）通过统一接口更新状态、查询位置，无感知切换实现。
    """

    @abstractmethod
    def update(self, dt: float) -> None:
        """每帧物理更新（dt 单位：秒）。

        根据当前 throttle/brake 输入和物理模型，更新内部状态（s, v, a）。
        """
        pass

    @abstractmethod
    def apply_throttle(self, value: float) -> None:
        """设置油门/功率手柄（value ∈ [0, 1]，0=怠速，1=全开）。"""
        pass

    @abstractmethod
    def apply_brake(self, value: float) -> None:
        """设置制动（value ∈ [0, 1]，0=无制动，1=全制动）。"""
        pass

    @abstractmethod
    def get_state(self) -> tuple[float, float, float]:
        """返回当前状态 (弧长 s, 速度 v, 加速度 a)。

        - s: 米
        - v: m/s
        - a: m/s²
        """
        pass

    @abstractmethod
    def reset(self, initial_s: float) -> None:
        """重置到起点（弧长 initial_s），速度/加速度归零。"""
        pass


class SimplePhysics(TrainPhysics):
    """玩具级物理模型（第一版，极简）。

    忽略牵引曲线、黏着、阻力、质量，只为让列车"能动、能停"。
    加速度 = throttle × a_max - brake × b_max，速度限制在 [0, v_max]。

    参数:
        path_length: 路径总长度（米），用于限制 s 不越界
        a_max: 最大加速度（m/s²），默认 1.0
        b_max: 最大制动减速度（m/s²），默认 2.0
        v_max: 最高速度（m/s），默认 30（~108 km/h）
    """

    def __init__(
        self,
        path_length: float,
        initial_s: float = 0.0,
        a_max: float = 1.0,
        b_max: float = 2.0,
        v_max: float = 30.0,
    ) -> None:
        self.path_length = path_length
        self.a_max = a_max
        self.b_max = b_max
        self.v_max = v_max

        self.s = initial_s
        self.v = 0.0
        self.a = 0.0

        self._throttle = 0.0
        self._brake = 0.0

    def update(self, dt: float) -> None:
        # 计算加速度
        self.a = self._throttle * self.a_max - self._brake * self.b_max

        # 更新速度（限制在 [0, v_max]）
        self.v += self.a * dt
        self.v = max(0.0, min(self.v, self.v_max))

        # 更新弧长（限制在 [0, path_length]）
        self.s += self.v * dt
        self.s = max(0.0, min(self.s, self.path_length))

        # 到达终点自动停车
        if self.s >= self.path_length:
            self.v = 0.0
            self.a = 0.0

    def apply_throttle(self, value: float) -> None:
        self._throttle = max(0.0, min(value, 1.0))

    def apply_brake(self, value: float) -> None:
        self._brake = max(0.0, min(value, 1.0))

    def get_state(self) -> tuple[float, float, float]:
        return (self.s, self.v, self.a)

    def reset(self, initial_s: float) -> None:
        self.s = initial_s
        self.v = 0.0
        self.a = 0.0
        self._throttle = 0.0
        self._brake = 0.0
