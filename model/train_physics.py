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
        consist,  # model.wagon.Consist
        track_metadata: dict | None = None,
    ) -> float:
        """计算瞬时加速度（m/s²）。

        参数:
            v: 当前速度（m/s）
            throttle: 油门/功率手柄（∈ [0, 1]，0=怠速，1=全开）
            brake: 制动（∈ [0, 1]，0=无制动，1=全制动）
            consist: 列车编组（用于获取 total_mass 等元数据）
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
        consist,  # model.wagon.Consist
        track_metadata: dict | None = None,
    ) -> float:
        throttle = max(0.0, min(throttle, 1.0))
        brake = max(0.0, min(brake, 1.0))

        a = throttle * self.a_max - brake * self.b_max

        if v >= self.v_max and a > 0:
            a = 0.0

        return a


class RealisticElectric(TrainPhysics):
    """真实电力机车物理模型（阶段 2：Wagon 级别计算）。

    特性：
    - 遍历 consist.wagons，区分动力车/拖车
    - 动力车：贡献牵引力（按各自 P_rated 计算）
    - 拖车：只贡献质量和阻力
    - 牵引曲线：恒转矩区 + 恒功率区
    - 阻力：Davis 公式（按总质量）
    - 黏着限制：整体编组（暂不细化到 Bogie）

    简化（后续阶段扩展）：
    - 黏着限制用总质量（不区分各车厢轴重）
    - 忽略坡度/曲线阻力
    - 所有动力车共享 throttle（后续可独立控制）

    参数:
        F_max_const: 单节动力车恒转矩区最大牵引力（kN），默认 300
        v_transition: 恒转矩/恒功率转换速度（m/s），默认 10
        mu_adhesion: 黏着系数，默认 0.3
        brake_force: 最大制动力（kN），默认 200
        davis_A/B/C: Davis 阻力公式系数
    """

    def __init__(
        self,
        F_max_const: float = 300.0,
        v_transition: float = 10.0,
        mu_adhesion: float = 0.3,
        brake_force: float = 200.0,
        davis_A: float = 2.0,
        davis_B: float = 0.01,
        davis_C: float = 0.0004,
    ) -> None:
        self.F_max_const = F_max_const
        self.v_transition = v_transition
        self.mu_adhesion = mu_adhesion
        self.brake_force = brake_force
        self.davis_A = davis_A
        self.davis_B = davis_B
        self.davis_C = davis_C

    def compute_acceleration(
        self,
        v: float,
        throttle: float,
        brake: float,
        consist,  # model.wagon.Consist
        track_metadata: dict | None = None,
    ) -> float:
        throttle = max(0.0, min(throttle, 1.0))
        brake = max(0.0, min(brake, 1.0))

        m_total = consist.total_mass  # 吨

        # 遍历车厢，累加动力车的牵引力
        F_traction_total = 0.0
        for wagon in consist.wagons:
            if wagon.is_powered:
                P_rated = wagon.P_rated  # kW
                if v < self.v_transition:
                    # 恒转矩区
                    F_wagon = throttle * self.F_max_const
                else:
                    # 恒功率区
                    F_wagon = throttle * P_rated / v if v > 0 else 0
                F_traction_total += F_wagon

        # 黏着限制（整体编组）
        F_adhesion = self.mu_adhesion * m_total * 1000 * 9.81 / 1000  # kN
        F_traction_total = min(F_traction_total, F_adhesion)

        # 阻力（Davis 公式，按总质量）
        R = self.davis_A + self.davis_B * v + self.davis_C * v * v

        # 制动力
        F_brake = brake * self.brake_force

        # 净力
        F_net = F_traction_total - R - F_brake

        # 加速度
        a = F_net / m_total

        return a


# 默认物理模型实例（2026-09-10，惰性落地 T2-4）。
#
# 为什么可以全列车共享一个实例：`compute_acceleration` 是**纯函数**，不写任何
# 字段（本模块两个实现都只存配置常量），所以"谁的 physics"没有意义。共享之后，
# 审计 B2 的那类问题（解挂两段共享父的 physics、连挂取**前车**的 physics 而把
# 后车的静默丢弃）**从结构上消失**——不存在"交接"，因为本来就没有身份。
#
# 注意：物理**参数**（质量/功率/黏着…）属于**车厢（A 桶）**，由 `Consist` 汇总后
# 传入；这里共享的只是公式集。若将来不同机车需要不同参数，应改为"参数来自车厢"
# 而不是"每列车一个实例"。
DEFAULT_PHYSICS = RealisticElectric()


