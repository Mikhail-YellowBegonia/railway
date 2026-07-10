from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class GeometricRole(Enum):
    """转向架的几何角色（用于确定弧长/割线求解的基准点）。"""
    LEADING = "leading"    # 前转向架（geometric_role = 1）
    TRAILING = "trailing"  # 后转向架（geometric_role = 2）


@dataclass
class BogieConfig:
    """转向架配置（车厢的子结构）。

    几何约束：两个转向架间的割线距离固定。
    物理属性：质量分配、轴数（后续真实物理用）。
    """
    geometric_role: GeometricRole  # 几何角色（前/后转向架）
    pos: float                      # 转向架相对车厢前车钩的位置（米，沿车厢纵向）
    load_share: float = 0.5         # 质量分配比例（0-1，两转向架之和应为 1）
    axle_count: int = 2             # 轴数（默认 2，后续物理用）
    # bogie_property: 预留，后续真实物理扩展（轴距、悬挂刚度等）


@dataclass
class WagonConfig:
    """车厢配置（刚体车厢模型的元数据）。

    定义一节车厢的几何、质量、动力属性。是真实物理层的输入。
    """
    length: float                   # 车厢总长（米，车钩到车钩）
    mass: float                     # 质量（吨）
    bogies: list[BogieConfig] = field(default_factory=list)  # 转向架列表（通常 2 个）
    coupler_1_pos: float = 0.0      # 前车钩位置（米，逻辑原点，通常 0）
    coupler_2_pos: float = 0.0      # 后车钩位置（米，= length 或略小）
    # engine_property: 预留，后续真实物理扩展（动力类型、功率曲线）

    def __post_init__(self):
        """自动填充 coupler_2_pos = length（如果未显式设置）。"""
        if self.coupler_2_pos == 0.0:
            self.coupler_2_pos = self.length

    @property
    def bogie_spacing(self) -> float:
        """两转向架间的割线距离（米）——刚体约束的核心参数。

        假设恰好两个转向架，返回它们位置差的绝对值。
        """
        if len(self.bogies) != 2:
            raise ValueError(f"bogie_spacing 需要恰好 2 个转向架，当前 {len(self.bogies)} 个")
        return abs(self.bogies[1].pos - self.bogies[0].pos)


@dataclass
class Consist:
    """列车编组（多节车厢的集合 + 全局运动状态）。

    管理多节车厢的串联、速度、加速度等全局属性。
    单节车厢是特例（wagons 只有一个元素）。
    """
    wagons: list[WagonConfig]       # 车厢列表（按车头到车尾顺序）
    velocity: float = 0.0           # 全车共享速度（m/s，质心或前转向架）
    acceleration: float = 0.0       # 全车共享加速度（m/s²）

    @property
    def total_mass(self) -> float:
        """编组总质量（吨）。"""
        return sum(w.mass for w in self.wagons)

    @property
    def total_length(self) -> float:
        """编组总长（米，所有车厢长度之和，暂不考虑车钩间隙）。"""
        return sum(w.length for w in self.wagons)


# ===== D2: 弧长/割线求解器 =====

def solve_rear_bogie_s(
    front_bogie_s: float,
    bogie_spacing: float,
    path_kinematics,  # model.kinematics.PathKinematics
) -> float:
    """根据前转向架弧长和固定割线距离，求解后转向架弧长（泰勒展开）。

    使用一阶泰勒展开近似: Δs = l + l³/(24R²)
    其中 l = bogie_spacing（割线距离），R = 轨道曲率半径。

    参数:
        front_bogie_s: 前转向架在 Path 上的弧长（米）
        bogie_spacing: 两转向架间固定割线距离（米）
        path_kinematics: 路径运动学对象，用于查询曲率

    返回:
        后转向架弧长（米）

    注意:
    - 后转向架在前转向架**后方**（s 更小），所以返回值 < front_bogie_s
    - 直线段: R → ∞, Δs ≈ l（退化为质点模型）
    - 弯道: Δs > l（外侧 Bogie 走得更远）
    """
    l = bogie_spacing

    # 查询前转向架所在位置的曲率半径（简化：用前转向架位置的局部曲率）
    # 更精确的做法是积分整段路径的曲率，但一阶近似下用单点曲率足够
    R = _estimate_curvature_radius(front_bogie_s, path_kinematics)

    # 泰勒展开修正
    if R > 1e6:  # 直线（曲率半径极大）
        delta_s = l
    else:
        delta_s = l + (l ** 3) / (24 * R ** 2)

    # 后转向架在前方向后（s 减小）
    rear_bogie_s = front_bogie_s - delta_s
    return max(0.0, rear_bogie_s)  # clamp 到路径起点


def _estimate_curvature_radius(
    s: float,
    path_kinematics,  # model.kinematics.PathKinematics
) -> float:
    """估算路径在弧长 s 处的曲率半径（米）。

    方法:
    - 直线段: 返回无穷大（1e9）
    - 圆弧段: 返回 edge.arc_radius

    简化假设: 用 s 所在 Edge 的曲率代表该点曲率（忽略跨 Edge 过渡）。
    """
    # 定位 s 落在哪个 segment（有向边）
    for i, (directed, s_start, seg_length) in enumerate(path_kinematics._segments):
        if s < s_start + seg_length or i == len(path_kinematics._segments) - 1:
            edge_id, direction = directed
            edge = path_kinematics.network.edges[edge_id]
            if edge.is_arc:
                return edge.arc_radius
            else:
                return 1e9  # 直线，曲率半径无穷大
    return 1e9  # fallback


# ===== 预定义配置（示例/测试用） =====

def create_simple_wagon(length: float = 20.0, mass: float = 50.0) -> WagonConfig:
    """创建一个简单的标准车厢（两转向架，对称布局）。

    参数:
        length: 车厢长度（米），默认 20m
        mass: 质量（吨），默认 50 吨

    转向架布局：前后各占 1/8 长度处（即两转向架间距 = 3/4 长度）。
    """
    bogie_spacing = length * 0.75
    bogie_1_pos = length * 0.125
    bogie_2_pos = bogie_1_pos + bogie_spacing

    return WagonConfig(
        length=length,
        mass=mass,
        bogies=[
            BogieConfig(geometric_role=GeometricRole.LEADING, pos=bogie_1_pos, load_share=0.5),
            BogieConfig(geometric_role=GeometricRole.TRAILING, pos=bogie_2_pos, load_share=0.5),
        ],
    )
