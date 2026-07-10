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
