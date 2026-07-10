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
    P_rated: float | None = None    # 额定功率（kW），None = 拖车，非 None = 动力车

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

    @property
    def is_powered(self) -> bool:
        """是否为动力车（有额定功率）。"""
        return self.P_rated is not None


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

    跨 Edge 边界处理:
    - 同 Edge: 用单点曲率（快速）
    - 跨 Edge: 用加权平均曲率（避免抖动）

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

    # 粗略估算后转向架位置（用于判断是否跨 Edge）
    rear_s_rough = max(0.0, front_bogie_s - l * 1.1)  # 稍微多留余量

    # 定位前后转向架所在 segment
    front_seg_idx = path_kinematics._locate_segment(front_bogie_s)
    rear_seg_idx = path_kinematics._locate_segment(rear_s_rough)

    # 跨 Edge 判断
    if front_seg_idx == rear_seg_idx:
        # 同一 Edge，用单点曲率（原有逻辑，性能最优）
        R = _estimate_curvature_radius(front_bogie_s, path_kinematics)
    else:
        # 跨 Edge，迭代求精曲率（两次计算：粗估 → 加权曲率 → 精算）
        # 第一次：用粗估 s2 的单点曲率
        R_rough = _estimate_curvature_radius(rear_s_rough, path_kinematics)
        if R_rough > 1e6:
            delta_s_rough = l
        else:
            delta_s_rough = l + (l ** 3) / (24 * R_rough ** 2)
        rear_s_iter1 = max(0.0, front_bogie_s - delta_s_rough)

        # 第二次：用 [rear_s_iter1, front_s] 的加权平均曲率
        rear_seg_iter1 = path_kinematics._locate_segment(rear_s_iter1)
        R = _weighted_average_curvature(
            front_bogie_s, rear_s_iter1, front_seg_idx, rear_seg_iter1, path_kinematics
        )

    # 泰勒展开修正
    if R > 1e6:  # 直线（曲率半径极大）
        delta_s = l
    else:
        delta_s = l + (l ** 3) / (24 * R ** 2)

    # 后转向架在前方向后（s 减小）
    rear_bogie_s = front_bogie_s - delta_s
    return max(0.0, rear_bogie_s)  # clamp 到路径起点


def _weighted_average_curvature(
    s1: float,
    s2: float,
    seg_idx1: int,
    seg_idx2: int,
    path_kinematics,  # model.kinematics.PathKinematics
) -> float:
    """计算 [s2, s1] 区间的加权平均曲率半径（跨 Edge 补丁）。

    按每段 Edge 在区间内的长度占比加权平均 1/R，然后取倒数得 R_avg。
    """
    segments = path_kinematics._segments
    total_length = 0.0
    weighted_curvature = 0.0  # 加权平均 κ = 1/R

    # 从后到前遍历涉及的 segments（s2 → s1）
    for i in range(seg_idx2, seg_idx1 + 1):
        if i >= len(segments):
            break
        directed, s_start, seg_length = segments[i]
        edge_id, direction = directed
        edge = path_kinematics.network.edges[edge_id]

        # 计算该 segment 在 [s2, s1] 区间的重叠长度
        seg_end = s_start + seg_length
        overlap_start = max(s2, s_start)
        overlap_end = min(s1, seg_end)
        overlap_length = max(0.0, overlap_end - overlap_start)

        if overlap_length > 0:
            total_length += overlap_length
            # 曲率 κ = 1/R（直线 κ=0）
            if edge.is_arc:
                curvature = 1.0 / edge.arc_radius
            else:
                curvature = 0.0  # 直线
            weighted_curvature += curvature * overlap_length

    # 加权平均曲率
    if total_length < 1e-6:
        return 1e9  # fallback：极短距离，当直线
    avg_curvature = weighted_curvature / total_length
    if avg_curvature < 1e-9:
        return 1e9  # 平均曲率接近 0，当直线
    return 1.0 / avg_curvature  # R_avg = 1 / κ_avg


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

def create_simple_wagon(
    length: float = 20.0,
    mass: float = 50.0,
    P_rated: float | None = None,
) -> WagonConfig:
    """创建一个简单的标准车厢（两转向架，对称布局）。

    参数:
        length: 车厢长度（米），默认 20m
        mass: 质量（吨），默认 50 吨
        P_rated: 额定功率（kW），None = 拖车，非 None = 动力车

    转向架布局：前后各占 1/8 长度处（即两转向架间距 = 3/4 长度）。
    """
    bogie_spacing = length * 0.75
    bogie_1_pos = length * 0.125
    bogie_2_pos = bogie_1_pos + bogie_spacing

    return WagonConfig(
        length=length,
        mass=mass,
        P_rated=P_rated,
        bogies=[
            BogieConfig(geometric_role=GeometricRole.LEADING, pos=bogie_1_pos, load_share=0.5),
            BogieConfig(geometric_role=GeometricRole.TRAILING, pos=bogie_2_pos, load_share=0.5),
        ],
    )
