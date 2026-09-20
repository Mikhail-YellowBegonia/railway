"""车厢（Car）与编组（Consist）的数据模型。

## 归属规则（2026-09-10 定案，完整研讨见 docs/wagon_centric_data.md）

**加新字段前先问一句**：它是**域数据**（描述"这节车厢是什么"）还是**运行期派生**
（描述"此刻这列编组怎么跑"）？

- **A 车厢域数据**：物理属性（长度/质量/功率/转向架/车钩位置）、载货量、
  **调度计划** → **归车厢所有**，随车厢走（解挂带走、连挂随车、逐车厢落盘）。
  **只读**——列车/编组不得写入（唯一例外：载货量等状态由车厢**自力更新**）。
- **B 编组运行期派生**：速度、位置窗口（occupancy）、controller、运动授权、
  由选中计划投影出的 route/goal → **归编组持有**，可随时重算；
  **禁止被当作域数据长期保存或跨编组交割**。
- **C 基础设施侧**（POI、信号等）：挂在 node/edge 上、随轨道落盘——不在本文件。

**车厢对象在解挂/连挂后是同一批对象（别名共享）**：这是**正确的同一性**而不是
隐患——同一节车厢当然只有一份状态。前提是 A 桶的物理属性**只读**、可变状态
（载货量）本来就该随车厢共享。实测证据（就地改 `mass` 会串味到父列车）见
`docs/roadmap.md` #9-B1。
"""
from __future__ import annotations

import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING

from model.wagon_data import WagonDataLog

if TYPE_CHECKING:
    from model.plan import Plan


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
    """车厢**配置**（A 桶·只读：描述"这节车厢是什么"）。

    定义一节车厢的几何、质量、动力属性与身份 —— 是真实物理层的输入。
    **列车/编组不得写入本对象的字段**（加新字段前先确认它属于 A 桶）；
    会变的状态（`data_log`、未来的载货/计划）不放这里，放 `Wagon`。

    2026-09-10 拍板（`docs/wagon_centric_data.md` §9.1）：**配置与运行时状态分家**——
    `WagonConfig`（本类，只读）+ `Wagon`（运行时对象，带 `tick`）。用"分家"代替
    "用语言机制强制只读"（Q3："只读是设计准则，不是强制逻辑"）。
    """
    length: float                   # [A·物理] 车厢总长（米，车钩到车钩）
    mass: float                     # [A·物理] 空载质量（吨）；载货量另计（未来）
    bogies: list[BogieConfig] = field(default_factory=list)  # [A·物理] 转向架（通常 2 个）
    coupler_1_pos: float = 0.0      # [A·物理] 前车钩位置（米，逻辑原点，通常 0）
    coupler_2_pos: float = 0.0      # [A·物理] 后车钩位置（米，= length 或略小）
    P_rated: float | None = None    # [A·物理] 额定功率（kW），None = 无动力。
    # 注意（2026-09-10 定，Q2）：**动力只是物理参数**（与 mass 同级，无足轻重），
    # 它不回答"这节车厢是不是机车"——那是 have_control 的事。
    wagon_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    # [A·身份] 车厢身份，与物理车厢 1:1 绑定，供按 wagon_id 挂靠车厢级数据。
    # 复制/新建车厢时必须重新生成；**同一节物理车厢的等价副本必须原样传递**，
    # 否则外部按 wagon_id 挂靠的状态（货物/损耗/计划）都会失联。
    have_control: bool = False
    # [A·决定性] 控制车标志：决定"这节车厢能不能持有并执行调度计划"。
    # 2026-09-10 定（Q2）：控制是**决定性属性**（与身份同级），动力只是物理参数。
    # 四种组合都合法：控制+动力=机车 / 控制无动力=驾驶拖车 / 有动力无控制=补机 /
    # 都无=普通车厢。**当前无人读取本字段**（惰性落地 T2-1），消费点（渲染、
    # 解挂提示、计划遴选）待计划层定稿后再接。
    priority: int = 0
    # [A·决定性] 多控制车冲突时的遴选优先级（**不查重**，允许重复）；
    # 相同优先级用 wagon_id 取**小**者胜（Q4 最简解法 + Q5 定方向）。
    # 同样**当前无人读取**。

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
        """是否有牵引动力（`P_rated` 非 None）。

        ⚠ **这不是"是不是机车"**（2026-09-10 定，Q2）：控制属性见 `have_control`。
        本 property 只回答物理问题，保留给牵引计算使用；渲染/提示等"语义"用途
        应改判为 `have_control`（见 docs/wagon_centric_data.md §6.2）。
        """
        return self.P_rated is not None


@dataclass(eq=False)
class Wagon:
    """车厢**运行时对象**（A 桶：描述"这节车厢此刻有什么状态"，并有自己的 tick）。

    2026-09-10 拍板拆分（`docs/wagon_centric_data.md` §9.1）：**配置与状态分家**——
    - `WagonConfig` = **只读配置**（几何/质量/功率/身份/控制车标志）→ `self.config`
    - `Wagon` = **运行时对象**：持有会变的状态（当前只有 `data_log`；未来：载货、
      调度计划与指令指针），并提供 `tick()`。
    **只读属性一律委托给 `config`**，所以读代码（运动学/物理/渲染/存档）不需要区分两者。
    身份语义同 `TrainEntity`：**按对象身份**（`eq=False`），编组变化时是**同一批对象**
    （这是正确的同一性，见模块顶部归属规则）。
    """
    config: WagonConfig
    data_log: WagonDataLog = field(default_factory=WagonDataLog)
    plan: "Plan | None" = None
    orientation: int = 1
    # 物理前端（config 的 coupler_1 一侧）相对编组逻辑前进方向。
    # +1 = 同向，-1 = 反向；连挂/解挂时随车厢对象保留。
    # [A·状态] 本车厢自己的数据包（未来承载调度命令/状态标记；近期无生产者）。
    # 2026-09-10 从 `Consist` 迁入、再从 `WagonConfig` 移到本类（T2-2 → 本次拆分）：
    # 数据包和计划**随车厢走**，解挂/连挂不需要任何归并/拆分记账（见 Q9）。

    # ------------------------------------------------------------------
    # 只读委托：配置字段（读方不必知道 Wagon / WagonConfig 的分工）
    # ------------------------------------------------------------------
    @property
    def wagon_id(self) -> str:
        return self.config.wagon_id

    @property
    def length(self) -> float:
        return self.config.length

    @property
    def mass(self) -> float:
        return self.config.mass

    @property
    def bogies(self) -> list[BogieConfig]:
        return self.config.bogies

    @property
    def coupler_1_pos(self) -> float:
        return self.config.coupler_1_pos

    @property
    def coupler_2_pos(self) -> float:
        return self.config.coupler_2_pos

    @property
    def P_rated(self) -> float | None:
        return self.config.P_rated

    @property
    def have_control(self) -> bool:
        return self.config.have_control

    @property
    def priority(self) -> int:
        return self.config.priority

    @property
    def is_powered(self) -> bool:
        return self.config.is_powered

    @property
    def bogie_spacing(self) -> float:
        return self.config.bogie_spacing

    def __post_init__(self) -> None:
        if self.orientation not in (1, -1):
            raise ValueError(f"Wagon.orientation={self.orientation} 非法（应为 ±1）")

    @property
    def logical_front_bogie_pos(self) -> float:
        if len(self.bogies) != 2:
            raise ValueError(f"车厢需要恰好 2 个转向架，当前 {len(self.bogies)} 个")
        return self.bogies[0].pos if self.orientation > 0 else self.length - self.bogies[1].pos

    @property
    def logical_rear_bogie_pos(self) -> float:
        if len(self.bogies) != 2:
            raise ValueError(f"车厢需要恰好 2 个转向架，当前 {len(self.bogies)} 个")
        return self.bogies[1].pos if self.orientation > 0 else self.length - self.bogies[0].pos

    @property
    def logical_front_coupler_pos(self) -> float:
        return self.coupler_1_pos if self.orientation > 0 else self.length - self.coupler_2_pos

    @property
    def logical_rear_coupler_pos(self) -> float:
        return self.coupler_2_pos if self.orientation > 0 else self.length - self.coupler_1_pos

    def reverse_relative_to_consist(self) -> None:
        """编组逻辑方向翻转时保持本车物理朝向不变。"""
        self.orientation *= -1

    # ------------------------------------------------------------------
    # 每帧更新钩子（车厢 tick）
    # ------------------------------------------------------------------
    def tick(self, dt: float) -> None:
        """**车厢自己的每帧更新钩子**（当前是空实现、**未接线**）。

        调用约定（2026-09-10 定，见 `docs/wagon_centric_data.md` §9.1）：
        - **由所属编组在自身 update 里遍历自己的车厢**调用
          （`for w in consist.wagons: w.tick(dt)`）——满足"外部只能调用车厢方法"（Q3），
          不需要全局注册表，且顺序天然确定（车头→车尾，顺带解决"轮询顺序"问题）。
        - **物理不得下放**：位置真值仍是编组的单一 `TrainState.s`（`RigidWagonKinematics`
          链式解算）。车厢 tick 只做**逻辑/服务**类更新（载货、状态自更新等）。
        - **计划指针的推进不在这里**：它是"到达事件"由列车**直接调用控制车的方法**
          （Q14），而不是每帧 tick 自己往前走。
        """
        return None

    def set_control_config(
        self,
        *,
        have_control: bool | None = None,
        priority: int | None = None,
    ) -> None:
        """在编组停放时由配置界面调用的最小控制车配置入口。"""
        if have_control is not None:
            self.config.have_control = have_control
        if priority is not None:
            self.config.priority = priority


@dataclass
class Consist:
    """列车编组（B 桶：只汇总 + 成员引用；**不存任何域数据**，见模块顶部归属规则）。

    管理多节车厢的串联。单节车厢是特例（wagons 只有一个元素）。

    - 允许：**只读汇总**（`total_mass` / `total_length` 等 property）、
      成员顺序（"此刻由哪些车厢按什么顺序组成"，随位置快照落盘）。
    - 禁止：把 A 桶域数据（物理属性副本 / 载货 / 计划）缓存在这里，
      或让编组级字段承担"跨编组交割"的语义——解挂/连挂时它们会被重建。
    """
    wagons: list[Wagon]             # 成员与顺序（身份指向 A 桶；不是数据副本）

    def __post_init__(self) -> None:
        """允许传入裸 `WagonConfig` 或 `Wagon`——裸配置自动包成运行时对象。

        这样既有的构造点（`Consist(wagons=[create_simple_wagon(...)])`）不用改，
        而编组内部**统一**持有 `Wagon`（车厢 tick 的宿主）。
        """
        self.wagons = [w if isinstance(w, Wagon) else Wagon(w) for w in self.wagons]

    @property
    def total_mass(self) -> float:
        """编组总质量（吨）。"""
        return sum(w.mass for w in self.wagons)

    def control_cars(self) -> list[Wagon]:
        """本编组里的**控制车**（A 桶 `have_control`）——未来"遴选计划"的输入面。

        2026-09-10 预留（惰性落地 T2-3）：**当前无调用者**。计划层定稿后，
        编组按"控制车 `priority` 大者胜 → `wagon_id` **小**者胜"遴选要执行哪节
        控制车的计划（Q4/Q5/Q6）。
        """
        return [w for w in self.wagons if w.have_control]

    def control_winner(self) -> Wagon | None:
        """按已定规则选出当前执行计划的控制车。"""
        return min(
            self.control_cars(),
            key=lambda wagon: (-wagon.priority, wagon.wagon_id),
            default=None,
        )

    @property
    def total_length(self) -> float:
        """编组总长（米，所有车厢长度之和，暂不考虑车钩间隙）。"""
        return sum(w.length for w in self.wagons)

    def split_at(self, wagon_idx: int) -> tuple["Consist", "Consist"]:
        """按 wagon_idx 拆分编组（0-indexed，前段含 wagons[0..wagon_idx]）。

        只切**成员列表**：车厢级数据（A 桶，含各自的 `data_log`）随车厢对象
        自然归属，不需要任何拆分记账——这是 data_log 迁入车厢后（T2-2）的直接
        简化，原来的 `wagon_id` 归属分配逻辑已整体删除。
        几何切分见 `TrainEntity.decouple_at`。
        """
        front_wagons = self.wagons[:wagon_idx + 1]
        rear_wagons = self.wagons[wagon_idx + 1:]
        return Consist(wagons=front_wagons), Consist(wagons=rear_wagons)

    def merged_with(self, rear: "Consist") -> "Consist":
        """将 rear 编组连挂到本编组车尾，返回合并后的新编组。

        只拼**成员列表**：两侧车厢各自的域数据（含 `data_log`）随车厢对象保留，
        不再需要归并记账（T2-2）。
        """
        return Consist(wagons=self.wagons + rear.wagons)

    def reverse_logical_direction(self) -> None:
        """翻转编组逻辑首尾，同时保持每节车厢的物理朝向。"""
        self.wagons.reverse()
        for wagon in self.wagons:
            wagon.reverse_relative_to_consist()


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
    have_control: bool | None = None,
    priority: int = 0,
) -> WagonConfig:
    """创建一个简单的标准车厢（两转向架，对称布局）。

    参数:
        length: 车厢长度（米），默认 20m
        mass: 质量（吨），默认 50 吨
        P_rated: 额定功率（kW），None = 无动力，非 None = 有动力
        have_control: 是否控制车。**None = 暂按 `P_rated is not None` 推导**
            （2026-09-10 惰性落地 T2-1 的临时策略：当前无消费点，故不影响行为；
            四种控制/动力组合最终如何由工厂默认产出，待计划层定稿 = Q2 细节）
        priority: 控制车遴选优先级（**不查重**；当前无消费点）

    转向架布局：前后各占 1/8 长度处（即两转向架间距 = 3/4 长度）。
    """
    bogie_spacing = length * 0.75
    bogie_1_pos = length * 0.125
    bogie_2_pos = bogie_1_pos + bogie_spacing

    if have_control is None:
        have_control = P_rated is not None

    return WagonConfig(
        length=length,
        mass=mass,
        P_rated=P_rated,
        have_control=have_control,
        priority=priority,
        bogies=[
            BogieConfig(geometric_role=GeometricRole.LEADING, pos=bogie_1_pos, load_share=0.5),
            BogieConfig(geometric_role=GeometricRole.TRAILING, pos=bogie_2_pos, load_share=0.5),
        ],
    )


def create_simple_car(
    length: float = 20.0,
    mass: float = 50.0,
    P_rated: float | None = None,
    have_control: bool | None = None,
    priority: int = 0,
) -> Wagon:
    """便捷工厂：**配置 + 运行时对象**一步到位（= `Wagon(create_simple_wagon(...))`）。

    参数含义同 `create_simple_wagon`。编组会自动把裸配置包成 `Wagon`，
    所以两种工厂都能用；这个入口给"需要车厢运行时对象（tick/状态）"的调用方。
    """
    return Wagon(create_simple_wagon(
        length=length, mass=mass, P_rated=P_rated,
        have_control=have_control, priority=priority,
    ))
