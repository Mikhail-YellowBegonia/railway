# 列车物理层设计

## 设计原则

物理层采用**接口抽象 + 可插拔实现**架构，支持从玩具级到真实级的渐进式扩展：
- `SimplePhysics`：极简模型（已实现）
- `RealisticElectric`：真实电力机车（已实现，阶段 1/2）
- 后续可扩展：`RealisticDiesel` / `RealisticSteam` 等

所有实现共享统一接口，调用方（GameLoop）无感知切换。

## 编组整体结算原则（重要）

**物理层必须对整个 Consist 整体计算加速度**，因为刚体约束决定了所有转向架共享 `v` 和 `a`。

**渐进式实现路径**：
- ✅ **阶段 1**：整体近似，不遍历 Wagon/Bogie，用 `total_mass` + 等效功率
- ✅ **阶段 2**：Wagon 级别，区分动力车/拖车（当前实现）
- ⏸️ **阶段 3**：Bogie 级别，独立计算黏着限制和功率分配（推迟，复杂度高/收益低）

**行业标准**：OpenTTD / Transport Fever 等成熟游戏均停在阶段 2 水平，已足够拟真。

## TrainPhysics 接口（抽象基类）

```python
class TrainPhysics(ABC):
    @abstractmethod
    def compute_acceleration(
        v: float,
        throttle: float,
        brake: float,
        consist: Consist,
        track_metadata: dict | None = None,
    ) -> float:
        """计算瞬时加速度（m/s²）。
        
        物理层无状态（纯函数），状态积分由调用方负责。
        """
```

**参数说明**：
- `v`: 当前速度（m/s）
- `throttle`: 油门 ∈ [0, 1]（0=怠速，1=全开）
- `brake`: 制动 ∈ [0, 1]（0=无制动，1=全制动）
- `consist`: 列车编组（包含 `total_mass`、各车厢 `P_rated` 等元数据）
- `track_metadata`: 轨道元数据（可选），如 `{'grade': float, 'curve_radius': float}`

**返回**：加速度（m/s²），可正（加速）可负（减速）

## SimplePhysics（玩具级）

**目标**：验证接口和交互流程，忽略一切真实复杂性。

**物理模型**：
- 加速度 `a = throttle × a_max - brake × b_max`
  - `a_max = 1.0 m/s²`（恒定，无牵引曲线）
  - `b_max = 2.0 m/s²`（恒定）
- 忽略质量、阻力、黏着、牵引曲线

**用途**：早期原型测试，已被 `RealisticElectric` 替代。

## RealisticElectric（真实电力机车）

**当前实现**：阶段 2（Wagon 级别细化）

**特性**：
1. **牵引曲线**：
   - 恒转矩区（v < 10 m/s）：`F = F_max_const`（默认 300 kN/节）
   - 恒功率区（v ≥ 10 m/s）：`F = P_rated / v`
   
2. **动力车/拖车区分**：
   - 遍历 `consist.wagons`，检查 `wagon.P_rated`
   - `P_rated != None` → 动力车，贡献牵引力
   - `P_rated == None` → 拖车，只贡献质量和阻力
   
3. **阻力**：Davis 公式 `R = A + Bv + Cv²`（按总质量）
   - 默认系数：`A=2.0, B=0.01, C=0.0004`
   
4. **黏着限制**：`F_traction ≤ μ × m_total × g`
   - 黏着系数 `μ = 0.3`（干燥钢轨）
   
5. **制动**：空气制动，恒定减速度（默认 200 kN）

**参数**：
```python
RealisticElectric(
    F_max_const=300.0,   # 单节动力车恒转矩区最大牵引力（kN）
    v_transition=10.0,   # 恒转矩/恒功率转换速度（m/s）
    mu_adhesion=0.3,     # 黏着系数
    brake_force=200.0,   # 最大制动力（kN）
    davis_A=2.0,         # Davis 公式系数
    davis_B=0.01,
    davis_C=0.0004,
)
```

**典型表现**：
- 1 动力车（3000 kW）+ 2 拖车（总质量 155 吨）：
  - 低速（0-10 m/s）：加速度 ~1.9 m/s²
  - 高速（30 m/s）：加速度 ~0.6 m/s²（恒功率特性）
- 3 动力车（9000 kW）+ 0 拖车（总质量 155 吨）：
  - 低速：加速度 ~2.9 m/s²
  - 高速：加速度 ~1.9 m/s²

**简化点**（后续横向扩展）：
- ❌ 坡度阻力（`track_metadata['grade']` 暂未使用）
- ❌ 曲线限速（`track_metadata['curve_radius']` 暂未使用）
- ❌ Bogie 级别黏着（用整体编组黏着）
- ❌ 再生制动（只有空气制动）

## WagonConfig 元数据

```python
@dataclass
class WagonConfig:
    length: float               # 车厢长度（米，车钩到车钩）
    mass: float                 # 质量（吨）
    bogies: list[BogieConfig]   # 转向架列表（通常 2 个）
    P_rated: float | None       # 额定功率（kW），None = 拖车
    
    @property
    def is_powered(self) -> bool:
        """是否为动力车。"""
        return self.P_rated is not None
```

**创建车厢**：
```python
# 动力车（3000 kW 电力机车）
locomotive = create_simple_wagon(length=20.0, mass=50.0, P_rated=3000.0)

# 拖车（客车或货车）
coach = create_simple_wagon(length=18.0, mass=45.0, P_rated=None)

# 编组
consist = Consist(wagons=[locomotive, coach, coach])
```

## 控制模式

**手动模式**（当前）：
- 用户输入 `throttle/brake` → 物理层计算 `a`
- 方向键 ↑：throttle = 1.0（加速）
- 方向键 ↓：brake = 1.0（制动）
- 松开：throttle/brake = 0（惰行）

**自动模式**（计划中）：
- AI 目标 `v_target` → PID 控制器 → `(throttle, brake)` → 物理层
- 物理层接口不变，控制逻辑外置

## 横向扩展计划（优先级高于阶段 3）

1. **轨道坡度阻力**：
   - `Edge` 新增 `grade: float`（‰）
   - 物理层读取 `track_metadata['grade']`
   - 附加阻力：`±m × g × sin(θ)`

2. **PID 控制器**（自动驾驶基础）：
   - `SimpleSpeedController`: `v_target` → `(throttle, brake)`
   - AI 能自动控制列车达到目标速度

3. **多列车支持**：
   - 每个列车独立 `(consist, kinematics, physics, state)`
   - 碰撞检测（同一 Edge 上距离检查）

4. **可选：柴油机车**：
   - 复用 `RealisticElectric` 框架
   - 只改牵引曲线（柴油机功率曲线 ≠ 恒功率）

## 未来可能的需求（占坑）

- **曲线限速**：轨道 Edge 记录曲线半径，物理层限制 `v_limit = sqrt(0.065 × R)`
- **信号限速**：调度层下发临时限速（黄灯减速、红灯停车）
- **多列车碰撞检测**：同一 Edge 上两列车弧长差 < 安全距离时触发紧急制动
- **能耗优化**：AI 调度优化油门/制动策略，最小化能耗或时间
- **再生制动**：电力机车回馈电网，柴油机无法回馈
- **车钩力**：起动/制动时编组内部车钩拉/压应力，限制加速度避免脱钩

---

**版本**：v0.2（阶段 2 完成，2026-07-10）  
**状态**：生产就绪，横向扩展中

