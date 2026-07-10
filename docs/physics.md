# 列车物理层设计（临时文档）

## 设计原则

物理层采用**接口抽象 + 可插拔实现**架构，支持从玩具级到真实级的渐进式扩展：
- 第一版：`SimplePhysics` 极简模型，只为让列车"能动、能停"
- 后续：`RealisticElectric` / `RealisticDiesel` 等真实物理模型

所有实现共享统一接口，调用方（GameLoop）无感知切换。

## TrainPhysics 接口（抽象基类）

```python
class TrainPhysics:
    def update(dt: float) -> None
        """每帧物理更新，返回新的状态（s, v）"""
    
    def apply_throttle(value: float) -> None
        """设置油门/功率手柄，value ∈ [0, 1]"""
    
    def apply_brake(value: float) -> None
        """设置制动，value ∈ [0, 1]"""
    
    def get_state() -> (float, float, float)
        """返回 (弧长 s, 速度 v, 加速度 a)"""
    
    def reset(initial_s: float) -> None
        """重置到起点"""
```

## SimplePhysics 第一版（当前实现）

**目标**：玩具级物理，验证接口和交互流程，忽略一切真实复杂性。

**状态**：
- `s`: 弧长（米）
- `v`: 速度（m/s）

**输入**：
- `throttle ∈ [0, 1]`: 油门（0=怠速，1=全开）
- `brake ∈ [0, 1]`: 制动（0=无制动，1=全制动）

**物理模型**（极简）：
- 加速度 `a = throttle × a_max - brake × b_max`
  - `a_max = 1.0 m/s²`（恒定最大加速度，无牵引曲线）
  - `b_max = 2.0 m/s²`（恒定最大制动减速度）
- 速度更新 `v' = a`
- 弧长更新 `s' = v`
- 速度限制 `v ∈ [0, v_max]`，默认 `v_max = 30 m/s`（~108 km/h）

**忽略的因素**（后续版本补充）：
- ❌ 牵引特性曲线（电机恒功率区/恒转矩区、内燃机功率曲线）
- ❌ 黏着系数限制（轮轨接触、空转/滑行）
- ❌ 阻力（空气阻力、滚动阻力、坡度阻力、曲线阻力）
- ❌ 质量/惯性（编组质量、旋转质量换算）
- ❌ 再生制动/空气制动混合
- ❌ 功率分配（动力车/拖车、轴重转移）

**交互方式**（临时，AI 调度前）：
- 方向键 ↑：throttle = 1.0（加速）
- 方向键 ↓：brake = 1.0（制动）
- 松开：throttle/brake = 0（惰行）

## 后续扩展点（待实现）

### RealisticElectric（真实电力机车）

**新增状态**：
- 牵引电机温度、电流
- 黏着状态（正常/空转）

**牵引特性**：
- 恒转矩区（低速）：`F_max = const`
- 恒功率区（高速）：`F_max = P_rated / v`
- 黏着限制：`F_actual = min(F_max, μ × m × g)`

**制动**：
- 再生制动优先（回馈电网，受速度/电流限制）
- 空气制动补充（机械摩擦）
- 紧急制动（最大减速度）

**阻力**：
- Davis 公式：`R = A + B×v + C×v²`（系数依车型）
- 坡度附加：`± m×g×sin(θ)`
- 曲线附加：`600 / R_curve`（半径 R，单位米）

### RealisticDiesel（真实内燃机车）

**牵引特性**：
- 柴油机功率曲线（转速-功率）
- 液力/电力传动效率曲线
- 换挡逻辑（档位-速度匹配）

**燃料消耗**：
- 怠速油耗
- 负荷相关油耗（BSFC 曲线）

### 多机重联/编组

**功率分配**：
- 按编组中动力车数量和位置分配牵引力
- 轴重转移（爬坡时后部动力车黏着下降）

**车钩力**：
- 起动/制动时编组内部车钩拉/压应力
- 限制加速度避免脱钩/碰撞

## 接口使用示例

```python
# GameLoop 中替换现有的 s += 10*dt
physics = SimplePhysics(path_kinematics, initial_s=0.0)

# 每帧更新
for frame in game_loop:
    # 读取玩家输入或 AI 决策
    if key_up_pressed:
        physics.apply_throttle(1.0)
    elif key_down_pressed:
        physics.apply_brake(1.0)
    else:
        physics.apply_throttle(0.0)
        physics.apply_brake(0.0)
    
    # 物理更新
    dt = clock.get_time() / 1000.0
    physics.update(dt)
    
    # 查询状态用于渲染
    s, v, a = physics.get_state()
    pose = path_kinematics.pose_at(s)
    draw_train(pose)
```

## 未来可能的需求（占坑）

- **坡度信息**：轨道 Edge 需记录坡度（‰），物理层查询 `edge.grade_at(t)`
- **曲线限速**：轨道 Edge 记录曲线半径，物理层限制 `v_limit = sqrt(0.065 × R)`
- **信号限速**：调度层下发临时限速（黄灯减速、红灯停车）
- **多列车碰撞检测**：同一 Edge 上两列车弧长差 < 安全距离时触发紧急制动
- **能耗优化**：AI 调度优化油门/制动策略，最小化能耗或时间

---

**版本**：v0.1（SimplePhysics 设计，2026-07-10）  
**状态**：草稿，随实现同步更新
