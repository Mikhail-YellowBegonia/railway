# 编辑器设计需求

总体目标：使用轻量化的方式，实现与《狂热运输2》类似的编辑体验。

---

## 1. 模式状态机

### 1.1 顶层级模式

| 按键 | 模式 | 说明 |
|------|------|------|
| `B`  | BUILD | 建造轨道 |
| `D`  | DELETE | 删除轨道（以边为单位，自动清理孤立节点） |
| `Esc` | （取消 / IDLE） | BUILD_ACTIVE 时取消当前建造退回 BUILD_IDLE；其它情况切换到 IDLE |
| `Q`  | 退出程序 | （Esc 不再用于退出） |

### 1.1.1 IDLE 模式

进入 IDLE 后：
- 左键点击无任何作用（不建造、不删除、不吸附响应）
- 中键拖拽平移视图、滚轮缩放仍然工作
- 此时鼠标的作用只剩"移动视图"

启动时默认进入 IDLE 模式。

### 1.2 Build 模式次级状态机

Build 模式内部有两个子状态：

```
                  click (M1)
    BUILD_IDLE ───────────────→ BUILD_ACTIVE
         ↑                          │
         │         click (M2)       │
         └──────────────────────────┘
         ↑                          │
         │        Esc (cancel)      │
         └──────────────────────────┘
```

| 状态 | 行为 |
|------|------|
| **BUILD_IDLE** | 鼠标移动 + 吸附检测；无预览；等待左键按下 |
| **BUILD_ACTIVE** | 鼠标移动时实时预览轨道；等待左键确认 M2 或 Esc 取消 |

实现见 `controller/editor.py` 的 `Editor` 类。

### 1.3 Delete 模式

**操作单位：边（Edge）为主，中间节点合并为辅**。

设计原则：用户期望与"线要素"互动，而不必关心节点的存在。
DELETE 模式提供两种互动：

- 点击吸附命中的 **边** → 删除该边；端点变孤立则连带清理
- 点击吸附命中的 **节点**（仅 `connection_count == 2` 的中间节点有效）
  → 尝试合并两侧的边为一条，详见 §4.8。不可合并时静默忽略。

悬停优先级：节点 > 边。当鼠标位于节点的吸附阈值内时，hover 给到节点
（即只可能触发"合并"），不可能误删邻接边。

#### 自动清理规则（删除边时）

删除一条边后，遍历其原本的两个端点：
- 若端点变为孤立（`connection_count == 0`）→ 一并删除
- 若端点仍连接其它边（`connection_count >= 1`）→ 保留

底线：操作完成后保证 **没有孤立节点、没有无头边**。

无次级状态。

---

## 2. 数据模型

### 2.1 SnapResult

吸附系统每帧输出一个 `SnapResult`：

```python
@dataclass
class SnapResult:
    snapped: bool                           # 是否发生了吸附
    position: Vec3                          # 吸附后的世界坐标
    tangent: Vec3 | None = None             # 最佳切线方向（端点唯一 / 路径按 reference 选）
    tangent_candidates: list[Vec3] = []     # 该位置所有候选切线（动态由 M2 选择最佳）
    snapped_node_id: int | None = None      # 吸附到的 Node ID（点吸附时）
    snapped_edge_id: int | None = None      # 吸附到的 Edge ID（路径吸附时）
    snapped_edge_t: float | None = None     # 路径吸附时沿边的参数 t ∈ [0,1]
```

`tangent` 与 `tangent_candidates` 的区别：
- `tangent`：吸附时计算的"单一最佳值"，仅在能确定方向时有值（端点；路径
  吸附 + 已传入 reference_pos）。供轻量场景或调试展示使用。
- `tangent_candidates`：所有合法候选；BUILD 阶段把它存进 `build_t1_candidates`，
  到 `_compute_plan` 阶段拿到 M2 后再选最佳。这是支持道岔分支建造和路径
  吸附 M1 的关键。

### 2.2 切线方向获取

一个 Node 处的切线方向由该 Node 连接的 Edge 决定。

**T1 语义**：T1 表示"从 M1 继续延伸的方向"，即新建段在 M1 处的切向。
为保持切线连续不折返，T1 必然是"远离 other 节点"的方向。

| Node 连接数 | T1 候选 |
|-------------|---------|
| 0（孤立节点）| `[]`（无切线约束，Case 1） |
| 1（端点）   | `[t]`，t = 该 Edge 在 Node 处沿"远离 other"方向的切向 |
| ≥ 2（道岔/中间）| `[t1, t2, ...]`，每条相邻边一个候选；最终选哪个由 M2 方向决定 |

**多候选的选择规则**：BUILD_ACTIVE 中提交时（M2 已知），从所有候选中
取 `t.dot((M2-M1).normalize())` 最大者。若最大值 < ~0（所有候选都与
(M2-M1) 钝角或垂直）→ 拒绝（按 §4.0）。

Edge 在 Node 处的"远离 other"切向计算：

- **直边**：`(node.position - other.position).normalize()`
- **弧边**：
  - `tangent = arc_normal × (node.position - arc_center)`
  - 若 `tangent · (node.position - other.position) < 0`，反号
  - 归一化

路径上任意点的切线方向（路径吸附用）：

- 路径吸附返回两个候选 `[forward, reverse]`，BUILD_ACTIVE 中按 (M2-M1)
  夹角选择，规则同上

---

## 3. 吸附系统

### 3.1 架构

吸附系统由多个独立的 `SnapProvider` 组成，按优先级依次执行
（实现：`controller/snap.py::SnapSystem.snap`）：

```
SnapSystem
  ├── PointSnapProvider      (优先级 1) 端点/节点
  ├── GridSnapProvider       (优先级 2) 格点
  ├── ParallelSnapProvider   (优先级 3) 平行参考点（Simple + lazy Complex）
  └── PathSnapProvider       (优先级 4) 既有边投影
```

每个 Provider 有独立的 `enabled: bool` 开关。每帧调用：按优先级遍历已启用的
Provider，取第一个命中结果直接返回，不再检查低优先级。全未命中则返回
`snapped=False` 的原始位置。

**阈值基准**：Point/Path 的吸附阈值以**屏幕像素**为基准（上层按
`camera.scale` 换算成世界米数后经 `world_threshold` 传入），使吸附半径的
视觉距离随缩放稳定。Parallel 用固定 `SNAP_THRESHOLD_PX = 12`。

**互斥约定**（键盘层，`controller/game_loop.py`）：G（格点）/ L（长度）/
A（角度）三者互斥，开一个自动关其余两个；P（平行）、LSHIFT（强制直线）、
LALT（Case 2T）独立切换。

### 3.2 PointSnapProvider（点吸附，优先级 1）

- 在所有节点中找距 cursor 最近且小于阈值者；命中则覆盖位置为该节点坐标
- 切线候选见 §2.2（端点 1 个，道岔 N 个，孤立 0 个）
- 默认 `enabled = True`

### 3.3 GridSnapProvider（格点吸附，优先级 2）

- 把 cursor 吸附到最近格点；格点粒度与视角缩放挂钩（不涉及建造解算调整，
  只改 M1/M2 落点）
- G 键切换，与 L/A 互斥

### 3.4 ParallelSnapProvider（平行吸附，优先级 3）

生成既有轨道的平行参考点，让用户沿参考点建造得到平行轨道。间距固定
`spacing = 5 m`，P 键切换，默认 `enabled = False`。

**Simple Case**：每个非孤立节点、每条出射切线、沿切线垂线两侧各生成一个
参考点（固定间距 `spacing`），参考点继承父节点切线方向。所有 Simple 参考点
由 `update_reference_points` 预生成缓存；`snap` 时按屏幕像素阈值找最近者。

**Complex Case（lazy 后处理拦截，非独立判定）**：Simple 参考点被命中后，
`_try_lazy_complex` 检测该参考点是否恰好落在**某条既有边**上：
- 仅对度数 ≥3 的**道岔节点**派生的参考点触发（普通端点跳过）
- 用"参考点到边的最近点"距离 < `δ = spacing * 0.2 = 1 m` 判定命中
  （非原始设想的"垂线段求交"，落地时简化为最近点+容差）
- 命中则改用**边上最近点 + 该边切线**返回（携带 `snapped_edge_id` / `t`，
  供截断），语义上是"接入这条基本平行、间距约 R 的既有边"
- 未命中则退化回 Simple 参考点

Complex 只在参考点被光标命中的那一帧才计算（lazy），并带 `edge_aabb` 粗筛。
commit 29ae808 的 60× 优化即把 Complex 从"预计算全部"改成这套 lazy 拦截。

**进阶特例（未实现，暂缓）**：若建造起点与终点都是同侧参考点，且两父节点
间存在唯一最短路径，用户期望沿整条路径一次性建造多段平行轨道。这需要轨道
拓扑的最短路径查找，实现困难，暂时逃避。相关空间/拓扑加速见 `docs/tiling.md`。

### 3.5 PathSnapProvider（路径吸附，优先级 4）

- 对每条边计算 cursor 的投影点（直线段 / 圆弧分别处理，见
  `model/geom_utils.project_point_on_edge`），取距 cursor 最近且小于阈值的边
- 切线候选始终为 `[forward, reverse]`；最佳值在 BUILD_ACTIVE 时按 §2.2 规则选
- `edge_direction` 仅直边有值（供 Case 2T §10.5 用）
- 默认 `enabled = True`

### 3.6 长度 / 角度吸附（建造解算内吸附）

不同于上述四个空间 Provider，这两项在**建造解算阶段**生效，会调整几何输出：

- **长度吸附**（L 键，仅直线建造）：把路径长度吸附到增量档位（如 100 m）
- **角度吸附**（A 键，仅单弧建造）：把弧的圆心角吸附到增量档位（如 ±30°）

L/A 与 G 三者互斥。

### 3.7 多边命中

PathSnapProvider 在每个 Edge 上计算最短投影距离，取全局最小。多条边同时被
命中时返回最近的那条。用户要吸附另一条边，需移开鼠标脱离当前边的吸附范围。

---

## 4. 建造逻辑

### 4.0 拒绝处理（统一约定）

任何"无法完成本次建造"的情况都属于"拒绝"，包括但不限于：

- 在 BUILD_IDLE 中：吸附全部关闭 + 鼠标附近有既有元素
- 在 BUILD_ACTIVE 中：M1 == M2、几何无解（例如 Case 2 弧半径过大）、连通性约束不满足
- 在 BUILD_ACTIVE 中：M2 落在违反规则的位置

**统一处理方式**：拒绝时 不修改网络、不切换状态。

| 拒绝发生在 | 拒绝后状态 |
|------------|------------|
| BUILD_IDLE 的 M1 检查 | 仍在 BUILD_IDLE，可显示警告光标，用户可移动鼠标重试或换其它操作 |
| BUILD_ACTIVE 的 M2 检查 | 仍在 BUILD_ACTIVE，M1 保留，用户可移动鼠标重选 M2 或按 Esc 取消 |

预览渲染层面：拒绝（`plan.valid == False`）时仍绘制预览几何，但用红色或警示色，明示"现在按下不会成功"。

### 4.1 开始建造（BUILD_IDLE 中按下鼠标）

行为：

1. 建造前检查：吸附全部关闭 且 `_has_nearby_element(world_pos)` 为真 → 拒绝（光标红色警告，不进入 BUILD_ACTIVE）
2. 确定 M1：优先使用 SnapResult 的 position 和 tangent_candidates（若吸附命中）；否则用原始 cursor 位置，候选为空
3. 进入 BUILD_ACTIVE，预览每帧由 `update_hover` 计算

`_has_nearby_element(pos)` 即检查 `WARNING_THRESHOLD = 0.5` 范围内
是否有节点或边。该阈值略大于 `SNAP_THRESHOLD = 0.3`，以覆盖"肉眼可见但
未触发吸附"的边界情况。

实现：`controller/editor.py::Editor._start_build`。

### 4.2 提交建造（BUILD_ACTIVE 中按下鼠标）

行为：

1. 确定 M2：同 §4.1 的 M1 处理（含吸附）
2. 调用 `_compute_plan` 产生 `ConstructionPlan`
3. 若 `plan.valid == False` → 拒绝（按 §4.0，保持 BUILD_ACTIVE）
4. 若 M1 / M2 通过路径吸附命中既有边内部 → 先截断（见 §4.7）
5. 应用 plan 到网络
6. 回到 BUILD_IDLE

实现：`controller/editor.py::Editor._commit_build`。

### 4.3 ConstructionPlan

```python
@dataclass
class ConstructionPlan:
    case: int                        # 1=直线, 2=单弧, 3=Biarc, 4=弧+直线复合, 5=Case 2T
    m1: Vec3
    m2: Vec3
    node_a_id: int | None            # M1 端的 Node ID（None 则需新建）
    node_b_id: int | None            # M2 端的 Node ID（None 则需新建）
    edge_geometry: list[Vec3]        # [] 直线, [B] 弧（Case 3/4 不用此字段）
    valid: bool                      # 几何是否合法
    # Case 3 Biarc 专用（方案 A：双 Edge + 中间 Node）
    biarc_mid: Vec3 | None
    biarc_geom_1: list[Vec3] | None
    biarc_geom_2: list[Vec3] | None
    # Case 4 弧+直线复合（§10.3）：双 Edge + 中间 Node
    composite_mid: Vec3 | None
    composite_arc_geom: list[Vec3] | None
    composite_tail_geom: list[Vec3] | None
    # Case 5 Case 2T（§10.5）：接入点由算法回算，截断信息随 plan 携带
    m2_split_edge_id: int | None
    m2_split_t: float | None
```

### 4.4 Case 1: 无切线约束 → 自由直线

输入：M1, M2，T1 候选为空（孤立节点 / 空白起点）。
输出：直线 Edge，`edge_geometry = []`。
节点：M1/M2 若有吸附目标 ID 则复用，否则新建。

### 4.5 Case 2: 单切线约束 → 唯一圆弧

```
输入：M1, T1_candidates (≥1 个), M2
处理：
  1. 从 T1_candidates 中选 T1*（与 (M2-M1) 夹角最小者）
  2. 若所有候选都与 (M2-M1) 夹角 ≥ 90°（dot < 阈值）→ 拒绝（Q5）
  3. 若 M2 在 T1* 的"背后"（M2-M1)·T1* ≤ 0）→ 拒绝（Q2）
  4. 解圆弧（见下"几何解算"）
  5. 若 R > MAX_ARC_RADIUS（默认 500.0）→ 退化为沿 T1* 的直线（Q3）
  6. 输出弧 Edge，geometry = [B]
```

**退化处理层级**（半径超限或几何不可解时）：

1. 优先尝试 §10.3 的"弧 + 直线"复合输出（`case=4`）
2. 复合也无解（M2 在固定半径圆内 / 弧角超 π）→ 退化为沿 T1 的纯直线：
   `M2' = M1 + ((M2-M1)·T1*) · T1*`（M2 在 T1* 射线上的投影）

纯直线退化保证起点切线连续（不出现 M1 处折角），肉眼观感为"沿既有
方向画到鼠标横向位置"。

**几何解算（圆心、半径、B 点）：**

给定 M1、T1*（单位切向）、M2，求一个以 M1 为切点（切于 T1*）、通过 M2 的圆。

1. 圆心 O 必在过 M1 且垂直于 T1* 的法线上：
   ```
   O = M1 + s · perp(T1*)        ，perp 在 XY 平面内逆时针 90°
   ```
2. 由 `|O - M1| = |O - M2| = R` 解出：
   ```
   s = |M2 - M1|² / (2 · perp(T1*) · (M2 - M1))
   ```
3. `R = |s|`；分母趋于 0 时（M2 几乎在 T1* 延长线上）→ 退化为直线
4. **arc_normal**：取 +Z 或 -Z，由 `(M1 - O) × T1*` 的 z 分量符号决定
5. **B 点**（切线交点）：M1 处切线 `M1 + k·T1*` 与 M2 处切线 `M2 + k'·T2`
   的交点。其中 M2 处切向 `T2 = arc_normal × (M2 - O)`，归一化后求 2x2 线性方程的交点。

实现见 `model/geom_utils.solve_case2_arc()`。

### 4.6 Case 3: 双切线约束 → 等半径 Biarc

输入：M1（切于 T1）, M2（切于 T2），T1/T2 同语义"远离自身节点的另一端，朝外延伸"。
输出：两段等半径圆弧首尾相接，共用中间点 M_mid（G1 连续）。

**算法概要：**

枚举 4 种手性组合 σ1, σ2 ∈ {+1, -1}（O1 在 T1 法线哪一侧、O2 在 T2 法线哪一侧），
对每组求解关于 R 的二次方程：

`(|dv|² - 4)·R² + 2·(d·dv)·R + |d|² = 0`，其中 `d = M2 - M1`，
`dv = σ2·perp(T2_in) - σ1·perp(T1)`，T2_in 是"沿 M_mid → M2 进入 M2"的方向（即 -T2_outward）。

每组解里取 R > 0 解，过滤"绕大圈"（任一弧扫角 > π），最后在所有合法解中取 **R 最大者**
（曲率最小，符合运输类游戏直觉）。

数据写入采用方案 A（参见 §4.9）：拆为两段连续 Edge + 中间 Node。

**Fast-path：共线退化为直线**

当 T1 与 T2_in 同向、且 (M2-M1) 与 T1 共线（cos ≥ 1-1e-6）时，整段就是直线，
直接走 Case 1 路径而不调用 biarc 求解。

**失败处理：**

- 4 组手性组合全部不可解 → 直接拒绝（保持 BUILD_ACTIVE，按 §4.0 静默）
- 半径超过 `MAX_ARC_RADIUS` → 直接拒绝，不向 Case 2 降级

实现：`controller/editor.py::Editor._try_case3_biarc`、
`model/geom_utils.py::solve_biarc / is_biarc_collinear_straight`。

**T2 候选枚举：** `_try_case3_biarc` 直接对 `t2_candidates` 列表内每一项
分别求解；C 形 / S 形 biarc 中 T2 与 (M2-M1) 反向也是合理配置，
不能用方向打分预筛。最终在所有候选解里取 R 最大者。

### 4.7 截断（Edge 分割）

当 M1 或 M2 通过路径吸附命中一条既有 Edge 的内部点时触发。
两端独立处理，commit 时执行。

流程：

1. 求截断点世界坐标 P 与参数 t（来自 PathSnapProvider 的结果）
2. 调用 `RailNetwork.split_edge_at(edge_id, t)`：
   - 直线 → 两段直线，`geometry = []`
   - 圆弧 → 两段同圆心同半径同向的弧，分别按"端点切线 ∩ 截断点切线"反算 B1、B2
3. 把建造计划中对应端点的 `node_a_id` / `node_b_id` 替换为新中间节点 ID
4. 应用计划（建造新边 / Biarc）

**无损分割保证**：
- 弧的 `arc_center` / `arc_radius` / `arc_normal` 在分割前后完全一致
- 两段角度之和 = 原角度
- 通过 §4.8 规则合并回去能精确还原原 B 点（`split → merge → split → merge` 闭环稳定）
- GeoJSON 往返保留完整元数据（已测试）

**同边双截断**：M1 与 M2 路径吸附到同一条边（在边的中段建一段重叠轨道）→ 静默拒绝。

实现：`model/rail_network.py::RailNetwork.split_edge_at`、
`model/geom_utils.py::split_arc_b_points`、
`controller/editor.py::Editor._commit_build`（截断在 plan 计算之后、apply 之前执行）。

### 4.8 删除与合并

Delete 模式点击一个中间节点（`connection_count == 2`）触发。

**判定**（缺一不可，否则静默拒绝）：

| 两侧边类型 | 合并条件 |
|------------|----------|
| 一直一弧（混合）| 永不可合并 |
| 两直线 | 共线检查：`(mid - a).normalized · (b - mid).normalized >= 1 - 1e-6`（约 0.08° 以内） |
| 两弧 | 半径相等（`< 1e-4`）+ 圆心重合（`< 1e-4`）+ `arc_normal` 同向 + 在 mid 处切线连续（按 mid→b 方向取向后内积 `>= 1 - 1e-6`） |

阈值常量见 `model/geom_utils.py::MERGE_*`。

**合并执行**：

1. 找到两条边各自的"远端"节点 other_a 和 other_b
2. 若 other_a == other_b（合并会形成自环）→ 拒绝
3. 移除两条边和中间节点
4. 用 other_a / other_b 和合并几何（直线 `[]` 或合并弧 `[B_new]`）建一条新边
   - 合并弧的 B 点：a_pos 处切线（指向 b 方向）与 b_pos 处切线（同向调整）的交点

端点节点（`connection_count == 1`）的"合并"无意义，点击不响应。
道岔节点（`connection_count >= 3`）同理。

实现：`controller/editor.py::Editor._try_merge_at_node`、
`model/geom_utils.py::can_merge_straight / can_merge_arcs / merged_arc_b_point`。

### 4.9 双弧 Edge 的数据模型考量

当前 Edge 模型只支持"一段弧"（geometry 中单 B 点）。Biarc 需要两段弧。
**采用方案 A**：Biarc 展开为两个连续的 Edge + 一个中间 Node。

- `Edge.geometry` 保持 `[]` 或 `[B]`，不变
- Biarc = 连续两个 Edge，中间自动插入一个 Node（用户视角下与普通中间节点无异）
- GeoJSON 直接输出两个 LineString
- 用户后续在 DELETE 模式下点击该中间节点，按 §4.8 等半径合并规则可还原为单弧（如果几何允许）

---

## 5. 几何工具函数

实现统一在 `model/geom_utils.py`。下表是当前已落地的函数索引，
新增函数请同步更新。

| 函数 | 用途 |
|------|------|
| `project_point_on_edge(p, edge, na, nb)` | 点投影到边，统一入口；返回 `(t, proj, distance)` |
| `project_on_segment(p, a, b)` | 点投影到直线段（夹紧到端点） |
| `project_on_arc(p, edge, na, nb)` | 点投影到圆弧（夹紧到弧角范围） |
| `tangent_along_edge(edge, na, nb, t)` | 边上参数 t 处的 forward 切线 |
| `tangent_at_arc_point_forward(edge, point)` | 圆弧某点处的 forward 切线 |
| `rotate_around_axis(v, axis, angle)` | Rodrigues 旋转 |
| `perp_xy(v)` | XY 平面内逆时针 90° |
| `solve_case2_arc(m1, t1, m2)` | Case 2 几何解：返回 `(center, B, normal, R)` 或 `None` |
| `solve_case2_composite(m1, t1, m2, max_r)` | Case 2 半径超限复合解（§10.3）：返回 `(P_mid, B, normal, tail_dir, R)` 或 `None` |
| `solve_case2t(m1, t1, m2_mouse, p0, t2_dir, max_r)` | Case 2T 单切线弧（§10.5）：返回 `(P, B, normal, R)` 或 `None` |
| `solve_biarc(m1, t1, m2, t2_in)` | Case 3 几何解：返回 `(M_mid, B1, B2, normal_1, normal_2, R)` 或 `None`。`t2_in` 是沿 M_mid→M2 进入方向 |
| `is_biarc_collinear_straight(t1, t2_in, m1, m2)` | Case 3 共线退化 fast-path 判定 |
| `split_arc_b_points(edge, na, nb, p)` | 弧在 p 处分两段，返回两段子弧的 B 点 `(B1, B2)` |
| `is_t1_consistent_with_target(t1, m1, m2)` | T1 是否指向 M2 一侧（用于 Q2 拒绝） |
| `project_along_direction(m1, t1, m2)` | M2 在 (M1, T1) 射线上的投影点（半径退化用） |
| `can_merge_straight(a, mid, b)` | 两直线是否可合并（共线检查） |
| `can_merge_arcs(e1, e2, mid, a, b)` | 两弧是否可合并（同心+同半径+同向+切线连续） |
| `merged_arc_b_point(e1, e2, a, b)` | 合并弧的新 B 点 |

平面约定：所有计算假定轨道在 XY 平面，`PLANE_NORMAL = Vec3(0, 0, 1)`。
弧的 `arc_normal` 取 `+Z` 或 `-Z`，由弧的旋转方向决定。

---

## 6. 预览渲染

### 6.1 预览协议

在 `BUILD_ACTIVE` 状态下，渲染器收到一个 `PreviewGeometry` 结构：

```python
@dataclass
class PreviewGeometry:
    m1: Vec3
    m2: Vec3
    case: int                       # 1..5，与 ConstructionPlan 同义
    edge_geometry: list[Vec3]       # Case 1/2/5 的 geometry
    valid: bool
    # Case 3 预览
    biarc_mid: Vec3 | None
    biarc_geom_1: list[Vec3] | None
    biarc_geom_2: list[Vec3] | None
    # Case 4 弧+直线复合
    composite_mid: Vec3 | None
    composite_arc_geom: list[Vec3] | None
    composite_tail_geom: list[Vec3] | None
    # Case 5 Case 2T：算法回算的接入点（用独立颜色标记）
    case2t_entry: Vec3 | None
```

渲染方式：
- 半透明或虚线绘制预览 Edge
- M1 位置绘制高亮锚点
- 若 `valid == False`，以红色/警告色绘制，提示无法建造

### 6.2 警告指示器

当 `BUILD_IDLE` 状态且满足以下条件时，光标显示红色警告：
- 所有吸附 Provider 的 `enabled == False`
- `_has_nearby_element(cursor_pos)` 返回 True

渲染：在光标周围绘制红色圆环（或红色十字），表示"建议开启吸附"。

---

## 7. 已完成范围一览

编辑器主干（Step 0–6）与追加需求（§10.1–§10.5）均已落地。

| 范围 | 关联章节 |
|------|----------|
| ✅ 三模式状态机（IDLE / BUILD / DELETE）+ BUILD 子状态机 | §1 |
| ✅ 点吸附 + 路径吸附（优先级：点 > 路径） | §3.2 / §3.3 |
| ✅ Case 1 自由直线 | §4.4 |
| ✅ Case 2 单切线弧 + 半径超限退化 | §4.5 |
| ✅ Case 3 等半径 Biarc（方案 A：双 Edge + 中间 Node） | §4.6 |
| ✅ Edge 截断（M1/M2 路径吸附到边内部） | §4.7 |
| ✅ DELETE 边删除 + 中间节点合并（直线共线 / 弧同心同半径同向） | §4.8 |
| ✅ 预览渲染（虚线 + M1 锚点 + 无效红色 + 警告光标） | §6 |
| ✅ 视图三键平移（IDLE 左/中/右键） | §10.1 |
| ✅ LSHIFT 强制直线（`force_straight`） | §10.2 |
| ✅ 弧+直线复合输出（`case=4`） | §10.3 |
| ✅ BUILD_ACTIVE 右键 = 取消 | §10.4 |
| ✅ LALT 单切线弧 Case 2T（`case=5`） | §10.5 |
| ✅ GeoJSON 双向往返（直线 / 弧 / 截断后弧元数据保持） | §9 |
| ✅ §12.1 公制约定 / 网格背景 / 建造 HUD / Ballast 占位 | §12.1 |
| ✅ 格点吸附（`GridSnapProvider`，G 键） | §3.3 |
| ✅ 长度吸附（L 键，仅直线）/ 角度吸附（A 键，仅单弧），G/L/A 互斥 | §3.6 |
| ✅ 平行吸附 Simple + Complex Case（`ParallelSnapProvider`，P 键） | §3.4 |
| ✅ 空间索引（uniform grid，`TILE_SIZE=50m`，340× 加速，I 键可视化） | §11 |

吸附功能的完整规格与分类见 §3。**唯一明确未实现**的是平行吸附 Simple Case
的进阶情况（同侧两参考点间沿最短路径一次性建造整条多段平行轨道），§3.4
标注为"暂时逃避"，需轨道拓扑最短路径查找。

---

## 8. 附录：同类参考

### Biarc 中间点策略（运输类游戏常用）

- **等半径**（equal radii）：两弧半径相等，对称美观
- **最小化最大曲率**（minimize max curvature）：找使两弧中较大曲率最小化的解
- **指定半径**（fixed radius）：允许用户指定其中一段弧的半径
- **等弧长**：两弧弧长接近

初期实现：**等半径策略**。其他策略可后续添加为 Biarc 策略的可选参数。

### 曲率限制

当前唯一的曲率约束是 `MAX_ARC_RADIUS = 500.0`（`controller/editor.py`），
作为 Case 2 半径超限退化的阈值。ConstructionPlan 未携带最小半径约束，
如需最小曲率半径的物理约束（列车曲线通过限制等），后续可在 plan 层扩展。

---

## 9. GeoJSON 兼容性

### 双弧写入
若采用方案 A（双 Edge + 中间 Node），GeoJSON 直接输出两个 3 点 LineString，无需特殊处理。

### 截断后的弧元数据
截断圆弧时需重新计算两半段的 B 点，确保写回 GeoJSON 后重载的弧元数据完全相等。截断算法必须精确（无信息丢失）。

---

## 10. 追加需求

主干功能之外的 UX / 几何补充。均已完成，本节作为对应特性的规格说明。

### 10.1 视图操作的输入扩展 ✅ 已实现

- IDLE 模式下，鼠标 **左键 / 中键 / 右键 拖拽** 都能平移视角。
- BUILD / DELETE 模式下保持原行为：左键 = 操作，中键 = 平移。
  右键在这两个模式下的语义见 §10.4。
- 实现：`controller/game_loop.py::_pan_buttons_for_mode`，平移触发集
  `PAN_BUTTONS_IDLE = {1, 2, 3}`，`PAN_BUTTONS_OTHER = {2}`。

### 10.2 强制直线建造（LSHIFT） ✅ 已实现

- BUILD_ACTIVE 中按住 `LSHIFT` 时，跳过弧 / Biarc 解算、直接走"沿 T1
  投影直线"路径（同 Q3 退化）：终点 = M2 在 (M1, T1) 射线上的投影。
  - 起点切线连续，方向受既有轨道约束，长度由鼠标控制
  - Q2/Q5 的拒绝条件仍适用
  - 无 T1 候选（孤立起点）时无效，仍走 Case 1 自由直线
- 用途：绕过 Case 2 / Case 3 的几何分支，长直线场景实用。
- 状态显示：BUILD 模式下激活时左上角追加 `[STRAIGHT]` 提示。
- 实现：`controller/editor.py::Editor.force_straight` +
  `controller/game_loop.py::_sync_modifiers` 每帧轮询。

### 10.3 弧半径超限的复合输出（替代 §4.5 的退化方案） ✅ 已实现

- 当 Case 2 解算的弧半径超过 `MAX_ARC_RADIUS` 时，输出 **圆弧 + 直线**
  复合结果：前段以 `MAX_ARC_RADIUS` 为半径作弧到中间点 P_mid，
  后段直线从 P_mid 切线连续延伸到 M2。
- 数据模型上扩展为 `case=4`：`ConstructionPlan` 携带 `composite_mid` /
  `composite_arc_geom`（[B]）/ `composite_tail_geom`（[]）。应用时拆为
  两条 Edge + 一个中间 Node（与 Biarc 方案 A 同构）。
- **几何**：圆心 `O = M1 + σ·R·perp(T1)`（σ 选 M2 所在侧），
  P_mid 是从 M2 向圆作外切线的切点。两个候选切点取弧角较小者，
  且要求 P_mid 处切线与 (M2-P_mid) 同向、弧角 ∈ (0, π)。
- **退化兜底**：M2 在固定半径圆内 / 弧角超 π → 复合无解 →
  回退到 §4.5 旧的"沿 T1 投影直线"方案。
- **优先级**：`force_straight`（§10.2）> 复合 > 纯直线退化。
- 实现：`model/geom_utils.py::solve_case2_composite` +
  `controller/editor.py::_try_case2_composite`。
- 切线连续性已验证（弧切线 · 直线方向 = ±1 至 1e-6）；GeoJSON 往返保持
  弧元数据。

### 10.4 BUILD_ACTIVE 中右键的语义 ✅ 已实现

- BUILD_ACTIVE 中右键 → 取消当前建造，回到 BUILD_IDLE（等同 Esc）。
- 其它模式（IDLE / BUILD_IDLE / DELETE）中右键行为不变：
  - IDLE 中右键拖拽 = 平移（见 §10.1）
  - BUILD_IDLE / DELETE 中右键拖拽 = 无效（仅中键平移）
- 采用简洁的"右键 = 取消"语义，而非"撤销最近一段"（后者需要建造历史栈，
  在当前单段建造模式下意义有限）。
- 实现：`controller/game_loop.py::_handle_mouse_down` 中右键分支。

### 10.5 单切线弧（Case 2T，手动触发） ✅ 已实现

**触发**：BUILD_ACTIVE + 按住 **LALT** + M2 路径吸附到**直边**（弧边不适用）。

**几何**：求一段弧 *切于 (M1, T1)* 且 *切于直线 L*（L = M2 所在边的方向线）。

- 设 `α = T1·T2_hat`，`β = perp(T2_hat)·(M1 - P0)`，`P0 = edge.node_a`
- 方程 `(α²-1)·s² + 2αβ·s + β² = 0`，两根 `s = -β/(α ± 1)` 对应 L 两侧的切圆
- 圆心 `O = M1 + s·perp(T1)`，半径 `R = |s|`，切点 `P = O - ((O-P0)·perp(T2))·perp(T2)`

**M2 鼠标位置**仅作"二选一"偏好信号（两个候选切点选离鼠标近者），
**精确接入点由算法决定**。

**数据模型**：`case=5`，`m2_split_edge_id` + `m2_split_t` 记录接入点截断信息。
应用时单弧 Edge（`geometry=[B]`），中间节点由截断产生。

**截断**：M2 端不用 snap 的 t，而用 plan 回算的 `m2_split_t`（在 `_commit_build` 中
特判 case=5）。

**预览**：弧 M1→B→entry 用普通虚线，接入点用**品红色方框**标记，强调
"算法回算的精确位置 ≠ 鼠标位置"。

**退化与拒绝**：
- α=±1（T1 与边平行/反平行）→ 必为半圆，不符合建造逻辑 → None
- R > MAX_ARC_RADIUS → None
- 接入点越界（t ∉ (0, 1) 开区间）→ None
- Q2: T1·(P-M1) < 0（严格反向）→ 拒绝；允许 90° 转弯

**优先级**：LSHIFT (`force_straight`, §10.2) > LALT (`force_case2t`, §10.5)
> 常规 Case 2/3。LALT 跳过 Q2/Q5 的 m2 方向检查（m2 仅作偏好）。

**切线连续性**：
- M1 处 = T1（继承既有轨道切线）
- 接入点处 = ±边方向（与目标直边切线连续）

**实现**：`model/geom_utils.py::solve_case2t` +
`controller/editor.py::_try_case2t` + `controller/snap.py::SnapResult.edge_direction`。

**用途**：远端路径吸附场景下精确接入既有直边，避免 Biarc 强行解算的
笨拙曲率分布。尤其垂直/斜向接入。

---

## 11. 当前阶段与近期方向

**编辑器基本功能已稳固**。主干（Step 0–6）、追加需求（§10.1–§10.5）、
§12.1 定稿的四项（公制/网格/HUD/Ballast），以及 §3 五类吸附中
的前四类 + 平行吸附 Simple/Complex Case 均已落地，几何算法通过端到端测试，
用户手感亦经人工验证。

**空间索引已实施**（commits `3591823` / `9417752` / `48f97ae`）：
- `model/spatial_index.py`：uniform grid 空间哈希（`TILE_SIZE = 50m`），
  增量插入/删除，稀疏 dict 查询
- `RailNetwork` 插桩：5 个增删点同步维护索引，暴露 `nearby_node_ids` /
  `nearby_edge_ids` 查询方法
- snap.py 三个 Provider 改走索引：Point/Path/Parallel 从 O(N) 全扫描
  降为 O(k) 邻近瓦片查询
- 性能验证：2601 节点 + 5100 边路网，加速比 **340.8×**，索引查询
  0.002 ms/次，满足 60fps 要求
- I 键可视化：瓦片边界 + 查询邻域高亮（debug 用）

**已知的下一步方向**：
- **进阶平行吸附**：沿两 Node 间最短路径一次性建造多段平行轨道
  （§3.4 平行吸附的进阶特例，需拓扑最短路径查找）

**近期开发重点仍是编辑器**：车辆、地形、经济等其它模块暂不启动。

**Z 轴（高程）暂缓引入**：
- 底层数据（`Vec3` / `arc_normal ∈ {±Z}` / `PLANE_NORMAL`）已保留 Z 分量，
  预留了未来扩展空间
- 但视觉层仍为平面渲染，缺乏可靠的高程调试与预览手段
- 在渲染 / 交互能提供合理反馈之前，不引入实际的 Z 数据变化
- 相关代码假定"所有几何在 XY 平面"这一约定短期不变

## 12. 新一轮需求探讨（已收敛）

> 本节记录的探讨已落地或已转入专项文档，保留作为背景。当前状态：
> - **建立坐标系与尺寸参考**（公制/网格/HUD/Ballast）→ 已实现，规格见 §12.1
> - **进阶吸附**（格点/长度角度/平行）→ 已实现，完整规格见 §3
> - **瓦片化数据结构** → 设计研讨稿见 `docs/tiling.md`（未实施）
> - **测试图像 Ballast** → 已实现底层 Ballast 占位（半宽 2 m）

**建立坐标系与尺寸参考**
1. 数据结构上，应当绑定到公制单位。
2. 坐标系上，特别强调GeoJSON的惯用坐标系为WGS84，但我们目前不采纳，先使用数学坐标系完成早期开发。
3. 图像层面，添加简易的网格背景。
4. UI层面，在建造轨道时显示路径长度，曲线半径，角度等信息。这是同类游戏中既有的设计，可以采纳。

**进阶的吸附功能**
这一块内容需要单独的文档来明确逻辑与算法，需要做的比较稳健，因此在这里仅提及一下。*实施前务必额外研讨*
吸附功能应当完成的任务（用户体验角度）：
- 建造平行轨道
- 建造对称的结构，如交叉渡线
- 连接到已有的路径和节点
- 吸附到网格线、固定角度或方向，例如30度或正东

这其中有一些不言自明的内容，很多游戏，比如Transport Fever系列，Cities:Skylines系列都是用类似的操作逻辑，提供类似的功能。

**测试图像**
目前我们使用线来表示轨道，如果使用有宽度的图像，有助于提升尺寸感
轨道被分为5个图层，按绘制顺序分别是：
0. BottomExtra（桥梁等额外内容）
1. Ballast（地基）
2. Sleeper（枕木）
3. Rail（铁轨）
4. TopExtra（架空线等额外内容）

需要指出，Sleeper图层和Rail图层涉及额外算法，这是由于：
- 枕木的间距需要保持均匀。特别地，若参考现实，道岔处的枕木摆放方式特殊且怪异
- 铁轨是平行于逻辑路径的，需要少量几何变换

因此出于便捷测试目的，推荐实现且仅实现Ballast图层

**Pygame已知问题**
Pygame的分辨率在使用Retina技术的MacOS屏幕上工作不正常，这是由于UHD（4K for 16:10）屏幕的逻辑分辨率被定义为1512*982。
等于说，Pygame中的1个逻辑像素，代表物理意义上的4个像素（2*2）方格。此问题在Linux发行版上也有可能出现。
不否认有解决办法，但通常是引入modernGL，这是我们暂时不愿做的。
鉴于这是一个已知问题，且应当不由我们背锅，无需解决，知晓即可。

### 12.1 本轮定稿（实施决策）

经研讨后确定本轮实施范围与口径如下：

**公制单位**：采用纯约定 **1 世界单位 = 1 米**，仅写入文档与常量注释，
不改动 `Vec3` / `model` 数据结构本身（避免污染纯几何层）。现有常量已隐含
米语义：`MAX_ARC_RADIUS = 500`（500 m 上限）、`SNAP_THRESHOLD = 0.3`（0.3 m）、
`Camera.scale`（像素/米）。

**坐标系**：继续使用数学坐标系（Y 轴向上，`world_to_screen` 中 `sy = h/2 - wy*scale`）。
WGS84 仅作未来备注，本轮不采纳。

**网格背景**：自适应档位。档位序列走 1/2/5 × 10ⁿ（含用户列举的 1/5/10/50/100 m），
随缩放跳档，保持屏幕密度稳定。绘制两级——次级（细/暗）+ 主级（= 次级×5，粗/亮）。
网格的世界坐标定义须可被未来的"吸附到网格线"直接复用（见上文进阶吸附，待单独研讨）。

**建造 HUD**：跟随光标的 tooltip，显示 **路径长度（m）/ 曲线半径（m）/ 圆心角（度）**。
复合几何**分段列出**：Case 3 Biarc 两段各列，Case 4 弧+直线分列弧段与直线段，
Case 5 单切线弧列弧段。角度取**弧的圆心角**。

**Ballast 测试图像**：仅实现最底层 **Ballast** 图层，带宽 **4 m**（半宽 2 m）。
其余 4 层（BottomExtra/Sleeper/Rail/TopExtra）本轮不做。**保留中心逻辑细线**
叠加在 Ballast 之上（调试用）。**道岔节点处多条 Ballast 带直接接受重叠**，
不做交叉口特殊几何（那属于 Sleeper 层）。

**本轮不做**：进阶吸附（仅预留网格世界定义，待单独文档研讨）、Retina/分辨率
（已知问题，知晓即可）。

以上四项（公制约定 / 网格 / HUD / Ballast）全部落在 `view` 层与文档层，
不触碰 `model` 几何内核与 `controller` 状态机。