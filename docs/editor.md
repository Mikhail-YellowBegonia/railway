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

吸附系统由多个独立的 `SnapProvider` 组成，按优先级依次执行：

```
SnapSystem
  ├── PointSnapProvider      (优先级 1)
  ├── PathSnapProvider       (优先级 2)
  ├── ParallelPointProvider  (优先级 3, 留后)
  └── ParallelPathProvider   (优先级 4, 留后)
```

每个 Provider 有一个独立的 `enabled: bool` 开关。

每帧调用：遍历所有已启用的 Provider，按优先级取第一个命中结果。高优先级的命中直接返回，不检查低优先级。

### 3.2 PointSnapProvider（点吸附）

- 阈值 `threshold = 0.3` 世界单位
- 在所有节点中找距 cursor 最近且小于阈值者；命中则覆盖位置为该节点坐标
- 切线候选见 §2.2（端点 1 个，道岔 N 个，孤立 0 个）
- 实现：`controller/snap.py::PointSnapProvider`

### 3.3 PathSnapProvider（路径吸附）

- 阈值 `threshold = 0.3` 世界单位
- 对每条边计算 cursor 的投影点（直线段 / 圆弧分别处理，见 `model/geom_utils.project_point_on_edge`），取距 cursor 最近且小于阈值的边
- 切线候选始终为 `[forward, reverse]`；最佳值在 BUILD_ACTIVE 时按 §2.2 规则选
- 实现：`controller/snap.py::PathSnapProvider`

### 3.4 多边命中

PathSnapProvider 在每个 Edge 上计算最短投影距离，取全局最小。多条边同时被命中时，返回距离最近的那条边。用户要吸附另一条边，需将鼠标移开以脱离当前边的吸附范围。

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
4. 若 `plan.split_edge_id` 非 None → 先截断（见 §4.7，待实现）
5. 应用 plan 到网络
6. 回到 BUILD_IDLE

实现：`controller/editor.py::Editor._commit_build`。

### 4.3 ConstructionPlan

```python
@dataclass
class ConstructionPlan:
    case: int                        # 1, 2, or 3
    m1: Vec3
    m2: Vec3
    node_a_id: int | None            # M1 端的 Node ID（None 则需新建）
    node_b_id: int | None            # M2 端的 Node ID（None 则需新建）
    edge_geometry: list[Vec3]        # [] 直线, [B] 弧（Case 3 不用此字段）
    split_edge_id: int | None        # 需要截断的 Edge ID（待实现）
    split_at_t: float | None         # 截断参数 t ∈ [0,1]
    valid: bool                      # 几何是否合法
    # Case 3 Biarc 专用（方案 A）
    biarc_mid: Vec3 | None           # 中间节点位置 M_mid
    biarc_geom_1: list[Vec3] | None  # 弧 1 几何（[B1]）
    biarc_geom_2: list[Vec3] | None  # 弧 2 几何（[B2]）
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

**退化为直线的处理**：当 Case 2 计算结果半径过大或几何不可解时，
为保持起点切线连续（避免 M1 处折角），不直接连 M1→M2，而是建造
M1 → M2'，其中 `M2' = M1 + ((M2-M1)·T1*) · T1*`，即 M2 在 T1* 上的
投影。这样肉眼看上去就是沿 T1* 方向画一条直线"画到鼠标横向位置"。

> 后续计划：当弧半径超限时，应改为"圆弧+直线"的复合结果（先按
> MAX_ARC_RADIUS 画一段弧，再接直线到 M2）。当前简化为单一直线，
> 是临时退化方案。

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
- 半径超过 `MAX_ARC_RADIUS` → 直接拒绝（按用户决议，不向 Case 2 降级；
  待后续观察实际失败率，必要时再考虑退化策略）

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
    case: int                       # 1, 2, or 3
    edge_geometry: list[Vec3]       # Case 1/2 的 geometry
    valid: bool                     # 当前计算是否合法
    # Case 3 预览
    biarc_mid: Vec3 | None
    biarc_geom_1: list[Vec3] | None
    biarc_geom_2: list[Vec3] | None
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

## 7. 实施步骤

| 步 | 内容 | 状态 | 说明 |
|----|------|------|------|
| **0** | **清理遗留代码** | ✅ 完成 | 删除 `model/geometry.py` 重复定义 |
| **1** | **Editor 模式重构 + 点吸附 + Case 1 + 预览 + 警告** | ✅ 完成 | 三模式（IDLE/BUILD/DELETE）+ BUILD 子状态机（IDLE/ACTIVE）。点吸附（PointSnapProvider）。Case 1 直线建造（四种端点组合）。实时预览（虚线+M1锚点）。警告指示器（红色十字+圆环）。拒绝处理统一为保持当前状态。 |
| **2** | **路径吸附 + DELETE 边导向** | ✅ 完成 | PathSnapProvider（直线+弧投影，`model/geom_utils.py` 几何工具集）。切线方向按 BUILD_ACTIVE 时 `(cursor - M1)` 夹角选择。DELETE 模式操作单位改为 Edge，自动清理孤立节点。优先级：点吸附 > 路径吸附。 |
| **3** | **Case 2 弧建造** | ✅ 完成 | T1 候选化重构（`build_t1_candidates`）：道岔 N 候选、路径吸附 forward/reverse 双候选、端点 1 候选。提交时按 (M2-M1) 夹角选最佳。Q2: T1 反向拒绝；Q3: R > MAX_ARC_RADIUS (500.0) 退化为沿 T1 投影的直线；Q5: 所有候选钝角时拒绝。`solve_case2_arc` 几何工具。弧预览（虚线弧 + 切线辅助线）。 |
| **4** | **删除-合并** | ✅ 完成 | DELETE 模式悬停优先级 节点 > 边；点击 connection==2 节点尝试合并两侧边。判定：直线共线检查（cos ≥ 1-1e-6）、弧同心同半径同向且切线连续。失败静默忽略。详见 §4.8。 |
| **5** | **Case 3 Biarc** | ✅ 完成 | 等半径双弧建造（方案 A：双 Edge + 中间 Node）。枚举 4 种手性组合，二次方程解 R，过滤"绕大圈"和半径超 MAX_ARC_RADIUS 的解，取 R 最大者。共线 fast-path 退化为直线。失败直接拒绝（按用户决议）。详见 §4.6。 |
| **6** | **Edge 截断** | ✅ 完成 | M1 / M2 路径吸附到边内部时 commit 阶段分裂原边为两段；同边双截断静默拒绝。直线均匀分割，弧用切线交点反算保证同心同半径同向，截断 → 合并 → GeoJSON 往返完全一致。详见 §4.7。 |

---

## 8. 附录：同类参考

### Biarc 中间点策略（运输类游戏常用）

- **等半径**（equal radii）：两弧半径相等，对称美观
- **最小化最大曲率**（minimize max curvature）：找使两弧中较大曲率最小化的解
- **指定半径**（fixed radius）：允许用户指定其中一段弧的半径
- **等弧长**：两弧弧长接近

初期实现：**等半径策略**。其他策略可后续添加为 Biarc 策略的可选参数。

### 曲率限制

开发初期不实施最小/最大曲率半径限制。当前仅在 Case 2 中以
`MAX_ARC_RADIUS = 500.0`（位于 `controller/editor.py`）作为退化阈值。
后续可扩展 ConstructionPlan 携带最小半径约束。

---

## 9. GeoJSON 兼容性

### 双弧写入
若采用方案 A（双 Edge + 中间 Node），GeoJSON 直接输出两个 3 点 LineString，无需特殊处理。

### 截断后的弧元数据
截断圆弧时需重新计算两半段的 B 点，确保写回 GeoJSON 后重载的弧元数据完全相等。截断算法必须精确（无信息丢失）。

---

## 10. 追加需求（待整理 / 落实）

> 此区收集开发过程中陆续追加的需求，等到合适的实施步骤完成后，再
> 整理到上文相应章节并标记完成。

### 10.1 视图操作的输入扩展

- IDLE 模式下，鼠标 **左键 或 右键 拖拽** 应能平移视角（当前仅中键支持）。
  目的：对触控板用户更友好，避免依赖中键。
- 在 BUILD / DELETE 模式下保持原行为：左键 = 操作，中键 = 平移；
  右键的行为待 §10.x 进一步规划。

### 10.2 强制直线建造（临时调试键）

- BUILD_ACTIVE 中按住 `LSHIFT` 时，跳过弧解算、直接走"沿 T1 投影直线"
  路径（同 Q3 退化）：终点 = M2 在 (M1, T1) 射线上的投影。
  - 起点切线连续，方向受既有轨道约束，长度由鼠标控制
  - Q2/Q5 的拒绝条件仍适用
  - 无 T1 候选（孤立起点）时无效，仍走 Case 1 自由直线
- 用途：测试期手动绕过 Case 2 / Case 3 的几何分支。
- 后续会作为正式 UX 设计的一部分重新规划，键位与触发方式都可能变。
- 状态显示：BUILD 模式下激活时左上角追加 `[STRAIGHT]` 提示。

### 10.3 弧半径超限的复合输出（替代 §4.5 的退化方案）

- 当 Case 2 解算的弧半径超过 `MAX_ARC_RADIUS` 时，当前简化为
  "沿 T1 单段直线"。
- 目标改为：输出 **圆弧 + 直线**（或反序）的复合结果——前段以
  `MAX_ARC_RADIUS` 为半径作弧到某中间点，后段直线到 M2，
  保证整体光滑、且终点严格落在 M2。
- 这涉及 ConstructionPlan 支持多段输出，复杂度较高，等基础工作齐备
  再回头处理。

### 10.4 BUILD_ACTIVE 中右键的语义

- 当前仅左键确认 M2、Esc 取消。
- Transport Fever 中右键有"撤销最近一段 / 退回上一步"的语义，
  需结合 §10.1 的视图操作冲突一并设计。

### 合并后的 GeoJSON
合并两条直线为一个直线、合并两条等半径弧为一个弧 → GeoJSON 自然对应。
