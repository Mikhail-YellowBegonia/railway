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

#### 状态流转伪代码

```python
class BuildState(Enum):
    IDLE = auto()
    ACTIVE = auto()

class Editor:
    build_state: BuildState = BuildState.IDLE
    build_m1: Vec3 | None = None       # M1 世界坐标
    build_m1_is_node: bool = False     # M1 是吸附到既有节点还是空白
    build_t1: Vec3 | None = None       # M1 处的切线方向 (有则为 Case2/3, 无则为 Case1)

    def handle_click(self, world_pos: Vec3, snap: SnapResult):
        if self.mode == EditMode.BUILD:
            if self.build_state == BuildState.IDLE:
                self._start_build(world_pos, snap)
            else:
                self._commit_build(world_pos, snap)

        elif self.mode == EditMode.DELETE:
            self._delete(snap)

    def handle_cancel(self):
        if self.build_state == BuildState.ACTIVE:
            self.build_state = BuildState.IDLE
            self.build_m1 = None
            self.build_t1 = None

    def handle_move(self, world_pos: Vec3, snap: SnapResult):
        if self.build_state == BuildState.ACTIVE:
            self.preview_m2 = world_pos   # 用于渲染预览
            self.preview_snap = snap
```

### 1.3 Delete 模式

**操作单位：边（Edge）**。

设计原则：用户期望与"线要素"互动，而不必关心节点的存在。
DELETE 模式因此简化为：

- 点击吸附命中的边 → 删除该边
- 端点节点 是否一并清理 由程序自动处理

#### 自动清理规则

删除一条边后，遍历其原本的两个端点：
- 若端点变为孤立（`connection_count == 0`）→ 一并删除
- 若端点仍连接其它边（`connection_count >= 1`）→ 保留

只要遵循"删除作用于 Edge"的约束，可以保证两条底线：
1. **没有孤立节点**：每次删除都会扫描端点
2. **没有无头边**：边的端点要么仍存在，要么本就和这条边一起被删除

不允许直接对节点执行删除。点吸附在 DELETE 模式下仅用于"明确指出鼠标命中的是节点而非边"，从而避免在节点附近误删邻接边——此时不响应点击。

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

```python
class PointSnapProvider:
    threshold: float = 0.3        # 世界单位
    enabled: bool = True

    def snap(self, world_pos: Vec3, network: RailNetwork) -> SnapResult | None:
        best_node_id = None
        best_dist = float('inf')
        for node_id, node in network.nodes.items():
            d = node.position.distance_to(world_pos)
            if d < self.threshold and d < best_dist:
                best_dist = d
                best_node_id = node_id

        if best_node_id is None:
            return None

        node = network.nodes[best_node_id]
        tangent = self._tangent_at_node(node, network)  # 见 §2.2
        return SnapResult(
            snapped=True,
            position=node.position,       # 位置覆盖为 Node 坐标
            tangent=tangent,
            snapped_node_id=best_node_id,
        )
```

### 3.3 PathSnapProvider（路径吸附）

```python
class PathSnapProvider:
    threshold: float = 0.3        # 世界单位
    enabled: bool = True

    def snap(self, world_pos: Vec3, network: RailNetwork) -> SnapResult | None:
        best_edge_id = None
        best_t = 0.0
        best_pos = Vec3()
        best_dist = float('inf')

        for edge_id, edge in network.edges.items():
            node_a = network.nodes[edge.node_a_id]
            node_b = network.nodes[edge.node_b_id]
            t, proj_pos, dist = self._project_on_edge(world_pos, edge, node_a, node_b)
            if dist < self.threshold and dist < best_dist:
                best_dist = dist
                best_edge_id = edge_id
                best_t = t
                best_pos = proj_pos

        if best_edge_id is None:
            return None

        tangent = self._tangent_along_edge(best_edge_id, best_t, network, world_pos)
        return SnapResult(
            snapped=True,
            position=best_pos,
            tangent=tangent,
            snapped_edge_id=best_edge_id,
            snapped_edge_t=best_t,
        )
```

**切线方向选择**（路径上点的正反切选择）：

给定路径上的点及其两个候选切线方向 `t_forward` 和 `t_reverse`：
- 在 `BUILD_ACTIVE` 状态且有 M1 时：取与 `(world_pos - M1)` 夹角较小的方向
- 在 `BUILD_IDLE` 状态时：不产生切线（tangent 为 None）

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

```python
def _start_build(self, world_pos: Vec3, snap: SnapResult):
    # 1. 建造前检查：吸附关闭且附近有元素 → 拒绝
    if not snap_enabled and self._has_nearby_element(world_pos):
        self._show_refusal()     # 光标红色警告，不进入 BUILD_ACTIVE
        return

    # 2. 确定 M1
    if snap.snapped:
        self.build_m1 = snap.position
        self.build_m1_is_node = (snap.snapped_node_id is not None)
        if snap.tangent is not None:
            self.build_t1 = snap.tangent    # Case 2 或 Case 3
        else:
            self.build_t1 = None            # Case 1
    else:
        self.build_m1 = world_pos
        self.build_m1_is_node = False
        self.build_t1 = None                # 孤立点, Case 1

    # 3. 进入建造活跃状态
    self.build_state = BuildState.ACTIVE
    self.preview_geometry = None  # 待每帧更新
```

`_has_nearby_element(world_pos)` 的判断逻辑：

```python
def _has_nearby_element(self, pos: Vec3) -> bool:
    # 检查是否有 Node 在 WARNING_THRESHOLD 内
    if self.network.node_id_at(pos, WARNING_THRESHOLD) is not None:
        return True
    # 检查是否有 Edge 在 WARNING_THRESHOLD 内
    if self.network.edge_id_at(pos, WARNING_THRESHOLD) is not None:
        return True
    return False
```

其中 `WARNING_THRESHOLD` 应略大于 `SNAP_THRESHOLD`（例如 `SNAP_THRESHOLD = 0.3`，`WARNING_THRESHOLD = 0.5`），确保"肉眼可见但未触发吸附"的距离能被警告覆盖。

### 4.2 提交建造（BUILD_ACTIVE 中按下鼠标）

```python
def _commit_build(self, world_pos: Vec3, snap: SnapResult):
    # 1. 确定 M2
    if snap.snapped:
        m2 = snap.position
        t2 = snap.tangent
    else:
        m2 = world_pos
        t2 = None

    # 2. 判断 Case 并计算几何 → 产生 ConstructionPlan
    plan = self._compute_plan(
        m1=self.build_m1, t1=self.build_t1,
        m2=m2, t2=t2,
        snap=snap,
    )

    # 3. 如果 plan 要求截断（M2 在既有边中间），先执行截断
    if plan.split_edge_id is not None:
        self._split_edge(plan.split_edge_id, plan.split_at_t)

    # 4. 应用 plan 到网络（建 Node/Edge）
    self._apply_plan(plan)

    # 5. 回到 IDLE
    self.build_state = BuildState.IDLE
    self.build_m1 = None
    self.build_t1 = None
```

### 4.3 ConstructionPlan

```python
@dataclass
class ConstructionPlan:
    case: int                        # 1, 2, or 3
    m1: Vec3
    m2: Vec3
    node_a_id: int | None            # M1 端的 Node ID（None 则需新建）
    node_b_id: int | None            # M2 端的 Node ID（None 则需新建）
    edge_geometry: list[Vec3]        # [] 直线, [B] 弧, [B1, B2] biarc
    split_edge_id: int | None        # 需要截断的 Edge ID
    split_at_t: float | None         # 截断参数 t ∈ [0,1]
```

### 4.4 Case 1: 空白→空白（唯一直线）

```
输入：M1, M2（无切线）
输出：直线 Edge，geometry = []
```

伪代码：

```python
def _case1_plan(m1, m2):
    return ConstructionPlan(
        case=1,
        m1=m1, m2=m2,
        node_a_id=None,          # 新建 Node
        node_b_id=None,          # 新建 Node
        edge_geometry=[],
        split_edge_id=None,
    )
```

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

```
输入：M1, M2, T1, T2
输出：两条圆弧首尾相接，共用一个中间点 M_mid
```

**等半径策略：**

两条弧的半径相等：`R1 = R2 = R`。

1. 弧 1：起于 M1（切于 T1），终于 M_mid
2. 弧 2：起于 M_mid（切于 T_mid），终于 M2（切于 T2）
3. 两弧在 M_mid 处切线连续（G1），即 T_mid 一致
4. 由于 `R1 = R2 = R`，且约束数量 = 未知数数量，可解

**算法概要（等半径 Biarc）：**

令 `d = M2 - M1`（端点连线向量）。

1. 构造两条射线：
   - L1：过 M1，方向 `perp(T1)`（圆心方向线）
   - L2：过 M2，方向 `perp(T2)`
2. 弧 1 的圆心 O1 在 L1 上：`O1 = M1 + r * perp(T1)`
3. 弧 2 的圆心 O2 在 L2 上：`O2 = M2 + r * perp(T2)`（`r` 有符号）
4. 约束：M_mid = 弧 1 终点 = 弧 2 起点，且 T_mid 连续
   等价于 `|O2 - O1| = 2R`（两圆心距离等于 2R，即两弧相切且方向恰好衔接）

   等等——更精确的约束：两弧在 M_mid 处 G1 连续 ⇔ `O1, M_mid, O2` 三点共线，且 `|O1 - M_mid| = |O2 - M_mid| = R`。

   这等价于 `|O2 - O1| = 2R`，且 M_mid 为 O1O2 中点。

5. 代入：
   ```
   |(M2 + r * perp(T2)) - (M1 + r * perp(T1))| = 2|r|
   |d + r * (perp(T2) - perp(T1))| = 2|r|
   ```

   展开得关于 `r` 的二次方程，取 |r| 较小的解（最小化曲率）。

6. 解得 r 后：
   - O1 = M1 + r * perp(T1)，O2 = M2 + r * perp(T2)
   - M_mid = (O1 + O2) / 2
   - T_mid = perp(M_mid - O1)（弧 1 在 M_mid 的切向）
   - B1 = 弧 1 的切线交点（O1 到两切点 M1, M_mid 的切线交点）
   - B2 = 弧 2 的切线交点（O2 到两切点 M_mid, M2 的切线交点）

7. Edge 几何：`geometry = [B1, B2, M_mid]` —— 待我们确定数据模型如何支持双弧（参见 §4.9）。

### 4.7 截断（Edge 分割）

当 M2 通过路径吸附命中一条既有 Edge 的内部点（而非节点）时触发。

```python
def _split_edge(self, edge_id: int, t: float):
    edge = self.network.edges[edge_id]
    node_a = self.network.nodes[edge.node_a_id]
    node_b = self.network.nodes[edge.node_b_id]

    # 1. 计算截断点世界坐标和在该点的切线
    split_pos = self._point_at_t(edge, node_a, node_b, t)

    # 2. 创建新 Node
    new_node = self.network.add_node(split_pos)

    # 3. 原 Edge 移除，替换为两个新 Edge
    if edge.is_arc:
        # 将圆弧在参数 t 处分成两段圆弧
        e1_geometry, e2_geometry = self._split_arc_geometry(edge, node_a, node_b, t)
    else:
        # 直线分裂为两段直线
        e1_geometry = []
        e2_geometry = []

    self.network.remove_edge(edge_id)
    self.network.add_edge(node_a, new_node, e1_geometry)
    self.network.add_edge(new_node, node_b, e2_geometry)
```

圆弧分裂：给定弧 Edge 在参数 t ∈ [0,1] 处截断：
- 计算截断点在弧上的世界坐标 P
- 弧 1：从 node_a 到 P，B1 = 弧 1 的切线交点（可反算）
- 弧 2：从 P 到 node_b，B2 = 弧 2 的切线交点（可反算）
- 注意：截断点 P 的切线方向与原弧在 P 处一致（无损分割）

### 4.8 删除与合并

Delete 模式点击一个中间 Node（connection_count == 2）：

```python
def _delete_intermediate_node(self, node_id: int):
    node = self.network.nodes[node_id]
    edge_ids = list(node.incident_edge_ids)
    if len(edge_ids) != 2:
        return  # 不处理端点或道岔

    e1 = self.network.edges[edge_ids[0]]
    e2 = self.network.edges[edge_ids[1]]

    # 检查合并条件
    if not self._can_merge(e1, e2):
        return  # 拒绝删除

    # 找出两个端点（非当前 node 的端点）
    other_a = e1.node_a_id if e1.node_a_id != node_id else e1.node_b_id
    other_b = e2.node_a_id if e2.node_a_id != node_id else e2.node_b_id

    # 移除两条边和中间节点
    self.network.remove_edge(edge_ids[0])
    self.network.remove_edge(edge_ids[1])
    self.network.remove_node(node_id)

    # 创建合并后的新边
    if e1.is_arc and e2.is_arc:
        merged_geometry = self._merge_arc_geometry(e1, e2, node, other_a, other_b)
    else:
        merged_geometry = []
    self.network.add_edge(
        self.network.nodes[other_a],
        self.network.nodes[other_b],
        merged_geometry,
    )

def _can_merge(self, e1: Edge, e2: Edge) -> bool:
    if not e1.is_arc and not e2.is_arc:
        return True  # 两条直线永远是共线的（因为我们只删除连接数为 2 的中间点）
    if e1.is_arc and e2.is_arc:
        return abs(e1.arc_radius - e2.arc_radius) < 1e-6
    return False  # 一条直线一条弧不可合并
```

端点 Node（connection_count == 1）直接删除并连带 Edge。

### 4.9 双弧 Edge 的数据模型考量

当前 Edge 模型只支持"一段弧"（geometry 中单 B 点）。Biarc 需要两段弧。选择：

**方案 A（暂定）**：Biarc 展开为两个连续的 Edge + 一个中间 Node。实现简单，对现有模型无侵入。

```
Edge.geometry 保持 [] 或 [B]，不变
Biarc = 连续两个 Edge，中间自动插入一个隐藏 Node
```

这样做的好处：无需修改现有模型、渲染器、GeoJSON 读写。
坏处：图上多了一个 Node（但这个 Node 是"虚拟"的——用户下次点击可选中它，编辑行为一致）。

**是否接受方案 A**？如接受，Biarc 直接转化为两个 Edge 创建。

---

## 5. 几何工具函数

### 5.1 点在边上的投影

```python
def project_point_on_edge(pos: Vec3, edge: Edge, node_a: Node, node_b: Node) -> tuple[float, Vec3, float]:
    """返回 (t, projected_position, distance)"""
    if edge.is_arc:
        return _project_on_arc(pos, edge, node_a, node_b)
    else:
        return _project_on_line(pos, node_a.position, node_b.position)

def _project_on_line(p: Vec3, a: Vec3, b: Vec3) -> tuple[float, Vec3, float]:
    ab = b - a
    ab_len_sq = ab.length_squared()
    if ab_len_sq == 0.0:
        return 0.0, a, p.distance_to(a)
    t = max(0.0, min(1.0, (p - a).dot(ab) / ab_len_sq))
    proj = a + ab * t
    return t, proj, p.distance_to(proj)

def _project_on_arc(p: Vec3, edge: Edge, node_a: Node, node_b: Node) -> tuple[float, Vec3, float]:
    """将 p 投影到圆弧上"""
    center = edge.arc_center
    radius = edge.arc_radius
    to_p = p - center
    if to_p.length() == 0.0:
        return 0.0, node_a.position, radius

    # 将 to_p 缩放到半径，夹紧到弧的角度范围
    on_circle = center + to_p.normalize() * radius

    # 将 on_circle 夹紧到弧的起止角度之间
    oa = node_a.position - center
    oc = node_b.position - center

    # 用 oa 和 oc 定义的角度范围 [0, edge.arc_angle_rad] 中插值
    angle_p = signed_angle(oa, on_circle - center, edge.arc_normal)
    clamped_angle = max(0.0, min(edge.arc_angle_rad, angle_p)) if edge.arc_angle_rad >= 0 else min(0.0, max(edge.arc_angle_rad, angle_p))

    # 绕 normal 旋转 oa 得到投影点
    rot_axis = edge.arc_normal
    rotated = rotate_around(oa, rot_axis, clamped_angle)
    projected = center + rotated

    t = clamped_angle / edge.arc_angle_rad if edge.arc_angle_rad != 0 else 0.0
    return t, projected, p.distance_to(projected)
```

### 5.2 切线计算

```python
def tangent_at_node(node: Node, edge: Edge, network: RailNetwork) -> Vec3 | None:
    """返回 Node 处沿 Edge 的外向切线方向"""
    other = network.nodes[edge.node_b_id if edge.node_a_id == node.node_id else edge.node_a_id]

    if not edge.is_arc:
        return (other.position - node.position).normalize()

    center = edge.arc_center
    to_node = node.position - center
    tangent = edge.arc_normal.cross(to_node)   # 平面内垂直于半径 = 切线
    outward_dir = (other.position - node.position).normalize()
    if tangent.dot(outward_dir) < 0:
        tangent = tangent * -1.0
    return tangent.normalize()
```

---

## 6. 预览渲染

### 6.1 预览协议

在 `BUILD_ACTIVE` 状态下，渲染器收到一个 `PreviewGeometry` 结构：

```python
@dataclass
class PreviewGeometry:
    m1: Vec3
    m2: Vec3
    case: int                  # 1, 2, or 3
    edge_geometry: list[Vec3]  # 待建 Edge 的 geometry
    valid: bool                # 当前计算是否合法
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
| **4** | **删除-合并** | 待实现 | 删除 connection_count==2 的中间节点时，检查两侧边是否可合并（共线直线 / 等半径等圆心弧）。合并成功则创建新边，否则拒绝删除。 |
| **5** | **Case 3 Biarc** | 待实现 | 等半径双弧建造（方案 A：双 Edge + 中间 Node）。数学求解：二次方程解 r，计算 M_mid。 |
| **6** | **Edge 截断** | 待实现 | M2 路径吸附到既有边的内部点时，插入新节点分裂原边为两段（直线/弧）。截断后的弧需重新计算 B 点保证 GeoJSON 往返一致。 |

---

## 8. 附录：同类参考

### Biarc 中间点策略（运输类游戏常用）

- **等半径**（equal radii）：两弧半径相等，对称美观
- **最小化最大曲率**（minimize max curvature）：找使两弧中较大曲率最小化的解
- **指定半径**（fixed radius）：允许用户指定其中一段弧的半径
- **等弧长**：两弧弧长接近

初期实现：**等半径策略**。其他策略可后续添加为 Biarc 策略的可选参数。

### 曲率限制

开发初期不实施最小/最大曲率半径限制。接口预留方式：

```python
def compute_case2_plan(m1, t1, m2, min_radius=None, max_radius=None):
    ...
    if min_radius is not None and R < min_radius:
        return None   # 拒绝
```

---

## 9. GeoJSON 兼容性

### 双弧写入
若采用方案 A（双 Edge + 中间 Node），GeoJSON 直接输出两个 3 点 LineString，无需特殊处理。

### 截断后的弧元数据
截断圆弧时需重新计算两半段的 B 点，确保写回 GeoJSON 后重载的弧元数据完全相等。截断算法必须精确（无信息丢失）。

### 合并后的 GeoJSON
合并两条直线为一个直线、合并两条等半径弧为一个弧 → GeoJSON 自然对应。
