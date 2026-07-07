# 瓦片化数据结构设计（研讨稿）

> 状态：**研讨中，未实施**。本文只定设计与接口，不改代码。
> 架构级改动会触及 `model` 内核，按 CLAUDE.md 约定须先研讨定稿。

---

## 1. 动机

当前 `RailNetwork` 用两个 `dict`（`nodes` / `edges`）平铺存储，所有
**空间查询都是 O(N) 线性扫描**：

| 位置 | 操作 | 频率 |
|------|------|------|
| `rail_network.py:146` `node_id_at` | 遍历所有 Node 找最近 | 建造 / 截断时 |
| `rail_network.py:236` `edge_id_at` | 遍历所有 Edge 算点到段距离 | 删除 / 命中测试 |
| `controller/snap.py` PointSnapProvider | 遍历所有 Node | **每帧** |
| `controller/snap.py` PathSnapProvider | 遍历所有 Edge | **每帧** |
| `controller/snap.py` ParallelSnapProvider `update_reference_points` | 遍历所有 Node 生成 Simple 参考点 | **每帧** |
| `controller/snap.py` ParallelSnapProvider `snap` | 遍历所有 Simple 参考点找最近 | **每帧** |

规模还小时无感，但吸附 Provider 每帧全扫描，随路网增长会线性变慢。

> 注：平行吸附的 **Complex Case 并非独立判定，而是 Simple Case 命中后的
> lazy 后处理拦截**（`_try_lazy_complex`）——只在某个 Simple 参考点被光标
> 命中的那一帧才触发，且已带 `edge_aabb` 粗筛、仅对度数 ≥3 的道岔节点生效。
> 它不属于"每帧全扫描"热点。commit 29ae808 的 60× 优化正是把 Complex
> 从"预计算全部"改成了这套 lazy 拦截。真正的每帧热点是上表两项。
瓦片化的核心目标：**把"每帧全扫描 N"降为"只扫描光标邻近瓦片里的少数元素"**。

---

## 2. 设计目标 / 非目标

**目标**
- 邻近查询（点、边、平行参考点）从 O(N) → O(k)，k = 邻近瓦片元素数
- 为未来"进阶平行吸附最短路径"提供拓扑/空间加速的落点
- 增删边时增量维护索引，不整表重建

**非目标（本轮不做）**
- 不做磁盘分块 / 流式加载（那是地图规模问题，当前是内存内查询问题）
- 不改 `Vec3`、不改弧几何算法
- 不引入 Z 轴分层（仍 XY 平面假设）

---

## 3. 术语澄清：这里的"瓦片"是什么

需求里"瓦片化"一词有歧义，先钉死语义：

- **不是** 渲染瓦片（tilemap 贴图），也不是 GIS 的 slippy-map 瓦片下载。
- **是** 一个纯内存的**空间分区索引（spatial hash / uniform grid）**：
  把 XY 平面切成边长 `TILE_SIZE`（世界单位=米）的方格，每个方格记录
  "哪些 Node / Edge 落在或穿过我"。查询时只看光标所在方格 + 8 邻域。

选 uniform grid 而非四叉树的理由：轨道分布相对均匀、无极端聚集，
均匀网格实现简单、增量维护便宜、常数因子小。若未来出现大范围空区 +
局部密集，再评估换四叉树。

`TILE_SIZE` 建议独立于渲染网格（`view/grid.py` 的自适应档位）：
索引瓦片是固定世界尺寸（例如 `TILE_SIZE = 32 m`），不随缩放变化；
渲染网格是视觉档位，随缩放跳档。两者概念不可混用。

---

## 4. 数据结构

```python
# 归属于 model 层（保持纯几何/数据，无 view/controller 依赖）
TileKey = tuple[int, int]   # (floor(x / TILE_SIZE), floor(y / TILE_SIZE))

class SpatialIndex:
    tile_size: float
    node_tiles: dict[TileKey, set[int]]   # 瓦片 -> Node ID 集
    edge_tiles: dict[TileKey, set[int]]   # 瓦片 -> Edge ID 集（一条边可占多格）
    _edge_footprint: dict[int, set[TileKey]]  # Edge ID -> 它占的瓦片（增量删除用）
```

**Node → 瓦片**：单点，落一格。

**Edge → 瓦片**：边有长度，可能横跨多格。落格策略：
- 直线：对包围盒覆盖的瓦片做线段-格相交（或先用包围盒粗筛，够用即可）
- 圆弧：用弧的包围盒覆盖的瓦片粗筛。弧包围盒可由圆心/半径/起止方向算出
  （保守用整圆包围盒 `center ± R` 也可，代价是多登记几格，查询仍正确）

粗筛（登记偏多）只影响性能不影响正确性——查询命中后仍会做精确的
`project_point_on_edge` 距离判定。所以第一版可以用**包围盒保守登记**，
后续再收紧。

---

## 5. 与 RailNetwork 的关系（关键设计选择）

三种耦合方式，倾向 **B**：

**A. 内嵌**：`SpatialIndex` 作为 `RailNetwork` 的私有字段，在 `add_edge` /
`remove_edge` / `add_node` / `remove_node` / `split_edge_at` 里同步维护。
- 优点：单一真相源，调用方无感，不可能忘记同步。
- 缺点：`RailNetwork` 变胖；纯几何层混入索引逻辑。

**B. 旁挂 + 内部委托（推荐）**：`SpatialIndex` 独立类（`model/spatial_index.py`），
但由 `RailNetwork` 持有一个实例并在增删处调用其 `insert_*/remove_*`。
对外暴露 `RailNetwork.nearby_nodes(pos, r)` / `nearby_edges(pos, r)`，
内部转发给索引。
- 优点：索引逻辑隔离、可单独测试；`model` 纯度保持（索引也是纯数据）；
  调用方（snap.py）只依赖 `RailNetwork` 的新查询方法，不直接碰索引。
- 缺点：仍需在 5 个增删点小心插桩。

**C. 完全外置**：索引放 controller 层，snap 系统自己维护。
- 缺点：网络增删与索引更新分离，极易失同步。**不推荐**。

选 B。理由：`SpatialIndex` 本身是纯数据结构，放 `model` 不违反依赖规则；
由 `RailNetwork` 统一在增删处维护，保证一致性；对外只暴露语义化查询方法。

### 增删插桩点（务必全覆盖，漏一处即失同步）

| 方法 | 索引动作 |
|------|----------|
| `add_node` | `index.insert_node(nid, pos)` |
| `add_edge` | `index.insert_edge(eid, footprint)` |
| `remove_edge` | `index.remove_edge(eid)` |
| `remove_node` | `index.remove_node(nid)`（其 incident edges 已由 remove_edge 清） |
| `split_edge_at` | 复合操作，靠上面四个自然覆盖，无需额外插桩 |

---

## 5.5 更新策略（性能要求：每帧 60fps 调用不卡）

**最终性能要求**：即使某查询逻辑上"只在需要时调用一次"，也必须能承受
每帧（60fps）调用而不卡顿。这条要求决定了以下三点设计（无需急于优化，
但设计上必须留出达成路径）。

### 写是偶发的，读才是每帧的

| 操作 | 何时 | 频率 | 代价目标 |
|------|------|------|----------|
| 写 insert/remove | commit 建造 / 删除 / 截断 | 偶发，每次几个元素 | O(元素占的瓦片数) |
| 读 nearby 查询 | snap 每帧 × 每个 Provider | **每帧** | O(邻近元素数 k)，与 N 无关 |

### 写：增量，绝不整表重建

靠 `_edge_footprint[eid] -> set[TileKey]` 记住每条边占了哪些格：

```
insert_edge(eid, footprint):
    _edge_footprint[eid] = footprint
    for tile in footprint: edge_tiles[tile].add(eid)   # 只碰这几格

remove_edge(eid):
    for tile in _edge_footprint.pop(eid):
        edge_tiles[tile].discard(eid)
        if not edge_tiles[tile]: del edge_tiles[tile]   # 空桶回收
```

代价 = O(该元素占的瓦片数)，与 N 无关。**移动不做原地更新**——拆成
remove + insert（几何变了等于 footprint 全变，原地 diff 不划算）。
`split_edge_at` 内部已调 `remove_edge` + `add_edge`，索引自然跟随，
无需单独插桩（见 §5 插桩表）。

### 读：与 N 无关的 O(k)

```
nearby_edge_ids(pos, radius):
    r = ceil(radius / tile_size)
    cx, cy = tile_of(pos)
    result = set()
    for tx in cx-r .. cx+r:
        for ty in cy-r .. cy+r:
            result |= edge_tiles.get((tx, ty), EMPTY)   # 缺格 = dict miss，零代价
    return result
```

`edge_tiles` 是 dict，**空瓦片不存在于字典里**——遍历空区域只是若干次
`dict.get` miss，不触碰任何元素。10 万条边路网里光标在空旷处的查询，
代价与只有 10 条边时几乎相同。

### 三个达标要点

1. **`TILE_SIZE` 定 k**：太大→每格元素多、退化回全扫描；太小→扫太多空格 +
   footprint 膨胀。经验：与典型查询半径（吸附阈值换算的世界米数）同量级。
   起点 32 m，第 9 步性能验证里实测调。
2. **缩放放大查询半径**：吸附阈值是屏幕像素基准（`editor.md` §3.1），
   缩小视角 → 世界半径变大 → `r_tiles` 变大。性能测试须覆盖
   "极限缩小 + 密集路网"最坏组合。
3. **脏标记 + 缓存（每帧不卡的终极形态）**：平行吸附
   `update_reference_points` 目前每帧重算全部 Simple 参考点。应改为
   **网络变动才重算**：写操作置 `_dirty`，`update` 时不脏则复用缓存。
   建造中拖动鼠标的绝大多数帧网络未变，参考点集合完全不变 → 稳态零计算。
   这比"每帧重算但只算邻近"更彻底。

---

## 6. 对外查询接口

在 `RailNetwork` 上新增（内部委托给 `SpatialIndex`）：

```python
def nearby_node_ids(self, pos: Vec3, radius: float) -> set[int]
def nearby_edge_ids(self, pos: Vec3, radius: float) -> set[int]
```

语义：返回**候选集**（可能含略超 radius 的元素，因瓦片粒度），调用方
仍需精确判定。radius 决定扫描的瓦片邻域范围：`ceil(radius / tile_size)` 圈。

### 改造 snap.py（本轮不改，仅记录目标形态）

三个 Provider 的每帧全扫描替换为：
- PointSnapProvider：`network.nearby_node_ids(cursor, SNAP_THRESHOLD)` 后精算
- PathSnapProvider：`network.nearby_edge_ids(cursor, SNAP_THRESHOLD)` 后精算
- ParallelSnapProvider：参考点由邻近 Node 派生，同样先取邻近集

`node_id_at` / `edge_id_at` 也改为走索引。

---

## 7. 正确性与测试策略

无测试框架，沿用 heredoc 脚本。核心不变式：

1. **索引-真相一致性**：任意增删序列后，`SpatialIndex` 里登记的每个
   (tile, id) 都对应真实存在的 Node/Edge，且每个 Node/Edge 都被登记。
   —— 写一个 `_verify_index_consistency()` 遍历比对，在测试里断言。
2. **查询完备性**：`nearby_*` 返回的候选集 ⊇ 暴力全扫描筛出的真实命中集
   （允许多，不允许漏）。对随机路网 + 随机查询点做对拍。
3. **等价性回归**：接入索引前后，snap 系统对同一光标位置的 `SnapResult`
   必须完全一致。用现有路网跑一遍光标网格采样对拍。

---

## 8. 与"进阶平行吸附"的关系

进阶平行吸附（`editor.md` §3.4 里被逃避的特例）需要"两 Node 间唯一最短路径"
的拓扑查找。那是**图算法**（BFS/Dijkstra over Node-Edge 图），不是空间查询，
瓦片索引不直接解决它。

但两者有协同：
- 平行吸附每帧的 `update_reference_points`（遍历所有 Node）与 `snap` 找最近
  参考点，都可由瓦片索引把范围缩到光标邻近 Node/参考点。
- Complex 的 lazy 后处理 `_try_lazy_complex` 目前遍历所有 Edge + `edge_aabb`
  粗筛；接索引后可直接取参考点邻近的 Edge 候选，省掉 AABB 粗筛那层循环。
- 最短路径的图遍历可由 `adjacent_edges_at` / `connectivity_for` 支撑，
  已存在，不依赖瓦片。

结论：先做瓦片索引（收益即时、风险可控，且能同时收紧平行吸附的每帧
参考点生成与 Complex lazy 拦截两处），最短路径平行建造作为其后的独立课题。

---

## 9. 实施顺序（定稿后）

1. `model/spatial_index.py`：`SpatialIndex` 类 + 单测（构造/增删/查询对拍）
2. `RailNetwork` 插桩 5 个增删点 + `nearby_*` 查询方法 + 一致性自检
3. 回归对拍：接索引前后 snap 结果等价
4. 改 `snap.py` 三 Provider + `node_id_at`/`edge_id_at` 走索引
5. 性能验证：构造大路网，测每帧 snap 耗时对比

每步独立可测、可回滚。第 1–2 步不改动任何现有调用方，纯增量。


