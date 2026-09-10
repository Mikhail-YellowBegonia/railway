# 会话持久化设计（roadmap #2）

> 状态：**已实现**（2026-09-08，roadmap #2 完成；实现见 `model/session.py`、
> `model/geojson_writer/loader.py`、`controller/game_loop.py`，回归 `tests/test_session.py`）。
> 对应 `docs/roadmap.md` 待办 #2「会话持久化」。

## 0. 背景与范围

### 0.1 roadmap 原始诉求
"`S` 键只存轨道几何（`manual_track.geojson`），列车位置/速度/编组、信号布局、
调度指令全部不落盘，退出重进就清空。完整存档系统（相机、命名存档）可以晚做，
但列车状态起步是最低可用性要求。"

### 0.2 现状核对（代码事实）
| 事实 | 位置 | 说明 |
|------|------|------|
| 信号已按坐标持久化 | `geojson_writer/loader` | `"signals"` 顶层字段，存 `{from,to}` 坐标，加载按坐标反查 DirectedEdge |
| `S` 键只写 network + signals | `game_loop.py` | 不写 trains |
| 启动只 load_geojson + load_signals | `game_loop.py` | 不还原 trains |
| edge_id 每次加载重分配 | `rail_network.py` `_next_edge_id` | 列车占用/route/goal 里的 edge_id 不能直存 |
| `wagon_id` 是 uuid、折返/连挂/解挂都保留 | `wagon.py` | 天然稳定，可直接存 |
| `TrainState.v` 是速度真源 | `train_entity.py` | `consist.velocity/acceleration` 是冗余，不存 |
| controller / authority 是运行时派生 | `train_entity.py` / `dispatch.py` | 由 assign_route / resume / 调度层重建，不存 |

## 1. 目标与非目标

### 1.1 本轮目标（完成即 roadmap #2 标 ✅）
1. `S` 键把**全部列车状态**（位置/速度/编组/待走 route/goal/目标速度）连同
   network、signals 一起写入 `manual_track.geojson` 顶层 `"trains"` 字段。
2. 启动时若存档含 `"trains"`，还原全部列车（含行驶中的列车——route/goal/
   剩余距离/v_target 一并恢复，列车继续跑）。
3. 旧存档无 `"trains"` 字段时优雅退化（空列车表，不报错）。
4. 往返稳定：save → load → save 生成的 trains 部分逐字节稳定（容差内）。
5. 自动化回归：模型层 + GameLoop 层往返。

### 1.2 明确不做（后续项）
- 相机位置/缩放、命名存档、存档槽。
- `consist.data_log` 的 payload（当前为空，重建为空 log）。
- 多物理模型类型切换（GameLoop 当前只用 `RealisticElectric`，存类型标记兜底）。
- 行驶中列车与信号预约表的精确恢复——预约表是运行时状态（`BlockManager.
  _reservations`），加载后由调度层从 restored 列车的 occupancy/route 重新推导，
  无需落盘（与"预约只在运行中存在"的语义一致）。

## 2. 序列化面（存什么）

每列车一个 JSON 对象，字段如下（`manual_track.geojson` 顶层 `"trains"` 数组）：

```
{
  "consist": [                    // 车厢列表，按车头→车尾顺序
    {
      "wagon_id": "...",          // uuid，直存
      "length": 20.0,
      "mass": 50.0,
      "P_rated": 3000.0,          // null = 拖车
      "coupler_1_pos": 0.0,
      "coupler_2_pos": 20.0,
      "bogies": [
        {"geometric_role": "leading", "pos": 2.5, "load_share": 0.5, "axle_count": 2},
        {"geometric_role": "trailing", "pos": 17.5, "load_share": 0.5, "axle_count": 2}
      ]
    }, ...
  ],
  "v": 0.0,                       // 当前速度 m/s
  "v_target": 0.0,                // 玩家设定巡航速度（非焦点列车也有，E 阶段保留）
  "occupied": [                   // 车身占用有向边序列（tail→head），按坐标反查
    {"a": [x,y,z], "b": [x,y,z]}, // 每项 = 一条边的两端节点坐标（direction 由
    ...                           //   a→b 顺序隐含：+1 = 沿 node_a→node_b）
  ],
  "occupied_offset": 0.0,         // 车尾在 occupied[0] 内的局部弧长偏移（米）
  "s": 0.0,                       // 车头弧长（相对 occupied_offset）
  "route": [                      // 待走有向边（车头尚未走到的部分），同样按坐标
    {"a": [...], "b": [...]}, ...
  ],
  "remaining_to_goal": 0.0,
  "goal": {                       // null = 无指令；否则三元组
    "edge": {"a": [...], "b": [...]},
    "t": 0.5,
    "direction": 1
  },
  "physics": "realistic_electric" // 物理模型类型标记（未来多模型用，当前固定）
}
```

### 2.1 关键约定

- **归属规则（2026-09-10 定，见 `docs/wagon_centric_data.md`）**：本存档面里
  **车厢条目 = A 桶「车厢域数据」**（物理属性；未来在此基础上逐车厢追加：
  `have_control`/`priority`/调度计划/载货量）——**随车厢走**；而
  `v`/`v_target`/`occupied`/`occupied_offset`/`s`/`route`/`goal`/`remaining_to_goal`
  = **B 桶「运行期派生」的位置快照**，只为恢复"此刻这列编组在哪、怎么跑"，
  随时可从 A 桶 + 轨道重算。**新增字段前先问它属于哪一桶**。
- **有向边按坐标反查**（与 `"signals"` 同一套语义）：存 `{a:[x,y,z], b:[x,y,z]}`
  表示"这条边从 a 端走到 b 端"（direction = +1 若加载后该边的 `node_a` 坐标 ≈ a，
  否则 -1）。反查用 `network.node_id_at(coord, epsilon)` 定位两端节点，再找它们
  之间的边。**存的是"从 a 到 b 的方向"**，不是 node_a→node_b 的几何方向——加载
  后按坐标对齐确定真实 direction，天然免疫 edge 方向的 node_a/node_b 分配顺序。
- **occupied 的窗口语义必须原样保留**：`occupied` 是滑动窗口（tail→head），
  `occupied_offset` 是车尾在 `occupied[0]` 内的偏移，`s` 是车头相对 offset 的
  弧长。三者**必须一起存、一起还原**，缺一会让 `RigidWagonKinematics` 重建出的
  车头位置错位（这是 TrainEntity._build_kinematics 用 `initial_offset=occupied_offset`
  的硬依赖）。
- **行驶中列车**：`route` 非空 + `remaining_to_goal` > 0 + `goal` 非 None 时，
  加载后调用 `assign_route(route, remaining_to_goal, goal)` 重建 controller，恢复
  行驶；`v` 已直接写入 state.v。停放列车（route 空）controller=None。
- **v_target**：TrainEntity.v_target 是运行时字段（非 state），但要恢复"非焦点
  列车按各自 v_target 自主运行"的 E 阶段语义，故存。

## 3. 实现方案

### 3.1 新模块 `model/session.py`（纯模型，无 view/controller 依赖）

```
serialize_trains(trains) -> list[dict]        # TrainEntity -> JSON 对象列表
deserialize_trains(data, network) -> list[TrainEntity]  # 按坐标反查还原
```

复用 `geojson_writer/loader` 现有的坐标匹配语义。`write_geojson` 增加可选参数
`trains=None`（与 `signals` 并列）；`load_geojson` 不变，新增 `load_trains(path, network)`
与 `load_signals` 平行。这样 S 键一行改动、启动一行改动，其余全在 model 层。

### 3.2 反查细节

`deserialize_trains` 对每个 `{a,b}` 有向边：
1. `node_a_id = network.node_id_at(a, epsilon)`，`node_b_id = network.node_id_at(b, epsilon)`；
2. 找 `node_a_id` 的 incident edge 中 `{node_a_id, node_b_id}` 命中的 edge；
3. `direction = +1` 若 `edge.node_a_id == node_a_id`（即 edge 的几何 node_a 就是
   存的 a），否则 `-1`。等价地：`direction = +1 if edge.node_a_id == node_a_id else -1`。
4. 任一坐标找不到节点/边 → 该列车**静默跳过**（容错优先，同 signals 语义）。

### 3.3 一致性与不变量

- **图拓扑不变量不受影响**：序列化/反序列化只读图、只写 trains 列表，绝不
  新建/删除 Node/Edge（列车停在 Edge 中途用标量 offset 表达，不 split）。
- **wagon_id 稳定**：车厢身份跨存档稳定（uuid 直存），折返/连挂/解挂后仍可
  按 id 追踪（未来货物/损耗挂靠不丢）。
- **往返稳定**：save→load→save 的 trains 部分应与首份一致（浮点容差 1e-4，
  与 GeoJSON 弧元数据同一标准）。

## 4. 测试计划

沿用仓库惯例（`PYTHONPATH=. .venv/bin/python tests/test_x.py`）：

### 4.1 模型层（新 `tests/test_session.py`）
1. 停放列车往返：构造多节编组停放列车 → serialize → deserialize → 车头世界
   坐标、occupancy 窗口（occupied/offset/s）、编组（车厢顺序/长度/质量/功率/
   wagon_id）逐项一致。
2. 行驶中列车往返：route 非空 + remaining_to_goal + goal + v 还原后
   `is_moving()` 为真、`remaining_to_goal` 一致、继续 tick 能到达 goal。
3. 解挂后的两段（split_sibling 关系）各自往返：位置连续保持。
4. 旧存档无 `"trains"` → 返回空列表，不报错。
5. 坐标反查方向正确性：direction=+1/-1 两种朝向的列车往返后 `current_direction()`
   一致（覆盖 node_a→node_b 与 node_b→node_a 两种占位方向）。
6. 往返字节稳定：serialize(deserialize(serialize(t))) == serialize(t)（浮点容差）。

### 4.2 GameLoop 层（SDL dummy）
1. 放一列车 → 行驶一段 → `S` 保存 → 新 GameLoop 实例加载 → 列车数量/位置/速度/
   编组一致，且能继续下达指令行驶。
2. 解挂成两段后保存 → 重载 → 仍两列，各自可独立下指令（split_sibling 豁免仍
   生效——注意 split_sibling 是运行时对象引用，重载后需重建互指关系）。

### 4.3 人工验收脚本
放 2 列车（一停一行驶）→ S → 退出重进 → 确认两车都在原位/继续行驶 → 解挂→
S→重进→仍两列。

## 5. 实施步骤（小步，每步可独立验证）

1. **模型层序列化**：`model/session.py` serialize/deserialize + `wagon`/`occupancy`
   字段到 dict；纯模型往返测试（4.1-1/2/3/5/6）。
2. **接入 GeoJSON**：`write_geojson(trains=...)` + `load_trains()`；4.1-4。
3. **接入 GameLoop**：S 键写 trains、启动读 trains；4.2。
4. **split_sibling 重建**：deserialize 后按 wagon 相邻关系或显式存 sibling 标记
   重建互指（4.2-2）。
5. **文档同步**：roadmap #2 ✅ + 记录；CLAUDE.md 补持久化约定；本文状态"已实现"。
6. **人工验收**，通过后进入 roadmap #3。

## 6. 设计决策记录（2026-09-08 全部拍板）

| # | 问题 | 决策 |
|---|------|------|
| 1 | 存档位置 | **单文件**：`manual_track.geojson` 顶层 `"trains"` 字段，与 `"features"`/`"signals"` 平级，S 键一次写完 |
| 2 | split_sibling 持久化 | **不存，动态重建**：重载后对"共享至少一条 occupied 边"的列车对互设 split_sibling——与运行时豁免的前置条件（共享边）语义等价，驶离后本就不需豁免 |
| 3 | couple_approach_partner | **不存**：连挂驶向途中豁免不落盘；重载后目标已在 goal 里，列车续行/重寻路重建，代价仅此一趟可能在信号前停一下 |
| 4 | 失败容错粒度 | **跳过单列**：单列车反查失败静默跳过，其余正常载入（与 signals 逐条容错一致） |

实施按 §5 进行；#2 完成（含测试与文档）后停下等你人工验收，再进入 #3。
