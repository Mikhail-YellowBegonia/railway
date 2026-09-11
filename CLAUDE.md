# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Python railway sandbox game inspired by Transport Fever 2 + AutoCAD. MVC architecture.

**编辑器已稳固**（Steps 0–6 + 吸附 + 空间索引全部完成，权威规格 `docs/editor.md`）。
当前重心不在编辑器，而在**补齐 demo 缺口并发布**：最大缺口是**计划式自动驾驶**
（demo 要做、设计待研讨）。**进度总账与优先级看 `docs/roadmap.md`**（含
「计划式自动驾驶」专项章节与「当前重心」）。

## demo 范围拍板（2026-09-10，用户）

- **demo 要做（最大缺口）**：**计划式自动驾驶**——创建计划并跟随计划。现状是
  "去哪里"仍由玩家逐次人工下达（右键设 goal / 车钩连挂指令 / `↑` 设巡航），
  没有"计划"这一层。**设计待研讨、尚无结论**，权威研讨记录见 `docs/roadmap.md`
  「计划式自动驾驶」（含 10 条待研讨问题）。**不得照抄任何一个同类游戏**，
  先研讨再实现；**研讨定稿前不要当"已规划"引用，也不要先写实现代码**。
  **前置依赖**：**POI 方案已给出并立档（`docs/poi.md`，2026-09-10）**——三类 POI
  （无向点/有向点/路段）、建筑=POI 集合、**最小可行范围 = 只做"有向点"且计划直接
  指向 POI**；调度命令的下达依据由此确定。落地顺序仍是 POI → 计划数据类型 →
  接入列车逻辑。
- **demo 要做（与上一条绑定）**：**编组元数据交割复核**（roadmap #9）——本项目
  主张**以车厢为最小单位**（同类游戏多以编组为原子单位），G 阶段原本就是为
  "拆分/拼合时元数据与调度计划交割不当"这一风险准备的。因顺序倒置（连挂/解挂
  先于计划系统落地）该风险未真正暴露，但现在要**回头补检查**：roadmap #9 的
  4 项任务是 ①检查数据结构（**已执行，只汇报不修复**）②构思 POI ③新增调度
  计划数据类型 ④接入列车逻辑；②~④ 与 #7 合并研讨。详见 `docs/roadmap.md`
  「G 阶段背景与四项任务」与 `docs/train_control.md` G 章节。
- **demo 最后一步**：**开源发布**（demo 完成同时开源；许可协议 / README /
  仓库卫生待办）。
- **demo 不做**：LOD（代码零实现）、真实物理扩展（roadmap #3）、无物理倒车
  （负向推进原语）、碰撞模型（无碰撞是有意取舍，见下「已知限制：调车/倒车」）、
  **货物/装载逻辑**（roadmap #10，2026-09-10 用户拍板"先缓缓"；参照：OpenTTD
  每次更新定量装载但瞬时结算、分配由 CargoDist 指导、其载具是有 tick 的）。
  ⚠ **注意"瓦片化数据结构（tiling）"不在这一列**——它**已经实现**了，就是
  `model/spatial_index.py`（uniform grid，340× 加速）；其设计稿
  `docs/tiling.md` 实施完成后已主动删除（`b8c9209`），指向它的 9 处陈旧引用
  （`model/spatial_index.py` 5 处 / `model/rail_network.py` 3 处 /
  `docs/editor.md` §3.4）已于 2026-09-10 全部清理。不追加投入的理由同样是
  "无性能瓶颈"。
- **未拍板是否 demo 不做**：UI 图形化（roadmap #5）、建造时自动合并临近 Node
  （#6）——文档里只写"排最后"，没有"demo 不做"结论。
- 约定：`docs/roadmap.md` 待办清单每项**必须显式标注**三种状态之一（✅ 已完成 /
  ⚠ demo 不做 / 未拍板），不留空——此前 #3~#6 正因留空而与代码实况脱节。

## Commands

- **Run game**: `uv run python main.py` (loads `test_track.geojson`)
- **Add deps**: `uv add <package>`
- **Ad-hoc tests**: `uv run python -c "..."` — there is no test framework; the
  established pattern is heredoc scripts that import editor/network modules,
  drive them programmatically, and assert. Examples appear throughout the
  conversation history when verifying each Step.
- **Regression scripts**: `tests/test_*.py` — 每个脚本自带断言并打印 ✅，无 test
  runner，逐个直接执行：
  `PYTHONPATH=. SDL_VIDEODRIVER=dummy .venv/bin/python tests/test_x.py`。
  覆盖几何/编辑、寻路、运动学与刚体车厢、物理、信号（放置 / 闭塞 / 预约 /
  远近场 / 调度）、连挂解挂、会话持久化、DELETE 保护、折返、PLAY 指令。
  改动 `model/` 或 `controller/` 后跑全套。（注意：某些沙箱环境里 `uv run` 会因
  `~/.cache/uv` 不可写而失败，直接调 `.venv/bin/python` 更稳。）

There are no lint, typecheck, or unit-test commands configured. Don't add them
without asking.

## Architecture

```
model/       Pure data + geometry, no view/controller deps
  vec3.py            Vec3 (3D arithmetic)
  rail_network.py    Node, Edge, RailNetwork (graph + connectivity + split_edge_at + turn_allowed)
  geom_utils.py      All geometry: projections, tangents, solve_case2_arc,
                     solve_biarc, merge predicates, split_arc_b_points
  geojson_loader.py  / geojson_writer.py — round-trip-safe arc serialization
  pathfinding.py     Edge-based Dijkstra, Path dataclass, turn_allowed integration
  spatial_index.py   Tile-based spatial index (340× speedup, 60fps保障)

view/        pygame-ce rendering only
  camera.py          World→screen mapping, pan/zoom
  renderer.py        Network drawing + preview (dashed straights/arcs/biarcs)

controller/  Wires it together
  game_loop.py       Event loop, key/mouse routing, modifier-key polling
  editor.py          Editor state machine (modes + build substates)
  snap.py            SnapSystem: Point/Grid/Path/Parallel SnapProviders + 长度/角度吸附
  build_plan.py      ConstructionPlan / PreviewGeometry dataclasses
```

Dependency rule: `model` must stay pure (no view/controller imports). `controller`
imports from `model` and `view`. `view` only sees `model`.

## Graph topology invariant (learned the hard way)

**`Node`/`Edge` only change when infrastructure changes (editor BUILD/DELETE,
future signal placement). Runtime train state never touches graph topology** —
it's always expressed as a scalar offset on an existing Edge (`(edge_id, t)`,
`OccupancyState.occupied_offset`/`s`). No mid-route stop, no couple/decouple,
no pathfinding call inserts a node or splits an edge.

This was learned twice: pathfinding used to `split_edge_at` at the start and
goal to give Dijkstra exact endpoints, producing throwaway nodes/edges that
needed GC. Both were removed (`start_offset`/`end_offset` scalar corrections
instead) — see `5ea8f16` (start) and `e3341bb` (goal) in git log. There is
now no "GC problem" to solve because there is nothing to collect: the graph
is never mutated by anything that isn't a deliberate infrastructure edit.
Fixed-block signaling (see `docs/train_control.md`) doesn't change this —
block occupancy is Edge-list granularity (`OccupancyState.occupied`), it
never needs to know *where* on an Edge a train sits. Signal placement itself
*is* infrastructure and should split edges like BUILD does.

## Editor state machine

Top-level modes: **IDLE** / **BUILD** / **DELETE**. BUILD has substates
**BUILD_IDLE** / **BUILD_ACTIVE** (after first click). All rejection paths preserve
current state — never auto-revert on bad input. Only "not buildable" gets explicit
visual feedback (red preview); other rejections are silent. See `docs/editor.md` §1, §4.0.

## Build cases

| Case | Trigger | Geometry |
|------|---------|----------|
| 1 | No T1 candidate | Free straight, `edge_geometry=[]` |
| 2 | T1 only (M1 has tangent, M2 doesn't) | Single arc, `[B]` |
| 3 | T1 + T2 candidates | Equal-radius biarc → **two consecutive Edges + middle Node** (Plan A) |

Tangent semantics: T1/T2 candidates point **away from the other end** of their
incident edge. The "best" candidate at commit time is the one with max dot product
against `(M2 - M1)` for T1, and enumerated per-candidate for T2 (biarc may want
reverse-direction T2 in C/S shapes — never pre-filter T2 by direction).

Force-straight (LSHIFT held): degenerates to "straight along T1, length controlled
by mouse". M2 is projected onto the (M1, T1) ray; **direction comes from existing
track**, not cursor.

## Edge geometry conventions

- `Edge.geometry == []` → straight; length = `|node_a - node_b|`.
- `Edge.geometry == [B]` → arc through `node_a → B → node_b`, where B is the
  intersection of the two endpoint tangents. `RailNetwork.add_edge` auto-computes
  `arc_center / arc_radius / arc_angle_rad / arc_start_dir / arc_normal`.
- Biarc never produces a single edge with two B points — it always splits into
  two consecutive Edges + an automatically-inserted middle Node.
- All arc math assumes the **XY plane**: `PLANE_NORMAL = +Z`, `arc_normal ∈ {+Z, -Z}`.

## Snap system

`SnapSystem.snap(world_pos, network, reference_pos)` tries Providers in order
（`controller/snap.py`，顺序与 `SnapSystem.snap` 实现一致）：
1. **PointSnapProvider** (threshold 0.3) — snaps to nearest Node, returns all
   incident-edge tangent candidates (endpoint=1, switch=N, isolated=0)
2. **GridSnapProvider**（格点吸附，`G` 键）— 公制格点
3. **ParallelSnapProvider**（平行吸附，`P` 键，spacing 5.0）— Simple / Complex Case
4. **PathSnapProvider** (threshold 0.3) — snaps to closest Edge, returns
   `[forward, reverse]` tangent candidates

另有**长度吸附（`L` 键，仅直线）/ 角度吸附（`A` 键，仅单弧）**：由 `Editor` 的
`length_snap_enabled` / `angle_snap_enabled` 标志实现（`controller/editor.py`），
与 `G` 格点吸附**互斥**，不是 SnapProvider。

`reference_pos` is M1 in BUILD_ACTIVE, None otherwise. Used by PathSnapProvider
to pick the "best" tangent for live preview display (does not affect the
candidate list).

## DELETE mode

Hover priority: **Node > Edge**.

- Click an edge → remove it; isolated endpoints get cleaned up.
- Click a `connection_count == 2` Node → attempt merge (silent if invalid).
  Strict checks: collinear straights (cos ≥ 1−1e-6) OR same-center / same-radius
  / same-direction / tangent-continuous arcs. Mixed straight+arc never merges.

The two invariants after any DELETE: **no orphan nodes, no headless edges**.

## Truncation (Step 6)

Path-snap on M1 or M2 splits the underlying Edge at commit time via
`RailNetwork.split_edge_at(edge_id, t)`. Arc splits use `split_arc_b_points`
(tangent-intersection inverse) to keep `arc_center / arc_radius / arc_normal`
identical across split→merge round trips. M1 and M2 snapping to the same edge
is rejected silently.

## GeoJSON round-trip

LineString features: 2 points = straight, 3 points `[A, B, C]` = arc with B as
the tangent-intersection. Constraints: A/B/C not collinear, `|BA| = |BC|`.
Arc metadata (center/radius/normal) is reconstructed deterministically from B
on load — round-trip is bit-stable for arc geometry within 1e-4.

## 会话持久化 (Session persistence, roadmap #2)

见 `docs/session_persistence.md`（完整规格与决策记录）。核心约定：

- `S` 键把 network + signals + **trains** 一起写进 `manual_track.geojson`；
  启动时 `load_geojson` + `load_signals` + `load_trains` 还原。
- **有向边不存 edge_id**（每次加载重分配），而是存"从 a 端走到 b 端"的两端
  节点坐标 `{"a":[...],"b":[...]}`，加载按坐标反查节点→找边→据 a→b 顺序恢复
  direction（`model/session.py::_directed_to_coords/_coords_to_directed`，与
  信号持久化同一套坐标匹配语义）。列车 `occupied`/`route`/`goal` 里的
  DirectedEdge 都走这个转换。
- 存：`consist`（wagon_id uuid 直存 + 几何/质量/功率/转向架）、`v`、`v_target`、
  `occupied`/`occupied_offset`/`s`（窗口三件套必须一起存，否则
  `_build_kinematics` 重建的车头位置错位）、`route`、`remaining_to_goal`、
  `goal`、`physics` 类型标记。
- 不存：`controller`/`authority_remaining`（运行时派生，行驶中列车由
  `assign_route` 重建 controller）、`consist.velocity/acceleration/data_log`
  （冗余/空）、`split_sibling`/`couple_approach_partner`（运行时对象引用）。
  `split_sibling` 加载后由 `rebuild_split_siblings` 按"共享 occupied 边"动态
  重建（与豁免前置条件语义等价）；`couple_approach_partner` 由续行/重寻路重建。
- 容错：单列车反查失败静默跳过（与 signals 逐条容错一致）。

## 已知限制：调车/倒车（2026-09 #1 验收反馈记录）

连挂/解挂已完成，编组作业局限在"前进对接 + 原方向驶离"。**仍无物理倒车
（推挽/负向推进）**：`advance_occupied_path` 只沿 route 正向推进，没有负向推进
原语，列车不能"倒着开"（车厢物理位置不动、车尾在前）。因此 AB 顺序相连解挂后，
A 在 B 前方想"倒出去"时只能先掉头（R 键）再前进——若前方是 B 等列车，无碰撞
模型下表现为**视觉穿过（非碰撞）**（用户已知问题 1，**有意取舍不修**）。

**玩家手动折返已提供**（2026-09 临时追加功能 1，`reverse_in_place` 改造，
回归 `tests/test_reverse_in_place.py`）：
- PLAY 模式选中**停放**列车按 `R` 原地掉头；**任意位置**可用，但要求车身所在
  轨道段无道岔（`connection_count() >= 3` 的节点），否则拒绝并提示。
- 语义（用户拍板，勿回退）：**只切换前进方向（逻辑），不反转列车编组（物理）**
  ——`consist` 顺序不变（**不调 `reversed_consist`**），车厢在轨道上的前后位置
  随掉头对调（等价整列车原地旋转 180°）。历史：早期实现同时反转 consist，
  会把编组顺序倒过来，与"掉头只换向、不改编组"的现实调车语义不符。
- `reverse_in_place() -> bool`（True 成功 / False 拒绝并打印原因）；调用后清
  route/remaining；行驶中（controller 非 None）拒绝。指令场景（寻路折返标记
  消费）仍走 `_do_auto_reversal` → 同一原语。
- 将来做真实调车（倒车/推挽）需先补"负向推进原语"。详细认知记录在
  `docs/roadmap.md`「已知限制与未来方向」。

## DELETE 保护：拒绝删除列车占用/预约的路段（2026-09）

用户报"删除有列车的轨道时游戏崩溃"。根因：列车 `occupied`/`route` 里的
`edge_id` 一旦被删即成悬空引用，下一帧 `TrainDispatcher.tick`
（`network.edges[eid]`）直接 KeyError（已复现）。修复（与 Transport Fever 2 /
OpenTTD 一致）：DELETE 模式点击轨道前由 `GameLoop._rail_delete_blocked_reason`
按 editor 的命中优先级（Node > Edge）检查命中对象是否落在
`_train_locked_edges()`（所有列车 `occupied ∪ route`）内 → 拒绝并打印提示；
`Editor` 保持对列车零耦合。自由路段照常可删。回归
`tests/test_delete_guard.py`。

## 连挂/解挂可靠性现状（2026-09 用户结论，勿过度承诺）

连挂/解挂在 bug2 现场修复（137fc8d 调车全放行 + 6a5e6b6 折返重算 +
ab8a5ea 门槛放宽）后**能用了**，但用户明确表示：实现仍不算鲁棒、对可靠性
保留意见；**demo 愿景不对连挂/解挂的可玩性作保证**。

**联合场景人工实测通过（2026-09-10，`c58e147` 之后）**：连挂 → 解挂 → 折返 →
寻路 → 再连挂 在同一张图上串起来实测符合预期，roadmap #1 的待复测项全部关闭。
**该结论没有自动化回归**——联合场景依赖真实 GUI 操作时序与多车实时交互，难以
形式化为脚本用例（用户 2026-09-10 拍板：不为它补测试，靠人工验收把关）。
**改动这一带的代码后必须请用户人工复测联合场景**；三个子能力回归
（`tests/test_couple_ui.py` / `test_reverse_in_place.py` / `test_play_orders.py`）
只覆盖各自局部（配对 / 折返 / 寻路），不能替代联合验收。

两个遗留已知问题：

1. **A 碾过 B（无碰撞）**：寻路/折返逻辑过紧时 A 可能从 B 车身穿过——系统
   本无碰撞判定（信号豁免/调车全放行都接受视觉重叠），记录于「已知限制：
   调车/倒车」，**不修**（无碰撞是有意取舍）。
2. **连挂合并折叠（已修，勿回退）**：60m+60m>100m 时车厢压扁。根因是旧
   `TrainEntity.couple_with` 用**前车单侧** `path_kin.sub_path(车头-合并长, 车头)`
   截取合并车身，越界被 clamp 成前车覆盖长 → 尾段塞不进 → 几何重叠。修复：
   合并窗口 = 后车 occupied + 前车 occupied 中后车没有的新边（贴住时后车头≈
   前车尾、窗口邻接连续），`occupied_offset` 继承后车、`s=合并车长`。回归
   `tests/test_couple_ui.py` §1b。若再出现折叠/几何异常先查此处窗口拼接。

## 编组数据结构审计结论（roadmap #9 第 1 项，2026-09-10，**只汇报未修复**）

用户要求复核"以车厢为最小单位"下的元数据交割（G 阶段原始风险）。**已确立的
不变量（实测）**：`wagon_id` 与 `WagonConfig` 对象跨 解挂↔连挂 往返保持
（顺序 + 对象同一性 + 车头世界坐标 <1e-6）；`data_log` 按 `wagon_id` 正确拆分/
归并且日志 id ⊆ 编组 wagon id；子段 `occupied` 是独立 list、父对象不被就地改动；
**`decouple_at`/`couple_with` 都断言 `is_parked()`（controller None **且** route
空）→ 行驶中与信号前等待（holding）的列车都不能改变编组**（这是 G 阶段风险没
爆发的关键原因）；`tick_reservations` 按对象身份清除被替换实体的预约。

**未修复的风险（动手做计划系统 #7/#9 前必读；完整清单与证据见
`docs/roadmap.md` #9 章节 A/B）**：

⚠ **状态更新（2026-09-10，T0~T2 落地后）**：**B2 / B4 / B5 / B6 已修或已结构性
消失**（physics 去身份共享、死字段删除、`couple_with(self)` 自反断言、`data_log`
迁入车厢后拆分/归并记账消失）；**B3 的答案是"计划归控制车（车厢），不能挂
`TrainEntity`"**（设计已定，实现属任务 3）；**B1 的处置已定（Q3）：不做强制**——
"只读"只是**设计准则**（防御性措施），`WagonConfig` 仍是可变 dataclass、`bogies`
仍是可变 list，所以下面第 1 条"**不要把新状态加到 `WagonConfig` 字段上**"**要靠
纪律遵守**（登记为研讨稿 §9 **R8**）。以下 6 条保留作背景与证据。

1. **车厢是共享可变别名**：解挂后两段与父列车、连挂后 merged 与两个来源**都是
   同一批 `WagonConfig` 对象**——就地改一个字段会跨编组串味（实测 `mass` 传播
   到父列车）。**不要把新状态加到 `WagonConfig` 字段上**；车厢级元数据应走
   `WagonDataPacket.payload`（**T2 起已随车厢托管**；但全仓仍零生产/消费者）。
2. **`physics` 交接无规则**：解挂后两段共享父的 physics；连挂后 merged 用
   **前车**的 physics，后车的被静默丢弃。今天全员 `RealisticElectric()` 默认
   参数故无差异；roadmap #3 落地前必须定"物理属于编组还是属于机车"。
3. **列车级元数据在编组变化时全部丢弃且无交割通道**（`goal`/`remaining_to_goal`/
   `v_target`/`authority_remaining`/`_stop_before_m`/`couple_approach_partner`/
   `controller` 实测全部重置）→ **"计划"不能挂在 `TrainEntity` 上**，否则解挂/
   连挂会静默丢掉计划。
4. **`Consist.velocity` / `Consist.acceleration` 是死字段**（全仓零引用、永远 0，
   与 `TrainState.v` 构成双重真源且从不更新），命名极易误导。
5. **`couple_with(self)` 无自反断言**：实测不报错 → 同一 `WagonConfig` 对象在
   编组内重复、`total_mass` 翻倍、几何重叠。UI 层有防护（`target_train is front`
   检查 / `find_couple_pair` 排除自身），模型层没有。
6. `data_log.split` 把**非车厢归属**的包一律归后段；`merge` **不去重**（同一
   包对象跨两段时合并后重复）。今天 `WagonDataPacket` 从未被实例化，故无现行
   影响，但第 3 项任务若想用它承载列车级/计划级数据会踩到。

## 数据结构重想：以车厢为中心（**研讨稿，未定稿，不要先实现**）

用户 2026-09-10 看完上面的审计结论后判断"现有数据结构不太稳固，建议重新思考"，
提出四点设想，完整记录 + 差距分析 + Q1~Q13 研讨问题 + **§6 逐字段归属草案** +
**§8 可无感落地提案**见
**`docs/wagon_centric_data.md`（研讨稿）**。要点（**未定稿**；Q1/Q2/Q6/Q8/Q11 已拍板）：

1. 车厢根本属性是 **`have_control`**（区分控制车/非控制车）；注意它与现有
   `P_rated`/`is_powered`（机车=有功率）**语义不同**，四种组合怎么支持待研讨。
2. **车厢属性只读、只能自力更新**：物理属性（空载重量、马力、性质）定死、不接受
   列车写入；载货量自力更新（近期不做）；**调度计划不可写**——"每个控制车都有
   一个司机，有自己的任务，与列车无关"。
3. **列车编组不存任何持久化数据，只汇总**：汇总物理属性算动力学；汇总全体控制车
   的计划、**遴选后执行**；向车厢发一过性命令（装卸货等，近期不做）。
4. 多控制车冲突先给**最简解法**：控制车自带 `priority`（**不查重**），冲突按
   优先级采纳，相同则用 `wagon_id` 大小决断；深层问题用户明确暂不回答。
   （注意：**牵引力仍是全体动力车叠加**，与"计划只取一辆"正交。）

**该设想顺带化解审计 B1/B2/B3/B4/B6**（物理属性只读后共享别名不再串味、physics
退化为无状态公式集、计划归车厢所以编组变化不丢、`Consist.velocity` 可删、
`data_log` 移入车厢后拆分/归并记账消失）；**B5 仍需单独补自反断言**。

✅ **Q1 已拍板（2026-09-10）= 方案 A**：**域数据（物理属性 / 载货 / 计划）归
车厢所有**（只读，载货量等由车厢自力更新）；**运行期派生状态**（`v`、
`occupancy` 窗口、`controller`、`authority_remaining`、`v_target`，以及**由选中
计划投影出来的 `route`/`goal`**）**由编组持有、可随时重算**，存档时仍写位置快照
（`docs/session_persistence.md` 的机制不变）。**不做"运动学真值下沉车厢"**——
`RigidWagonKinematics` 刚体链与 `session` 位置还原**保持现状、不重写**。

**新字段的判定规则（先问这一句）**：它是**域数据**（描述这节车厢本身 → 车厢拥有、
随车厢走、只读）还是**运行期派生**（描述此刻这列编组怎么跑 → 编组持有、可随时
重算/丢弃）？**编组持有的字段不得被当作域数据长期保存或跨编组交割。**

因此：计划在控制车上、编组的 `route`/`goal` 只是它的**投影**——解挂/连挂丢投影
**不是丢数据**，新编组会重新遴选控制车并重新投影（这正是"司机与列车无关"）。
`Consist.velocity/acceleration` 与 `data_log` 判定为违规（前者编组持有域数据性
质的字段且从不更新；后者车厢级数据却托管于编组）→ 分别按 B4/Q9 处理。

✅ **Q2 已拍板**：控制/动力**四种组合都支持**（控制+动力=机车、控制无动力=驾驶
拖车、有动力无控制=补机、都无=普通车厢）。**控制是车厢的决定性属性**——
`have_control` 与身份同级，决定"能否持有并执行任务"；**动力只是物理参数的一部分**
（`P_rated` 与 `mass` 同级，无足轻重）。→ 现有 `is_powered`（= `P_rated is not
None`）**不再承担"机车"语义**，其三处用途（物理牵引 ✓ 保持 / 渲染 `Loco`·`Coach` /
解挂提示）要改判。

✅ **Q6 已拍板**：**解挂后无控制车的段静止**；**调度计划只有控制车才能持有**——
这不是取舍而是 Q1/Q2 的推论。推论（2026-09-10 用户确认"基本正确"）：连挂时
"落选计划"**不搬移、不丢弃**——计划始终留在其控制车上，遴选只决定"此刻执行谁的"；
**且没有"挂起"标记**（见下 Q13）。

✅ **Q8 已拍板最小单元（完整调研未开始）**：**POI 的最小单元是"有向点" = 1 node +
1 edge，从 edge 指向 node；edge 只提供方向、不提供长度**（GIS 视角的带属性点要
素），可表达"站台末端""有方向的停车点"。**注意 POI 属于新的第三归属桶
"基础设施侧"**（挂 node/edge、随轨道落盘）——原来"域数据 / 运行期派生"两桶放不下
它（信号其实一直是这一桶）。

**逐字段归属草案**见研讨稿 **§6**（**草稿、不求全、接受持续修改与扩充**）：四桶 =
A 车厢域数据 / B 编组运行期派生 / C 基础设施侧 / D 删除·迁出，已把
`WagonConfig`·`Consist`·`TrainState`·`TrainEntity`·调度侧·`session` 的每个现行
字段逐条判了归属，并新暴露 Q11、Q12（编组构成落盘）。

**Q6 补充 + Q13/Q14 ✅ 已拍板（2026-09-10 第二轮，用户答复）**：计划 = **有序命令
列表 + 指令指针**；存在「**等待连挂**」命令（示例：1 前往A点 → 2 等待连挂 → 3 空命令
（解挂后跳转至此）→ 4 前往B点）。机制三条：
1. **事件 = 直接调用车厢的对应方法**（编组 → 车厢，不做事件总线/队列）。
2. **"挂起"不存在**：**没有挂起标记**，指令的执行与读取不会停止，只是有些命令的
   内容本身是"等待…"——**看起来像挂起，其实在跑循环**。⇒ 计划类型里**不要**加
   `suspended`/`paused` 状态字段。
3. **指令指针属于控制车自己（车厢级）**；**步进 = 列车到达目标后直接调用控制车的
   方法**（Q14）。实现必须满足：**单一步进者**（只有当前持有控制权的那列车能步进）
   + **到达事件幂等**（每"到站"只触发一次；现状可用 `GameLoop.run()` 的
   `moving_before`/`just_stopped` 模式，连挂的自动对接就挂在该事件上）。
**解挂时间线（用户给出，代码顺序照此写）**：①C 执行解挂指令 → ②生成 A、B →
③**C 把信号发给 A、B**（调用车厢方法）→ ④**销毁 C** → ⑤A、B 独立运行
（**先通知、后销毁**）。

**Q11 已拍板**：计划的**悬空引用 → 该条命令跳过不执行、计划仍保留**（OpenTTD
「非法的调度计划」）——**执行期跳过**，不是编辑期拒绝（与 DELETE 保护策略不同，
可并存）。

**Q3/Q4/Q5/Q7 ✅ 已拍板（2026-09-10 第二轮，用户答复）**：
- **Q3 "只读"是设计准则，不是强制逻辑**（防御性措施）⇒ **不做** frozen/私有字段/
  访问控制；**T3 的"只读冻结"一项撤销**。"自力更新" = **wagon 自有字段只能由
  wagon 类下属的逻辑更新**（外部只能**调用车厢的方法**）。因此**装载反过来**：
  车厢从列车（调度计划）拿到"可装"信号，**自己从站台"抢"货**（不是站台给车厢装货）
  ——用户承认"很怪，但符合 wagon_centric 理念"。
- **Q4 克隆 = 新车**：相当于玩家新购买（若将来有经济系统），**空车、新车、填缺省值、
  不继承原状态** ⇒ 此前担心的"克隆共享可变状态"不再是问题。
- **Q5 优先级相同 → `wagon_id`（uuid4 字符串）取小者胜**；遴选规则完全确定
  （priority 大者胜 → id 小者胜，不查重），**尚未实现**（无消费点）。
- **Q7 数据流是单向管道**：**调度计划 → 寻路需求 → 寻路结果 → 预约路径 → 物理模拟
  → 位移**，后者消费前者；**遴选每帧进行**。现有链路已符合（`_issue_goal_order` →
  `assign_route` → `TrainDispatcher`/`reserve_path` → `update`/physics → `s`），
  只缺最上游的"调度计划"那一段。
**仍待研讨**：Q12 与研讨稿 **§9 风险登记 R1~R9**。
**已拍板补充**：**Q15 计划的根本特征**（条目+指针、**走完跳回第一项**、
最基本命令 **goto → 建筑/POI**、demo 重点做 goto、连挂/解挂视情况加入、
**空计划 = 停车等待**）；**POI 方案与最小可行范围**见 **`docs/poi.md`**
（三类 POI、建筑=POI 集合、MVP 只做"有向点"；★ 核对结论：**有向点 ≡
`DirectedEdge`，现有 `goal_directed` 已保证"从固定方向到达"，用户设想的
"挖成本图"对 MVP 不需要**）；**R1「车厢没有 tick」已详答**（研讨稿 §9.1：
**历史遗留、非设计决定**；补 tick 时**物理不得下放**；"拆不拆
`Wagon`/`WagonConfig`"待拍板）；**货物/装载 = demo 不做**（roadmap #10）。

**可"无感落地"的准备工作**见研讨稿 **§8**（用户要求"先把不添加新功能的部分做掉"）。
✅ **T0 / T1 / T2 已于 2026-09-10 全部落地**（用户批准；三次提交：`4919434` T0 注释、
`fc6808f` T1 删死代码、T2 惰性结构准备）：
- **T0**：归属规则 + 四桶写进 `model/wagon.py` / `model/train_entity.py` 模块 docstring
  与 `docs/session_persistence.md` §2.1（新字段先问"域数据还是运行期派生"）。
- **T1**：删 `Consist.velocity`/`acceleration`（死字段）与
  `WagonConfig.reversed_config()`/`Consist.reversed_consist()`（死代码 + 身份隐患）。
- **T2**：`WagonConfig` 增 `have_control`/`priority`（**加了但当前无任何消费点**，
  session 已存取且兼容旧档：缺键时按当时行为推导"控制车⇔有动力"）；`data_log` 从
  `Consist` **迁入车厢**（容器改名 `WagonDataLog`，`split/merge` 记账删除，
  `split_at`/`merged_with` 只切/拼成员列表）；新增 `Consist.control_cars()`（无调用者）；
  physics **去身份**（模块级 `train_physics.DEFAULT_PHYSICS` 全列车共享，B2 从结构上消失）；
  `couple_with(self)` 加自反断言（B5 关闭）。
  验证：全套 14 项回归通过 + 真实存档往返（旧档读回按当时行为推导、往返字节稳定、
  `manual_track.geojson` 未被改动）。
- **T3 仍未做**：~~只读冻结~~（**Q3 已撤销——"只读"是设计准则、不做强制**）、
  载货、**计划/命令类型**、POI、`route`/`goal` 投影重构。
  ⚠ 在 Q3/Q5/Q13 等定稿前，**不要**按本设想继续做结构性改动；
  但注意这批字段已在代码里（`have_control`/`priority`/`control_cars`），
  **在计划层定稿前不要给它们接消费点**（否则等于提前定行为）。

## Input reference

| Key / mouse | Mode | Action |
|-------------|------|--------|
| `B` | any | Switch to BUILD mode |
| `D` | any | Switch to DELETE mode |
| `Esc` | BUILD_ACTIVE | Cancel current build, return to BUILD_IDLE |
| `Esc` | other | Switch to IDLE |
| Right click | BUILD_ACTIVE | Cancel current build (same as Esc) |
| `Q` | any | Quit program |
| `S` | any | Save network + signals + trains to manual_track.geojson (loaded on startup if exists) |
| `F` | any | Toggle pathfinding test mode (debug) |
| `I` | any | Toggle spatial index visualization (debug) |
| `C` | any | Toggle camera follow (train tracking) |
| `K` | PLAY | 编组确认键：悬停内部车钩=解挂；悬停其它列车端头车钩=连挂（规格见 `docs/consist_ui.md`） |
| `R` | PLAY | 选中停放列车原地折返（任意位置；只换前进方向、编组顺序不变；要求车身所在段无道岔） |
| `Space` | PLAY | 悬停内部车钩(停放)=解挂确认；否则 = 紧急停止 |
| `Enter` | PLAY | 悬停内部车钩(停放)=解挂确认（同 K） |
| `↑` | Train active | Throttle (accelerate) |
| `↓` | Train active | Brake (decelerate) |
| `LSHIFT` (held) | BUILD_ACTIVE | Force straight along T1 |
| `LALT` (held) | BUILD_ACTIVE | Force Case 2T single-tangent arc (M2 path-snap to straight edge) |
| Left/Right/Middle drag | IDLE | Pan camera |
| Middle drag | BUILD/DELETE | Pan camera |
| Left click | PLAY | 选列车 / 放置（左键**永不**触发放大/编组操作） |
| Left click | Pathfinding test | Select start/goal nodes (F mode) |
| Scroll | any | Zoom |

车钩悬停交互（PLAY 模式，`docs/consist_ui.md`）：内部车钩=灰圆点（解挂点），
端头车钩=青方块（连挂点），命中半径按屏幕像素（不随缩放）。悬停内部车钩 →
tooltip「解挂 → N1+N2 节」→ K/空格/回车 确认；悬停其它列车端头车钩 →
tooltip「连挂目标 #k」→ K 确认（已贴住直接连挂；未贴住则驶向对方车尾，到位
自动连挂）。任何右键寻路停车在其它停放列车端头车钩 1m 内也自动连挂（2026-09
定稿，见 §9-4 决策记录）。判定逻辑在 `controller/coupling.py`（纯模型可测）。

## PLAY 驾驶与巡航（2026-09 bug2 决策，勿回退）

手动驾驶：`↑/↓` 按住累积目标巡航速度（`self.train_v_target`，±5 m/s·dt，
上限 30 m/s），列车只在"有指令（route 非空）**且**巡航 > 0"时才行驶；停放车
巡航=0 时下达指令不会自动起步（下达打印提示"按 ↑ 起步"）。

**下达寻路指令不清零巡航**（bug2，勿回退）：`_issue_goal_order`（右键设目的地
与 K 连挂驶向共用）**不得**把 `self.train_v_target` 清零——真实 GUI 主循环
`run()` 每帧执行 `active_train.v_target = train_v_target`（巡航写回），清零会
把玩家已建立的巡航抹掉：玩家"停车按 ↑ 设速 → 右键设目的地"或"行驶中右键改向"
后列车刹停，表现为"橙色路径可见（route 已下达）但车不动"（2026-09 bug2，回归
`tests/test_play_orders.py`）。巡航归零只允许在：空格急停、放置新车、换选到
巡航 0 的车（各列车 v_target 存实体上，换选时按 822 行载入）。

**折返后 remaining 按几何重算 + 连挂驶向固定到达方向**（bug2 根因，勿回退）：
用户实测"A 驶向 B 连挂却在下一个 node 停车"。两个叠加缺陷（commit 6a5e6b6）：
1. `TrainEntity._do_auto_reversal` 折返后 `remaining_to_goal` 曾直接保留折返前
   值——寻路被迫绕行、含死端折返往返段时（单向信号使直路被禁，如 manual_track
   node700 信号把 660→700 禁成单行），该值仍按"继续正向行驶"计、虚高一条往返
   段，列车越过 goal 冲到下一授权边界才停。现在按折返几何重算：车头前方到路径
   尾弧长（occ 剩余 + route 全长）− goal 距其段尾折算(eo) − 停车提前量
   （`train._stop_before_m`，`_apply_route_result` 下达时记录，连挂驶向=
   head_hook_offset）。**不重新寻路**（避免折返振荡）。
2. 连挂驶向（K/右键吸附）固定 `goal_direction = 目标列车 current_direction()`，
   不再枚举 ±1——枚举可能选到"绕行后从反方向接近目标车尾"的路径（贴上了却因
   `_ends_aligned` 朝向相反连不上）。被单向信号挡住无法正向到达时明确报不可达。
   普通右键寻路仍枚举（玩家不表达进站方向）。

Modifier keys are polled per frame in `GameLoop._sync_modifiers`, not edge-triggered.

## Pathfinding (转向许可与寻路)

**Turn permission** (`RailNetwork.turn_allowed`): 几何自动推断,无需道岔配置。
判据为**前进半平面**——到达节点的行进方向 `d_in` 与离开方向 `d_out` 夹角
严格 < 90°(`d_in · d_out > 0`)。这样直通(0°)和缓分股(~32°)许可,发卡弯
(~148°)、正交(90°)、掉头(180°)禁止。初版用 cos(150°) 阈值,交叉渡线的
4 联通点会误判 148° 发卡弯为许可(dot=-0.847 通过 >=-0.866),改为前进半平面
后彻底修复(合法/非法两侧余量极大:32° vs 148°)。

**Pathfinding** (`model/pathfinding.py`): Edge-based Dijkstra。搜索状态 =
有向边 `(edge_id, dir)`,`dir ∈ {+1, -1}`(+1 沿 node_a→node_b,-1 反向)。
邻接由 `turn_allowed` 决定,代价/可通行走 `cost_fn`/`passable_fn` 钩子
(信号层以后注入约束,不返工)。`Path` dataclass = 有向边序列 + total_cost,
就是给运动学层的契约。`find_path_between_nodes` 是节点间寻路入口,枚举
所有出发有向边取最短。

**Debug 测试**(F 键叠加态): 左键依次点选两节点 → 自动求路 → 控制台打印
段数/总长/有向边序列;第三次点击重置。可视化:起点绿圈/终点红圈,路径橙色
加粗,每段中点顺序编号 1,2,3…。

`RailNetwork.simple_segment_from_endpoint(endpoint_node_id)`: 从死端
(`connection_count()==1`) 沿二度节点链走到下一个道岔或另一死端，返回
`(total_length, turnout_node_id, edge_ids)`。原是 `pathfinding.py` 里只服务
折返的自由函数，2026-09 提升为 `RailNetwork` 方法——纯图遍历不依赖寻路概念，
且是通用基础设施：折返时判断死端 simple segment 能否容纳车身、信号系统的
"安全停车位置"判定（车身是否会跨在道岔上）是同一个几何问题，理应共用同一份
实现。

## 信号系统 (Signal System)

见 `docs/train_control.md` 「信号系统 Roadmap」章节获取完整设计文档、
OpenTTD Path Signal 调研笔记和分步实施状态。核心要点：

- **固定闭塞**，不做移动闭塞。信号状态由 edge/node 占用直接衍生，与列车
  运动状态无关。
- **One-Way PBS 语义**(`model/signal.py::SignalTable`)：信号槛位是
  `DirectedEdge`，只有"正面"，反方向永久禁止通行，不允许背靠背放置两个
  相对信号。
- **Block 边界判据是"节点"，与"能不能通行"无关**（2026-09 bug 修复后
  确立）：`BlockManager._compute_block` 的 DFS 走到某节点时，若该节点
  挂着任意方向的信号就停（`_node_has_any_signal`，按 tail_node 判断信号
  是否物理位于该节点，不是任意相邻边任意方向），与信号是不是"背面"、
  能不能通行完全脱钩。这是为了同时满足：环线场景不能被绕背面吞并整个
  网络；单线双向两端各放一个相对信号时，两个 block 应对称、都覆盖中间
  整段区间。原始实现（只检查前方来向是否正对信号）和第一次修复尝试
  （检查边的任一方向）都被推翻过，详见 `docs/train_control.md`「Step 4
  检查点期间发现并修复的第二个 bug」的完整记录，改动前请先读。
- **颜色完全自动化 + PBS 通路判定**（2026-09 起）：`SignalTable` 只记录
  放置位置,不存颜色；颜色由 `model/block.py::BlockManager` 实时推导，玩家
  无法手动切换红绿。判定不是"block 内有车就红"，而是 `_has_free_path`——
  从信号出发沿 turn_allowed 走，只要存在一条 edge 全部既不被占用也不被
  预约的路径走到下一个信号/死端就 GREEN，所有分支都堵死才 RED。
- **占用判定 = edge 交集，不是完全互斥**（2026-09 用户澄清，务必先读）：
  老版简单闭塞 = "block 内任何位置有车就整块红灯"（完全互斥，更安全、
  现实在用）；PBS = "本车预约的 edge 集合与其它占用交集为空即可放行"
  （edge 交集，更现代，本项目目标）。典型场景：单线 2 线车站
  `主线-【A道/B道】-主线`，道岔无信号时 A/B 股道并入同一粗 block，完全
  互斥会让停在 B道 避让的列车卡死 A道 出站。实现：`_reservations` 存
  `dict[TrainEntity, set[int]]`（train → 预约的具体 edge_id），`reserve_path`
  只查本车路径 edge 是否被他人预约、不再展开到整块；`compute_colors` 走
  `_has_free_path` 判通路；调度层物理占用检查也只查本车实际要走的边。
- 渲染用等边三角形，Layout 模式（见 `view/renderer.py` 顶部），屏幕像素
  基准，不随缩放变化。
- **信号与网络编辑的耦合缺口**：`SignalTable` 存的 `DirectedEdge` 引用
  `edge_id`，但 `Editor`（DELETE 模式删边/合并节点）完全不知道
  `SignalTable` 的存在，两者刻意零耦合。删掉信号所在的边后会留下悬空
  引用，靠 `SignalTable.prune_missing(network)` 自我清理（`GameLoop.run()`
  每帧在 `block_manager.rebuild()` 之前调用一次），不侵入 `Editor` 的
  删除逻辑。
- **进路预约**（Step 4 起，Step 6 改 edge 交集）：`BlockManager.reserve_path`
  只检查本车 `path_edge_ids` 是否被别的车预约（edge 交集）；单线双向对向
  block 共享 edge 的场景天然被同一套 edge 级冲突拦下（方向令牌，事前阻止
  而非事后死锁检测）。释放条件按 edge 判断 `e ∈ (occupied ∪ route)`（不是
  只看 occupied 快照，避免刚下指令就误释放前方未走到的 edge）。
  `tick_reservations` 只信任当前 `trains` 列表，解挂/连挂产生的旧
  `TrainEntity` 一旦被移出该列表，其预约立即清理。
- **远场/近场寻路拆分**（Step 5，2026-09 起，见 `docs/train_control.md`
  「Step 5」完整记录）：`find_path_from_point`（唯一跑 Dijkstra 的地方）
  只用 `SignalTable.passable_topology_only`（拓扑 + One-Way PBS 反方向
  硬性禁止，不看占用/预约）算出完整远场路径；`BlockManager.reserve_path`/
  `make_passable_fn` 不再接寻路，改为对 `truncate_to_next_signal` 截出的
  近场段（前方一个闭塞区间）单独调用。`truncate_to_next_signal` 的截断
  边界 = 信号实际保护的 block（不是"走到信号跟前"就停——这两者错位过
  一格，是个真实 bug，教训是这类边界必须用真实 `GameLoop` 端到端验证，
  纯单元测试测不出预约集合和 block 集合对不上）。
- **信号接入运动控制**（Step 6，2026-09 起，见 `docs/train_control.md`
  「Step 6」完整记录）：核心是**运动授权（Movement Authority）**——列车
  只能驶到已预约闭塞区间末端。`model/dispatch.py::TrainDispatcher` 每帧对
  每列车做"预约推进 + 授权边界计算 + 状态转移"，
  写入 `TrainEntity.authority_remaining`；`update()` 制动目标改为
  `min(remaining_to_goal, authority_remaining)`，红灯前连续制动曲线停车、
  绿灯续约恢复。预约预算"最多 2 个受保护区间"（当前段 + 前方一段），但只数
  **车头前方**已预约的 block（`_walk_frontier` 返回的 `blocks_ahead`）——不能
  用"总共持有几个 block"，否则长列车（车身横跨多个 block、车尾未驶离的
  block 仍被 tick_reservations 持有）会在绿灯前被永久卡死（2026-09 人工测试
  发现）。`TrainEntity.hard_stop()` 从占位实现为兜底急停（授权边界落到车头
  之后时触发，带可见警告）。新增状态 `is_holding()`（信号前等待，保留
  route/goal），`is_parked()` 重定义为"无控制器且无指令"——这两者必须区分，
  否则等待中的列车会被当作可解挂/可折返。`TrainDispatcher.tick` 接收
  `trains` 列表，把"被其他车物理占用的 block"也判红灯（补齐 `reserve_path`
  只看预约表的缺口）。红灯语义从 Step 4 的"下达时拒绝指令"改为"接受指令、
  信号前等待、绿灯续行"。顺带修复 `find_path_from_point` `start_offset` 未
  考虑 `direction=-1` 的 bug（逆向起点 remaining 算成 0）。
- **编组作业信号豁免**（roadmap #1 追加，2026-09 起，见 `docs/consist_ui.md`
  §5.5）：连挂驶向 / 解挂后分离驶离，都是"列车要开进/开出被另一列车占用的
  受保护闭塞区间"，必须冒进信号。**本仓库无碰撞判定**，所以不需要 OpenTTD
  fork 的"旁路撞车 + 重定义连挂"——只在 `TrainDispatcher.tick` 构造
  `others_occupied` 时排除两个**配对专属**豁免：`train.couple_approach_partner`
  （驶向某列车端头车钩的连挂指令）与共享至少一条 occupied 边的
  `train.split_sibling`（解挂出的前/后段）。**被绕开的只有这一处物理占用检查**；
  `reserve_path`（预约表冲突）、`compute_colors`（红灯）、`_walk_frontier`/
  授权边界计算都**不绕开**，授权仍 clamp 在车钩处（stop_before + `update` 的
  `min(remaining, authority)`）。豁免按对象引用 + 共享边动态判定，第三方列车
  不受影响（有回归测试兜底）。

## 渲染约定：Layout 模式

`view/renderer.py` 当前的全部渲染归为 **Layout（极简）模式**：没有美术
素材前，所有要素（轨道/节点/信号/列车/车钩）依然完整可见可交互，但表现
脱离真实比例尺——线是逻辑细线，标记是几何图形，交互性图标用固定屏幕像素
常量定义（不随 `camera.scale` 缩放）。这不是占位实现，是长期共存的渲染
方式：引入真实美术素材后不会修改/替换这套方法，而是新开一套渲染方法
并存。因此这里的图形选择可以用"够用就行"的标准，不需要为将来换皮预留
抽象层。

## Working with `docs/editor.md`

The editor design doc is the source of truth for behavior. §3 covers the full
snap system (Point/Grid/Parallel/Path + length/angle), §7 lists the completed
scope (Steps 0–6, §10.1–§10.5, §12.1, snap features, and spatial index, all ✅).
§11 records the editor's own stance, spatial index shipped
(340× speedup, satisfies 60fps), next direction is advanced parallel snap
(multi-segment along shortest path); Z-axis deferred until visual debugging
catches up. **（2026-09-10 更新：§11 里"近期开发重点仍是编辑器"已不代表项目
重心——编辑器已稳固，重心转为 roadmap #7 计划式自动驾驶；§12 的"瓦片化数据
结构"**已实施**，即 `model/spatial_index.py`，不要误记为待办。）**

**Pseudocode blocks in the doc are non-normative — implement to the
behavior description, not the code samples.** When changing editor behavior,
update the relevant §3 / §4 / §5 sections; when adding new follow-up
requirements, append them as §12+ items.

**Keep docs in sync with code (learned the hard way).** Docs have drifted behind
the code before — snap features shipped while editor.md still marked them
"待研讨". Rule: when a feature is finalized (functionality frozen), update the
docs in the SAME change. Backfilling / reorganizing older sections is optional
and can be deferred, but at minimum the doc MUST point out the latest progress
(mark it ✅ in §7 and note it in §11) so editor.md never lies about what exists.

## Conventions used in this codebase

- Always respond to the user in Chinese (project convention).
- Match existing file style: type hints throughout, `from __future__ import annotations`,
  dataclasses for plain data, snake_case Python.
- Geometry tolerances live as module constants (`SNAP_THRESHOLD`, `MAX_ARC_RADIUS`,
  `MERGE_RADIUS_TOL`, `MERGE_DIR_DOT_MIN`, `BIARC_COLLINEAR_DOT_MIN`). Add new
  ones at the call site's module.
- Prefer immutable returns; `Vec3` arithmetic is non-mutating.
