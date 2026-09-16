这里记录列车控制相关需求

目前我们的进度是：已经完成列车几何与物理逻辑（基础）、寻路、couple/decouple 与车厢级
身份绑定（wagon_id）。信号系统正式启动，采用固定闭塞，分四步走（见下方「信号系统
Roadmap」）；多列车路径冲突/避让暂不实现，等信号系统完成后再回头处理。

大致Roadmap：
A. 寻路算法微调，支持在轨道尽头掉头，支持将Edge上的任意点作为寻路起止点。**[已完成]**
B. 实现PID控制器，整合进手动控制中 **[跳过，直接实现C]**
C. 实现列车自动驾驶与停车，替代手动控制，这对后续测试有益 **[已完成]**
D. 迁移必要逻辑和锚点，从Consist到Wagon，为编组变化作准备 **[已完成]**
E. 多列车测试 **[已完成]**
F. 支持couple/decouple **[已完成]**
G. 整理multiple unit逻辑
到此为止可认为列车控制功能得到了完善，且工作量已相当不小。
A到C是最重要的，而D到G是额外的，复杂度很高但意义重大

**当前状态（2026-08）**：
- ✅ A 阶段完成：Edge 途中寻路、折返支持、手动朝向选择
- ⏭️ B 阶段跳过：直接使用简单加减速控制，未实现 PID
- ✅ C 阶段完成：自动寻路 + BrakingController 自动停车（含 PID 巡航 + 制动曲线）
- ✅ D 阶段完成：`WagonConfig.wagon_id`（uuid，`reversed_config` 保留原 id）+
  `Consist.data_log`（`ConsistDataLog`，随 `split_at`/`merged_with` 同步归属）
- ✅ E 阶段完成：多列车并行、RTS 式交互（左键选择/放置，右键下达寻路指令）
- ✅ F 阶段完成：`decouple_at`/`couple_with` 内部断言 `is_parked()`（数据层不变量，
  不依赖 GameLoop 调用方自觉）；GameLoop 接入车钩悬停/几何判定
- ⏳ G 阶段未开始：暂无 multiple unit（一个 consist 内多机车）的核心逻辑冲突处理

**已知 glitch（暂不修复，待后续稳定后处理）**：
- 寻路算法与列车运动逻辑有偶发的轻微错误：Edge/Path 拼接偶尔不正确，导致列车位置轻微跳变

**已修复的控制器 bug（2026-08）**：`BrakingController` 的 `safety_margin > 1` 会让
"remaining <= d_stop" 判据在全力制动过程中双向翻转（d_stop 比 remaining 掉得更快），
导致 throttle/brake 持续震荡直到停车前（实测 15m/s 巡航场景震荡 161 次）。修复：
制动区加闩锁（`_braking`），一旦进入不再用该判据重新判断退出；同时到达判据从单纯
`remaining < 1.0` 放宽为"闩锁生效且速度低于 STOP_SPEED"，避免 `safety_margin` 保守
估计导致车停在目标前几米却永远不被判定为到达。见 `model/train_controller.py`，
回归测试 `tests/test_braking_controller.py` / `tests/test_stop_at_node.py`。

**已修复的控制器 bug 2（2026-09）：提前停车**——上面这次修复本身留下了一个未被
发现的副作用。到达判据"闩锁生效且速度低于 STOP_SPEED"意味着只要速度降到
`STOP_SPEED=0.3` 就直接判定到达，完全不看 `remaining` 还剩多少。`safety_margin=1.3`
本来就故意高估了制动距离（30% 安全余量），全力刹车到 `STOP_SPEED` 时通常还剩若干米
没走完——提前量 = `0.075·v_进入制动时速度²`，随速度平方增长，是确定性物理模型下
可精确复现的固定偏差，不是随机误差。这个 bug 在 2026-08 那次修复完成后就已经存在，
但被同期修复的震荡 bug 意外掩盖：反复的油门/刹车切换会让车一点点往前蹭，凑巧蹭过了
判定阈值，所以此前没被注意到。用户在真实存档截图报告"列车走完预定弧长前就提前停车"
后复测确认。

修复：不再用速度阈值代替距离判据。改成三段式——巡航 → 全力制动（直到速度降到
`STOP_SPEED` 以下）→ 低速爬行（追 `CRAWL_SPEED=0.5`，类比真实列车进站前的行为）→
`remaining <= STOP_EPSILON=0.05` 才真正停车。新增 `_crawling` 闩锁（同 `_braking`
的设计），避免速度在 `STOP_SPEED` 附近来回穿越时在两个子状态间反复切换。实测四档
巡航速度（5/10/15/20 m/s）停车误差都稳定在 4cm 左右（STOP_EPSILON 容差内），彻底
消除了"提前几米停在非整数、非节点位置"的现象。

回归测试更新：`tests/test_braking_controller.py`（断言改为"停车距离 <= STOP_EPSILON"
而不是"停车速度 < STOP_SPEED"——后者在三段式模型下不再是判据，爬行阶段目标速度本身
就高于 STOP_SPEED，是设计意图；切换次数上限从 1 放宽到 3，对应三段式模型的三次正常
状态转换）；`tests/test_stop_at_node.py` 补充了停车位置精度断言（原测试只验证"能停
下、速度归零"，没验证停车位置精确度，这次的 bug 恰好就是精度问题，必须补上才能真正
覆盖）。**验证时的一个坑**：`get_all_wagon_poses` 返回的是车厢几何中心（两转向架连线
中点），不是车头最前端，拿它验证"是否到达目标节点"是错误的参照点（会显示比实际误差
大出半个车身长的数字）——正确做法是用 `kinematics._path_kin.pose_at(train.state.abs_s)`
直接查车头转向架位置，两个测试文件都已改用这个方式。

**已知待办（记录，不紧急）**：HUD 上的弧长显示（`draw_train_hud` 的
`total_length`）取自 `RigidWagonKinematics.total_length = occupied 总长 − 车尾偏移`，
而 `occupied` 是滑动窗口——车没有移动的某一帧，如果正好触发了窗口头部弹出旧边
（`advance_occupied_path` 内部逻辑），分母会突然变小，玩家会看到形如"60/134 → 60/128"
的数字回退，容易被误读成"进度倒退"。这是纯 UX 展示问题（把内部实现细节直接暴露给了
玩家），不影响真实停车位置或逻辑正确性，优先级低于本次的提前停车修复，值得后续优化
（比如 HUD 分母改用寻路时算出的固定总长，不随 occupied 窗口滑动而变化）。

**已修复的控制器 bug 3（2026-09）：制动曲线过早减速+缓慢爬行**——bug 2 引入的三段式
方案（全力制动到 `STOP_SPEED` → 固定速度 `CRAWL_SPEED=0.5m/s` 爬行 → 到点）本身留下
了一个没预见到的尺度问题。`safety_margin` 高估的制动距离随速度平方增长，而"转入爬行"
的判据是固定速度阈值，跟"还剩多远"完全脱钩——巡航速度越高，转入爬行时剩下的距离越
夸张：用户实测报告"速度过早降到 1.8km/h，然后以这个速度缓慢接近停车点"，量化验证：

| 巡航速度 | 转入爬行时剩余距离 | 以 1.8km/h 爬完需要 |
|---|---|---|
| 5 m/s | 2.3m | 4.6秒 |
| 10 m/s | 7.1m | 14.2秒 |
| 15 m/s | 14.0m | 28秒 |
| 20 m/s | 22.3m | 44.6秒 |

修复：改成**连续制动曲线**（真实 ATO/列车自动驾驶的标准做法）——目标速度不再是固定
阈值，是随剩余距离连续衰减的曲线 `v_brake_target = sqrt(2·a_brake·remaining/margin)`，
制动阶段用 bang-bang 硬控制追这条曲线（`v > 目标` 全力刹车，`v < 目标 且 v < v_cruise`
全力加速，否则空转）——**不能用 PID 追这条曲线**，第一次尝试用 `SimpleSpeedController`
追动态目标时，PID 增益是为追基本恒定的巡航速度调的，追一个快速衰减的目标会有响应
滞后，实测直接冲过终点（`remaining` 归零时 `v_final` 还有 6+ m/s，完全没减速下来）。
改成硬控制后四档巡航速度实测都能平滑停在 `STOP_EPSILON` 容差内，速度曲线连续下降，
没有"先砸到固定低速再匀速爬"的两段式手感。移除了 `STOP_SPEED`/`CRAWL_SPEED`/
`_crawling`，`BrakingController.update()` 只保留 `_braking` 一个状态标志。

回归测试更新：`tests/test_braking_controller.py` 的震荡检测改为统计 `_braking` 标志
本身的翻转次数（进入制动后应只翻一次），不再统计 throttle/brake 输出的翻转次数——
连续曲线下的 bang-bang 控制在离散时间步下必然有高频的输出切换（车速贴着连续下降的
目标曲线来回跨越），这是良性微调（加速度本身没有跳变），不是早期版本"完全退回巡航
再重新判定进制动"那种恶性宏观震荡，两者不能用同一个计数器区分。

## 信号系统 Roadmap（固定闭塞，分四步）

前提与边界（2026-08 讨论确定）：
- 采用**固定闭塞**，不做移动闭塞（复杂度不敢做）。信号状态由 edge/node 占用状态直接
  衍生，与列车具体运动状态（速度等）无关。
- **图拓扑不变量**：信号是 `DirectedEdge` 的属性（旁挂表，不进 `Node`/`Edge` 字段），
  不是节点属性。放置信号本身属于"基础设施变化"，允许 `split_edge_at`；但信号状态
  切换、闭塞占用判定，永远不触碰图拓扑（详见 `CLAUDE.md`「Graph topology invariant」）。
- 停车点分两类：真 Node（车站/信号机/道岔/waypoint/depot，持久化基础设施）和伪 Node
  （列车中途停车/连挂解挂产生的临时位置，只用 Edge 上的标量偏移表示，从不写回图）。
  目前 couple/decouple 已验证完全不需要伪 Node。
- 短期目标：不碰信号逻辑本身的多列车路径冲突/避让**先不做**；先在这四步范围内把
  基本功能做扎实，得到可靠原型后进行第一次开源，再继续往下推进信号。

**Step 1：定点停车健壮性 + hard_stop 接口占位** ✅ 已完成
- `BrakingController` 制动区震荡 bug 修复（见上方「已修复的控制器 bug」）。
- 端到端验证目标恰好落在 Node 上（`goal_t=0.0/1.0`）时无抖动到达。
- `TrainEntity.hard_stop()` 接口占位（`NotImplementedError`，无调用点）：对应 JGRPP
  "realistic braking 来不及安全停车"的兜底场景，留给 Step 3/4 信号强制停车时接入。
  触发时必须打印可见警告，不能静默吞掉（JGRPP 社区反馈：静默兜底会被玩家当成 bug）。

**Step 2：信号机数据结构 + 纯手动 + 寻路接入** ✅ 已完成（人工检查点通过）
- `model/signal.py`：`SignalState`（GREEN/RED）+ `SignalTable`
  （`dict[DirectedEdge, SignalState]` 旁挂表，`place`/`toggle`/`passable` 等）。
  `passable` 直接符合 `pathfinding.PassableFn` 签名，可直接传给 `find_path*`。
- 新建信号默认 **GREEN**（这一步纯手动、不做安全兼顾，不意外阻断现有行车）。
- 槛位消解：`RailNetwork.resolve_directed_edge_by_click(node_id, click_pos)`——
  一个节点最多有 `len(incident_edge_ids)` 个独立槛位，按"点击方向 · 离开切线方向"
  点积最大者选定。**二度节点的退化场景**（点击方向与轨道正交，或 Edge 中途
  `split_edge_at` 产生的新节点位置精确等于点击坐标、`click_dir` 恒为零——这是
  split 场景的常规路径，不是概率性边界情况）：确定性兜底为"该节点作为 `node_a`
  的那条边"（原 edge 正方向），玩家想要另一方向只需点击时朝目标边稍微偏移。
- GameLoop 新增 `EditMode.SIGNAL`（`H` 键切换），左键在 Edge 中途点击会先
  `split_edge_at` 再放置/切换信号；`GameLoop.signals: SignalTable` 持有实例，
  已接入 `_find_path_any_goal_direction` 的 `passable_fn`。
- 回归测试：`tests/test_signal.py`。

**Step 2 追加（人工检查点后，2026-09）**：

1. **One-Way PBS 语义**（借鉴 OpenTTD Path Signal，见下方「OpenTTD Path Signal
   调研」）：一个槛位只存"正面"一个状态，反方向永久禁止通行（不是"无信号=自由
   通行"），物理上只需要一盏灯。`SignalTable`：
   - `place(directed)` 返回 `bool`——背面（`is_blocked_backside`）拒绝放置，
     不能在已有单向信号的反方向再放一个相对的信号。
   - `toggle(directed)` 返回 `SignalState | None`——背面没有正面信号，`None`
     表示无操作，不是"新建再翻转"。
   - `passable(edge, direction)` 判定顺序：本方向有信号看状态 → 反方向有信号
     则本方向是背面永久禁止 → 都没有则自由通行。
   - 持久化不需要单独存储背面状态，加载后由 `passable`/`is_blocked_backside`
     动态推导，语义在往返后保持一致。
2. **图形**：等边三角形，顶角指向通行方向（`view/renderer.py::_draw_signal_triangle`），
   替代之前的圆点——没有美术素材前更直观区分朝向，背面天然没有图标（物理上只有
   一盏灯，不用画两个圆）。
3. **持久化（最简版，Step 2 阶段，后续可替换）**：`node_id`/`edge_id` 每次加载
   都重新分配，不能直接存 id。改为在 GeoJSON 顶层新增 `"signals"` 字段（跟
   `"features"` 平级，不进 LineString 几何格式），每条记录存
   `{"from": [x,y,z], "to": [x,y,z], "state": "green"/"red"}`（信号所在节点坐标
   + 该边另一端节点坐标），加载时按坐标反查 `DirectedEdge`（复用
   `node_id_at` 的容差匹配，与建网络同一套语义）。`write_geojson(network, path,
   signals=...)` / `load_signals(path, network)`（`model/geojson_loader.py`）。
   旧存档没有 `"signals"` 字段时优雅退化为空表，不报错。`GameLoop` 的 `S` 键
   已同步保存 `self.signals`，启动加载已同步还原。
   **已知局限（刻意从简，后续可能要改）**：按坐标匹配意味着如果两个节点坐标
   完全重合（理论上不应该发生，`node_id_at` 本身就是靠坐标去重的）会有歧义；
   更稳妥的方案是给 `Node` 加一个持久 UUID，但那会影响 `model` 纯几何层的最小
   接口，Step 2 阶段不做这个改动，坐标匹配对当前测试规模足够用。

回归测试见 `tests/test_signal.py`（槛位消解三方向、默认绿灯不阻断、切红后寻路
正确不可达、One-Way PBS 背面禁止与拒绝放置、二度节点退化兜底、持久化往返、
旧存档无 signals 字段的优雅退化）。

**⏸️ 人工检查点** ✅ 已通过（2026-09）：信号放置/切换/寻路绕行表现符合预期。

**Step 2 追加 2（人工检查点后，2026-09）**：One-Way PBS 图形改为等边三角形，
顶角指向通行方向；持久化补齐，坐标匹配的最简方案（见上）。

**Step 3：固定闭塞区间 + 占用影响信号** ✅ 已完成（2026-09）
- **信号从"手动状态"变为"占用推导的只读值"**：`SignalTable`（`model/signal.py`）
  收窄为纯粹的放置记录（`set[DirectedEdge]`），不再存颜色；`toggle`/`get`/
  `SignalState` 从该模块移除。玩家在 SIGNAL 模式下只能"放置"，点击已有信号
  只打印提示，不改变任何状态——手动覆盖会立刻被下一帧的自动推导覆盖，
  保留切换入口没有意义（决策记录：2026-09 讨论确定完全自动化，不做手动
  覆盖优先级机制，理由是后者逻辑更复杂且容易在信号系统还没做完闭塞前引入
  状态优先级 bug，与"少踩坑"的目标冲突）。
- **`model/block.py::BlockManager`**：新模块，`SignalState`（GREEN/RED）搬到
  这里，语义变成"block 是否被占用"而不是"玩家设的颜色"。
  - `rebuild(network, signals)`：从每个信号的正面方向出发，沿
    `turn_allowed` 允许的转向做 DFS，收集途经边直到遇到下一个信号（该方向
    的正面）或死端。**未设信号的道岔会把所有分支并入同一 block**——现实
    固定闭塞里无信号道岔天然属于同一闭塞区间，不特殊处理（已用三度节点
    测试验证：east 槛位的 block 沿途扩展到死端，不在无信号的道岔处提前
    截断）。
  - `compute_colors(trains)`：block 边集合与任意列车 `occupancy.occupied`
    的 edge_id 集合有交集就判 RED，否则 GREEN——即时判定，不是预约，没有
    "车尾完全清出才释放"的时序概念（那是 Step 4 的事）。
  - 重算频率：每帧无条件 `rebuild()`（`ponytail`：网络规模小，O(edges ×
    signals) 可忽略，换掉了在每个网络变更点手动触发失效标记的侵入式改动；
    数百边以上再改成拓扑变化时才重算的脏标记）。
- **阶段性局限**（按计划字面表述执行，需要在下一步开工前重新审视）：
  这一步只让信号灯颜色数据正确，**完全没有接入寻路**——`find_path_from_point`
  不再传 `passable_fn=self.signals.passable`（Step 2 接进去的那条线已断开），
  列车物理上会无视红灯继续开，因为寻路根本不知道有信号存在。这不是 bug，
  是刻意的渐进验证手段：把"占用→颜色对不对"和"红灯是否正确让寻路绕行"分成
  两个独立可检查的问题，不混在一起排查。Step 4 做进路预约时会重新、完整地
  接回寻路（经过安全等待点、方向令牌等设计后再接，不是这里临时接一下）。
- 回归测试：`tests/test_signal.py`（槛位消解、One-Way PBS 放置放置/拒绝、
  block 划分含未设信号道岔的验证、占用→RED/清空→GREEN 的实时推导、退化
  兜底、持久化往返）；`GameLoop` 端到端烟测（放置→重复点击无操作→占用变红→
  渲染红色三角形不崩→保存重载）。

**⏸️ 人工检查点** ✅ 已通过（2026-09）：颜色只读、占用变红/驶离变绿、
道岔无信号处不产生虚假边界、寻路暂不受信号影响（会闯红灯，符合刻意的
阶段性局限）均验证符合预期。

**Step 3 追加微调（人工检查点后，2026-09）**：

1. **信号图标比例尺无关**：三角形之前用世界单位（`SIGNAL_OFFSET` 米）
   定义偏移/大小，缩小地图看全局时会缩到看不清。改为屏幕像素基准
   （`SIGNAL_OFFSET_PX` / `SIGNAL_TRIANGLE_RADIUS_PX`，`view/renderer.py`），
   偏移方向用 `world_to_screen` 是仿射变换（无旋转）的性质，把世界方向
   向量的 `(x, -y)` 分量归一化后直接当屏幕方向用，全程留在屏幕像素坐标里
   计算，不再依赖 `camera.scale`。已验证不同缩放下偏移量恒定。
2. **命名："Layout / 极简" 渲染模式**：`view/renderer.py` 当前的全部渲染
   正式定名为 Layout 模式——没有美术素材前，所有要素完整可见可交互，但
   表现脱离真实比例尺（线是逻辑细线，标记是几何图形，交互性图标用固定
   屏幕像素常量）。这不是占位实现，是长期共存的渲染方式：引入真实美术
   素材后不会修改/替换这套方法，而是新开一套渲染方法并存，因此这里的
   图形选择可以用"够用就行"的标准。见 `view/renderer.py` 顶部注释、
   `CLAUDE.md`「渲染约定：Layout 模式」。
3. **`simple_segment_from_endpoint` 提升为 `RailNetwork` 方法**：原是
   `pathfinding.py` 里只服务折返的自由函数，提升后是通用基础设施——
   折返判断死端 simple segment 能否容纳车身、Path Signal 的"安全等待点"
   判定（车身是否会跨在道岔上）是同一个几何问题，理应共用同一份实现。
   三个调用点已同步改为 `network.simple_segment_from_endpoint(...)`：
   `pathfinding.py::neighbors`（折返判定）、`train_entity.py::reverse_in_place`
   （截断 turnout 背面的边）。已用真实折返场景端到端验证。见
   `CLAUDE.md`「Pathfinding」小节的说明。

## OpenTTD Path Signal 调研（供 Step 3/4 设计参考）

结论：Path Signal 本质仍是**固定闭塞**，只是把"block 内有车就是红灯"的粗粒度
占用判断，换成了"预约一条穿过 block 到下一个安全等待点的具体路径，不冲突就能
共享同一个 block"的判断（[OpenTTD Manual: Signals](https://wiki.openttd.org/en/Manual/Signals)）。
这跟 Step 4 的设计方向完全对应，`OccupancyState.occupied` + `route` 已经具备
"这辆车打算走哪些具体的边"这个信息，Step 4 要做的就是把它注册成全局可查询的
预约表并加冲突检查。

三点值得直接借鉴到 Step 3/4：

1. **安全等待点（safe waiting point）**：Path Signal 红灯时列车只能停在"车身
   完全不侵占任何道岔"的位置——"it is only safe for a train to wait in front
   of a junction"，never immediately behind one。这条规则在代码里已经有一个
   独立发现的雏形：`reverse_in_place()` 用 `truncate_to_segment` +
   `simple_segment_from_endpoint` 保证车身不跨在道岔上，只是当时是为了解决
   折返问题。Step 3/4 划分 block 边界/预约终点时，应该显式复用这条判定标准，
   能提前排除一批"停车点本身不安全"引发的死锁。
2. **对寻路的影响是代价而非纯布尔**：反向通过一个单向 Path Signal 技术上允许
   但会被寻路器加罚分劝退（除非目的地就在信号背后），跟现有 `REVERSAL_PENALTY`
   （劝退不必要折返而非硬性禁止）是同一种设计哲学。当前 `passable_fn` 是纯
   布尔，Step 2 阶段够用；Step 4 接入预约后，可以考虑"这条路径会让别的车等待"
   用惩罚而非直接拒绝，让寻路自然倾向不拥堵的路径。
3. **JGRPP 的方向令牌（slot）机制，替代通用死锁检测**：单线双向场景，每个方向
   各设一个容量固定的令牌槛，进入前必须先拿到本方向令牌；对向令牌被占用时，
   本方向的车根本不会去申请，直接在信号前等待——事前预防，不是"两车都上路后
   再检测死锁"。这比通用图论死锁检测更简单，且贴合固定闭塞的定位，**建议作为
   Step 4 单线场景死锁避免的首选实现方式**（替换早前笔记里"死锁检测 + 拒绝
   分配"的方案）。

JGRPP 还有一套更重的机制（per-signal 脚本、slot/counter、programmable
pre-signal），本质是信号可编程化，用于按列车属性/编组/时间动态调整路由——
明显超出当前"少踩坑、先出可靠原型"的范围，**Step 3 做完后再讨论要不要借鉴**，
不在这轮展开。

**Step 3：固定闭塞区间 + 占用影响信号（寻路暂不受影响）** ✅ 已完成（详见上方
「信号系统 Roadmap」章节的完整记录，含人工检查点通过、三项收尾微调）。

**Step 4：寻路结果衍生进路预约，对信号机产生影响** ✅ 已完成（2026-09）

- **预约表**：`BlockManager._reservations: dict[DirectedEdge, TrainEntity]`。
  在 `GameLoop._issue_path_order` 里，寻路成功后立即对整条路径涉及的 block
  一次性 `reserve_path()`（全有或全无），不是列车驶入后才逐段预约。
- **死锁避免走"方向令牌"，不是通用死锁检测**：按最终采纳的调研结论（本节
  下方 JGRPP 笔记第 3 点），`reserve_path` 把冲突判定下沉到 **edge_id 级别**
  ——检查本次预约涉及的所有 block 展开后的全部 edge_id，是否已被别的列车
  通过*任意* block 持有。这天然覆盖了"两个不同 DirectedEdge key 的 block
  因中途无信号被合并、结果边集合完全重叠"的单线双向场景（已用
  `tests/test_reservation.py` 场景 1 验证），事前阻止，不需要等两车都上路
  再检测环路。
- **释放条件是 `occupied ∪ route`，不是只看 occupied 快照**：最初一版实现
  只检查 `block & occupied`，会在"刚下指令、车身还没开始走"的窗口误释放
  尚未走到的前方 block——写测试时发现并推翻重写。现在 `tick_reservations`
  用 `block & (occupied ∪ route)`，只要这段路还在待走路由里就不释放，车尾
  完全清出且不再需要才释放。折返不受影响：`reverse_in_place` 只反转方向，
  不改变 `occupied` 的 edge_id 集合。
- **`tick_reservations` 只信任当前传入的 `trains` 列表**：不在列表里的
  holder（典型场景是解挂/连挂产生的全新 `TrainEntity`，旧对象从
  `GameLoop.trains` 移除后不再被任何地方更新）的预约立即释放，不需要在
  decouple/couple 调用处额外插入释放逻辑，避免遗漏导致的永久悬挂预约。
- **`passable_fn` 组合**：`BlockManager.make_passable_fn(train)` 返回符合
  `PassableFn` 签名的闭包，只检查预约冲突；`turn_allowed` 的几何转向约束
  仍由 `pathfinding.neighbors()` 内部单独处理，两者是天然分离的责任链，
  不需要手动拼接成一个 if-else（原计划考虑过 `turn_ok and not_reserved`
  显式组合，实测两个约束本来就作用在寻路的不同阶段，不必强行合并）。
- 回归测试：`tests/test_reservation.py`（9 个场景：单线双向 block 边集合
  重叠验证、方向令牌拒绝、同列车重复预约幂等、颜色推导、release 时机含
  route 未耗尽、悬挂预约清理、`make_passable_fn` 阻断/放行）；`GameLoop`
  端到端烟测（真实寻路 → 预约成功 → 第二辆车被 `passable_fn` 正确阻断 →
  两车驶离后释放变绿）。

**Step 4 检查点期间发现并修复的 bug（2026-09）**：DELETE 模式删除一条挂有
信号的边（或删除度数为 2 的节点触发合并，原边被替换成新边）后，
`SignalTable` 里残留的 `DirectedEdge` 仍指向已不存在的 `edge_id`，下一帧
`BlockManager.rebuild()` 在 `head_node()` 里对 `network.edges[edge_id]`
取值直接 `KeyError` 崩溃。根因：`Editor._delete()`/`_try_merge_at_node()`
完全不知道 `SignalTable` 的存在——两者原本刻意零耦合（SIGNAL 模式是在
`GameLoop` 单独接的线），这也是没预料到的耦合缺口。

修法不是往 `Editor` 里插清理调用（那样会把两个刻意分离的模块绑死），而是
让 `SignalTable` 自己对着当前网络做悬空引用清理：新增
`SignalTable.prune_missing(network)`，移除所有 `edge_id` 不在
`network.edges` 里的槛位，返回被移除的列表。`GameLoop.run()` 每帧在
`block_manager.rebuild()` 之前调用一次——这样删边、删节点、合并三种场景
（不管 Editor 内部具体怎么改的网络）都被同一套逻辑覆盖，不需要在每个
网络变更点手动插清理代码。回归测试补在 `tests/test_signal.py`
（复现 KeyError 场景：手动删除边和节点模拟 `Editor._delete()` 的效果，
验证 `prune_missing` 正确清理且 `rebuild()` 不再崩溃）。

**Step 4 检查点期间发现并修复的第二个 bug（2026-09）：block 划分被环线
"绕背面"吞并整个网络**。用户在真实存档上截图发现：列车停在一处，一大片
本该互相独立的信号同时变红。用真实存档数据复现后确认：8 个信号里有 4 个
的 block 膨胀成了全部 34 条边（整张地图）。

根因分析过程（连续两轮方案都被推翻，记录下来避免以后重蹈）：

1. **原始实现**只检查"前方来向是否正对着一个信号"
   （`signals.has_signal(nxt)`），信号只有正面被记录、背面完全没有对应
   记录。DFS 从信号背面经过时检测不到任何东西，会毫无察觉地穿过去继续
   扩张。直线轨道上这条路径永远不会被触发（DFS 只会往前走），但只要
   地图上有一个折返环线，DFS 就能绕一圈从背面杀回主线——这正是用户
   最先提出的关键洞察："block 划分与信号背面能不能通行，并无必然
   关系"，即 block 边界应该是纯粹的基础设施属性，跟"能不能通行"完全
   脱钩。

2. **第一次修复尝试**（已推翻）：改成"进入某条边前，检查这条边任一
   方向是否有信号"。这在单线双向场景里矫枉过正：A--edge_a--M--edge_b--B，
   A 端放一个面向东的信号、B 端放一个面向西的信号，两者本该联手守护
   `{edge_a, edge_b}` 整段区间（这正是用户提出的第二个洞察：从两个方向
   分别发起 DFS 会得到不对称的结果，需要显式处理）。但这个方案在 DFS
   走到 M、准备进入 edge_b 前，就因为"edge_b 反方向有信号"被拒绝进入，
   导致 edge_b 整条都没能计入以 A 端信号为起点的 block——错误地把边界
   提前到了 M，而 B 端信号物理上明明站在节点 B，不该在节点 M 就被挡住。
   用 `tests/test_reservation.py` 的单线双向场景测出这个问题。

3. **最终方案**：判据从"边"移到"节点"——`BlockManager._compute_block`
   每走到一个新节点，用 `_node_has_any_signal` 检查这个节点上是否挂着
   任意方向的信号（不看是不是当前方向的正面），有则把刚走过的这条边
   计入 block 后停止继续扩张，没有则正常继续。这个判据同时满足：单线
   双向场景里 edge_b 被正确计入（M 无信号，正常经过；B 有信号，走到
   即停，但边本身已计入），且环线绕背面场景里 DFS 会在第一次绕回任何
   已设信号的节点时正确止步，不会继续扩张吞并整个网络。
   - **实现时的第二个坑**（已推翻）：`_node_has_any_signal` 最初直接
     检查"节点相邻边、任意方向是否有信号"，结果每个 block 走一步就
     自我卡死（size=1）——因为 DFS 第一跳的到达节点，天然包含"刚走
     过来的那条边"，而那条边上恰好挂着自己出发的那个信号（物理位置在
     边的另一端，不在当前节点上），被误判成"当前节点有信号"。修正为
     只检查"信号的物理位置（tail_node）正好是这个节点"的那个方向，
     不是任意方向。
   - 信号物理位置的方向约定：`(edge_id, direction)`，`direction=+1`
     时 tail 是 `node_a_id`，`direction=-1` 时 tail 是 `node_b_id`（与
     `pathfinding.head_node`/`_directed_from` 的既有约定一致）。

回归测试：`tests/test_reservation.py` 场景 1（单线双向两个信号切出对称
block，验证不再被矫枉过正的第一次修复方案提前截断）；`tests/test_signal.py`
新增六边形环线场景（两个信号切环成两段独立弧，各 3 条边，验证不再被
绕背面吞并）——这两个测试合起来覆盖了导致连续两轮方案被推翻的两种
场景，缺一个都无法复现当时踩的坑。

## Step 5：远场/近场寻路拆分 ✅ 已完成（2026-09）

**问题背景**：Step 4 的 `_issue_path_order` 一次寻路同时承担两个职责——
判定可达性（这个目标理论上能不能走到）和实际下达调度（现在能不能真的
走）。这两件事混在一起会导致一个真实缺陷：如果直接从起点用带预约冲突
检查的 `passable_fn` 寻路到终点，一条铁路上不可能一路绿灯，大概率会
判成不可达；即使凑巧全绿灯预约成功，也会把整条铁路预约死，别的列车
完全无法调度。

**用户澄清的关键概念**（这次讨论中纠正了一次我最初的误解）：远场/近场
不是两次独立寻路，是**级联但不重跑 Dijkstra**的两步：
1. **远场寻路**：只跑一次 Dijkstra，只看轨道拓扑 + One-Way PBS 的反方向
   硬性禁止（拓扑级单向限制，不是信号灯颜色），得到完整的全程路径。
   这一步"红灯不代表这里不能走"——判可达性用不到闭塞占用/预约信息。
2. **近场调度**：拿着远场算好的完整路径，不重新寻路，只做一次线性扫描
   ——按 OpenTTD 原版逻辑，只预约"前方一个闭塞区间"，不做 JGRPP 的
   Long Reserve 多区间预留。"路径预约的责任只是确保面前没有别人，
   无需确保一路畅通"。

OpenTTD 有时会在行驶中触发重新寻路（远场重跑一次 Dijkstra），但触发
条件复杂，本轮不实现——列车拿到远场路径后按它一路走到底，中途遇到的
信号/占用只影响近场预约成功与否，不影响列车已经在走的 `route`（这次
范围明确排除了"逐区间自动推进预约、失败就自动停车"的连续机制，那需要
接入 `TrainEntity.hard_stop()`，留给以后单独做）。

**实现**：
- `SignalTable.passable_topology_only(edge, direction)`：远场寻路用的
  `passable_fn`，只查 `is_blocked_backside`，不查任何占用/预约状态。
  符合 `pathfinding.PassableFn` 签名，可直接传给 `find_path*`。
- `BlockManager.truncate_to_next_signal(network, signals, route)`：把
  远场路径截断成"近场段"——无保护路段（原样经过，不占用任何 block，
  不需要预约）+ 遇到的第一个信号实际保护的完整 block（从信号出发向前
  延伸到下一个信号或死端）。一路无信号时返回整条 route 不截断。
- `GameLoop._find_path_any_goal_direction` 不再接受 `requesting_train`
  参数、不再用 `make_passable_fn` 过滤——这是唯一跑 Dijkstra 的地方
  （远场），只用 `passable_topology_only`。`_issue_path_order` 里
  `_issue_to_goal` 先拿到远场完整 path，再调用
  `truncate_to_next_signal` 得到近场段，只对近场段调用
  `reserve_path`——远场 path 本身原样交给 `assign_route`，列车物理上
  按完整路径驱动，不受近场只预约一段这件事的限制。

**实现过程中发现并修复的一个真实 bug**：`truncate_to_next_signal` 第一版
实现在"走到第一个挂信号的节点就停"，这跟 `BlockManager._compute_block`
算出的 block 方向定义正好错位一格——block 是"从信号出发向前延伸"，不是
"走到信号跟前"。结果近场段截出来的是信号**前面**那一截（无保护路段），
完全没包含信号实际保护的 block；`reserve_path` 拿着这段去检查冲突，永远
查不到交集，预约表在寻路成功后仍然是空的（预约形同虚设）。这个问题只有
用真实 `GameLoop` 端到端跑一遍才暴露出来——纯单元测试（`test_far_near_field.py`
最初几版）因为没有验证"预约的 block 集合是否真的覆盖了信号保护的边"，
没能测出来，教训是端到端验证不能省。

修复：改成"标记是否已进入某个 block"（进入条件是当前边本身就是某个
信号，即 `signals.has_signal(directed)` 为真）——进入后继续走，直到到达
下一个挂信号的节点（标志这个 block 走完，遇到下一个 block 的边界）才
截断。

回归测试：`tests/test_far_near_field.py`（7 个场景：远场无视占用找到
完整路径、近场截断=无保护段+第一区间、近场预约与更靠后的占用互不影响、
反方向硬性禁止在远场也生效、近场段精确对齐真实占用时预约正确失败、
无信号场景不截断、单信号且 block 延伸到死端时不出错）；`GameLoop` 端到
端烟测（两个信号场景：远场 route 是完整 4 段路径，近场只预约第一个
区间对应的 edge，第二个区间的 edge 确认未被预约）。

## Step 6：信号接入运动控制 ✅ 已完成（2026-09）

这是 `roadmap.md` 待办 #1（最高优先级），也是 Step 5 明确推迟的那段
"逐区间自动推进预约、失败就自动停车"的连续机制。Step 5 之前，灯色/预约
算得再对也只是可视化装饰——`TrainEntity.hard_stop()` 是占位接口从未被
调用，列车物理上按远场完整路径驱动，无视红灯/未预约区间（闯红灯、车辆
视觉重叠都是这条缺口的直接后果）。

**核心概念：运动授权（Movement Authority）**——列车只能驶到"已预约闭塞
区间末端"（下一个信号节点），到点必须停；绿灯时向前推进预约、红灯时停
车等待。新增纯模型模块 `model/dispatch.py::TrainDispatcher`，每帧对每列车
tick 一次：

1. **预约推进**：沿列车剩余路径逐边扫描，确定"已授权到哪"（`TrainDispatcher.
   _frontier_edge_count`：无保护边或已持有 block 的边都算已授权），再对授权
   边界之后的下一个受保护区间调用 `BlockManager.truncate_to_next_signal` +
   `reserve_path` 续约。最多同时预约 2 个受保护区间（"当前段 + 前方一段"，
   OpenTTD one-block-ahead，避免把整条线路预约死，也避免绿灯前频繁停车）。
   预约失败（被别的车预约 or 物理占用）则边界保持，列车停车等待。
2. **授权边界**：沿列车自身 route 计算（不是 block 的全 DFS 边集，所以
   道岔分支不影响"车该停在哪"）——授权边界 = 最后一个已持有 block 的
   末端（下一个信号节点）。`TrainEntity.authority_remaining` 记录车头到
   边界的距离，`update()` 的制动目标改为 `min(remaining_to_goal,
   authority_remaining)`，BrakingController 连续制动曲线在红灯前平滑停车。
3. **红灯停车余量**：授权边界落在红灯区间前端时，制动目标再往回缩
   `SIGNAL_STOP_MARGIN = 0.5m`，让车头停在信号机之前而非精确压线——否则
   制动曲线的离散时间步过冲会让车头越过信号节点、把红灯区间的边吞进
   occupancy（"闯红灯"的亚厘米级表现，也是车辆视觉重叠的直接原因）。
   实测停车点在信号前 ~0.53m。
4. **物理占用视同红灯**：`tick()` 接收 `trains` 列表，把"被其他车
   `occupied` 覆盖的 block"也判为红灯（与 `compute_colors` 的占用语义一致），
   弥补 Step 4 `reserve_path` 只看预约表、不查物理占用的缺口——否则用户
   手工摆放的静止列车（无预约）会被运行中的列车无视并开进同一个 block。
   无信号路段（不属于任何 block）仍不做碰撞避让，保持"多列车冲突/避让
   暂不实现"的范围。
5. **状态机扩展**：`TrainEntity` 新增 `is_holding()`（信号前等待：无控制器
   但保留 route/goal）；`is_parked()` 重定义为"无控制器且无待走指令"。
   `_hold_at_signal()` 停住但保留指令，绿灯续约后 `resume()` 重建控制器继续；
   `emergency_stop()` 才是真正放弃指令。到达授权边界（红灯）→ hold，到达
   goal → emergency_stop。
6. **hard_stop 兜底（占位→实现）**：授权边界落到车头之后（运行中删除/合并
   挂信号的边、或运行中放置新信号导致 block 重新划分，列车已越过的位置突然
   变成红灯区），减速曲线已来不及，`TrainDispatcher` 调用 `TrainEntity.
   hard_stop()` 强制速度归零并打印可见警告——牺牲物理连续性保住"不闯红灯"。
   正常运行时边界始终在车头前方，此路径不会被触发。

**hard_stop 误判修复（2026-09，roadmap #1 验收反馈"信号死锁依然存在"）**：
授权边界落到车头之后（`raw_authority < 0`）有**两种成因**，旧实现一概
`hard_stop`（清空 route/goal），把正常场景误判成"闯红灯"：
- (a) 列车**行驶中**越过安全制动点（删/并信号导致 block 重划）→ 这才是
  hard_stop 本义，必须清指令 + 可见警告。
- (b) 列车**静止起步**时前方受保护区间被别的车占用 → 预约失败、`_walk_frontier`
  停在车头脚下第一条边（frontier=0），`raw_authority` 变负。旧实现把它
  hard_stop → 清空 route **丢指令**，玩家重新下令又再丢——表现成"信号死锁"
  （实测：对向两车同时起步，双车 hard_stop 锁死）。
修复：`dispatch.tick` 按 `v` 区分——`v > STOP_EPSILON` 才 hard_stop；静止则
`hold_at_signal()` 进入 holding（保留 route/goal，绿灯后续行）。新增
`TrainEntity.hold_at_signal()`（公开，语义同原私有 `_hold_at_signal`）。
回归 `tests/test_dispatch.py` 场景 6（对向起步 holding + 绿灯续行）。

**行为语义变化（相对 Step 4）**：下达指令时不再"预约第一区间 + 失败直接
拒绝"，改为"远场寻路判可达性 → 直接下达完整 route"，预约/等待完全交给
`TrainDispatcher`。红灯不再是"指令被拒绝"，而是"接受指令、开到信号前停车
等待、绿灯自动续行"——对应 OpenTTD/真实列车语义（用户确认采纳）。

**顺带修复的 bug**：`find_path_from_point` 的 `start_offset` 无条件用
`start_t × edge.length`，忽略了 `start_direction = -1` 的情形（此时列车从
node_b 出发，已走过 `(1 - start_t) × length`），导致逆向起点时
`remaining_to_goal` 算错（同 Edge 逆向场景实测算成 0，真实应为 6m）。已按
有向边尾端点修正，回归补在 `tests/test_dispatch.py` 的 Part 0。

**人工测试后修复的 bug（2026-09）：长列车在绿灯前卡死**。用户在"最简图"
上人工测试报告"信号机开放后列车不再续行"。复现根因：预约预算用了
`len(held) < MAX_HELD_BLOCKS`（"总共持有几个 block"），但 `held` 包含车身
横跨的所有 block——长列车（默认编组 60m，block 只有 20m 时）车身同时占据
3 条边、持有 2 个 block，预算被"车尾尚未驶离的 block"占满，导致"向前预约
下一区间"这步被永久跳过，列车在绿灯前停车且永不续约。修复：把预算判据从
"总共持有"改为"**车头前方**已预约几个 block"——`TrainDispatcher._walk_frontier`
沿剩余路径从车头向前扫描，返回 `(授权边界边数, 车头前方已持有的 block key
集合)`，预算只数这个集合的 `len`。车尾尚未驶离的 block 不在 `remaining_path`
里、天然不计入。长列车回归补在 `tests/test_dispatch.py` 场景 4（60m 车身 +
20m block，绿灯连续通过不卡死）。

**区间占用的定义：完全互斥 → edge 交集（2026-09，用户澄清）**。用户指出
当前实现展现的是老版简单闭塞的"完全互斥"（block 内任何位置有车就整块红灯），
而原本目标是 PBS 的"edge 交集"（本车预约的路径 edge 与其它占用交集为空即可
放行）。典型死锁：单线铁路上的 2 线车站 `主线-【A道/B道】-主线`，按现代习惯
在 A道 放右行单向 PBS、B道 放左行单向 PBS，道岔处不放信号，A道/B道 被并入
同一粗 block——列车已完全进入 B道 避让后，仍"位于 A道 出口区间内"，整块红灯
卡死 A道 出站列车。改为 edge 交集后，两车占据的 edge 集合交集为空，就该绿灯
放行。**两种标准**：完全互斥（更安全、现实仍在用）vs edge 交集（更现代、
OpenTTD PBS，本项目目标）。修复：

- `BlockManager._reservations` 从 `dict[DirectedEdge, TrainEntity]`（block key
  → train，整块互斥）改为 `dict[TrainEntity, set[int]]`（train → 预约的具体
  edge_id）。
- `reserve_path` 冲突判定只查 `path_edge_ids` 本身是否被别的车预约，不再
  展开到整个 block 的全部边；`tick_reservations` 按 edge 逐条释放。
- `compute_colors` 改为 `_has_free_path`：从信号出发沿 turn_allowed 走，只要
  存在一条 edge 全部既不被占用也不被预约的路径走到下一个信号/死端就 GREEN，
  所有分支都堵死才 RED。这样 A/B 股道不再互锁。
- `TrainDispatcher` 的物理占用检查也从"整个 block"改为"本车实际要走的
  `next_seg` 边"。

回归：`tests/test_dispatch.py` 场景 5（单线 2 线车站会让线：B道被占时进站信号
保持 GREEN、A道列车仍顺利通过）。

**后续修正：保护包络不是 PBS 的区间分割（2026-09）**。`BlockManager` 从每个
信号沿所有允许分支 DFS 到下一个信号得到的集合，现明确称为“保护包络”：不同
入口的包络可以重叠，一个包络也可以在道岔处分叉，因此它不是路网 partition，
不能作为预约预算的计数单位。旧 `_walk_frontier` 会把一条已预约 edge 所属的
所有包络 key 都计入 `blocks_ahead`；两个入口汇流后共享 edge 时，一个实际路径
片段会被误算成两个，达到预算上限后永远不申请下一片段。

修复后，PBS 调度严格按 `固定 route → 路径局部片段 → edge 预约 → movement
authority` 工作：`truncate_to_next_route_span` 沿本车路径截取下一个片段；
`_walk_frontier` 只统计车头当前 edge 之后、已预约前缀中的信号入口。全局保护
包络只保留给受保护 edge 判定、信号显示和调试，不再参与预约预算。信号颜色仍
表示“信号后存在至少一条自由通路”，不保证某列车指定路径已经获得授权；当前
将这种差异视为 UI/UX 问题，不影响运动安全。

**预约生命周期补强（2026-09）**：仅按 `edge ∈ occupied ∪ route` 保留预约时，
若固定路线在环线/多锚点路径中远处再次经过同一 `edge_id`，第一次经过产生的预约
会被未来 occurrence 错误延寿。现在预约额外记录 edge 是否已被车身实际进入：尚未
进入时由未来 route 保留；进入后以车身是否清出为释放依据。未来再次接近同一 edge
时由滚动路径片段正常重新预约，不长期霸占资源。

回归测试：`tests/test_dispatch.py`（7 部分：direction=-1 起始偏移修复；红灯
前停车等待 + 车头不越过信号 + 占用释放后自动续行到终点；绿灯连续通过全程
不进入等待态；无信号直行不受约束；长列车横跨多 block 不卡死；单线 2 线车站
edge 交集不互锁；重叠保护包络不虚增路径片段预算）。

我们遵循一个典型工作流：
1. 拆解需求，变成可一口气实现的小步
2. 遴选需求，砍掉多余的，补上忘记的
3. 排序和实施，先考虑依赖关系，再考虑复杂度
下面是详细说明

## A 寻路算法微调 **[已完成]**

我们无需改动核心算法就能实现上述功能。

**实现成果**：
- ✅ Edge 途中寻路（起点和终点都可以在 Edge 上的任意位置）
- ✅ 折返支持（总是允许折返，通过高代价避免不必要折返）
- ✅ 朝向约束（列车保持朝向，禁止原地掉头，但可以先到终点再折返）
- ✅ 手动朝向选择（放置列车时显示虚影预览）
- ✅ 虚拟节点调试可视化（黄色圆圈标记寻路分割点）

**关键设计决策**：
1. **折返代价机制**：`REVERSAL_PENALTY = 200`，Dijkstra 自动避免不必要折返
2. **总是允许折返**：不再根据列车当前状态判断，寻路算法自由规划"先到终点再折返"
3. **路径拼接**：车尾覆盖路径 + 新路径，保证占位连续性
4. **虚拟节点**：临时分割 Edge，寻路后映射回原网络

### A1 起止点在 Edge 途中

寻路的起止点可能是 Edge 途中的点，而寻路结果 Path 是基于 Node 的。

**设计决策：Path 结构不变，偏移量作为独立参数传递。**

不在 Path 里增加 start_t/end_t 字段，而是让调用层维护两个浮点数：
- `start_offset: float`（米）：列车在起始 Edge 上已走过的弧长，即初始 s 值
- `end_offset: float`（米）：终止 Edge 末尾需要截去的弧长，即 total_length 的修正量

这样 Path 只描述”走哪些 Edge”，偏移量在调用层处理：

```python
path, start_offset, end_offset = find_path_from_point_to_point(
    network, start_edge_id, start_t, end_edge_id, end_t
)
kin = RigidWagonKinematics(network, path, consist, initial_offset=start_offset)
kin.total_length -= end_offset  # 终点提前结束
```

**虚拟临时节点（终点在 Edge 途中，且终点无方向限制）：**

进行寻路时，仍使用 Node 输入算法，比如：
```
n1-(start->)-n2-n3-n4-(end)-n5
```
因为边(n1,n2)上已经有方向，以 n1 为寻路起点，start_offset = start 到 n1 的弧长差。

终点处：若 end 无方向限制，从 end 处将边(n4,n5)切开，在内存中临时插入虚拟节点 np，
寻路到 np，寻路完成后丢弃（不修改网络）。end_offset 为 0（np 即精确终点）。

若 end 明确携带方向（后续引入调度和信号限制），直接寻路到 n4 或 n5，
end_offset = 目标点到所选节点的弧长差。

### A2 折返（掉头）

折返不是物理上旋转 180 度，而是列车切换前进方向，在业务术语中称为”折返”。
无论列车的物理朝向是什么，都不影响其运行和寻路，至少目前不作限制。

我们默认所有连接数为 1 的 Node 处都可以折返。

**实现方式：临时”掉头边”**

在 Dijkstra 搜索内部，遇到连接数为 1 的 Node 时，临时插入一条反向的同 Edge（掉头边），
不修改网络，仅在搜索图中生效。代价 = 该 Edge 弧长的两倍 + 固定换向惩罚。

运动学层遇到掉头边时，s 推进方向反转（从前向后变为从后向前）。

## B PID控制 **[已跳过]**
PID控制器并不复杂，在此不多赘述，若担心性能问题，可适当降低PID控制器的轮询频率。
玩家可选择开启PID控制，这样就可以直接设定预期速度了。
该功能会在C阶段被自动驾驶接管，留好接口。

**实际情况**：直接跳过，C 阶段使用手动加减速控制，足够测试使用。

## C 自动驾驶 **[基础版已完成]**

**当前实现**：
- ✅ 自动寻路：点击目标位置，自动规划路径（支持 Edge 途中和 Node）
- ✅ 手动加减速：↑ 加速、↓ 减速、空格 紧急停止
- ✅ 制动区可视化：HUD 显示速度、加速度、制动距离
- ✅ 自动驾驶入口：`AutoDriveController`（带安全裕度 1.2）
- ✅ **自动停车：已实现**（2026-09-10 修正——此处原写"❌ 未实现（需要预测控制
  逻辑）"已过时）：`model/train_controller.py::BrakingController` 的连续制动
  曲线已接线进 `TrainEntity`（`controller` 字段；`update()` 制动目标 =
  `min(remaining_to_goal, authority_remaining)`，红灯前平滑停车）。实测：
  `tests/test_stop_at_node.py` 停准误差 0.042 m；`tests/test_play_orders.py`
  场景 C 折返后精确停在 goal。

**设计决策**：
- 列车按照玩家点选的位置，自动寻路并出发E
- 行驶过程由玩家手动控制**巡航速度**（`↑/↓` 累积 `v_target`，上限 30 m/s）；
  **到点停车由 BrakingController 自动完成**（2026-09）
- 制动逻辑已实现（HUD 显示），并已集成进运动授权（Step 6）

**C 阶段的剩余缺口不是停车，而是"计划"层**——现在"去哪里"仍靠玩家逐次人工
下达（右键设 goal / 车钩连挂指令 / `↑` 设巡航），没有"创建计划并跟随计划"这
一层。2026-09-10 用户将其立项为 **demo 最大缺口：demo 要做、设计待研讨**，
研讨问题清单见 `docs/roadmap.md`「计划式自动驾驶」。**研讨定稿前不要当
"已规划"引用、不要先写实现代码。**

## D 迁移工作
为什么主流的铁路模拟游戏，如OpenTTD和Transport Fever，都不支持列车连挂与解挂？
我的答案是，它们将主要逻辑绑定在列车编组上，而非车厢上，这使得编组改变是不可接受的。
所以，若我们真的决定支持可变编组，就必须将必要的逻辑迁移到Wagon上面，重新划定Consist的范围
具体怎么做？我不清楚，尚需研讨

## E 多列车测试
这里不需要进一步解释，性能也是通过条件之一

## F 连挂与解挂
连挂的核心在于判定功能是否开启，以及两列车是否处于合适的相对位置，可以通过简单的几何计算实现，细节待议
解挂的核心在于保持解挂前后车厢位置的正确性，因为车厢位置取决于第一个车厢第一个转向架的位置，这很重要
此外还需探讨很多边界问题

**F 阶段完整 UI（2026-09 已完成，roadmap #1）**：数据层原语
（`decouple_at`/`couple_with`、`Consist.split_at/merged_with`）不变，交互升级为
正式车钩交互，完整规格见 **`docs/consist_ui.md`**（设计决策 §9 已全部拍板）。
核心：PLAY 模式下内部车钩（灰圆点）= 解挂点（悬停 → tooltip「解挂 N1+N2」→
K/空格/回车 确认），端头车钩（青方块）= 连挂点（悬停 → 「连挂目标 #k」→ K
确认）；驶向对方车尾的连挂指令会提前 `head_hook_offset` 停车，让车头**车钩**
（而非转向架）停到对方尾钩，停车事件帧自动连挂；任何右键寻路停在其它停放
列车端头车钩 1m 内也自动连挂（§9-4 拍板）。纯判定逻辑抽到
`controller/coupling.py`（`find_couple_pair` / `try_couple_to`，可脱离 GUI 测），
回归测试 `tests/test_couple_ui.py`。

## G Multiple Unit 逻辑

这里主要解决自动驾驶等核心逻辑的冲突。我们已经认定，将主要逻辑绑定在机车上，而一个consist中会出现多个机车
这提供了额外灵活性，同时也需要合理安排，避免冲突和覆盖，同一时间在同一consist内，只有一份核心逻辑的拷贝是生效的

**本节为什么长期为空 + 现在的状态（2026-09-10 用户交代）**：G 阶段的原始动机不是
MU 本身，而是**"以车厢为最小单位"带来的交割风险**——同类铁路模拟游戏几乎都把
列车（编组）当原子单位，本项目主张车厢级；当时担心拆分/拼合时**列车元数据与
调度计划的交割**会沿常见实践做错。实际算错了两件事：① **顺序倒置**（计划式
自动驾驶还没做，先做了连挂/解挂，前提反了）；② **风险没有发生**（目前看连挂/
解挂并未踩到 G 阶段预料的风险点）。参照信息：《狂热运输》《OpenTTD》都没有
连挂/解挂；《模拟火车》（Trainz 起）有，但闭源、核心逻辑无从获知——只能自研。

**因此 G 阶段的实质工作已改判为四项任务**（2026-09-10 用户下达，详见
`docs/roadmap.md`「G 阶段背景与四项任务」，待办 #9）：
1. **检查数据结构**在连挂/解挂过程中的表现（**已执行**）：结论 = 5 条已确立
   不变量（身份/几何往返、data_log 归属、父子不互相就地改动、"编组变化时不存在
   进行中的指令"由模型强制、预约按对象身份释放）+ 6 条风险缺口（车厢对象别名
   串味、physics 交割规则缺失、列车级元数据无交割通道、`Consist.velocity` 死
   字段、`couple_with(self)` 无自反断言、data_log 对非车厢归属包不友好）——
   **只汇报、未修复**；完整清单与实测证据见 roadmap #9 章节 A/B 两节。
2. 构思 **POI 系统**（站台等）——没有 POI 则调度命令没有下达依据。
3. 新增**调度计划相关数据类型**（注意 roadmap #9-B3 的约束：**计划不能挂
   `TrainEntity`**（解挂/连挂会静默丢），也**不能挂 `WagonConfig` 字段**
   （对象别名会串味）；`WagonDataPacket.payload` 是已验证的车厢级元数据通道）。
4. 调度计划**接入列车逻辑，实现自动**。
（2~4 与 roadmap #7「计划式自动驾驶」的研讨是同一件事，须合并讨论；本文
「C 自动驾驶」章节已注明 C 阶段的剩余缺口正是"计划"层。）
