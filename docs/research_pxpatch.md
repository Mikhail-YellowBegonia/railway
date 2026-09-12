# 调研笔记：OpenTTD-PXPatch（连挂/解挂 + 调度计划）

> **性质**：**调研记录，不是决策**。结论分「他们怎么做 / 对我们的启示 / 置信度」，
> 采纳与否由研讨拍板；采纳后写进 `wagon_centric_data.md` / `poi.md` / 未来的计划层文档。
> **完成时间**：2026-09-10。**执行方式**：后台 subagent（独立上下文）完成，未改动仓库。
> **三轮取证**：① 源码反推（§1–§4）② 二次调研"等待如何退出"（§5）
> ③ **官方中文文档逐页取证**（§6，看玩家可见语义）——三轮结论互相印证，
> 出现分歧处以**源码为准**并已在本稿标注。
> **来源**：OpenTTD-PXPatch 中文官方文档 + 其 DeepWiki 摘要 + **仓库源码**
> （分支 `px-patch`）。**注意**：自定义调度计划表那一页正文只有 "WIP"，该部分结论
> **改由源码得出**（`OT_EXECUTE_SCHEDULE` 实现）。

## 0. 来源清单（可复现）

| 来源 | 读了什么 |
|---|---|
| [中文文档目录](https://pulsexlb.github.io/OpenTTD-PXPatch-Docs/zh/) | Hugo Book 站，导航树完整 |
| [1 列车解挂](https://pulsexlb.github.io/OpenTTD-PXPatch-Docs/zh/docs/1-%E5%88%97%E8%BD%A6%E8%A7%A3%E6%8C%82/)（含 [1.1](https://pulsexlb.github.io/OpenTTD-PXPatch-Docs/zh/docs/1-%E5%88%97%E8%BD%A6%E8%A7%A3%E6%8C%82/1.1-%E7%AE%80%E5%8D%95%E7%9A%84%E5%88%97%E8%BD%A6%E8%A7%A3%E6%8C%82/) / [1.2 操作](https://pulsexlb.github.io/OpenTTD-PXPatch-Docs/zh/docs/1-%E5%88%97%E8%BD%A6%E8%A7%A3%E6%8C%82/1.2-%E8%A7%A3%E6%8C%82%E5%88%97%E8%BD%A6%E6%93%8D%E4%BD%9C/) / [1.3 客流分配](https://pulsexlb.github.io/OpenTTD-PXPatch-Docs/zh/docs/1-%E5%88%97%E8%BD%A6%E8%A7%A3%E6%8C%82/1.3-%E8%A7%A3%E6%8C%82%E8%B0%83%E5%BA%A6%E8%AE%A1%E5%88%92%E5%AE%A2%E6%B5%81%E5%88%86%E9%85%8D/)） | 解挂机制全貌 |
| [2 列车对接](https://pulsexlb.github.io/OpenTTD-PXPatch-Docs/zh/docs/2-%E5%88%97%E8%BD%A6%E5%AF%B9%E6%8E%A5/)（含 [2.1](https://pulsexlb.github.io/OpenTTD-PXPatch-Docs/zh/docs/2-%E5%88%97%E8%BD%A6%E5%AF%B9%E6%8E%A5/2.1-%E7%AE%80%E5%8D%95%E7%9A%84%E5%88%97%E8%BD%A6%E5%AF%B9%E6%8E%A5/) / [2.2 操作](https://pulsexlb.github.io/OpenTTD-PXPatch-Docs/zh/docs/2-%E5%88%97%E8%BD%A6%E5%AF%B9%E6%8E%A5/2.2-%E5%88%97%E8%BD%A6%E5%AF%B9%E6%8E%A5%E6%93%8D%E4%BD%9C/)） | 对接的两种命令与限制 |
| [DeepWiki: Orders, Timetables, and Scheduled Dispatch](https://deepwiki.com/pulsexlb/OpenTTD-patches/5.2-orders-timetables-and-scheduled-dispatch) / [Vehicle Core](https://deepwiki.com/pulsexlb/OpenTTD-patches/5.1-vehicle-core) | 章节总览与命令类型 |
| 源码 `pulsexlb/OpenTTD-patches@px-patch` | `src/order_type.h`、`order_base.h`、`order_cmd.cpp`、`train_cmd.cpp`、`vehicle_base.h`、`vehicle.cpp`、`train.h`、`docs/order_list_serialisation_reference.md` |

**抓不到的**：`自定义调度计划表` 页正文只有 "WIP"（改读源码）；
英文站导航为空；JGRPP 官方文档未直接引用（其行为均以 px-patch 自身源码验证）。

---

## 1. 逐条结论

### Q1 连挂/解挂是否作为计划命令存在？→ **是，且是三种独立命令**

- `OT_GOTO_COUPLE`（前往连接）、`OT_WAIT_COUPLE`（等待连接）、`OT_DECOUPLE`（解挂）
  —— `src/order_type.h:102-104`。
- **解挂把"解挂后干什么"编码成一个策略枚举** `OrderDecoupleOrdersFlags`
  （`order_type.h:113-124`）：`KEEP_ORDERS`(0) / `KEEP_ORDERS_NO_LOAD`(1) /
  `WAIT_FOR_COUPLE`(3) / `LOAD_AND_WAIT`(4) / `EXECUTE_SCHEDULE`(5) /
  `LOAD_AND_SCHEDULE`(6)。运行期由 `SplitOrders()`（`train_cmd.cpp:6054`）按枚举
  **为后段现场构造全新的 OrderList**（例如 `ODOF_LOAD_AND_WAIT` → 后段获得
  `[前往本站, 等待连挂]` 两条命令）。
- **"等待"在数据上没有 suspended 状态**：`OT_WAIT_COUPLE` 每帧被正常执行，
  `cur_speed = 0` 置零 + `HoldWaitingTrainBody()` 只保车身占用，然后 `return true`
  （`train_cmd.cpp:8937-8948`）。同理 `OT_WAITING`（时刻表等待）走 `HandleWaiting()`：
  "检查条件不满足就 return，满足才 `IncrementImplicitOrderIndex()`"（`vehicle.cpp:3960`）。
  ⇒ **"等待"= 命令的执行语义，不是计划/车辆上的字段**——与我们 Q13 的判断同构。

### Q2 事件驱动与指针步进 → **物理接触时直接调用；但有一个 re-enter 陷阱**

- 连挂**无事件总线**：碰撞检测发现合法接触对象即 `Couple(moving_front, target)`
  然后停车（`train_cmd.cpp:7322`）。
- 合并时**整份调度状态从等待车交割给行驶车**：
  `primary_order` / `primary_order_index` / `cur_real_order_index`
  （`train_cmd.cpp:6661-6666`）；**紧接着必须 `IncrementRealOrderIndex()` 越过那条
  wait 命令**，否则合并车会重新进入等待、永不前进（`train_cmd.cpp:6674-6676`，
  源码注释明写）。
- 步进原语：`IncrementImplicitOrderIndex()` / `UpdateRealOrderIndex()`，**跳过
  `OT_IMPLICIT` 与 `OT_DECOUPLE` 条目**，到末尾**回绕到 0**（`vehicle_base.h:1029-1045`）。

### Q3 多机车/多控制单元 → **他们没有这个概念（我们是另一条路）**

- **单一 primary 车头模型**：`IsPrimaryVehicle() = IsFrontEngine() || IsFrontWagon()`
  （`train.h:225`），所有订单逻辑先 `->Primary()` 归一化（`Couple()` 开头即如此，
  `train_cmd.cpp:6682-6683`）。**计划只有一份，不存在冲突与遴选。**
- 唯一的"竞争仲裁"在**"谁来连挂谁"**：`couple_target / couple_claimant /
  couple_claim_cost`（`train.h:178-185`）——一台等待车只能被一台行驶车认领，
  挑战者必须**路径代价严格更小**才能抢走（平局保留原认领者，`train_cmd.cpp:6401`），
  且标注 **"Transient couple-claim state (not saved)"**。

### Q4 命令种类、跳转、条件、循环、非法计划

- 类型极多（`order_type.h:78-106`）：`GOTO_STATION / GOTO_DEPOT / LOADING /
  LEAVESTATION / GOTO_WAYPOINT / CONDITIONAL / IMPLICIT / WAITING /
  LOADING_ADVANCE / SLOT / COUNTER / LABEL / SLOT_GROUP / GOTO_COUPLE /
  WAIT_COUPLE / DECOUPLE / EXECUTE_SCHEDULE`。
- **跳转/条件**：`OT_CONDITIONAL`，条件变量含时间/时刻表槽位/需检修/待运货物量/
  空闲站台数/**百分比随机**/无条件跳转；**防死循环**：条件跳转深度超过
  `min(64, GetNumOrders())` 即放弃（`order_cmd.cpp:5204`）。
- **循环**：**没有"循环"命令**——循环就是"跑到末尾回绕到 0"（`UpdateRealOrderIndex`），
  需要别的循环语义时**显式加一条无条件跳转**。证据：`AdoptDecoupleSchedule()`
  为表达"首次跑一次站台命令、之后只重复计划"，现场拼装
  `[goto 站, execute_schedule, conditional(无条件跳至 index=1)]`（`train_cmd.cpp:6031-6041`）。
- **"计划里执行另一个计划"**：`OT_EXECUTE_SCHEDULE` + `primary_order` /
  `primary_order_index` **双指针**（"我在跑别人的表，但记住我在自己表的哪一格"，
  `vehicle_base.h:383-384`），末尾 `ReturnFromExecuteSchedule()` 归位
  （`order_cmd.cpp:5166`）。
- **非法计划**：`CheckOrders()`（`order_cmd.cpp:4265`）**每 20 天巡检、只发新闻提示、
  不改计划**（虚空命令 / 站台不兼容 / 首尾重复 / 站太少），且受 `order_review_system`
  设置开关。

### Q5 原子单位改造 → **他们仍是"计划归编组"，靠"抄指针 / 建新表"回避交割**

- 计划 `OrderList` 挂在车辆链上、可被多处共享（`num_vehicles` 计数，
  `order_base.h:1328`）。解挂按策略做两件截然不同的事：
  - `ODOF_KEEP_ORDERS`：`u->orders = v->orders;` + `u->AddToShared(v)` ——
    **两段共享同一个 OrderList 对象**，并把游标（`cur_real_order_index` /
    `cur_implicit_order_index` / `current_order`）一并抄过去（"两列车跑同一张表且同步"）。
  - 需要不同计划的策略：`DeleteVehicleOrders()` 后**为后段新建独立表**，或整表接管
    玩家创建的 schedule（`AdoptDecoupleSchedule`）。

### Q6 站台/停靠点 → **命令上的两个字段，不是基础设施实体**

- `OrderStopLocation` = `near-end / middle / far-end / through`，外加
  **到站方向** `stop-direction` = `north-east / south-east / north-west / south-west`
  （`docs/order_list_serialisation_reference.md`）。即"停在哪 + 朝哪进站"是
  **命令的属性字段**，站台本身不是可被独立引用的图上实体。

### Q7 货物/客流（边界）

- 官方文档 [1.3 客流分配](https://pulsexlb.github.io/OpenTTD-PXPatch-Docs/zh/docs/1-%E5%88%97%E8%BD%A6%E8%A7%A3%E6%8C%82/1.3-%E8%A7%A3%E6%8C%82%E8%B0%83%E5%BA%A6%E8%AE%A1%E5%88%92%E5%AE%A2%E6%B5%81%E5%88%86%E9%85%8D/) 全文一句话：
  「**我们没有实现列车基于车厢的客流分配功能**，因此在解挂时，如果一个列车有装载
  能力，且执行的线路与原线路不同时，强烈建议选择解挂装卸调度计划，通过这种方法，
  不符合原线路的客流将在解挂站卸载并联运。」
- 即：**改线路时不做"解挂即装卸"就会让客流跟着车厢跑到错误的线路上**。
  他们用**玩法规则**（建议玩家选特定策略枚举）规避，而非在数据层解决。

---

## 2. 三条最重要的可借鉴点

1. **"等待"是命令的执行语义，且必须处理"步进越过它"**：连挂交割指针后必须显式
   `IncrementRealOrderIndex()` 越过 wait 命令（`train_cmd.cpp:6674`）——
   这正是我们 Q14"先通知、后销毁 + 单一步进者"实现时**一定会踩的坑**。
2. **解挂 = 一条命令 + 一个"后段计划从哪来"的策略枚举**，而不是"一个产生两列车的
   动作"。`OrderDecoupleOrdersFlags` 六种取值把"保留原计划 / 装卸后等待连挂 /
   接管玩家计划"全部收纳在一条命令里，运行期才现场拼装后段的 OrderList
   ——为我们"解挂时间线（C 通知 A、B）"提供了现成的语义分层。
3. **"执行另一个计划"用双指针**：`orders`（当前在跑的表）+ `primary_order` /
   `primary_order_index`（自己的表 + 回归格号），末尾归位；目标表被删时就地接管。
   将来若要做"计划嵌套 / 临时借用他车计划"，这是零成本的成熟形态。

## 3. 三条陷阱/反例

1. **参照物本身是"计划归编组"**（全部逻辑先 `->Primary()`）——**不能拿它论证
   "多控制车遴选"**；它唯一的仲裁（认领 + 路径代价、平局给在位者）发生在
   "谁连挂谁"，与我们 priority/id tie-break 不同层。
2. **他们从不"执行期跳过失效命令"**：只有"每 20 天巡检 + 新闻提示"与 `OT_DUMMY`。
   我们的"跳过该条、计划保留"是**自己的一套**，**别被"业界标准"反过来质疑**。
3. **解挂后两段共享同一可变计划对象**（`u->orders = v->orders` + `AddToShared`）：
   与我们审计 B1"车厢是共享可变别名"**同型风险**；他们能接受是因为计划属于编组，
   而我们主张计划归控制车——**照抄这条会直接冲突**。

## 4. 待人工确认的疑点（本文档最重要的一节）

1. **Q11 的出处需要更正**：我们文档写"计划的悬空引用 → 跳过执行（参考 OpenTTD
   「非法的调度计划」）"。调研**未在 px-patch 源码中找到"执行期跳过失效目标"**，
   只有"周期巡检 + 新闻提示"与 `OT_DUMMY`。⇒ 「非法计划」这个概念确实存在，
   但**"自动跳过"是我们自己的延伸**，表述应改为"借鉴非法计划提示的概念，
   但采用执行期跳过"（已在 `wagon_centric_data.md` Q11 就地更正）。
2. **"等待连挂"如何退出循环**：px-patch 里等待车是被**外部行驶车的物理接触**解除的，
   计划本身不变；而我们的示例是「2 等待连挂 → **3 空命令（解挂后跳转至此）** → 4 前往B点」。
   需拍板：指针从等待命令出去，是**事件主动改写指针**（对应 px-patch 的
   `IncrementRealOrderIndex`），还是**空命令作为占位被自然走完**？两条路径对
   "单一步进者 + 到达事件幂等"的约束不同。
3. **`priority` 并列取 `wagon_id` 小者在连挂后是否稳定**：px-patch 把这类字段
   标为 transient / not saved，且**平局刻意给在位者**以保稳定。我们的 uuid 字符串序
   本身稳定，但"落选计划不搬移"意味着连挂后执行权可能在同编组内易主
   ⇒ **建议加一条回归断言**锁住"解挂/连挂往返后同一编组的执行控制车不变"。
4. **站台建模是路线分歧，不是实现细节**：他们用"命令上的 `stop-location` +
   `stop-direction` 两个字段"，我们建 **POI（基础设施侧实体）**。二者不可混用
   （前者零新增实体、后者可被独立引用与复用）⇒ 已写进 `docs/poi.md` §8，
   防止后续被"OpenTTD 就是这么做的"带偏。

---

## 5. 二次调研：px-patch 如何退出"等待"命令（2026-09-10）

> 起因：用户裁决"先看 pulsexlb 到底怎么解决'等待如何退出'，暂不自己拍板"。
> 三问三答，全部有源码位置。**注意：本轮推翻了我们一个想当然的假设。**

### Q1 连挂那一刻，"等待"命令怎么被越过 → **由 `Couple()` 末尾主动推进**

源码事实（`train_cmd.cpp:6660-6679`）：交割**整份**调度状态后，**条件式强制越格**：

```cpp
v->primary_order = u->primary_order;
v->primary_order_index = u->primary_order_index;
v->cur_real_order_index = u->cur_real_order_index;
/* ...Advance past it, otherwise the merged consist would re-enter the wait and never continue. */
if (u->current_order.IsType(OT_WAIT_COUPLE)) v->IncrementRealOrderIndex();
```

末尾再 `IncrementImplicitOrderIndex(); ProcessOrders(v);`（`train_cmd.cpp:6739-6741`），
随后才是物理合并 `TryTrainCouple()`。

**跳过规则**（`SkipToNextRealOrderIndex()`，`vehicle_base.h:1033`）：

```cpp
do { cur_real_order_index++;
     if (>= GetNumOrders()) { = 0; wrapped = true; }
} while (IsType(OT_IMPLICIT) || IsType(OT_DECOUPLE));
```

⇒ **只跳过 `OT_IMPLICIT` 与 `OT_DECOUPLE`，末尾回绕到 0**；**不跳过**
`OT_WAIT_COUPLE` / `OT_WAITING` / `OT_DUMMY` / `OT_LABEL`。
（另有一个 `UpdateRealOrderIndex()`，`vehicle_base.h:1110`：只跳 `OT_IMPLICIT`、
不带回绕标记，是"发车前规整"，与前者不同。）

**等待车自身**无需处置：它在 `Couple()` 开头就被 `->Primary()` 归一化，计划状态
已整体拷给行驶车，随后被并入并销毁——**不存在"带着残留等待命令活着"的等待车**。

### Q2 被解挂侧怎么处置 → **该场景在 px-patch 里不可达**

- 解挂**只有一个触发点**（`train_cmd.cpp:7003`）：
  `consist->current_order.GetDestination() == station && GetDecouple() == ODF_DECOUPLE`
  —— 即"**进站 + 当前命令是带解挂标记的站台命令**"。
- 而 `OT_WAIT_COUPLE` **既无 `dest` 也无解挂位**（`MakeWaitCouple()` 只设 `type`，
  `order_cmd.cpp:256-258`），且等待车每帧被 `HoldWaitingTrainBody()` 钉住不前进
  （`train_cmd.cpp:8937-8948`）⇒ **"等的人没来、车被强拆"这条路径在 px-patch 里走不到**。
  （已搜标识符：`OT_WAIT_COUPLE` / `OT_DECOUPLE` / `GetDecouple()` / `CanDecouple` /
  `GetDecoupleVehicle` / `SplitOrders` / `DecoupleTrain` / `decouple_part` / `JustDecoupled`；
  `GetDecouple()` 全文只有 5 处引用，无第二处解挂入口。）
- **旁证（作者的预见，但没定语义）**：`SplitOrders()` 之后有两行防御性清理
  （`train_cmd.cpp:7043-7044`）：
  `if (consist->current_order.IsType(OT_WAIT_COUPLE)) FreeTrainTrackReservation(consist);`
  —— 只**释放进路预约**，**不推进指针、不清除等待命令**。
- 新 OrderList 的起点：`ODOF_KEEP_ORDERS` 分支逐字段抄游标；**走"新建表/接管计划"
  分支则一律从 index 0 起**（`train_cmd.cpp:6045-6048` 显式置 0）。
  **没有任何"把等待标记为已满足"的处理。**
- ⇒ **"等待的退出"在 px-patch 里只有"连挂"一个出口**（另一处弱相关：时刻表等待
  超时会 `MakeDummy()` 把那条命令变成真正的 no-op，然后被自动跳过，见 Q3）。

### Q3 "空命令"与跳转目标的表达 → **他们根本没有"空命令"这一原语**

- **`OT_DUMMY` 不是玩家占位符，而是"目标被删后的残骸标记"**：`MakeDummy()` 只在
  站点/车库被批量删除时（`order_cmd.cpp:4360-4370`、`4410-4418`）与**等待超时**时
  （`vehicle.cpp:3977`）被调用；且它是**真 no-op**——`UpdateOrderDest` 里
  `case OT_DUMMY: case OT_LABEL:` 直接 `IncrementRealOrderIndex()` 走人
  （`order_cmd.cpp:5419-5425`）。**不能当跳转落点**（玩家无法主动插入，且会被删除站点改写）。
- `OT_LABEL`（`OLST_TEXT` / `OLST_DEPARTURES_VIA` / …，`order_type.h:147-152`）是
  **出发板/展示**用，也不是跳转目标；`OT_IMPLICIT` 是游戏生成、对玩家隐藏、被索引
  推进自动跳过；`OT_SLOT_GROUP` 属 Trace Restrict，非占位。
- **跳转目标是"订单表下标"**：`GetConditionSkipToOrder() { return this->flags; }`
  （`order_base.h:562`）——`OT_CONDITIONAL` 直接把下标塞进 `flags`；
  `OT_EXECUTE_SCHEDULE` 的目标是**另一张表的 ID**（`SetDestination(ol->index)`），
  回归位置另存 `primary_order`/`primary_order_index`（`vehicle_base.h:383-384`），
  末尾 `ReturnFromExecuteSchedule()` 归位（`order_cmd.cpp:5166`）。
  ⚠ `order_list_serialisation_reference.md` 里的 `jump-to: <label>` **只是 JSON
  导入导出的字符串化呈现**，落地存储仍是下标。**没有标签式跳转。**
- **⚠ 推翻我们此前的推测**：`AdoptDecoupleSchedule()` 拼的三条
  （`train_cmd.cpp:6031-6041`）**全部是有实义的命令**——① `MakeGoToStation`（真前往）
  ② `MakeExecuteSchedule`（去执行另一张表）③ `MakeConditional(execute_index)` +
  `SetConditionVariable(Unconditionally)`（**无条件回跳到 index=1**）。
  ⇒ **他们没有任何"空命令/哨兵条目"原语**；回跳落点就是一条普通的条件跳转命令。

### 对我们的三条可操作结论（调研 agent 给出，本稿认可）

1. **"等待的退出"必须由外部事件显式推进指针，且推进后要校验落点**。px-patch 的连挂
   出口形态可直接照搬：**交割指针 → 若停在等待命令上则强制步进 → 再跑一次计划处理**。
   映射到我们：**解挂回收控制权时，也要判断"当前指针是否停在等待命令上"**——
   我们的"先通知、后销毁"顺序正是这个调用的落点。
2. **推进原语必须显式定义"跳过哪些命令类型"**（他们只跳 `OT_IMPLICIT`/`OT_DECOUPLE`
   并回绕）。我们目前只有 goto ⇒ **建议现在就定**：哪些类型参与"指针自然步进"，
   哪些**只由事件推进**（等待类必属后者）。
3. **不要引入"空命令/哨兵"原语**。用户示例里的"3 空命令"应重新定义为**一条真有语义的
   命令**（如"接管计划已恢复"/"无条件跳至下一条"），否则将来无法区分"刻意的空操作"
   与"失配的残骸"（他们的 `OT_DUMMY` 就是后者）。

### 仍需用户拍板的两点

1. **被跳过的失效命令在 GUI 里长什么样**：他们用 dummy（可见、可被玩家手动删除）；
   我们已定"悬空引用 → 跳过该条、计划保留"（比他们宽容）⇒ 需要决定**跳过条目如何显示**。
2. **跳转目标用"下标"还是"标签"**：px-patch 只有"下标"与"另一张表 ID"两种，
   **没有标签式跳转**。若我们要"按名字跳"，那是**新增原语**，需明确它与"下标指针"模型的关系。

---

## 6. 三轮取证：官方**文档**侧（2026-09-10，前两轮读的是源码）

前两轮从源码反推语义；本轮把用户指定的**中文官方文档**逐页读完（10 页，含
[1.1](https://pulsexlb.github.io/OpenTTD-PXPatch-Docs/zh/docs/1-%E5%88%97%E8%BD%A6%E8%A7%A3%E6%8C%82/1.1-%E7%AE%80%E5%8D%95%E7%9A%84%E5%88%97%E8%BD%A6%E8%A7%A3%E6%8C%82/) /
[1.2 操作](https://pulsexlb.github.io/OpenTTD-PXPatch-Docs/zh/docs/1-%E5%88%97%E8%BD%A6%E8%A7%A3%E6%8C%82/1.2-%E8%A7%A3%E6%8C%82%E5%88%97%E8%BD%A6%E6%93%8D%E4%BD%9C/) /
[1.3](https://pulsexlb.github.io/OpenTTD-PXPatch-Docs/zh/docs/1-%E5%88%97%E8%BD%A6%E8%A7%A3%E6%8C%82/1.3-%E8%A7%A3%E6%8C%82%E8%B0%83%E5%BA%A6%E8%AE%A1%E5%88%92%E5%AE%A2%E6%B5%81%E5%88%86%E9%85%8D/) /
[2.1](https://pulsexlb.github.io/OpenTTD-PXPatch-Docs/zh/docs/2-%E5%88%97%E8%BD%A6%E5%AF%B9%E6%8E%A5/2.1-%E7%AE%80%E5%8D%95%E7%9A%84%E5%88%97%E8%BD%A6%E5%AF%B9%E6%8E%A5/) /
[2.2 操作](https://pulsexlb.github.io/OpenTTD-PXPatch-Docs/zh/docs/2-%E5%88%97%E8%BD%A6%E5%AF%B9%E6%8E%A5/2.2-%E5%88%97%E8%BD%A6%E6%93%8D%E4%BD%9C/) /
[3 模块化机场](https://pulsexlb.github.io/OpenTTD-PXPatch-Docs/zh/docs/3-%E6%A8%A1%E5%9D%97%E5%8C%96%E6%9C%BA%E5%9C%BA/) /
[自定义调度计划表](https://pulsexlb.github.io/OpenTTD-PXPatch-Docs/zh/docs/%E8%87%AA%E5%AE%9A%E4%B9%89%E8%B0%83%E5%BA%A6%E8%AE%A1%E5%88%92%E8%A1%A8/)，
后者正文仍只有 "WIP"）。文档侧的价值：**玩家可见语义**（源码看不到），
且能验证前两轮的源码结论是否与作者自己的描述一致。**以下逐条都是文档原文照录。**

### 6.1 解挂命令**自带"两部分各自的调度计划"** → 印证我们的 Q16（文档级铁证）

1.2 页原文："可以编辑**解挂后两个部分**列车的调度计划，以及解挂的数量。"
下拉菜单 6 个选项，文档把每个选项**"实际生成的调度计划"**列了出来：

| 选项 | 行为 | 实际生成的调度计划（原文） |
|---|---|---|
| 保留调度计划不装卸 | 保留原计划、不装卸 | 原有调度计划（**备注：将共享原调度计划**） |
| 装卸并保留调度计划 | 保留原计划 + 本站装卸 | 原有调度计划（同上） |
| 等待对接不装卸 | 在解挂站等待连接 | 一条「等待连接」 |
| 装卸并等待对接 | 本站装卸后等待对接 | 一条「前往解挂站」+ 一条「等待连接」 |
| 使用调度计划不装卸 | 执行玩家自定义计划表 | 一条「执行调度计划」 |
| 装卸并使用调度计划 | 装卸后执行自定义计划表 | 一条「前往解挂站」+ 一条「执行调度计划」+ **一条「条件性命令跳过：总是跳至第二项」** |

⇒ **"解挂后干什么"是解挂命令的属性，不是继承来的状态**——这正是我们 **Q16**
（解挂命令声明后段计划来源）所采纳的方向，且**他们的选项表就是 Q16 需要的那份枚举**
（我们只需把"装卸"两项去掉：货物 demo 不做）。
⇒ 反差也要记下：**"保留原计划"那一项的备注是"将共享原调度计划"**——他们真的共享
同一对象（与源码 `AddToShared` 一致），我们 Q16 已明确**不共享、现场构造/指派**。

### 6.2 连挂后的计划来源**也有显式规则**（默认取"前往方"） → 我们缺这条表述

2.2 页原文："默认情况下，完成对接后的列车将使用**原前往对接列车**的调度计划，
不过也可以在「管理调度计划」中勾选「**使用等待对接列车的调度计划**」选项。"
⇒ 在他们的模型里，"合并后执行谁的计划"是**一个可配置的策略**，默认给**主动方**。
⇒ **对我们的启示（重要）**：我们已定 Q16 的解挂侧，但**连挂侧的"合并后执行谁的计划"
还只有 Q5/Q6 的推论**（遴选 `priority` 大者 → `wagon_id` 小者；落选计划不搬移、
不丢弃、留在其控制车上，**不交割**）。这与他们"交割一份计划"是**相反路线**，
必须写清楚，否则任务 3 写实现时容易顺手照抄成"合并即继承对方计划"。

### 6.3 跳转目标的**玩家可见语义 = 序号**（"总是跳至第二项"） → 支持 Q17-1 选"下标"

同一个"装卸并使用"选项生成的第三条命令，文档里的名字就是
「**条件性命令跳过：总是跳至第二项**」——**"第二项"是位置表述**，与源码
`SetConditionVariable(Unconditionally)` + `SetConditionSkipToOrder(1)` 完全对应
（见 §5 Q3）。⇒ **连玩家手册都用序号表达跳转目标**，不引入"标签"这一层；
他们**唯一的跨表引用**用"另一张表的 ID"（`OT_EXECUTE_SCHEDULE`），也不是名字。
⇒ 结论：**"下标 + 跨计划用 ID" 是他们验证过的组合，且没有暴露可用性问题**。

### 6.4 他们的"静默失败"是**文档自陈的缺陷** → 支持 Q17-3 选"可见 + 带原因"

- 1.1 页："目前**解挂失败时，游戏不会给出任何提示**。"
- 2.1 页："目前**对接失败时，游戏不会给出任何提示**。"
（语气明显是"待改进"，不是设计取舍。）
⇒ 我们 Q11 已定"悬空引用 → 跳过该条、计划保留"，比他们宽容；**失败原因可见**
是他们的**现存缺口**，我们补上不算"照抄"，反而更彻底。

### 6.5 「前往连接」= 全图搜索 + 5 类限制；找不到目标 → **stuck（被信号阻塞）**

2.1 原文："当一个列车调度计划为前往连接时，它将默认在**全图范围内**搜索所有等待
对接的列车……如果一列列车无法找到任何一列符合要求的等待对接的列车，该列车将被
设置为 **stuck** 状态……列车将被会任何一个信号灯阻塞，直到出现一列符合要求的列车。"
2.2 页列出 5 类对接限制：**对接空车/满载、货物类型、路签、连接数量（车厢数）、车站**。
⇒ **新待研讨（我们尚无对应设计）**：我们的"前往连挂"命令要不要搜索+限制？
最重要的是**"找不到目标"的语义**——他们用 stuck（**等于永久等待**），
而我们 Q11 对"悬空引用"用的是**跳过该条命令**。两者冲突的根源是
**"永久失效"与"暂时不可用"没有区分**：
- 引用对象**不存在**（POI 被删）→ 永久 ⇒ 跳过（Q11）。
- 对象**存在但条件不满足**（还没有车在等对接）→ 暂时 ⇒ **等待**（命令语义就是等）。
⇒ 建议把这条区分写成规则（详见 §7 待研讨 Q18）。

### 6.6 解挂的**前置条件** → 直接支撑 POI 的"有向点要带停车位置"与"站台长度"

1.1 页的"列车解挂要求"（不满足就不执行解挂）：
- **解挂列车不完全停靠在站台内（列车长度 > 站台长度）** ⇒ **站台有长度、列车要整列
  停进去**：这既需要"停车位置"（我们的有向点要带 `t`），也需要**站台的范围/长度**
  概念（我们的 **路段 POI**）。
- **解挂位置在双头机车内部** ⇒ 解挂点不能落在机车中间（与我们的"内部车钩"概念对应）。
- **解挂后的列车不符合 start-stop 规则**（newgrf）⇒ 我们的等价物是"解挂后无控制车的
  段静止"（Q6），**我们已有定论**。
- 1.1 页另有两条**用户可见的坑**（值得当反例记）：① "如果解挂车厢节数 > 实际车厢
  节数，那么列车也会解挂一节车厢"；② "当解挂列车节数为 0 辆时，将自动解挂车头的机车"
  ——数字越界与 0 都走**隐式默认**，我们应避免这种"隐式兜底"（宁可报错/拒绝）。

### 6.7 对照记录：他们承认的物理坑（我们已避开）

2.1 页末："**双头机车对接后，可能会反转部分车厢顺序**，这与对接方向有关，
且仅会在双头机车上出现。" ⇒ 这正是我们 2026-09 拍板防止的事
（**折返只换向、不反转编组**；`reversed_consist` 已删）。**记为"我们的选择有外部佐证"。**

### 6.8 文档侧对 Q17 三问的净结论（供用户拍板时参考）

| Q17 子问 | 文档侧证据 | 倾向 |
|---|---|---|
| ① 跳转目标：下标 vs 标签 | 他们**只有下标**（玩家手册也写"第二项"），跨表另有"表 ID" | **下标 + 跨计划引用用 ID**，不做标签 |
| ② 步进语义表 | 文档不涉及（源码只有 `SkipToNextRealOrderIndex` 那套，见 §5 Q1/Q2） | 仍须我们自己定（任务 3） |
| ③ 失效命令显示 | 他们**连失败都不提示**（自陈缺陷）；只有 dummy 残骸可见 | **可见 + 带原因**（补他们的缺口） |

---

## 7. 三轮取证新带出的待研讨（编号沿用 `wagon_centric_data.md` §5）

### Q18（新）"命令失效"要不要分**永久**与**暂时**？

**问题**：px-patch 对"前往连接找不到目标"给的是 **stuck = 永久等待**（被信号阻塞，
直到有车来），而对"目标被删"给的是 **dummy 残骸 + 跳过**。我们目前只有一条规则
（Q11：悬空引用 → 跳过该条），**没有区分**这两类。它们该不该分开？

**倾向（待用户拍板）**：**分**——判据是"**被引用的对象还在不在**"，而不是
"现在能不能做"：

| 情形 | 判据 | 语义 |
|---|---|---|
| **永久失效** | 引用的**对象不存在**了（POI/建筑被删、计划表 ID 无效） | 该条**跳过**（Q11 不变），**GUI 标红 + 原因**（Q17-3） |
| **暂时不可用** | 对象**在**，但**条件不满足**（还没车在等对接、目标车正在被别的车连挂） | **原地等待**（这正是"等待/前往连接"命令的本义），**GUI 提示在等什么** |

理由：把人能一眼看懂的区别压成一种行为，会同时坏掉两头——真失效时列车"卡在那等
一个永不再来的站台"（玩家的计划成了僵尸），或条件未到时"静默跳过一整段计划"
（玩家不知道为什么列车少走了一站）。

### Q19（新）「前往连挂」要不要**目标搜索 + 限制条件**？

px-patch 的「前往连接」是**全图搜索** + 5 类限制（空车/满载、货物类型、路签、
车厢数、车站）。我们现状：连挂是**手动**（K 键 / 右键吸附到某车端头车钩），
**没有"去找一辆符合条件的车"这一层**。
**待定**：demo 的计划层是否纳入"前往连挂"命令？若纳入，**是否要搜索与限制**
（还是只允许"指向某个具体 POI/列车"的最简形态）？

### Q20（新，小）POI 需要"停车位置"与"站台范围/长度"

px-patch 的解挂触发条件之一是**整列车必须完全停靠在站台内**（列车长度 ≤ 站台长度）。
⇒ 站台不只是"一个点"，还带**范围/长度**；停车也不只是"停在 POI 上"，而要
**停在范围内**。这条直接落到 `poi.md`（新增 **O9**）。

### 表述补丁（不算新问题）：连挂侧"合并后执行谁的计划"

见 §6.2。我们的路线是 **遴选（Q5/Q6）+ 计划不交割**，与 px-patch 的
**交割一份计划（默认给前往方）** 相反。**建议明确记为已定**（由 Q1/Q6 推出）：
"**连挂不交割计划**——合并后由遴选出的控制车执行其自己的计划，落选方的计划
**留在其车厢上不动**，将来解挂后原样继续。"（用户若无异议即视为定案。）
