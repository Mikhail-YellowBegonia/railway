# 调研笔记：OpenTTD-PXPatch（连挂/解挂 + 调度计划）

> **性质**：**调研记录，不是决策**。结论分「他们怎么做 / 对我们的启示 / 置信度」，
> 采纳与否由研讨拍板；采纳后写进 `wagon_centric_data.md` / `poi.md` / 未来的计划层文档。
> **完成时间**：2026-09-10。**执行方式**：后台 subagent（独立上下文）完成，未改动仓库。
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
