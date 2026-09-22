# ADR-001: 默认实时寻路，命令级固定路径

## Status

Accepted — 2026-09-22

## Context

调度计划既包含确定性意图，也包含 Station/Platform 中任一可用目标等声明式意图。
站台占用、车钩暴露状态、闭塞/信号和路网拓扑都可能在编辑后变化，因此不能把
“编辑期得到唯一确定路径”作为所有命令的约束。项目已有稳定的底层运行管线：

```text
路径生产 → TrainState.route → 预约 → 授权边界 → 物理位移
```

需要调整的是路径生产者的策略，而不是底层执行模型。

约束：

- 普通确定性条目应继续拥有可解释、稳定的路径语义。
- 声明式条目允许由游戏逻辑按需解析和刷新。
- 物理执行层、信号预约、车钩认领和编组交割规则应继续复用。
- 运行期刷新不得把 POI 变成无约束的自由寻路；每次刷新都必须产生一份可验证的
  当前执行路径。

## Options Considered

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| 所有计划编辑期冻结唯一 `FixedRoute` | 无法表达任一站台/动态占用 | 放弃 |
| 所有命令每帧无条件重建路径 | 开销和目标抖动不可控 | 放弃 |
| 改写预约、授权和物理执行层 | 影响面过大且没有必要 | 放弃 |
| 保留执行层，只切换命令级路径生产策略 | 兼容现有实现，动态与固定并存 | 采用 |

## Decision

每个 `PlanItem` 拥有命令级 `PathPolicy`：

1. `DYNAMIC` 是默认策略。激活和刷新时使用现有寻路器生成运行期快照，不写回计划意图。
2. `FIXED` 仅在玩家显式添加“路径固定”标签后使用，并携带 `FixedRoute`。
3. 带 `fixed_route` 但没有策略字段的旧条目按 `FIXED` 兼容处理。
4. Station、Platform 和非确定性 selector 不得被强制冻结成唯一编辑期路线。

运行期分三层：

```text
Plan intent → Resolution snapshot → Execution lease
```

- Intent：计划中长期稳定的目标表达。
- Resolution snapshot：当前具体 POI 成员、edge、车钩、目标点和路径。
- Execution lease：列车当前的 `route`、`goal`、已认领车钩和解析身份。

声明式连挂第一版只使用默认策略：停放目标、排除本编组、排除已认领端头，按进入
方向位置排序，再以 `wagon_id` 和端头顺序稳定打破平局。不加入车型、长度或其它筛选。

## Consequences

- `FixedRoute` 不再是所有条目的默认字段；动态条目只把解析结果作为运行期快照。
- POI/Station 解析发生在计划调度器，而不是 POI 基础模型中。
- 暂时的信号/闭塞等待不自动触发换目标；只有目标或路径快照失效时才刷新。
- 不使用固定路径时，游戏行为保持实时寻路，接近 OpenTTD/Transport Fever 2 的常规运行模型。
- 固定路径是试验阶段的高级补丁，不应成为计划默认模式或底层方法依赖。
- 动态实现应区分暂时阻塞与解析快照失效，避免信号等待造成无意义换目标。
- 现有 `edge_id + direction` selector 继续兼容，作为确定范围 selector。

## Revisit Trigger

- 需要跨会话保存计划时，必须补充 POI/拓扑稳定引用与解析版本。
- 需要站台长度匹配、车型筛选或多列车调度时，扩展 selector 策略，而不是修改物理
  连挂执行层。

## Implementation Boundary

- `model.plan.PathPolicy` 与 `PlanItem.effective_path_policy` 表达策略。
- `PlanDispatcher` 选择动态解析或固定路线，然后复用现有 `assign_route`、预约、授权
  和物理执行。
- `resolve_plan_item()` 继续作为唯一路径解析入口；动态结果不得写回 `fixed_route`。
