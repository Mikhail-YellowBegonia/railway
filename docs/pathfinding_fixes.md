# 寻路系统调试记录

本文档记录列车寻路系统的关键设计决策和修复历程。

## 核心问题概述

列车在刚体车厢运动学集成后，出现一系列位置跳变、方向错误、路径重复等问题。经过系统性调试，定位并修复了 7 个关键 bug，并完成 2 个交互优化。

---

## 关键修复（按时间顺序）

### 1. 朝向约束禁止折返
**问题**：`find_path_between_nodes` 忽略了 `start_direction` 参数，导致列车可以原地掉头。

**修复**：
```python
# 起始有向边：根据 start_direction 确定
if start_direction > 0:
    start_directed = (start_edge_id, 1)  # 正向
else:
    start_directed = (start_edge_id, -1)  # 反向
```

**文件**：`model/pathfinding.py:184-192`

---

### 2. 路径拼接逻辑修复（位置跳变）
**问题**：车尾覆盖路径未正确传递给 `assign_path`，导致 `s` 初始化为 0，列车位置跳变。

**根本原因**：
- 车尾路径 `tail_path` 用于保持占位连续
- 但 `s` 应该初始化为车头在拼接路径中的位置（`s_head_in_tail`）
- 旧代码 `s=0` 导致车头跳到新路径起点

**修复**：
```python
tail_path, initial_offset_tail, s_head_in_tail = self.train.tail_coverage_path()
full_path = Path(edges=tail_path.edges + path.edges, ...)
self.train.assign_path(full_path, initial_offset_tail, end_offset)
self.train.state.s = s_head_in_tail  # 手动调整 s 为车头位置
```

**文件**：`controller/game_loop.py:466-520`

---

### 3. Edge 去重检查 `(edge_id, direction)`
**问题**：只检查 `edge_id` 重复，忽略了方向，导致同 Edge 反向拼接未被去重。

**修复**：
```python
if (tail_path.edges and path.edges and
    tail_path.edges[-1] == path.edges[0]):  # 比较完整元组
    full_path = Path(edges=tail_path.edges + path.edges[1:], ...)
```

**文件**：`controller/game_loop.py:523-531`

---

### 4. 同 Edge 反向目标返回不可达
**问题**：目标在同一 Edge 的反方向时，直接绕路，但没有检查 `allow_reversal`。

**修复**：
```python
if start_edge_id == goal_edge_id:
    # 目标在前方 → 直接返回
    if (start_direction > 0 and start_t <= goal_t) or \
       (start_direction < 0 and start_t >= goal_t):
        return path, start_offset, end_offset
    # 目标在反方向 → 检查是否允许折返
    if not allow_reversal:
        return None
```

**文件**：`model/pathfinding.py:247-267`

---

### 5. `current_direction()` 返回车头实际方向
**问题**：旧逻辑根据 `path.edges[0]` 判断方向，但列车可能已经行驶到后续边。

**修复**：
```python
def current_direction(self) -> int:
    cur_edge_id, _ = self.current_edge_and_t()
    for eid, d in self.state.path.edges:
        if eid == cur_edge_id:
            return d
    return 1  # fallback
```

**文件**：`model/train_entity.py:47-52`

---

### 6. 端点 fallback 朝向判断
**问题**：`start_t` 靠近端点时，直接从端点出发，但方向判断错误。

**修复**：
```python
if start_directed is None:
    if start_t < 0.5:
        # 靠近 node_a，根据 start_direction 构造有向边
        if start_direction > 0:
            start_directed = (start_edge_id, 1)  # 正向
        else:
            start_directed = (start_edge_id, -1)  # 负方向（从 node_b 出发）
    else:
        # 靠近 node_b，根据 start_direction 构造有向边
        if start_direction > 0:
            start_directed = (start_edge_id, 1)  # 正向（从 node_a 出发）
        else:
            start_directed = (start_edge_id, -1)  # 负方向
```

**文件**：`model/pathfinding.py:297-312`

---

### 7. 出发后隐藏橙色预览列车
**问题**：预览列车（橙色）和实时列车（红色）同时显示，混淆视觉。

**修复**：
```python
if self.train is not None and self.train.is_parked():
    preview_kin = RigidWagonKinematics(...)
    draw_debug_train(..., occupied=False)  # 仅停放时显示
```

**文件**：`controller/game_loop.py:127-137`

---

### 8. 允许在任意节点折返
**问题**：旧逻辑仅在终端节点（`connection_count == 1`）允许折返，复杂场景失败。

**修复**：
```python
# 折返：allow_reversal=True 时允许在任意节点沿原边反向出发
if allow_reversal:
    reversed_dir = (edge_id, -direction)
    if passable_fn(network.edges[edge_id], -direction):
        result.append((reversed_dir, REVERSAL_PENALTY))
```

**文件**：`model/pathfinding.py:97-102`

---

### 9. 总是允许折返，通过代价避免
**问题**：根据列车当前状态判断 `allow_reversal`，列车不在终点时无法规划折返路径。

**根本原因**：
- 旧逻辑：列车不在终点 → `allow_reversal=False` → 寻路完全禁止折返
- 但列车应该能规划"先到终点 → 停车 → 折返"的路径

**修复**：
```python
# 寻路时总是允许折返（通过高代价自然避免不必要的折返）
allow_reversal = True
```

**设计原理**：
- 折返边有高代价（`REVERSAL_PENALTY = 200`）
- Dijkstra 自动选择最优路径
- 能不折返 → 选择直达路径
- 必须折返 → 选择折返路径（代价+200）

**文件**：`controller/game_loop.py:469-475`

---

## 交互优化

### 1. 显示寻路虚拟节点
在调试视图中渲染虚拟分割点（黄色空心圆圈），帮助理解 Edge 中途寻路逻辑。

**文件**：`controller/game_loop.py:143-152`

---

### 2. 手动指定初始朝向 + 虚影预览
放置列车改为两步流程：
1. **第一步**：点击节点 → 显示所有方向的虚影（橙色）
2. **第二步**：点击相邻节点 → 确定朝向，创建列车实体

**实现**：为每条关联边生成临时 path 和 kinematics，绘制橙色虚影。

**文件**：`controller/game_loop.py:378-435, 154-181`

---

## 调试工具

### 日志系统
添加 `debug` 参数到 `find_path_from_point`，输出：
- 寻路起点、终点、朝向、`allow_reversal`
- 同 Edge 特殊处理（前方/反方向）
- 临时网络寻路参数
- 最终结果和路径

**启用方法**：
```python
result = find_path_from_point(..., debug=True)
```

**文件**：`model/pathfinding.py:208, 231-370`

---

## 关键设计原则

### 1. 位置表示的一致性
- **车头位置**：`state.s` 表示车头转向架在路径上的位置
- **车尾覆盖**：`tail_coverage_path()` 返回车尾占位路径，保证拼接时连续
- **初始化**：新路径 `s` 应为车头在拼接路径中的位置，不能简单设为 0

### 2. Edge 去重的完整性
- 必须检查 `(edge_id, direction)` 完整元组
- 仅检查 `edge_id` 会漏掉反向重复

### 3. 折返代价机制
- `REVERSAL_PENALTY = 200`：高代价避免不必要折返
- 总是允许折返：寻路算法自由规划，代价机制保证合理性
- 支持"先到终点再折返"的路径

### 4. 端点 fallback 的方向
- `start_t < 0.5`：靠近 `node_a`，根据 `start_direction` 构造有向边
- `start_t >= 0.5`：靠近 `node_b`，根据 `start_direction` 构造有向边
- **关键**：直接使用 `(start_edge_id, start_direction)` 构造有向边

---

## 测试场景

### 环线寻路
- 列车在环线上任意位置
- 目标在前方 → 直达路径
- 目标在反方向 → 先到终点，折返返回

### 多分支场景
- 列车在分支节点
- 目标需要折返 → 正常规划折返路径

### Edge 中途寻路
- 起点在 Edge 中途（`start_t ≠ 0 或 1`）
- 终点在 Edge 中途（`goal_t ≠ 0 或 1`）
- 虚拟节点正确分割，路径正确映射

---

## 未来工作

### 自动停车
- 根据制动距离提前减速
- 精确停在目标位置
- 需要预测控制逻辑

### 多列车测试
- 碰撞检测
- 占位冲突解决
- 性能优化

### 可变编组（D-G 阶段）
- 连挂/解挂逻辑
- Wagon 级别的状态管理
- Multiple Unit 逻辑

---

## 提交历史

```
cf2bc9d fix(pathfinding): 总是允许折返，通过代价自然避免
09bd099 fix(pathfinding): 允许在任意节点折返（不限终端）
0ba0737 feat(train): 终点停车后允许折返
7b9c076 fix(visual): 修复 Camera.world_to_screen 调用签名
b58de8a fix(visual): 修复虚拟节点位置计算
4ef1770 feat(train): 手动指定初始朝向 + 虚影预览
ecdcc18 feat(visual): 显示寻路虚拟节点（调试用）
a042df6 docs: 记录列车系统关键设计决策和调试要点
8e0d430 fix(pathfinding): 修复端点 fallback 时的朝向判断
668b419 fix(train): 修复 current_direction 和初始朝向选择
bb80671 fix(pathfinding): 禁止同 Edge 反向目标，去重检查 direction
b968349 fix(train): 路径拼接时去重，避免 Edge 重复导致转向架回跳
2087b87 fix(train): 修复路径拼接逻辑，解决位置跳变
f68649c fix(visual): 出发后隐藏橙色预览列车
6788491 fix(train): 拼接车尾路径保持占位连续
cb94fac fix(pathfinding): 朝向约束禁止折返
```

---

## 总结

通过系统性调试和设计优化，列车寻路系统现已稳定支持：
- ✅ 刚体车厢运动学（转向架位置求解）
- ✅ Edge 途中寻路（起点和终点任意位置）
- ✅ 折返支持（总是允许，代价机制避免不必要折返）
- ✅ 朝向约束（保持列车朝向，禁止原地掉头）
- ✅ 路径拼接（车尾覆盖 + 去重，保证连续性）
- ✅ 手动朝向选择（虚影预览）
- ✅ 调试可视化（虚拟节点、制动区）

所有核心功能已完成，系统可以稳定运行并支持复杂场景测试。
