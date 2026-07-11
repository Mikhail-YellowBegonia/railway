# 火车驾驶游戏

## 技术栈
- Python >= 3.11, uv, pygame-ce 2.5.7
- MVC 架构: model / view / controller

## 轨道模型
- 三维坐标 (x, y, z), 本地米制, Z 暂存高程
- 图拓扑: Node + Edge, 无向
- Node 连接数: 1(尽头) 2(中间) 3(道岔) 4(交叉)
- 默认全互通, 连通性约束后续细化
- GeoJSON LineString 定义轨道:
  - 2 点 = 直线
  - 3 点 [A,B,C] = 圆弧 (minor arc), B 为 A/C 两切线的交点
  - ≥4 点 = 非法
  - 约束: A/B/C 不共线, |BA| = |BC|
- 圆弧 Edge 存储: center, radius, signed_angle, start_dir, normal
- sample_arc_points() 用于渲染, 模型层可直接使用曲线方程

## 渲染层
- pygame-ce 2D 调试视图
- Camera: 平移(左键拖拽)/缩放(滚轮), Y轴向上
- Renderer: 直线(灰色) 圆弧(蓝色) 节点(颜色按连接数)
- 运行: `uv run python main.py`

## 列车运动学与物理
- **刚体车厢模型**: 每节车厢由多个转向架约束，保持刚体长度不变
- **运动学**: `RigidWagonKinematics` 求解转向架位置（弧长 → 世界坐标）
- **物理**: `RealisticElectric` 计算牵引力、阻力、制动力，输出加速度
- **状态管理**: `TrainEntity` 统一管理 `TrainState`（路径、位置、速度）
- **位置真值**: `state.s` 是唯一位置来源，由 `TrainEntity.update()` 累加（`s += v * dt`）
- **路径表示**: `Path` 包含有向边序列 `[(edge_id, direction), ...]`，`direction ∈ {-1, +1}`

## 寻路系统
- **find_path**: 基于 Dijkstra 的有向边寻路，支持转向约束（`turn_allowed`）
- **find_path_from_point**: 支持从 Edge 中途出发，分割虚拟节点，指定朝向约束
- **朝向约束**: `start_direction` 参数禁止原地折返，列车保持朝向不变（必要时绕远）
- **关键修复**:
  - 端点 fallback 时根据 `(start_t, start_direction)` 直接构造有向边，不依赖 `_directed_from`
  - `current_direction()` 从车头当前所在 edge 查找方向，而不是 `path.edges[0]`
  - 路径拼接去重时比较完整元组 `(edge_id, direction)`，避免误判反向边

## 路径拼接与占位连续性
- **车尾覆盖路径**: `tail_coverage_path()` 返回 `(tail_path, initial_offset_tail, s_head_in_tail)`
- **拼接逻辑**:
  1. `full_path = tail_path.edges + path.edges`（去重检查 `(edge_id, direction)`）
  2. `full_start_offset = initial_offset_tail`（车尾在首段的偏移）
  3. `assign_path()` 后手动设置 `state.s = s_head_in_tail`（车头位置）
- **设计要点**:
  - `sub_path()` 返回完整 edge 段 + `initial_offset`，不裁剪到精确弧长
  - 拼接保证车尾到车头的完整覆盖，避免位置跳变

## 交互模式
- **F 键**: 切换列车模式
- **首次点击**: 在节点放置列车（优先选择正方向边，即 `node_a_id == node_id` 的边）
- **后续点击**: 从当前位置寻路到目标（支持 Edge 中途，朝向约束生效）
- **键盘控制**: ↑ 加速，↓ 减速，空格 紧急停止
- **可视化**:
  - 橙色路径线 + 橙色预览列车（停放时）
  - 红色边框 + 红色转向架（运行中，实时占位）
  - HUD 显示速度、加速度、制动区距离

## 待实现的微调
1. **显示虚拟节点**: 在寻路时临时创建的虚拟分割点（调试用）
2. **手动指定初始朝向**: 放置列车时让玩家点击相邻节点选择朝向
3. **朝向预览虚影**: 放置前显示列车虚影（需要生成临时车尾 path）

## 关键设计决策记录

### 位置表示与更新
- **唯一真值**: `state.s` 是车头在当前 `path` 上的弧长位置，所有几何查询从此派生
- **更新职责**: `TrainEntity.update()` 负责 `s += v * dt`，`kinematics` 只负责几何查询
- **路径切换**: `assign_path()` 重置 `state.s = 0`，调用方需手动调整到实际位置

### 寻路起点与朝向
- **从车头出发**: `find_path_from_point` 从车头位置（`current_edge_and_t()`）开始寻路
- **车尾拼接**: 寻路后拼接 `tail_coverage_path()`，确保新路径覆盖整列车身
- **朝向传递**: `start_direction` 从 `current_direction()` 获取，必须反映车头当前所在 edge 的实际方向

### Edge 重复与去重
- **何时重复**: 车头在 Edge 0 上，`tail_path` 包含完整 Edge 0，新 `path` 也从 Edge 0 开始
- **去重策略**: 比较 `tail_path.edges[-1] == path.edges[0]`（完整元组），相同则跳过 `path.edges[0]`
- **注意**: 反向边 `(0, -1)` 和正向边 `(0, +1)` 不应去重，它们是不同的有向边

### 端点 fallback
- **何时触发**: `start_t < 0.01` 或 `start_t > 0.99`，不分割虚拟节点
- **旧错误**: 用 `_directed_from(edge_id, start_node_id)`，`start_node_id` 选错导致反向
- **修复**: 直接根据 `(start_t, start_direction)` 构造有向边，不依赖节点推断

### 同 Edge 反向目标
- **判定**: `start_edge_id == goal_edge_id` 且目标在反方向（如 `dir=+1, start_t=0.8, goal_t=0.2`）
- **处理**: `allow_reversal=False` 时直接返回 `None`（不可达），不尝试绕路
- **合理性**: 列车不能原地掉头，必须保持朝向（绕远路或提示不可达）
