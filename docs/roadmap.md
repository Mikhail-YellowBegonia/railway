已完成：
- 编辑器核心功能（Steps 0-6，完整建造/删除/吸附/截断）
- 空间索引（340× 性能提升，60fps 保障）
- 转向许可与寻路算法（几何自动推断 + edge-based Dijkstra）
- 运动学质点模型（Path + s → Pose）
- 物理层 SimplePhysics（无状态接口，方向键控制）
- Debug 列车可视化（完整闭环：编辑 → 寻路 → 列车运动）
- 交互优化（A1 相机跟随 + A2 速度 HUD + A3 空格重置）
- 刚体车厢模型（D1-D5：数据结构 + 弧长/割线求解 + 刚体运动学 + 集成 + 可视化）
- 信号系统 Step 1-6（One-Way PBS 放置、固定闭塞划分、占用推导灯色、
  进路预约、远场/近场寻路拆分、**信号接入运动控制**）——详见
  `docs/train_control.md`。列车现已受信号制约：红灯前平滑停车等待、绿灯
  自动续行、hard_stop 兜底。
- 连挂/解挂完整交互（roadmap 待办 #1，2026-09）——规格与决策见
  `docs/consist_ui.md`，PLAY 模式下任意节间解挂、悬停端头连挂、驶向车尾
  到位自动连挂。
- 会话持久化（roadmap 待办 #2，2026-09）——`S` 键存列车状态、启动还原，
  规格见 `docs/session_persistence.md`。

进行中：
- 无

待办事项（2026-09 校准，按依赖/风险排序，与信号系统人工验收时一并
review 出的第一版 demo 缺口；信号接入运动控制已在此轮完成，见
`docs/train_control.md` Step 6）：
1. **连挂/解挂真实交互** ✅ 已完成（2026-09）：K 键调试式交互升级为正式车钩
   交互（完整规格与决策记录见 `docs/consist_ui.md`，纯判定逻辑在
   `controller/coupling.py`）：内部车钩（灰圆点）悬停 → tooltip「解挂 →
   N1+N2」→ K/空格/回车 确认；端头车钩（青方块，单色定稿）悬停 → tooltip
   「连挂目标 #k」→ K 确认；未贴住时 K 下达"驶向对方车尾"指令（车钩对齐：
   提前 head_offset 停车），到位停车事件帧自动连挂；任何右键寻路停在其它
   列车端头车钩 1m 内同样自动连挂（§9-4 定稿）。回归测试
   `tests/test_couple_ui.py`（模型层 + GameLoop 端到端）。
2. **会话持久化** ✅ 已完成（2026-09）：列车位置/速度/编组/待走 route/goal/
   目标速度随 `S` 键连同轨道、信号一起写入 `manual_track.geojson` 顶层
   `"trains"` 字段，启动自动还原（含行驶中列车）。规格与决策见
   `docs/session_persistence.md`，序列化逻辑在 `model/session.py`（有向边按坐标
   反查、wagon_id 直存、split_sibling 按共享边动态重建、单列反查失败跳过）。
   回归 `tests/test_session.py`（模型层 7 项 + GameLoop 端到端）。
3. 真实物理扩展（RealisticElectric: 牵引曲线、黏着、阻力）——
   `SimplePhysics` 已支撑基本"能开能停"体验，这是手感打磨，不阻塞
   demo 可玩性。
4. LOD 与相关优化
5. UI 改进，图形化——按 roadmap 排序原则，永远排最后。
6. 自动合并临近 Node（QoL/防误触，建造时实时合并，优先级最低）

排序原则：
1. 先按依赖关系，底层算法优先（刚体车厢 ✅ → 信号系统 ✅ → 信号接入
   运动控制 ✅ → 真实物理）
2. 再按实施风险，简单改动优先
3. 美工/UI 按需穿插，永远排最后

当前重心：真实物理扩展（roadmap #3；#1 连挂/解挂、#2 会话持久化已完成）


