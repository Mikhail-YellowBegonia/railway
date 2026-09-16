"""信号-运动调度（Step 6）：把固定闭塞 + 进路预约接入列车运动控制。

职责：每帧对每列车做一次"运动授权"计算与预约推进——列车物理上只能驶到
已预约闭塞区间末端（授权边界），前方红灯时在信号前平滑停车等待，绿灯时
向前推进预约并恢复行驶。这是 docs/train_control.md「Step 5」明确推迟的
"逐区间自动推进预约、失败就自动停车"的连续机制，现在补齐。

设计约束（与 CLAUDE.md「Graph topology invariant」一致）：调度只读图拓扑、
写 BlockManager 的预约表、写 TrainEntity.authority_remaining；绝不改
Node/Edge、绝不改 occupancy 的拓扑结构，列车状态仍只是标量偏移。

关键语义：
- **授权边界** = 列车已预约的最后一个闭塞区间的末端（下一个信号节点）。
  边界沿列车自身 route 计算（不是 block 的全 DFS 边集），所以道岔分支
  不影响"车该停在哪"。
- **最多预约当前路径片段 + 前方一个路径片段**。预算只沿本车固定 route
  统计信号入口，不统计全局信号保护包络；后者允许重叠、分叉，不是可计数的
  互斥区间。这样既避免把整条线路预约死，也不会把双向区间重复计数。
- **红灯停车余量**：授权边界落在红灯区间前端时，制动目标再往回缩
  SIGNAL_STOP_MARGIN，让车头停在信号机之前而不是精确压线——否则制动曲线
  的离散时间步过冲会让车头越过信号节点、把红灯区间的边吞进 occupancy
  （"闯红灯"的亚厘米级表现，也是车辆视觉重叠的直接原因）。
- **hard_stop 兜底**：授权边界落到车头之后（运行中删除/合并挂信号的边、
  或运行中放置新信号导致 block 重新划分），无法靠减速曲线挽回，强制急停。

纯模型模块，无 view/controller/pygame 依赖，可脱离 GUI 用脚本回归。
"""
from __future__ import annotations

from model.block import BlockManager
from model.pathfinding import DirectedEdge
from model.rail_network import RailNetwork
from model.signal import SignalTable
from model.train_controller import BrakingController
from model.train_entity import TrainEntity

# 车头当前 edge 之后最多提前预约的路径片段数。当前 edge 所在片段不计入：
# 列车进入当前片段后，应该能立即申请下一个片段。这也避免了从全局保护包络
# 反查"当前属于几个 block"——双向信号的包络天然重叠，不能作为预算单位。
MAX_RESERVED_SPANS_AHEAD = 1

# 红灯前停车的安全余量（米）：车头停在信号机之前而非精确压线。
SIGNAL_STOP_MARGIN = 0.5

# 授权边界落在车头之后超过该值（米）判为"已越过红灯"，触发 hard_stop。
HARD_STOP_EPS = 1e-6


class TrainDispatcher:
    """协调 network/signals/block_manager 与单列车之间的每帧调度。"""

    def __init__(
        self,
        network: RailNetwork,
        signals: SignalTable,
        block_manager: BlockManager,
    ) -> None:
        self.network = network
        self.signals = signals
        self.block_manager = block_manager

    def tick(
        self,
        train: TrainEntity,
        dt: float,
        v_target: float,
        trains: list[TrainEntity] | None = None,
    ) -> None:
        """推进一列车：预约推进 → 计算授权 → 状态转移 → 物理推进。

        trains: 当前全部列车，用于把"被其他车物理占用的 block"也判为红灯
        （与 compute_colors 的占用语义一致，避免视觉重叠）。传 None 时只按
        预约表判定（退化，供最小测试用）。
        """
        if train.is_parked():
            return  # 无指令，无动作

        # 调车全放行（2026-09 bug2 现场，用户裁决"调车忽略一切限制"）：
        # 列车正在"驶向某停放列车端头车钩"（couple_approach_partner 指向它，
        # 连挂驶向指令）时，整条预约/闭塞/占用链对它不适用——它就是要开进
        # 对方占用的受保护区间，授权直接给到目标停点（remaining_to_goal），
        # 由 update 的 min(remaining, authority) + stop_before 保证不压目标车钩。
        # holding（信号前等待）状态在此分支按剩余授权直接恢复，不再依赖 block
        # 续约。partner 是同会话显式设置（K/右键吸附）；会话重载后由
        # model/session.rebuild_couple_partners 按 goal 几何一次性重建（运行时
        # 引用不落盘）——**不做每帧几何猜测**，否则"普通寻路碰巧把 goal 设在
        # 别人车钩处"的第三方会被误豁免（回归安全网 tests/test_couple_ui.py）。
        partner = train.couple_approach_partner
        if partner is not None:
            if train.is_holding():
                if train.state.remaining_to_goal > BrakingController.STOP_EPSILON:
                    train.resume()
                else:
                    return
            train.authority_remaining = max(0.0, train.state.remaining_to_goal)
            train.update(dt, v_target)
            return

        others_occupied: set[int] = set()
        if trains is not None:
            for other in trains:
                if other is train:
                    continue
                if self._consist_maneuver_exempt(train, other):
                    continue  # 编组作业豁免，见 _consist_maneuver_exempt
                others_occupied |= {eid for eid, _d in other.state.occupancy.occupied}

        remaining_path = [train.head_directed_edge()] + list(train.state.occupancy.route)

        # 1) 预约推进：若固定路径上、车头当前 edge 之后已预约的路径片段不足
        #    MAX_RESERVED_SPANS_AHEAD 且前方仍有未授权边，尝试预约下一个片段。
        #    路径片段由本车 route 上的信号入口/下一安全边界定义；全局 DFS
        #    保护包络允许重叠、分叉，不参与预算。失败
        #    （被预约 or 被物理占用）则边界保持，列车停车等待。
        held_edges = self.block_manager.held_edges(train)
        frontier, reserved_spans_ahead = self._walk_frontier(
            remaining_path, held_edges,
        )
        if (
            reserved_spans_ahead < MAX_RESERVED_SPANS_AHEAD
            and frontier < len(remaining_path)
        ):
            next_seg = self.block_manager.truncate_to_next_route_span(
                self.network, self.signals, remaining_path[frontier:],
            )
            # 物理占用视同红灯：只看列车实际要走的边（edge 交集），不展开到
            # 整个 block——否则道岔无信号时 A/B 股道会因共享粗 block 互锁。
            next_seg_eids = {eid for eid, _d in next_seg}
            if not (next_seg_eids & others_occupied):
                self.block_manager.reserve_path(train, [eid for eid, _d in next_seg])
            held_edges = self.block_manager.held_edges(train)
            frontier, reserved_spans_ahead = self._walk_frontier(
                remaining_path, held_edges,
            )

        red_ahead = frontier < len(remaining_path)
        raw_authority = self._distance_head_to_frontier(remaining_path, frontier, train)
        if red_ahead:
            raw_authority -= SIGNAL_STOP_MARGIN

        # 2) hard_stop 兜底 vs 信号前等待的区分（2026-09 修复，重要）：
        #    授权边界落到车头之后（raw_authority < 0）有两种成因，不能一概
        #    hard_stop：
        #    (a) 列车**正在行驶**却已越过安全制动点（运行中删/并挂信号的边、
        #        或运行中放新信号导致 block 重划，减速曲线来不及）——这才是
        #        hard_stop 的本义（JGRPP realistic braking 的"无法安全制动"），
        #        必须清空指令、可见警告。
        #    (b) 列车**静止起步**时，前方受保护区间被别的车占用 → 预约失败、
        #        frontier 停在车头脚下第一条边，raw_authority 变负。这不该
        #        hard_stop（车没闯红灯，只是暂时等绿灯），否则会**清空 route
        #        丢指令**，玩家重新下令又再丢——表现成"信号死锁"（2026-09
        #        复杂场景实测：对向两车同时起步，双车 hard_stop 锁死）。
        #        正确处理是进入 holding（保留 route/goal，绿灯后续行）。
        #    判据：v 是否显著 > 0。起步时 v==0；闯红灯时 v>0。
        if raw_authority < -HARD_STOP_EPS:
            if train.is_moving() and train.state.v > BrakingController.STOP_EPSILON:
                train.hard_stop()
                return
            # 静止（起步预约失败）→ 保留指令停车等待（等绿灯，由下方状态转移续行）
            if train.is_moving():
                train.hold_at_signal()
            train.authority_remaining = 0.0
            return

        authority = max(0.0, raw_authority)

        # 3) 状态转移：信号前等待 →（续约成功）恢复 /（仍红灯）继续等。
        if train.is_holding():
            if authority > BrakingController.STOP_EPSILON:
                train.resume()
            else:
                return

        # 4) 写入授权并推进物理。
        train.authority_remaining = authority
        train.update(dt, v_target)

    # ------------------------------------------------------------------
    # 编组作业信号豁免（docs/consist_ui.md §5.5）
    # ------------------------------------------------------------------

    @staticmethod
    def _consist_maneuver_exempt(train: TrainEntity, other: TrainEntity) -> bool:
        """判定 other 是否应被 train 的物理占用检查豁免（编组作业配对列车）。

        **这是本系统里唯一被编组作业绕开的信号检查**，语义与边界如下：

        1. `train.couple_approach_partner is other`：train 正驶向 other 的端头
           车钩（连挂驶向）。此时 train 在保护区外、other 占着目标区间，两者
           通常不共享 occupied 边；豁免让 train 能预约进入 other 占用的区间，
           授权边界仍 clamp 在车钩处（§5.2 的 stop_before / update 的
           min(remaining, authority) 保证车头不压线）。

        2. `train.split_sibling is other`（且两者仍共享至少一条 occupied 边）：
           解挂后的前/后段。豁免仅在两者**仍共享边**时生效——两段驶离到不再
           重叠后自动失效，无需显式清除；这是"解挂后前段驶离不被后段占的
           共享边卡住"的关键。

        不被绕开的：`reserve_path`（预约表冲突）、`compute_colors`（信号灯仍
        红，视觉即"冒进"）、`_walk_frontier`/授权边界计算。豁免只作用于
        tick() 里 others_occupied 这一处。
        """
        if train.couple_approach_partner is other:
            return True
        if train.split_sibling is other:
            a_edges = {eid for eid, _d in train.state.occupancy.occupied}
            b_edges = {eid for eid, _d in other.state.occupancy.occupied}
            return bool(a_edges & b_edges)
        return False

    # ------------------------------------------------------------------
    # 授权几何
    # ------------------------------------------------------------------

    def _walk_frontier(
        self, remaining_path: list[DirectedEdge], held_edges: set[int],
    ) -> tuple[int, int]:
        """沿 remaining_path 返回 (授权边界边数, 前方已预约路径片段数)。

        授权边界 = 能连续通过的最长前缀的边数：前缀里每条边要么被我预约
        （edge 交集语义），要么不属于任何信号保护包络；遇到"受保护但未预约"
        的边即停。

        预算不是全局 block key 的数量。`BlockManager` 的 DFS 结果是每个信号的
        保护包络：双向信号的包络可以完全重叠，道岔处也可以分叉，因此它不是
        路网的 partition，更不是 PBS 的预约单位。这里改为只数固定路径上、
        车头当前 edge（index 0）之后已经通过预约前缀覆盖的信号入口；每个入口
        对应一个实际 route span。当前 edge 所在片段不计入，允许滚动预约前方
        一个片段，同时避免长列车车尾和重叠包络虚增预算。
        """
        protected_edges: set[int] = set()
        for edges in self.block_manager.all_protection_envelopes().values():
            protected_edges.update(edges)

        frontier = 0
        reserved_spans_ahead = 0
        for index, directed in enumerate(remaining_path):
            edge_id, _direction = directed
            if edge_id not in protected_edges:
                frontier = index + 1  # 无保护路段：天然可通行
                continue
            if edge_id in held_edges:
                if index > 0 and self.signals.has_signal(directed):
                    reserved_spans_ahead += 1
                frontier = index + 1
            else:
                break  # 受信号保护但未被预约：授权边界
        return frontier, reserved_spans_ahead

    def _distance_head_to_frontier(
        self, remaining_path: list[DirectedEdge], frontier: int, train: TrainEntity,
    ) -> float:
        """车头到授权边界节点（remaining_path[frontier-1] 的 head 节点）的
        有符号弧长。frontier=0（车头已在红灯边内）时返回负值。
        """
        covered = sum(
            self.network.edges[eid].length for eid, _d in remaining_path[:frontier]
        )
        head_edge_len = self.network.edges[remaining_path[0][0]].length
        return covered - self._head_fraction(train) * head_edge_len

    @staticmethod
    def _head_fraction(train: TrainEntity) -> float:
        """车头在其当前有向边上已走过的比例（0 = 有向边尾端点，1 = 头端点）。"""
        _eid, t_edge = train.current_edge_and_t()  # t_edge 是 node_a→node_b 参数
        direction = train.current_direction()
        return t_edge if direction > 0 else (1.0 - t_edge)
