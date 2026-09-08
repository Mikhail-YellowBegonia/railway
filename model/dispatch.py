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
- **最多预约 2 个受保护区间**（当前段 + 前方一段，OpenTTD one-block-ahead），
  避免把整条线路预约死，也避免列车在绿灯前频繁停车。
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

# 车头前方最多预约的受保护闭塞区间数（"当前段 + 前方一段"）。注意预算只
# 数"车头前方"的 block——车身横跨多个 block 时（长列车），车尾尚未完全驶离
# 的 block 仍被 tick_reservations 持有，但那是"车头后方/正下方"，不该占用
# 前方预约预算（否则长列车会在绿灯前被永久卡死）。
MAX_AHEAD_BLOCKS = 2

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

        others_occupied: set[int] = set()
        if trains is not None:
            others_occupied = {
                eid for other in trains if other is not train
                for eid, _d in other.state.occupancy.occupied
            }

        remaining_path = [train.head_directed_edge()] + list(train.state.occupancy.route)

        # 1) 预约推进：若车头前方已预约的 block 数不足 MAX_AHEAD_BLOCKS 且更
        #    前方还有区间，尝试预约下一个受保护区间（全有或全无）。失败
        #    （被预约 or 被物理占用）则边界保持，列车停车等待。
        held = self.block_manager.held_blocks(train)
        frontier, blocks_ahead = self._walk_frontier(remaining_path, held)
        if len(blocks_ahead) < MAX_AHEAD_BLOCKS and frontier < len(remaining_path):
            next_seg = self.block_manager.truncate_to_next_signal(
                self.network, self.signals, remaining_path[frontier:],
            )
            # 物理占用视同红灯：block 边集与其它车 occupied 有交集则不预约。
            block_key = next_seg[0]
            full_block = self.block_manager.block_edges(block_key)
            if not (full_block & others_occupied):
                self.block_manager.reserve_path(train, [eid for eid, _d in next_seg])
            held = self.block_manager.held_blocks(train)
            frontier, blocks_ahead = self._walk_frontier(remaining_path, held)

        red_ahead = frontier < len(remaining_path)
        raw_authority = self._distance_head_to_frontier(remaining_path, frontier, train)
        if red_ahead:
            raw_authority -= SIGNAL_STOP_MARGIN

        # 2) hard_stop 兜底：授权边界已落到车头之后（网络编辑/信号放置导致
        #    的异常），减速曲线已来不及，强制停车保住"不闯红灯"。
        if raw_authority < -HARD_STOP_EPS and train.is_moving():
            train.hard_stop()
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
    # 授权几何
    # ------------------------------------------------------------------

    def _walk_frontier(
        self, remaining_path: list[DirectedEdge], held_blocks: set[DirectedEdge],
    ) -> tuple[int, set[DirectedEdge]]:
        """沿 remaining_path 从车头向前走，返回 (授权边界边数, 车头前方已预约
        的 block key 集合)。

        授权边界 = 能连续通过的最长前缀的边数：前缀里每条边要么属于某列车已
        持有的 block，要么不属于任何 block（无保护路段）；遇到"属于某 block
        但未被持有"的边即停（红灯边界）。

        blocks_ahead 只统计"车头前方"扫描到的已持有 block——车身横跨多个
        block 时，车尾尚未驶离的 block 不在 remaining_path 里、不计入，因此
        不会被它挤占"前方预约预算"（长列车绿灯前卡死 bug 的根因）。
        """
        held_edge_sets = {d: self.block_manager.block_edges(d) for d in held_blocks}
        all_edge_sets = list(self.block_manager.all_blocks().values())
        frontier = 0
        blocks_ahead: set[DirectedEdge] = set()
        for i, (eid, _d) in enumerate(remaining_path):
            if not any(eid in bs for bs in all_edge_sets):
                frontier = i + 1  # 无保护路段：天然可通行
                continue
            held_here = [d for d, bs in held_edge_sets.items() if eid in bs]
            if held_here:
                blocks_ahead.update(held_here)
                frontier = i + 1
            else:
                break  # 属于某 block 但未被持有：红灯边界
        return frontier, blocks_ahead

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
