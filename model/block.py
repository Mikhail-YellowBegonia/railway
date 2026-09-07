"""固定闭塞区间划分 + 占用推导信号颜色 + 进路预约（Step 3/4）。

Block / 预约设计（Step 4，见 docs/train_control.md 调研记录）：

- 预约表 `_reservations: dict[DirectedEdge, TrainEntity]`——每个 block（以
  其信号的 DirectedEdge 为 key）当前被哪辆车预约。预约在调度层收到新指令
  时对整条路径一次性写入（`reserve_path`），不是列车驶入后才写。

- **冲突判定在 edge_id 级别，不是 block/方向级别**：两个方向相反的 block
  可能物理上共享同一段单线（典型场景：单线双向运行，两端各放一个面向
  对方的单向信号，各自的 block 沿相反方向走到对面）。若只按"检查反方向
  同一个 block key"判断冲突，会漏掉这种场景（两个 block 的 DirectedEdge
  key 完全不同，但 edge_id 集合有重叠）。所以 `reserve_path` 展开成
  "本次预约涉及的所有 edge_id"，逐一检查是否已被其他列车通过*任意*
  block 预约占用——这天然实现了 JGRPP 方向令牌的效果：对向 block 只要
  和本次要预约的 block 共享哪怕一条 edge，就会被判定冲突，事前阻止而
  不是事后检测死锁。

- **释放条件用"占用 ∪ 未来路由"，不是单用当前占用**：如果只检查"block
  是否与当前 occupied 有交集"，会在列车刚收到指令、还没开始走的那些
  "预约了但车身还没到"的前方 block 上立刻误释放（正是这个原因，一开始
  的实现被推翻重写）。正确条件是 `block & (occupied ∪ route) == ∅` 才
  释放——route 是寻路结果里"车头尚未走到的部分"，只要 block 还在 route
  里，就还没走完，不该释放。折返不受影响：折返只反转方向，不改变
  occupied 的 edge_id 集合，`tick_reservations` 天然对折返前后一视同仁。

- **`tick_reservations` 只信任当前传入的 `trains` 列表**：任何持有预约但
  不在这个列表里的对象（典型场景：解挂/连挂会创建全新的 `TrainEntity`，
  旧对象从 `GameLoop.trains` 里移除后不再被任何地方更新），其预约会被
  立即释放，不需要在 decouple_at/couple_with 调用处额外插入释放逻辑——
  旧对象一旦不在 `trains` 里，下一帧自动清理，没有永久悬挂的预约。

- 寻路接入：`make_passable_fn(train)` 返回符合 `PassableFn` 签名的闭包，
  只检查"这条 edge 是否被别的列车预约"；One-Way PBS 的背面禁止在
  `SignalTable.allows_direction` 里单独判定，两者在 GameLoop 组合成责任链
  （`turn_allowed` 已经在 `neighbors()` 内部单独处理，不在这条链里）。
"""
from __future__ import annotations

from enum import Enum
from typing import TYPE_CHECKING

from model.pathfinding import DirectedEdge, head_node, _directed_from
from model.rail_network import Edge, RailNetwork
from model.signal import SignalTable

if TYPE_CHECKING:
    from model.train_entity import TrainEntity


class SignalState(Enum):
    GREEN = "green"
    RED = "red"      # block 被占用（占用 = 物理占用 or 预约）


class BlockManager:
    """固定闭塞区间管理：block 划分、占用/预约推导信号颜色、passable_fn。"""

    def __init__(self) -> None:
        self._blocks: dict[DirectedEdge, frozenset[int]] = {}
        self._reservations: dict[DirectedEdge, "TrainEntity"] = {}

    def rebuild(self, network: RailNetwork, signals: SignalTable) -> None:
        """重新计算所有信号的 block 边集合，清理失效预约（对应已删除的信号）。"""
        self._blocks = {
            directed: self._compute_block(network, signals, directed)
            for directed in signals.all_signals()
        }
        stale = [k for k in self._reservations if k not in self._blocks]
        for k in stale:
            del self._reservations[k]

    def _compute_block(
        self, network: RailNetwork, signals: SignalTable, start: DirectedEdge,
    ) -> frozenset[int]:
        """DFS 展开 block：走到某个节点时，若该节点上挂着任意方向的信号
        （不论朝向、不论是不是本次出发信号自己的背面），就停止从这个
        节点继续扩张——但到达该节点所经过的那条边仍然计入 block。

        block 边界是纯粹的基础设施属性（这个物理节点上有没有信号），与
        "能不能通行"完全无关；信号是钉在节点上的，不是钉在边上的。

        2026-09 bug 修复记录（共两轮）：

        第一轮（错误方案，已推翻）：只检查"前方来向正对着我的信号"
        （`signals.has_signal(nxt)`），遇到信号背面时返回 False，DFS
        毫无察觉地穿过去继续扩张——直线轨道上这条路径永远不会被触发
        （DFS 只会往前走，不会从背面绕回来），但只要地图上有一个折返
        环线，DFS 就能绕一圈从背面杀回主线，导致 block 吞掉整个网络
        （真实复现：4 个信号的 block 膨胀成全部 34 条边）。

        第二轮（同样错误，已推翻）：改成"进入某条边前，检查这条边任一
        方向有没有信号"（`has_signal(nxt) or has_signal(reverse(nxt))`）。
        这在单线双向场景里矫枉过正：A--edge_a--M--edge_b--B，A 端放一个
        面向东的信号、B 端放一个面向西的信号，M 无信号。这两个信号本该
        联手守护 edge_a+edge_b 这一整段区间（block_east == block_west ==
        {edge_a, edge_b}）。但该方案在 DFS 走到 M、准备进入 edge_b 前，
        发现 edge_b 的反方向（B 端那个信号）有信号，直接拒绝进入，导致
        edge_b 整条都进不了 block_east——错误地把边界提前到了 M，而
        B 端信号物理上明明站在节点 B，不该在节点 M 就被它挡住。

        现在的方案：判据从"边"移到"节点"——DFS 每走到一个新节点，检查
        这个节点上是否挂着任意方向的信号（`_node_has_any_signal`），若有
        则计入刚走过的这条边后停止继续扩张，若没有则正常继续。这同时
        满足了两个约束：
        1. 单线双向场景：走到 M（无信号）正常继续，走到 B（有信号，
           不论朝向）正确停下，edge_b 被计入 block_east，与 block_west
           完全对称——消除了"从不同信号出发算出不对称 block"的问题
           （用户在头脑风暴中提出的洞察，是这次修复的直接依据）。
        2. 环线绕后场景：DFS 绕一圈从背面回到某个已设信号的节点时，
           同样会被 `_node_has_any_signal` 挡住，不会继续扩张吞并整个
           网络。
        3. 面向未来的 PBS（背面允许通行，不是本项目当前范围）：block
           边界完全不触碰"能不能通行"这个语义，即使以后信号种类扩展到
           允许背面通行的 PBS，这段代码也不需要改——通行规则的变化只
           影响 passable_fn/寻路层。
        """
        block_edges: set[int] = set()
        visited: set[DirectedEdge] = set()
        stack = [start]
        while stack:
            directed = stack.pop()
            if directed in visited:
                continue
            visited.add(directed)
            block_edges.add(directed[0])
            node_id = head_node(network, directed)
            if self._node_has_any_signal(network, signals, node_id):
                continue  # 到达信号所在节点，边已计入，但不再向外扩张
            for to_edge_id in network.adjacent_edges_at(node_id, directed[0]):
                if not network.turn_allowed(node_id, directed[0], to_edge_id):
                    continue
                nxt = _directed_from(network, to_edge_id, node_id)
                stack.append(nxt)
        return frozenset(block_edges)

    @staticmethod
    def _node_has_any_signal(
        network: RailNetwork, signals: SignalTable, node_id: int,
    ) -> bool:
        """node_id 上是否物理挂着信号（任意朝向）。

        信号 (eid, direction) 的物理位置是 tail_node(eid, direction)——
        dir=+1 时 tail 是 node_a_id，dir=-1 时 tail 是 node_b_id。对每条
        相邻边只检查"tail 正好是 node_id"的那个方向，不是任意方向：
        否则第一跳的到达节点必然包含"刚走过来的那条边"，而那条边上
        恰好挂着我们自己出发的信号（物理位置在边的另一端，不在这个
        节点上），会被误判成"当前节点有信号"导致 DFS 走一步就自我
        卡死——这是本次修复过程中先出现、又推翻重写的一版 bug。
        """
        node = network.nodes.get(node_id)
        if node is None:
            return False
        for eid in node.incident_edge_ids:
            edge = network.edges[eid]
            direction = 1 if edge.node_a_id == node_id else -1
            if signals.has_signal((eid, direction)):
                return True
        return False

    def block_edges(self, directed: DirectedEdge) -> frozenset[int]:
        return self._blocks.get(directed, frozenset())

    def held_blocks(self, train: "TrainEntity") -> set[DirectedEdge]:
        """train 当前持有的所有 block key（预约表的反查）。"""
        return {d for d, t in self._reservations.items() if t is train}

    def all_blocks(self) -> dict[DirectedEdge, frozenset[int]]:
        """返回当前所有信号的 block 划分（key -> 边集合），供调度层判断某条
        边是否属于任意 block（= 受保护），从而区分无保护路段。"""
        return dict(self._blocks)

    def truncate_to_next_signal(
        self, network: RailNetwork, signals: SignalTable, route: list[DirectedEdge],
    ) -> list[DirectedEdge]:
        """把远场寻路给出的完整路径截断到"前方第一个闭塞区间为止"
        （Step 5：近场调度只预约前方一个区间，对应 OpenTTD 原版逻辑，
        不做 JGRPP 的 Long Reserve 多区间预留）。

        近场段 = 走到第一个信号为止的无保护路段（原样经过，不需要
        预约——不属于任何 block）+ 该信号实际保护的完整 block（这才是
        真正需要预约的部分，从信号出发一直到下一个信号/死端）。

        **实现踩过的坑（已修正）**：最初实现在"到达第一个挂信号的节点"
        就直接截断，把信号本身**保护的那段路**（block_edges 算出来的、
        从信号出发向前延伸的边集合）整个漏掉了——这跟 `_compute_block`
        的方向定义正好错位一格：block 是"从信号出发往前"，不是"走到
        信号跟前"。结果是 `reserve_path` 拿着"信号前面那一截"去检查
        冲突，永远查不到交集，预约形同虚设（用真实 GameLoop 端到端
        验证时发现：预约表在寻路成功后仍然是空的）。

        正确算法：沿 route 走，标记"是否已经进入一个 block"（进入条件是
        当前 directed 本身就是某个信号——即 `signals.has_signal(directed)`
        为真，说明这条边正是某个 block 的第一条边）；一旦进入，继续走
        直到到达下一个挂信号的节点（`_node_has_any_signal`，标志这个
        block 已经走完，遇到了下一个 block 的边界）才截断。若一路都没
        进入任何 block（route 中没有信号），或进入后一路开到底都没遇到
        第二个边界（block 延伸到死端），返回整条 route 不截断。
        """
        entered_block = False
        for i, directed in enumerate(route):
            if not entered_block and signals.has_signal(directed):
                entered_block = True
            if entered_block:
                arrival_node = head_node(network, directed)
                if self._node_has_any_signal(network, signals, arrival_node):
                    return route[: i + 1]
        return list(route)

    # ------------------------------------------------------------------
    # 预约接口
    # ------------------------------------------------------------------

    def _edge_owners(self) -> dict[int, set]:
        """edge_id -> 当前持有该 edge 所在任意 block 预约的列车集合。"""
        owners: dict[int, set] = {}
        for directed, train in self._reservations.items():
            for eid in self._blocks.get(directed, frozenset()):
                owners.setdefault(eid, set()).add(train)
        return owners

    def reserve_path(self, train: "TrainEntity", path_edge_ids: list[int]) -> bool:
        """为 train 预约 path_edge_ids 途经的所有信号 block。

        全有或全无：本次路径涉及的所有 block 的全部 edge，只要有一条被
        别的列车持有（不管持有方是通过哪个 block/方向拿到的），整个预约
        失败并返回 False，不改变任何现有状态——这就是单线对向冲突的
        判定入口，见本文件顶部说明。

        路径不涉及任何有信号的 block 时（没放信号的路段）视为无需预约，
        直接返回 True。
        """
        path_edge_set = set(path_edge_ids)
        target_blocks = [
            directed for directed, block in self._blocks.items()
            if block & path_edge_set
        ]
        if not target_blocks:
            return True

        owners = self._edge_owners()
        touched_edges: set[int] = set()
        for directed in target_blocks:
            touched_edges |= self._blocks[directed]

        for eid in touched_edges:
            for owner in owners.get(eid, ()):
                if owner is not train:
                    return False

        for directed in target_blocks:
            self._reservations[directed] = train
        return True

    def tick_reservations(self, trains) -> None:
        """按当前 trains 列表释放已经不需要的预约。

        释放条件：holder 不在 trains 里（已被移除/替换，见本文件顶部
        关于 decouple/couple 的说明），或者 block 与 holder 的
        (occupied ∪ route) 已经没有交集（车身已完全驶离，且这段路也不在
        待走的路由里了）。
        """
        live = set(trains)
        for directed in list(self._reservations):
            holder = self._reservations[directed]
            if holder not in live:
                del self._reservations[directed]
                continue
            block = self._blocks.get(directed, frozenset())
            occ = holder.state.occupancy
            needed = {e for e, _ in occ.occupied} | {e for e, _ in occ.route}
            if not (block & needed):
                del self._reservations[directed]

    def is_reserved_by_other(self, directed: DirectedEdge, train: "TrainEntity") -> bool:
        """directed 对应的 block 是否被除 train 之外的列车持有预约。

        供测试/调试查询用；寻路层走 make_passable_fn（按 edge_id 级别
        判定，覆盖对向 block 共享 edge 的场景），这里是 block key 级别
        的直接查询，语义更简单但不做跨 block 的 edge 级别冲突检查。
        """
        holder = self._reservations.get(directed)
        return holder is not None and holder is not train

    # ------------------------------------------------------------------
    # passable_fn 接口（接入寻路）
    # ------------------------------------------------------------------

    def make_passable_fn(self, requesting_train: "TrainEntity"):
        """返回符合 pathfinding.PassableFn 签名 (Edge, direction) -> bool
        的闭包：该 edge 是否被别的列车预约。owners 只在闭包创建时算一次，
        供本次寻路调用全程复用，不必每条候选边都重新扫描 `_reservations`。
        """
        owners = self._edge_owners()

        def passable(edge: Edge, direction: int) -> bool:
            for owner in owners.get(edge.edge_id, ()):
                if owner is not requesting_train:
                    return False
            return True

        return passable

    # ------------------------------------------------------------------
    # 颜色推导
    # ------------------------------------------------------------------

    def compute_colors(self, trains) -> dict[DirectedEdge, SignalState]:
        """按物理占用 + 预约推导信号颜色（占用或预约 = RED）。"""
        occupied_edge_ids: set[int] = set()
        for train in trains:
            for edge_id, _direction in train.state.occupancy.occupied:
                occupied_edge_ids.add(edge_id)

        colors: dict[DirectedEdge, SignalState] = {}
        for directed, block in self._blocks.items():
            occupied = bool(block & occupied_edge_ids)
            reserved = directed in self._reservations
            colors[directed] = (
                SignalState.RED if (occupied or reserved) else SignalState.GREEN
            )
        return colors
