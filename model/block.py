"""信号保护包络 + 占用推导信号颜色 + 路径预约（Step 3/4/6）。

保护包络 / 预约设计（Step 4 起，Step 6 改为 PBS / edge 交集语义，见
docs/train_control.md 调研记录与「Step 6」）：

- **保护包络不是区间分割**：每个信号沿所有允许分支 DFS 到下一个信号，
  得到该信号可能保护的 edge 包络。不同入口的包络可以重叠，一个包络也可以
  在道岔处分叉；它不是路网 partition，不能用包络 key 数量表示列车预约了
  几段路径。列车授权必须沿自己的固定 route 截取 route span。

- 预约表 `_reservations: dict[TrainEntity, set[int]]`——每辆车预约的**具体
  edge_id 集合**（PBS 语义），不是"block key → train"的整块互斥。预约在
  调度层每帧对下一个 route span 的具体边写入（`reserve_path`）。

- **冲突判定 = edge 交集，不是整块互斥**（2026-09 用户澄清）：老版简单
  闭塞是"block 内任何位置有车就整块红灯"（完全互斥，更安全但会在会让线
  A/B 股道处死锁）；PBS 是"本车要走的 edge 集合与其它占用/预约的 edge
  集合交集为空即可放行"（edge 交集，更现代）。`reserve_path` 只检查
  `path_edge_ids` 本身是否被别的车预约，不再展开到整个 block 的全部边。
  单线双向场景（对向两个信号各自 block 完全重叠）依然正确：重叠的边会在
  edge 级别被同一套冲突判定拦下（方向令牌效果）。

- **颜色 = 是否存在一条无占用/无预约的通路**：`compute_colors` 对每个信号
  做 `_has_free_path`（从信号出发沿 turn_allowed 走，只要有一条 edge 全部
  既不被占用也不被预约的路径走到下一个信号/死端就 GREEN）。这样道岔无
  信号时被并入同一 block 的 A/B 股道不会互锁：B 股道被占不影响 A 股道作为
  通路让信号保持绿灯。

- **释放条件用"占用 ∪ 未来路由"，不是单用当前占用**：如果只检查"预约 edge
  是否与当前 occupied 有交集"，会在列车刚收到指令、还没开始走的那些
  "预约了但车身还没到"的前方 edge 上立刻误释放（正是这个原因，一开始
  的实现被推翻重写）。正确条件是按 edge 逐个判断 `e ∈ (occupied ∪ route)`
  才保留——route 是寻路结果里"车头尚未走到的部分"。折返不受影响：折返只
  反转方向，不改变 occupied 的 edge_id 集合，`tick_reservations` 天然对
  折返前后一视同仁。

- **`tick_reservations` 只信任当前传入的 `trains` 列表**：任何持有预约但
  不在这个列表里的对象（典型场景：解挂/连挂会创建全新的 `TrainEntity`，
  旧对象从 `GameLoop.trains` 里移除后不再被任何地方更新），其预约会被
  立即释放，不需要在 decouple_at/couple_with 调用处额外插入释放逻辑——
  旧对象一旦不在 `trains` 里，下一帧自动清理，没有永久悬挂的预约。

- 寻路接入：`make_passable_fn(train)` 返回符合 `PassableFn` 签名的闭包，
  只检查"这条 edge 是否被别的列车预约"；One-Way PBS 的背面禁止在
  `SignalTable.passable_topology_only` 里单独判定，两者在 GameLoop 组合成
  责任链（`turn_allowed` 已经在 `neighbors()` 内部单独处理，不在这条链里）。
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
    RED = "red"      # 信号前方已无通路（所有分支都被占用/预约堵死）


class BlockManager:
    """信号保护包络、路径 edge 预约及信号颜色管理。"""

    def __init__(self) -> None:
        self._protection_envelopes: dict[DirectedEdge, frozenset[int]] = {}
        # 预约表（PBS / edge 交集语义）：train -> 该车预约的具体 edge_id 集合。
        # 与 Step 4 的"block key -> train"不同——预约不再整块互斥，而是按
        # 列车实际要走的边粒度登记，两列车的预约 edge 集合交集为空即可共享
        # 同一个（粗粒度）block。
        self._reservations: dict["TrainEntity", set[int]] = {}
        # 已被车身实际进入过的预约 edge。未进入时，route 中的未来引用负责保留
        # 预约；进入后，一旦车身清出就释放，即使同一 edge_id 在更远的未来路线
        # 中再次出现。否则环线/多锚点路线会把“过去这次”预约错误延寿到下一次。
        self._entered_reservations: dict["TrainEntity", set[int]] = {}

    def rebuild(self, network: RailNetwork, signals: SignalTable) -> None:
        """重算所有信号保护包络，清理引用已删除 edge 的预约。"""
        self._protection_envelopes = {
            directed: self._compute_protection_envelope(network, signals, directed)
            for directed in signals.all_signals()
        }
        # 轨道删除/合并后，预约里可能残留已不存在的 edge_id（跟信号悬空引用
        # 同一类问题），这里顺带剪掉。
        live_edges = set(network.edges)
        for train in list(self._reservations):
            self._reservations[train] &= live_edges
            entered = self._entered_reservations.get(train)
            if entered is not None:
                entered &= self._reservations[train]
            if not self._reservations[train]:
                del self._reservations[train]
                self._entered_reservations.pop(train, None)

    def _compute_protection_envelope(
        self, network: RailNetwork, signals: SignalTable, start: DirectedEdge,
    ) -> frozenset[int]:
        """DFS 展开信号保护包络：到任意信号节点时停止继续扩张。

        结果是从 start 出发可能采用的所有路径的 edge 并集，允许和其他信号
        的包络重叠，也允许在道岔处分叉；它不是互斥闭塞区间。

        走到某个节点时，若该节点上挂着任意方向的信号
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
        envelope_edges: set[int] = set()
        visited: set[DirectedEdge] = set()
        stack = [start]
        while stack:
            directed = stack.pop()
            if directed in visited:
                continue
            visited.add(directed)
            envelope_edges.add(directed[0])
            node_id = head_node(network, directed)
            if self._node_has_any_signal(network, signals, node_id):
                continue  # 到达信号所在节点，边已计入，但不再向外扩张
            for to_edge_id in network.adjacent_edges_at(node_id, directed[0]):
                if not network.turn_allowed(node_id, directed[0], to_edge_id):
                    continue
                nxt = _directed_from(network, to_edge_id, node_id)
                stack.append(nxt)
        return frozenset(envelope_edges)

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

    def protection_envelope_edges(self, directed: DirectedEdge) -> frozenset[int]:
        """返回一个信号的全分支保护包络。"""
        return self._protection_envelopes.get(directed, frozenset())

    def held_edges(self, train: "TrainEntity") -> set[int]:
        """train 当前预约的具体 edge_id 集合（PBS / edge 交集语义）。"""
        return set(self._reservations.get(train, ()))

    def all_protection_envelopes(self) -> dict[DirectedEdge, frozenset[int]]:
        """返回各信号的保护包络（信号槛位 -> 可能到达的 edge 集合）。

        这些集合允许重叠和分叉，不是路网 partition，也不是 PBS 预约单位。
        预约单位是列车固定路径上截取出的 route span；本查询只用于判断 edge
        是否位于某个信号保护范围内，以及信号显示/调试。
        """
        return dict(self._protection_envelopes)

    def truncate_to_next_route_span(
        self, network: RailNetwork, signals: SignalTable, route: list[DirectedEdge],
    ) -> list[DirectedEdge]:
        """把固定路径截到“无保护前缀 + 下一个受保护路径片段”为止。

        这是路径局部的 route span，不是 `_compute_protection_envelope` 的全局
        DFS 包络：
        遇到道岔时只包含本车 route 选择的分支；双向信号产生多少重叠包络，
        也不会改变本车路径片段的身份。

        原称“前方第一个闭塞区间”，容易让调用方把保护包络误当作互斥区间。
        （Step 5：近场调度只预约前方一个区间，对应 OpenTTD 原版逻辑，
        不做 JGRPP 的 Long Reserve 多区间预留）。

        近场段 = 走到第一个信号为止的无保护路段 + 从该信号出发、沿本车
        固定路径走到下一个信号节点/死端的 edge。前者虽无信号保护，也一并
        登记在本车 edge 预约中；后者才是受保护的 route span。

        **实现踩过的坑（已修正）**：最初实现在"到达第一个挂信号的节点"
        就直接截断，把信号本身**保护的那段路**整个漏掉了——这跟保护包络
        的方向定义正好错位一格：保护范围是"从信号出发往前"，不是"走到
        信号跟前"。结果是 `reserve_path` 拿着"信号前面那一截"去检查
        冲突，永远查不到交集，预约形同虚设（用真实 GameLoop 端到端
        验证时发现：预约表在寻路成功后仍然是空的）。

        正确算法：沿 route 走，标记"是否已经进入受保护片段"（进入条件是
        当前 directed 本身就是某个信号——即 `signals.has_signal(directed)`
        为真，说明这条边正是某个 block 的第一条边）；一旦进入，继续走
        直到到达下一个挂信号的节点（`_node_has_any_signal`，标志这个
        block 已经走完，遇到了下一个 block 的边界）才截断。若一路都没
        进入任何 block（route 中没有信号），或进入后一路开到底都没遇到
        第二个边界（片段延伸到死端），返回整条 route 不截断。
        """
        entered_span = False
        for i, directed in enumerate(route):
            if not entered_span and signals.has_signal(directed):
                entered_span = True
            if entered_span:
                arrival_node = head_node(network, directed)
                if self._node_has_any_signal(network, signals, arrival_node):
                    return route[: i + 1]
        return list(route)

    # ------------------------------------------------------------------
    # 预约接口
    # ------------------------------------------------------------------

    def _edge_owners(self) -> dict[int, set]:
        """edge_id -> 预约了该 edge 的列车集合（由预约表反查）。"""
        owners: dict[int, set] = {}
        for train, edges in self._reservations.items():
            for eid in edges:
                owners.setdefault(eid, set()).add(train)
        return owners

    def reserve_path(self, train: "TrainEntity", path_edge_ids: list[int]) -> bool:
        """为 train 预约 path_edge_ids 里的具体 edge（PBS / edge 交集语义）。

        **与 Step 4 的关键区别**：冲突判定不再展开到"整块 block 的全部边"，
        而是只看 train 实际要走的 `path_edge_ids` 本身。两条列车只要预约的
        edge 集合交集为空，就能共享同一个（粗粒度）block——这正是 PBS 的
        行为，也是单线车站 A/B 股道（道岔无信号、被并入同一 block）不互锁
        死锁的前提。反之，老版简单闭塞是"整块互斥"：block 内任何位置有车
        就整块红灯，A/B 股道互相卡死。

        全有或全无：`path_edge_ids` 里只要有一条 edge 被别的列车预约，整个
        预约失败并返回 False，不改变任何现有状态。
        """
        path_edge_set = set(path_edge_ids)
        owners = self._edge_owners()
        for eid in path_edge_set:
            for owner in owners.get(eid, ()):
                if owner is not train:
                    return False
        self._reservations.setdefault(train, set()).update(path_edge_set)
        occupied = {eid for eid, _direction in train.state.occupancy.occupied}
        entered_now = path_edge_set & occupied
        if entered_now:
            self._entered_reservations.setdefault(train, set()).update(entered_now)
        return True

    def tick_reservations(self, trains) -> None:
        """按当前 trains 列表释放已经不需要的预约 edge。

        释放条件：holder 不在 trains 里；或预约 edge 已被车身进入且现在已经
        清出；或尚未进入但也不再位于未来 route。不能只用
        ``edge ∈ occupied ∪ route``：同一 edge 在路线远处再次出现时，会把
        已经完成的那次预约错误保留到未来 occurrence。
        """
        live = set(trains)
        for train in list(self._reservations):
            if train not in live:
                del self._reservations[train]
                self._entered_reservations.pop(train, None)
                continue
            occ = train.state.occupancy
            occupied = {e for e, _ in occ.occupied}
            future = {e for e, _ in occ.route}
            entered = self._entered_reservations.setdefault(train, set())
            entered.update(self._reservations[train] & occupied)
            self._reservations[train] = {
                edge_id for edge_id in self._reservations[train]
                if edge_id in occupied or (edge_id not in entered and edge_id in future)
            }
            entered &= self._reservations[train]
            if not self._reservations[train]:
                del self._reservations[train]
                self._entered_reservations.pop(train, None)

    def is_reserved_by_other(self, directed: DirectedEdge, train: "TrainEntity") -> bool:
        """directed 的保护包络内是否有 edge 被其他列车预约。

        供测试/调试查询用；寻路层走 make_passable_fn（按 edge_id 级别判定），
        这个包络查询不代表本车固定路径冲突；调度必须使用 reserve_path。
        """
        envelope = self._protection_envelopes.get(directed, frozenset())
        owners = self._edge_owners()
        for eid in envelope:
            for owner in owners.get(eid, ()):
                if owner is not train:
                    return True
        return False

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

    def compute_colors(
        self, trains, network: RailNetwork, signals: SignalTable,
    ) -> dict[DirectedEdge, SignalState]:
        """按"是否还有一条无占用/无预约的通路"推导信号颜色（PBS / edge 交集）。

        与老版简单闭塞"block 内有车就整块红灯"不同：这里对每个信号做一次
        `_has_free_path`——只要从该信号出发，存在一条 edge 全部既不被物理
        占用、也不被预约的路径走到下一个信号/死端，就判 GREEN；所有分支都
        被堵死才判 RED。这样道岔无信号时被并入同一 block 的 A/B 股道不会
        互相锁死：A 股道被占不影响 B 股道作为一条通路让信号保持绿灯。
        """
        occupied_edge_ids: set[int] = set()
        for train in trains:
            for edge_id, _direction in train.state.occupancy.occupied:
                occupied_edge_ids.add(edge_id)
        reserved_edge_ids: set[int] = set()
        for edges in self._reservations.values():
            reserved_edge_ids |= edges
        blocked = occupied_edge_ids | reserved_edge_ids

        colors: dict[DirectedEdge, SignalState] = {}
        for directed in self._protection_envelopes:
            colors[directed] = (
                SignalState.GREEN
                if self._has_free_path(network, signals, directed, blocked)
                else SignalState.RED
            )
        return colors

    def _has_free_path(
        self,
        network: RailNetwork,
        signals: SignalTable,
        directed: DirectedEdge,
        blocked_edges: set[int],
    ) -> bool:
        """从信号 directed 出发，是否存在一条全程不被 blocked 的路径走到
        下一个信号（或死端）。存在则 True（该信号可放行，GREEN）。"""
        stack: list[DirectedEdge] = [directed]
        visited: set[DirectedEdge] = set()
        while stack:
            cur = stack.pop()
            if cur in visited:
                continue
            visited.add(cur)
            if cur[0] in blocked_edges:
                continue  # 这条边被占/被预约，此分支不通
            node = head_node(network, cur)
            if self._node_has_any_signal(network, signals, node):
                return True  # 走到下一个信号节点 = 完整通过本 block
            nxt = [
                to_edge_id for to_edge_id in network.adjacent_edges_at(node, cur[0])
                if network.turn_allowed(node, cur[0], to_edge_id)
            ]
            if not nxt:
                return True  # 走到死端 = block 终点，同样算一条通路
            for to_edge_id in nxt:
                stack.append(_directed_from(network, to_edge_id, node))
        return False
