"""信号机数据结构（Step 3：One-Way PBS 放置 + 只读位置；颜色不再存储于此）。

Step 3 的关键改动（相对 Step 2）：信号灯颜色改为由闭塞占用状态自动推导
（见 model/block.py::BlockManager），不再是玩家可以手动切换的独立状态。
SignalTable 因此收窄为纯粹的"放置事实"——只记录信号存在于哪个槛位
（DirectedEdge），不存储颜色。玩家在 H 模式下能做的只剩"放置"，
Step 2 的手动 toggle 交互已废弃（颜色现在完全由占用状态决定，手动
覆盖会立刻被下一帧的自动推导覆盖，保留切换入口没有意义）。

设计要点（详见 CLAUDE.md「Graph topology invariant」+ docs/train_control.md
的头脑风暴与 OpenTTD 调研记录）：
- 信号是有向边的属性，不是节点的属性——一个节点最多有
  `len(incident_edge_ids)` 个独立信号槛位，每个槛位保护"进入某条边的
  某个方向"这件事本身，与"从哪条边过来"无关（同一槛位可能被多条不同
  的来向共享，符合现实道岔口信号的行为）。
- 信号机本身放置在真节点上（玩家点击 Edge 中途会先 split_edge_at，
  由 controller 层负责，见 editor.py），signals 集合的元素仍是
  DirectedEdge，不需要额外记录"挂在哪个 Node"——DirectedEdge 已经
  隐含了这个信息（tail_node 就是信号所在的物理位置）。

**One-Way PBS 语义**（借鉴 OpenTTD Path Signal 的单向信号）：一个信号
槛位只有"正面"存在与否，反方向永久禁止通行（不是"无信号=自由通行"）
——这样物理上只需要一个朝向的图标，省掉背面那盏灯。因此 `_signals`
只存"正面"槛位；反方向的禁止在需要判定通行性的地方（未来 Step 4 接入
寻路时）通过反查动态推导，不是存储状态，也不允许再对同一物理位置的
反方向放置第二个信号（两个相对的单向门在同一点矛盾）。
"""
from __future__ import annotations

from model.pathfinding import DirectedEdge


class SignalTable:
    """信号机放置记录：DirectedEdge 的集合（单向 PBS 语义，不存颜色）。

    不进图拓扑（不是 Node/Edge 的字段），只是 RailNetwork 之外的一份
    附加数据。放置/移除信号是"基础设施变化"（需要重新划分闭塞区间，
    见 BlockManager.rebuild），但集合本身仍然只是旁挂数据，不写回
    Node/Edge，天然满足"图拓扑只因基础设施变化而改变"这条不变量。
    """

    def __init__(self) -> None:
        self._signals: set[DirectedEdge] = set()

    def has_signal(self, directed: DirectedEdge) -> bool:
        """该槛位是否有一个"正面"信号（不含反向推导出的永久禁止）。"""
        return directed in self._signals

    def is_blocked_backside(self, directed: DirectedEdge) -> bool:
        """directed 是否是另一个单向信号的背面（反方向已有正面信号）。

        背面永久禁止通行，且不能在背面再放一个信号（两个相对的单向门
        在同一物理位置矛盾）。
        """
        edge_id, direction = directed
        return (edge_id, -direction) in self._signals

    def place(self, directed: DirectedEdge) -> bool:
        """在指定槛位放置一个单向信号。

        返回 False（拒绝，不修改任何状态）当 directed 是另一个信号的
        背面时——不能在已有单向信号的反方向再放一个相对的信号。
        已存在时视为幂等，返回 True。
        """
        if self.is_blocked_backside(directed):
            return False
        self._signals.add(directed)
        return True

    def remove(self, directed: DirectedEdge) -> None:
        self._signals.discard(directed)

    def all_signals(self) -> set[DirectedEdge]:
        return set(self._signals)

    def passable_topology_only(self, edge, direction: int) -> bool:
        """远场寻路用的 passable_fn（Step 5：远场/近场寻路拆分）：只看
        拓扑级的单向硬性限制（One-Way PBS 背面永久禁止），不看闭塞
        占用/预约状态（那是近场调度的事）。

        设计依据（2026-09 讨论确定）：远场寻路负责回答"这个目标理论上
        走不走得通"，判定可达性、展示预期路径——不该因为"眼前有别的车"
        就判定不可达（红灯不代表这里真的不能走，只代表暂时被占用）。
        但反方向的硬性禁止是拓扑层面的单向限制，不是信号灯颜色，无论
        远场近场都必须遵守——否则远场规划出的路径可能在几何上根本不
        存在（背面永久不可通行，不是"暂时不能走"）。

        符合 pathfinding.PassableFn 签名 (Edge, direction) -> bool，
        可直接传给 find_path*。
        """
        return not self.is_blocked_backside((edge.edge_id, direction))

    def prune_missing(self, network) -> list[DirectedEdge]:
        """移除引用已不存在的 edge_id 的信号，返回被移除的槛位列表。

        DELETE 模式删边、或把 connection_count==2 节点两侧的边合并成一条
        新边时（`controller/editor.py::_delete`/`_try_merge_at_node`），
        原来挂在被删边上的信号会变成悬空引用——`Editor` 本来就不知道
        `SignalTable` 的存在（两者故意零耦合，SIGNAL 模式单独在 GameLoop
        里接线），不该为了这一个场景把两者绑在一起。改为在这里做自我
        清理：轨道都没了，挂在它上面的信号自然跟着消失，每帧调一次即可
        保持同步，不需要在网络变更的每个调用点手动插清理逻辑。
        """
        removed = [d for d in self._signals if d[0] not in network.edges]
        for d in removed:
            self._signals.discard(d)
        return removed
