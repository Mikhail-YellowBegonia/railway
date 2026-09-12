"""simple_segment 全图分区（计划层 P0：分段器与校验）。

**为什么需要它**：路径级计划把"锚点 / 控制点"钉在图上，需要"段"这个概念来做
显示、校验与调试；按 Q22-1/Q22-2 的裁决，段**只是派生视图**——不进存档、不留 id、
不参与引用（引用一律落在物理要素：节点 / 边）。见 `docs/plan_layer_roadmap.md`
的 P0 与 §2。

**切分规则（本模块的核心，2026-09-10 定）**——节点是**切割点**当且仅当：

1. ``connection_count() != 2``（道岔 / 死端 / 孤立点），**或**
2. 度数 == 2 但**过境转向不许可**（``turn_allowed`` 为 False，例如手工搭出的
   90° 急折、发卡弯）。

⇒ 切完之后"段"才具备**段内可通行**这一性质：从一端走到另一端，中途每个节点的
过境转向都几何许可。**只按度数切是不够的**——``RailNetwork
.simple_segment_from_endpoint`` 就看度数而不看 ``turn_allowed``，会把走不通的链
当成一整段（2026-09-10 实测反例：``A(0,0) - M(100,0) - N(100,10)``，M 度数 = 2、
约 90° 急折，``turn_allowed(M, e1, e2)`` 双向皆 False，而该函数仍返回整段）。

**与既有函数的关系**：``simple_segment_from_endpoint``（折返可行性 / 信号
"安全停车位置"判定）**语义保持不变**，本模块既不改它、也不复用它——它的用途是
"从死端量长度"，本模块要做的是**覆盖全图、每边恰属一段**的分区。
其"不看 ``turn_allowed``、会把走不通的链当成一整段"的**已知盲区已登记在案、
不修**（2026-09-10 用户拍板；对照回归见 `tests/test_segments.py` ②）。

⭐ **切分总准则（用户 2026-09-10 口径）**：**分段宁可更激进——多切总是安全的，
少切才会把走不通的链当成一段**；遇到"切得对不对"的疑难就**多切一刀**，
具体问题再具体分析。本模块因此采取"能切就切"的规则（度数 ≠ 2 **或** 过境转向
不许可），而不是"尽量少切、尽量拼长"。

**不变量**（``validate_partition`` 逐条自查）：
覆盖（每条 edge 恰属一段）/ 顺序（``node_ids[i] --edge_ids[i]-- node_ids[i+1]``
与边端点一致）/ 可通行（内部节点度数 = 2 且过境转向许可）/ 端点（图中有切割点时，
段的端点必须是切割点）/ 长度（等于各边长度之和）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator, Sequence

from model.rail_network import RailNetwork


@dataclass(frozen=True)
class Segment:
    """一段 simple_segment（**派生视图**，不落盘、无稳定 id）。

    ``index`` 只在同一次 `build_partition` 结果内稳定，跨帧/跨编辑都会变——
    因此**不得**被计划或其他持久化数据引用（Q22-2）。
    """

    index: int
    node_ids: tuple[int, ...]
    edge_ids: tuple[int, ...]
    length: float

    @property
    def a_node_id(self) -> int:
        """起点端节点（沿 `edge_ids` 顺序走时的起点）。"""
        return self.node_ids[0]

    @property
    def b_node_id(self) -> int:
        """终点端节点。"""
        return self.node_ids[-1]

    @property
    def is_closed_loop(self) -> bool:
        """首尾同节点：无切割点的纯环，或回到起点道岔的"灯泡线"。"""
        return self.node_ids[0] == self.node_ids[-1]

    @property
    def interior_node_ids(self) -> tuple[int, ...]:
        """内部节点（不含两端）；按定义它们度数 = 2 且过境转向许可。"""
        return self.node_ids[1:-1]

    def edge_ids_from(self, node_id: int) -> tuple[int, ...]:
        """从某个端点出发的边序列（给 b 端则返回反转序列）。

        闭合环（首尾同节点）返回存储顺序（两个方向都合法）。
        """
        if node_id == self.node_ids[0]:
            return self.edge_ids
        if node_id == self.node_ids[-1]:
            return tuple(reversed(self.edge_ids))
        raise ValueError(f"节点 {node_id} 不是段 {self.index} 的端点")

    def __repr__(self) -> str:  # pragma: no cover - 调试用
        return (
            f"Segment(index={self.index}, {self.a_node_id}->{self.b_node_id}, "
            f"edges={list(self.edge_ids)}, length={self.length:.2f})"
        )


class SegmentPartition:
    """一次 `build_partition` 的结果（构建后只读，含 O(1) 反查表）。"""

    def __init__(self, segments: Sequence[Segment]) -> None:
        self.segments: tuple[Segment, ...] = tuple(segments)
        by_edge: dict[int, int] = {}
        by_node: dict[int, list[int]] = {}
        for seg in self.segments:
            for edge_id in seg.edge_ids:
                by_edge[edge_id] = seg.index
            for node_id in seg.node_ids:
                bucket = by_node.setdefault(node_id, [])
                if seg.index not in bucket:
                    bucket.append(seg.index)
        self._by_edge = by_edge
        self._by_node = {node_id: tuple(idx) for node_id, idx in by_node.items()}

    def __len__(self) -> int:
        return len(self.segments)

    def __iter__(self) -> Iterator[Segment]:
        return iter(self.segments)

    @property
    def total_edge_count(self) -> int:
        """被段覆盖的边数（正常应等于网络边数）。"""
        return len(self._by_edge)

    def segment_of_edge(self, edge_id: int) -> Segment | None:
        """边所属的段（不在任何段里则 None，正常不该发生）。"""
        idx = self._by_edge.get(edge_id)
        return None if idx is None else self.segments[idx]

    def segments_at_node(self, node_id: int) -> tuple[Segment, ...]:
        """经过该节点的所有段。

        切割点通常有"度数"个（每个方向一段）；内部节点恰好 1 个；
        孤立点没有段。
        """
        return tuple(self.segments[idx] for idx in self._by_node.get(node_id, ()))


def is_cut_node(network: RailNetwork, node_id: int) -> bool:
    """节点是否为分段的切割点（度数 ≠ 2，或二度但过境转向不许可）。"""
    node = network.nodes.get(node_id)
    if node is None:
        return False
    if node.connection_count() != 2:
        return True
    edge_ids = sorted(node.incident_edge_ids)
    # 二度节点上"从一边进、从另一边出"是唯一走法，只需检查这一种；
    # dot 对称 ⇒ 反向检查结果相同。
    return not network.turn_allowed(node_id, edge_ids[0], edge_ids[1])


def build_partition(network: RailNetwork) -> SegmentPartition:
    """把整张图切成若干 simple_segment（覆盖全部边，不重不漏）。

    实现：先定切割点，再从每个切割点沿每条未走过的边"走到下一个切割点"；
    图里**没有任何切割点**时（纯环，例如正则多边形环）走兜底分支，
    从任一未用边出发绕一圈闭合。
    """
    cut_nodes = {node_id for node_id in network.nodes if is_cut_node(network, node_id)}
    used: set[int] = set()
    segments: list[Segment] = []

    def walk(start_node_id: int, first_edge_id: int) -> None:
        node_ids = [start_node_id]
        edge_ids: list[int] = []
        current = start_node_id
        edge_id: int | None = first_edge_id
        while edge_id is not None:
            if edge_id in used:
                # 理论上不可达（每条边只会被走一次）；保险起见不吞掉整段。
                break
            used.add(edge_id)
            edge_ids.append(edge_id)
            edge = network.edges[edge_id]
            next_node_id = (
                edge.node_b_id if edge.node_a_id == current else edge.node_a_id
            )
            node_ids.append(next_node_id)
            if next_node_id == start_node_id or next_node_id in cut_nodes:
                break
            # next 不是切割点 ⇒ 度数必为 2 ⇒ 只能沿另一条边继续
            remaining = network.nodes[next_node_id].incident_edge_ids - {edge_id}
            edge_id = next(iter(remaining), None)
            current = next_node_id
        segments.append(
            Segment(
                index=len(segments),
                node_ids=tuple(node_ids),
                edge_ids=tuple(edge_ids),
                length=sum(network.edges[e].length for e in edge_ids),
            )
        )

    for node_id in sorted(cut_nodes):
        for edge_id in sorted(network.nodes[node_id].incident_edge_ids):
            if edge_id not in used:
                walk(node_id, edge_id)

    # 兜底：无切割点的纯环（及理论上不该出现的残留边）
    for edge_id in sorted(network.edges):
        if edge_id not in used:
            walk(network.edges[edge_id].node_a_id, edge_id)

    return SegmentPartition(segments)


def validate_partition(network: RailNetwork, partition: SegmentPartition) -> list[str]:
    """自查分区不变量，返回问题描述列表（空列表 = 全部通过）。"""
    problems: list[str] = []

    # 覆盖：每条边恰属一段
    seen: dict[int, int] = {}
    for seg in partition.segments:
        for edge_id in seg.edge_ids:
            if edge_id in seen:
                problems.append(
                    f"段 {seg.index} 与段 {seen[edge_id]} 重复覆盖 edge {edge_id}"
                )
            seen[edge_id] = seg.index
    for edge_id in network.edges:
        if edge_id not in seen:
            problems.append(f"edge {edge_id} 未被任何段覆盖")

    has_cut_node = any(is_cut_node(network, node_id) for node_id in network.nodes)

    for seg in partition.segments:
        # 顺序：node_ids[i] --edge_ids[i]-- node_ids[i+1]
        for i, edge_id in enumerate(seg.edge_ids):
            edge = network.edges.get(edge_id)
            if edge is None:
                problems.append(f"段 {seg.index} 引用了不存在的 edge {edge_id}")
                continue
            a_id, b_id = seg.node_ids[i], seg.node_ids[i + 1]
            if {edge.node_a_id, edge.node_b_id} != {a_id, b_id}:
                problems.append(
                    f"段 {seg.index} 第 {i} 条边 {edge_id} 与节点序列不符"
                    f"（{a_id}->{b_id}）"
                )
        # 段内可通行：内部节点度数 = 2 且过境转向许可
        for node_id in seg.interior_node_ids:
            node = network.nodes[node_id]
            if node.connection_count() != 2:
                problems.append(
                    f"段 {seg.index} 内部节点 {node_id} 度数 = "
                    f"{node.connection_count()}（应为 2）"
                )
                continue
            edge_ids = sorted(node.incident_edge_ids)
            if not network.turn_allowed(node_id, edge_ids[0], edge_ids[1]):
                problems.append(
                    f"段 {seg.index} 内部节点 {node_id} 过境转向不许可（段内走不通）"
                )
        # 端点必须是切割点（图里存在切割点时）
        if has_cut_node:
            for node_id in (seg.node_ids[0], seg.node_ids[-1]):
                if not is_cut_node(network, node_id):
                    problems.append(f"段 {seg.index} 端点 {node_id} 不是切割点")
        # 长度
        total = sum(
            network.edges[e].length for e in seg.edge_ids if e in network.edges
        )
        if abs(total - seg.length) > 1e-9:
            problems.append(
                f"段 {seg.index} length={seg.length} 与边长度之和 {total} 不符"
            )

    return problems
