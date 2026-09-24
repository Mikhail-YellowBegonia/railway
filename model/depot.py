"""Depot POI path resolution and generation-capacity helpers."""
from __future__ import annotations

from model.pathfinding import DirectedEdge, Path
from model.poi import POI, POIKind
from model.rail_network import RailNetwork


def ordered_depot_path(
    depot: POI,
    network: RailNetwork,
    direction: int = 1,
) -> Path:
    """Resolve a Depot's unordered edge members into a directed finite path.

    ``direction=1`` follows the canonical endpoint-to-endpoint traversal and
    ``direction=-1`` reverses it. The POI validation already rejects closed
    sets; this helper additionally rejects branches and disconnected members so
    a spawn operation can never receive an ambiguous path.
    """
    if depot.kind != POIKind.DEPOT or depot.member_ids == ():
        raise ValueError("只能从非空 Depot POI 生成列车")
    if depot.member_kind.value != "edge":
        raise ValueError("Depot 必须由 edge 组成")
    problems = depot.validate(network)
    if problems:
        raise ValueError("；".join(problems))

    member_ids = set(depot.member_ids)
    endpoints: dict[int, list[int]] = {}
    for edge_id in member_ids:
        edge = network.edges[edge_id]
        endpoints.setdefault(edge.node_a_id, []).append(edge_id)
        endpoints.setdefault(edge.node_b_id, []).append(edge_id)
    terminal_nodes = [node_id for node_id, edges in endpoints.items() if len(edges) == 1]
    if len(terminal_nodes) != 2:
        raise ValueError("Depot 必须是一条有两个端点的非闭合连续轨道")

    current = terminal_nodes[0]
    remaining = set(member_ids)
    directed: list[DirectedEdge] = []
    while remaining:
        candidates = [eid for eid in endpoints[current] if eid in remaining]
        if len(candidates) != 1:
            raise ValueError("Depot edge 集合包含分叉或无法排序")
        edge_id = candidates[0]
        edge = network.edges[edge_id]
        edge_direction = 1 if edge.node_a_id == current else -1
        directed.append((edge_id, edge_direction))
        current = edge.node_b_id if edge_direction > 0 else edge.node_a_id
        remaining.remove(edge_id)

    if current != terminal_nodes[1]:
        raise ValueError("Depot edge 集合不是一条完整连续路径")
    if direction not in (1, -1):
        raise ValueError("Depot 生成方向必须是 +1 或 -1")
    if direction < 0:
        directed = [(edge_id, -edge_direction) for edge_id, edge_direction in reversed(directed)]
    return Path(
        edges=directed,
        total_cost=sum(network.edges[edge_id].length for edge_id, _ in directed),
    )


def depot_length(depot: POI, network: RailNetwork) -> float:
    """Return the finite usable track length of a Depot."""
    return ordered_depot_path(depot, network).total_cost
