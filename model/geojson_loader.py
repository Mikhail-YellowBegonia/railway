from __future__ import annotations

import json
from pathlib import Path

from model.rail_network import RailNetwork, Vec3


def load_geojson(path: str | Path, epsilon: float = 0.01) -> RailNetwork:
    with open(path, "r") as f:
        data = json.load(f)

    network = RailNetwork()
    lines = _extract_linestrings(data)

    for coords in lines:
        if _validate_linestring(coords):
            _add_linestring(network, coords, epsilon)

    return network


def load_signals(path: str | Path, network: RailNetwork, epsilon: float = 0.01):
    """从同一份 GeoJSON 的顶层 "signals" 字段还原信号放置位置。

    与 write_geojson 配对：按坐标反查 (from_node, to_node) 对应的
    DirectedEdge——node_id_at 用的容差语义与建网络时一致，只要坐标能
    精确匹配到已加载的节点就能还原。找不到对应节点/边的记录静默跳过
    （容错优先，不因为一条信号记录失效就中断整个加载）。

    Step 3 起不再还原颜色——颜色由占用状态实时推导（BlockManager），
    这里只还原"信号放置在哪"。用 place() 而不是内部直接写入，是因为
    place() 自带 One-Way PBS 背面校验：若某份手工编辑过的存档不小心
    存了背靠背的一对信号，加载时会跳过第二条而不是产生非法状态。

    返回一个新建的 SignalTable；旧存档没有 "signals" 字段时返回空表。
    """
    from model.signal import SignalTable

    with open(path, "r") as f:
        data = json.load(f)

    signals = SignalTable()
    records = data.get("signals", [])
    for rec in records:
        from_pos = Vec3(*_pad_coord(rec["from"]))
        to_pos = Vec3(*_pad_coord(rec["to"]))
        from_node_id = network.node_id_at(from_pos, epsilon)
        to_node_id = network.node_id_at(to_pos, epsilon)
        if from_node_id is None or to_node_id is None:
            continue
        from_node = network.nodes[from_node_id]
        edge_id = None
        for eid in from_node.incident_edge_ids:
            edge = network.edges[eid]
            if {edge.node_a_id, edge.node_b_id} == {from_node_id, to_node_id}:
                edge_id = eid
                break
        if edge_id is None:
            continue
        edge = network.edges[edge_id]
        direction = 1 if edge.node_a_id == from_node_id else -1
        signals.place((edge_id, direction))

    return signals


def load_trains(path: str | Path, network: RailNetwork, epsilon: float = 0.01):
    """从同一份 GeoJSON 的顶层 "trains" 字段还原全部列车（roadmap #2）。

    与 write_geojson(trains=...) 配对：序列化/反序列化逻辑在
    model/session.py，这里只负责读文件 + 转发。旧存档没有 "trains" 字段
    时返回空列表（优雅退化，不报错）。

    返回 list[TrainEntity]；单列车反查失败由 session.deserialize_trains
    静默跳过（容错优先，见 docs/session_persistence.md §6 决策 4）。
    """
    from model.session import deserialize_trains

    with open(path, "r") as f:
        data = json.load(f)
    records = data.get("trains", [])
    return deserialize_trains(records, network)


def _validate_linestring(coords: list[tuple[float, float, float]]) -> bool:
    n = len(coords)
    if n < 2:
        print(f"warning: LineString has {n} points, skipped (minimum 2)")
        return False
    if n > 3:
        print(f"warning: LineString has {n} points, skipped (maximum 3)")
        return False
    if n == 2:
        return True

    a = Vec3(*coords[0])
    b = Vec3(*coords[1])
    c = Vec3(*coords[2])

    if a.distance_to(c) < 1e-9:
        print("warning: arc endpoints coincide, skipped")
        return False

    ca = b - a
    cb = b - c
    if ca.cross(cb).length() < 1e-9:
        print("warning: arc points are collinear, skipped")
        return False

    la = ca.length()
    lc = cb.length()
    if abs(la - lc) > 0.001:
        print(f"warning: arc tangent lengths differ ({la:.3f} vs {lc:.3f}), skipped")
        return False

    return True


def _add_linestring(
    network: RailNetwork,
    coords: list[tuple[float, float, float]],
    epsilon: float,
) -> None:
    start_pos = Vec3(*coords[0])
    end_pos = Vec3(*coords[-1])

    if len(coords) == 3:
        interior = [Vec3(*coords[1])]
    else:
        interior = []

    exist_a = network.node_id_at(start_pos, epsilon)
    exist_b = network.node_id_at(end_pos, epsilon)

    if exist_a is not None:
        node_a = network.nodes[exist_a]
    else:
        node_a = network.add_node(start_pos)

    if exist_b is not None:
        node_b = network.nodes[exist_b]
    else:
        node_b = network.add_node(end_pos)

    if node_a.node_id != node_b.node_id:
        network.add_edge(node_a, node_b, interior)


def _extract_linestrings(data: dict) -> list[list[tuple[float, float, float]]]:
    results: list[list[tuple[float, float, float]]] = []

    def _walk(obj):
        if isinstance(obj, dict):
            gtype = obj.get("type")
            if gtype == "FeatureCollection":
                for feat in obj.get("features", []):
                    _walk(feat)
            elif gtype == "Feature":
                _walk(obj.get("geometry", {}))
            elif gtype == "LineString":
                coords = obj.get("coordinates", [])
                padded = [_pad_coord(c) for c in coords]
                results.append(padded)
        elif isinstance(obj, list):
            for item in obj:
                _walk(item)

    _walk(data)
    return results


def _pad_coord(coord: list[float]) -> tuple[float, float, float]:
    if len(coord) >= 3:
        return (coord[0], coord[1], coord[2])
    elif len(coord) == 2:
        return (coord[0], coord[1], 0.0)
    return (0.0, 0.0, 0.0)
