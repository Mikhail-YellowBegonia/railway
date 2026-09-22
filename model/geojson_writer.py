from __future__ import annotations

import json
from pathlib import Path

from model.rail_network import RailNetwork


def write_geojson(
    network: RailNetwork, path: str | Path, signals=None, trains=None, pois=None,
    stations=None,
) -> None:
    """写出网络（+ 可选信号数据 + 可选列车状态）。

    signals: model.signal.SignalTable | None。Step 2 阶段的最简持久化：
    node_id/edge_id 每次加载都重新分配，不能直接存 id，改存"信号所在
    节点坐标 + 该边另一端节点坐标"，加载时按坐标反查（复用 node_id_at
    的按坐标去重机制，同一套容差语义）。存成顶层 "signals" 字段，跟
    "features" 平级——不进 LineString 几何格式，旧存档没有这个字段时
    优雅退化成"无信号"。这是刻意从简的表示，后续如果需要更稳定的引用
    方式（比如给 Node 加持久 UUID）可以替换，不影响这里的调用方接口。

    trains: list[model.train_entity.TrainEntity] | None。roadmap #2 会话持久化：
    列车占用/route/goal 的 DirectedEdge 按坐标存（见 model/session.py），
    存成顶层 "trains" 数组，与 "features"/"signals" 平级。旧存档没有该字段
    优雅退化为"无列车"。

    Step 3 起不再存储颜色——颜色由占用状态实时推导（BlockManager），
    持久化的只是"信号放置在哪"这个事实。
    """
    features: list[dict] = []

    for edge in network.edges.values():
        node_a = network.nodes[edge.node_a_id]
        node_b = network.nodes[edge.node_b_id]

        if edge.is_arc and edge.geometry:
            b = edge.geometry[0]
            coords = [
                [node_a.position.x, node_a.position.y, node_a.position.z],
                [b.x, b.y, b.z],
                [node_b.position.x, node_b.position.y, node_b.position.z],
            ]
        else:
            coords = [
                [node_a.position.x, node_a.position.y, node_a.position.z],
                [node_b.position.x, node_b.position.y, node_b.position.z],
            ]

        features.append({
            "type": "Feature",
            "geometry": {
                "type": "LineString",
                "coordinates": coords,
            },
        })

    data = {"type": "FeatureCollection", "features": features}

    if signals is not None:
        signal_records = []
        for edge_id, direction in signals.all_signals():
            edge = network.edges.get(edge_id)
            if edge is None:
                continue
            from_node_id = edge.node_a_id if direction > 0 else edge.node_b_id
            to_node_id = edge.node_b_id if direction > 0 else edge.node_a_id
            from_pos = network.nodes[from_node_id].position
            to_pos = network.nodes[to_node_id].position
            signal_records.append({
                "from": [from_pos.x, from_pos.y, from_pos.z],
                "to": [to_pos.x, to_pos.y, to_pos.z],
            })
        data["signals"] = signal_records

    if trains is not None:
        from model.session import serialize_trains
        data["trains"] = serialize_trains(trains)

    if pois is not None:
        data["pois"] = _serialize_pois(network, pois)
    if stations is not None:
        data["stations"] = [
            {
                "station_id": station.station_id,
                "name": station.name,
                "platform_ids": list(station.platform_ids),
            }
            for station in stations.all()
            if not station.validate(pois)
        ]

    with open(path, "w") as f:
        json.dump(data, f, indent=2)


def _serialize_pois(network: RailNetwork, pois) -> list[dict]:
    records: list[dict] = []
    for poi in pois.all():
        # 轨道编辑后的 POI 拓扑改写策略尚未定义。当前宁可跳过整条失效 POI，
        # 也不能只写剩余成员而静默改变集合语义。
        if poi.validate(network):
            continue
        members = []
        if poi.member_kind.value == "node":
            for node_id in sorted(poi.member_ids):
                node = network.nodes.get(node_id)
                if node is None:
                    continue
                p = node.position
                members.append({"node": [p.x, p.y, p.z]})
        else:
            for edge_id in sorted(poi.member_ids):
                edge = network.edges.get(edge_id)
                if edge is None:
                    continue
                a = network.nodes[edge.node_a_id].position
                b = network.nodes[edge.node_b_id].position
                members.append({
                    "edge": {
                        "a": [a.x, a.y, a.z],
                        "b": [b.x, b.y, b.z],
                    }
                })
        if members:
            records.append({
                "poi_id": poi.poi_id,
                "name": poi.name,
                "kind": poi.kind.value,
                "member_kind": poi.member_kind.value,
                "members": members,
            })
    return records
