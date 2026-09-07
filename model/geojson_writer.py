from __future__ import annotations

import json
from pathlib import Path

from model.rail_network import RailNetwork


def write_geojson(network: RailNetwork, path: str | Path, signals=None) -> None:
    """写出网络（+ 可选信号数据）。

    signals: model.signal.SignalTable | None。Step 2 阶段的最简持久化：
    node_id/edge_id 每次加载都重新分配，不能直接存 id，改存"信号所在
    节点坐标 + 该边另一端节点坐标"，加载时按坐标反查（复用 node_id_at
    的按坐标去重机制，同一套容差语义）。存成顶层 "signals" 字段，跟
    "features" 平级——不进 LineString 几何格式，旧存档没有这个字段时
    优雅退化成"无信号"。这是刻意从简的表示，后续如果需要更稳定的引用
    方式（比如给 Node 加持久 UUID）可以替换，不影响这里的调用方接口。

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

    with open(path, "w") as f:
        json.dump(data, f, indent=2)
