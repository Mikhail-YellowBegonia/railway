from __future__ import annotations

import json
from pathlib import Path

from model.rail_network import RailNetwork


def write_geojson(network: RailNetwork, path: str | Path) -> None:
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

    with open(path, "w") as f:
        json.dump(data, f, indent=2)
