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
