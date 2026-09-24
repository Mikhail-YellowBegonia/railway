"""独立 POI 基础模型：无方向 Node/Edge 集合，不依赖计划或 selector。"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from uuid import uuid4


class POIKind(str, Enum):
    PLATFORM = "platform"
    DEPOT = "depot"
    WAYPOINT = "waypoint"


class POIMemberKind(str, Enum):
    EDGE = "edge"
    NODE = "node"


@dataclass(frozen=True, slots=True)
class POI:
    poi_id: str
    name: str
    kind: POIKind
    member_kind: POIMemberKind
    member_ids: tuple[int, ...]

    def validate(self, network=None) -> list[str]:
        problems: list[str] = []
        if not self.poi_id:
            problems.append("POI id 不能为空")
        if not self.name.strip():
            problems.append("POI 名称不能为空")
        if not self.member_ids:
            problems.append("POI 至少需要一个成员")
        if len(set(self.member_ids)) != len(self.member_ids):
            problems.append("POI 成员不能重复")
        valid_kinds = (
            {POIKind.PLATFORM, POIKind.DEPOT}
            if self.member_kind == POIMemberKind.EDGE
            else {POIKind.WAYPOINT}
        )
        if self.kind not in valid_kinds:
            problems.append(
                f"{self.member_kind.value} POI 类型不匹配：{self.kind.value}"
            )
        if network is not None:
            source = (
                network.edges
                if self.member_kind == POIMemberKind.EDGE
                else network.nodes
            )
            missing = sorted(member_id for member_id in self.member_ids if member_id not in source)
            if missing:
                problems.append(f"POI 成员不存在：{missing}")
            if self.member_kind == POIMemberKind.EDGE and not missing:
                if not _edge_sequence_is_continuous(self.member_ids, source):
                    problems.append("edge POI 必须组成一条按顺序连续、无分叉的路径")
                elif _edge_members_form_closed_loop(self.member_ids, source):
                    problems.append("Platform/Depot 的 edge 集合不能闭合")
        return problems


class POITable:
    """POI 的增删查容器；不持有 RailNetwork，也不承担拓扑改写。"""

    def __init__(self) -> None:
        self._pois: dict[str, POI] = {}

    def create(
        self,
        member_kind: POIMemberKind,
        member_ids,
        *,
        kind: POIKind | None = None,
        name: str | None = None,
        poi_id: str | None = None,
        network=None,
    ) -> POI:
        members = tuple(int(member_id) for member_id in member_ids)
        if kind is None:
            kind = (
                POIKind.PLATFORM
                if member_kind == POIMemberKind.EDGE
                else POIKind.WAYPOINT
            )
        if name is None:
            prefix = {
                POIKind.PLATFORM: "Platform",
                POIKind.DEPOT: "Depot",
                POIKind.WAYPOINT: "Waypoint",
            }[kind]
            name = self._next_default_name(prefix)
        poi = POI(
            poi_id=poi_id or str(uuid4()),
            name=name,
            kind=kind,
            member_kind=member_kind,
            member_ids=members,
        )
        problems = poi.validate(network)
        if problems:
            raise ValueError("；".join(problems))
        if poi.poi_id in self._pois:
            raise ValueError(f"POI id 已存在：{poi.poi_id}")
        self._pois[poi.poi_id] = poi
        return poi

    def get(self, poi_id: str) -> POI | None:
        return self._pois.get(poi_id)

    def remove(self, poi_id: str) -> POI | None:
        return self._pois.pop(poi_id, None)

    def all(self) -> tuple[POI, ...]:
        return tuple(self._pois.values())

    def containing(self, member_kind: POIMemberKind, member_id: int) -> tuple[POI, ...]:
        return tuple(
            poi for poi in self._pois.values()
            if poi.member_kind == member_kind and member_id in poi.member_ids
        )

    def __len__(self) -> int:
        return len(self._pois)

    def _next_default_name(self, prefix: str) -> str:
        used = {poi.name for poi in self._pois.values()}
        number = 1
        while f"{prefix} {number}" in used:
            number += 1
        return f"{prefix} {number}"


@dataclass(frozen=True, slots=True)
class Station:
    station_id: str
    name: str
    platform_ids: tuple[str, ...]

    def validate(self, platforms: POITable | None = None) -> list[str]:
        problems: list[str] = []
        if not self.station_id:
            problems.append("车站 id 不能为空")
        if not self.name.strip():
            problems.append("车站名称不能为空")
        if not self.platform_ids:
            problems.append("车站至少需要一个 platform")
        if len(set(self.platform_ids)) != len(self.platform_ids):
            problems.append("车站 platform 不能重复")
        if platforms is not None:
            missing = [pid for pid in self.platform_ids if platforms.get(pid) is None]
            if missing:
                problems.append(f"车站引用的 platform 不存在：{missing}")
            non_platform = [
                pid for pid in self.platform_ids
                if platforms.get(pid) is not None
                and platforms.get(pid).kind != POIKind.PLATFORM
            ]
            if non_platform:
                problems.append(f"车站只能引用 platform：{non_platform}")
        return problems


class StationTable:
    def __init__(self) -> None:
        self._stations: dict[str, Station] = {}

    def create(self, platform_ids, *, name: str | None = None,
               station_id: str | None = None,
               platforms: POITable | None = None) -> Station:
        members = tuple(str(pid) for pid in platform_ids)
        station = Station(
            station_id or str(uuid4()),
            name or self._next_default_name(),
            members,
        )
        problems = station.validate(platforms)
        if problems:
            raise ValueError("；".join(problems))
        if station.station_id in self._stations:
            raise ValueError(f"车站 id 已存在：{station.station_id}")
        self._stations[station.station_id] = station
        return station

    def get(self, station_id: str) -> Station | None:
        return self._stations.get(station_id)

    def remove(self, station_id: str) -> Station | None:
        return self._stations.pop(station_id, None)

    def containing_platform(self, platform_id: str) -> tuple[Station, ...]:
        return tuple(
            station for station in self._stations.values()
            if platform_id in station.platform_ids
        )

    def remove_platform_reference(
        self, platform_id: str,
    ) -> tuple[tuple[Station, ...], tuple[Station, ...]]:
        """清理对 platform 的引用，返回（已更新车站，已删除空车站）。"""
        updated: list[Station] = []
        removed: list[Station] = []
        for station_id, station in tuple(self._stations.items()):
            if platform_id not in station.platform_ids:
                continue
            remaining = tuple(
                candidate for candidate in station.platform_ids
                if candidate != platform_id
            )
            if not remaining:
                removed.append(self._stations.pop(station_id))
                continue
            replacement = Station(station.station_id, station.name, remaining)
            self._stations[station_id] = replacement
            updated.append(replacement)
        return tuple(updated), tuple(removed)

    def all(self) -> tuple[Station, ...]:
        return tuple(self._stations.values())

    def __len__(self) -> int:
        return len(self._stations)

    def _next_default_name(self) -> str:
        used = {station.name for station in self._stations.values()}
        number = 1
        while f"Station {number}" in used:
            number += 1
        return f"Station {number}"


def poi_world_bounds(poi: POI, network) -> tuple[float, float, float, float] | None:
    """返回 POI 成员几何的 XY 包络；不含显示 padding。"""
    points = []
    if poi.member_kind == POIMemberKind.NODE:
        for node_id in poi.member_ids:
            node = network.nodes.get(node_id)
            if node is not None:
                points.append(node.position)
    else:
        for edge_id in poi.member_ids:
            edge = network.edges.get(edge_id)
            if edge is None:
                continue
            if edge.is_arc:
                points.extend(edge.sample_arc_points(30))
            else:
                a = network.nodes.get(edge.node_a_id)
                b = network.nodes.get(edge.node_b_id)
                if a is not None:
                    points.append(a.position)
                if b is not None:
                    points.append(b.position)
    if not points:
        return None
    xs = [point.x for point in points]
    ys = [point.y for point in points]
    return min(xs), min(ys), max(xs), max(ys)


def _edge_sequence_is_continuous(member_ids: tuple[int, ...], edges) -> bool:
    if len(member_ids) <= 1:
        return True
    first = edges[member_ids[0]]
    possible_ends = {first.node_a_id, first.node_b_id}
    for edge_id in member_ids[1:]:
        edge = edges[edge_id]
        next_ends: set[int] = set()
        if edge.node_a_id in possible_ends:
            next_ends.add(edge.node_b_id)
        if edge.node_b_id in possible_ends:
            next_ends.add(edge.node_a_id)
        if not next_ends:
            return False
        possible_ends = next_ends
    return True


def _edge_members_form_closed_loop(member_ids: tuple[int, ...], edges) -> bool:
    """Return whether the selected edge subgraph has no endpoint.

    Platform and Depot are finite, traversable facilities. A closed edge set has
    no unambiguous spawn/arrival endpoint, so it is rejected for both kinds.
    """
    degrees: dict[int, int] = {}
    for edge_id in member_ids:
        edge = edges[edge_id]
        degrees[edge.node_a_id] = degrees.get(edge.node_a_id, 0) + 1
        degrees[edge.node_b_id] = degrees.get(edge.node_b_id, 0) + 1
    return bool(degrees) and all(degree == 2 for degree in degrees.values())
