"""POI 模式草稿与增删查交互状态。"""
from __future__ import annotations

from model.poi import POI, POIKind, POIMemberKind, POITable, StationTable


class POIEditor:
    def __init__(
        self,
        network,
        pois: POITable,
        stations: StationTable | None = None,
    ) -> None:
        self.network = network
        self.pois = pois
        self.stations = stations
        self.member_kind: POIMemberKind | None = None
        self.member_ids: list[int] = []
        self.poi_kind: POIKind | None = None
        self.poi_name: str | None = None
        self.hovered_poi_id: str | None = None
        self.station_mode: bool = False
        self.station_platform_ids: list[str] = []

    def reset_draft(self) -> None:
        self.member_kind = None
        self.member_ids.clear()
        self.poi_kind = None
        self.poi_name = None
        self.station_mode = False
        self.station_platform_ids.clear()

    def begin_station(self) -> str:
        self.member_kind = None
        self.member_ids.clear()
        self.station_mode = True
        self.station_platform_ids.clear()
        return "Station：请选择一个或多个 Platform，Enter 创建"

    def begin_depot(self) -> str:
        """Switch the current edge draft to Depot semantics."""
        if self.station_mode:
            return "Depot：请先退出 Station 创建状态"
        if self.member_kind == POIMemberKind.NODE:
            return "Depot：只能选择 edge，当前 node 草稿已保留"
        self.poi_kind = POIKind.DEPOT
        return "Depot：请选择连续 edge，Enter 创建；V 可切回 Platform"

    def begin_platform(self) -> str:
        if self.station_mode:
            return "Platform：请先退出 Station 创建状态"
        if self.member_kind == POIMemberKind.NODE:
            return "Platform：只能选择 edge，当前 node 草稿已保留"
        self.poi_kind = POIKind.PLATFORM
        return "Platform：请选择连续 edge，Enter 创建"

    def set_name(self, name: str | None) -> str:
        cleaned = (name or "").strip()
        self.poi_name = cleaned or None
        return f"POI 名称：{self.poi_name or '使用自动名称'}"

    def append_name_text(self, text: str) -> str:
        self.poi_name = (self.poi_name or "") + text
        return f"POI 名称：{self.poi_name or '使用自动名称'}"

    def backspace_name(self) -> str:
        self.poi_name = (self.poi_name or "")[:-1] or None
        return f"POI 名称：{self.poi_name or '使用自动名称'}"

    def cancel_station(self) -> str:
        self.station_mode = False
        self.station_platform_ids.clear()
        return "Station：已取消创建"

    def toggle_station_platform(self, poi_id: str | None) -> str:
        if not self.station_mode:
            return "Station：当前不在车站创建状态"
        poi = self.pois.get(poi_id or "")
        if poi is None or poi.kind != POIKind.PLATFORM:
            return "Station：请选择已有 Platform"
        if poi.poi_id in self.station_platform_ids:
            self.station_platform_ids.remove(poi.poi_id)
            return f"Station：已取消 {poi.name}"
        self.station_platform_ids.append(poi.poi_id)
        return f"Station：已选择 {poi.name}（共 {len(self.station_platform_ids)} 个 Platform）"

    def toggle_member(self, member_kind: POIMemberKind, member_id: int) -> str:
        source = self.network.edges if member_kind == POIMemberKind.EDGE else self.network.nodes
        if member_id not in source:
            return "POI：目标元素不存在"
        if self.member_kind is not None and self.member_kind != member_kind:
            return "POI：一个对象只能由 edge 或 node 其中一类组成"
        self.member_kind = member_kind
        if member_kind == POIMemberKind.NODE:
            self.poi_kind = POIKind.WAYPOINT
        elif self.poi_kind not in (POIKind.PLATFORM, POIKind.DEPOT):
            self.poi_kind = POIKind.PLATFORM
        if member_id in self.member_ids:
            self.member_ids.remove(member_id)
            if not self.member_ids:
                self.member_kind = None
            return f"POI：已取消 {member_kind.value} {member_id}"
        self.member_ids.append(member_id)
        return f"POI：已选择 {member_kind.value} {member_id}（共 {len(self.member_ids)} 个）"

    def backspace(self) -> str:
        if self.station_mode:
            if not self.station_platform_ids:
                return "Station：草稿为空"
            platform_id = self.station_platform_ids.pop()
            platform = self.pois.get(platform_id)
            return f"Station：已撤销 {platform.name if platform is not None else platform_id}"
        if not self.member_ids:
            return "POI：草稿为空"
        member_id = self.member_ids.pop()
        if not self.member_ids:
            self.member_kind = None
        return f"POI：已撤销成员 {member_id}"

    def confirm(self) -> tuple[POI | None, str]:
        if self.station_mode:
            return None, "Station 创建失败：请使用 confirm_station()"
        if self.member_kind is None or not self.member_ids:
            return None, "POI 创建失败：请先选择 node 或 edge"
        try:
            poi = self.pois.create(
                self.member_kind, self.member_ids, kind=self.poi_kind,
                name=self.poi_name, network=self.network,
            )
        except ValueError as exc:
            return None, f"POI 创建失败：{exc}"
        self.reset_draft()
        label = poi.kind.value
        return poi, f"POI 已创建：{poi.name}（{len(poi.member_ids)} 个 {label} edge/node）"

    def confirm_station(self):
        if not self.station_mode or self.stations is None:
            return None, "Station 创建失败：当前不在车站创建状态"
        if not self.station_platform_ids:
            return None, "Station 创建失败：请先选择 Platform"
        try:
            station = self.stations.create(
                self.station_platform_ids, platforms=self.pois,
            )
        except ValueError as exc:
            return None, f"Station 创建失败：{exc}"
        self.station_mode = False
        self.station_platform_ids.clear()
        return station, f"Station 已创建：{station.name}（{len(station.platform_ids)} 个 Platform）"

    def delete_hovered(self) -> tuple[POI | None, str]:
        if self.hovered_poi_id is None:
            return None, "POI 删除失败：请先悬停一个 POI"
        poi = self.pois.remove(self.hovered_poi_id)
        self.hovered_poi_id = None
        if poi is None:
            return None, "POI 删除失败：对象已不存在"
        message = f"POI 已删除：{poi.name}"
        if poi.kind == POIKind.PLATFORM and self.stations is not None:
            updated, removed = self.stations.remove_platform_reference(poi.poi_id)
            affected_count = len(updated) + len(removed)
            message += (
                f"；已从 {affected_count} 个车站清理引用"
                f"，删除 {len(removed)} 个空车站"
            )
        return poi, message
