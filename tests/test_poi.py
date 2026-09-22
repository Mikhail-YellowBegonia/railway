"""POI 基础模型、编辑草稿、渲染与持久化回归。"""
from __future__ import annotations

import json
import os
import tempfile

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame

from controller.editor import EditMode, Editor
from controller.poi_editor import POIEditor
from model.geojson_loader import load_geojson, load_pois
from model.geojson_writer import write_geojson
from model.poi import POIKind, POIMemberKind, POITable, StationTable
from model.rail_network import RailNetwork
from model.vec3 import Vec3
from view.camera import Camera
from view.renderer import Renderer


net = RailNetwork()
n0 = net.add_node(Vec3(0, 0, 0))
n1 = net.add_node(Vec3(100, 0, 0))
n2 = net.add_node(Vec3(200, 0, 0))
e0 = net.add_edge(n0, n1)
e1 = net.add_edge(n1, n2)
n3 = net.add_node(Vec3(500, 0, 0))
n4 = net.add_node(Vec3(600, 0, 0))
e2 = net.add_edge(n3, n4)

# ① 增删查 + 默认属性
table = POITable()
platform = table.create(POIMemberKind.EDGE, [e0.edge_id, e1.edge_id], network=net)
waypoint = table.create(POIMemberKind.NODE, [n1.node_id], network=net)
assert platform.kind == POIKind.PLATFORM and platform.name == "Platform 1"
assert waypoint.kind == POIKind.WAYPOINT and waypoint.name == "Waypoint 1"
assert table.get(platform.poi_id) is platform
assert table.containing(POIMemberKind.EDGE, e0.edge_id) == (platform,)
assert len(table.all()) == 2
assert table.remove(waypoint.poi_id) is waypoint and table.get(waypoint.poi_id) is None
print("✅ ① POI 增删查与 edge→platform / node→waypoint 默认属性正确")

stations = StationTable()
station = stations.create([platform.poi_id], platforms=table)
assert station.name == "Station 1" and station.platform_ids == (platform.poi_id,)
assert stations.get(station.station_id) is station
assert stations.remove(station.station_id) is station
print("✅ ①b Station 二层集合可增删查并只引用 platform")

try:
    table.create(POIMemberKind.EDGE, [e0.edge_id, e2.edge_id], network=net)
    raise AssertionError("不连续 edge 不应创建 platform")
except ValueError as exc:
    assert "连续" in str(exc)
print("✅ ①c Platform 拒绝不连续 edge 集合")

# ② 草稿类型互斥、切换选择、确认和撤销
drafts = POITable()
editor = POIEditor(net, drafts)
assert "已选择" in editor.toggle_member(POIMemberKind.EDGE, e0.edge_id)
assert "只能由 edge 或 node" in editor.toggle_member(POIMemberKind.NODE, n0.node_id)
assert "已选择" in editor.toggle_member(POIMemberKind.EDGE, e1.edge_id)
created, message = editor.confirm()
assert created is not None and created.member_ids == (e0.edge_id, e1.edge_id)
assert "已创建" in message and editor.member_kind is None
editor.toggle_member(POIMemberKind.NODE, n0.node_id)
assert "已撤销" in editor.backspace() and editor.member_kind is None
print("✅ ② POI 草稿只允许同类成员，Enter 确认所需状态与 Backspace 撤销正确")

# ②a Station GUI 草稿：从已有 Platform 选择集合并创建 Station。
station_drafts = StationTable()
station_editor = POIEditor(net, table, station_drafts)
assert "请选择" in station_editor.begin_station()
assert "已选择" in station_editor.toggle_station_platform(platform.poi_id)
created_station, message = station_editor.confirm_station()
assert created_station is not None
assert created_station.platform_ids == (platform.poi_id,)
assert "已创建" in message and not station_editor.station_mode
print("✅ ②a Station 草稿可从已有 Platform 集合创建 Station")

# ②b 删除 Platform 时清理 Station 引用；无剩余 Platform 的 Station 同步删除
delete_pois = POITable()
kept_platform = delete_pois.create(
    POIMemberKind.EDGE, [e0.edge_id], name="Keep", network=net,
)
deleted_platform = delete_pois.create(
    POIMemberKind.EDGE, [e1.edge_id], name="Delete", network=net,
)
delete_stations = StationTable()
kept_station = delete_stations.create(
    [deleted_platform.poi_id, kept_platform.poi_id],
    name="Keep Station",
    platforms=delete_pois,
)
empty_station = delete_stations.create(
    [deleted_platform.poi_id], name="Empty Station", platforms=delete_pois,
)
delete_editor = POIEditor(net, delete_pois, delete_stations)
delete_editor.hovered_poi_id = deleted_platform.poi_id
deleted, message = delete_editor.delete_hovered()
assert deleted is deleted_platform and delete_pois.get(deleted_platform.poi_id) is None
assert delete_stations.get(kept_station.station_id).platform_ids == (kept_platform.poi_id,)
assert delete_stations.get(empty_station.station_id) is None
assert "2 个车站" in message and "删除 1 个空车站" in message
assert delete_stations.containing_platform(deleted_platform.poi_id) == ()
with tempfile.TemporaryDirectory() as tmp:
    cleaned_save = os.path.join(tmp, "cleaned_poi.geojson")
    write_geojson(net, cleaned_save, pois=delete_pois, stations=delete_stations)
    cleaned_pois = load_pois(cleaned_save, load_geojson(cleaned_save))
    from model.geojson_loader import load_stations
    cleaned_stations = load_stations(cleaned_save, cleaned_pois)
    assert len(cleaned_stations) == 1
    assert cleaned_stations.all()[0].platform_ids == (kept_platform.poi_id,)
print("✅ ②b 删除 Platform 会清理 Station 引用，并删除失去全部 Platform 的空车站")

# ③ 持久化按坐标恢复，不依赖运行时 id；旧档无 pois 优雅退化
with tempfile.TemporaryDirectory() as tmp:
    save = os.path.join(tmp, "poi.geojson")
    station = stations.create([platform.poi_id], platforms=table)
    write_geojson(net, save, pois=table, stations=stations)
    raw = json.load(open(save))
    assert raw["pois"][0]["member_kind"] == "edge"
    assert "direction" not in json.dumps(raw["pois"])
    loaded_net = load_geojson(save)
    loaded = load_pois(save, loaded_net)
    assert len(loaded) == 1
    got = loaded.all()[0]
    assert got.poi_id == platform.poi_id and got.name == platform.name
    assert got.kind == POIKind.PLATFORM and len(got.member_ids) == 2
    from model.geojson_loader import load_stations
    loaded_stations = load_stations(save, loaded)
    assert len(loaded_stations) == 1
    assert loaded_stations.all()[0].platform_ids == (platform.poi_id,)

    old = os.path.join(tmp, "old.geojson")
    write_geojson(net, old)
    assert len(load_pois(old, load_geojson(old))) == 0
print("✅ ③ POI 顶层字段持久化往返正确，成员无方向，旧存档兼容")

# ④ 编辑器新增 POI 顶层模式
base_editor = Editor(net)
base_editor.set_mode(EditMode.POI)
assert base_editor.mode == EditMode.POI
base_editor.handle_cancel()
assert base_editor.mode == EditMode.IDLE
print("✅ ④ 编辑器状态机包含 POI 模式，Esc 可退出")

# ⑤ 半透明带 padding 包络可在轨道绘制前独立输出
pygame.init()
surface = pygame.Surface((320, 240), pygame.SRCALPHA)
camera = Camera()
renderer = Renderer(surface, camera)
surface.fill((0, 0, 0, 0))
renderer.draw_pois(net, table)
alpha_pixels = sum(
    1 for x in range(surface.get_width()) for y in range(surface.get_height())
    if surface.get_at((x, y)).a > 0
)
assert alpha_pixels > 0
pygame.quit()
print("✅ ⑤ POI 半透明矩形包络渲染产生可见像素")

# ⑥ GameLoop 端到端：J → 选择 → Enter → S → 重载
with tempfile.TemporaryDirectory() as tmp:
    import controller.game_loop as gl_module

    save = os.path.join(tmp, "poi_e2e.geojson")
    write_geojson(net, save)
    old_save_path = gl_module.SAVE_PATH
    gl_module.SAVE_PATH = save
    try:
        game = gl_module.GameLoop(save)
        game._handle_keydown(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_j))
        assert game.editor.mode == EditMode.POI
        first_edge = next(iter(game.network.edges.values()))
        a = game.network.nodes[first_edge.node_a_id].position
        b = game.network.nodes[first_edge.node_b_id].position
        game._poi_left_click((a + b) * 0.5)
        game._handle_keydown(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_RETURN))
        assert len(game.pois) == 1
        game._handle_keydown(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_s))
        reloaded = gl_module.GameLoop(save)
        assert len(reloaded.pois) == 1
    finally:
        gl_module.SAVE_PATH = old_save_path
print("✅ ⑥ GameLoop：J 模式创建、S 保存与启动重载 POI 通过")

print("\n全部通过")
