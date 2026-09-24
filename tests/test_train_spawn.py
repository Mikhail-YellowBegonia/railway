"""Depot POI and initial consist placement regression."""
from __future__ import annotations

import os
import tempfile

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame

from model.depot import ordered_depot_path
from model.poi import POIKind, POIMemberKind, POITable
from model.rail_network import RailNetwork
from model.train_spawn import spawn_train_at_depot
from model.vec3 import Vec3
from model.wagon import Consist, create_simple_car
from model.geojson_writer import write_geojson
from controller.editor import EditMode


net = RailNetwork()
n0 = net.add_node(Vec3(0, 0, 0))
n1 = net.add_node(Vec3(100, 0, 0))
n2 = net.add_node(Vec3(200, 0, 0))
e0 = net.add_edge(n0, n1)
e1 = net.add_edge(n1, n2)
pois = POITable()
depot = pois.create(
    POIMemberKind.EDGE, [e0.edge_id, e1.edge_id],
    kind=POIKind.DEPOT, name="Main Depot", network=net,
)
assert depot.kind == POIKind.DEPOT
path = ordered_depot_path(depot, net, direction=1)
reverse = ordered_depot_path(depot, net, direction=-1)
assert path.edges == [(e0.edge_id, 1), (e1.edge_id, 1)]
assert reverse.edges == [(e1.edge_id, -1), (e0.edge_id, -1)]
print("✅ Depot edge 集合可按两个方向解析为有限路径")

chain_a = net.add_node(Vec3(0, 100, 0))
chain_b = net.add_node(Vec3(50, 100, 0))
chain_c = net.add_node(Vec3(100, 100, 0))
loop_a = net.add_edge(chain_a, chain_b)
loop_b = net.add_edge(chain_b, chain_c)
loop_c = net.add_edge(chain_c, chain_a)
try:
    pois.create(POIMemberKind.EDGE, [loop_a.edge_id, loop_b.edge_id, loop_c.edge_id], network=net)
    raise AssertionError("闭合 Platform 不应创建")
except ValueError as exc:
    assert "不能闭合" in str(exc)
try:
    pois.create(
        POIMemberKind.EDGE, [loop_a.edge_id, loop_b.edge_id, loop_c.edge_id],
        kind=POIKind.DEPOT, network=net,
    )
    raise AssertionError("闭合 Depot 不应创建")
except ValueError as exc:
    assert "不能闭合" in str(exc)
print("✅ Platform/Depot 共同拒绝闭合 edge 集合")

consist = Consist(wagons=[
    create_simple_car(length=20.0, mass=50.0, P_rated=3000.0, have_control=True),
    create_simple_car(length=20.0, mass=50.0, P_rated=None, have_control=False),
])
train = spawn_train_at_depot(net, depot, consist, direction=1)
assert train.is_parked()
assert train.state.v == 0.0
assert train.state.occupancy.route == []
poses = train.kinematics.get_all_wagon_poses(train.state.s)
assert len(poses) == 2
assert abs(poses[0].position.x - poses[1].position.x) > 10.0
assert train.state.occupancy.occupied_offset >= 0.0
print("✅ 编组生成后车厢沿 Depot 排开，而非挤在同一点")

too_long = Consist(wagons=[
    create_simple_car(length=110.0, mass=50.0),
    create_simple_car(length=110.0, mass=50.0),
])
try:
    spawn_train_at_depot(net, depot, too_long)
    raise AssertionError("超过 Depot 长度的编组不应生成")
except ValueError as exc:
    assert "编组过长" in str(exc)
print("✅ 超过 Depot 长度的编组被拒绝")

# Minimal GameLoop integration: click Depot, add a coach, generate, then reject
# a second generation while the Depot remains occupied.
with tempfile.TemporaryDirectory() as tmp:
    import controller.game_loop as gl_module

    save = os.path.join(tmp, "depot_spawn.geojson")
    write_geojson(net, save, pois=pois)
    old_save = gl_module.SAVE_PATH
    gl_module.SAVE_PATH = save
    try:
        game = gl_module.GameLoop(save)
        game._change_mode(EditMode.PLAY)
        game._play_left_click(Vec3(100, 0, 0))
        assert game.train_placement_depot_id == depot.poi_id
        game._handle_consist_builder_action("consist.catalog.coach")
        game._handle_consist_builder_action("consist.add")
        game._handle_consist_builder_action("consist.move.left")
        game._handle_consist_builder_action("consist.reverse")
        game._handle_consist_builder_action("consist.complete")
        assert len(game.trains) == 1
        assert game.active_train is game.trains[0]
        game._play_left_click(Vec3(100, 0, 0))
        assert game.train_placement_depot_id is None
        assert len(game.trains) == 1
    finally:
        gl_module.SAVE_PATH = old_save
        pygame.quit()
print("✅ GameLoop：Depot 点击→编组添加→生成，已占用 Depot 拒绝再次生成")

print("\n全部通过")
