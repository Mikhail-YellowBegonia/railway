"""Trace real search/plan/train flows and diagnose a repeated occupancy join."""
import contextlib
import io
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
from unittest.mock import patch

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame

from controller.action_router import action_for_key
from controller.game_loop import GameLoop
from controller.ui_state import FeedbackCenter
from model import route_debug
from model.occupancy import OccupancyState, advance_occupied_path
from model.pathfinding import find_path_from_point
from model.plan import Anchor, Plan, PlanItem
from model.plan_dispatch import PlanDispatcher
from model.plan_path import PathStart, resolve_plan_item
from model.rail_network import RailNetwork
from model.train_entity import TrainEntity, TrainState
from model.train_physics import SimplePhysics
from model.vec3 import Vec3
from model.wagon import Consist, create_simple_car
from view.camera import Camera
from view.renderer import draw_network_ids


network = RailNetwork()
nodes = [network.add_node(Vec3(x, 0, 0)) for x in (0, 100, 200, 300)]
edges = [network.add_edge(a, b).edge_id for a, b in zip(nodes, nodes[1:])]
e0, e1, e2 = edges
stdout = io.StringIO()

with tempfile.TemporaryDirectory(prefix="test-route-debug-") as directory:
    log_path = Path(directory) / "routes.jsonl"
    # Off means no output, including the same-edge shortcut.
    with contextlib.redirect_stdout(stdout):
        off = find_path_from_point(network, e0, .2, 1, e0, .8, 1)
    assert stdout.getvalue() == "" and off is not None

    with contextlib.redirect_stdout(stdout):
        route_debug.set_enabled(True, log_path=log_path)
        direct = find_path_from_point(network, e0, .2, 1, e0, .8, 1)
        assert direct == off
        assert find_path_from_point(network, e0, .8, 1, e0, .2, 1) is None
        resolved = resolve_plan_item(network, PathStart(e0, .2, 1),
                                     PlanItem.goto((e2, .5, 1), (Anchor(nodes[1].node_id),)))
        assert resolved.ok and list(resolved.path.edges) == [(eid, 1) for eid in edges]
        wagon = create_simple_car(length=10, mass=20, P_rated=1000, have_control=True)
        train = TrainEntity(TrainState(OccupancyState([(e0, 1)], 0, 20, []),
                                      0, 0, Consist([wagon])), network, SimplePhysics())
        wagon.plan = Plan([PlanItem.goto((e2, .5, 1))])
        PlanDispatcher(network).tick(train, [train])
        assert train.state.occupancy.route == [(e1, 1), (e2, 1)]
        train.emergency_stop()
        assert train.reverse_in_place()
        # Trace keyword-based coupling too; diagnostics must preserve the API.
        front = TrainEntity(TrainState(OccupancyState([(e1, 1)], 0, 60, []),
                                      0, 0, Consist([create_simple_car(length=10, mass=20)])), network, SimplePhysics())
        rear = TrainEntity(TrainState(OccupancyState([(e1, 1)], 0, 50, []),
                                     0, 0, Consist([create_simple_car(length=10, mass=20)])), network, SimplePhysics())
        merged = front.couple_with(rear=rear)
        assert len(merged.state.consist.wagons) == 2
        route_debug.set_enabled(False)

    records = [json.loads(line) for line in log_path.read_text().splitlines()]
    begins = {r["query"]: r for r in records if r["kind"] == "search.begin"}
    results = [r for r in records if r["kind"] == "search.result"]
    assert {r["query"] for r in results} == set(begins)
    assert any(r["found"] is False for r in results)
    assert any(r.get("edges") == [[e0, 1]] and r["start_offset"] == 20 for r in results)
    assert any(begins[r["query"]]["source"].endswith("resolve_plan_item")
               and len(r.get("edges", [])) == 3 for r in results)
    assert any(r["parent"] in begins for r in begins.values())
    assignment = next(r for r in records if r["kind"] == "train.assign_route")
    assert assignment["after"]["actual_head"]["edge"] == [e0, 1]
    assert assignment["after"]["route"]["edges"] == [[e1, 1], [e2, 1]]
    assert assignment["after"]["plan_command"] == "goto"
    reversal = next(r for r in records if r["kind"] == "train.reverse_in_place")
    assert reversal["before"]["actual_head"]["edge"] == [e0, 1]
    assert reversal["after"]["actual_head"]["edge"] == [e0, -1]
    coupling = next(r for r in records if r["kind"] == "train.couple_with")
    assert coupling["partner"] is not None and len(coupling["after"]["train"]) == 2

    # Observation must expose an illegal repeated join without silently fixing
    # it or rejecting legal same-edge reversal pairs.
    with contextlib.redirect_stdout(stdout):
        route_debug.set_enabled(True)
        advance_occupied_path(network, OccupancyState([(e0, 1), (e1, 1)], 0, 199, [(e1, 1)]), 2, 10)
        route_debug.set_enabled(False)
    records = [json.loads(line) for line in log_path.read_text().splitlines()]
    joined = next(r["joined"] for r in records if r["kind"] == "occupancy.append")
    assert joined["repeated_edges"] == {str(e1): [2, 3]}
    assert joined["disconnected_after"][0]["after"] == 2
    legal = route_debug.sequence_details(network, [(e1, 1), (e1, -1)])
    assert legal["reversal_after"] == [1] and not legal["disconnected_after"]

    # F3 must bypass a consuming Schedule GUI, and toggling off must leave
    # world/overlay state untouched.
    pygame.init()
    loop = GameLoop.__new__(GameLoop)
    loop.trains = []
    loop.feedback = FeedbackCenter()
    loop.gui = SimpleNamespace(process_event=lambda event: (_ for _ in ()).throw(AssertionError("GUI consumed F3")))
    event = pygame.event.Event(pygame.KEYDOWN, key=pygame.K_F3, mod=0)
    for context in ("workspace", "workspace_play", "schedule", "plan_edit", "consist_builder"):
        assert action_for_key(event, context) == "debug.route.toggle"
    with contextlib.redirect_stdout(stdout):
        loop._handle_event(event)
        assert route_debug.is_enabled()
        loop._handle_event(event)
        assert not route_debug.is_enabled()

# Real arc geometry: arrow at the curve midpoint, pointing node_a -> node_b.
arc_network = RailNetwork()
a = arc_network.add_node(Vec3(0, 0, 0))
b = arc_network.add_node(Vec3(100, 0, 0))
arc = arc_network.add_edge(a, b, [Vec3(50, 50, 0)])
camera = Camera()
camera.scale = 2
camera.pan_x = -50
surface = pygame.Surface((400, 300))
font = pygame.font.Font(None, 20)
polygons = []
with patch("view.renderer.pygame.draw.polygon", side_effect=lambda surface, color, pts, *args: polygons.append(pts)):
    draw_network_ids(surface, camera, arc_network, font)
assert polygons
tip, left, right = polygons[0]
base_x, base_y = (left[0] + right[0]) / 2, (left[1] + right[1]) / 2
assert tip[0] > base_x and abs(tip[1] - base_y) < 2
assert 90 < tip[1] < 130, "arrow must follow the arc, not the straight chord at y=150"
pygame.quit()
print("✅ 寻路调试：快捷返回/失败/锚点/实时调度/折返/重复拼接/模态快捷键/圆弧方向")
