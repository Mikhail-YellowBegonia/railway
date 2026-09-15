"""P4 GameLoop 端到端：受保护切边、固定路线改写和 PBS 准入。"""

import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame

import controller.game_loop as gl_module
from controller.editor import EditMode
from controller.game_loop import GameLoop
from model.geojson_writer import write_geojson
from model.occupancy import OccupancyState
from model.plan import FixedRoute, Plan, PlanItem
from model.rail_network import RailNetwork
from model.train_entity import TrainEntity, TrainState
from model.train_physics import RealisticElectric
from model.vec3 import Vec3
from model.wagon import Consist, create_simple_wagon


network = RailNetwork()
a = network.add_node(Vec3(0.0, 0.0, 0.0))
b = network.add_node(Vec3(100.0, 0.0, 0.0))
c = network.add_node(Vec3(200.0, 0.0, 0.0))
e0 = network.add_edge(a, b).edge_id
e1 = network.add_edge(b, c).edge_id

save_path = os.path.join(tempfile.gettempdir(), "topology_guard_game_loop.geojson")
write_geojson(network, save_path)
old_save_path = gl_module.SAVE_PATH
pygame.init()
try:
    gl_module.SAVE_PATH = save_path
    game = GameLoop(save_path)
    wagons = [create_simple_wagon(length=20.0, mass=50.0, P_rated=3000.0)]
    train = TrainEntity(
        TrainState(
            occupancy=OccupancyState([(e0, 1)], 40.0, 60.0, [(e1, 1)]),
            remaining_to_goal=140.0,
            v=0.0,
            consist=Consist(wagons=wagons),
            goal=(e1, 0.75, 1),
        ),
        game.network,
        RealisticElectric(),
    )
    train.kinematics = train._build_kinematics()
    train.state.consist.wagons[0].plan = Plan([PlanItem.goto(
        (e1, 0.75, 1), fixed_route=FixedRoute(((e1, 1),), 0.0, 25.0),
    )])
    game.trains = [train]

    # ① 待走路线可切；route / goal / FixedRoute 同步指向子边。
    mid = game._split_edge_with_guard(e1, 0.5)
    assert mid is not None
    child_edges = sorted(game.network.nodes[mid].incident_edge_ids)
    assert len(child_edges) == 2
    route = train.state.occupancy.route
    assert len(route) == 2 and {eid for eid, _ in route} == set(child_edges)
    assert train.state.goal is not None and train.state.goal[0] in child_edges
    plan_item = train.state.consist.wagons[0].plan.current()
    assert plan_item is not None and plan_item.fixed_route is not None
    assert set(plan_item.fixed_route.used_edge_ids()) == set(child_edges)
    print("✅ ① GameLoop 切分同步改写运行 route / goal / FixedRoute")

    # ② 固定路线引用的子边不可 DELETE，且控制台检查不需 P3b 才生效。
    game.editor.set_mode(EditMode.DELETE)
    game.editor.hovered_edge_id = child_edges[0]
    assert game._rail_delete_blocked_reason() is not None
    print("✅ ② 固定路线引用阻止 DELETE")

    # ③ 去掉计划后，普通运行 route 仍受同一保护：反向 PBS 不能让既有手动指令失效。
    train.state.consist.wagons[0].plan = None
    before_signals = game.signals.all_signals()
    game._signal_left_click(Vec3(149.9, 0.0, 0.0))
    assert game.signals.all_signals() == before_signals
    print("✅ ③ 反向 One-Way PBS 放置被运行 route 准入拒绝")
finally:
    gl_module.SAVE_PATH = old_save_path
    if os.path.exists(save_path):
        os.remove(save_path)
    pygame.quit()

print("\n全部通过")
