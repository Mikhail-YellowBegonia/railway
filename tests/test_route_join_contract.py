"""Route ownership at the actual head: stale margins, rerouting and legal loops."""
import math
import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

from controller.game_loop import GameLoop
from model.occupancy import OccupancyState, trim_occupied_ahead_of_head
from model.pathfinding import find_path_from_point, head_node, tail_node
from model.plan import Plan, PlanItem
from model.plan_dispatch import PlanDispatcher
from model.rail_network import RailNetwork
from model.train_entity import TrainEntity, TrainState
from model.train_physics import SimplePhysics
from model.vec3 import Vec3
from model.wagon import Consist, create_simple_car


network = RailNetwork()
nodes = [network.add_node(Vec3(x, 0, 0)) for x in (0, 100, 200, 300)]
e0, e1, e2 = [network.add_edge(a, b).edge_id for a, b in zip(nodes, nodes[1:])]


def parked():
    wagon = create_simple_car(length=10, mass=20, P_rated=1000, have_control=True)
    return TrainEntity(TrainState(OccupancyState([(e0, 1), (e1, 1)], 0, 50, []),
                                  0, 0, Consist([wagon])), network, SimplePhysics())


def drive(train):
    previous = train.kinematics.get_all_bogie_poses(train.state.s)[0]
    for _ in range(60 * 60):
        train.update(1 / 60, 12)
        for left, right in zip(train.state.occupancy.occupied, train.state.occupancy.occupied[1:]):
            assert head_node(network, left) == tail_node(network, right)
        current = train.kinematics.get_all_bogie_poses(train.state.s)[0]
        for before, after in zip(previous, current):
            assert before.position.distance_to(after.position) < .3
        previous = current
        if train.is_parked():
            break
    assert train.is_parked()
    return train.kinematics._path_kin.pose_at(train.state.abs_s).position.x


# A legacy window's last edge is not the actual head. Manual goto must not
# reverse a correctly oriented train, and assigning a new future discards only
# unused window suffixes, preserving both bogie positions and path history.
train = parked()
assert train.head_directed_edge() == (e0, 1)
before = train.kinematics.get_all_bogie_poses(train.state.s)[0]
path, so, eo = find_path_from_point(network, e0, .5, 1, e2, .5, 1)
loop = GameLoop.__new__(GameLoop)
loop._apply_route_result(train, path, so, eo, e2, .5, 1)
assert train.current_direction() == 1
assert train.state.occupancy.occupied == [(e0, 1)]
assert train.state.occupancy.route == [(e1, 1), (e2, 1)]
after = train.kinematics.get_all_bogie_poses(train.state.s)[0]
assert all(a.position.distance_to(b.position) < 1e-9 for a, b in zip(before, after))
assert abs(drive(train) - 250) < .2

# An explicit plan reverse does not prohibit a later *necessary* endpoint
# reversal. The old pointer-based heuristic wrongly rejected this reachable
# destination. Execute both reversals, maintaining bogie continuity throughout.
train = parked()
train.state.occupancy = OccupancyState([(e1, 1)], 0, 50, [])
train.kinematics = train._build_kinematics()
driver = train.state.consist.control_winner()
driver.plan = Plan([PlanItem.reverse(), PlanItem.goto((e2, .5, 1))])
plans = PlanDispatcher(network)
plans.tick(train, [train])
plans.tick(train, [train])
resolved = train.plan_execution.resolved_route
assert resolved is not None
assert ((e0, -1), (e0, 1)) in tuple(zip(resolved.edges, resolved.edges[1:]))
# Automatic reversal swaps logical bogies in place; check world pose sets,
# not the old logical front/rear labels on that frame.
for _ in range(60 * 60):
    before = train.kinematics.get_all_bogie_poses(train.state.s)[0]
    direction = train.current_direction()
    train.update(1 / 60, 12)
    after = train.kinematics.get_all_bogie_poses(train.state.s)[0]
    if direction == train.current_direction():
        assert all(a.position.distance_to(b.position) < .3 for a, b in zip(before, after))
    else:
        assert all(min(a.position.distance_to(b.position) for b in after) < .3 for a in before)
    assert all(head_node(network, a) == tail_node(network, b)
               for a, b in zip(train.state.occupancy.occupied, train.state.occupancy.occupied[1:]))
    if train.is_parked():
        break
assert train.is_parked()
assert abs(train.kinematics._path_kin.pose_at(train.state.abs_s).position.x - 250) < .2

# At an automatic endpoint reversal a long consist's new head can be several
# edges beyond the point-model reversal marker. Those route occurrences are
# now behind the head and must not be appended to occupied a second time.
short = RailNetwork()
short_nodes = [short.add_node(Vec3(x, 0, 0)) for x in range(0, 151, 25)]
short_edges = [short.add_edge(a, b).edge_id for a, b in zip(short_nodes, short_nodes[1:])]
for goal_edge, goal_t, expected in ((short_edges[5], .5, 137.5), (short_edges[2], .7, 67.5)):
    wagons = [create_simple_car(length=20, mass=20, P_rated=1000 if i == 0 else None,
                               have_control=i == 0) for i in range(3)]
    train = TrainEntity(TrainState(OccupancyState([(e, -1) for e in reversed(short_edges)],
                                                 0, 60, []), 0, 0, Consist(wagons)),
                        short, SimplePhysics())
    path, so, eo = find_path_from_point(short, short_edges[3], .6, -1,
                                       goal_edge, goal_t, 1, allow_reversal=True, consist_length=60)
    loop._apply_route_result(train, path, so, eo, goal_edge, goal_t, 1)
    reversals = 0
    for _ in range(60 * 60):
        before = train.kinematics.get_all_bogie_poses(train.state.s)
        direction = train.current_direction()
        train.update(1 / 60, 12)
        after = train.kinematics.get_all_bogie_poses(train.state.s)
        if direction != train.current_direction():
            reversals += 1
            after = [(rear, front) for front, rear in reversed(after)]
        assert all(a.position.distance_to(b.position) < .3
                   for pair_a, pair_b in zip(before, after) for a, b in zip(pair_a, pair_b)), (
                       train.state.occupancy, direction, train.current_direction())
        occupied = train.state.occupancy.occupied
        assert all(head_node(short, a) == tail_node(short, b)
                   for a, b in zip(occupied, occupied[1:])), occupied
        if train.is_parked():
            break
    assert train.is_parked() and reversals == 1
    assert abs(train.kinematics._path_kin.pose_at(train.state.abs_s).position.x - expected) < .2

# A six-edge ring permits forward turns. Locate the second occurrence of its
# first edge by arc length; retain that occurrence and every historical edge.
# Keep a legitimate future lap in route intact (no edge-id deduplication).
ring = RailNetwork()
vertices = [ring.add_node(Vec3(100 * math.cos(i * math.pi / 3),
                                  100 * math.sin(i * math.pi / 3), 0)) for i in range(6)]
lap = [(ring.add_edge(vertices[i], vertices[(i + 1) % 6]).edge_id, 1) for i in range(6)]
length = sum(ring.edges[eid].length for eid, _ in lap)
future = lap + [lap[0]]
state = OccupancyState(lap + lap, 0, length + 20, future)
trimmed = trim_occupied_ahead_of_head(ring, state)
assert trimmed.occupied == lap + [lap[0]]
assert trimmed.route == future and trimmed.s == state.s and trimmed.occupied_offset == 0
assert state.occupied == lap + lap, "normalization must not mutate the old snapshot"
boundary = trim_occupied_ahead_of_head(ring, OccupancyState(lap, 0, ring.edges[lap[0][0]].length, []))
assert boundary.occupied == lap[:2], "node boundary agrees with PathKinematics"
print("✅ 路线接入契约：历史余量/手动 goto/合法二次折返/跨多边自动折返/环线 occurrence/节点边界")
