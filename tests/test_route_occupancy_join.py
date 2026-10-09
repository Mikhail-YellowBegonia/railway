"""Regression: coupled train reverses across a node, then consumes E16 twice.

The occupied window has a coarse tail margin; its old tail is on E16, while
the real tail bogie is already on E18. Reversal turns that margin into an
untravelled edge ahead of the logical head. A freshly resolved goto includes
E16 too. Verify every bogie remains continuous through both edge boundaries.
"""
import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

from model.occupancy import OccupancyState
from model.pathfinding import head_node, tail_node
from model.plan import Plan, PlanItem
from model.plan_dispatch import PlanDispatcher
from model.rail_network import RailNetwork
from model.train_entity import TrainEntity, TrainState
from model.train_physics import SimplePhysics
from model.vec3 import Vec3
from model.wagon import Consist, create_simple_car


network = RailNetwork()
node15 = network.add_node(Vec3(0, 0, 0))
node16 = network.add_node(Vec3(50, 0, 0))
far = network.add_node(Vec3(152.645691121938, 0, 0))
west = network.add_node(Vec3(-25.6614227804845, 0, 0))
goal_node = network.add_node(Vec3(-125.6614227804845, 0, 0))
network._next_edge_id = 16
e16 = network.add_edge(node15, node16).edge_id
network._next_edge_id = 18
e18 = network.add_edge(node16, far).edge_id
network._next_edge_id = 68
e68 = network.add_edge(node15, west).edge_id
e69 = network.add_edge(west, goal_node).edge_id


def cars(control=False):
    return [create_simple_car(length=20, mass=20, P_rated=1000 if control and i == 0 else None,
                              have_control=control and i == 0) for i in range(2)]


def parked(head, wagons):
    return TrainEntity(TrainState(OccupancyState([(e16, 1), (e18, 1)], 0, head, []),
                                  0, 0, Consist(wagons)), network, SimplePhysics())


def bogies_by_wagon(train):
    return {wagon.wagon_id: (front.position, rear.position)
            for wagon, (front, rear) in zip(train.state.consist.wagons,
                train.kinematics.get_all_bogie_poses(train.state.s))}


def assert_continuous(before, after, maximum):
    for wagon_id, positions in before.items():
        # Logical front/rear swap on reversal; pair by the closest position.
        for position in positions:
            distance = min(position.distance_to(p) for p in after[wagon_id])
            assert distance < maximum, (wagon_id, distance, position, after[wagon_id])


front = parked(127.68896399018, cars(control=True))
rear = parked(87.68896399018, cars())
driver = front.state.consist.control_winner()
driver.plan = Plan([PlanItem.reverse(), PlanItem.goto((e69, .5, 1))])
before_merge = {**bogies_by_wagon(front), **bogies_by_wagon(rear)}
train = front.couple_with(rear)
assert_continuous(before_merge, bogies_by_wagon(train), 1e-6)
assert train.kinematics.real_tail_bogie_abs_s(train.state.s) > 50
plans = PlanDispatcher(network)
plans.tick(train, [train])
assert driver.plan.pointer == 1
assert train.current_directed_edge_and_t()[0] == e18
assert train.current_direction() == -1
assert_continuous(before_merge, bogies_by_wagon(train), 1e-6)

plans.tick(train, [train])
assert train.plan_execution.resolved_route.edges == ((e18, -1), (e16, -1), (e68, 1), (e69, 1))
assert train.state.occupancy.route == [(e16, -1), (e68, 1), (e69, 1)]
before_motion = bogies_by_wagon(train)
assert_continuous(before_merge, before_motion, 1e-6)

visited = set()
for _ in range(60 * 40):
    train.update(1 / 60, 12)
    occupied = train.state.occupancy.occupied
    for a, b in zip(occupied, occupied[1:]):
        assert head_node(network, a) == tail_node(network, b), occupied
        assert a != b, occupied
    after_motion = bogies_by_wagon(train)
    assert_continuous(before_motion, after_motion, .3)
    before_motion = after_motion
    visited.add(train.current_directed_edge_and_t()[0])
    if train.is_parked():
        break
assert train.is_parked()
assert visited == {e18, e16, e68, e69}, visited
head = train.kinematics._path_kin.pose_at(train.state.abs_s).position
assert abs(head.x - (-75.6614227804845)) < .2, head
print("✅ 两列连挂→跨节点余量折返→动态 goto：E16 只走一次，所有转向架连续且到站正确")
