"""动态 goto_couple 回归：计划不保存路线，激活从当前车头解析。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from model.occupancy import OccupancyState
from model.plan import Plan, PlanItem
from model.plan_dispatch import PlanDispatcher
from model.rail_network import RailNetwork
from model.train_entity import TrainEntity, TrainState
from model.train_physics import SimplePhysics
from model.vec3 import Vec3
from model.wagon import Consist, create_simple_car


network = RailNetwork()
n0 = network.add_node(Vec3(0.0, 0.0, 0.0))
n1 = network.add_node(Vec3(100.0, 0.0, 0.0))
n2 = network.add_node(Vec3(200.0, 0.0, 0.0))
n3 = network.add_node(Vec3(300.0, 0.0, 0.0))
e0 = network.add_edge(n0, n1).edge_id
e1 = network.add_edge(n1, n2).edge_id
e2 = network.add_edge(n2, n3).edge_id


def parked(edge_id: int, s: float, wagon):
    return TrainEntity(
        TrainState(
            OccupancyState([(edge_id, 1)], 0.0, s, []),
            0.0, 0.0, Consist([wagon]),
        ),
        network,
        SimplePhysics(),
    )


driver_wagon = create_simple_car(length=10.0, mass=20.0, P_rated=1000.0, have_control=True)
target_wagon = create_simple_car(length=10.0, mass=20.0, P_rated=None)
driver = parked(e0, 20.0, driver_wagon)
target = parked(e2, 60.0, target_wagon)
driver_wagon.plan = Plan([PlanItem.goto_couple(edge_id=e2, direction=None)])

dispatcher = PlanDispatcher(network)
dispatcher.tick(driver, [driver, target])
execution = driver.plan_execution
assert execution is not None
assert driver_wagon.plan.items[0].fixed_route is None
assert execution.resolved_route is not None
assert execution.resolved_route.edges[0] == (e0, 1)
assert driver.state.goal is not None and driver.state.goal[0] == e2

# Simulate the normal post-coupling/reversal situation: the logical head is
# now already on the middle edge, while the same plan item remains active.
# Rebuild kinematics exactly as TrainEntity does after a physical state change.
driver.emergency_stop()
driver.state.occupancy = OccupancyState([(e1, 1)], 0.0, 20.0, [])
driver.kinematics = driver._build_kinematics()
driver.plan_execution = None
dispatcher.tick(driver, [driver, target])
execution = driver.plan_execution
assert execution is not None
assert execution.resolved_route is not None
assert execution.resolved_route.edges[0] == (e1, 1)
assert driver.state.goal is not None and driver.state.goal[0] == e2
assert "固定路线" not in driver.plan_status

print("✅ goto_couple 计划只保存动态 selector；车头跳变后从新 edge 实时寻路")

# The same invariant applies to ordinary goto items.
goto_wagon = create_simple_car(length=10.0, mass=20.0, P_rated=1000.0, have_control=True)
goto_train = parked(e0, 20.0, goto_wagon)
goto_wagon.plan = Plan([PlanItem.goto((e2, 0.6, 1))])
dispatcher.tick(goto_train, [goto_train])
assert goto_wagon.plan.items[0].fixed_route is None
assert goto_train.plan_execution.resolved_route.edges[0] == (e0, 1)
goto_train.emergency_stop()
goto_train.state.occupancy = OccupancyState([(e1, 1)], 0.0, 20.0, [])
goto_train.kinematics = goto_train._build_kinematics()
goto_train.plan_execution = None
dispatcher.tick(goto_train, [goto_train])
assert goto_train.plan_execution.resolved_route.edges[0] == (e1, 1)
print("✅ goto 计划只保存目标；车头跳变后从新 edge 实时寻路")

# An explicit plan reverse already changed the live logical head.  The next
# dynamic goto must not ask Dijkstra to insert a second implicit reversal
# marker, which would make one physical edge appear twice in the route.
reverse_wagon = create_simple_car(length=10.0, mass=20.0, P_rated=1000.0, have_control=True)
reverse_train = parked(e1, 20.0, reverse_wagon)
reverse_wagon.plan = Plan([
    PlanItem.reverse(),
    PlanItem.goto((e0, 0.2, -1)),
])
dispatcher.tick(reverse_train, [reverse_train])
assert reverse_wagon.plan.pointer == 1
assert reverse_train.current_direction() == -1
dispatcher.tick(reverse_train, [reverse_train])
resolved = reverse_train.plan_execution.resolved_route
assert resolved is not None
assert all(
    resolved.edges[index][0] != resolved.edges[index + 1][0]
    or resolved.edges[index][1] == resolved.edges[index + 1][1]
    for index in range(len(resolved.edges) - 1)
), resolved.edges
assert reverse_train.state.occupancy.route == [(e0, -1)]
print("✅ 显式 reverse 后动态 goto 不再隐式重复消费同一 edge")
