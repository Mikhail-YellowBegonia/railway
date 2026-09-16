"""P3b：控制车计划投影、到达步进与零寻路回归。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from model.block import BlockManager
from model.dispatch import TrainDispatcher
from model.occupancy import OccupancyState
from model.plan import FixedRoute, Plan, PlanItem
from model.plan_dispatch import PlanDispatcher
from model.rail_network import RailNetwork
from model.signal import SignalTable
from model.train_entity import TrainEntity, TrainState
from model.train_physics import SimplePhysics
from model.vec3 import Vec3
from model.wagon import Consist, create_simple_car


def make_line():
    network = RailNetwork()
    a = network.add_node(Vec3(0.0, 0.0, 0.0))
    b = network.add_node(Vec3(10.0, 0.0, 0.0))
    c = network.add_node(Vec3(20.0, 0.0, 0.0))
    e0 = network.add_edge(a, b).edge_id
    e1 = network.add_edge(b, c).edge_id
    return network, e0, e1


network, e0, e1 = make_line()
wagon = create_simple_car(length=5.0, mass=30.0, P_rated=1000.0, have_control=True)
forward = PlanItem.goto(
    (e1, 1.0, 1), fixed_route=FixedRoute(((e0, 1), (e1, 1)), 0.0, 0.0),
)
backward = PlanItem.goto(
    (e0, 0.0, -1),
    fixed_route=FixedRoute(((e1, 1), (e1, -1), (e0, -1)), 10.0, 0.0),
)
wagon.plan = Plan([forward, backward])
train = TrainEntity(
    TrainState(OccupancyState([(e0, 1)], 0.0, 0.0, []), 0.0, 0.0, Consist([wagon])),
    network,
    SimplePhysics(a_max=2.0, b_max=3.0),
)
plans = PlanDispatcher(network)
signals = SignalTable()
blocks = BlockManager()
blocks.rebuild(network, signals)
movement = TrainDispatcher(network, signals, blocks)

# ① 运行期只投影冻结路径，首条正向 goto 到站后步进到反向条目。
plans.tick(train)
assert train.state.occupancy.route == [(e1, 1)]
assert train.state.goal == (e1, 1.0, 1)
# 临时手动指令覆盖时，计划令牌须失效，不能在行驶中抢回既有 route。
train.assign_route([], 5.0, (e0, 0.5, 1))
plans.tick(train)
assert train.plan_execution is None and train.state.goal == (e0, 0.5, 1)
train.emergency_stop()
plans.tick(train)
assert train.plan_execution is not None and train.state.goal == (e1, 1.0, 1)
print("✅ ① 临时行车覆盖会解除计划令牌，停放后才可重新激活")
for _ in range(60 * 30):
    movement.tick(train, 1.0 / 60.0, train.v_target, [train])
    if train.is_parked():
        break
assert train.is_parked() and train.state.goal == (e1, 1.0, 1)
plans.on_arrival(train)
assert wagon.plan.pointer == 1
print("✅ ② 固定路线一次性投影，到站只步进一次")

# ② 反向固定路线含死端折返；执行层消费已有 route，不调用 P2/Dijkstra。
plans.tick(train)
assert train.state.occupancy.route == [(e1, -1), (e0, -1)]
for _ in range(60 * 45):
    movement.tick(train, 1.0 / 60.0, train.v_target, [train])
    if train.is_parked():
        break
assert train.is_parked() and train.state.goal == (e0, 0.0, -1)
plans.on_arrival(train)
assert wagon.plan.pointer == 0
print("✅ ③ 死端折返后消费原冻结路径并回绕，不重新寻路")

# ③ 不能从当前车头消费固定路线时安全等待，既不改 route 也不掉头补路。
wrong_start = PlanItem.goto(
    (e1, 1.0, 1), fixed_route=FixedRoute(((e1, 1),), 0.0, 0.0),
)
wagon.plan = Plan([wrong_start])
plans.tick(train)
assert train.is_parked() and train.state.occupancy.route == []
assert train.current_edge_and_t()[0] == e0
print("✅ ④ 固定路线起点不匹配时安全等待，不重新寻路")

# ④ 永久失效条目只在一圈内跳过，随后激活可执行条目。
valid_again = PlanItem.goto(
    (e1, 1.0, 1), fixed_route=FixedRoute(((e0, -1), (e0, 1), (e1, 1)), 10.0, 0.0),
)
wagon.plan = Plan([
    PlanItem.goto((999, 1.0, 1), fixed_route=FixedRoute(((999, 1),), 0.0, 0.0)),
    valid_again,
])
plans.tick(train)
assert wagon.plan.pointer == 1
assert train.state.occupancy.route == [(e0, 1), (e1, 1)]
print("✅ ⑤ 永久失效条目跳过，计划保留并在一圈内继续")

# ⑤ 优先级高者胜；相同优先级由 wagon_id 的稳定字典序打破平手。
low = create_simple_car(length=5.0, mass=30.0, P_rated=1000.0, have_control=True, priority=1)
high = create_simple_car(length=5.0, mass=30.0, P_rated=1000.0, have_control=True, priority=2)
low.plan = Plan([forward])
high.plan = Plan([forward])
train.state.consist = Consist([low, high])
assert plans._winner(train) is high
high.plan = None
train.plan_execution = None
plans.tick(train)
assert train.plan_execution is None
assert "没有计划" in train.plan_status
high.plan = Plan([forward])
low.config.priority = high.config.priority
expected = min((low, high), key=lambda item: item.wagon_id)
assert plans._winner(train) is expected
print("✅ ⑥ 控制车稳定遴选；胜出者无计划时报错且不降级")

print("\n全部通过")
