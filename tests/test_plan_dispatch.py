"""P3b：控制车计划投影、到达步进与零寻路回归。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from model.block import BlockManager
from model.dispatch import TrainDispatcher
from model.occupancy import OccupancyState
from model.plan import END_FRONT, END_REAR, FixedRoute, Plan, PlanItem, TrainRef
from model.plan_dispatch import PlanDispatcher
from model.rail_network import RailNetwork
from model.signal import SignalTable
from model.train_entity import TrainEntity, TrainState
from model.train_physics import SimplePhysics
from model.vec3 import Vec3
from model.wagon import Consist, create_simple_car
from controller.coupling import end_coupler_pos, head_hook_offset
from controller.coupling import find_couple_pair


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

# ⑦ P7：前往连挂只消费冻结路线，并把目标绑定到指定车厢所在的停放编组。
network2 = RailNetwork()
n0 = network2.add_node(Vec3(0.0, 0.0, 0.0))
n1 = network2.add_node(Vec3(100.0, 0.0, 0.0))
couple_edge = network2.add_edge(n0, n1).edge_id
approach_wagon = create_simple_car(
    length=10.0, mass=30.0, P_rated=1000.0, have_control=True,
)
target_wagon = create_simple_car(length=10.0, mass=30.0, P_rated=None)
approach = TrainEntity(
    TrainState(OccupancyState([(couple_edge, 1)], 0.0, 10.0, []), 0.0, 0.0,
               Consist([approach_wagon])),
    network2,
    SimplePhysics(a_max=2.0, b_max=3.0),
)
target = TrainEntity(
    TrainState(OccupancyState([(couple_edge, 1)], 70.0, 10.0, []), 0.0, 0.0,
               Consist([target_wagon])),
    network2,
    SimplePhysics(a_max=2.0, b_max=3.0),
)
couple_route = FixedRoute(((couple_edge, 1),), 10.0, 30.0)
approach_wagon.plan = Plan([
    PlanItem.goto_couple(
        TrainRef(target_wagon.wagon_id, END_REAR), fixed_route=couple_route,
    ),
    PlanItem.wait_couple(),
])
couple_plans = PlanDispatcher(network2)
_target_pos, _target_edge, target_t = end_coupler_pos(target, "tail")
couple_plans.tick(approach, [approach, target])
assert approach.plan_execution is not None
assert approach.couple_approach_partner is target
assert approach.state.goal == (couple_edge, target_t, 1)
expected_remaining = target_t * 100.0 - 10.0 - head_hook_offset(approach)
assert abs(approach.state.remaining_to_goal - expected_remaining) < 1e-9
print("✅ ⑦ 前往连挂投影冻结路线，目标后钩只作末边小范围校正")

# 激活后目标被删除时须立刻停车、解除信号豁免并跳过，不能追逐旧实体。
couple_plans.tick(approach, [approach])
assert approach.is_parked() and approach.couple_approach_partner is None
assert approach_wagon.plan.pointer == 1
approach_wagon.plan.pointer = 0

# ⑧ 目标车厢消失是永久失效；前端与不再暴露的后端只等待并明确报错。
approach.emergency_stop()
approach_wagon.plan = Plan([
    PlanItem.goto_couple(
        TrainRef("missing-wagon", END_REAR), fixed_route=couple_route,
    ),
    PlanItem.wait_couple(),
])
couple_plans.tick(approach, [approach, target])
assert approach_wagon.plan.pointer == 1
assert "等待连挂" in approach.plan_status

approach_wagon.plan = Plan([
    PlanItem.goto_couple(
        TrainRef(target_wagon.wagon_id, END_FRONT), fixed_route=couple_route,
    ),
])
couple_plans.tick(approach, [approach, target])
assert approach_wagon.plan.pointer == 0 and approach.plan_execution is None
assert "不支持" in approach.plan_status

extra_wagon = create_simple_car(length=10.0, mass=30.0, P_rated=None)
target.state.consist = Consist([target_wagon, extra_wagon])
approach_wagon.plan = Plan([
    PlanItem.goto_couple(
        TrainRef(target_wagon.wagon_id, END_REAR), fixed_route=couple_route,
    ),
])
couple_plans.tick(approach, [approach, target])
assert approach_wagon.plan.pointer == 0 and "未暴露" in approach.plan_status
print("✅ ⑧ 目标消失才跳过；不支持端头与换边只等待并报错")

# ⑨ 连挂事件只推进合并后胜出控制车：goto_couple / wait_couple 均越过一次。
target.state.consist = Consist([target_wagon])
approach_wagon.plan = Plan([
    PlanItem.goto_couple(
        TrainRef(target_wagon.wagon_id, END_REAR), fixed_route=couple_route,
    ),
    PlanItem.wait_couple(),
])
target_control = create_simple_car(
    length=10.0, mass=30.0, P_rated=None, have_control=True, priority=-1,
)
target_control.plan = Plan([PlanItem.wait_couple(), forward])
target.state.consist = Consist([target_wagon, target_control])
participants = (
    {approach_wagon.wagon_id},
    {target_wagon.wagon_id, target_control.wagon_id},
)
merged = approach.couple_with(target)
couple_plans.on_coupled(merged, participants)
assert approach_wagon.plan.pointer == 1
assert target_control.plan.pointer == 0

target_control.config.priority = 2
approach_wagon.plan.pointer = 0
merged2 = TrainEntity(
    TrainState(merged.state.occupancy, 0.0, 0.0, merged.state.consist),
    network2,
    SimplePhysics(a_max=2.0, b_max=3.0),
)
couple_plans.on_coupled(merged2, participants)
assert target_control.plan.pointer == 1
assert approach_wagon.plan.pointer == 0
print("✅ ⑨ 合并后重新遴选，只推进胜出计划并越过 wait_couple")

# 同侧引用不能借一次无关连挂事件误推进。
target_control.plan = Plan([
    PlanItem.goto_couple(
        TrainRef(target_wagon.wagon_id, END_REAR), fixed_route=couple_route,
    ),
])
couple_plans.on_coupled(merged2, participants)
assert target_control.plan.pointer == 0
assert "不匹配" in merged2.plan_status
print("✅ ⑩ 前往连挂只接受目标来自合并前对侧的事件")

# ⑪ 端到端：冻结计划驶近 → 物理停车对位 → 配对合并 → 指针推进。
network3 = RailNetwork()
p0 = network3.add_node(Vec3(0.0, 0.0, 0.0))
p1 = network3.add_node(Vec3(120.0, 0.0, 0.0))
physical_edge = network3.add_edge(p0, p1).edge_id
driver = create_simple_car(
    length=10.0, mass=30.0, P_rated=1000.0, have_control=True,
)
receiver = create_simple_car(length=10.0, mass=30.0, P_rated=None)
driving = TrainEntity(
    TrainState(OccupancyState([(physical_edge, 1)], 0.0, 10.0, []), 0.0, 0.0,
               Consist([driver])),
    network3,
    SimplePhysics(a_max=2.0, b_max=3.0),
)
waiting = TrainEntity(
    TrainState(OccupancyState([(physical_edge, 1)], 80.0, 10.0, []), 0.0, 0.0,
               Consist([receiver])),
    network3,
    SimplePhysics(a_max=2.0, b_max=3.0),
)
_pos, _edge, receiver_tail_t = end_coupler_pos(waiting, "tail")
physical_route = FixedRoute(
    ((physical_edge, 1),),
    10.0,
    network3.edges[physical_edge].length * (1.0 - receiver_tail_t),
)
driver.plan = Plan([
    PlanItem.goto_couple(
        TrainRef(receiver.wagon_id, END_REAR), fixed_route=physical_route,
    ),
    PlanItem.wait_couple(),
])
physical_plans = PlanDispatcher(network3)
physical_blocks = BlockManager()
physical_signals = SignalTable()
physical_blocks.rebuild(network3, physical_signals)
physical_dispatch = TrainDispatcher(network3, physical_signals, physical_blocks)
physical_plans.tick(driving, [driving, waiting])
for _ in range(60 * 30):
    physical_dispatch.tick(driving, 1.0 / 60.0, driving.v_target, [driving, waiting])
    if driving.is_parked():
        break
assert driving.is_parked()
match = find_couple_pair(driving, [driving, waiting])
assert match is not None and match.merged_head is waiting and match.merged_rear is driving
groups = (
    {wagon.wagon_id for wagon in match.merged_head.state.consist.wagons},
    {wagon.wagon_id for wagon in match.merged_rear.state.consist.wagons},
)
physical_merged = match.merged_head.couple_with(match.merged_rear)
physical_plans.on_coupled(physical_merged, groups)
assert driver.plan.pointer == 1
assert len(physical_merged.state.consist.wagons) == 2
print("✅ ⑪ 冻结计划完成真实驶近、车钩对位、合并与事件推进")

print("\n全部通过")
