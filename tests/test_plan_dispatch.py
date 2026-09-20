"""P3b：控制车计划投影、到达步进与零寻路回归。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from model.block import BlockManager
from model.dispatch import TrainDispatcher
from model.occupancy import OccupancyState
from model.plan import END_FRONT, END_REAR, FixedRoute, Plan, PlanItem, TrainRef
from model.plan_dispatch import PLAN_CRUISE_SPEED, PlanDispatcher
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
wagon.plan = Plan(
    [forward, backward],
    loop_route=FixedRoute(
        ((e0, -1), (e0, 1), (e1, 1)), 10.0, 0.0,
    ),
    requires_closed_cycle=True,
)
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

# 回绕后的第三段必须消费显式 n→1 接缝：先在 e0 死端折返，再返回第一目标。
plans.tick(train)
assert train.plan_execution is not None
assert train.state.occupancy.route == [(e0, 1), (e1, 1)]
assert train.state.goal == forward.goal
train.emergency_stop()
train.plan_execution = None
print("✅ ③b 无环线路回绕后下发显式 n→1 死端折返路线")

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

# ⑫ 节点边界有两种等价表示：入边末端与出边起点。制动停在节点前几厘米时，
# 下一条固定路径可从合法出边继续；不切边、不改位置、不重新寻路。
network4 = RailNetwork()
q0 = network4.add_node(Vec3(0.0, 0.0, 0.0))
q1 = network4.add_node(Vec3(10.0, 0.0, 0.0))
q2 = network4.add_node(Vec3(20.0, 0.0, 0.0))
incoming = network4.add_edge(q0, q1).edge_id
outgoing = network4.add_edge(q1, q2).edge_id
handoff_wagon = create_simple_car(
    length=5.0, mass=30.0, P_rated=1000.0, have_control=True,
)
handoff_wagon.plan = Plan([
    PlanItem.goto(
        (outgoing, 1.0, 1),
        fixed_route=FixedRoute(((outgoing, 1),), 0.0, 0.0),
    ),
])
handoff = TrainEntity(
    TrainState(OccupancyState([(incoming, 1)], 4.96, 5.0, []), 0.0, 0.0,
               Consist([handoff_wagon])),
    network4,
    SimplePhysics(a_max=2.0, b_max=3.0),
)
handoff_plans = PlanDispatcher(network4)
handoff_plans.tick(handoff, [handoff])
assert handoff.state.occupancy.route == [(outgoing, 1)]
assert abs(handoff.state.remaining_to_goal - 10.04) < 1e-9
assert set(network4.edges) == {incoming, outgoing}

# 逆向行驶使用同一契约，t 与有向弧长换算不得颠倒。
reverse_wagon = create_simple_car(
    length=5.0, mass=30.0, P_rated=1000.0, have_control=True,
)
reverse_wagon.plan = Plan([
    PlanItem.goto(
        (incoming, 0.0, -1),
        fixed_route=FixedRoute(((incoming, -1),), 0.0, 0.0),
    ),
])
reverse_handoff = TrainEntity(
    TrainState(OccupancyState([(outgoing, -1)], 4.96, 5.0, []), 0.0, 0.0,
               Consist([reverse_wagon])),
    network4,
    SimplePhysics(a_max=2.0, b_max=3.0),
)
handoff_plans.tick(reverse_handoff, [reverse_handoff])
assert reverse_handoff.state.occupancy.route == [(incoming, -1)]
assert abs(reverse_handoff.state.remaining_to_goal - 10.04) < 1e-9

# 同一节点但非法急折仍必须拒绝，不能借“位置等价”绕过转向约束。
sharp_network = RailNetwork()
r0 = sharp_network.add_node(Vec3(0.0, 0.0, 0.0))
r1 = sharp_network.add_node(Vec3(10.0, 0.0, 0.0))
r2 = sharp_network.add_node(Vec3(10.0, 10.0, 0.0))
sharp_in = sharp_network.add_edge(r0, r1).edge_id
sharp_out = sharp_network.add_edge(r1, r2).edge_id
assert not sharp_network.turn_allowed(r1.node_id, sharp_in, sharp_out)
sharp_wagon = create_simple_car(
    length=5.0, mass=30.0, P_rated=1000.0, have_control=True,
)
sharp_wagon.plan = Plan([
    PlanItem.goto(
        (sharp_out, 1.0, 1),
        fixed_route=FixedRoute(((sharp_out, 1),), 0.0, 0.0),
    ),
])
sharp_train = TrainEntity(
    TrainState(OccupancyState([(sharp_in, 1)], 4.96, 5.0, []), 0.0, 0.0,
               Consist([sharp_wagon])),
    sharp_network,
    SimplePhysics(a_max=2.0, b_max=3.0),
)
sharp_plans = PlanDispatcher(sharp_network)
sharp_plans.tick(sharp_train, [sharp_train])
assert sharp_train.state.occupancy.route == []
assert "固定路线可消费位置" in sharp_train.plan_status
print("✅ ⑫ 节点边界停车误差可拓扑接续，非法转向仍被拒绝")

# ⑬ 完整时序：上一条实际制动停在入边末端前，再由到达事件激活出边计划。
network5 = RailNetwork()
s0 = network5.add_node(Vec3(0.0, 0.0, 0.0))
s1 = network5.add_node(Vec3(100.0, 0.0, 0.0))
s2 = network5.add_node(Vec3(200.0, 0.0, 0.0))
first_edge = network5.add_edge(s0, s1).edge_id
second_edge = network5.add_edge(s1, s2).edge_id
sequence_wagon = create_simple_car(
    length=10.0, mass=30.0, P_rated=1000.0, have_control=True,
)
sequence_wagon.plan = Plan([
    PlanItem.goto(
        (first_edge, 1.0, 1),
        fixed_route=FixedRoute(((first_edge, 1),), 20.0, 0.0),
    ),
    PlanItem.goto(
        (second_edge, 1.0, 1),
        fixed_route=FixedRoute(((second_edge, 1),), 0.0, 0.0),
    ),
])
sequence_train = TrainEntity(
    TrainState(OccupancyState([(first_edge, 1)], 10.0, 10.0, []), 0.0, 0.0,
               Consist([sequence_wagon])),
    network5,
    SimplePhysics(a_max=2.0, b_max=3.0),
)
sequence_plans = PlanDispatcher(network5)
sequence_blocks = BlockManager()
sequence_signals = SignalTable()
sequence_blocks.rebuild(network5, sequence_signals)
sequence_dispatch = TrainDispatcher(
    network5, sequence_signals, sequence_blocks,
)
sequence_plans.tick(sequence_train, [sequence_train])
for _ in range(60 * 30):
    sequence_dispatch.tick(
        sequence_train, 1.0 / 60.0, sequence_train.v_target, [sequence_train],
    )
    if sequence_train.is_parked():
        break
assert sequence_train.is_parked()
stopped_edge, stopped_t = sequence_train.current_edge_and_t()
assert stopped_edge == first_edge and 0.0 < (1.0 - stopped_t) * 100.0 <= 0.1
sequence_plans.on_arrival(sequence_train)
assert sequence_wagon.plan.pointer == 1
sequence_plans.tick(sequence_train, [sequence_train])
assert sequence_train.plan_execution is not None
assert sequence_train.state.occupancy.route == [(second_edge, 1)]
assert sequence_train.state.remaining_to_goal > 100.0
print("✅ ⑬ 实际制动停车后，下一条相邻出边计划可连续激活")

# ⑭ 循环接缝：第一条固定路线的 edge 序列仍有效时，回到其首边的另一 t
# 可直接从实际位置消费，不能要求回到建表时的历史 start_offset。
network6 = RailNetwork()
u0 = network6.add_node(Vec3(0.0, 0.0, 0.0))
u1 = network6.add_node(Vec3(100.0, 0.0, 0.0))
u2 = network6.add_node(Vec3(200.0, 0.0, 0.0))
loop_start_edge = network6.add_edge(u0, u1).edge_id
loop_goal_edge = network6.add_edge(u1, u2).edge_id
loop_wagon = create_simple_car(
    length=10.0, mass=30.0, P_rated=1000.0, have_control=True,
)
loop_wagon.plan = Plan([
    PlanItem.goto(
        (loop_goal_edge, 1.0, 1),
        fixed_route=FixedRoute(
            ((loop_start_edge, 1), (loop_goal_edge, 1)), 20.0, 0.0,
        ),
    ),
])
loop_train = TrainEntity(
    TrainState(OccupancyState([(loop_start_edge, 1)], 30.0, 10.0, []), 0.0, 0.0,
               Consist([loop_wagon])),
    network6,
    SimplePhysics(a_max=2.0, b_max=3.0),
)
loop_plans = PlanDispatcher(network6)
loop_plans.tick(loop_train, [loop_train])
assert loop_train.state.occupancy.route == [(loop_goal_edge, 1)]
assert abs(loop_train.state.remaining_to_goal - 160.0) < 1e-9

# 若实际位置已越过单边目标，仍须拒绝，不能沿固定方向倒着补路。
past_goal_wagon = create_simple_car(
    length=10.0, mass=30.0, P_rated=1000.0, have_control=True,
)
past_goal_wagon.plan = Plan([
    PlanItem.goto(
        (loop_start_edge, 0.5, 1),
        fixed_route=FixedRoute(((loop_start_edge, 1),), 20.0, 50.0),
    ),
])
past_goal = TrainEntity(
    TrainState(OccupancyState([(loop_start_edge, 1)], 60.0, 10.0, []), 0.0, 0.0,
               Consist([past_goal_wagon])),
    network6,
    SimplePhysics(a_max=2.0, b_max=3.0),
)
loop_plans.tick(past_goal, [past_goal])
assert past_goal.state.occupancy.route == [] and past_goal.plan_execution is None
assert "固定路线可消费位置" in past_goal.plan_status
print("✅ ⑭ 循环回到首边不同 t 可续行，越过目标仍拒绝")

# ⑮ 实机同类：计划回绕时车头已在冻结路线内部 edge，不要求返回首 edge。
v0 = network6.add_node(Vec3(300.0, 0.0, 0.0))
internal_edge = network6.add_edge(u2, v0).edge_id
internal_wagon = create_simple_car(
    length=10.0, mass=30.0, P_rated=1000.0, have_control=True,
)
internal_wagon.plan = Plan([
    PlanItem.goto(
        (internal_edge, 1.0, 1),
        fixed_route=FixedRoute(
            ((loop_start_edge, 1), (loop_goal_edge, 1), (internal_edge, 1)),
            0.0,
            0.0,
        ),
    ),
])
internal_train = TrainEntity(
    TrainState(OccupancyState([(loop_goal_edge, 1)], 30.0, 10.0, []), 0.0, 0.0,
               Consist([internal_wagon])),
    network6,
    SimplePhysics(a_max=2.0, b_max=3.0),
)
loop_plans.tick(internal_train, [internal_train])
assert internal_train.state.occupancy.route == [(internal_edge, 1)]
assert abs(internal_train.state.remaining_to_goal - 160.0) < 1e-9
print("✅ ⑮ 可从冻结路径内部有向 edge 重定位并消费后缀")

# ⑯ 两目标环线的回绕必须消费独立闭环路径，不能重用建表位置到第一目标的路线。
ring_wagon = create_simple_car(
    length=10.0, mass=30.0, P_rated=1000.0, have_control=True,
)
bootstrap = FixedRoute(((loop_start_edge, -1),), 0.0, 50.0)
return_to_start = FixedRoute(
    ((internal_edge, -1), (loop_goal_edge, -1), (loop_start_edge, -1)), 0.0, 50.0,
)
ring_wagon.plan = Plan(
    [
        PlanItem.goto((loop_start_edge, 0.5, -1), fixed_route=bootstrap),
        PlanItem.goto(
            (internal_edge, 1.0, 1),
            fixed_route=FixedRoute(
                ((loop_start_edge, -1), (loop_start_edge, 1),
                 (loop_goal_edge, 1), (internal_edge, 1)),
                50.0,
                0.0,
            ),
        ),
    ],
    pointer=0,
    loop_route=return_to_start,
    has_wrapped=True,
)
ring_train = TrainEntity(
    TrainState(OccupancyState([(internal_edge, -1)], 0.0, 10.0, []), 0.0, 0.0,
               Consist([ring_wagon])),
    network6,
    SimplePhysics(a_max=2.0, b_max=3.0),
)
loop_plans.tick(ring_train, [ring_train])
assert ring_train.plan_execution is not None
assert ring_train.state.occupancy.route == [
    (loop_goal_edge, -1), (loop_start_edge, -1),
]
assert ring_train.state.goal == (loop_start_edge, 0.5, -1)
print("✅ ⑯ 回绕第一条时使用末目标→第一目标的独立冻结接缝")

# ⑰ GUI 手动巡航值可能每帧写回 0；计划执行期间必须重新取得速度控制权。
# 若列车因此曾在目标前停成 parked，也应从冻结路线当前后缀恢复，不能静默卡住。
speed_wagon = create_simple_car(
    length=10.0, mass=30.0, P_rated=1000.0, have_control=True,
)
speed_wagon.plan = Plan([PlanItem.goto(
    (loop_goal_edge, 1.0, 1),
    fixed_route=FixedRoute(
        ((loop_start_edge, 1), (loop_goal_edge, 1)), 20.0, 0.0,
    ),
)])
speed_train = TrainEntity(
    TrainState(OccupancyState([(loop_start_edge, 1)], 20.0, 10.0, []), 0.0, 0.0,
               Consist([speed_wagon])),
    network6,
    SimplePhysics(a_max=2.0, b_max=3.0),
)
loop_plans.tick(speed_train, [speed_train])
assert speed_train.plan_execution is not None
speed_train.v_target = 0.0  # 模拟 GameLoop 在下一帧写回手动巡航值。
loop_plans.tick(speed_train, [speed_train])
assert speed_train.v_target == PLAN_CRUISE_SPEED

speed_train.emergency_stop()  # 模拟目标前因速度所有权冲突而真正停放并丢 route。
assert speed_train.is_parked() and speed_train.plan_execution is not None
loop_plans.tick(speed_train, [speed_train])
assert speed_train.plan_execution is not None
assert speed_train.state.occupancy.route == [(loop_goal_edge, 1)]
assert speed_train.v_target == PLAN_CRUISE_SPEED
print("✅ ⑰ 计划持续持有巡航速度，目标前意外 parked 可从冻结后缀恢复")

# ⑱ 玩家循环计划没有 n→1 路线时必须在首轮前拒绝执行，不能等回绕后
# 再误用 bootstrap 路线。
unclosed_wagon = create_simple_car(
    length=10.0, mass=30.0, P_rated=1000.0, have_control=True,
)
unclosed_wagon.plan = Plan(
    [PlanItem.goto(
        (loop_goal_edge, 1.0, 1),
        fixed_route=FixedRoute(
            ((loop_start_edge, 1), (loop_goal_edge, 1)), 20.0, 0.0,
        ),
    )],
    requires_closed_cycle=True,
)
unclosed_train = TrainEntity(
    TrainState(OccupancyState([(loop_start_edge, 1)], 20.0, 10.0, []), 0.0, 0.0,
               Consist([unclosed_wagon])),
    network6,
    SimplePhysics(a_max=2.0, b_max=3.0),
)
loop_plans.tick(unclosed_train, [unclosed_train])
assert unclosed_train.plan_execution is None
assert unclosed_train.state.occupancy.route == []
assert "缺少末条→第一条" in unclosed_train.plan_status
print("✅ ⑱ 玩家循环计划缺少 n→1 路线时首轮前明确拒绝执行")

# ⑲ P7 事件链在最后一条推进后必须停在尾后，不得回绕或重新寻路。
complete_wagon = create_simple_car(
    length=10.0, mass=30.0, P_rated=1000.0, have_control=True,
)
complete_wagon.plan = Plan([PlanItem.wait_couple()], repeat=False)
complete_wagon.plan.advance()
complete_train = TrainEntity(
    TrainState(OccupancyState([(loop_start_edge, 1)], 20.0, 10.0, []), 0.0, 0.0,
               Consist([complete_wagon])),
    network6,
    SimplePhysics(a_max=2.0, b_max=3.0),
)
loop_plans.tick(complete_train, [complete_train])
assert complete_wagon.plan.is_complete
assert complete_train.is_parked() and complete_train.state.occupancy.route == []
assert "一次性事件链已消费完毕" in complete_train.plan_status
print("✅ ⑲ P7 一次性事件链完成后停车，不回绕也不寻路")

# ⑳ 固定 edge 连挂：沿进入方向选择第一个可达端头，而非绑定 wagon_id。
edge_net = RailNetwork()
q0 = edge_net.add_node(Vec3(0.0, 0.0, 0.0))
q1 = edge_net.add_node(Vec3(200.0, 0.0, 0.0))
qe = edge_net.add_edge(q0, q1).edge_id
edge_driver = create_simple_car(
    length=10.0, mass=30.0, P_rated=1000.0, have_control=True,
)
edge_driver.plan = Plan([PlanItem.goto_couple(
    edge_id=qe,
    fixed_route=FixedRoute(((qe, 1),), 10.0, 0.0),
)])
edge_train = TrainEntity(
    TrainState(OccupancyState([(qe, 1)], 0.0, 10.0, []), 0.0, 0.0,
               Consist([edge_driver])), edge_net, SimplePhysics(),
)
near_wagon = create_simple_car(length=10.0, mass=30.0)
far_wagon = create_simple_car(length=10.0, mass=30.0)
near_train = TrainEntity(
    TrainState(OccupancyState([(qe, 1)], 50.0, 10.0, []), 0.0, 0.0,
               Consist([near_wagon])), edge_net, SimplePhysics(),
)
far_train = TrainEntity(
    TrainState(OccupancyState([(qe, 1)], 120.0, 10.0, []), 0.0, 0.0,
               Consist([far_wagon])), edge_net, SimplePhysics(),
)
edge_plans = PlanDispatcher(edge_net)
edge_plans.tick(edge_train, [edge_train, far_train, near_train])
assert edge_train.plan_execution is not None
assert edge_train.plan_execution.target_wagon_id == near_wagon.wagon_id
assert edge_train.couple_approach_partner is near_train
print("✅ ⑳ 固定 edge 连挂按进入方向选择第一个可达端头")

# 反向进入同一 edge 时应从 node_b 一侧扫描，即优先选择 t 更大的端头。
reverse_driver = create_simple_car(
    length=10.0, mass=30.0, P_rated=1000.0, have_control=True,
)
reverse_driver.plan = Plan([PlanItem.goto_couple(
    edge_id=qe,
    fixed_route=FixedRoute(((qe, -1),), 10.0, 0.0),
)])
reverse_train = TrainEntity(
    TrainState(OccupancyState([(qe, -1)], 0.0, 10.0, []), 0.0, 0.0,
               Consist([reverse_driver])), edge_net, SimplePhysics(),
)
edge_plans.tick(reverse_train, [reverse_train, near_train, far_train])
assert reverse_train.plan_execution is not None
assert reverse_train.plan_execution.target_wagon_id == far_wagon.wagon_id
assert reverse_train.couple_approach_partner is far_train
print("✅ ⑳b 固定 edge 反向进入时从高 t 端选择第一个可达端头")

# 激活后锁定端头；运行中若出现一个更靠前的候选，必须停车等待而非换目标。
inserted_wagon = create_simple_car(length=10.0, mass=30.0)
inserted_train = TrainEntity(
    TrainState(OccupancyState([(qe, 1)], 25.0, 10.0, []), 0.0, 0.0,
               Consist([inserted_wagon])), edge_net, SimplePhysics(),
)
edge_plans.tick(edge_train, [edge_train, inserted_train, near_train, far_train])
assert edge_train.plan_execution is None
assert edge_train.couple_approach_partner is None
assert "移动" in edge_train.plan_status or "重新确认" in edge_train.plan_status
print("✅ ⑳c 固定 edge 端头激活后锁定，候选变化不会静默切换目标")

# ㉑ 动作条目：折返立即步进；解挂排入延迟动作，成功后循环回绕。
action_wagons = [
    create_simple_car(length=10.0, mass=30.0, P_rated=1000.0,
                      have_control=(i == 0))
    for i in range(3)
]
action_wagons[0].plan = Plan([PlanItem.reverse(), PlanItem.decouple(1)])
action_train = TrainEntity(
    TrainState(OccupancyState([(qe, 1)], 0.0, 60.0, []), 0.0, 0.0,
               Consist(action_wagons)), edge_net, SimplePhysics(),
)
edge_plans.tick(action_train, [action_train])
assert action_wagons[0].plan.pointer == 1 and action_train.current_direction() == -1
edge_plans.tick(action_train, [action_train])
actions = edge_plans.take_decouple_actions()
assert len(actions) == 1 and actions[0].after == 1
edge_plans._decouple_actions = actions
from controller.game_loop import GameLoop
loop = GameLoop.__new__(GameLoop)
loop.plan_dispatcher = edge_plans
loop.trains = [action_train]
loop.active_train = action_train
loop._execute_plan_decouples()
assert actions[0].plan.pointer == 0
assert len(loop.trains) == 2
assert action_wagons[0] in loop.active_train.state.consist.wagons
print("✅ ㉑ 计划折返与‘车头后第 n 位解挂’由 GameLoop 原子执行并循环步进")

print("\n全部通过")
