"""P5：最小计划编辑器的门禁、冻结和错误反馈。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from controller.plan_editor import PlanEditor
from model.occupancy import OccupancyState
from model.rail_network import RailNetwork
from model.train_entity import TrainEntity, TrainState
from model.train_physics import SimplePhysics
from model.vec3 import Vec3
from model.wagon import Consist, create_simple_car
from model.plan import Plan, PlanCommand, PlanItem


network = RailNetwork()
a = network.add_node(Vec3(0.0, 0.0, 0.0))
b = network.add_node(Vec3(10.0, 0.0, 0.0))
c = network.add_node(Vec3(20.0, 0.0, 0.0))
x = network.add_node(Vec3(0.0, 20.0, 0.0))
y = network.add_node(Vec3(10.0, 20.0, 0.0))
e0 = network.add_edge(a, b).edge_id
e1 = network.add_edge(b, c).edge_id
isolated = network.add_edge(x, y).edge_id
wagon = create_simple_car(length=5.0, mass=30.0, P_rated=1000.0, have_control=True)
train = TrainEntity(
    TrainState(OccupancyState([(e0, 1)], 0.0, 0.0, []), 0.0, 0.0, Consist([wagon])),
    network,
    SimplePhysics(),
)
editor = PlanEditor(network)

# ① 只有停放且有控制车的列车能进入。
train.assign_route([(e1, 1)], 20.0, (e1, 1.0, 1))
ok, message = editor.enter(train)
assert not ok and "完全停放" in message
train.emergency_stop()
ok, message = editor.enter(train)
assert ok and editor.owner is wagon
print("✅ ① 计划编辑门禁明确拒绝行驶中列车")

# ② 不可达必须保留失败原因，Enter 不得静默创建条目。
message = editor.click_edge(isolated, 0.5)
assert "计划解析失败" in message and editor.draft.failure
ok, message = editor.confirm()
assert not ok and "计划确认失败" in message and wagon.plan is None
print("✅ ② 不可达草稿标红所需原因完整，确认被拒绝")

# ③ 设置可达终点后冻结；第二条以上一条终点为构造起点。
assert "计划终点" in editor.click_edge(e1, 1.0)
ok, message = editor.confirm()
assert ok and wagon.plan is not None and len(wagon.plan) == 1
assert wagon.plan.requires_closed_cycle
assert "缺少末条→第一条" in wagon.plan.validate_cycle(network)[0]
first = wagon.plan.items[0]
assert first.fixed_route is not None and first.fixed_route.edges == ((e0, 1), (e1, 1))
assert "计划终点" in editor.click_edge(e0, 0.0)
ok, message = editor.confirm()
assert ok and len(wagon.plan) == 2
second = wagon.plan.items[1]
assert second.fixed_route is not None
assert second.fixed_route.edges[0] == (e1, 1)
assert second.fixed_route.edges[-1] == (e0, -1)
ok, message = editor.finalize_cycle()
assert ok, message
assert wagon.plan.loop_route is not None
assert wagon.plan.loop_route.edges[0] == (e0, -1)
assert wagon.plan.loop_route.edges[-1] == (e1, 1)
print("✅ ③ 连续确认两条冻结路线，并冻结末目标返回首目标的循环接缝")

# ④ 回退顺序先终点、后锚点；取消清空草稿但保留已确认计划。
assert "锚点" in editor.click_node(b.node_id)
assert "终点" in editor.click_edge(e1, 0.5)
assert "终点" in editor.backspace()
assert editor.draft.goal is None and len(editor.draft.anchors) == 1
assert "锚点" in editor.backspace()
editor.cancel()
assert not editor.active and len(wagon.plan) == 2
print("✅ ④ Backspace 分层回退，Esc 语义只丢草稿")

# ⑤ 退出门禁：各条目本身可达但 n→1 不可达时，闭环确认必须失败。
one_way_wagon = create_simple_car(
    length=5.0, mass=30.0, P_rated=1000.0, have_control=True,
)
one_way_train = TrainEntity(
    TrainState(OccupancyState([(e0, 1)], 0.0, 0.0, []), 0.0, 0.0,
               Consist([one_way_wagon])),
    network,
    SimplePhysics(),
)
one_way_editor = PlanEditor(network, passable_fn=lambda _edge, direction: direction > 0)
assert one_way_editor.enter(one_way_train)[0]
assert "计划终点" in one_way_editor.click_edge(e0, 0.5)
assert one_way_editor.confirm()[0]
assert "计划终点" in one_way_editor.click_edge(e1, 1.0)
assert one_way_editor.confirm()[0]
ok, message = one_way_editor.finalize_cycle()
assert not ok and "计划闭环失败" in message
assert one_way_editor.active and one_way_wagon.plan.loop_route is None
print("✅ ⑤ n→1 不可达时拒绝退出，计划不会以开放链开始执行")

# ⑥ P7 最小入口：K 冻结驶向目标后钩，W 直接追加零速等待。
wagon.plan = None
ok, _message = editor.enter(train)
assert ok
target_wagon = create_simple_car(length=5.0, mass=30.0, P_rated=None)
target = TrainEntity(
    TrainState(OccupancyState([(e1, 1)], 0.0, 5.0, []), 0.0, 0.0,
               Consist([target_wagon])),
    network,
    SimplePhysics(),
)
ok, message = editor.append_goto_couple(target, "head")
assert ok and wagon.plan is not None and len(wagon.plan) == 1
couple_item = wagon.plan.items[0]
assert couple_item.command is PlanCommand.GOTO_COUPLE
assert couple_item.train_ref is None
assert couple_item.couple_selector is not None
assert couple_item.couple_selector.edge_id == e1
assert couple_item.fixed_route is not None
assert couple_item.couple_selector.direction == couple_item.fixed_route.edges[-1][1]
ok, message = editor.append_wait_couple()
assert ok and len(wagon.plan) == 2
assert wagon.plan.items[1].command is PlanCommand.WAIT_COUPLE
ok, message = editor.finalize_cycle()
assert ok and "循环执行" in message
assert wagon.plan.repeat and not wagon.plan.requires_closed_cycle
assert wagon.plan.loop_route is None
wagon.plan.advance()
wagon.plan.advance()
assert not wagon.plan.is_complete and wagon.plan.current() is couple_item
print("✅ ⑥ 编辑态 K/W 创建固定-edge连挂链，计划可回绕")

# P7c：允许用本编组暴露端头所在 edge 预建未来 selector，不绑定本车厢。
wagon.plan = None
editor.cancel()
ok, _message = editor.enter(train)
assert ok
ok, message = editor.append_goto_couple(train, "head")
assert ok, message
assert wagon.plan is not None and len(wagon.plan) == 1
own_selector = wagon.plan.items[0].couple_selector
assert own_selector is not None
assert wagon.plan.items[0].train_ref is None
assert own_selector.direction in (1, -1)
print("✅ ⑥b 本编组暴露端可用于预建 selector，计划不绑定当前车厢")

# P7c UX：不依赖受保护的本车端头，直接在目标 edge 上按 K 预建 selector。
wagon.plan = None
ok, message = editor.append_goto_couple_edge(e1, target_t=0.5)
assert ok, message
edge_selector = wagon.plan.items[0].couple_selector
assert edge_selector is not None and edge_selector.edge_id == e1
assert edge_selector.direction in (1, -1)
print("✅ ⑥c 直接选择 edge 创建 selector，支持目标仍在本编组时的 headshunt")

# ⑦ 最小管理能力：编辑态可删除当前条目或清空计划；不在数据层隐式重算路线。
wagon.plan = Plan([PlanItem.wait_couple(), PlanItem.wait_couple()], repeat=False)
ok, message = editor.remove_current()
assert ok and "已删除第 1 条" in message and len(wagon.plan) == 1
ok, message = editor.clear_plan()
assert ok and wagon.plan is None
print("✅ ⑦ 编辑态 Delete / Ctrl+Delete 所需的删除当前条目和清空计划能力")

print("\n全部通过")
