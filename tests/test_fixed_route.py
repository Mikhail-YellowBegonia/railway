"""P3a 固定路径模型回归。

只验证确认后的数据契约；不创建 Wagon、不接 GameLoop/dispatch，确保 P3a 本身没有
用户可见行为。P3b 只能消费本文件所验证的 FixedRoute，不得重新调用 P2/Dijkstra。
"""

from dataclasses import FrozenInstanceError
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from model.plan import FixedRoute, Plan, PlanCommand, PlanItem
from model.plan_path import PathStart, resolve_plan_item
from model.rail_network import RailNetwork
from model.vec3 import Vec3


def build_line() -> tuple[RailNetwork, int, int, int]:
    network = RailNetwork()
    a = network.add_node(Vec3(0.0, 0.0, 0.0))
    b = network.add_node(Vec3(100.0, 0.0, 0.0))
    c = network.add_node(Vec3(200.0, 0.0, 0.0))
    e_ab = network.add_edge(a, b).edge_id
    e_bc = network.add_edge(b, c).edge_id
    return network, e_ab, e_bc, b.node_id


network, e_ab, e_bc, b_id = build_line()
goal = (e_bc, 0.6, 1)
draft = PlanItem.goto(goal)
resolution = resolve_plan_item(
    network, PathStart(e_ab, 0.2, 1), draft,
)
assert resolution.ok and resolution.path is not None, resolution.failure
fixed = resolution.path.freeze()

# ① P2 只构造一次，冻结对象包含完整路径与偏移，不持有解析器状态。
assert fixed.edges == ((e_ab, 1), (e_bc, 1))
assert abs(fixed.remaining_to_goal(network) - 140.0) < 1e-9
assert fixed.validate(network, goal=goal) == []
try:
    fixed.edges = ()  # type: ignore[misc]
    raise AssertionError("FixedRoute 必须 frozen")
except FrozenInstanceError:
    pass
print("✅ ① P2 解析结果可冻结为不可变 FixedRoute，距离记账正确")

# ② P3a 允许尚未确认的草稿；P3b 的严格入口必须拒绝它，杜绝运行时回退寻路。
assert draft.validate(network) == []
assert "尚未冻结固定路径" in draft.validate(network, require_fixed_route=True)[0]
confirmed = PlanItem.goto(goal, fixed_route=fixed)
assert confirmed.validate(network, require_fixed_route=True) == []
assert Plan([confirmed]).validate(network, require_fixed_route=True) == []
assert PlanItem(PlanCommand.WAIT_COUPLE, fixed_route=fixed).validate(network)
print("✅ ② 草稿/已确认条目边界明确，严格执行入口只接受 FixedRoute")

# ③ 固定路线必须连续、转向可通、终点方向一致；反向同边只可在死端。
bad_jump = FixedRoute(((e_ab, 1), (e_ab, 1)), 0.0, 0.0)
assert any("不连续" in p for p in bad_jump.validate(network))
bad_goal = FixedRoute(((e_ab, 1), (e_bc, 1)), 0.0, 0.0)
assert any("末边" in p for p in bad_goal.validate(network, goal=(e_bc, 0.6, -1)))
bad_turn = FixedRoute(((e_ab, 1), (e_ab, -1)), 0.0, 0.0)
assert any("非死端折返" in p for p in bad_turn.validate(network))
assert FixedRoute((), 0.0, 0.0).validate(network) == ["固定路径为空"]
assert any(
    "超过总长度" in p
    for p in FixedRoute(((e_ab, 1),), 60.0, 60.0).validate(network)
)
print("✅ ③ 不连续、非法折返、终点不一致与空路线均被拒绝")

# ④ 信号兼容是 P4 的准入输入：正面信号可共存，背面单向信号会封死固定路线。
assert not fixed.conflicts_with_signal((e_bc, 1))
assert fixed.conflicts_with_signal((e_bc, -1))
assert not fixed.conflicts_with_signal((9999, -1))
print("✅ ④ FixedRoute 可识别反向 One-Way PBS 信号冲突，供 P4 拒绝放置")

print("\n全部通过")
