"""P4 纯模型：切边的固定路线、目标与控制点改写。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from model.plan import Anchor, FixedRoute, PlanItem
from model.topology_guard import EdgeSplit, routes_conflict_with_signal


split = EdgeSplit(
    old_edge_id=10,
    t=0.4,
    old_node_a_id=1,
    old_node_b_id=2,
    first_edge_id=11,
    second_edge_id=12,
)

# ① 正反两个方向都必须保持连续，反向顺序不能错误地仍是 first → second。
assert split.replace_directed((10, 1)) == ((11, 1), (12, 1))
assert split.replace_directed((10, -1)) == ((12, -1), (11, -1))
assert split.replace_directed((99, 1)) == ((99, 1),)
print("✅ ① 有向边切分映射保持双向行车顺序")

# ② 固定路线、目标 t 和控制点出口在同一个纯改写函数中同步更新。
item = PlanItem.goto(
    (10, 0.8, -1),
    anchors=(Anchor(1, 10), Anchor(2, 10)),
    fixed_route=FixedRoute(((9, 1), (10, 1), (10, -1)), 5.0, 20.0),
)
rewritten = split.rewrite_plan_item(item)
assert rewritten.fixed_route is not None
assert rewritten.fixed_route.edges == ((9, 1), (11, 1), (12, 1), (12, -1), (11, -1))
assert rewritten.goal == (12, (0.8 - 0.4) / 0.6, -1)
assert rewritten.anchors == (Anchor(1, 11), Anchor(2, 12))
assert item.fixed_route.edges == ((9, 1), (10, 1), (10, -1)), "原条目不可变"
print("✅ ② 固定路线 / goal / 控制点出口原子改写，旧条目未变")

# ③ 信号只能阻止反向通行，正向路线仍可保留。上例刻意含折返，故另建单向路线。
one_way = PlanItem.goto(
    (12, 0.5, 1), fixed_route=FixedRoute(((11, 1), (12, 1)), 0.0, 50.0),
)
assert not routes_conflict_with_signal([one_way], (12, 1))
assert routes_conflict_with_signal([one_way], (12, -1))
print("✅ ③ 单向 PBS 背面冲突判定正确")

print("\n全部通过")
