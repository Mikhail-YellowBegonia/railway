"""命令级路径策略：动态默认，固定显式。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from model.plan import FixedRoute, PathPolicy, PlanItem
from model.rail_network import RailNetwork
from model.vec3 import Vec3


network = RailNetwork()
a = network.add_node(Vec3(0.0, 0.0, 0.0))
b = network.add_node(Vec3(100.0, 0.0, 0.0))
edge_id = network.add_edge(a, b).edge_id

dynamic = PlanItem.goto((edge_id, 0.8, 1))
assert dynamic.path_policy is PathPolicy.DYNAMIC
assert dynamic.effective_path_policy is PathPolicy.DYNAMIC
assert dynamic.validate(network) == []

fixed = PlanItem.goto(
    (edge_id, 0.8, 1),
    path_policy=PathPolicy.FIXED,
    fixed_route=FixedRoute(((edge_id, 1),), 0.0, 0.0),
)
assert fixed.effective_path_policy is PathPolicy.FIXED
assert fixed.validate(network) == []

legacy = PlanItem.goto(
    (edge_id, 0.8, 1),
    fixed_route=FixedRoute(((edge_id, 1),), 0.0, 0.0),
)
assert legacy.effective_path_policy is PathPolicy.FIXED

print("✅ 动态默认、固定显式、旧 fixed_route 兼容策略通过")
