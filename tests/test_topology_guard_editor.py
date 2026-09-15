"""P4 编辑器回归：双端切分必须在写入前整体预检。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from controller.build_plan import ConstructionPlan
from controller.editor import BuildState, EditMode, Editor
from controller.snap import SnapResult
from model.rail_network import RailNetwork
from model.vec3 import Vec3


network = RailNetwork()
a = network.add_node(Vec3(0.0, 0.0, 0.0))
b = network.add_node(Vec3(100.0, 0.0, 0.0))
c = network.add_node(Vec3(0.0, 100.0, 0.0))
d = network.add_node(Vec3(100.0, 100.0, 0.0))
first_edge = network.add_edge(a, b).edge_id
blocked_edge = network.add_edge(c, d).edge_id

split_calls: list[tuple[int, float]] = []
checks: list[tuple[int, float]] = []


def can_split(edge_id: int, t: float) -> bool:
    checks.append((edge_id, t))
    return edge_id != blocked_edge


def split_edge(edge_id: int, t: float) -> int | None:
    split_calls.append((edge_id, t))
    return network.split_edge_at(edge_id, t)


editor = Editor(network, split_edge, can_split)
editor.set_mode(EditMode.BUILD)
editor.build_state = BuildState.ACTIVE
editor.build_m1 = Vec3(50.0, 0.0, 0.0)
editor.build_m1_edge_id = first_edge
editor.build_m1_edge_t = 0.5
editor.build_t1_candidates = []

# 隔离本回归于 P4 的写入边界：几何算法已在其他编辑器回归中覆盖。
editor._compute_plan = lambda **_kwargs: ConstructionPlan(  # type: ignore[method-assign]
    case=1,
    m1=editor.build_m1,
    m2=Vec3(50.0, 100.0, 0.0),
)
before_edges = set(network.edges)
editor._commit_build(
    Vec3(50.0, 100.0, 0.0),
    SnapResult(True, Vec3(50.0, 100.0, 0.0), snapped_edge_id=blocked_edge, snapped_edge_t=0.5),
)

assert checks == [(first_edge, 0.5), (blocked_edge, 0.5)]
assert split_calls == [], "第二条拒绝时，第一条不得已先切分"
assert set(network.edges) == before_edges
assert editor.build_state is BuildState.ACTIVE
print("✅ 双端 BUILD 在全部预检通过前不写入拓扑")
