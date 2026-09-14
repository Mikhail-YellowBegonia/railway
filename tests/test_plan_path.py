"""锚点解析器回归 —— 计划层 P2（`docs/plan_layer_roadmap.md` §4.1）。

规格：条目（目标 + 有序锚点序列 + 控制点）→ 一条 `route`（有向边序列）+ 偏移修正，
**不切边、不改图**、**失败即失败**（硬约束，绝不自动绕行）。

自建图（两条路 + 一个死端 + 一个孤立分量），不依赖 `manual_track.geojson`：

    L(0,0) --E_LS--> S(100,0) --S-M1--> M1(300,0) --M1-G--> G(500,0) --G-Z--> Z(700,0)
                       |                                        ^
                       +--S-P1--> P1(200,60) --P1-P2--> P2(400,60) --P2-G--+
                                                        |
                                                        +--P2-Q--> Q(500,300)（死端）

    · 短路 L-S-M1-G-Z = 700 m；长路（经 P1/P2）≈ 733.24 m ⇒ 无锚点时应取短路
    · S 是度 3 道岔：从 E_LS 进来可以直行（去 M1）或缓分股（去 P1），两者都许可
    · P1 是度 2 共线节点 ⇒ 可作为"经过"锚点，强制走长路

覆盖：
① **无锚点 ⇒ 与 `find_path_from_point` 完全等价**（边缘契约：行为一字不差）；
② **普通锚点 = 硬约束**：经过 P1 必须走长路（且路线确实经过 P1）；
③ **控制点**：在 S 指定出口（去 P1 / 去 M1）分别强制长路 / 短路；
   ——含"控制点正好落在起点节点上"这一特例（`find_path` 的 force_leave 会误判绕远路）；
④ 偏移与剩余弧长记账：`total_cost = 路径各边弧长之和`、`remaining = total − start − end`；
⑤ **失败即失败**：死端锚点无法通行、孤立分量不可达、死出口被条目校验拦下、
   目标在身后（无锚点）不可达、连挂条目缺 `couple_target`；
⑥ `WAIT_COUPLE` 不产生路径（`is_no_path`，不是失败）；
⑦ `GOTO_COUPLE` 给出 `couple_target` 后与等价 `goto` 解析一致；
⑧ 真实存档（若存在）：无锚点与 `find_path_from_point` 等价 + 锚点强制绕另一侧。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from model.pathfinding import find_path_from_point
from model.plan import Anchor, PlanItem, TrainRef
from model.plan_path import PathStart, resolve_plan_item
from model.rail_network import RailNetwork
from model.vec3 import Vec3

net = RailNetwork()
L = net.add_node(Vec3(0.0, 0.0, 0.0))
S = net.add_node(Vec3(100.0, 0.0, 0.0))
M1 = net.add_node(Vec3(300.0, 0.0, 0.0))
G = net.add_node(Vec3(500.0, 0.0, 0.0))
Z = net.add_node(Vec3(700.0, 0.0, 0.0))
P1 = net.add_node(Vec3(200.0, 60.0, 0.0))
P2 = net.add_node(Vec3(400.0, 60.0, 0.0))
Q = net.add_node(Vec3(500.0, 300.0, 0.0))
E_LS = net.add_edge(L, S).edge_id          # 起点所在边（L→S）
E_SM1 = net.add_edge(S, M1).edge_id
E_M1G = net.add_edge(M1, G).edge_id
E_GZ = net.add_edge(G, Z).edge_id
E_SP1 = net.add_edge(S, P1).edge_id
E_P1P2 = net.add_edge(P1, P2).edge_id
E_P2G = net.add_edge(P2, G).edge_id
E_P2Q = net.add_edge(P2, Q).edge_id

# 起点：E_LS 上 t=0.5、朝 S 走（+1）⇒ start_offset = 50
START = PathStart(edge_id=E_LS, t=0.5, direction=1)
# 终点：E_GZ 上 t=0.5、朝 Z 走（+1）⇒ end_offset = 100
GOAL = (E_GZ, 0.5, 1)
SHORT = (E_LS, 1), (E_SM1, 1), (E_M1G, 1), (E_GZ, 1)
LONG = (E_LS, 1), (E_SP1, 1), (E_P1P2, 1), (E_P2G, 1), (E_GZ, 1)

assert abs(net.edges[E_LS].length - 100.0) < 1e-9
assert net.nodes[S.node_id].connection_count() == 3, "S 应是道岔"
assert net.nodes[P1.node_id].connection_count() == 2, "P1 应是二度节点"


def _sum_len(edges):
    return sum(net.edges[e].length for e, _d in edges)


# ── ① 无锚点 ⇒ 与 find_path_from_point 等价 ─────────────────────────────
direct = find_path_from_point(net, E_LS, 0.5, 1, *GOAL)
assert direct is not None, "前提：短径可达"
d_path, d_start_off, d_end_off = direct
res = resolve_plan_item(net, START, PlanItem.goto(GOAL))
assert res.ok, res.failure
assert res.path.edges == tuple(d_path.edges) == SHORT, res.path.edges
assert abs(res.path.total_cost - d_path.total_cost) < 1e-9
assert abs(res.path.start_offset - d_start_off) < 1e-9
assert abs(res.path.end_offset - d_end_off) < 1e-9
assert abs(res.path.total_cost - _sum_len(SHORT)) < 1e-9, "总代价应等于各边弧长之和"
assert abs(res.path.remaining_to_goal - (700.0 - 50.0 - 100.0)) < 1e-9
assert not res.path.passes_through(net, P1.node_id)
print("✅ ① 无锚点：与 find_path_from_point 完全等价（取 700 m 短径），记账一致")

# ── ② 普通锚点 = 硬约束 ────────────────────────────────────────────────
res_anchor = resolve_plan_item(
    net, START, PlanItem.goto(GOAL, (Anchor(P1.node_id),))
)
assert res_anchor.ok, res_anchor.failure
assert res_anchor.path.edges == LONG, res_anchor.path.edges
assert res_anchor.path.passes_through(net, P1.node_id), "必须经过锚点 P1"
assert res_anchor.path.total_cost > res.path.total_cost, "硬约束把路线变长了"
assert abs(res_anchor.path.total_cost - _sum_len(LONG)) < 1e-9
print(
    f"✅ ② 锚点 P1 = 硬约束：改走长路（{_sum_len(LONG):.2f} m > {_sum_len(SHORT):.1f} m），"
    f"且确实经过 P1"
)

# ── ③ 控制点（含"落在起点节点上"的特例）────────────────────────────────
# 出口 = 去 P1（缓分股）⇒ 长路
res_exit_long = resolve_plan_item(
    net, START, PlanItem.goto(GOAL, (Anchor(S.node_id, E_SP1),))
)
assert res_exit_long.ok, res_exit_long.failure
assert res_exit_long.path.edges == LONG, res_exit_long.path.edges
assert res_exit_long.path.passes_through(net, S.node_id), "必须经过 S 并从指定边离开"
# 出口 = 去 M1（直行）⇒ 短路
res_exit_short = resolve_plan_item(
    net, START, PlanItem.goto(GOAL, (Anchor(S.node_id, E_SM1),))
)
assert res_exit_short.ok, res_exit_short.failure
assert res_exit_short.path.edges == SHORT, res_exit_short.path.edges
assert abs(res_exit_short.path.total_cost - _sum_len(SHORT)) < 1e-9, (
    "控制点落在起点节点上时，该段应退化为'就当前这条边'，不得绕远路"
)
print("✅ ③ 控制点：出口选 P1 ⇒ 长路；出口选 M1 ⇒ 短路（起点节点上的控制点不绕路）")

# ── ④ 多锚点 + 记账 ────────────────────────────────────────────────────
multi = resolve_plan_item(
    net, START, PlanItem.goto(GOAL, (Anchor(S.node_id, E_SP1), Anchor(P2.node_id)))
)
assert multi.ok, multi.failure
assert multi.path.edges == LONG, multi.path.edges
assert multi.path.passes_through(net, S.node_id) and multi.path.passes_through(net, P2.node_id)
assert abs(multi.path.total_cost - _sum_len(LONG)) < 1e-9
assert abs(multi.path.remaining_to_goal - (_sum_len(LONG) - 50.0 - 100.0)) < 1e-9
assert multi.path.used_edge_ids() == tuple(e for e, _ in LONG)
print("✅ ④ 两个锚点（控制点 + 普通锚点）串联，代价与剩余弧长记账正确")

# ── ⑤ 失败即失败 ───────────────────────────────────────────────────────
# (a) 死端锚点：没有任何"到达边 → 离开边"组合
res_dead_end = resolve_plan_item(net, START, PlanItem.goto(GOAL, (Anchor(Q.node_id),)))
assert not res_dead_end.ok and "无法通行" in res_dead_end.failure, res_dead_end

# (b) 孤立分量：锚点本身可通行（二度共线），但从起点走不到
iso_a = net.add_node(Vec3(0.0, 1000.0, 0.0))
iso_b = net.add_node(Vec3(100.0, 1000.0, 0.0))
iso_c = net.add_node(Vec3(200.0, 1000.0, 0.0))
net.add_edge(iso_a, iso_b)
net.add_edge(iso_b, iso_c)
res_iso = resolve_plan_item(net, START, PlanItem.goto(GOAL, (Anchor(iso_b.node_id),)))
assert not res_iso.ok and "不可达" in res_iso.failure, res_iso

# (c) 死出口：条目校验阶段就被拦下（与 P1 的 validate 一致）
net_kink = RailNetwork()
k0 = net_kink.add_node(Vec3(0.0, 0.0, 0.0))
k1 = net_kink.add_node(Vec3(100.0, 0.0, 0.0))
k2 = net_kink.add_node(Vec3(100.0, 10.0, 0.0))
e_k01 = net_kink.add_edge(k0, k1).edge_id
e_k12 = net_kink.add_edge(k1, k2).edge_id
res_dead = resolve_plan_item(
    net_kink,
    PathStart(edge_id=e_k01, t=0.0, direction=1),
    PlanItem.goto((e_k12, 0.5, 1), (Anchor(k1.node_id, e_k12),)),
)
assert not res_dead.ok and "条目非法" in res_dead.failure, res_dead

# (d) 目标在身后且无锚点（同边同向、t 已过）⇒ 现有寻路也判不可达（无环可绕）
res_behind = resolve_plan_item(net, PathStart(E_LS, 0.5, 1), PlanItem.goto((E_LS, 0.2, 1)))
assert not res_behind.ok and "不可达" in res_behind.failure, res_behind

# (e) 连挂条目缺目标钩位
res_couple_missing = resolve_plan_item(
    net, START, PlanItem.goto_couple(TrainRef(wagon_id="w-1"))
)
assert not res_couple_missing.ok and "couple_target" in res_couple_missing.failure
print("✅ ⑤ 失败即失败：死端锚点 / 孤立分量 / 死出口 / 目标在身后 / 缺连挂钩位")

# ── ⑥ 等待类条目不产生路径 ─────────────────────────────────────────────
res_wait = resolve_plan_item(net, START, PlanItem.wait_couple())
assert res_wait.path is None and res_wait.is_no_path and res_wait.failure == ""
print("✅ ⑥ WAIT_COUPLE：is_no_path（不产生路径，也不算失败）")

# ── ⑦ 连挂条目给出 couple_target 后与等价 goto 一致 ────────────────────
res_couple = resolve_plan_item(
    net, START, PlanItem.goto_couple(TrainRef(wagon_id="w-1")), couple_target=GOAL
)
assert res_couple.ok, res_couple.failure
assert res_couple.path.edges == res.path.edges, res_couple.path.edges
assert abs(res_couple.path.total_cost - res.path.total_cost) < 1e-9
print("✅ ⑦ GOTO_COUPLE + couple_target ⇒ 与等价 goto 解析一致")

# ── ⑧ 真实存档（若存在）：等价 + 锚点强制绕另一侧 ──────────────────────
save_path = "manual_track.geojson"
if os.path.exists(save_path):
    from model.geojson_loader import load_geojson
    from model.segments import is_cut_node

    real = load_geojson(save_path)
    edges_sorted = sorted(real.edges)
    compared = 0
    anchor_checked = False
    for start_edge_id in edges_sorted:
        for direction in (1, -1):
            for goal_edge_id in edges_sorted:
                if goal_edge_id == start_edge_id:
                    continue
                for goal_direction in (1, -1):
                    direct_real = find_path_from_point(
                        real, start_edge_id, 0.25, direction,
                        goal_edge_id, 0.75, goal_direction,
                    )
                    if direct_real is None:
                        continue
                    real_start = PathStart(start_edge_id, 0.25, direction)
                    real_goal = (goal_edge_id, 0.75, goal_direction)
                    got = resolve_plan_item(real, real_start, PlanItem.goto(real_goal))
                    assert got.ok, got.failure
                    assert got.path.edges == tuple(direct_real[0].edges)
                    assert abs(got.path.total_cost - direct_real[0].total_cost) < 1e-9
                    assert abs(got.path.remaining_to_goal
                               - (got.path.total_cost - direct_real[1] - direct_real[2])) < 1e-9
                    compared += 1
                    # 找一个"无锚点路线没经过的切割点（道岔）"，用它强制绕另一侧
                    if not anchor_checked:
                        cut_nodes = sorted(
                            nid for nid in real.nodes if is_cut_node(real, nid)
                        )
                        untouched = [
                            nid for nid in cut_nodes
                            if not got.path.touches_node(real, nid)
                        ]
                        if untouched:
                            target = untouched[0]
                            forced = resolve_plan_item(
                                real, real_start,
                                PlanItem.goto(real_goal, (Anchor(target),)),
                            )
                            assert forced.ok, f"锚点 {target} 应可达：{forced.failure}"
                            assert forced.path.passes_through(real, target), (
                                f"锚点 {target} 是硬约束，路线必须经过它"
                            )
                            assert forced.path.edges != got.path.edges, (
                                "锚点在另一侧 ⇒ 路线应当换一条"
                            )
                            print(
                                f"✅ ⑧ 真实存档：无锚点解析与 find_path_from_point 等价"
                                f"（{compared} 组对比）；锚点 {target} 强制改走另一侧"
                                f"（{len(got.path.edges)} → {len(forced.path.edges)} 段）"
                            )
                            anchor_checked = True
                    if compared >= 6 and anchor_checked:
                        break
                if compared >= 6 and anchor_checked:
                    break
            if compared >= 6 and anchor_checked:
                break
        if compared >= 6 and anchor_checked:
            break
    assert compared > 0, "真实存档上至少应有一组可对比的起点/终点"
    if not anchor_checked:
        print("ℹ️  真实存档：本组起终点未找到『未被经过的道岔』，跳过锚点用例")
else:
    print(f"ℹ️  跳过 ⑧：{save_path} 不存在（不影响本测试结论）")

print("\n全部通过")
