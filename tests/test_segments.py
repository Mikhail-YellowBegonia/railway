"""路段分区（simple_segment 全图分区）回归 —— 计划层 P0。

规格见 `docs/plan_layer_roadmap.md` P0 与 `model/segments.py` 顶部：
切分点 = **度数 ≠ 2** 或 **二度但过境转向不许可**；切完"段"才具备
**段内可通行**这一性质。本测试**自建图**，不依赖 `manual_track.geojson`
（仓库既有约定：避免用户存档拓扑漂移导致测试失效）；真实存档只在存在时
**断言不变量**并打印观测值。

覆盖：
① 直线链（全二度 + 两端死端）→ 1 段；
② **90° 急折的二度节点必须切开**（"段内可通行"规则的存在理由），
   并顺带确认 `simple_segment_from_endpoint` 的**旧语义未被改动**；
③ 道岔（度数 ≠ 2）处切开，覆盖不重不漏；内部节点查询；
④ **无切割点的纯环**（正六边形，六处转向 60° 全部许可）→ 1 段、首尾同节点；
⑤ 灯泡线（道岔 + 绕回自身的环）→ 环自成一段；
⑥ 以上各图 `validate_partition` 全部返回空；
⑦ 真实存档（若存在）只断言不变量。
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from model.rail_network import RailNetwork
from model.segments import (
    build_partition,
    is_cut_node,
    validate_partition,
)
from model.vec3 import Vec3


def _check(partition, network, label):
    problems = validate_partition(network, partition)
    assert not problems, f"{label}: 分区自查失败 -> {problems}"
    assert partition.total_edge_count == len(network.edges), (
        f"{label}: 覆盖边数 {partition.total_edge_count} != 网络边数 {len(network.edges)}"
    )


# ── ① 直线链：A - B - C - D（B/C 二度共线，A/D 死端）───────────────────────
net1 = RailNetwork()
a = net1.add_node(Vec3(0.0, 0.0, 0.0))
b = net1.add_node(Vec3(100.0, 0.0, 0.0))
c = net1.add_node(Vec3(200.0, 0.0, 0.0))
d = net1.add_node(Vec3(300.0, 0.0, 0.0))
e_ab = net1.add_edge(a, b)
e_bc = net1.add_edge(b, c)
e_cd = net1.add_edge(c, d)

p1 = build_partition(net1)
_check(p1, net1, "① 直线链")
assert len(p1) == 1, f"直线链应切成 1 段，实际 {len(p1)} 段"
seg = p1.segments[0]
assert seg.node_ids == (a.node_id, b.node_id, c.node_id, d.node_id), seg.node_ids
assert seg.edge_ids == (e_ab.edge_id, e_bc.edge_id, e_cd.edge_id), seg.edge_ids
assert seg.a_node_id == a.node_id and seg.b_node_id == d.node_id
assert seg.interior_node_ids == (b.node_id, c.node_id)
assert not seg.is_closed_loop
assert abs(seg.length - 300.0) < 1e-9, seg.length
assert seg.edge_ids_from(a.node_id) == seg.edge_ids
assert seg.edge_ids_from(d.node_id) == tuple(reversed(seg.edge_ids)), \
    "从 b 端出发应返回反转序列"
assert p1.segment_of_edge(e_bc.edge_id) is seg
assert p1.segments_at_node(b.node_id) == (seg,), "二度内部节点只属于 1 段"
print("✅ ① 直线链 → 1 段（4 节点 3 边），端点=两端死端，端点反查/边长/边序正确")

# ── ② 90° 急折的二度节点必须切开 ─────────────────────────────────────────
net2 = RailNetwork()
k0 = net2.add_node(Vec3(0.0, 0.0, 0.0))
k1 = net2.add_node(Vec3(100.0, 0.0, 0.0))   # 度数 2，但过境约 90° 急折
k2 = net2.add_node(Vec3(100.0, 10.0, 0.0))
f1 = net2.add_edge(k0, k1)
f2 = net2.add_edge(k1, k2)

assert net2.nodes[k1.node_id].connection_count() == 2
assert not net2.turn_allowed(k1.node_id, f1.edge_id, f2.edge_id), "急折处应不许可"
assert not net2.turn_allowed(k1.node_id, f2.edge_id, f1.edge_id), "反向同样不许可"
assert is_cut_node(net2, k1.node_id), "急折的二度节点必须判为切割点"

p2 = build_partition(net2)
_check(p2, net2, "② 急折")
assert len(p2) == 2, f"急折处必须切开 ⇒ 2 段，实际 {len(p2)} 段"
assert [s.edge_ids for s in p2.segments] == [(f1.edge_id,), (f2.edge_id,)], \
    [s.edge_ids for s in p2.segments]
for s in p2.segments:
    assert not s.interior_node_ids, "切开后每段都不应再有内部节点"

# 旧语义未被改动（对照）：simple_segment_from_endpoint 只看度数，仍把急折当成一整段
old_len, old_turnout, old_edges = net2.simple_segment_from_endpoint(k0.node_id)
assert old_edges == [f1.edge_id, f2.edge_id], \
    f"simple_segment_from_endpoint 语义被改动了？实际 {old_edges}"
assert abs(old_len - (100.0 + 10.0)) < 1e-9
print("✅ ② 90° 急折的二度节点被切开（2 段）；simple_segment_from_endpoint 旧语义未变")

# ── ③ 道岔处切开 + 内部节点查询 ─────────────────────────────────────────
net3 = RailNetwork()
t = net3.add_node(Vec3(0.0, 0.0, 0.0))        # 度数 3 = 道岔
m = net3.add_node(Vec3(100.0, 0.0, 0.0))      # 二度共线（臂 1 内部）
arm_a = net3.add_node(Vec3(200.0, 0.0, 0.0))
arm_b = net3.add_node(Vec3(0.0, 100.0, 0.0))  # 臂 2 端点
arm_c = net3.add_node(Vec3(-100.0, 0.0, 0.0)) # 臂 3 端点
g_tm = net3.add_edge(t, m)
g_ma = net3.add_edge(m, arm_a)
g_tb = net3.add_edge(t, arm_b)
g_tc = net3.add_edge(t, arm_c)

p3 = build_partition(net3)
_check(p3, net3, "③ 道岔")
assert len(p3) == 3, f"三度道岔 ⇒ 3 段，实际 {len(p3)} 段"
assert p3.total_edge_count == 4
assert len(p3.segments_at_node(t.node_id)) == 3, "道岔节点应关联 3 段"
long_seg = p3.segment_of_edge(g_tm.edge_id)
assert long_seg is p3.segment_of_edge(g_ma.edge_id), "同臂两条边应属同一段"
assert long_seg.interior_node_ids == (m.node_id,)
assert len(p3.segments_at_node(m.node_id)) == 1, "内部节点只属于 1 段"
assert p3.segment_of_edge(g_tb.edge_id).edge_ids == (g_tb.edge_id,)
print("✅ ③ 道岔处切开（3 段 / 4 边，覆盖不重不漏），内部节点反查唯一")

# ── ④ 无切割点的纯环（正六边形：六处转向 60°，全部许可）──────────────────
net4 = RailNetwork()
radius = 100.0
loop_nodes = [
    net4.add_node(
        Vec3(radius * math.cos(math.tau * i / 6), radius * math.sin(math.tau * i / 6), 0.0)
    )
    for i in range(6)
]
loop_edges = [
    net4.add_edge(loop_nodes[i], loop_nodes[(i + 1) % 6]) for i in range(6)
]
assert not any(is_cut_node(net4, n.node_id) for n in loop_nodes), \
    "正六边形环上不应有切割点（60° 转向全部许可）"
p4 = build_partition(net4)
_check(p4, net4, "④ 纯环")
assert len(p4) == 1, f"无切割点的纯环应为 1 段，实际 {len(p4)} 段"
ring = p4.segments[0]
assert ring.is_closed_loop and ring.node_ids[0] == ring.node_ids[-1]
assert len(ring.edge_ids) == 6 and len(ring.node_ids) == 7
assert set(ring.edge_ids) == {e.edge_id for e in loop_edges}
assert len(ring.interior_node_ids) == 5
print("✅ ④ 无切割点的纯环（正六边形）→ 1 段闭合环，6 边 / 首尾同节点")

# ── ⑤ 灯泡线：道岔 + 绕回自身的环 ───────────────────────────────────────
net5 = RailNetwork()
hub = net5.add_node(Vec3(radius, 0.0, 0.0))          # 环上 0° 的顶点 = 道岔
ring_nodes = [
    net5.add_node(
        Vec3(radius * math.cos(math.tau * i / 6), radius * math.sin(math.tau * i / 6), 0.0)
    )
    for i in range(1, 6)
]
tail_node = net5.add_node(Vec3(radius + 100.0, 0.0, 0.0))
net5.add_edge(hub, tail_node)
net5.add_edge(hub, ring_nodes[0])
for i in range(4):
    net5.add_edge(ring_nodes[i], ring_nodes[i + 1])
net5.add_edge(ring_nodes[4], hub)

assert net5.nodes[hub.node_id].connection_count() == 3
p5 = build_partition(net5)
_check(p5, net5, "⑤ 灯泡线")
assert len(p5) == 2, f"灯泡线应切成 2 段（环 + 尾巴），实际 {len(p5)} 段"
balloon = p5.segment_of_edge(net5.edges[1].edge_id)
tail = p5.segment_of_edge(net5.edges[0].edge_id)
assert balloon.is_closed_loop and balloon.a_node_id == hub.node_id
assert len(balloon.edge_ids) == 6 and len(balloon.interior_node_ids) == 5
assert tail.edge_ids == (net5.edges[0].edge_id,) and not tail.is_closed_loop
assert len(p5.segments_at_node(hub.node_id)) == 2
print("✅ ⑤ 灯泡线 → 环自成一段（首尾同为道岔）+ 尾巴一段")

# ── ⑥ 孤立点 / 空图不产生空段 ────────────────────────────────────────────
net6 = RailNetwork()
net6.add_node(Vec3(0.0, 0.0, 0.0))
net6.add_node(Vec3(50.0, 0.0, 0.0))
p6 = build_partition(net6)
assert len(p6) == 0, f"孤立节点不应产生段，实际 {len(p6)} 段"
assert validate_partition(net6, p6) == []
print("✅ ⑥ 孤立节点不产生空段")

# ── ⑦ 真实存档：只断言不变量（不锁死段数，避免存档漂移）─────────────────
save_path = "manual_track.geojson"
if os.path.exists(save_path):
    from model.geojson_loader import load_geojson

    net_save = load_geojson(save_path)
    p_save = build_partition(net_save)
    _check(p_save, net_save, "⑦ 真实存档")
    cut_count = sum(1 for nid in net_save.nodes if is_cut_node(net_save, nid))
    print(
        f"✅ ⑦ 真实存档 {save_path}：{len(net_save.nodes)} 节点 / "
        f"{len(net_save.edges)} 边 → {len(p_save)} 段（全边覆盖、自查通过；"
        f"切割点 {cut_count} 个）"
    )
else:
    print(f"ℹ️  跳过 ⑦：{save_path} 不存在（不影响本测试结论）")

print("\n全部通过")
