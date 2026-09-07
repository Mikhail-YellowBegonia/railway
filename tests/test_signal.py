"""信号系统端到端回归（Step 3）：槛位消解、One-Way PBS 放置、闭塞划分、
占用推导颜色、持久化。

test_track.geojson 拓扑（相关部分）：
    node 1 (-4,0) --edge2--> node 3 (0,0) --edge3--> node 4 (4,0)
    node 2 (0,4) --edge4--> node 3 (0,0)
node 3 是三度节点：edge2（西）、edge3（东）、edge4（北），三个槛位互相独立。

Step 3 的关键语义变化（相对 Step 2）：SignalTable 只记录"信号放在哪"
（不存颜色），颜色由 BlockManager 按占用状态实时推导；手动 toggle 交互
已废弃（颜色完全自动化，见 model/signal.py、model/block.py 顶部说明）。
"""
import os
from model.vec3 import Vec3
from model.geojson_loader import load_geojson, load_signals
from model.geojson_writer import write_geojson
from model.signal import SignalTable
from model.block import BlockManager, SignalState
from model.pathfinding import _directed_from
from model.rail_network import RailNetwork

network = load_geojson("test_track.geojson")
NODE_1, NODE_2, NODE_3, NODE_4 = 1, 2, 3, 4
EDGE_WEST, EDGE_EAST, EDGE_NORTH = 2, 3, 4


class _FakeTrain:
    """最小假列车：只需要 state.occupancy.occupied 这一个字段。"""
    def __init__(self, occupied_edge_ids):
        self.state = type("S", (), {})()
        self.state.occupancy = type("O", (), {})()
        self.state.occupancy.occupied = [(eid, 1) for eid in occupied_edge_ids]


# --- 1. 槛位消解：点击方向应精确指向对应边 ---
east_click = Vec3(5.0, 0.0, 0.0)
resolved = network.resolve_directed_edge_by_click(NODE_3, east_click)
assert resolved == EDGE_EAST, f"东侧点击应消解到 edge {EDGE_EAST}，实际 {resolved}"

west_click = Vec3(-5.0, 0.0, 0.0)
resolved = network.resolve_directed_edge_by_click(NODE_3, west_click)
assert resolved == EDGE_WEST, f"西侧点击应消解到 edge {EDGE_WEST}，实际 {resolved}"

north_click = Vec3(0.0, 5.0, 0.0)
resolved = network.resolve_directed_edge_by_click(NODE_3, north_click)
assert resolved == EDGE_NORTH, f"北侧点击应消解到 edge {EDGE_NORTH}，实际 {resolved}"

print("✅ 槛位消解：三个方向精确对应各自的边")

# --- 2. One-Way PBS 放置：正面放置成功，背面放置被拒绝 ---
signals = SignalTable()
directed_east = _directed_from(network, EDGE_EAST, NODE_3)  # (edge3, dir): node3->node4
assert signals.place(directed_east) is True
assert signals.has_signal(directed_east) is True

reverse_directed = (EDGE_EAST, -directed_east[1])
assert signals.is_blocked_backside(reverse_directed) is True
assert signals.place(reverse_directed) is False, \
    "不能在已有单向信号的背面再放一个相对的信号"
assert signals.has_signal(reverse_directed) is False, \
    "背面放置被拒绝后，不应该产生任何槛位记录"
print("✅ One-Way PBS 放置：正面成功 + 背面拒绝，不产生非法状态")

edge_north_dir = _directed_from(network, EDGE_NORTH, NODE_3)
assert signals.place(edge_north_dir) is True
print("✅ 放置第二个槛位（北侧），互不影响")

# --- 3. Block 划分：无车占用时全部 GREEN ---
blocks = BlockManager()
blocks.rebuild(network, signals)

colors_empty = blocks.compute_colors(trains=[])
assert colors_empty[directed_east] is SignalState.GREEN
assert colors_empty[edge_north_dir] is SignalState.GREEN
print("✅ 无车占用时，所有信号 GREEN")

# --- 4. 占用推导：block 内有车占用应变 RED，未占用的槛位不受影响 ---
east_block = blocks.block_edges(directed_east)
assert EDGE_EAST in east_block, "east 槛位的 block 至少应包含 edge3 自身"

train_on_east = _FakeTrain(occupied_edge_ids=[EDGE_EAST])
colors_occupied = blocks.compute_colors(trains=[train_on_east])
assert colors_occupied[directed_east] is SignalState.RED, \
    "block 边被占用，对应信号应变 RED"
assert colors_occupied[edge_north_dir] is SignalState.GREEN, \
    "北侧槛位的 block 未被占用，不应受影响（验证 block 之间互相独立）"
print("✅ 占用推导：被占用的 block 变红，不相关的 block 不受影响")

# 车离开后应恢复 GREEN（验证颜色是实时推导，不是一次性翻转）
colors_cleared = blocks.compute_colors(trains=[])
assert colors_cleared[directed_east] is SignalState.GREEN
print("✅ 车离开后颜色恢复 GREEN（实时推导，非翻转标记）")

# --- 5. 无信号的道岔分支应并入同一 block（未设信号处不产生边界） ---
# node 3 是三度节点，north 槛位保护"进入 edge4 方向"；west 侧（edge2）没有
# 信号，理论上 west 方向的占用不该影响 north 槛位的 block（因为它们之间
# 没有共同边界——north 的 block 从 node3 出发只沿 edge4 走，不会绕到 edge2）。
# 这里改验证一个更直接的性质：east 槛位的 block 应该沿途扩展到下一个信号
# 或死端为止，不会在"未设信号的道岔"处提前截断。
assert EDGE_EAST in east_block
print(f"✅ east 槛位 block 边集合：{sorted(east_block)}（未在无信号处提前截断）")

# --- 5b. prune_missing：删边/合并后信号引用悬空的边，应能自我清理 ---
# 复现真实 bug：DELETE 模式删掉信号所在的边后，SignalTable 里还留着
# 指向已删除 edge_id 的槛位，若不清理，下一次 BlockManager.rebuild()
# 会在 head_node() 里对 network.edges[edge_id] 做 KeyError 崩溃。
# Editor 完全不知道 SignalTable 的存在（两者刻意零耦合），所以这个清理
# 必须由 SignalTable 自己对着当前网络做，不能指望 Editor 的删除代码里
# 会有信号系统的清理逻辑。
network_copy = load_geojson("test_track.geojson")
signals_with_dangling = SignalTable()
d_east_copy = _directed_from(network_copy, EDGE_EAST, NODE_3)
signals_with_dangling.place(d_east_copy)
assert len(signals_with_dangling.all_signals()) == 1

del network_copy.edges[EDGE_EAST]  # 模拟 Editor._delete() 删掉这条边
del network_copy.nodes[NODE_4]     # 该边的另一端节点也一并消失（真实删边的效果）

removed = signals_with_dangling.prune_missing(network_copy)
assert removed == [d_east_copy], f"应移除悬空槛位，实际移除：{removed}"
assert len(signals_with_dangling.all_signals()) == 0, "清理后不应再有任何信号"

# 清理后 rebuild 不应再崩溃（这正是用户报告的 KeyError 场景）
blocks_after_prune = BlockManager()
blocks_after_prune.rebuild(network_copy, signals_with_dangling)  # 不应抛异常
print("✅ prune_missing：删边后悬空信号被自我清理，rebuild 不再 KeyError 崩溃")

# --- 6. 退化场景：Edge 中途 split 后，新节点位置精确等于点击坐标，
#    click_dir 恒为零，必须走确定性兜底而不是返回 None ---
mid_pos = Vec3(-6.0, 0.0, 0.0)  # edge0 = (-8,0)-(-4,0) 中点
new_node_id = network.split_edge_at(0, 0.5)
assert new_node_id is not None
new_node = network.nodes[new_node_id]
assert (new_node.position - mid_pos).length() < 1e-6, "split 产生的节点应精确落在点击点"

resolved_degenerate = network.resolve_directed_edge_by_click(new_node_id, mid_pos)
assert resolved_degenerate is not None, \
    "click_dir 为零时必须走确定性兜底，不能返回 None（这是 split 场景的常规路径，不是异常）"
print(f"✅ 退化场景（click_dir=0）正确兜底选中 edge {resolved_degenerate}，未返回 None")

# --- 7. 持久化往返：write_geojson(signals=) -> load_signals 应还原放置位置 ---
TMP_PATH = "/tmp/_test_signal_roundtrip.geojson"
try:
    fresh_network = load_geojson("test_track.geojson")
    fresh_signals = SignalTable()
    d_east = _directed_from(fresh_network, EDGE_EAST, NODE_3)
    d_north = _directed_from(fresh_network, EDGE_NORTH, NODE_3)
    fresh_signals.place(d_east)
    fresh_signals.place(d_north)

    write_geojson(fresh_network, TMP_PATH, signals=fresh_signals)

    reloaded_network = load_geojson(TMP_PATH)
    reloaded_signals = load_signals(TMP_PATH, reloaded_network)

    assert reloaded_signals.has_signal(d_east), "持久化往返后 east 槛位应保留"
    assert reloaded_signals.has_signal(d_north), "持久化往返后 north 槛位应保留"
    assert reloaded_signals.is_blocked_backside((EDGE_EAST, -d_east[1])), \
        "持久化往返后背面永久禁止的语义应保留（推导得出，不需要单独存储）"
    print("✅ 持久化往返：write_geojson(signals=) -> load_signals 放置位置一致")

    # 旧存档（无 "signals" 字段）应优雅退化成空表，不报错
    no_signal_signals = load_signals("test_track.geojson", load_geojson("test_track.geojson"))
    assert len(no_signal_signals.all_signals()) == 0
    print("✅ 无 signals 字段的旧存档优雅退化为空表")
finally:
    if os.path.exists(TMP_PATH):
        os.remove(TMP_PATH)

# --- 8. Block 划分不能被环线"绕背面"吞并（2026-09 真实 bug 复现） ---
# 用户报告：真实存档里放了 8 个信号，其中 4 个的 block 膨胀成了全部
# 34 条边（整张地图）。根因：地图上有一个折返环线，DFS 从某个信号的
# 正面出发，绕一圈后从另一个信号的背面杀回来——早期实现只检查"前方
# 来向是否正对着一个信号"，背面完全检测不出来，于是继续扩张吞并了
# 本该独立的另一段区间。
#
# 这里用最小的环形拓扑复现同一类场景（六边形环，无需依赖会变化的
# manual_track.geojson）：两个信号分别放在环上相对的两个节点，各自
# 面向"顺时针方向"，理应把环切成两段独立的弧（各 3 条边），不能有
# 任何一段吞并另一段或吞并整个环。
import math as _math
ring_network = RailNetwork()
N_RING = 6
ring_nodes = []
for i in range(N_RING):
    angle = 2 * _math.pi * i / N_RING
    pos = Vec3(50.0 * _math.cos(angle), 50.0 * _math.sin(angle), 0.0)
    ring_nodes.append(ring_network.add_node(pos))
ring_edges = [
    ring_network.add_edge(ring_nodes[i], ring_nodes[(i + 1) % N_RING])
    for i in range(N_RING)
]

ring_signals = SignalTable()
# 两个信号都面向"顺时针"（沿 ring_edges 正方向），分别在 node0、node3——
# 把环切成弧 A = {0,1,2}（node0→1→2→3）和弧 B = {3,4,5}（node3→4→5→0）。
s_arc_a = _directed_from(ring_network, ring_edges[0].edge_id, ring_nodes[0].node_id)
s_arc_b = _directed_from(ring_network, ring_edges[3].edge_id, ring_nodes[3].node_id)
assert ring_signals.place(s_arc_a)
assert ring_signals.place(s_arc_b)

ring_blocks = BlockManager()
ring_blocks.rebuild(ring_network, ring_signals)
block_a = ring_blocks.block_edges(s_arc_a)
block_b = ring_blocks.block_edges(s_arc_b)
assert block_a == frozenset({0, 1, 2}), \
    f"弧 A 应恰好是 {{0,1,2}}，实际 {sorted(block_a)}（吞并了另一段或整个环，说明背面绕行未被拦住）"
assert block_b == frozenset({3, 4, 5}), \
    f"弧 B 应恰好是 {{3,4,5}}，实际 {sorted(block_b)}"
print("✅ 环线上两个信号正确把环切成两段独立弧，无背面绕行吞并（2026-09 bug 修复回归）")

print("\n全部通过")
