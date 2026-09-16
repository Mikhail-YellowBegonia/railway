"""远场/近场寻路拆分回归测试（Step 5）。

核心断言：
1. 远场寻路（`SignalTable.passable_topology_only`）只看拓扑单向限制，
   不看闭塞占用/预约——即使目标路径上的信号已被别的列车预约（红灯），
   远场寻路依然应该能找到完整路径，不因为"眼前有车"就判不可达。
2. 近场截断（`BlockManager.truncate_to_next_signal`）= 走到第一个信号
   为止的无保护路段（不需要预约） + 该信号实际保护的完整 block（从
   信号出发向前延伸到下一个信号/死端）。**不是**"走到第一个挂信号的
   节点就停"——那是本次实现踩过的一个真实 bug：截出来的段和
   `BlockManager._compute_protection_envelope` 的边界错位一格，导致
   `reserve_path` 永远查不到交集、预约形同虚设（用真实 GameLoop 端到端
   验证时发现，见 model/block.py 顶部记录）。
3. One-Way PBS 的反方向硬性禁止无论远场近场都必须遵守——这是拓扑级
   限制，不是信号灯状态，`passable_topology_only` 必须挡住背面。

test_track.geojson 拓扑（相关部分）：
    node0(-8,0)死端 --edge0--> node1(-4,0)三度 --edge2--> node3(0,0)三度
    --edge3--> node4(4,0)二度 --edge5--> node5(8,0)死端
"""
from model.wagon import create_simple_wagon, Consist
from model.geojson_loader import load_geojson
from model.pathfinding import find_path_from_point, _directed_from
from model.occupancy import OccupancyState
from model.train_entity import TrainEntity, TrainState
from model.train_physics import SimplePhysics
from model.signal import SignalTable
from model.block import BlockManager

network = load_geojson("test_track.geojson")

EDGE_0, EDGE_2, EDGE_3, EDGE_5 = 0, 2, 3, 5


def make_parked_train(occupied_edge_id, direction, length=5.0):
    consist = Consist(wagons=[create_simple_wagon(length=length, mass=30.0)])
    occ = OccupancyState(
        occupied=[(occupied_edge_id, direction)], occupied_offset=0.0, s=0.0, route=[],
    )
    state = TrainState(occupancy=occ, remaining_to_goal=0.0, v=0.0, consist=consist)
    return TrainEntity(state, network, SimplePhysics())


# 两个信号：一个在 node1（面向 edge2），一个在 node3（面向 edge3）。
# 这样第一个信号的 block 精确等于 [edge2]（下一个信号在 node3 封闭边界），
# 能真正验证"只截到第一个区间为止，不含第二个信号后面的路段"。
signals = SignalTable()
signal_at_node1 = _directed_from(network, EDGE_2, 1)  # node1 出发，面向 edge2
signal_at_node3 = _directed_from(network, EDGE_3, 3)  # node3 出发，面向 edge3
assert signals.place(signal_at_node1)
assert signals.place(signal_at_node3)

blocks = BlockManager()
blocks.rebuild(network, signals)
assert blocks.protection_envelope_edges(signal_at_node1) == frozenset({EDGE_2}), \
    ("信号1的保护包络应恰好是 [edge2]，实际 "
     f"{sorted(blocks.protection_envelope_edges(signal_at_node1))}")
assert blocks.protection_envelope_edges(signal_at_node3) == frozenset({EDGE_3, EDGE_5}), \
    ("信号2的保护包络应是 [edge3, edge5]（一路延伸到死端），实际 "
     f"{sorted(blocks.protection_envelope_edges(signal_at_node3))}")

# --- 1. 占用信号2保护的 edge3 不应阻断远场寻路 ---
occupier = make_parked_train(EDGE_3, 1)
assert blocks.reserve_path(occupier, [EDGE_3])

far_result = find_path_from_point(
    network, 0, 0.0, 1, 5, 1.0, 1,
    passable_fn=signals.passable_topology_only,
    allow_reversal=True, consist_length=5.0,
)
assert far_result is not None, \
    "远场寻路不该因为 edge3 被别的列车预约（红灯）就判不可达"
far_path, _start_offset, _end_offset = far_result
far_edge_ids = [eid for eid, _d in far_path.edges]
assert far_edge_ids == [EDGE_0, EDGE_2, EDGE_3, EDGE_5]
assert EDGE_3 in far_edge_ids, "远场路径应该正常穿过被占用的 edge3（远场不看占用）"
print("✅ 远场寻路无视闭塞占用/预约，正确算出穿过被占用路段的完整路径")

# --- 2. 近场截断：应恰好是 [edge0（无保护，原样经过）, edge2（信号1
#    保护的完整 block）]，不含 edge3/edge5（那是信号2 保护的下一个区间，
#    这次范围只预约前方一个区间，不做 Long Reserve 式的多区间预留）。
near_field_edges = blocks.truncate_to_next_route_span(network, signals, far_path.edges)
near_field_edge_ids = [eid for eid, _d in near_field_edges]
assert near_field_edge_ids == [EDGE_0, EDGE_2], \
    (f"近场段应恰好是 [edge0, edge2]（无保护段 + 信号1 的完整 block），"
     f"实际 {near_field_edge_ids}")
assert len(near_field_edges) < len(far_path.edges), \
    "近场段应该比完整远场路径短（只预约前方一个区间，不是走到底）"
print(f"✅ 近场截断 = 无保护段 + 第一个区间：{near_field_edge_ids}"
      f"（远场完整路径 {far_edge_ids}）")

# --- 3. 近场预约不应该被 edge3 上的占用影响——edge3 属于信号2 的 block，
#    根本不在这次近场段（信号1 的 block）里，两者互相独立 ---
requester = make_parked_train(1, 1)
assert blocks.reserve_path(requester, near_field_edge_ids) is True, \
    "近场段（信号1 的 block）与 occupier 占用的 edge3（信号2 的 block）无关，预约应该成功"
print("✅ 近场预约与远场路径上更靠后的占用互不影响")

# --- 4. One-Way PBS 反方向硬性禁止：远场寻路也必须遵守，不能因为"远场
#    不看信号"就连拓扑限制也一起无视了 ---
reverse_of_signal = (signal_at_node3[0], -signal_at_node3[1])
assert signals.passable_topology_only(network.edges[EDGE_3], reverse_of_signal[1]) is False, \
    "反方向硬性禁止是拓扑级限制，passable_topology_only 必须挡住，不能只在近场才管"
print("✅ passable_topology_only 正确遵守 One-Way PBS 反方向硬性禁止")

# --- 5. 近场段精确对齐一个真实被占用的 block 时，预约必须失败——
#    与场景 3 对照：场景 3 是"近场段与占用无关"，这里是"近场段本身
#    就是被占用的那个 block"，两者都要验证，缺一个都不足以说明
#    reserve_path 真的在检查正确的东西。用信号1 的 block（[edge2]）
#    构造对齐场景。
occupier_2 = make_parked_train(EDGE_2, 1)
signals_2 = SignalTable()
assert signals_2.place(signal_at_node1)
assert signals_2.place(signal_at_node3)
blocks_2 = BlockManager()
blocks_2.rebuild(network, signals_2)
assert blocks_2.reserve_path(occupier_2, [EDGE_2])

far_result_2 = find_path_from_point(
    network, 0, 0.0, 1, 5, 1.0, 1,
    passable_fn=signals_2.passable_topology_only,
    allow_reversal=True, consist_length=5.0,
)
assert far_result_2 is not None
near_field_2 = blocks_2.truncate_to_next_route_span(
    network, signals_2, far_result_2[0].edges,
)
near_field_2_ids = [eid for eid, _d in near_field_2]
assert near_field_2_ids == [EDGE_0, EDGE_2], \
    f"近场段应恰好是 [edge0, edge2]，实际 {near_field_2_ids}"

requester_2 = make_parked_train(EDGE_5, 1)
assert blocks_2.reserve_path(requester_2, near_field_2_ids) is False, \
    "近场段精确对齐已被占用的 block（edge2）时，预约必须失败"
print("✅ 近场段精确对齐真实被占用 block 时预约正确失败")

# --- 6. 一路无信号的场景：近场截断应该返回整条远场路径不截断 ---
no_signal_network = load_geojson("test_track.geojson")
no_signals = SignalTable()
no_blocks = BlockManager()
no_blocks.rebuild(no_signal_network, no_signals)
plain_result = find_path_from_point(
    no_signal_network, 0, 0.0, 1, 5, 1.0, 1,
    allow_reversal=True, consist_length=5.0,
)
assert plain_result is not None
plain_path, _, _ = plain_result
plain_truncated = no_blocks.truncate_to_next_route_span(
    no_signal_network, no_signals, plain_path.edges,
)
assert plain_truncated == list(plain_path.edges), \
    "完全没有信号时，近场截断应该返回整条远场路径，不截断"
print("✅ 无信号场景下近场截断等于整条远场路径")

# --- 7. 只有一个信号、其 block 一路延伸到死端（没有第二个边界）时，
#    近场段应该等于从信号开始的完整剩余路径，不会因为"找不到下一个
#    边界"而出错或漏掉尾部 ---
single_signal_network = load_geojson("test_track.geojson")
single_signals = SignalTable()
single_signal = _directed_from(single_signal_network, EDGE_3, 3)
assert single_signals.place(single_signal)
single_blocks = BlockManager()
single_blocks.rebuild(single_signal_network, single_signals)
single_result = find_path_from_point(
    single_signal_network, 0, 0.0, 1, 5, 1.0, 1,
    passable_fn=single_signals.passable_topology_only,
    allow_reversal=True, consist_length=5.0,
)
assert single_result is not None
single_near = single_blocks.truncate_to_next_route_span(
    single_signal_network, single_signals, single_result[0].edges,
)
single_near_ids = [eid for eid, _d in single_near]
assert single_near_ids == [EDGE_0, EDGE_2, EDGE_3, EDGE_5], \
    (f"单信号且 block 延伸到死端时，近场段应覆盖到底，"
     f"实际 {single_near_ids}")
print("✅ 单信号 block 延伸到死端时，近场段正确覆盖到底（无第二边界不出错）")

print("\n全部通过")
