"""Step 4 进路预约端到端回归：预约冲突、方向令牌（单线双向）、释放时机、
折返不误释放、寻路接入。

用一个自建的最小网络做单线双向场景（比 test_track.geojson 的交叉渡线拓扑
更精确可控）：

    A (0,0) --edge_a(5m)-- M (5,0) --edge_b(5m)-- B (10,0)

在 A 端放一个面向东（进入 edge_a 正方向）的信号，在 B 端放一个面向西
（进入 edge_b、朝 M 方向）的信号，中间节点 M 不放信号（度数为 2，不产生
边界）——这是现实单线双向运行的标准布置：两端各自保护"进入这段单线"，
中途没有信号。block_east（从 A 出发）和 block_west（从 B 出发）分别沿
相反方向 DFS，因为 M 没有信号不会在此截断，最终两个 block 的边集合完全
相同（{edge_a, edge_b}），却是两个不同的 DirectedEdge key（不同 edge_id
起点），**不会**触发 SignalTable 的背靠背放置校验（那条校验只管"同一条
edge 的两个方向"，这里是两条不同的 edge）——真正的冲突只应该在
`BlockManager.reserve_path` 的 edge 级别检测里体现，这正是本测试要验证
的重点。

（注：若改成单一 edge 两端各放一个相对方向的信号，会被 One-Way PBS
的背靠背校验直接拒绝——那是两个信号对同一条 edge 提出矛盾的单向主张，
本身就不该被允许，不是这里要测的场景。）
"""
from model.vec3 import Vec3
from model.rail_network import RailNetwork
from model.wagon import create_simple_wagon, Consist
from model.occupancy import OccupancyState
from model.train_entity import TrainEntity, TrainState
from model.train_physics import SimplePhysics
from model.signal import SignalTable
from model.block import BlockManager, SignalState
from model.pathfinding import _directed_from

network = RailNetwork()
node_a = network.add_node(Vec3(0.0, 0.0, 0.0))
node_m = network.add_node(Vec3(5.0, 0.0, 0.0))
node_b = network.add_node(Vec3(10.0, 0.0, 0.0))
edge_a = network.add_edge(node_a, node_m)
edge_b = network.add_edge(node_m, node_b)

# 孤立的一段轨道，跟单线双向场景完全不连通，仅用于"车头此刻在别处、
# 还没走到目标 block"这类测试场景——需要一个真实存在的 edge_id
# （TrainEntity 构造时会校验 occupied 引用的边确实存在于网络里）。
node_x = network.add_node(Vec3(100.0, 0.0, 0.0))
node_y = network.add_node(Vec3(105.0, 0.0, 0.0))
edge_elsewhere = network.add_edge(node_x, node_y)
ELSEWHERE_EID = edge_elsewhere.edge_id

signals = SignalTable()
d_east = _directed_from(network, edge_a.edge_id, node_a.node_id)  # (edge_a, +1)：A 端，面向东
d_west = _directed_from(network, edge_b.edge_id, node_b.node_id)  # (edge_b, -1)：B 端，面向西
assert signals.place(d_east)
assert signals.place(d_west)

blocks = BlockManager()
blocks.rebuild(network, signals)
block_east = blocks.protection_envelope_edges(d_east)
block_west = blocks.protection_envelope_edges(d_west)
assert block_east == frozenset({edge_a.edge_id, edge_b.edge_id})
assert block_west == frozenset({edge_a.edge_id, edge_b.edge_id})
print("✅ 单线双向：两端各自的 block 边集合完全相同（中间无信号不截断），"
      "但 key 是不同的 DirectedEdge")


def make_train(occupied_edge_id, direction, s=0.0):
    consist = Consist(wagons=[create_simple_wagon(length=5.0, mass=30.0)])
    occ = OccupancyState(
        occupied=[(occupied_edge_id, direction)], occupied_offset=0.0, s=s, route=[],
    )
    state = TrainState(occupancy=occ, remaining_to_goal=0.0, v=0.0, consist=consist)
    return TrainEntity(state, network, SimplePhysics())


SHARED_EID = edge_a.edge_id  # edge_a 同时属于 block_east 和 block_west

train_1 = make_train(SHARED_EID, 1)
train_2 = make_train(SHARED_EID, -1)

# --- 1. 方向令牌：train_1 先预约东向 block，train_2 想预约西向 block（对向，
#    共享 edge_a）应该被拒绝 ---
assert blocks.reserve_path(train_1, [SHARED_EID]) is True
assert blocks.reserve_path(train_2, [SHARED_EID]) is False, \
    "对向 block 共享 edge_a，train_2 应该被 train_1 的预约挡住（方向令牌）"
print("✅ 方向令牌：对向预约冲突被正确拒绝，不是等两车都上路才检测死锁")

# 同一列车重复预约同一路径应该成功（幂等，不是"被自己挡住"）
assert blocks.reserve_path(train_1, [SHARED_EID]) is True
print("✅ 同一列车重复预约自己已持有的路径：幂等成功")

# --- 2. 颜色：两个方向的信号都应该显示 RED（block 被占用/预约） ---
colors = blocks.compute_colors([train_1], network, signals)
assert colors[d_east] is SignalState.RED
assert colors[d_west] is SignalState.RED, \
    "对向 block 共享同一条 edge，train_1 占用时对向信号也应该显示占用/预约中"
print("✅ 预约期间两端信号都显示 RED（对向共享同一物理轨道）")

# --- 3. 释放：train_1 的 occupied 和 route 都不再涉及该 block 时才释放 ---
blocks.tick_reservations([train_1])
assert blocks.is_reserved_by_other(d_west, train_2) is True, \
    "train_1 仍在占用 edge_a（occupied 未变），预约不应被释放"

train_1.state.occupancy.occupied = [(ELSEWHERE_EID, 1)]  # 模拟车身已完全驶离
train_1.state.occupancy.route = []
blocks.tick_reservations([train_1])
assert blocks.reserve_path(train_2, [SHARED_EID]) is True, \
    "train_1 已驶离且 route 也不再需要该 block，预约应被释放，train_2 可以预约"
print("✅ 释放时机正确：occupied ∪ route 都不再涉及该 block 才释放")

# --- 4. route 未耗尽时不应误释放（即便 occupied 暂时不含该 edge） ---
# 先清空 train_2 在上一步留下的预约（占用 ∪ route 都不再涉及该 block），
# 否则 train_3 的预约会被 train_2 挡住——这不是 bug，是预约互斥的正常
# 表现，这里只是需要先腾出 block 才能测下一个独立场景。
train_2.state.occupancy.occupied = [(ELSEWHERE_EID, 1)]
train_2.state.occupancy.route = []
blocks.tick_reservations([train_2])

train_3 = make_train(occupied_edge_id=ELSEWHERE_EID, direction=1)  # 车头还没走到该 block
train_3.state.occupancy.route = [(SHARED_EID, 1)]                  # 但该 edge 在待走路由里
assert blocks.reserve_path(train_3, [SHARED_EID]) is True
blocks.tick_reservations([train_3])
assert blocks.is_reserved_by_other(d_east, make_train(SHARED_EID, 1)) is True, \
    "该 edge 仍在 train_3 的 route 里（还没走到），不该被误释放"
print("✅ route 未耗尽时不会被误释放（不是只看 occupied 快照）")

# --- 5. is_reserved_by_other：预约持有者对自己不算冲突 ---
blocks.tick_reservations([train_3])
assert blocks.is_reserved_by_other(d_east, train_3) is False
print("✅ 预约持有者查询自己的 block 不算冲突")

# --- 6. tick_reservations 只信任传入的 trains 列表：不在列表里的持有者
#    （模拟 decouple/couple 产生的旧对象被移除）应立即释放 ---
# 先清空 train_3 的预约（同样是腾出 block 给下一个独立场景用，不是 bug）。
train_3.state.occupancy.occupied = [(ELSEWHERE_EID, 1)]
train_3.state.occupancy.route = []
blocks.tick_reservations([train_3])

train_4 = make_train(SHARED_EID, 1)
assert blocks.reserve_path(train_4, [SHARED_EID]) is True
blocks.tick_reservations([])  # train_4 已从 GameLoop.trains 移除（模拟解挂后旧对象作废）
assert blocks.is_reserved_by_other(d_east, make_train(SHARED_EID, 1)) is False, \
    "holder 不在当前 trains 列表里，预约应立即释放，不留下悬挂预约"
print("✅ 不在 trains 列表里的持有者（解挂/连挂产生的旧对象）预约被立即清理")

# --- 7. passable_fn：预约冲突应该正确阻断寻路层的候选边 ---
train_5 = make_train(SHARED_EID, 1)
train_6 = make_train(SHARED_EID, -1)
assert blocks.reserve_path(train_5, [SHARED_EID]) is True
passable_fn_for_6 = blocks.make_passable_fn(train_6)
assert passable_fn_for_6(edge_a, -1) is False, \
    "train_6 请求通过 train_5 已预约的 edge，passable_fn 应返回 False"
passable_fn_for_5 = blocks.make_passable_fn(train_5)
assert passable_fn_for_5(edge_a, 1) is True, \
    "train_5 查询自己已预约的 edge，passable_fn 应返回 True（不阻断自己）"
print("✅ make_passable_fn 正确阻断非持有者、放行持有者")

# --- 8. 重复 edge occurrence：驶离第一次后必须释放，不能因远处再次出现而延寿 ---
# 先释放上一独立场景的 train_5 预约。
train_5.state.occupancy.occupied = [(ELSEWHERE_EID, 1)]
train_5.state.occupancy.route = []
blocks.tick_reservations([train_5])
train_7 = make_train(edge_a.edge_id, 1)
train_7.state.occupancy.route = [(edge_b.edge_id, 1), (edge_a.edge_id, 1)]
assert blocks.reserve_path(train_7, [edge_a.edge_id]) is True
# 第一次经过 edge_a 后车身已清出；未来路线稍后会再次经过同一个 edge_id。
train_7.state.occupancy.occupied = [(edge_b.edge_id, 1)]
blocks.tick_reservations([train_7])
train_8 = make_train(edge_a.edge_id, -1)
assert blocks.reserve_path(train_8, [edge_a.edge_id]) is True, \
    "过去 occurrence 的预约应在车身清出后释放，不得被未来同 edge 引用延寿"
print("✅ 重复 edge occurrence：第一次驶离即释放，未来再次经过时重新预约")

print("\n全部通过")
