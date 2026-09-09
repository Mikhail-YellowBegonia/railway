"""Step 6 信号接入运动控制端到端回归：红灯前停车等待、绿灯续行、无信号直行，
以及 direction=-1 起始偏移修复的回归。

网络（自建直线单线，便于精确断言）：
    A(0) --e0(20m)-- B(20) --e1(20m)-- C(40) --e2(20m)-- D(60)

信号：
    S1 在 B，面向 e1（(e1,+1)）→ block1 = {e1}（在 C 处被 S2 封闭）
    S2 在 C，面向 e2（(e2,+1)）→ block2 = {e2}（延伸到死端 D）

列车 A→D 的远场路径 = [e0, e1, e2]，无保护前缀 = [e0]，block1 = [e1]，
block2 = [e2]。这正好能验证"只预约当前段+前方一段、红灯前停车、绿灯续行"。
"""
from model.vec3 import Vec3
from model.rail_network import RailNetwork
from model.wagon import create_simple_wagon, Consist
from model.occupancy import OccupancyState
from model.train_entity import TrainEntity, TrainState
from model.train_physics import SimplePhysics
from model.signal import SignalTable
from model.block import BlockManager, SignalState
from model.dispatch import TrainDispatcher
from model.pathfinding import find_path_from_point, _directed_from

DT = 1.0 / 60.0
V_TARGET = 10.0


def build_network():
    net = RailNetwork()
    a = net.add_node(Vec3(0.0, 0.0, 0.0))
    b = net.add_node(Vec3(20.0, 0.0, 0.0))
    c = net.add_node(Vec3(40.0, 0.0, 0.0))
    d = net.add_node(Vec3(60.0, 0.0, 0.0))
    e0 = net.add_edge(a, b)
    e1 = net.add_edge(b, c)
    e2 = net.add_edge(c, d)
    return net, e0, e1, e2, (a.node_id, b.node_id, c.node_id, d.node_id)


def make_train(net, occupied, consist_length=5.0):
    consist = Consist(wagons=[create_simple_wagon(length=consist_length, mass=30.0)])
    occ = OccupancyState(occupied=occupied, occupied_offset=0.0, s=0.0, route=[])
    state = TrainState(occupancy=occ, remaining_to_goal=0.0, v=0.0, consist=consist)
    return TrainEntity(state, net, SimplePhysics(a_max=2.0, b_max=3.0, v_max=30.0))


# =========================================================================
# Part 0：find_path_from_point direction=-1 起始偏移修复（2026-09 bug）
# =========================================================================
probe_net = RailNetwork()
pa = probe_net.add_node(Vec3(0, 0, 0))
pb = probe_net.add_node(Vec3(10, 0, 0))
pe = probe_net.add_edge(pa, pb)
r = find_path_from_point(probe_net, pe.edge_id, 0.8, -1, pe.edge_id, 0.2, -1)
p, so, eo = r
assert abs((p.total_cost - so - eo) - 6.0) < 1e-9, \
    f"direction=-1 同 Edge：remaining 应为 6m，实际 {p.total_cost - so - eo}"
print("✅ direction=-1 起始偏移修复：逆向同 Edge remaining_to_goal 正确")

# =========================================================================
# 场景 1：红灯前停车等待 → 绿灯续行到终点
# =========================================================================
net, e0, e1, e2, (A, B, C, D) = build_network()
signals = SignalTable()
s1 = _directed_from(net, e1.edge_id, B)  # (e1, +1)
s2 = _directed_from(net, e2.edge_id, C)  # (e2, +1)
assert signals.place(s1)
assert signals.place(s2)
blocks = BlockManager()
blocks.rebuild(net, signals)
assert blocks.block_edges(s1) == frozenset({e1.edge_id})
assert blocks.block_edges(s2) == frozenset({e2.edge_id})

# 占用者：停放在 e2 上（物理占用 block2，无预约）
occupier = make_train(net, [(e2.edge_id, 1)])
# 受测列车：A 出发向东
train = make_train(net, [(e0.edge_id, 1)])
train.assign_route([(e1.edge_id, 1), (e2.edge_id, 1)], 60.0,
                   (e2.edge_id, 1.0, 1))
dispatcher = TrainDispatcher(net, signals, blocks)
trains = [train, occupier]

held_any_time = False
for _ in range(60 * 120):  # 120 秒上限
    for t in trains:
        dispatcher.tick(t, DT, V_TARGET, trains)
    if train.is_holding():
        held_any_time = True
        break

assert held_any_time, "block2 被占时，列车应在信号 S2 前停车等待"
# 关键断言：等待时车头没有越过 S2（e2 未被吞进 occupied）
assert e2.edge_id not in {eid for eid, _d in train.state.occupancy.occupied}, \
    "红灯等待时车头不得越过信号进入 e2（不闯红灯）"
head_pos = train.kinematics._path_kin.pose_at(train.state.abs_s).position
dist_to_C = (head_pos - net.nodes[C].position).length()
assert dist_to_C < 1.0, f"红灯停车点应贴近信号 S2（C），实际距 C {dist_to_C:.3f}m"
print(f"✅ 红灯前停车等待：车头停在 S2 前 {dist_to_C:.3f}m，未进入 block2")

# 清空占用者（移到孤立的别处），绿灯出现
occupier.state.occupancy = OccupancyState(
    occupied=[(e0.edge_id, 1)], occupied_offset=0.0, s=0.0, route=[],
)
# 继续跑，列车应续约 block2 并开到终点 D
reached = False
for _ in range(60 * 120):
    for t in trains:
        dispatcher.tick(t, DT, V_TARGET, trains)
    if train.is_parked():
        reached = True
        break
assert reached, "占用释放后列车应续约并到达终点"
head_pos = train.kinematics._path_kin.pose_at(train.state.abs_s).position
assert (head_pos - net.nodes[D].position).length() < 0.2, \
    f"列车应到达终点 D，实际距 D {(head_pos - net.nodes[D].position).length():.3f}m"
print("✅ 绿灯续行：占用释放后自动续约并到达终点")

# =========================================================================
# 场景 2：绿灯连续通过（无占用）——全程不应进入等待态
# =========================================================================
net2, f0, f1, f2, (A2, B2, C2, D2) = build_network()
signals2 = SignalTable()
s21 = _directed_from(net2, f1.edge_id, B2)
s22 = _directed_from(net2, f2.edge_id, C2)
signals2.place(s21)
signals2.place(s22)
blocks2 = BlockManager()
blocks2.rebuild(net2, signals2)
train2 = make_train(net2, [(f0.edge_id, 1)])
train2.assign_route([(f1.edge_id, 1), (f2.edge_id, 1)], 60.0,
                    (f2.edge_id, 1.0, 1))
disp2 = TrainDispatcher(net2, signals2, blocks2)
trains2 = [train2]
held_during_green = False
reached2 = False
for _ in range(60 * 120):
    for t in trains2:
        disp2.tick(t, DT, V_TARGET, trains2)
    if train2.is_holding():
        held_during_green = True
    if train2.is_parked():
        reached2 = True
        break
assert reached2, "全程绿灯时列车应到达终点"
assert not held_during_green, "全程绿灯时列车不应在信号前停车等待"
print("✅ 绿灯连续通过：全程无等待、直达终点")

# =========================================================================
# 场景 3：无信号直行（授权 = goal，无任何信号约束）
# =========================================================================
net3, g0, g1, g2, (A3, B3, C3, D3) = build_network()
signals3 = SignalTable()
blocks3 = BlockManager()
blocks3.rebuild(net3, signals3)
train3 = make_train(net3, [(g0.edge_id, 1)])
train3.assign_route([(g1.edge_id, 1), (g2.edge_id, 1)], 60.0,
                    (g2.edge_id, 1.0, 1))
disp3 = TrainDispatcher(net3, signals3, blocks3)
trains3 = [train3]
reached3 = False
for _ in range(60 * 120):
    for t in trains3:
        disp3.tick(t, DT, V_TARGET, trains3)
    if train3.is_parked():
        reached3 = True
        break
assert reached3, "无信号时列车应直达终点"
print("✅ 无信号直行：无约束直达终点")

# =========================================================================
# 场景 4：长列车横跨多个 block，绿灯下不得卡死（2026-09 bug 回归）
# =========================================================================
# 60m 车身 vs 20m 的 block：车身同时占据 3 条边，即同时持有 2 个 block。
# 早前的预约预算用"总共持有几个 block"（len(held)）判定，长列车会因
# 车尾尚未驶离的 block 占满预算而在绿灯前被永久卡死——现在改为只数
# "车头前方"已预约的 block（_walk_frontier 返回的 blocks_ahead）。
net4 = RailNetwork()
nodes4 = [net4.add_node(Vec3(x, 0.0, 0.0)) for x in range(0, 140, 20)]
edges4 = [net4.add_edge(nodes4[i], nodes4[i + 1]) for i in range(len(nodes4) - 1)]
signals4 = SignalTable()
for i in range(1, len(edges4)):
    signals4.place(_directed_from(net4, edges4[i].edge_id, nodes4[i].node_id))
blocks4 = BlockManager()
blocks4.rebuild(net4, signals4)

consist4 = Consist(wagons=[create_simple_wagon(length=60.0, mass=30.0)])
occ4 = OccupancyState(occupied=[(edges4[0].edge_id, 1)], occupied_offset=0.0,
                      s=0.0, route=[])
state4 = TrainState(occupancy=occ4, remaining_to_goal=0.0, v=0.0, consist=consist4)
train4 = TrainEntity(state4, net4, SimplePhysics(a_max=2.0, b_max=3.0, v_max=30.0))
train4.assign_route([(e.edge_id, 1) for e in edges4[1:]], 120.0,
                    (edges4[-1].edge_id, 1.0, 1))
disp4 = TrainDispatcher(net4, signals4, blocks4)
trains4 = [train4]

reached4 = False
stuck_holding = False
for _ in range(60 * 240):
    for t in trains4:
        disp4.tick(t, DT, V_TARGET, trains4)
    blocks4.rebuild(net4, signals4)
    blocks4.tick_reservations(trains4)
    if train4.is_holding():
        stuck_holding = True
        break
    if train4.is_parked():
        reached4 = True
        break
assert reached4 and not stuck_holding, \
    "长列车（60m 横跨 3 个 20m block）绿灯下不得卡死——预约预算必须只数车头前方"
print("✅ 长列车跨多个 block：绿灯连续通过不卡死")

# =========================================================================
# 场景 5：单线 2 线车站（会让线）——PBS edge 交集，不整块互斥（2026-09）
# =========================================================================
# 主线 → 道岔T1 →【A道(直线) / B道(绕行)】→ 道岔T2 → 主线。道岔处不放信号，
# A道/B道 被并入同一个粗 block。老版简单闭塞（整块互斥）下，列车停在 B道
# 避让时会让整个 block 红灯、卡死 A道出站的列车；PBS（edge 交集）下，B道
# 与 A道 的 edge 集合交集为空，应准予放行、信号保持绿灯。
net5 = RailNetwork()
W5 = net5.add_node(Vec3(0.0, 0.0, 0.0))
T1_5 = net5.add_node(Vec3(20.0, 0.0, 0.0))
T2_5 = net5.add_node(Vec3(60.0, 0.0, 0.0))
E5 = net5.add_node(Vec3(80.0, 0.0, 0.0))
B5 = net5.add_node(Vec3(40.0, 10.0, 0.0))
w_in5 = net5.add_edge(W5, T1_5)          # 西主线
A5 = net5.add_edge(T1_5, T2_5)           # A道（直线）
B1_5 = net5.add_edge(T1_5, B5)           # B道（绕行上段）
B2_5 = net5.add_edge(B5, T2_5)           # B道（绕行下段）
e_out5 = net5.add_edge(T2_5, E5)         # 东主线

signals5 = SignalTable()
s_in5 = _directed_from(net5, w_in5.edge_id, W5.node_id)
signals5.place(s_in5)
blocks5 = BlockManager()
blocks5.rebuild(net5, signals5)
assert {A5.edge_id, B1_5.edge_id, B2_5.edge_id} <= blocks5.block_edges(s_in5), \
    "A道/B道 应因道岔无信号而被并入同一粗 block"

# 避让列车停在 B道（占用 B道，无预约）
train_b = make_train(net5, [(B1_5.edge_id, 1), (B2_5.edge_id, 1)])
# 颜色：B道被占，但 A道仍是一条通路 → 进站信号应 GREEN（不再是整块红灯）
assert blocks5.compute_colors([train_b], net5, signals5)[s_in5] is SignalState.GREEN, \
    "B道被占时，A道仍为通路，进站信号应 GREEN（edge 交集，不整块互斥）"
print("✅ 2 线车站：B道被占时进站信号保持 GREEN（A道通路未被误锁）")

# 东行列车走 A道，应能穿过（预约 A道 的 edge 与 B道 占用交集为空）
train_a = make_train(net5, [(w_in5.edge_id, 1)])
train_a.assign_route([(A5.edge_id, 1), (e_out5.edge_id, 1)], 80.0,
                     (e_out5.edge_id, 1.0, 1))
disp5 = TrainDispatcher(net5, signals5, blocks5)
trains5 = [train_a, train_b]
reached5 = False
stuck5 = False
for _ in range(60 * 120):
    for t in trains5:
        disp5.tick(t, DT, V_TARGET, trains5)
    blocks5.rebuild(net5, signals5)
    blocks5.tick_reservations(trains5)
    if train_a.is_holding():
        stuck5 = True
        break
    if train_a.is_parked():
        reached5 = True
        break
assert reached5 and not stuck5, \
    "B道被占不应卡死 A道出站列车（PBS edge 交集语义）"
print("✅ 2 线车站：A道列车在 B道被占时仍顺利通过")

# =========================================================================
# 场景 6：对向起步预约失败 → holding 保留指令，不 hard_stop 丢指令（2026-09）
# =========================================================================
# 单线 X--e0--Y--e1--Z--e2--W，X 端信号(朝+1) 与 W 端信号(朝-1) 保护中间区间。
# 两车对向、各自车头停在无保护首段内、同时起步驶向对方：第一帧预约失败时，
# frontier 停在车头脚下（frontier=0），旧实现因 raw_authority<0 触发 hard_stop
# 清空 route —— 玩家重新下令又再丢，表现为"信号死锁"。修复后应进入 holding
# 且保留 route/goal（绿灯后续行）。
net6 = RailNetwork()
X6 = net6.add_node(Vec3(0, 0, 0)); Y6 = net6.add_node(Vec3(40, 0, 0))
Z6 = net6.add_node(Vec3(80, 0, 0)); W6 = net6.add_node(Vec3(120, 0, 0))
e6a = net6.add_edge(X6, Y6); e6b = net6.add_edge(Y6, Z6); e6c = net6.add_edge(Z6, W6)
sig6 = SignalTable()
sig6.place(_directed_from(net6, e6a.edge_id, X6.node_id))  # X 朝 +1
sig6.place(_directed_from(net6, e6c.edge_id, W6.node_id))  # W 朝 -1
blk6 = BlockManager(); blk6.rebuild(net6, sig6)
disp6 = TrainDispatcher(net6, sig6, blk6)
# A 停在 e6a 内（头 x=20，朝 +x），B 停在 e6c 内（头 x=100，朝 -x）
tA6 = make_train(net6, [(e6a.edge_id, 1)], consist_length=20.0)
tA6.state.occupancy.s = 20.0
tA6.kinematics = tA6._build_kinematics()
tB6 = make_train(net6, [(e6c.edge_id, -1)], consist_length=20.0)
tB6.state.occupancy.s = 20.0
tB6.kinematics = tB6._build_kinematics()
trains6 = [tA6, tB6]
# 各自对向下令
_se, _st = tA6.current_edge_and_t(); _sd = tA6.current_direction()
r6 = find_path_from_point(net6, _se, _st, _sd, e6c.edge_id, 0.0, 1, debug=False)
p6, so6, eo6 = r6
tA6.assign_route(p6.edges[1:], max(0.0, p6.total_cost - so6 - eo6), (e6c.edge_id, 0.0, 1))
_se, _st = tB6.current_edge_and_t(); _sd = tB6.current_direction()
r6b = find_path_from_point(net6, _se, _st, _sd, e6a.edge_id, 1.0, -1, debug=False)
p6b, so6b, eo6b = r6b
tB6.assign_route(p6b.edges[1:], max(0.0, p6b.total_cost - so6b - eo6b), (e6a.edge_id, 1.0, -1))
for _ in range(3):
    disp6.tick(tA6, DT, V_TARGET, trains6)
    disp6.tick(tB6, DT, V_TARGET, trains6)
    blk6.tick_reservations(trains6)
# 起步预约失败：两车都应进入 holding 且保留 route（而非 hard_stop 清空）
assert tA6.is_holding() and tB6.is_holding(), \
    f"对向起步应进入 holding 等待，而非丢指令（A.hold={tA6.is_holding()} B.hold={tB6.is_holding()}）"
assert len(tA6.state.occupancy.route) > 0 and len(tB6.state.occupancy.route) > 0, \
    "holding 必须保留 route/goal（绿灯后续行），不得 hard_stop 清空"
print("✅ 对向起步：预约失败进入 holding 保留指令，不再 hard_stop 丢指令锁死")

# 绿灯续行：B 从 trains 移除（模拟驶离消失），A 应能续约并驶出
trains6.remove(tB6)
blk6.tick_reservations(trains6)
reached6 = False
for _ in range(60 * 60):
    disp6.tick(tA6, DT, V_TARGET, trains6)
    blk6.rebuild(net6, sig6)
    blk6.tick_reservations(trains6)
    if tA6.is_parked():
        reached6 = True
        break
assert reached6, "绿灯后 A 应续行到终点（holding 语义：保留指令可恢复）"
print("✅ 对向起步绿灯续行：B 让开后 A 从 holding 恢复并到达")

print("\n全部通过")
