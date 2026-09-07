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
from model.block import BlockManager
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

print("\n全部通过")
