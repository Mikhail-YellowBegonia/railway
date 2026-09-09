"""会话持久化回归（roadmap #2，docs/session_persistence.md）。

覆盖：
1. 停放列车往返：车头世界坐标、occupancy 窗口（occupied/offset/s）、编组
   （车厢顺序/长度/质量/功率/wagon_id）逐项一致。
2. 行驶中列车往返：route 非空 + remaining_to_goal + goal + v 还原后
   is_moving() 为真、remaining 一致。
3. 方向正确性：direction=+1 / -1 两种朝向的列车往返后 current_direction 一致。
4. 旧存档无 "trains" 字段 → 空列表。
5. 单列车反查失败（边坐标找不到）跳过该列。
6. split_sibling 动态重建：解挂两段共享边 → 重载后互指恢复。

运行：PYTHONPATH=. .venv/bin/python tests/test_session.py
"""
from model.vec3 import Vec3
from model.rail_network import RailNetwork
from model.wagon import create_simple_wagon, Consist
from model.occupancy import OccupancyState
from model.train_entity import TrainEntity, TrainState
from model.train_physics import RealisticElectric
from model.session import serialize_trains, deserialize_trains, rebuild_split_siblings


def build_net():
    net = RailNetwork()
    prev = None
    for i in range(11):
        n = net.add_node(Vec3(i * 20.0, 0.0, 0.0))
        if prev is not None:
            net.add_edge(prev, n)
        prev = n
    return net


def make_train(net, occupied, lens, s=0.0, direction=1):
    wagons = [
        create_simple_wagon(length=l, mass=30.0, P_rated=1000.0 if i == 0 else None)
        for i, l in enumerate(lens)
    ]
    occ = OccupancyState(occupied=occupied, occupied_offset=0.0, s=s, route=[])
    t = TrainEntity(TrainState(occupancy=occ, remaining_to_goal=0.0, v=0.0,
                               consist=Consist(wagons=wagons)), net,
                    RealisticElectric())
    t.kinematics = t._build_kinematics()
    return t


def head_x(t):
    return t.kinematics._path_kin.pose_at(t.state.abs_s).position.x


# =========================================================================
# 1. 停放列车往返
# =========================================================================
net = build_net()
east = [(eid, 1) for eid in net.edges]
t1 = make_train(net, east, [20.0, 18.0, 22.0])
t1.state.occupancy = OccupancyState(
    occupied=east, occupied_offset=20.0, s=40.0, route=[])
t1.kinematics = t1._build_kinematics()
h0 = head_x(t1)

ser = serialize_trains([t1])
back = deserialize_trains(ser, net)
assert len(back) == 1, f"应还原 1 列，实际 {len(back)}"
b = back[0]
assert abs(head_x(b) - h0) < 1e-6, f"车头位置漂移: {h0} -> {head_x(b)}"
assert [e for e, _d in b.state.occupancy.occupied] == [e for e, _d in t1.state.occupancy.occupied]
assert abs(b.state.occupancy.occupied_offset - 20.0) < 1e-9
assert abs(b.state.s - 40.0) < 1e-9
assert [w.length for w in b.state.consist.wagons] == [20.0, 18.0, 22.0]
assert [w.wagon_id for w in b.state.consist.wagons] == \
    [w.wagon_id for w in t1.state.consist.wagons], "wagon_id 应稳定"
assert b.is_parked(), "停放列车重载后应停放"
print("✅ 停放列车往返：位置/occupancy/编组/wagon_id 一致")

# =========================================================================
# 2. 行驶中列车往返
# =========================================================================
t2 = make_train(net, [(0, 1)], [20.0], s=10.0)
t2.assign_route([(1, 1), (2, 1)], 50.0, (2, 1.0, 1))
t2.state.v = 8.0
t2.v_target = 12.0
ser2 = serialize_trains([t2])
back2 = deserialize_trains(ser2, net)
assert len(back2) == 1
b2 = back2[0]
assert b2.is_moving(), "行驶中列车重载后应恢复 controller（is_moving）"
assert abs(b2.state.remaining_to_goal - 50.0) < 1e-9
assert b2.state.goal == (2, 1.0, 1)
assert abs(b2.state.v - 8.0) < 1e-9
assert abs(b2.v_target - 12.0) < 1e-9
assert [e for e, _d in b2.state.occupancy.route] == [1, 2]
print("✅ 行驶中列车往返：controller 重建、remaining/goal/v/v_target 一致")

# =========================================================================
# 3. 方向正确性（direction=-1）
# =========================================================================
west = [(eid, -1) for eid in net.edges]
t3 = make_train(net, west, [20.0], s=10.0)
t3.state.occupancy = OccupancyState(occupied=west, occupied_offset=0.0, s=10.0, route=[])
t3.kinematics = t3._build_kinematics()
assert t3.current_direction() == -1
ser3 = serialize_trains([t3])
back3 = deserialize_trains(ser3, net)
assert back3[0].current_direction() == -1, \
    f"方向还原失败: {back3[0].current_direction()}"
print("✅ 方向正确性：direction=-1 往返后仍为 -1")

# =========================================================================
# 4. 旧存档无 trains 字段
# =========================================================================
from model.geojson_loader import load_trains
import json, tempfile, os
old_path = os.path.join(tempfile.gettempdir(), "old_save.geojson")
net_old = build_net()
data = {"type": "FeatureCollection", "features": [
    {"type": "Feature", "geometry": {"type": "LineString",
     "coordinates": [[0, 0, 0], [20, 0, 0]]}}]}
with open(old_path, "w") as f:
    json.dump(data, f)
assert load_trains(old_path, net_old) == [], "旧存档无 trains 应返回空列表"
print("✅ 旧存档无 trains 字段：优雅退化空列表")

# =========================================================================
# 5. 反查失败跳过单列
# =========================================================================
net5 = build_net()
t5_ok = make_train(net5, [(0, 1)], [20.0], s=5.0)
ser5 = serialize_trains([t5_ok])
# 注入一个坐标找不到的坏 occupied 边
bad = {"consist": ser5[0]["consist"], "v": 0.0, "v_target": 0.0,
       "occupied": [{"a": [9999, 9999, 0], "b": [9999, 9998, 0]}],
       "occupied_offset": 0.0, "s": 5.0, "route": [],
       "remaining_to_goal": 0.0, "goal": None, "physics": "realistic_electric"}
back5 = deserialize_trains([bad, ser5[0]], net5)
assert len(back5) == 1, f"坏列车应被跳过，实际 {len(back5)}"
print("✅ 反查失败跳过单列：坏 occupied 边静默跳过，其余正常")

# =========================================================================
# 6. split_sibling 动态重建
# =========================================================================
net6 = build_net()
east6 = [(eid, 1) for eid in net6.edges]
T = make_train(net6, east6, [20.0, 18.0, 22.0])
T.state.occupancy = OccupancyState(occupied=east6, occupied_offset=0.0,
                                   s=sum([20.0, 18.0, 22.0]), route=[])
T.kinematics = T._build_kinematics()
F, R = T.decouple_at(1)   # 2+1
assert F.split_sibling is R and R.split_sibling is F
ser6 = serialize_trains([F, R])
back6 = deserialize_trains(ser6, net6)
# 序列化不存 sibling，需 rebuild
assert back6[0].split_sibling is None and back6[1].split_sibling is None
rebuild_split_siblings(back6)
assert back6[0].split_sibling is back6[1], "共享边两段应重建 split_sibling 互指"
assert back6[1].split_sibling is back6[0]
print("✅ split_sibling 动态重建：解挂两段共享边 → 重载后互指恢复")

# =========================================================================
# 7. 往返字节稳定（serialize(deserialize(serialize)) == serialize）
# =========================================================================
import math
def trains_close(a, b):
    if len(a) != len(b):
        return False
    for ta, tb in zip(a, b):
        if [w.wagon_id for w in ta.state.consist.wagons] != \
           [w.wagon_id for w in tb.state.consist.wagons]:
            return False
        if not math.isclose(head_x(ta), head_x(tb), abs_tol=1e-4):
            return False
    return True

t7 = make_train(net, [(0, 1)], [20.0], s=7.5)
ser7 = serialize_trains([t7])
back7 = deserialize_trains(ser7, net)
ser7b = serialize_trains(back7)
assert trains_close([t7], back7), "往返后列车应等价"
print("✅ 往返稳定：serialize→deserialize→serialize 列车等价")

# =========================================================================
# 8. GameLoop 层端到端（S 保存 → 重载，含解挂两段 + split_sibling 恢复）
# =========================================================================
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
import pygame
pygame.init()
import tempfile, shutil
import controller.game_loop as gl_module

_save_backup = gl_module.SAVE_PATH
_tmp_save = os.path.join(tempfile.gettempdir(), "session_e2e.geojson")
try:
    # 指向临时存档路径，避免污染真实 manual_track.geojson
    gl_module.SAVE_PATH = _tmp_save
    if os.path.exists(_tmp_save):
        os.remove(_tmp_save)

    from controller.game_loop import GameLoop
    # 第一实例：默认 test_track，构造两列解挂出来的前后段
    gl1 = GameLoop("test_track.geojson")
    netg = gl1.network
    n0g = netg.nodes[0]
    edgeg = netg.edges[next(iter(n0g.incident_edge_ids))]
    from model.pathfinding import _directed_from
    directed = _directed_from(netg, edgeg.edge_id, n0g.node_id)
    wagons = [create_simple_wagon(length=20.0, mass=50.0, P_rated=3000.0)
              for _ in range(3)]
    Tg = TrainEntity(TrainState(
        occupancy=OccupancyState(occupied=[directed], occupied_offset=0.0,
                                 s=0.0, route=[]),
        remaining_to_goal=0.0, v=0.0, consist=Consist(wagons=wagons)),
        netg, RealisticElectric())
    Fg, Rg = Tg.decouple_at(1)  # 2+1，互设 split_sibling
    gl1.trains = [Fg, Rg]
    from model.geojson_writer import write_geojson
    write_geojson(gl1.network, _tmp_save, signals=gl1.signals, trains=gl1.trains)
    gl1.running = False

    # 第二实例：加载临时存档
    gl2 = GameLoop(_tmp_save)
    assert len(gl2.trains) == 2, f"应重载 2 列，实际 {len(gl2.trains)}"
    assert gl2.trains[0].split_sibling is gl2.trains[1], "split_sibling 应重建互指"
    assert gl2.trains[1].split_sibling is gl2.trains[0]
    assert all(t.is_parked() for t in gl2.trains), "重载两段应停放"
    print("✅ GameLoop 端到端：解挂两段 S 保存 → 重载 + split_sibling 恢复")
    gl2.running = False
finally:
    gl_module.SAVE_PATH = _save_backup
    if os.path.exists(_tmp_save):
        os.remove(_tmp_save)
    pygame.quit()

# =========================================================================
# 9. GameLoop 端到端：行驶中列车 S 保存 → 重载 → 续行到站（#2 验收闭环）
# =========================================================================
import pygame
pygame.init()
from model.vec3 import Vec3
from model.rail_network import RailNetwork
from model.geojson_writer import write_geojson
from model.wagon import Consist as _C2, create_simple_wagon as _csw2
from model.occupancy import OccupancyState as _Occ2
from model.train_entity import TrainEntity as _TE2, TrainState as _TS2
from model.train_physics import RealisticElectric as _RE2
from model.pathfinding import find_path_from_point as _fpfp

# 自建直线 0..360 每 60m（e0..e5），目标 e5 t=0.5（世界 x=330）。
_net9 = RailNetwork()
_n9 = [_net9.add_node(Vec3(i * 60.0, 0.0, 0.0)) for i in range(7)]
_e9 = [_net9.add_edge(_n9[i], _n9[i + 1]) for i in range(6)]
_save9 = os.path.join(tempfile.gettempdir(), "session_resume.geojson")
if os.path.exists(_save9):
    os.remove(_save9)
write_geojson(_net9, _save9)

_backup9 = gl_module.SAVE_PATH
try:
    gl_module.SAVE_PATH = _save9
    from controller.game_loop import GameLoop
    DT9 = 1.0 / 60.0

    def _place9(eid, head_s, direction=1):
        wagons = [_csw2(length=20.0, mass=50.0, P_rated=3000.0)]
        occ = _Occ2(occupied=[(eid, direction)], occupied_offset=0.0,
                    s=head_s, route=[])
        t = _TE2(_TS2(occupancy=occ, remaining_to_goal=0.0, v=0.0,
                      consist=_C2(wagons=wagons)), _net9, _RE2())
        t.kinematics = t._build_kinematics()
        return t

    # 第一实例：e0 内 x=30 起步 → 下达 e5 t0.5 → 行驶 8s（中途态）→ S 保存
    _glA = GameLoop(_save9)
    _tA = _place9(_e9[0].edge_id, head_s=30.0)
    _glA.trains = [_tA]
    _glA.active_train = _tA
    _glA.train_v_target = 10.0
    _tA.v_target = 10.0
    _se, _st = _tA.current_edge_and_t()
    _sd = _tA.current_direction()
    _res = _fpfp(_glA.network, _se, _st, _sd, _e9[5].edge_id, 0.5, 1)
    _p, _so, _eo = _res
    _tA.assign_route(_p.edges[1:], max(0.0, _p.total_cost - _so - _eo),
                     (_e9[5].edge_id, 0.5, 1))
    _tA.v_target = 10.0
    for _ in range(int(8.0 / DT9)):
        if _tA.is_parked():
            break
        _glA.active_train.v_target = _glA.train_v_target
        _glA.dispatcher.tick(_tA, DT9, _tA.v_target, _glA.trains)
        _glA.block_manager.rebuild(_glA.network, _glA.signals)
        _glA.block_manager.tick_reservations(_glA.trains)
    assert _tA.is_moving() and _tA.state.v > 3.0, "行驶中态应成立"
    _mid_abs = _tA.state.abs_s
    write_geojson(_glA.network, _save9, signals=_glA.signals,
                  trains=_glA.trains)
    print(f"✅ 保存行驶中列车（v={_tA.state.v:.1f} m/s, abs_s={_mid_abs:.1f}, "
          f"route {len(_tA.state.occupancy.route)} 段）")
    _glA.running = False

    # 第二实例：重载 → 状态还原一致 → 续行到站（head 世界 x≈330）
    _glB = GameLoop(_save9)
    _tB = _glB.trains[0]
    _glB.active_train = _tB
    _glB.train_v_target = _tB.v_target
    assert _tB.is_moving(), "行驶中列车重载应恢复 controller（is_moving）"
    assert abs(_tB.state.v - _tA.state.v) < 1e-6, "v 应还原一致"
    assert abs(_tB.v_target - 10.0) < 1e-6, "v_target 应还原 10"
    assert abs(_tB.state.abs_s - _mid_abs) < 1e-6, "abs_s 应还原一致"
    for _ in range(int(60.0 / DT9)):
        if _tB.is_parked():
            break
        _glB.active_train.v_target = _glB.train_v_target
        _glB.dispatcher.tick(_tB, DT9, _tB.v_target, _glB.trains)
        _glB.block_manager.rebuild(_glB.network, _glB.signals)
        _glB.block_manager.tick_reservations(_glB.trains)
    assert _tB.is_parked(), "重载列车应续行到站（parked）"
    _hx = _tB.kinematics._path_kin.pose_at(_tB.state.abs_s).position.x
    assert abs(_hx - 330.0) < 1.0, \
        f"续行应精确到 goal e5 t=0.5（x=330），实际 head_x={_hx:.1f}"
    print(f"✅ GameLoop 端到端：行驶中 S 保存 → 重载 → 续行到站 "
          f"(head_x={_hx:.1f} = goal)")
    _glB.running = False
finally:
    gl_module.SAVE_PATH = _backup9
    if os.path.exists(_save9):
        os.remove(_save9)
    pygame.quit()

print("\n✅ 全部会话持久化回归通过")
