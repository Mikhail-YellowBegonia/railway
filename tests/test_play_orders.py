"""PLAY 模式寻路指令回归：巡航速度（v_target）在"下达寻路指令"后应保留。

2026-09 bug2（用户报告：橙色路径可见但设了 v_target 列车不动）：
真实 GUI 主循环 run() 每帧执行 `active_train.v_target = train_v_target`
（写回巡航），而 `_issue_goal_order` 旧实现下达指令后把 `train_v_target`
清零 → 玩家"停车按 ↑ 设巡航 → 右键设目的地"或"行驶中右键改向"后巡航被
抹掉，列车刹停——表现为有路径（route 已下达、渲染成橙色）却不动。

修复：下达寻路指令**不清零巡航**（急停/空格仍清零，见 game_loop.py 空格
分支）；停放车巡航=0 时下达后打印"按 ↑ 起步"，手动驾驶语义不变。

回归场景：
A) 先设巡航 8 m/s → 下达寻路指令 → 巡航保留、列车按巡航前进（bug2 主场景）；
B) 停放巡航 0 → 下达指令 → 巡航仍 0（手动驾驶，需玩家 ↑ 起步）。
"""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import pygame
pygame.init()

import tempfile
import controller.game_loop as gl_module
from model.vec3 import Vec3
from model.rail_network import RailNetwork
from model.geojson_writer import write_geojson
from model.wagon import Consist, create_simple_wagon
from model.occupancy import OccupancyState
from model.train_entity import TrainEntity, TrainState
from model.train_physics import RealisticElectric

DT = 1.0 / 60.0

# 自建直线网络：0..240 每 60m（e0..e3），写临时 geojson 供 GameLoop 加载。
net = RailNetwork()
ns = [net.add_node(Vec3(i * 60.0, 0.0, 0.0)) for i in range(5)]
edges = [net.add_edge(ns[i], ns[i + 1]) for i in range(4)]

_tmp = os.path.join(tempfile.gettempdir(), "play_orders_net.geojson")
if os.path.exists(_tmp):
    os.remove(_tmp)
write_geojson(net, _tmp)

_backup = gl_module.SAVE_PATH
try:
    gl_module.SAVE_PATH = _tmp  # 指向自建存档 → GameLoop 加载它
    from controller.game_loop import GameLoop
    gl = GameLoop(_tmp)
    e0, e2 = edges[0], edges[2]

    def place(eid, head_s, direction=1):
        wagons = [create_simple_wagon(length=20.0, mass=50.0, P_rated=3000.0)]
        occ = OccupancyState(occupied=[(eid, direction)], occupied_offset=0.0,
                             s=head_s, route=[])
        t = TrainEntity(TrainState(occupancy=occ, remaining_to_goal=0.0, v=0.0,
                                   consist=Consist(wagons=wagons)), gl.network,
                        RealisticElectric())
        t.kinematics = t._build_kinematics()
        return t

    def sim(t, seconds):
        """复刻 run() 每帧巡航写回 + dispatcher.tick 推进，跑满或到站。"""
        for _ in range(int(seconds / DT)):
            if t.is_parked():
                break
            gl.active_train.v_target = gl.train_v_target  # == run() 巡航写回
            gl.dispatcher.tick(t, DT, t.v_target, gl.trains)
            gl.block_manager.rebuild(gl.network, gl.signals)
            gl.block_manager.tick_reservations(gl.trains)

    # ---- 场景 A（bug2 主场景）：先设巡航 8 → 下达寻路指令 → 巡航保留并前进
    tA = place(e0.edge_id, head_s=36.0)      # 车头在 e0 中部朝 +1（x=36）
    gl.trains = [tA]
    gl.active_train = tA
    gl.train_v_target = 8.0                  # 玩家按住 ↑ 建立的巡航
    tA.v_target = 8.0
    gl._issue_goal_order(tA, e2.edge_id, 0.5, "bug2 场景 A")
    assert gl.train_v_target == 8.0, \
        f"下达寻路指令后巡航应保留（bug2 回归），实际 {gl.train_v_target}"
    assert tA.state.occupancy.route, "指令应已下达（route 非空）"
    start_abs = tA.state.abs_s
    sim(tA, 12.0)
    assert tA.state.abs_s > start_abs + 5.0, \
        f"设巡航后下达指令列车应前进（bug2 回归），位移={tA.state.abs_s - start_abs:.1f}m"
    print(f"✅ bug2 回归 A：下达指令后巡航保留，列车前进 "
          f"{tA.state.abs_s - start_abs:.1f} m")

    # ---- 场景 B：停放巡航 0 → 下达 → 巡航仍 0（手动驾驶语义不变）
    tB = place(e0.edge_id, head_s=20.0)
    gl.trains = [tB]
    gl.active_train = tB
    gl.train_v_target = 0.0
    tB.v_target = 0.0
    gl._issue_goal_order(tB, e2.edge_id, 0.5, "bug2 场景 B")
    assert gl.train_v_target == 0.0, "停放巡航 0 下达后应仍为 0（需玩家 ↑ 起步）"
    print("✅ 手动驾驶语义：巡航 0 下达后不自动起步（打印按 ↑ 提示）")

    gl.running = False
finally:
    gl_module.SAVE_PATH = _backup
    if os.path.exists(_tmp):
        os.remove(_tmp)
    pygame.quit()

print("\n✅ 全部 PLAY 寻路指令（巡航保留）回归通过")
