"""DELETE 保护回归（2026-09 用户报"删除有列车的轨道崩溃"）。

修复语义（与 Transport Fever 2 / OpenTTD 一致）：**拒绝删除被列车占用
（occupied）或已预约/待走（route）的轨道**——被删的 edge_id 会留在列车状态里
成悬空引用，下一帧 `TrainDispatcher.tick`（`network.edges[eid]`）或
kinematics 重建即 KeyError 崩溃（已复现）。检查在 GameLoop 的 DELETE 点击
分支（`_rail_delete_blocked_reason`），Editor 保持对列车零耦合。

覆盖：
① 被占用边 → 拒绝（边保留）；
② 被 route 预约的边 → 拒绝；
③ 自由边 → 允许删除，删后 tick/rebuild 不崩；
④ 端到端走 `_handle_mouse_down` 的真实点击路径。
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
from controller.editor import EditMode

# 自建 3 边直线（-100..0 / 0..100 / 100..200），不依赖 manual_track 存档内容。
net = RailNetwork()
n0 = net.add_node(Vec3(-100.0, 0.0, 0.0))
n1 = net.add_node(Vec3(0.0, 0.0, 0.0))
n2 = net.add_node(Vec3(100.0, 0.0, 0.0))
n3 = net.add_node(Vec3(200.0, 0.0, 0.0))
e0 = net.add_edge(n0, n1)
e1 = net.add_edge(n1, n2)
e2 = net.add_edge(n2, n3)
save = os.path.join(tempfile.gettempdir(), "delete_guard.geojson")
if os.path.exists(save):
    os.remove(save)
write_geojson(net, save)

DT = 1.0 / 60.0
_backup = gl_module.SAVE_PATH
try:
    gl_module.SAVE_PATH = save
    from controller.game_loop import GameLoop
    gl = GameLoop(save)
    gnet = gl.network
    ge0, ge1, ge2 = gnet.edges[0], gnet.edges[1], gnet.edges[2]

    def mk(eid, offset, s):
        wagons = [create_simple_wagon(length=20.0, mass=50.0, P_rated=3000.0)]
        occ = OccupancyState(occupied=[(eid, 1)], occupied_offset=offset, s=s, route=[])
        t = TrainEntity(TrainState(occupancy=occ, remaining_to_goal=0.0, v=0.0,
                                   consist=Consist(wagons=wagons)), gnet,
                        RealisticElectric())
        t.kinematics = t._build_kinematics()
        return t

    # A 占 e1（车身 x=40..100）；B 占 e0 且 route 预约 e1
    tA = mk(ge1.edge_id, 40.0, 60.0)
    tB = mk(ge0.edge_id, 40.0, 60.0)
    tB.state.occupancy.route = [(ge1.edge_id, 1)]
    gl.trains = [tA, tB]
    gl.editor.set_mode(EditMode.DELETE)

    locked = gl._train_locked_edges()
    assert {ge0.edge_id, ge1.edge_id} <= locked, f"占用+预约边应锁定，实际 {locked}"
    print("✅ 锁定集合 = 列车 occupied ∪ route:", sorted(locked))

    # ① 占用边拒绝
    gl.editor.update_hover(Vec3(95.0, 0.0, 0.0))          # e1 远端（避开车身命中）
    assert gl.editor.hovered_edge_id == ge1.edge_id
    assert gl._rail_delete_blocked_reason() is not None, "占用边必须拒绝删除"
    # ② 预约边拒绝（e1 亦被 tB 预约；单独验 e1 已在 ①；此处验"仅预约"场景）
    tA.state.occupancy.occupied = [(ge0.edge_id, 1)]       # 让 A 改占 e0，使 e1 仅被预约
    tA.kinematics = tA._build_kinematics()
    gl.editor.update_hover(Vec3(95.0, 0.0, 0.0))
    assert gl._rail_delete_blocked_reason() is not None, "被预约的边必须拒绝删除"
    print("✅ 占用边 / 预约边删除均被拒绝")
    # ③ 自由边允许
    gl.editor.update_hover(Vec3(150.0, 0.0, 0.0))         # e2 无列车
    assert gl.editor.hovered_edge_id == ge2.edge_id
    assert gl._rail_delete_blocked_reason() is None, "自由边应可删除"
    print("✅ 自由边（无列车占用/预约）允许删除")
finally:
    gl_module.SAVE_PATH = _backup
    if os.path.exists(save):
        os.remove(save)

# ④ 端到端：真实 mouse-down 点击路径
pygame.init()
_save2 = os.path.join(tempfile.gettempdir(), "delete_guard2.geojson")
if os.path.exists(_save2):
    os.remove(_save2)
write_geojson(net, _save2)
try:
    gl_module.SAVE_PATH = _save2
    gl2 = GameLoop(_save2)
    gnet2 = gl2.network
    gl2.editor.set_mode(EditMode.DELETE)

    def mk2(eid, offset, s):
        wagons = [create_simple_wagon(length=20.0, mass=50.0, P_rated=3000.0)]
        occ = OccupancyState(occupied=[(eid, 1)], occupied_offset=offset, s=s, route=[])
        t = TrainEntity(TrainState(occupancy=occ, remaining_to_goal=0.0, v=0.0,
                                   consist=Consist(wagons=wagons)), gnet2,
                        RealisticElectric())
        t.kinematics = t._build_kinematics()
        return t

    tA2 = mk2(1, 40.0, 60.0)      # 占 e1
    tB2 = mk2(0, 40.0, 60.0)      # 占 e0
    tB2.state.occupancy.route = [(1, 1)]   # 预约 e1
    gl2.trains = [tA2, tB2]
    sw, sh = gl2.surface.get_size()

    def click_world(x, y=0.0):
        # 真实 GUI 里 run() 每帧先 update_hover 再处理点击事件；测试同样先更新
        gl2.editor.update_hover(Vec3(x, y, 0.0))
        sx, sy = gl2.camera.world_to_screen(x, y, sw, sh)
        ev = pygame.event.Event(pygame.MOUSEBUTTONDOWN,
                                {"button": 1, "pos": (int(sx), int(sy))})
        gl2._handle_mouse_down(ev)

    # ④a 点击被预约的 e1（x=10 远离车身）→ 拒绝，边保留，tick 不崩
    before = sorted(gl2.network.edges.keys())
    click_world(10.0)
    assert sorted(gl2.network.edges.keys()) == before, "被占用/预约的边不应被删除"
    for t in gl2.trains:
        gl2.dispatcher.tick(t, DT, t.v_target, gl2.trains)
    gl2.block_manager.rebuild(gl2.network, gl2.signals)
    print("✅ 端到端：点击被占用/预约轨道被拒绝，边保留且 tick/rebuild 不崩")

    # ④b 点击自由边 e2（x=150）→ 删除成功，tick 不崩
    click_world(150.0)
    after = sorted(gl2.network.edges.keys())
    assert 2 not in after, f"自由边应被删除，实际 {after}"
    for t in gl2.trains:
        gl2.dispatcher.tick(t, DT, t.v_target, gl2.trains)
    print("✅ 端到端：自由边可正常删除，删后 tick 不崩")
finally:
    gl_module.SAVE_PATH = _backup
    # 只删自己的临时文件——**不要**用 gl_module.SAVE_PATH 判断（它已恢复成
    # 真实存档 manual_track.geojson，曾因此误删用户存档，2026-09 教训）
    if os.path.exists(_save2):
        os.remove(_save2)
    pygame.quit()

print("\n✅ 全部 DELETE 保护（拒绝删除列车占用/预约路段）回归通过")
