"""原地折返回归：列车逻辑首尾互换，车厢物理姿态保持。

2026-09 用户规格（临时追加功能 1）：
- **任意位置**可调用（不限死端），但**仍要求车身所在段无道岔**
  （`connection_count() >= 3` 的节点）——车身跨分歧点时镜像会把分歧边一起带上
  （Step3 死循环 bug 同源），直接拒绝并返回 False。
- 列车逻辑方向与车厢物理方向分离：`consist` 列表反转，每节
  `Wagon.orientation` 同步翻转；按 wagon_id 观察的世界位置与物理朝向不变。
- 场景一 = 玩家手动折返（PLAY 模式选中停放列车按 R）；
  场景二 = 指令要求折返（`_do_auto_reversal` 折返标记消费，另见 test_dispatch）。

覆盖：① 单边中部掉头：逻辑顺序反转 + 物理位置/朝向不变 + route 清空；
② 二次掉头完全复原；③ 跨道岔拒绝且状态不变；④ R 键端到端（停放可折返、
行驶中拒绝）。
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
from model.train_controller import BrakingController
from controller.editor import EditMode


def w3():
    return [create_simple_wagon(length=20.0, mass=50.0, P_rated=3000.0)
            for _ in range(3)]


# ---------------------------------------------------------------------------
# 1. 模型层：单条 300m 直边中部掉头
# ---------------------------------------------------------------------------
net = RailNetwork()
na = net.add_node(Vec3(0.0, 0.0, 0.0))
nb = net.add_node(Vec3(300.0, 0.0, 0.0))
e0 = net.add_edge(na, nb)          # 单边 300m，列车停在 x=[140,200] 段（非死端）
occ = OccupancyState(occupied=[(e0.edge_id, 1)], occupied_offset=0.0,
                     s=200.0, route=[])
t = TrainEntity(TrainState(occupancy=occ, remaining_to_goal=0.0, v=0.0,
                           consist=Consist(wagons=w3())), net, RealisticElectric())
t.kinematics = t._build_kinematics()

ids_before = [w.wagon_id for w in t.state.consist.wagons]
pos_before = [p.position.x for p in t.kinematics.get_all_wagon_poses(t.state.s)]
heading_before = [p.heading.x for p in t.kinematics.get_all_wagon_poses(t.state.s)]
dir_before = t.current_direction()
assert dir_before == 1

assert t.reverse_in_place() is True, "单边中部应可折返（任意位置）"
ids_after = [w.wagon_id for w in t.state.consist.wagons]
pos_after = [p.position.x for p in t.kinematics.get_all_wagon_poses(t.state.s)]
assert ids_after == list(reversed(ids_before)), "逻辑车头/车尾顺序必须反转"
assert t.current_direction() == -1, "前进方向应翻转"
assert not t.state.occupancy.route and t.state.remaining_to_goal == 0.0, \
    "折返后应清空 route/remaining（旧指令基于旧方向）"
heading_after = [p.heading.x for p in t.kinematics.get_all_wagon_poses(t.state.s)]
for i in range(3):
    assert abs(pos_after[i] - pos_before[2 - i]) < 1e-6
    assert abs(heading_after[i] - heading_before[2 - i]) < 1e-6
print(f"✅ 任意位置掉头：逻辑顺序反转 {[w[:4] for w in ids_after]}，"
      f"物理位置保持 {[round(x,1) for x in pos_after]}，"
      f"方向 {dir_before} → {t.current_direction()}")

# 2. 二次折返完全复原
assert t.reverse_in_place() is True
pos_back = [p.position.x for p in t.kinematics.get_all_wagon_poses(t.state.s)]
assert t.current_direction() == 1
for i in range(3):
    assert abs(pos_back[i] - pos_before[i]) < 1e-6, "二次折返应完全复原"
print("✅ 二次掉头完全复原（位置与朝向均回到折返前）")

# ---------------------------------------------------------------------------
# 3. 跨道岔拒绝（车身段内含 degree>=3 节点）
# ---------------------------------------------------------------------------
net2 = RailNetwork()
c0 = net2.add_node(Vec3(0.0, 0.0, 0.0))
c1 = net2.add_node(Vec3(100.0, 0.0, 0.0))     # 道岔（3 分支）
c2 = net2.add_node(Vec3(200.0, 0.0, 0.0))
c3 = net2.add_node(Vec3(100.0, 100.0, 0.0))
ea = net2.add_edge(c0, c1)
eb = net2.add_edge(c1, c2)
ec = net2.add_edge(c1, c3)
occ2 = OccupancyState(occupied=[(ea.edge_id, 1), (eb.edge_id, 1)],
                      occupied_offset=60.0, s=60.0, route=[])
t2 = TrainEntity(TrainState(occupancy=occ2, remaining_to_goal=0.0, v=0.0,
                            consist=Consist(wagons=w3())), net2, RealisticElectric())
t2.kinematics = t2._build_kinematics()
head_before = t2.kinematics.get_all_wagon_poses(t2.state.s)[0].position.x
assert t2.reverse_in_place() is False, "车身跨道岔必须拒绝折返"
assert t2.current_direction() == 1, "拒绝后方向不应改变"
head_after = t2.kinematics.get_all_wagon_poses(t2.state.s)[0].position.x
assert abs(head_after - head_before) < 1e-9, "拒绝后位置不应改变"
print("✅ 跨道岔（车身段内含分歧点）折返被拒绝，状态不变")

# 同一车身链若仅为头头/尾尾连挂做内部逻辑归一化，不套用调度折返门禁。
poses_before = {
    wagon.wagon_id: (pose.position, pose.heading)
    for wagon, pose in zip(
        t2.state.consist.wagons,
        t2.kinematics.get_all_wagon_poses(t2.state.s),
    )
}
assert t2.reverse_for_coupling() is True
poses_after = {
    wagon.wagon_id: (pose.position, pose.heading)
    for wagon, pose in zip(
        t2.state.consist.wagons,
        t2.kinematics.get_all_wagon_poses(t2.state.s),
    )
}
for wagon_id, (position, heading) in poses_before.items():
    new_position, new_heading = poses_after[wagon_id]
    assert (new_position - position).length() < 1e-6
    assert new_heading.dot(heading) > 1.0 - 1e-6
print("✅ 连挂内部逻辑归一化可跨道岔，且保持逐节车厢物理姿态")

# 4. 行驶中拒绝（controller 非 None）
t3 = TrainEntity(TrainState(
    occupancy=OccupancyState(occupied=[(e0.edge_id, 1)], occupied_offset=0.0,
                             s=200.0, route=[]),
    remaining_to_goal=0.0, v=5.0, consist=Consist(wagons=w3())),
    net, RealisticElectric())
t3.kinematics = t3._build_kinematics()
t3.controller = BrakingController(t3.physics, t3.state.consist)
assert t3.reverse_in_place() is False, "行驶中必须拒绝折返"
print("✅ 行驶中折返被拒绝（需先完全停车）")

# ---------------------------------------------------------------------------
# 5. R 键端到端（PLAY 模式：停放可折返 / 行驶中拒绝）
# ---------------------------------------------------------------------------
save = os.path.join(tempfile.gettempdir(), "reverse_key.geojson")
if os.path.exists(save):
    os.remove(save)
write_geojson(net, save)
_backup = gl_module.SAVE_PATH
try:
    gl_module.SAVE_PATH = save
    from controller.game_loop import GameLoop
    gl = GameLoop(save)
    gnet = gl.network
    tK = TrainEntity(TrainState(
        occupancy=OccupancyState(occupied=[(0, 1)], occupied_offset=0.0,
                                 s=200.0, route=[]),
        remaining_to_goal=0.0, v=0.0, consist=Consist(wagons=w3())),
        gnet, RealisticElectric())
    tK.kinematics = tK._build_kinematics()
    gl.trains = [tK]
    gl.active_train = tK
    gl.editor.set_mode(EditMode.PLAY)

    def press_r():
        gl._handle_event(pygame.event.Event(pygame.KEYDOWN, {"key": pygame.K_r}))

    press_r()
    assert tK.current_direction() == -1, "R 键应让停放列车折返"
    print("✅ R 键：PLAY 模式选中停放列车按 R → 原地折返")

    # 行驶中按 R → 拒绝（方向保持）
    tK.controller = BrakingController(tK.physics, tK.state.consist)
    d_before = tK.current_direction()
    press_r()
    assert tK.current_direction() == d_before, "行驶中按 R 不应折返"
    print("✅ R 键：行驶中按 R 被拒绝（提示先停车）")
finally:
    gl_module.SAVE_PATH = _backup
    if os.path.exists(save):
        os.remove(save)
    pygame.quit()

print("\n✅ 全部原地折返（逻辑换向 / 车厢物理朝向保持 / 道岔拒绝 / R 键）回归通过")
