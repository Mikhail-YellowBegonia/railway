"""端到端回归：目标恰好落在 Node 上（goal_t=0.0 或 1.0）时不发生停车抖动。

Step 1（定点停车）关注的边界场景：remaining_to_goal 收敛到 0 附近时，
BrakingController 的到达判据不应因浮点误差在"到达/未到达"之间反复翻转。
用真实 TrainEntity + find_path_between_nodes 全链路驱动，不 mock 任何层。
"""
from model.wagon import create_simple_wagon, Consist
from model.rail_network import RailNetwork
from model.geojson_loader import load_geojson
from model.pathfinding import find_path_between_nodes
from model.occupancy import OccupancyState
from model.train_entity import TrainEntity, TrainState
from model.train_physics import SimplePhysics

network = load_geojson("test_track.geojson")
path = find_path_between_nodes(network, 0, 5)
assert path is not None

consist = Consist(wagons=[create_simple_wagon(length=20.0, mass=50.0)])

# 从节点 0 出发，停在 occupied 首边的起点（park_directed 决定车头朝向）
start_directed = path.edges[0]
occ = OccupancyState(occupied=[start_directed], occupied_offset=0.0, s=0.0, route=[])
state = TrainState(occupancy=occ, remaining_to_goal=0.0, v=0.0, consist=consist)
train = TrainEntity(state, network, SimplePhysics())

# 目标：节点 5，即整条 path 走完、goal_t 恰好落在最后一条边的端点（Node 上）
route = path.edges[1:]
train.assign_route(route, path.total_cost)

dt = 1.0 / 60.0
v_target = 10.0
max_steps = 60 * 120  # 120 秒安全上限
reached_step = None

for step in range(max_steps):
    train.update(dt, v_target)
    if train.is_parked():
        reached_step = step
        break

assert reached_step is not None, "未能在步数上限内到达节点目标"
assert train.state.v == 0.0, "到达后速度未归零"
assert train.is_parked(), "到达后应处于停放状态"

# 停车位置精度：用底层 pose_at(abs_s) 查车头（转向架）位置，不能用
# get_all_wagon_poses（返回车厢几何中心，天然落后车头半个车身长，
# 拿它验证"是否到达目标节点"是错误的参照点——之前排查提前停车 bug
# 时踩过这个坑）。目标节点 5 位于世界坐标 (8, 0)（见 test_track.geojson）。
from model.train_controller import BrakingController
head_pos = train.kinematics._path_kin.pose_at(train.state.abs_s).position
dist_to_goal = (head_pos - network.nodes[5].position).length()
assert dist_to_goal <= BrakingController.STOP_EPSILON + 1e-6, \
    (f"车头停止位置距目标节点 {dist_to_goal:.4f}m，超出 STOP_EPSILON——"
     f"这正是 2026-09 修复的提前停车 bug 的端到端回归")

print(f"✅ 目标为 Node 时无抖动到达（step={reached_step}），车速归零，"
      f"车头精确停在目标节点附近（误差 {dist_to_goal:.4f}m）")
