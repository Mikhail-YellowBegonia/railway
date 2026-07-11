"""D6 多节编组测试：验证链式求解和可视化。"""
from model.wagon import create_simple_wagon, Consist
from model.rigid_kinematics import RigidWagonKinematics
from model.rail_network import RailNetwork
from model.geojson_loader import load_geojson
from model.pathfinding import find_path_between_nodes

# 加载网络
network = load_geojson("test_track.geojson")
print(f"网络：{len(network.nodes)} 节点，{len(network.edges)} 边")

# 找一条测试路径（test_track 只有 6 个节点，用 0 → 5）
path = find_path_between_nodes(network, 0, 5)
if not path:
    print("未找到路径")
    exit(1)
print(f"路径：{len(path.edges)} 段，总长 {path.total_cost:.2f} m")

# 测试 1: 单节车厢（原有行为）
wagon = create_simple_wagon(length=20.0, mass=50.0)
consist_single = Consist(wagons=[wagon])
kin_single = RigidWagonKinematics(network, path, consist_single)

poses_single = kin_single.get_all_wagon_poses(100.0)
bogies_single = kin_single.get_all_bogie_poses(100.0)
print(f"\n单节车厢（s=100m）：")
print(f"  车厢数: {len(poses_single)}")
print(f"  首节位置: {poses_single[0].position}")
print(f"  转向架对数: {len(bogies_single)}")

# 测试 2: 三节编组
wagon1 = create_simple_wagon(length=20.0, mass=50.0)
wagon2 = create_simple_wagon(length=18.0, mass=45.0)
wagon3 = create_simple_wagon(length=22.0, mass=60.0)
consist_multi = Consist(wagons=[wagon1, wagon2, wagon3])
kin_multi = RigidWagonKinematics(network, path, consist_multi)

print(f"\n三节编组：")
print(f"  总质量: {consist_multi.total_mass} 吨")
print(f"  总长: {consist_multi.total_length} m")

poses_multi = kin_multi.get_all_wagon_poses(200.0)
bogies_multi = kin_multi.get_all_bogie_poses(200.0)
print(f"\n三节编组（s=200m）：")
print(f"  车厢数: {len(poses_multi)}")
for i, pose in enumerate(poses_multi):
    print(f"  第 {i+1} 节位置: ({pose.position.x:.2f}, {pose.position.y:.2f})")
print(f"  转向架对数: {len(bogies_multi)}")

# 验证链式约束：第 i 节后转向架 = 第 i+1 节前转向架
print(f"\n链式约束验证（车钩共享转向架位置）：")
for i in range(len(bogies_multi) - 1):
    rear_i = bogies_multi[i][1].position
    front_i1 = bogies_multi[i+1][0].position
    dist = (rear_i - front_i1).length()
    print(f"  第 {i+1} 节后转向架 <-> 第 {i+2} 节前转向架：距离 {dist:.6f} m")
    assert dist < 1e-3, f"链式约束失败：距离 {dist} > 1mm"

print("\n✅ D6 多节编组测试通过")
