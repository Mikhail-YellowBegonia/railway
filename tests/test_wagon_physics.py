"""阶段 2 测试：Wagon 级别物理（动拖混编）。"""
from model.wagon import create_simple_wagon, Consist
from model.train_physics import RealisticElectric

# 场景 1：3 节动力车（对比阶段 1）
print("=" * 60)
print("场景 1：3 节动力车（各 3000 kW）")
print("=" * 60)
consist_all_powered = Consist(wagons=[
    create_simple_wagon(length=20.0, mass=50.0, P_rated=3000.0),
    create_simple_wagon(length=18.0, mass=45.0, P_rated=3000.0),
    create_simple_wagon(length=22.0, mass=60.0, P_rated=3000.0),
])
physics = RealisticElectric()

print(f"总质量: {consist_all_powered.total_mass} 吨")
print(f"动力车: {sum(1 for w in consist_all_powered.wagons if w.is_powered)} 节")
print(f"总功率: {sum(w.P_rated for w in consist_all_powered.wagons if w.is_powered)} kW")
print(f"\n速度 (m/s) | 加速度 (m/s²)")
print("-" * 40)
for v in [0, 5, 10, 15, 20, 30]:
    a = physics.compute_acceleration(v, 1.0, 0.0, consist_all_powered)
    print(f"{v:6.1f}     | {a:12.3f}")

# 场景 2：1 节动力车 + 2 节拖车
print("\n" + "=" * 60)
print("场景 2：1 节动力车（3000 kW）+ 2 节拖车")
print("=" * 60)
consist_mixed = Consist(wagons=[
    create_simple_wagon(length=20.0, mass=50.0, P_rated=3000.0),  # 动力车
    create_simple_wagon(length=18.0, mass=45.0, P_rated=None),    # 拖车
    create_simple_wagon(length=22.0, mass=60.0, P_rated=None),    # 拖车
])

print(f"总质量: {consist_mixed.total_mass} 吨")
print(f"动力车: {sum(1 for w in consist_mixed.wagons if w.is_powered)} 节")
print(f"总功率: {sum(w.P_rated for w in consist_mixed.wagons if w.is_powered)} kW")
print(f"\n速度 (m/s) | 加速度 (m/s²)")
print("-" * 40)
for v in [0, 5, 10, 15, 20, 30]:
    a = physics.compute_acceleration(v, 1.0, 0.0, consist_mixed)
    print(f"{v:6.1f}     | {a:12.3f}")

# 场景 3：纯拖车编组（无动力）
print("\n" + "=" * 60)
print("场景 3：3 节拖车（无动力，只有阻力）")
print("=" * 60)
consist_no_power = Consist(wagons=[
    create_simple_wagon(length=20.0, mass=50.0, P_rated=None),
    create_simple_wagon(length=18.0, mass=45.0, P_rated=None),
    create_simple_wagon(length=22.0, mass=60.0, P_rated=None),
])

print(f"总质量: {consist_no_power.total_mass} 吨")
print(f"动力车: {sum(1 for w in consist_no_power.wagons if w.is_powered)} 节")
print(f"\n速度 (m/s) | 加速度 (m/s²)")
print("-" * 40)
for v in [0, 5, 10, 15, 20, 30]:
    a = physics.compute_acceleration(v, 1.0, 0.0, consist_no_power)
    print(f"{v:6.1f}     | {a:12.3f}")

print("\n" + "=" * 60)
print("✅ 阶段 2 测试完成")
print("预期：场景 1 加速度最大（9000 kW），场景 2 中等（3000 kW），场景 3 为负（无动力）")
print("=" * 60)
