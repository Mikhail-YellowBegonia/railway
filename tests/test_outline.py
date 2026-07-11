"""测试车厢轮廓计算（验证几何逻辑）。"""
from model.vec3 import Vec3
from model.kinematics import Pose

# 模拟车厢 pose
pos = Vec3(10.0, 5.0, 0.0)
heading = Vec3(1.0, 0.0, 0.0)  # 朝向 +X

# 车厢尺寸
wagon_length = 19.0  # 20m - 1m 车钩
wagon_width = 3.0

# 计算右方向（2D 右手系：heading 逆时针旋转 90°）
right = Vec3(-heading.y, heading.x, 0.0)
print(f"heading: {heading}")
print(f"right: {right}")

# 4 个角点
half_len = wagon_length / 2.0
half_wid = wagon_width / 2.0

corners = [
    pos + heading * half_len + right * half_wid,   # 前左
    pos + heading * half_len - right * half_wid,   # 前右
    pos - heading * half_len - right * half_wid,   # 后右
    pos - heading * half_len + right * half_wid,   # 后左
]

print(f"\n车厢中心: {pos}")
print(f"车厢尺寸: {wagon_length} × {wagon_width} m")
print(f"\n角点（世界坐标）：")
for i, c in enumerate(corners):
    print(f"  {i}: ({c.x:.2f}, {c.y:.2f})")

# 验证：前左和前右应该在 x=19.5，y 相差 3m
assert abs(corners[0].x - 19.5) < 0.01
assert abs(corners[1].x - 19.5) < 0.01
assert abs(corners[0].y - corners[1].y - 3.0) < 0.01

print("\n✅ 车厢轮廓几何计算正确")
