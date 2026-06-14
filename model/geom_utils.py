"""轨道几何工具：投影、切线、弧旋转等。

约定：
- 所有计算假定轨道在 XY 平面（Z 仅高程）。
- 圆弧 normal 方向与 RailNetwork._compute_arc 保持一致。
- 切线分 forward / reverse 两种：forward 指 t 增大方向（即从 node_a 沿弧到 node_b 的方向）。
"""
from __future__ import annotations

import math

from model.rail_network import Edge, Node
from model.vec3 import Vec3


def project_point_on_edge(
    p: Vec3, edge: Edge, node_a: Node, node_b: Node
) -> tuple[float, Vec3, float]:
    """点到边的投影。返回 (t, projected_position, distance)，t ∈ [0, 1]。"""
    if edge.is_arc:
        return project_on_arc(p, edge, node_a, node_b)
    return project_on_segment(p, node_a.position, node_b.position)


def project_on_segment(p: Vec3, a: Vec3, b: Vec3) -> tuple[float, Vec3, float]:
    """点到线段的投影（夹紧到端点之间）。"""
    ab = b - a
    ab_len_sq = ab.length_squared()
    if ab_len_sq == 0.0:
        return 0.0, a, p.distance_to(a)
    t = max(0.0, min(1.0, (p - a).dot(ab) / ab_len_sq))
    proj = a + ab * t
    return t, proj, p.distance_to(proj)


def project_on_arc(
    p: Vec3, edge: Edge, node_a: Node, node_b: Node
) -> tuple[float, Vec3, float]:
    """点到圆弧的投影（夹紧到弧的角度范围内）。

    利用 edge 的 arc_center / arc_radius / arc_angle_rad / arc_start_dir / arc_normal。
    """
    center = edge.arc_center
    radius = edge.arc_radius
    angle_total = edge.arc_angle_rad
    start_dir = edge.arc_start_dir
    normal = edge.arc_normal

    if center is None or start_dir is None or normal is None or abs(angle_total) < 1e-12:
        # 退化为直线段
        return project_on_segment(p, node_a.position, node_b.position)

    to_p = p - center
    if to_p.length() < 1e-12:
        # p 与圆心重合：取 a 端为投影
        return 0.0, node_a.position, radius

    on_circle_dir = to_p.normalize()
    # 计算从 start_dir 到 on_circle_dir 的有符号角度（绕 normal）
    cos_a = start_dir.dot(on_circle_dir)
    sin_a = start_dir.cross(on_circle_dir).dot(normal)
    angle_p = math.atan2(sin_a, cos_a)  # ∈ [-π, π]

    # 夹紧到 [0, angle_total] 或 [angle_total, 0]（angle_total 可正可负）
    if angle_total >= 0:
        clamped = max(0.0, min(angle_total, angle_p))
    else:
        clamped = min(0.0, max(angle_total, angle_p))

    rotated = rotate_around_axis(start_dir, normal, clamped)
    projected = center + rotated * radius
    t = clamped / angle_total
    return t, projected, p.distance_to(projected)


def tangent_along_edge(
    edge: Edge, node_a: Node, node_b: Node, t: float
) -> Vec3:
    """边上参数 t 处的 forward 切线方向（从 node_a 指向 node_b 的方向）。"""
    if not edge.is_arc:
        diff = node_b.position - node_a.position
        if diff.length() < 1e-12:
            return Vec3(1.0, 0.0, 0.0)
        return diff.normalize()

    angle_at_t = edge.arc_angle_rad * t
    rotated_dir = rotate_around_axis(edge.arc_start_dir, edge.arc_normal, angle_at_t)
    point = edge.arc_center + rotated_dir * edge.arc_radius
    return tangent_at_arc_point_forward(edge, point)


def tangent_at_arc_point_forward(edge: Edge, point: Vec3) -> Vec3:
    """圆弧在某点处的 forward 切线方向（沿 t 增大方向）。

    forward 方向 = sign(arc_angle_rad) * (arc_normal × radius_vec) / radius
    """
    radius_vec = point - edge.arc_center
    tangent = edge.arc_normal.cross(radius_vec)
    if tangent.length() < 1e-12:
        return Vec3(1.0, 0.0, 0.0)
    tangent = tangent.normalize()
    if edge.arc_angle_rad < 0:
        tangent = tangent * -1.0
    return tangent


def rotate_around_axis(v: Vec3, axis: Vec3, angle: float) -> Vec3:
    """Rodrigues 旋转公式，绕单位化后的 axis 旋转 v。"""
    if axis.length_squared() == 0.0:
        return v
    k = axis.normalize()
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return v * cos_a + k.cross(v) * sin_a + k * (k.dot(v)) * (1.0 - cos_a)
