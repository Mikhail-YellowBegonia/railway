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


# ===== Case 2: 单切线约束圆弧 =====

PLANE_NORMAL = Vec3(0.0, 0.0, 1.0)  # 假定轨道在 XY 平面


def perp_xy(v: Vec3) -> Vec3:
    """XY 平面内逆时针 90° 旋转。"""
    return Vec3(-v.y, v.x, 0.0)


def solve_case2_arc(
    m1: Vec3, t1: Vec3, m2: Vec3
) -> tuple[Vec3, Vec3, Vec3, float] | None:
    """求一个以 M1 为切点（切于 T1）、通过 M2 的圆弧。

    返回 (center, B, arc_normal, radius) 或 None（不可解）。

    几何推导：
    - 圆心 O 必在过 M1 且垂直于 T1 的法线上：O = M1 + s * n，n = perp_xy(T1)
    - |O - M1| = |O - M2| = R
    - 解出 s = |d|² / (2 * n·d)，其中 d = M2 - M1
    - 当 n·d 趋于 0 时（M2 几乎落在 T1 延长线上）→ 半径无穷，返回 None 让上层退化为直线

    切线交点 B：M1 处切线 (M1 + k·T1) 与 M2 处切线 (M2 + k'·T2) 的交点。
    M2 处切向 T2 与 M2-O 垂直。
    """
    t1n = t1.normalize()
    if t1n.length() < 1e-9:
        return None

    n = perp_xy(t1n)  # M1 处法线（指向某一侧的圆心方向）
    d = m2 - m1
    nd = n.dot(d)

    if abs(nd) < 1e-9:
        # M2 几乎在 T1 延长线上 → 半径无穷
        return None

    s = d.length_squared() / (2.0 * nd)
    center = m1 + n * s
    radius = abs(s)

    # 圆弧法向：M1 处的"沿弧方向" T1 = arc_normal × (M1 - center) / radius
    # arc_normal 为 +Z 或 -Z，由 T1 与 (M1 - center) 的关系决定
    radius_vec = m1 - center
    # 平面 z 方向的 normal：满足 z × radius_vec = R * t1
    # 即 sign 取决于 (radius_vec × t1) · z 的符号
    cross_z = radius_vec.x * t1n.y - radius_vec.y * t1n.x
    arc_normal = Vec3(0.0, 0.0, 1.0 if cross_z > 0 else -1.0)

    # 求 B：两条切线 M1+k·T1 和 M2+k'·T2 的交点
    # M2 处切向 T2 = arc_normal × (M2 - center) / radius
    radius_vec_2 = m2 - center
    t2 = arc_normal.cross(radius_vec_2)
    t2_len = t2.length()
    if t2_len < 1e-9:
        return None
    t2 = t2 * (1.0 / t2_len)

    # 解 M1 + k * T1 = M2 + k' * T2  →  k * T1 - k' * T2 = d
    # 在 XY 平面内是 2x2 线性方程
    det = t1n.x * (-t2.y) - t1n.y * (-t2.x)
    if abs(det) < 1e-9:
        return None
    k = (d.x * (-t2.y) - d.y * (-t2.x)) / det
    b_point = m1 + t1n * k
    return center, b_point, arc_normal, radius


def is_t1_consistent_with_target(t1: Vec3, m1: Vec3, m2: Vec3) -> bool:
    """检查 T1 是否"指向" M2 一侧（即 (M2-M1)·T1 > 0）。

    若返回 False，意味着 M2 在 T1 的"背后"，按 Q2 决议应拒绝。
    """
    d = m2 - m1
    if d.length() < 1e-9:
        return False
    return d.normalize().dot(t1.normalize()) > 1e-6


def project_along_direction(m1: Vec3, t1: Vec3, m2: Vec3) -> Vec3:
    """计算 M2 在 (M1, T1) 射线上的投影点。

    用于"半径过大退化为直线"时确定终点：保证沿 T1 方向。
    """
    t1n = t1.normalize()
    d = m2 - m1
    proj_len = d.dot(t1n)
    return m1 + t1n * proj_len


# ===== 合并判定（DELETE 中间节点）=====

# 合并几何严格阈值。距离/位置类用绝对阈值；方向类用 cos(夹角) 接近 1。
MERGE_RADIUS_TOL = 1e-4         # 半径差
MERGE_POSITION_TOL = 1e-4       # 圆心距离
MERGE_DIR_DOT_MIN = 1.0 - 1e-6  # cos(夹角) 至少为此（约 0.08° 以内）


def can_merge_straight(
    a_pos: Vec3, mid_pos: Vec3, b_pos: Vec3
) -> bool:
    """两条直线相邻边能否合并为一条直线。

    几何条件：A→mid 与 mid→B 共线（同向，单位向量内积 ≈ 1）。
    """
    v1 = mid_pos - a_pos
    v2 = b_pos - mid_pos
    if v1.length() < 1e-9 or v2.length() < 1e-9:
        return False
    return v1.normalize().dot(v2.normalize()) >= MERGE_DIR_DOT_MIN


def can_merge_arcs(
    edge_1, edge_2, mid_pos: Vec3, a_pos: Vec3, b_pos: Vec3
) -> bool:
    """两条圆弧相邻边能否合并为一条圆弧。

    严格条件（缺一不可）：
    - 半径相等：|R1 - R2| < tol
    - 圆心重合：|center1 - center2| < tol
    - 同向：在 mid_pos 处，弧 1 的"沿 t 增大方向切线"指向 mid（从 a 来）；
      弧 2 的"沿 t 增大方向切线"从 mid 出发指向 b。两者夹角 ≈ 0。
    - normal 同向：cross 积同号（保证两弧绕同一圆转同方向，不会一个顺时针一个逆时针）
    """
    if abs(edge_1.arc_radius - edge_2.arc_radius) > MERGE_RADIUS_TOL:
        return False

    if edge_1.arc_center is None or edge_2.arc_center is None:
        return False
    if edge_1.arc_center.distance_to(edge_2.arc_center) > MERGE_POSITION_TOL:
        return False

    # normal 同向（XY 平面假设下，比较 z 分量符号）
    if edge_1.arc_normal is None or edge_2.arc_normal is None:
        return False
    if edge_1.arc_normal.dot(edge_2.arc_normal) < MERGE_DIR_DOT_MIN:
        return False

    # 切线连续性：mid_pos 处两弧切向应同向
    # 弧 1 在 mid_pos 处的"指向 b"切线 = 沿弧 1 从 a 到 mid 的延伸方向
    # 弧 2 在 mid_pos 处的"指向 b"切线 = 沿弧 2 从 mid 到 b 的方向
    t1 = tangent_at_arc_point_forward(edge_1, mid_pos)
    # edge_1 的 forward 方向是从 node_a → node_b；如果 mid 是 edge_1 的 node_b 那 forward
    # 已经就是"指向 b"方向；如果 mid 是 edge_1 的 node_a 则要反号。但调用方保证
    # mid 是被合并的中间节点，edge_1 的另一端是 a_pos。判断 a_pos 在 edge_1 上是 node_a
    # 还是 node_b 比较繁琐，等价检查：t1 应大致指向 (b_pos - mid_pos) 方向。
    flow_dir = (b_pos - mid_pos)
    if flow_dir.length() < 1e-9:
        return False
    flow = flow_dir.normalize()
    if t1.dot(flow) < 0:
        t1 = t1 * -1.0

    t2 = tangent_at_arc_point_forward(edge_2, mid_pos)
    if t2.dot(flow) < 0:
        t2 = t2 * -1.0

    return t1.dot(t2) >= MERGE_DIR_DOT_MIN


def merged_arc_b_point(
    edge_1, edge_2, a_pos: Vec3, b_pos: Vec3
) -> Vec3 | None:
    """两条等圆心等半径的弧合并后的新 B 点（切线交点）。

    用 a_pos 处的切线（指向 b 方向）和 b_pos 处的切线（指向 a 方向反向，即沿弧出发方向）
    求两条切线的交点。
    """
    # a_pos 在弧 1 上；先确定弧 1 在 a_pos 处的切向
    # 通过 tangent_at_arc_point_forward 取 forward 方向，再调整使其"远离 mid"——
    # 简化：直接以 a→b 的整体方向校正
    flow = b_pos - a_pos
    if flow.length() < 1e-9:
        return None
    flow_n = flow.normalize()

    t_a = tangent_at_arc_point_forward(edge_1, a_pos)
    if t_a.dot(flow_n) < 0:
        t_a = t_a * -1.0

    t_b = tangent_at_arc_point_forward(edge_2, b_pos)
    # b 处切线应指向 a 的反向；调整到与 flow 同向
    if t_b.dot(flow_n) < 0:
        t_b = t_b * -1.0

    # 求 a_pos + k * t_a = b_pos + k' * (-t_b) 的交点（B 应在两段切线的"前方交点"）
    # 即 a_pos + k*t_a - b_pos + k'*t_b = 0  →  k*t_a + k'*t_b = b_pos - a_pos
    det = t_a.x * t_b.y - t_a.y * t_b.x
    if abs(det) < 1e-9:
        return None
    d = b_pos - a_pos
    k = (d.x * t_b.y - d.y * t_b.x) / det
    return a_pos + t_a * k


# ===== Case 3: 等半径 Biarc 求解 =====

# Biarc 共线退化阈值：T1∥T2 同向且 d∥T1 时退化为直线
BIARC_COLLINEAR_DOT_MIN = 1.0 - 1e-6


def _biarc_arc_b_point(
    m_a: Vec3, t_a: Vec3, m_b: Vec3, t_b: Vec3
) -> Vec3 | None:
    """两条切线 (m_a, t_a) 和 (m_b, t_b) 的交点；用于反算弧的 B 点。

    解：m_a + k * t_a = m_b - k' * t_b（即 t_a 与 -t_b 在前方相交）
    """
    det = t_a.x * (-t_b.y) - t_a.y * (-t_b.x)
    if abs(det) < 1e-9:
        return None
    d = m_b - m_a
    k = (d.x * (-t_b.y) - d.y * (-t_b.x)) / det
    return m_a + t_a * k


def _biarc_arc_normal(center: Vec3, m_start: Vec3, t_start: Vec3) -> Vec3:
    """根据起点位置和切线，确定弧 normal（+Z 或 -Z）。"""
    radius_vec = m_start - center
    cross_z = radius_vec.x * t_start.y - radius_vec.y * t_start.x
    return Vec3(0.0, 0.0, 1.0 if cross_z > 0 else -1.0)


def _biarc_arc_sweep_angle(
    center: Vec3, m_start: Vec3, m_end: Vec3, arc_normal: Vec3
) -> float:
    """弧从 m_start 到 m_end 的扫角（沿 arc_normal 决定的方向，0 ≤ θ ≤ 2π）。"""
    v_start = m_start - center
    v_end = m_end - center
    # 平面内带符号角：normal·(v_start × v_end)
    cross_z = v_start.x * v_end.y - v_start.y * v_end.x
    cos_a = v_start.x * v_end.x + v_start.y * v_end.y
    r2 = v_start.length() * v_end.length()
    if r2 < 1e-12:
        return 0.0
    cos_n = max(-1.0, min(1.0, cos_a / r2))
    angle = math.acos(cos_n)  # [0, π]
    sign = arc_normal.z * (1.0 if cross_z >= 0 else -1.0)
    if sign < 0:
        angle = 2.0 * math.pi - angle
    return angle


def is_biarc_collinear_straight(t1: Vec3, t2: Vec3, m1: Vec3, m2: Vec3) -> bool:
    """T1∥T2 同向 且 d∥T1 → 共线直线 fast-path。

    满足时调用方应直接走 Case 1（M1→M2 直线）。
    """
    t1n = t1.normalize()
    t2n = t2.normalize()
    if t1n.dot(t2n) < BIARC_COLLINEAR_DOT_MIN:
        return False
    d = m2 - m1
    if d.length() < 1e-9:
        return False
    return d.normalize().dot(t1n) >= BIARC_COLLINEAR_DOT_MIN


def solve_biarc(
    m1: Vec3, t1: Vec3, m2: Vec3, t2: Vec3
) -> tuple[Vec3, Vec3, Vec3, Vec3, Vec3, float] | None:
    """等半径 Biarc 求解：两段弧 R1=R2=R，在中间点 G1 连续衔接 M1, M2。

    输入：M1 处切于 T1（指向 M2 一侧），M2 处切于 T2（指向 M1 一侧的反向，即"延伸方向"）。

    返回 `(m_mid, b1, b2, arc1_normal, arc2_normal, radius)` 或 `None`（不可解）。

    `b1, b2` 是两段弧的切线交点，配合 `Edge.geometry=[B]` 直接生成两段连续的 Edge。

    算法：
    - 弧 1 圆心 O1 = M1 + σ1·R·perp(T1)；弧 2 圆心 O2 = M2 + σ2·R·perp(T2)，σ∈{+1,-1}
    - G1 连续 ⇔ |O2 - O1| = 2R，且 M_mid = (O1 + O2)/2
    - 解关于 R 的二次方程，过滤 R ≤ 0 与"绕大圈"（任一弧扫角 > π）的解
    - 枚举 4 个 σ 组合，每组取较大 R 解；最终在所有合法解中取 R 最大者（曲率最小）
    """
    t1n = t1.normalize()
    t2n = t2.normalize()
    if t1n.length() < 1e-9 or t2n.length() < 1e-9:
        return None

    d = m2 - m1
    if d.length() < 1e-9:
        return None

    n1_base = perp_xy(t1n)  # σ1 = +1 时 O1 在 M1 + R·n1_base 处
    n2_base = perp_xy(t2n)

    best: tuple[Vec3, Vec3, Vec3, Vec3, Vec3, float] | None = None
    best_r = -1.0

    for sigma1 in (1.0, -1.0):
        for sigma2 in (1.0, -1.0):
            v1 = n1_base * sigma1
            v2 = n2_base * sigma2
            dv = v2 - v1
            # |d + R·dv|² = 4R²  →  (|dv|² - 4)·R² + 2(d·dv)·R + |d|² = 0
            a = dv.length_squared() - 4.0
            b = 2.0 * d.dot(dv)
            c = d.length_squared()

            # 候选 R：求所有 R > 0 解
            r_candidates: list[float] = []
            if abs(a) < 1e-12:
                # 退化为线性 b·R + c = 0
                if abs(b) > 1e-12:
                    r = -c / b
                    if r > 1e-9:
                        r_candidates.append(r)
            else:
                disc = b * b - 4.0 * a * c
                if disc < -1e-9:
                    continue
                disc = max(disc, 0.0)
                sqd = math.sqrt(disc)
                for r in ((-b + sqd) / (2.0 * a), (-b - sqd) / (2.0 * a)):
                    if r > 1e-9:
                        r_candidates.append(r)

            for r in r_candidates:
                o1 = m1 + v1 * r
                o2 = m2 + v2 * r
                m_mid = (o1 + o2) * 0.5

                # 验证：M_mid 在两弧上（半径都 ≈ r）
                r1_check = m_mid.distance_to(o1)
                r2_check = m_mid.distance_to(o2)
                if abs(r1_check - r) > 1e-4 or abs(r2_check - r) > 1e-4:
                    continue

                # 弧 1 的 normal：由 (M1 - O1) 与 T1 的关系决定
                an1 = _biarc_arc_normal(o1, m1, t1n)
                # 弧 2 的 normal：在 M_mid 处的切线必须与弧 1 在 M_mid 处一致；
                # 弧 2 起于 M_mid 终于 M2，T_mid 应等于"弧 1 在 M_mid 处的 forward"
                t_mid_arc1 = an1.cross(m_mid - o1)
                t_mid_arc1_len = t_mid_arc1.length()
                if t_mid_arc1_len < 1e-9:
                    continue
                t_mid = t_mid_arc1 * (1.0 / t_mid_arc1_len)
                an2 = _biarc_arc_normal(o2, m_mid, t_mid)

                # 验证弧 2 在 M2 处的切向 ≈ T2
                t2_check = an2.cross(m2 - o2)
                t2_check_len = t2_check.length()
                if t2_check_len < 1e-9:
                    continue
                t2_check = t2_check * (1.0 / t2_check_len)
                if t2_check.dot(t2n) < BIARC_COLLINEAR_DOT_MIN:
                    continue

                # 排除"绕大圈"：任一弧扫角 > π 视为不合理
                sweep1 = _biarc_arc_sweep_angle(o1, m1, m_mid, an1)
                sweep2 = _biarc_arc_sweep_angle(o2, m_mid, m2, an2)
                if sweep1 > math.pi + 1e-6 or sweep2 > math.pi + 1e-6:
                    continue

                # 反算 B 点
                b1 = _biarc_arc_b_point(m1, t1n, m_mid, t_mid)
                b2 = _biarc_arc_b_point(m_mid, t_mid, m2, t2n)
                if b1 is None or b2 is None:
                    continue

                if r > best_r:
                    best_r = r
                    best = (m_mid, b1, b2, an1, an2, r)

    return best
