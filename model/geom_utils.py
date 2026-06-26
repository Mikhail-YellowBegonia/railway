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


def solve_case2_composite(
    m1: Vec3, t1: Vec3, m2: Vec3, max_radius: float
) -> tuple[Vec3, Vec3, Vec3, Vec3, float] | None:
    """Case 2 半径超限时的复合解：固定半径 max_radius 的弧 + 直线收尾。

    返回 (P_mid, B, arc_normal, tail_dir, radius) 或 None（不可解）。

    - 弧从 M1 切于 T1 出发，半径恰为 max_radius，弯到中间点 P_mid
    - 直线段从 P_mid 切线连续地延伸到 M2（M2 必须在 P_mid 处切线的"前方"）
    - B 是弧的切线交点（geometry = [B] 写入 Edge）
    - tail_dir 是直线段的单位方向（从 P_mid 指向 M2）

    几何推导：
    - 选圆心方向 σ ∈ {+1, -1}：使 M2 落在弧的"凹侧"（与 perp(T1) · (M2-M1) 同号）
    - O = M1 + σ · R · perp(T1)
    - P 在圆上 → P = O + R · u（|u|=1）
    - P 处切线 t_P = arc_normal × u 必须沿 (M2 - P) 方向
      ⇔ (M2 - P) ⊥ (P - O)
      ⇔ (M2 - O) · u - R = 0
    - 设 v = M2 - O，|v|² = vx² + vy²，方程 v·u = R
    - 与 |u|=1 联立两个解；选弧角 θ = atan2(...) 最小（且 > 0、且 M2 在切线前方）者

    退化条件：
    - |M2 - O| < R → M2 在圆内 → 直线段不存在 → 返回 None（让上层降级到纯直线）
    - 弧角 ≥ π → 拒绝（"绕大圈"，不合直觉）
    """
    t1n = t1.normalize()
    if t1n.length() < 1e-9:
        return None
    if max_radius <= 0:
        return None

    n = perp_xy(t1n)  # M1 处法线（圆心方向）
    d = m2 - m1
    if d.length() < 1e-9:
        return None

    # 选择圆心侧 σ：M2 在哪一侧 perp(T1) 就往那一侧弯
    nd = n.dot(d)
    sigma = 1.0 if nd >= 0 else -1.0
    n_signed = n * sigma

    O = m1 + n_signed * max_radius
    R = max_radius

    v = m2 - O
    v_len_sq = v.length_squared()
    if v_len_sq < R * R + 1e-12:
        # M2 在圆上或圆内 → 不存在外切线 → 复合无解
        return None

    # 求 u 满足 v·u = R, |u|=1 → 在以 v_hat 为参考的坐标系下
    # u = (R/|v|) · v_hat ± sqrt(1 - R²/|v|²) · perp(v_hat)
    v_len = math.sqrt(v_len_sq)
    v_hat = v * (1.0 / v_len)
    cos_a = R / v_len
    sin_a = math.sqrt(max(0.0, 1.0 - cos_a * cos_a))
    perp_v = perp_xy(v_hat)

    # M1 处径向（从 O 指向 M1）：起始 u₀
    u_start = (m1 - O) * (1.0 / R)

    candidates: list[tuple[float, Vec3, Vec3]] = []  # (theta, P, tail_dir_unit)
    for s in (+1.0, -1.0):
        u = v_hat * cos_a + perp_v * (sin_a * s)
        P = O + u * R

        # 弧角 θ：从 u_start 转到 u 的有向角度（按 arc_normal 旋转方向）
        # arc_normal 同 solve_case2_arc：sign 取决于 (m1 - O) × t1 的 z 分量
        cross_z = (m1 - O).x * t1n.y - (m1 - O).y * t1n.x
        arc_normal_z = 1.0 if cross_z > 0 else -1.0
        # 有向角：u_start → u 沿 arc_normal_z 方向旋转
        cos_t = u_start.dot(u)
        cross_u = u_start.x * u.y - u_start.y * u.x  # z 分量
        sin_t = cross_u * arc_normal_z
        theta = math.atan2(sin_t, cos_t)
        if theta <= 1e-9:
            continue  # 0 度或反向 → 无效弧
        if theta >= math.pi - 1e-9:
            continue  # 绕大圈

        # P 处切线 t_P = arc_normal × (P - O) / R，必须指向 M2 一侧
        radius_vec = P - O
        # arc_normal × radius_vec：z=1 时是 (-ry, rx)；z=-1 时反号
        if arc_normal_z > 0:
            t_p = Vec3(-radius_vec.y, radius_vec.x, 0.0) * (1.0 / R)
        else:
            t_p = Vec3(radius_vec.y, -radius_vec.x, 0.0) * (1.0 / R)

        m2_minus_p = m2 - P
        if t_p.dot(m2_minus_p) < 1e-9:
            continue  # 切线方向与 (M2 - P) 不同向 → 直线段会反向

        # tail_dir：P → M2 单位方向，应与 t_p 同向
        tail_len = m2_minus_p.length()
        if tail_len < 1e-9:
            continue  # P 与 M2 重合 → 不存在直线段
        tail_dir = m2_minus_p * (1.0 / tail_len)

        candidates.append((theta, P, tail_dir))

    if not candidates:
        return None

    # 选弧角最小者（曲率方向与 σ 配合下，最自然的接入路径）
    candidates.sort(key=lambda x: x[0])
    theta, P, tail_dir = candidates[0]

    # 确定 arc_normal（同 solve_case2_arc 规则）
    cross_z = (m1 - O).x * t1n.y - (m1 - O).y * t1n.x
    arc_normal = Vec3(0.0, 0.0, 1.0 if cross_z > 0 else -1.0)

    # 求 B：M1 处切线 (M1 + k·T1) 与 P 处切线 (P + k'·tail_dir) 的交点
    # 解 k·T1 - k'·tail_dir = P - M1
    rhs = P - m1
    det = t1n.x * (-tail_dir.y) - t1n.y * (-tail_dir.x)
    if abs(det) < 1e-9:
        return None
    k = (rhs.x * (-tail_dir.y) - rhs.y * (-tail_dir.x)) / det
    b_point = m1 + t1n * k

    return P, b_point, arc_normal, tail_dir, R


def solve_case2t(
    m1: Vec3,
    t1: Vec3,
    m2_mouse: Vec3,
    p0: Vec3,
    t2_dir: Vec3,
    max_radius: float,
) -> tuple[Vec3, Vec3, Vec3, float] | None:
    """单切线弧 Case 2T（§10.5）：求一段弧 *切于 (M1, T1)* 且 *切于直线 L=(P0, T2_dir)*。

    与 solve_case2_arc 的区别：精确接入点由算法决定，不强制等于 m2_mouse。
    用户的 m2_mouse 仅用作"二选一"的偏好信号（哪个解更接近鼠标）。

    返回 (m2_actual, B, arc_normal, radius) 或 None（不可解 / 半径超限 / Q2 不通过）。

    几何：
    - 圆心 O = M1 + s · perp(T1)，半径 R = |s|
    - O 到 L 的距离 = R → 二次方程 (α²-1)·s² + 2αβ·s + β² = 0
      其中 α = T1·T2_hat,  β = perp(T2_hat)·(M1 - P0)
    - 两根 s = -β/(α+1) 或 s = -β/(α-1)，分别对应 L 两侧的切圆
    - 切点 P = O - ((O - P0)·perp(T2_hat)) · perp(T2_hat)（圆心到 L 的垂足）

    退化：
    - α=±1（T1 与 L 平行 / 反平行）→ 切点处弧角必为 π（半圆），不符合
      建造逻辑 → 直接拒绝（None）
    - R > max_radius → 拒绝
    - Q2: T1·(P - M1) < 0（严格反向）→ 拒绝；允许 90° 转弯（dot=0）
    """
    t1n = t1.normalize()
    t2n = t2_dir.normalize()
    if t1n.length() < 1e-9 or t2n.length() < 1e-9:
        return None
    if max_radius <= 0:
        return None

    m_perp = perp_xy(t2n)  # L 的法线方向
    alpha = t1n.dot(t2n)
    beta = m_perp.dot(m1 - p0)

    # α=±1：T1 与 L 共线 → 切点处弧角为 π（半圆），无法用单弧 [B] 表示，
    # 且修建半圆不符合建造逻辑 → 直接拒绝。
    if abs(abs(alpha) - 1.0) < 1e-9:
        return None

    candidates_s: list[float] = [
        -beta / (alpha + 1.0),
        -beta / (alpha - 1.0),
    ]

    n1 = perp_xy(t1n)

    # 在所有合法解里挑离 m2_mouse 最近的切点
    best: tuple[float, Vec3, Vec3, float] | None = None  # (s, O, P, R)
    best_dist = float("inf")

    for s in candidates_s:
        if abs(s) < 1e-9:
            continue  # 退化为零半径
        radius = abs(s)
        if radius > max_radius:
            continue

        O = m1 + n1 * s
        # 切点 P：O 到 L 的垂足 = O - ((O - P0)·m_perp) · m_perp
        d_signed = (O - p0).dot(m_perp)
        P = O - m_perp * d_signed

        # Q2 检查：T1 · (P - M1) >= 0（允许 90° 转弯，仅拒绝严格反向）
        dir_to_p = P - m1
        if dir_to_p.length() < 1e-9:
            continue
        if t1n.dot(dir_to_p) < -1e-6:
            continue

        dist = P.distance_to(m2_mouse)
        if dist < best_dist:
            best_dist = dist
            best = (s, O, P, radius)

    if best is None:
        return None

    _, O, P, radius = best

    # arc_normal：与 solve_case2_arc 同规则
    radius_vec_m1 = m1 - O
    cross_z = radius_vec_m1.x * t1n.y - radius_vec_m1.y * t1n.x
    arc_normal = Vec3(0.0, 0.0, 1.0 if cross_z > 0 else -1.0)

    # P 处切线 t_P = arc_normal × (P - O) / R
    radius_vec_p = P - O
    if arc_normal.z > 0:
        t_p = Vec3(-radius_vec_p.y, radius_vec_p.x, 0.0) * (1.0 / radius)
    else:
        t_p = Vec3(radius_vec_p.y, -radius_vec_p.x, 0.0) * (1.0 / radius)

    # 求 B：M1 处切线与 P 处切线的交点
    rhs = P - m1
    det = t1n.x * (-t_p.y) - t1n.y * (-t_p.x)
    if abs(det) < 1e-9:
        return None
    k = (rhs.x * (-t_p.y) - rhs.y * (-t_p.x)) / det
    b_point = m1 + t1n * k

    return P, b_point, arc_normal, radius


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


# ===== Edge 截断 =====


def split_arc_b_points(
    edge: Edge, node_a: Node, node_b: Node, p: Vec3
) -> tuple[Vec3, Vec3] | None:
    """计算把弧在点 p 处分成两段后，两段子弧的 B 点（切线交点）。

    保证两段同心同半径，且在 p 处切线连续。
    返回 `(B1, B2)`，其中 B1 是 node_a→p 段的 B 点，B2 是 p→node_b 段的 B 点。
    返回 None 表示几何不可解（例如 p 与端点重合）。
    """
    if not edge.is_arc:
        return None

    # 端点切线（指向 b 方向）
    t_a = tangent_at_arc_point_forward(edge, node_a.position)
    t_p = tangent_at_arc_point_forward(edge, p)
    t_b = tangent_at_arc_point_forward(edge, node_b.position)

    # B1 = (node_a 处切线) ∩ (p 处反向切线) — 两条向 B1 汇聚的射线
    # 用与 _biarc_arc_b_point 相同的解法：m_a + k·t_a = m_p - k'·t_p
    b1 = _arc_split_intersect(node_a.position, t_a, p, t_p)
    b2 = _arc_split_intersect(p, t_p, node_b.position, t_b)
    if b1 is None or b2 is None:
        return None
    return b1, b2


def _arc_split_intersect(
    m_a: Vec3, t_a: Vec3, m_b: Vec3, t_b: Vec3
) -> Vec3 | None:
    """两条切线 (m_a, t_a) 和 (m_b, t_b) 在前方的交点。

    解 m_a + k * t_a = m_b - k' * t_b（即 t_a 与 -t_b 在前方相交）。
    """
    det = t_a.x * (-t_b.y) - t_a.y * (-t_b.x)
    if abs(det) < 1e-9:
        return None
    d = m_b - m_a
    k = (d.x * (-t_b.y) - d.y * (-t_b.x)) / det
    return m_a + t_a * k
