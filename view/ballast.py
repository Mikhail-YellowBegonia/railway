from __future__ import annotations

from model.rail_network import Edge, Node, RailNetwork
from model.vec3 import Vec3

# Ballast（道床）图层：本轮唯一实现的测试图像图层。
# 带宽 4 m（半宽 2 m），见 docs/editor.md §12.1。世界单位（米），渲染时乘 scale。
BALLAST_HALF_WIDTH = 2.0

# 弧带按逻辑弧的采样段数生成边界（与 Edge.sample_arc_points 对齐即可）。
ARC_SEGMENTS = 30


def _perp_xy(v: Vec3) -> Vec3:
    """XY 平面内左侧法向（逆时针 90°）。零向量返回零向量。"""
    if v.length() < 1e-12:
        return Vec3()
    n = v.normalize()
    return Vec3(-n.y, n.x, 0.0)


def _straight_band(a: Vec3, b: Vec3, half: float) -> list[Vec3]:
    """直线段的道床带多边形（世界坐标，四点，顺序成环）。"""
    n = _perp_xy(b - a)
    if n.length() < 1e-12:
        return []
    off = n * half
    return [a + off, b + off, b - off, a - off]


def _arc_band(edge: Edge, half: float) -> list[Vec3]:
    """弧段的道床带多边形：外边界正向采样 + 内边界逆向采样，成环。

    以圆心为基准，把弧的采样点沿半径方向向外/向内各偏移 half，
    分别得到 R+half 与 R-half 上的点列，拼成一个环带多边形。
    """
    pts = edge.sample_arc_points(ARC_SEGMENTS)
    if len(pts) < 2 or edge.arc_center is None:
        return []

    center = edge.arc_center
    outer: list[Vec3] = []
    inner: list[Vec3] = []
    for p in pts:
        radial = p - center
        if radial.length() < 1e-12:
            continue
        rn = radial.normalize()
        outer.append(p + rn * half)
        inner.append(p - rn * half)

    if len(outer) < 2 or len(inner) < 2:
        return []
    # 外边界正序 + 内边界逆序 → 闭合环
    return outer + inner[::-1]


def edge_ballast_polygon(edge: Edge, node_a: Node, node_b: Node) -> list[Vec3]:
    """返回单条边的道床带多边形（世界坐标点列）。空列表表示无法生成。"""
    half = BALLAST_HALF_WIDTH
    if edge.is_arc:
        poly = _arc_band(edge, half)
        if poly:
            return poly
        # 弧退化时回退为直线带
    return _straight_band(node_a.position, node_b.position, half)


def iter_ballast_polygons(network: RailNetwork):
    """遍历网络所有边，产出 (edge, 世界坐标多边形) 供渲染。"""
    for edge in network.edges.values():
        node_a = network.nodes[edge.node_a_id]
        node_b = network.nodes[edge.node_b_id]
        poly = edge_ballast_polygon(edge, node_a, node_b)
        if len(poly) >= 3:
            yield edge, poly
