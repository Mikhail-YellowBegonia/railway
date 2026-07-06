from __future__ import annotations

import math
from dataclasses import dataclass

from model.rail_network import _compute_arc
from model.vec3 import Vec3


@dataclass
class SegmentMetric:
    """建造预览中单个几何段的度量（用于 HUD 展示）。

    kind: "line" 或 "arc"。直线段 radius/angle_deg 为 None。
    单位：length 为米，radius 为米，angle_deg 为度。
    """
    kind: str
    length: float
    radius: float | None = None
    angle_deg: float | None = None


def _straight_metric(a: Vec3, b: Vec3) -> SegmentMetric:
    return SegmentMetric(kind="line", length=a.distance_to(b))


def _arc_metric(a: Vec3, b: Vec3, c: Vec3) -> SegmentMetric:
    """由三点（起点、切线交点 B、终点）反算弧的长度/半径/圆心角。

    _compute_arc 返回 (center, radius, angle, normal)，angle 为有符号圆心角。
    不可解（退化为直线）时按直线处理。
    """
    result = _compute_arc(a, b, c)
    if result is None:
        return _straight_metric(a, c)
    _center, radius, angle, _normal = result
    length = radius * abs(angle)
    return SegmentMetric(
        kind="arc",
        length=length,
        radius=radius,
        angle_deg=math.degrees(abs(angle)),
    )


def compute_metrics(preview) -> list[SegmentMetric]:
    """从 PreviewGeometry 提取分段度量列表。

    - Case 1 / Case 2 退化：单直线段
    - Case 2 / Case 5：单弧段
    - Case 3 Biarc：两段弧
    - Case 4 复合：弧段 + 直线段
    无效预览返回空列表。
    """
    if preview is None or not preview.valid:
        return []

    case = preview.case

    if case == 3:
        # Biarc：两段弧（M1→B1→mid, mid→B2→M2）
        if (
            preview.biarc_mid is None
            or not preview.biarc_geom_1
            or not preview.biarc_geom_2
        ):
            return []
        return [
            _arc_metric(preview.m1, preview.biarc_geom_1[0], preview.biarc_mid),
            _arc_metric(preview.biarc_mid, preview.biarc_geom_2[0], preview.m2),
        ]

    if case == 4:
        # 复合：弧段（M1→B→mid）+ 直线段（mid→M2）
        if preview.composite_mid is None or not preview.composite_arc_geom:
            return []
        return [
            _arc_metric(preview.m1, preview.composite_arc_geom[0], preview.composite_mid),
            _straight_metric(preview.composite_mid, preview.m2),
        ]

    if preview.edge_geometry:
        # Case 2 / Case 5：单弧（M1→B→M2）
        return [_arc_metric(preview.m1, preview.edge_geometry[0], preview.m2)]

    # Case 1 或退化直线
    return [_straight_metric(preview.m1, preview.m2)]


def format_lines(metrics: list[SegmentMetric]) -> list[str]:
    """把度量格式化为 HUD 显示行。多段时逐段列出并附总长。

    使用纯 ASCII 文本：pygame 默认字体不含中文/特殊字形（见问题反馈），
    避免出现方框。角度用 deg，半径用 R，长度用 L。
    """
    if not metrics:
        return []

    lines: list[str] = []
    multi = len(metrics) > 1
    for i, m in enumerate(metrics):
        prefix = f"[{i + 1}] " if multi else ""
        if m.kind == "line":
            lines.append(f"{prefix}Line  L {m.length:.1f}m")
        else:
            lines.append(
                f"{prefix}Arc  L {m.length:.1f}m  R {m.radius:.1f}m  "
                f"{m.angle_deg:.1f} deg"
            )

    if multi:
        total = sum(m.length for m in metrics)
        lines.append(f"Total {total:.1f}m")

    return lines
