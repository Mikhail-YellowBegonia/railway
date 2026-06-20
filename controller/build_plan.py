from __future__ import annotations

from dataclasses import dataclass, field

from model.vec3 import Vec3


@dataclass
class ConstructionPlan:
    """建造计划：从 M1 到 M2 的一次完整建造操作的描述"""
    case: int                                  # 1=直线, 2=单弧, 3=Biarc, 4=弧+直线复合
    m1: Vec3
    m2: Vec3
    node_a_id: int | None = None               # M1 端的 Node ID（None 则需新建）
    node_b_id: int | None = None               # M2 端的 Node ID（None 则需新建）
    edge_geometry: list[Vec3] = field(default_factory=list)  # [] 直线, [B] 弧
    split_edge_id: int | None = None           # 需要截断的 Edge ID
    split_at_t: float | None = None            # 截断参数 t ∈ [0,1]
    valid: bool = True                         # 几何是否合法
    # Case 3 (Biarc) 专用：方案 A 双 Edge + 中间 Node
    biarc_mid: Vec3 | None = None              # 中间节点位置 M_mid
    biarc_geom_1: list[Vec3] | None = None     # 弧 1 的 geometry（[B1]）
    biarc_geom_2: list[Vec3] | None = None     # 弧 2 的 geometry（[B2]）
    # Case 4 (弧+直线复合，§10.3) 专用：双 Edge + 中间 Node
    composite_mid: Vec3 | None = None          # 中间节点（弧的终点 = 直线的起点）
    composite_arc_geom: list[Vec3] | None = None    # 弧段 geometry [B]
    composite_tail_geom: list[Vec3] | None = None   # 直线段 geometry []


@dataclass
class PreviewGeometry:
    """预览几何：BUILD_ACTIVE 状态下渲染待建轨道的描述"""
    m1: Vec3
    m2: Vec3
    case: int                                  # 1, 2, 3, or 4
    edge_geometry: list[Vec3] = field(default_factory=list)
    valid: bool = True
    # Case 3 预览
    biarc_mid: Vec3 | None = None
    biarc_geom_1: list[Vec3] | None = None
    biarc_geom_2: list[Vec3] | None = None
    # Case 4 预览
    composite_mid: Vec3 | None = None
    composite_arc_geom: list[Vec3] | None = None
    composite_tail_geom: list[Vec3] | None = None
