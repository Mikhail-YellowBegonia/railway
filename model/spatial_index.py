from __future__ import annotations

import math
from typing import TYPE_CHECKING

from model.vec3 import Vec3

if TYPE_CHECKING:
    from model.rail_network import Node, Edge

# 瓦片尺寸（世界单位 = 米）。设计依据见 docs/editor.md §11（空间索引）。
# 起点 50 m：与典型 Edge 长度（中位数 ~40 m）和查询半径（吸附阈值 <10 m）平衡。
TILE_SIZE = 50.0

TileKey = tuple[int, int]  # (tile_x, tile_y)


class SpatialIndex:
    """空间分区索引（uniform grid / 空间哈希）。

    把 XY 平面切成边长 TILE_SIZE 的方格，每格记录落在其中的 Node/Edge。
    查询时只扫描光标邻近瓦片，把 O(N) 全扫描降为 O(k)（k = 邻近元素数）。

    设计约束（docs/editor.md §11 归档，原设计稿 docs/tiling.md 已在实施完成后删除）：
    - 旁挂索引：只读辅助结构，不替代 RailNetwork 的 dict 存储
    - 增量更新：insert/remove 只碰涉及的瓦片，O(元素占格数)，不整表重建
    - 稀疏存储：空瓦片不存在于 dict，查询空区域只是若干次 dict miss
    """

    def __init__(self, tile_size: float = TILE_SIZE) -> None:
        self.tile_size = tile_size
        # 瓦片 -> Node/Edge ID 集合（稀疏存储，空格不存在）
        self.node_tiles: dict[TileKey, set[int]] = {}
        self.edge_tiles: dict[TileKey, set[int]] = {}
        # Edge ID -> 它占的瓦片集合（用于增量删除）
        self._edge_footprint: dict[int, set[TileKey]] = {}

    def insert_node(self, node_id: int, pos: Vec3) -> None:
        """插入 Node（单点，占一格）。"""
        tile = self._tile_of(pos)
        if tile not in self.node_tiles:
            self.node_tiles[tile] = set()
        self.node_tiles[tile].add(node_id)

    def remove_node(self, node_id: int, pos: Vec3) -> None:
        """删除 Node。只从它所在的瓦片里移除，不做空桶回收（见 docs/editor.md §11 归档）。"""
        tile = self._tile_of(pos)
        if tile in self.node_tiles:
            self.node_tiles[tile].discard(node_id)
            # 不做空桶回收（当前决策），避免频繁 del 的 rehash 代价

    def insert_edge(self, edge_id: int, edge: Edge, node_a: Node, node_b: Node) -> None:
        """插入 Edge。用包围盒计算 footprint（最粗策略）。"""
        footprint = self._compute_edge_footprint(edge, node_a, node_b)
        self._edge_footprint[edge_id] = footprint
        for tile in footprint:
            if tile not in self.edge_tiles:
                self.edge_tiles[tile] = set()
            self.edge_tiles[tile].add(edge_id)

    def remove_edge(self, edge_id: int) -> None:
        """删除 Edge。靠 _edge_footprint 精确知道要清哪些格，增量 O(占格数)。"""
        footprint = self._edge_footprint.pop(edge_id, set())
        for tile in footprint:
            if tile in self.edge_tiles:
                self.edge_tiles[tile].discard(edge_id)
                # 不做空桶回收（当前决策）

    def nearby_node_ids(self, pos: Vec3, radius: float) -> set[int]:
        """返回距 pos 世界距离 <= radius 的 Node 候选集（可能略多，调用方需精算）。"""
        result: set[int] = set()
        for tile in self._tiles_in_radius(pos, radius):
            result |= self.node_tiles.get(tile, set())
        return result

    def nearby_edge_ids(self, pos: Vec3, radius: float) -> set[int]:
        """返回距 pos 世界距离 <= radius 的 Edge 候选集（可能略多，调用方需精算）。"""
        result: set[int] = set()
        for tile in self._tiles_in_radius(pos, radius):
            result |= self.edge_tiles.get(tile, set())
        return result

    def _tile_of(self, pos: Vec3) -> TileKey:
        """世界坐标 -> 瓦片坐标（向下取整）。"""
        tx = math.floor(pos.x / self.tile_size)
        ty = math.floor(pos.y / self.tile_size)
        return (tx, ty)

    def _tiles_in_radius(self, pos: Vec3, radius: float) -> list[TileKey]:
        """返回距 pos 世界距离 <= radius 的所有瓦片（矩形邻域，略保守）。"""
        r_tiles = math.ceil(radius / self.tile_size)
        cx, cy = self._tile_of(pos)
        tiles = []
        for tx in range(cx - r_tiles, cx + r_tiles + 1):
            for ty in range(cy - r_tiles, cy + r_tiles + 1):
                tiles.append((tx, ty))
        return tiles

    def _compute_edge_footprint(self, edge: Edge, node_a: Node, node_b: Node) -> set[TileKey]:
        """计算 Edge 占的瓦片集合。使用包围盒（最粗策略）。

        - 直线：端点包围盒覆盖的瓦片
        - 圆弧：弧段包围盒（由圆心、半径、起止角计算）覆盖的瓦片

        包围盒登记偏多但不影响正确性——查询命中后仍会做精确距离判定。
        若性能验证发现长边占几十格导致查询膨胀，可收紧到光栅化（原设计稿 §9 的备选
        方案，稿子已随实施完成归档删除）。
        """
        if edge.is_arc and edge.arc_center is not None:
            # 弧段包围盒：从圆心、半径、起止方向算出
            aabb_min, aabb_max = self._arc_aabb(edge, node_a, node_b)
        else:
            # 直线包围盒：两端点的 min/max
            aabb_min = Vec3(
                min(node_a.position.x, node_b.position.x),
                min(node_a.position.y, node_b.position.y),
                0.0
            )
            aabb_max = Vec3(
                max(node_a.position.x, node_b.position.x),
                max(node_a.position.y, node_b.position.y),
                0.0
            )

        # 包围盒覆盖的瓦片范围
        min_tile = self._tile_of(aabb_min)
        max_tile = self._tile_of(aabb_max)
        footprint: set[TileKey] = set()
        for tx in range(min_tile[0], max_tile[0] + 1):
            for ty in range(min_tile[1], max_tile[1] + 1):
                footprint.add((tx, ty))
        return footprint

    def _arc_aabb(self, edge: Edge, node_a: Node, node_b: Node) -> tuple[Vec3, Vec3]:
        """计算圆弧的包围盒（XY 平面）。

        圆弧包围盒需考虑：端点 + 弧扫过的极值点（取决于起止角是否跨越坐标轴）。
        这里用保守估计：圆心 ± 半径，再与两端点取 min/max。
        （若未来需精确收紧，可判定扫角是否跨越 0°/90°/180°/270° 四个极值角）。
        """
        center = edge.arc_center
        r = edge.arc_radius
        # 保守包围盒：整圆
        circle_min = Vec3(center.x - r, center.y - r, 0.0)
        circle_max = Vec3(center.x + r, center.y + r, 0.0)
        # 与端点取交集（弧段不可能超出整圆）
        aabb_min = Vec3(
            min(circle_min.x, node_a.position.x, node_b.position.x),
            min(circle_min.y, node_a.position.y, node_b.position.y),
            0.0
        )
        aabb_max = Vec3(
            max(circle_max.x, node_a.position.x, node_b.position.x),
            max(circle_max.y, node_a.position.y, node_b.position.y),
            0.0
        )
        return aabb_min, aabb_max
