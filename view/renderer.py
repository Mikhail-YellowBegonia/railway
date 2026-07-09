from __future__ import annotations

import math

import pygame

from model.rail_network import RailNetwork
from model.vec3 import Vec3
from view.camera import Camera
from view.grid import draw_grid
from view.ballast import iter_ballast_polygons
from view.build_metrics import compute_metrics, format_lines
from controller.editor import EditMode, BuildState, Editor

COLOR_BG = (30, 30, 30)
COLOR_EDGE_STRAIGHT = (180, 180, 180)
COLOR_EDGE_ARC = (80, 160, 220)
COLOR_TANGENT = (80, 80, 60)
COLOR_TANGENT_POINT = (100, 100, 70)
COLOR_CENTER = (60, 80, 60)
COLOR_NODE: dict[int, tuple[int, int, int]] = {
    1: (220, 60, 60),
    2: (200, 200, 200),
    3: (220, 200, 60),
    4: (60, 200, 100),
}
COLOR_HOVER = (255, 255, 100)
COLOR_HOVER_EDGE = (255, 200, 80)
COLOR_TEXT = (200, 200, 200)
COLOR_PREVIEW = (120, 220, 255)
COLOR_PREVIEW_INVALID = (255, 80, 80)
COLOR_M1_ANCHOR = (100, 255, 100)
COLOR_WARNING = (255, 60, 60)
COLOR_CASE2T_ENTRY = (255, 100, 255)  # §10.5 算法回算的接入点标记（品红）
COLOR_HUD_BG = (20, 20, 20)           # 建造 HUD tooltip 背景
COLOR_HUD_TEXT = (210, 220, 230)      # 建造 HUD 文本
COLOR_BALLAST = (72, 66, 58)          # 道床带填充（暖灰，衬于逻辑细线之下）
COLOR_TILE_BOUNDARY = (60, 60, 60)    # 瓦片边界（淡灰）
COLOR_TILE_QUERY = (120, 120, 60)     # 查询邻域高亮（黄灰）

MODE_NAMES: dict[EditMode, str] = {
    EditMode.IDLE: "IDLE",
    EditMode.BUILD: "BUILD (B)",
    EditMode.DELETE: "DELETE (D)",
}


class Renderer:
    def __init__(self, surface: pygame.Surface, camera: Camera) -> None:
        self.surface = surface
        self.camera = camera
        self._font = pygame.font.Font(None, 20)

    def clear(self) -> None:
        self.surface.fill(COLOR_BG)

    def draw_grid(self) -> None:
        """绘制自适应档位网格背景（在 clear 之后、draw_network 之前）。"""
        draw_grid(self.surface, self.camera)

    def _draw_ballast(self, network: RailNetwork, cam: Camera, w: int, h: int) -> None:
        """绘制 Ballast 道床带（4 m 宽填充多边形）。道岔处允许重叠。"""
        for _edge, poly in iter_ballast_polygons(network):
            screen_pts = [cam.world_to_screen(p.x, p.y, w, h) for p in poly]
            if len(screen_pts) >= 3:
                pygame.draw.polygon(self.surface, COLOR_BALLAST, screen_pts)

    def _draw_parallel_reference_points(self, editor, cam: Camera, w: int, h: int) -> None:
        """绘制平行吸附参考点(Simple Case,全部橙色圆圈)。

        Complex Case 已改为 lazy 按需计算,不再预计算/可视化。
        """
        provider = editor.snap_system.parallel_snap
        ref_points = provider._reference_points

        COLOR_REF = (255, 150, 0)  # 橙色
        COLOR_LINE = (128, 128, 128)  # 灰色
        RADIUS_PX = 3

        # 绘制参考点
        for ref_pos, _ref_tangent, parent_id in ref_points:
            sx, sy = cam.world_to_screen(ref_pos.x, ref_pos.y, w, h)

            # 简单视口剔除
            if sx < -50 or sx > w + 50 or sy < -50 or sy > h + 50:
                continue

            # 绘制圆圈
            pygame.draw.circle(self.surface, COLOR_REF, (int(sx), int(sy)), RADIUS_PX)

            # 连线到父节点(虚线效果)
            parent_node = editor.network.nodes.get(parent_id)
            if parent_node:
                px, py = cam.world_to_screen(parent_node.position.x, parent_node.position.y, w, h)
                mid_x, mid_y = (sx + px) / 2, (sy + py) / 2
                pygame.draw.line(self.surface, COLOR_LINE, (int(sx), int(sy)), (int(mid_x), int(mid_y)), 1)

    def draw_network(self, network: RailNetwork, editor: Editor | None = None) -> None:
        w = self.surface.get_width()
        h = self.surface.get_height()
        cam = self.camera

        # Ballast 道床带：铺在逻辑细线之下，提供尺寸感（§12.1）
        self._draw_ballast(network, cam, w, h)

        # 平行参考点可视化(调试,需要 editor)
        if editor and editor.parallel_snap_enabled:
            self._draw_parallel_reference_points(editor, cam, w, h)

        for edge in network.edges.values():
            node_a = network.nodes[edge.node_a_id]
            node_b = network.nodes[edge.node_b_id]

            if edge.is_arc:
                if edge.geometry:
                    mid = edge.geometry[0]
                    self._draw_tangent_lines(node_a.position, mid, node_b.position, cam, w, h)
                    self._draw_tangent_point(mid, cam, w, h)

                if edge.arc_center is not None:
                    self._draw_center_dot(edge.arc_center, cam, w, h)

                pts = edge.sample_arc_points(30)
                if pts:
                    screen_pts = [cam.world_to_screen(p.x, p.y, w, h) for p in pts]
                    pygame.draw.aalines(self.surface, COLOR_EDGE_ARC, False, screen_pts)
            else:
                x1, y1 = cam.world_to_screen(node_a.position.x, node_a.position.y, w, h)
                x2, y2 = cam.world_to_screen(node_b.position.x, node_b.position.y, w, h)
                pygame.draw.aaline(self.surface, COLOR_EDGE_STRAIGHT, (x1, y1), (x2, y2))

        for node in network.nodes.values():
            cx, cy = cam.world_to_screen(node.position.x, node.position.y, w, h)
            count = node.connection_count()
            node_radius = max(4, 8 - count)
            color = COLOR_NODE.get(count, COLOR_NODE[1])
            pygame.draw.circle(self.surface, color, (int(cx), int(cy)), node_radius)

    def draw_overlay(self, network: RailNetwork, editor: Editor, mouse_world: Vec3) -> None:
        w = self.surface.get_width()
        h = self.surface.get_height()
        cam = self.camera

        # 悬停边高亮（DELETE 模式）
        if editor.hovered_edge_id is not None:
            edge = network.edges.get(editor.hovered_edge_id)
            if edge:
                node_a = network.nodes[edge.node_a_id]
                node_b = network.nodes[edge.node_b_id]
                if edge.is_arc:
                    pts = edge.sample_arc_points(30)
                    if len(pts) >= 2:
                        screen_pts = [cam.world_to_screen(p.x, p.y, w, h) for p in pts]
                        pygame.draw.lines(self.surface, COLOR_HOVER_EDGE, False, screen_pts, 3)
                else:
                    x1, y1 = cam.world_to_screen(node_a.position.x, node_a.position.y, w, h)
                    x2, y2 = cam.world_to_screen(node_b.position.x, node_b.position.y, w, h)
                    pygame.draw.line(self.surface, COLOR_HOVER_EDGE, (x1, y1), (x2, y2), 3)

        # 悬停节点高亮（吸附目标）
        if editor.hovered_node_id is not None:
            node = network.nodes.get(editor.hovered_node_id)
            if node:
                cx, cy = cam.world_to_screen(node.position.x, node.position.y, w, h)
                pygame.draw.circle(self.surface, COLOR_HOVER, (int(cx), int(cy)), 10, 2)

        # BUILD_ACTIVE: 预览几何 + M1 锚点
        if editor.mode == EditMode.BUILD and editor.build_state == BuildState.ACTIVE:
            self._draw_preview(editor, cam, w, h)
            self._draw_build_hud(editor, mouse_world, cam, w, h)

        # 警告指示器
        if editor.show_warning:
            self._draw_warning(mouse_world, cam, w, h)

        # 模式标签
        self._draw_mode_text(editor, w)

    def _draw_preview(self, editor: Editor, cam: Camera, w: int, h: int) -> None:
        """绘制 BUILD_ACTIVE 状态下的预览几何"""
        preview = editor.preview
        if preview is None:
            return

        color = COLOR_PREVIEW if preview.valid else COLOR_PREVIEW_INVALID

        if preview.case == 1 or (preview.case == 2 and not preview.edge_geometry):
            # 直线预览（Case 1 或 Case 2 退化）
            x1, y1 = cam.world_to_screen(preview.m1.x, preview.m1.y, w, h)
            x2, y2 = cam.world_to_screen(preview.m2.x, preview.m2.y, w, h)
            _draw_dashed_line(self.surface, color, (x1, y1), (x2, y2), dash_len=10, gap_len=5)
        elif preview.case == 2 and len(preview.edge_geometry) == 1:
            # 弧预览：通过 [A, B, C] 三点采样
            self._draw_arc_preview(
                preview.m1, preview.edge_geometry[0], preview.m2, color, cam, w, h
            )
        elif preview.case == 3 and preview.valid:
            # Biarc 预览：两段弧 + 中间锚点
            self._draw_biarc_preview(preview, color, cam, w, h)
        elif preview.case == 3 and not preview.valid:
            # Biarc 不可建造：用红色直线连接 M1→M2 作为占位（提示用户）
            x1, y1 = cam.world_to_screen(preview.m1.x, preview.m1.y, w, h)
            x2, y2 = cam.world_to_screen(preview.m2.x, preview.m2.y, w, h)
            _draw_dashed_line(self.surface, color, (x1, y1), (x2, y2), dash_len=10, gap_len=5)
        elif preview.case == 4 and preview.valid:
            # Case 4 弧+直线复合（§10.3）：弧段 + 直线段 + 中间锚点
            self._draw_composite_preview(preview, color, cam, w, h)
        elif preview.case == 5 and preview.valid and len(preview.edge_geometry) == 1:
            # Case 5 单切线弧（§10.5）：弧 M1→B→entry + 接入点独立标记
            self._draw_arc_preview(
                preview.m1, preview.edge_geometry[0], preview.m2, color, cam, w, h
            )
            if preview.case2t_entry is not None:
                ex, ey = cam.world_to_screen(
                    preview.case2t_entry.x, preview.case2t_entry.y, w, h
                )
                # 接入点用品红色方框标记（区别于普通节点/锚点）
                pygame.draw.circle(self.surface, COLOR_CASE2T_ENTRY, (int(ex), int(ey)), 6, 2)
        elif preview.case == 5 and not preview.valid:
            # Case 2T 不可解：红色直线占位
            x1, y1 = cam.world_to_screen(preview.m1.x, preview.m1.y, w, h)
            x2, y2 = cam.world_to_screen(preview.m2.x, preview.m2.y, w, h)
            _draw_dashed_line(self.surface, color, (x1, y1), (x2, y2), dash_len=10, gap_len=5)

        # 绘制 M1 锚点
        if editor.build_m1 is not None:
            ax, ay = cam.world_to_screen(editor.build_m1.x, editor.build_m1.y, w, h)
            pygame.draw.circle(self.surface, COLOR_M1_ANCHOR, (int(ax), int(ay)), 6)
            pygame.draw.circle(self.surface, (255, 255, 255), (int(ax), int(ay)), 6, 1)

    def _draw_arc_preview(
        self,
        a: Vec3,
        b: Vec3,
        c: Vec3,
        color: tuple[int, int, int],
        cam: Camera,
        w: int,
        h: int,
    ) -> None:
        """绘制弧预览：临时构造一个 Edge 并用其 sample_arc_points 渲染。

        同时绘制切线辅助线（M1→B→M2 虚线）。
        """
        from model.rail_network import Edge, _compute_arc

        result = _compute_arc(a, b, c)
        if result is None:
            # 不可解：退化为直线
            x1, y1 = cam.world_to_screen(a.x, a.y, w, h)
            x2, y2 = cam.world_to_screen(c.x, c.y, w, h)
            _draw_dashed_line(self.surface, color, (x1, y1), (x2, y2), dash_len=10, gap_len=5)
            return

        center, radius, angle, normal = result
        edge = Edge(
            edge_id=-1,
            node_a_id=-1,
            node_b_id=-1,
            geometry=[b],
            length=radius * abs(angle),
            is_arc=True,
            arc_center=center,
            arc_radius=radius,
            arc_angle_rad=angle,
            arc_start_dir=(a - center).normalize(),
            arc_normal=normal,
        )
        pts = edge.sample_arc_points(30)
        if len(pts) < 2:
            return
        screen_pts = [cam.world_to_screen(p.x, p.y, w, h) for p in pts]
        _draw_dashed_polyline(self.surface, color, screen_pts, dash_len=10, gap_len=5)

        # 切线辅助线 M1→B→M2（半透明灰色）
        x1, y1 = cam.world_to_screen(a.x, a.y, w, h)
        xb, yb = cam.world_to_screen(b.x, b.y, w, h)
        x2, y2 = cam.world_to_screen(c.x, c.y, w, h)
        _draw_dashed_line(self.surface, COLOR_TANGENT, (x1, y1), (xb, yb), dash_len=4, gap_len=4)
        _draw_dashed_line(self.surface, COLOR_TANGENT, (xb, yb), (x2, y2), dash_len=4, gap_len=4)

    def _draw_biarc_preview(
        self,
        preview,
        color: tuple[int, int, int],
        cam: Camera,
        w: int,
        h: int,
    ) -> None:
        """绘制 Case 3 双弧预览：两段弧 + 中间锚点高亮。"""
        if (
            preview.biarc_mid is None
            or preview.biarc_geom_1 is None
            or preview.biarc_geom_2 is None
        ):
            return
        if not preview.biarc_geom_1 or not preview.biarc_geom_2:
            return

        m_mid = preview.biarc_mid
        b1 = preview.biarc_geom_1[0]
        b2 = preview.biarc_geom_2[0]

        # 弧 1: M1 → B1 → M_mid
        self._draw_arc_preview(preview.m1, b1, m_mid, color, cam, w, h)
        # 弧 2: M_mid → B2 → M2
        self._draw_arc_preview(m_mid, b2, preview.m2, color, cam, w, h)

        # 中间节点锚点（小圆点）
        mx, my = cam.world_to_screen(m_mid.x, m_mid.y, w, h)
        pygame.draw.circle(self.surface, COLOR_M1_ANCHOR, (int(mx), int(my)), 4)

    def _draw_composite_preview(
        self,
        preview,
        color: tuple[int, int, int],
        cam: Camera,
        w: int,
        h: int,
    ) -> None:
        """绘制 Case 4 弧+直线复合预览（§10.3）：弧段 + 直线段 + 中间锚点。"""
        if (
            preview.composite_mid is None
            or preview.composite_arc_geom is None
            or preview.composite_tail_geom is None
        ):
            return
        if not preview.composite_arc_geom:
            return

        p_mid = preview.composite_mid
        b = preview.composite_arc_geom[0]

        # 弧段：M1 → B → P_mid
        self._draw_arc_preview(preview.m1, b, p_mid, color, cam, w, h)

        # 直线段：P_mid → M2
        x1, y1 = cam.world_to_screen(p_mid.x, p_mid.y, w, h)
        x2, y2 = cam.world_to_screen(preview.m2.x, preview.m2.y, w, h)
        _draw_dashed_line(self.surface, color, (x1, y1), (x2, y2), dash_len=10, gap_len=5)

        # 中间节点锚点（小圆点，与 Biarc 同风格）
        pygame.draw.circle(self.surface, COLOR_M1_ANCHOR, (int(x1), int(y1)), 4)

    def _draw_build_hud(
        self, editor: Editor, mouse_world: Vec3, cam: Camera, w: int, h: int
    ) -> None:
        """绘制建造 HUD tooltip（跟随光标），显示分段长度/半径/圆心角。"""
        lines = format_lines(compute_metrics(editor.preview))
        if not lines:
            return

        surfs = [self._font.render(line, True, COLOR_HUD_TEXT) for line in lines]
        line_h = self._font.get_linesize()
        pad = 6
        box_w = max(s.get_width() for s in surfs) + pad * 2
        box_h = line_h * len(surfs) + pad * 2

        # 跟随光标，默认放在光标右下；越界则翻转到另一侧，避免超出窗口
        mx, my = cam.world_to_screen(mouse_world.x, mouse_world.y, w, h)
        ox, oy = 16, 16
        bx = mx + ox
        by = my + oy
        if bx + box_w > w:
            bx = mx - ox - box_w
        if by + box_h > h:
            by = my - oy - box_h
        bx = max(0, bx)
        by = max(0, by)

        bg = pygame.Surface((box_w, box_h))
        bg.set_alpha(210)
        bg.fill(COLOR_HUD_BG)
        self.surface.blit(bg, (bx, by))
        for i, s in enumerate(surfs):
            self.surface.blit(s, (bx + pad, by + pad + i * line_h))

    def _draw_warning(self, mouse_world: Vec3, cam: Camera, w: int, h: int) -> None:
        """绘制警告光标（红色圆环）"""
        cx, cy = cam.world_to_screen(mouse_world.x, mouse_world.y, w, h)
        pygame.draw.circle(self.surface, COLOR_WARNING, (int(cx), int(cy)), 14, 2)
        # 红色十字
        pygame.draw.line(self.surface, COLOR_WARNING, (cx - 8, cy), (cx + 8, cy), 2)
        pygame.draw.line(self.surface, COLOR_WARNING, (cx, cy - 8), (cx, cy + 8), 2)

    def _draw_mode_text(self, editor: Editor, screen_w: int) -> None:
        label = MODE_NAMES.get(editor.mode, "")
        if editor.force_straight and editor.mode == EditMode.BUILD:
            label = label + "  [STRAIGHT]"
        if editor.grid_snap_enabled:
            label = label + "  [GRID]"
        if editor.length_snap_enabled:
            label = label + "  [LENGTH]"
        if editor.angle_snap_enabled:
            label = label + "  [ANGLE]"
        if editor.parallel_snap_enabled:
            label = label + "  [PARALLEL]"
        surf = self._font.render(label, True, COLOR_TEXT)
        self.surface.blit(surf, (10, 10))

    def _draw_tangent_lines(
        self, a: Vec3, b: Vec3, c: Vec3, cam: Camera, w: int, h: int
    ) -> None:
        x1, y1 = cam.world_to_screen(a.x, a.y, w, h)
        x2, y2 = cam.world_to_screen(b.x, b.y, w, h)
        x3, y3 = cam.world_to_screen(c.x, c.y, w, h)
        _draw_dashed_line(self.surface, COLOR_TANGENT, (x1, y1), (x2, y2))
        _draw_dashed_line(self.surface, COLOR_TANGENT, (x2, y2), (x3, y3))

    def _draw_tangent_point(self, pos: Vec3, cam: Camera, w: int, h: int) -> None:
        sx, sy = cam.world_to_screen(pos.x, pos.y, w, h)
        pygame.draw.circle(self.surface, COLOR_TANGENT_POINT, (int(sx), int(sy)), 3, 1)

    def _draw_center_dot(self, pos: Vec3, cam: Camera, w: int, h: int) -> None:
        sx, sy = cam.world_to_screen(pos.x, pos.y, w, h)
        pygame.draw.circle(self.surface, COLOR_CENTER, (int(sx), int(sy)), 2)


def _draw_dashed_line(
    surface: pygame.Surface,
    color: tuple[int, int, int],
    start: tuple[float, float],
    end: tuple[float, float],
    dash_len: int = 8,
    gap_len: int = 6,
) -> None:
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
    length = math.sqrt(dx * dx + dy * dy)
    if length == 0:
        return
    ux = dx / length
    uy = dy / length
    pos = 0.0
    segment = dash_len
    while pos < length:
        seg_end = min(pos + segment, length)
        sx = x1 + ux * pos
        sy = y1 + uy * pos
        ex = x1 + ux * seg_end
        ey = y1 + uy * seg_end
        pygame.draw.line(surface, color, (sx, sy), (ex, ey))
        pos = seg_end + gap_len
        segment = dash_len


def _draw_dashed_polyline(
    surface: pygame.Surface,
    color: tuple[int, int, int],
    points: list[tuple[float, float]],
    dash_len: int = 8,
    gap_len: int = 6,
) -> None:
    """对折线段逐段画虚线（不跨段保持 dash 相位，简单实现就够用）。"""
    for i in range(len(points) - 1):
        _draw_dashed_line(surface, color, points[i], points[i + 1], dash_len, gap_len)


def draw_spatial_index_debug(
    surface: pygame.Surface,
    camera: Camera,
    network: RailNetwork,
    cursor_world_pos: Vec3 | None = None,
    query_radius: float = 30.0,
) -> None:
    """绘制空间索引的调试可视化（I 键切换）。

    - 淡灰色网格：所有非空瓦片的边界
    - 黄灰色高亮：光标附近查询邻域的瓦片
    - 用于验证瓦片划分、查询范围是否合理
    """
    w = surface.get_width()
    h = surface.get_height()
    index = network._spatial_index
    tile_size = index.tile_size

    # 收集所有非空瓦片
    all_tiles = set(index.node_tiles.keys()) | set(index.edge_tiles.keys())

    # 如果有光标位置，计算查询邻域
    query_tiles = set()
    if cursor_world_pos is not None:
        query_tiles = set(index._tiles_in_radius(cursor_world_pos, query_radius))

    # 绘制所有非空瓦片边界
    for tile_key in all_tiles:
        tx, ty = tile_key
        # 瓦片世界坐标范围
        wx0 = tx * tile_size
        wy0 = ty * tile_size
        wx1 = wx0 + tile_size
        wy1 = wy0 + tile_size

        # 转屏幕坐标
        sx0, sy0 = camera.world_to_screen(wx0, wy0, w, h)
        sx1, sy1 = camera.world_to_screen(wx1, wy1, w, h)

        # 视口剔除：屏幕外不画
        if sx1 < 0 or sx0 > w or sy1 < 0 or sy0 > h:
            continue

        # 选颜色：查询邻域高亮
        color = COLOR_TILE_QUERY if tile_key in query_tiles else COLOR_TILE_BOUNDARY

        # 画矩形边界
        rect = pygame.Rect(int(sx0), int(sy0), int(sx1 - sx0), int(sy1 - sy0))
        pygame.draw.rect(surface, color, rect, 1)

