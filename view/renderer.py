from __future__ import annotations

import math
import os

import pygame

from model.rail_network import RailNetwork
from model.vec3 import Vec3
from view.camera import Camera
from view.grid import draw_grid
from view.ballast import iter_ballast_polygons
from view.build_metrics import compute_metrics, format_lines
from controller.editor import EditMode, BuildState, Editor

# ===== Layout 渲染模式（约定，2026-09） =====
#
# 当前没有美术素材，本模块全部渲染都归为"Layout"（极简）模式：所有游戏
# 要素（轨道、节点、信号、列车、车钩……）依然完整可见、可交互，但表现
# 脱离真实比例尺——线是逻辑细线，标记是几何图形（圆点/三角形），大小
# 用固定的屏幕像素常量定义（不随 camera.scale 缩放），不追求"看起来像
# 真实铁路"。
#
# 这不是临时占位，是长期共存的一套渲染方式：引入真实美术素材后不会
# 修改/替换这套方法，而是新开一套渲染方法（比如 view/renderer_art.py），
# 两套并存、按需切换。因此这里的图形选择可以放心用"够用就行"的标准，
# 不需要预留"将来换皮"的抽象层。
#
# 换算基准：Ballast/信号偏移等少数元素仍以世界单位定义（真实宽度语义），
# 其余交互性图标（车钩、信号三角形）用屏幕像素基准（COUPLER_RADIUS_PX、
# SIGNAL_OFFSET_PX 等），缩放地图看全局时不会缩到看不清或反而大得离谱。

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
COLOR_SIGNAL_GREEN = (60, 220, 90)    # 信号灯：绿（放行）
COLOR_SIGNAL_RED = (230, 50, 50)      # 信号灯：红（禁止）
# 信号图标以屏幕像素为基准（与 COUPLER_RADIUS_PX 同一约定），不随 camera.scale
# 缩放——Layout 模式（见本文件顶部说明）的图形元素本来就跟世界尺度脱钩，
# 缩小地图看全局时图标不能跟着缩到看不见。
SIGNAL_OFFSET_PX = 22                 # 信号图标中心沿离开方向偏移节点的屏幕距离（像素）
SIGNAL_TRIANGLE_RADIUS_PX = 11         # 三角形外接圆屏幕半径（像素）


def _load_font(size: int) -> pygame.font.Font:
    """尝试按候选路径加载支持中文的系统字体，失败回退默认字体。"""
    for path in (
        "/System/Library/Fonts/STHeiti Light.ttc",          # macOS
        "/System/Library/Fonts/Supplemental/Songti.ttc",
        "/Library/Fonts/Arial Unicode MS.ttf",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",  # Linux
        "C:/Windows/Fonts/msyh.ttc",                        # Windows
    ):
        if os.path.exists(path):
            try:
                return pygame.font.Font(path, size)
            except Exception:
                continue
    return pygame.font.Font(None, size)


MODE_NAMES: dict[EditMode, str] = {
    EditMode.IDLE: "IDLE",
    EditMode.BUILD: "BUILD (B)",
    EditMode.DELETE: "DELETE (D)",
    EditMode.PLAY: "PLAY (P)",
    EditMode.SIGNAL: "SIGNAL (H)",
}


class Renderer:
    def __init__(self, surface: pygame.Surface, camera: Camera) -> None:
        self.surface = surface
        self.camera = camera
        self._font = _load_font(18)

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

    def draw_overlay(
        self, network: RailNetwork, editor: Editor, mouse_world: Vec3,
        signals=None,          # model.signal.SignalTable | None
        signal_colors=None,    # dict[DirectedEdge, model.block.SignalState] | None
    ) -> None:
        w = self.surface.get_width()
        h = self.surface.get_height()
        cam = self.camera

        if signals is not None:
            self._draw_signals(network, signals, signal_colors or {}, cam, w, h)

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

    def _draw_signals(
        self, network: RailNetwork, signals, signal_colors: dict, cam: Camera, w: int, h: int,
    ) -> None:
        """绘制所有信号槛位（Layout 模式，见本文件顶部说明；One-Way PBS
        语义，只有正面一个信号）。

        每个槛位画在其所在 Node 位置、沿"该信号许可通行的方向"（即
        leaving_dir——车驶离该节点、进入受保护 Edge 的方向）偏移
        SIGNAL_OFFSET_PX 像素处——同一节点的多个槛位天然错开，不会重叠。
        图标是等边三角形，顶角指向通行方向（直观区分朝向，不需要美术
        素材）。偏移/半径都是屏幕像素常量，缩放地图时图标大小不变
        （Layout 模式的图形元素本来就跟世界比例尺脱钩）。

        Step 3：颜色不再是 SignalTable 自带的状态，改由 signal_colors
        （BlockManager.compute_colors 的结果）按占用推导；查不到时
        （比如 BlockManager 还没 rebuild 过）默认按 GREEN 画，不因为
        缺一帧数据就让信号消失或崩溃。
        """
        from model.block import SignalState as BlockSignalState

        for directed in signals.all_signals():
            edge_id, direction = directed
            edge = network.edges.get(edge_id)
            if edge is None:
                continue
            node_id = edge.node_a_id if direction > 0 else edge.node_b_id
            node = network.nodes.get(node_id)
            if node is None:
                continue
            leaving_dir = network.leaving_direction_at(node_id, edge_id)
            if leaving_dir is None:
                continue
            state = signal_colors.get(directed, BlockSignalState.GREEN)
            color = COLOR_SIGNAL_RED if state is BlockSignalState.RED else COLOR_SIGNAL_GREEN
            self._draw_signal_triangle(node.position, leaving_dir, color, cam, w, h)

    def _draw_signal_triangle(
        self, node_pos: Vec3, direction: Vec3, color: tuple[int, int, int],
        cam: Camera, w: int, h: int,
    ) -> None:
        """在 node_pos 附近画一个等边三角形（比例尺无关，屏幕像素定size），
        顶角指向 direction（世界空间方向向量）。

        world_to_screen 是仿射变换（等比缩放 + y 轴翻转，无旋转），所以
        把世界方向向量的 (x, -y) 分量归一化，就得到同一方向在屏幕空间的
        单位向量——之后全部计算留在屏幕像素坐标里，不再依赖 camera.scale，
        这样图标大小和位置偏移都不随缩放变化。
        """
        ncx, ncy = cam.world_to_screen(node_pos.x, node_pos.y, w, h)
        sdx, sdy = direction.x, -direction.y
        length = math.hypot(sdx, sdy)
        if length < 1e-9:
            return
        sdx, sdy = sdx / length, sdy / length

        center_x = ncx + sdx * SIGNAL_OFFSET_PX
        center_y = ncy + sdy * SIGNAL_OFFSET_PX

        angles = (0.0, 120.0, -120.0)
        pts = []
        for deg in angles:
            rad = math.radians(deg)
            rx = sdx * math.cos(rad) - sdy * math.sin(rad)
            ry = sdx * math.sin(rad) + sdy * math.cos(rad)
            pts.append((center_x + rx * SIGNAL_TRIANGLE_RADIUS_PX,
                        center_y + ry * SIGNAL_TRIANGLE_RADIUS_PX))
        pygame.draw.polygon(self.surface, color, pts)
        pygame.draw.polygon(self.surface, (20, 20, 20), pts, 1)

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


# 寻路测试可视化配色
COLOR_PATH = (255, 120, 40)          # 路径高亮（橙）
COLOR_PATH_START = (60, 220, 100)    # 起点（绿）
COLOR_PATH_GOAL = (255, 60, 60)      # 终点（红）
COLOR_PATH_LABEL = (255, 255, 255)   # 顺序编号


def _edge_screen_points(edge, network: RailNetwork, camera: Camera, w: int, h: int):
    """一条边的屏幕折线点（直线两端；圆弧采样）。"""
    node_a = network.nodes[edge.node_a_id]
    node_b = network.nodes[edge.node_b_id]
    if edge.is_arc:
        pts = edge.sample_arc_points(30)
        return [camera.world_to_screen(p.x, p.y, w, h) for p in pts]
    return [
        camera.world_to_screen(node_a.position.x, node_a.position.y, w, h),
        camera.world_to_screen(node_b.position.x, node_b.position.y, w, h),
    ]


def draw_pathfinding_debug(
    surface: pygame.Surface,
    camera: Camera,
    network: RailNetwork,
    font: pygame.font.Font,
    start_node_id: int | None,
    goal_node_id: int | None,
    path,
) -> None:
    """绘制寻路测试结果（F 键叠加态）。

    - 起点/终点节点：绿/红实心圈
    - 路径边：橙色加粗折线，每段中点标注行进顺序编号（1,2,3…）
    """
    w = surface.get_width()
    h = surface.get_height()

    # 已选节点标记（即使还没算出路径也显示）
    for node_id, color in ((start_node_id, COLOR_PATH_START), (goal_node_id, COLOR_PATH_GOAL)):
        if node_id is None:
            continue
        node = network.nodes.get(node_id)
        if node is None:
            continue
        cx, cy = camera.world_to_screen(node.position.x, node.position.y, w, h)
        pygame.draw.circle(surface, color, (int(cx), int(cy)), 9)
        pygame.draw.circle(surface, (255, 255, 255), (int(cx), int(cy)), 9, 2)

    if path is None:
        return

    for order, (edge_id, _direction) in enumerate(path.edges, start=1):
        edge = network.edges.get(edge_id)
        if edge is None:
            continue
        screen_pts = _edge_screen_points(edge, network, camera, w, h)
        if len(screen_pts) >= 2:
            pygame.draw.lines(surface, COLOR_PATH, False, screen_pts, 4)
        # 顺序编号标在该段折线中点
        mid = screen_pts[len(screen_pts) // 2]
        label = font.render(str(order), True, COLOR_PATH_LABEL)
        surface.blit(label, (int(mid[0]) + 4, int(mid[1]) - 8))


def draw_plan_editor_overlay(
    surface: pygame.Surface,
    camera: Camera,
    network: RailNetwork,
    font: pygame.font.Font,
    plan_editor,
) -> None:
    """显示 P5 草稿锚点、候选路径和失败原因。"""
    draft = plan_editor.draft
    resolution = draft.resolution
    color = (255, 70, 70) if draft.failure else (80, 230, 255)
    owner = plan_editor.owner
    if owner is not None and owner.plan is not None:
        routes = [(item.fixed_route, item) for item in owner.plan.items]
        routes.append((owner.plan.loop_route, None))
        for fixed_route, item in routes:
            if fixed_route is None:
                continue
            confirmed_color = (
                (255, 70, 70) if (
                    item.validate(network, require_fixed_route=True)
                    if item is not None else fixed_route.validate(
                        network, goal=owner.plan.items[0].goal if owner.plan.items else None,
                    )
                )
                else (80, 210, 120)
            )
            for edge_id, _direction in fixed_route.edges:
                edge = network.edges.get(edge_id)
                if edge is None:
                    continue
                points = _edge_screen_points(
                    edge, network, camera, surface.get_width(), surface.get_height(),
                )
                if len(points) >= 2:
                    pygame.draw.lines(surface, confirmed_color, False, points, 3)
    if resolution is not None and resolution.path is not None:
        for edge_id, _direction in resolution.path.edges:
            edge = network.edges.get(edge_id)
            if edge is None:
                continue
            points = _edge_screen_points(
                edge, network, camera, surface.get_width(), surface.get_height(),
            )
            if len(points) >= 2:
                pygame.draw.lines(surface, color, False, points, 5)
    for order, anchor in enumerate(draft.anchors, start=1):
        node = network.nodes.get(anchor.node_id)
        if node is None:
            continue
        x, y = camera.world_to_screen(
            node.position.x, node.position.y, surface.get_width(), surface.get_height(),
        )
        pygame.draw.circle(surface, (255, 220, 60), (int(x), int(y)), 10, 2)
        label = font.render(str(order), True, (255, 255, 255))
        surface.blit(label, (int(x) + 7, int(y) - 14))
    message = draft.failure or "左键节点=锚点 · 道岔后点出边=控制点 · 左键轨道=终点"
    text = font.render(message[:90], True, color)
    surface.blit(text, (18, surface.get_height() - 30))


def draw_plan_hud(
    surface: pygame.Surface,
    font: pygame.font.Font,
    train,
    editing: bool,
) -> None:
    """显示胜出控制车的当前条目、指针与错误状态。"""
    winner = train.state.consist.control_winner()
    if winner is None:
        line = "计划: 无控制车"
    elif winner.plan is None:
        line = f"计划: 控制车 {winner.wagon_id[:8]} 无计划"
    elif winner.plan.is_empty:
        line = "计划: 空计划"
    else:
        item = winner.plan.current()
        if item is None:
            line = "计划: 一次性事件链已完成"
        else:
            line = f"计划: {winner.plan.pointer + 1}/{len(winner.plan)} {item.label}"
    if editing:
        line += " 【编辑中】"
    validation = ""
    if winner is not None and winner.plan is not None and not winner.plan.is_empty:
        problems = winner.plan.validate_cycle(train.network)
        current = winner.plan.current()
        if not problems and current is not None:
            problems = current.validate(
                train.network, require_fixed_route=True,
            )
        if problems:
            validation = "计划错误: " + problems[0]
    status = validation or getattr(train, "plan_status", "")
    lines = [line] + ([status] if status else [])
    for index, value in enumerate(lines):
        text = font.render(value[:70], True, (255, 220, 100) if status else (220, 230, 240))
        surface.blit(text, (18, 104 + index * 20))


# Debug 列车可视化配色
COLOR_TRAIN = (255, 140, 0)          # 列车方块（橙）
COLOR_TRAIN_OCCUPIED = (220, 20, 60) # 实时占位（红）
COLOR_TRAIN_HEADING = (255, 220, 80)  # heading 箭头（亮黄）
COLOR_COUPLER = (140, 140, 140)       # 车钩常亮（灰）
COLOR_COUPLER_HOVER = (255, 235, 60)  # 车钩悬停高亮（黄）
COUPLER_RADIUS_PX = 5                 # 车钩圆圈屏幕半径（像素）
# 端头车钩（head/tail，docs/consist_ui.md §6，2026-09 定稿：单色方块，
# head/tail 靠 tooltip 文字区分）：方块比内部圆点大一号，作为连挂目标标记。
COLOR_END_COUPLER = (120, 200, 230)   # 端头车钩方块（青）
END_COUPLER_SIZE_PX = 9               # 端头方块半边长（像素）
COLOR_CONTROL_CAR = (80, 210, 235)    # 控制车标记（青）
COLOR_CONTROL_WINNER = (255, 220, 80) # 当前胜出控制车外环（黄）


def draw_debug_train(
    surface: pygame.Surface,
    camera: Camera,
    kinematics,  # PathKinematics 或 RigidWagonKinematics
    s: float,
    occupied: bool = False,
    color: tuple[int, int, int] | None = None,
) -> None:
    """绘制 debug 列车：质点（方块+箭头）或刚体车厢（多节转向架+连线）。

    color 若指定则覆盖 occupied 选色。
    """
    if kinematics is None:
        return

    if color is None:
        color = COLOR_TRAIN_OCCUPIED if occupied else COLOR_TRAIN

    w = surface.get_width()
    h = surface.get_height()

    # 判断运动学类型（duck typing）
    is_rigid = hasattr(kinematics, 'get_all_bogie_poses')

    if is_rigid:
        # D6: 多节编组可视化（循环绘制所有车厢）
        bogie_pairs = kinematics.get_all_bogie_poses(s)
        wagon_poses = kinematics.get_all_wagon_poses(s)
        consist = kinematics.consist
        control_winner = consist.control_winner()

        for i, ((front_pose, rear_pose), wagon_pose) in enumerate(zip(bogie_pairs, wagon_poses)):
            wagon_config = consist.wagons[i]

            # 车厢外轮廓（长方形）
            wagon_length = wagon_config.length - 1.0  # 减去车钩长度
            wagon_width = 3.0  # ponytail: 固定宽度，WagonConfig 未定义该字段

            pos = wagon_pose.position
            heading = wagon_pose.heading
            right = Vec3(-heading.y, heading.x, 0.0)  # 2D 右手系垂直方向

            # 4 个角点（世界坐标）
            half_len = wagon_length / 2.0
            half_wid = wagon_width / 2.0
            corners_world = [
                pos + heading * half_len + right * half_wid,   # 前左
                pos + heading * half_len - right * half_wid,   # 前右
                pos - heading * half_len - right * half_wid,   # 后右
                pos - heading * half_len + right * half_wid,   # 后左
            ]

            # 转屏幕坐标
            corners_screen = [
                camera.world_to_screen(c.x, c.y, w, h) for c in corners_world
            ]
            corners_screen = [(int(x), int(y)) for x, y in corners_screen]

            # 绘制轮廓（线框）
            pygame.draw.polygon(surface, color, corners_screen, 2)

            # 控制车：居中画驾驶台方框；当前遴选胜出者再加黄色外环。
            # 使用几何标记而非文字，缩放时仍能与轨道、信号清楚区分。
            if wagon_config.have_control:
                cx, cy = camera.world_to_screen(pos.x, pos.y, w, h)
                marker = pygame.Rect(int(cx) - 4, int(cy) - 4, 8, 8)
                pygame.draw.rect(surface, COLOR_CONTROL_CAR, marker)
                pygame.draw.rect(surface, (225, 255, 255), marker, 1)
                if wagon_config is control_winner:
                    pygame.draw.circle(
                        surface, COLOR_CONTROL_WINNER, (int(cx), int(cy)), 8, 2,
                    )

            # 转向架位置转屏幕坐标
            fx, fy = camera.world_to_screen(front_pose.position.x, front_pose.position.y, w, h)
            rx, ry = camera.world_to_screen(rear_pose.position.x, rear_pose.position.y, w, h)

            # 连线（车厢中心线）
            pygame.draw.line(surface, color, (int(fx), int(fy)), (int(rx), int(ry)), 1)

            # 前转向架（绿色圆圈）
            pygame.draw.circle(surface, (60, 220, 100), (int(fx), int(fy)), 5)
            pygame.draw.circle(surface, (255, 255, 255), (int(fx), int(fy)), 5, 1)

            # 后转向架（红色圆圈）
            pygame.draw.circle(surface, (255, 60, 60), (int(rx), int(ry)), 5)
            pygame.draw.circle(surface, (255, 255, 255), (int(rx), int(ry)), 5, 1)

        # 车钩连接点（常亮灰色，N-1 个）
        coupler_positions = kinematics.get_coupler_positions(s)
        for cp in coupler_positions:
            cx, cy = camera.world_to_screen(cp.x, cp.y, w, h)
            pygame.draw.circle(surface, COLOR_COUPLER, (int(cx), int(cy)), COUPLER_RADIUS_PX)
            pygame.draw.circle(surface, (200, 200, 200), (int(cx), int(cy)), COUPLER_RADIUS_PX, 1)

        # 端头车钩（head/tail）方块标记——连挂目标（docs/consist_ui.md §6）。
        # 单色方块（青色），head/tail 的区分靠悬停 tooltip 文字。
        _head_data, _tail_data = kinematics.get_end_coupler_data(s)
        for _label, (_cp, _eid, _t) in (("head", _head_data), ("tail", _tail_data)):
            cx, cy = camera.world_to_screen(_cp.x, _cp.y, w, h)
            rect = pygame.Rect(int(cx) - END_COUPLER_SIZE_PX,
                               int(cy) - END_COUPLER_SIZE_PX,
                               END_COUPLER_SIZE_PX * 2, END_COUPLER_SIZE_PX * 2)
            pygame.draw.rect(surface, COLOR_END_COUPLER, rect)
            pygame.draw.rect(surface, (230, 250, 255), rect, 1)

    else:
        # 质点模型可视化（原有方块+箭头）
        pose = kinematics.pose_at(s)
        pos = pose.position
        heading = pose.heading

        # 世界坐标转屏幕
        cx, cy = camera.world_to_screen(pos.x, pos.y, w, h)

        # 画方块（10×10 像素）
        rect = pygame.Rect(int(cx - 5), int(cy - 5), 10, 10)
        pygame.draw.rect(surface, color, rect)
        pygame.draw.rect(surface, (255, 255, 255), rect, 1)  # 白色边框

        # 画 heading 箭头（从方块中心指出 20 像素）
        arrow_len_world = 5.0  # 世界单位
        arrow_end_world = pos + heading * arrow_len_world
        ax, ay = camera.world_to_screen(arrow_end_world.x, arrow_end_world.y, w, h)
        pygame.draw.line(surface, COLOR_TRAIN_HEADING, (int(cx), int(cy)), (int(ax), int(ay)), 3)

        # 箭头尖端（简单三角形）
        import math
        angle = math.atan2(ay - cy, ax - cx)
        tip_size = 8
        tip1_x = ax - tip_size * math.cos(angle - 2.7)
        tip1_y = ay - tip_size * math.sin(angle - 2.7)
        tip2_x = ax - tip_size * math.cos(angle + 2.7)
        tip2_y = ay - tip_size * math.sin(angle + 2.7)
        pygame.draw.polygon(
            surface,
            COLOR_TRAIN_HEADING,
            [(int(ax), int(ay)), (int(tip1_x), int(tip1_y)), (int(tip2_x), int(tip2_y))],
        )


def draw_train_hud(
    surface: pygame.Surface,
    font: pygame.font.Font,
    s: float,
    v: float,
    a: float,
    total_length: float,
    v_target: float = 0.0,
    braking: bool = False,
) -> None:
    """绘制列车状态 HUD：速度、目标速度、弧长、加速度、制动状态。"""
    v_kmh = v * 3.6
    v_target_kmh = v_target * 3.6
    phase = "【制动】" if braking else "巡航"
    lines = [
        f"速度:    {v_kmh:6.1f} km/h  {phase}",
        f"目标:    {v_target_kmh:6.1f} km/h  (↑/↓ 调节)",
        f"弧长: {s:7.1f} / {total_length:.1f} m",
        f"加速度: {a:+5.2f} m/s²",
    ]

    # 文本渲染
    line_height = 20
    padding = 8
    bg_width = 220
    bg_height = len(lines) * line_height + padding * 2

    # 半透明背景
    bg_surface = pygame.Surface((bg_width, bg_height))
    bg_surface.set_alpha(180)  # 半透明
    bg_surface.fill((20, 20, 20))  # 深灰色
    surface.blit(bg_surface, (10, 10))

    # 文本
    for i, line in enumerate(lines):
        text = font.render(line, True, (255, 255, 255))
        surface.blit(text, (10 + padding, 10 + padding + i * line_height))


def draw_console_log(
    surface: pygame.Surface,
    font: pygame.font.Font,
    messages: list[str],
    max_lines: int = 18,
) -> None:
    """右下角半透明控制台回显（最近 max_lines 条）。"""
    if not messages:
        return
    w, h = surface.get_width(), surface.get_height()
    visible = messages[-max_lines:]
    line_h = font.get_linesize()
    pad = 5
    surfs = [font.render(m[:90], True, (180, 190, 175)) for m in visible]
    box_w = min(max(s.get_width() for s in surfs) + pad * 2, w - 16)
    box_h = line_h * len(surfs) + pad * 2
    bx = w - box_w - 8
    by = h - box_h - 8
    bg = pygame.Surface((box_w, box_h))
    bg.set_alpha(110)
    bg.fill((8, 8, 8))
    surface.blit(bg, (bx, by))
    for i, s in enumerate(surfs):
        surface.blit(s, (bx + pad, by + pad + i * line_h))


def draw_coupler_highlight(
    surface: pygame.Surface,
    camera: Camera,
    world_pos: Vec3,
    w: int,
    h: int,
) -> None:
    """在指定内部车钩位置画高亮圆圈（解挂悬停，docs/consist_ui.md §4）。"""
    cx, cy = camera.world_to_screen(world_pos.x, world_pos.y, w, h)
    pygame.draw.circle(surface, COLOR_COUPLER_HOVER, (int(cx), int(cy)), COUPLER_RADIUS_PX + 3)
    pygame.draw.circle(surface, (255, 255, 255), (int(cx), int(cy)), COUPLER_RADIUS_PX + 3, 1)


def draw_end_coupler_highlight(
    surface: pygame.Surface,
    camera: Camera,
    world_pos: Vec3,
    w: int,
    h: int,
) -> None:
    """在指定端头车钩位置画高亮方块（连挂悬停，docs/consist_ui.md §5）。"""
    cx, cy = camera.world_to_screen(world_pos.x, world_pos.y, w, h)
    size = END_COUPLER_SIZE_PX + 4
    rect = pygame.Rect(int(cx) - size, int(cy) - size, size * 2, size * 2)
    pygame.draw.rect(surface, COLOR_COUPLER_HOVER, rect)
    pygame.draw.rect(surface, (255, 255, 255), rect, 2)


def draw_text_tooltip(
    surface: pygame.Surface,
    font: pygame.font.Font,
    screen_pos: tuple[float, float],
    text: str,
) -> None:
    """通用浮动提示（跟随鼠标屏幕坐标）。车钩悬停 tooltip（§4/§5）用。"""
    surf = font.render(text, True, (230, 235, 225))
    pad = 4
    w, h = surface.get_width(), surface.get_height()
    mx, my = screen_pos
    bx = int(mx) + 12
    by = int(my) - surf.get_height() - 8
    bx = max(0, min(bx, w - surf.get_width() - pad * 2))
    by = max(0, by)
    bg = pygame.Surface((surf.get_width() + pad * 2, surf.get_height() + pad * 2))
    bg.set_alpha(170)
    bg.fill((18, 18, 24))
    surface.blit(bg, (bx, by))
    surface.blit(surf, (bx + pad, by + pad))


def draw_train_tooltip(
    surface: pygame.Surface,
    font: pygame.font.Font,
    screen_pos: tuple[float, float],
    train_idx: int,
    v: float,
) -> None:
    """鼠标悬停列车时的浮动提示（跟随鼠标屏幕坐标）。"""
    text = f"Train #{train_idx}  {v * 3.6:.1f} km/h"
    draw_text_tooltip(surface, font, screen_pos, text)


def draw_consist_panel(
    surface: pygame.Surface,
    font: pygame.font.Font,
    train_idx: int,
    train,  # TrainEntity
    selected_wagon_index: int = 0,
) -> None:
    """编组详情与最小控制车配置面板。"""
    consist = train.state.consist
    winner = consist.control_winner()
    lines = [f"=== Train #{train_idx} Consist ==="]
    for i, w in enumerate(consist.wagons):
        role = "Loco" if w.is_powered else "Coach"
        control = "CTRL" if w.have_control else "----"
        active = " ACTIVE" if w is winner else ""
        selected = ">" if i == selected_wagon_index else " "
        lines.append(
            f"{selected}[{i+1}] {role:<5} {w.length:.0f}m {w.mass:.0f}t "
            f"{control} P={w.priority:+d}{active}"
        )
    lines.append("1-9 select  [/] priority  C control")
    lines.append("Only while parked; plan editing locks config")

    line_h = font.get_linesize()
    pad = 8
    box_w = max(font.size(l)[0] for l in lines) + pad * 2
    box_h = line_h * len(lines) + pad * 2
    w_scr, h_scr = surface.get_width(), surface.get_height()
    bx = w_scr - box_w - 8
    by = h_scr // 2 - box_h // 2  # 屏幕垂直居中右侧

    bg = pygame.Surface((box_w, box_h))
    bg.set_alpha(200)
    bg.fill((15, 15, 25))
    surface.blit(bg, (bx, by))
    for i, line in enumerate(lines):
        color = (180, 220, 255) if i == 0 else (200, 200, 200)
        surface.blit(font.render(line, True, color), (bx + pad, by + pad + i * line_h))
