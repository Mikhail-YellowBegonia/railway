from __future__ import annotations

import math

import pygame

from model.rail_network import RailNetwork
from model.vec3 import Vec3
from view.camera import Camera
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

    def draw_network(self, network: RailNetwork) -> None:
        w = self.surface.get_width()
        h = self.surface.get_height()
        cam = self.camera

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

        # 绘制预览边（Step 1 仅 Case 1 直线）
        if preview.case == 1:
            x1, y1 = cam.world_to_screen(preview.m1.x, preview.m1.y, w, h)
            x2, y2 = cam.world_to_screen(preview.m2.x, preview.m2.y, w, h)
            _draw_dashed_line(self.surface, color, (x1, y1), (x2, y2), dash_len=10, gap_len=5)
        # TODO: Case 2 弧预览, Case 3 Biarc 预览

        # 绘制 M1 锚点
        if editor.build_m1 is not None:
            ax, ay = cam.world_to_screen(editor.build_m1.x, editor.build_m1.y, w, h)
            pygame.draw.circle(self.surface, COLOR_M1_ANCHOR, (int(ax), int(ay)), 6)
            pygame.draw.circle(self.surface, (255, 255, 255), (int(ax), int(ay)), 6, 1)

    def _draw_warning(self, mouse_world: Vec3, cam: Camera, w: int, h: int) -> None:
        """绘制警告光标（红色圆环）"""
        cx, cy = cam.world_to_screen(mouse_world.x, mouse_world.y, w, h)
        pygame.draw.circle(self.surface, COLOR_WARNING, (int(cx), int(cy)), 14, 2)
        # 红色十字
        pygame.draw.line(self.surface, COLOR_WARNING, (cx - 8, cy), (cx + 8, cy), 2)
        pygame.draw.line(self.surface, COLOR_WARNING, (cx, cy - 8), (cx, cy + 8), 2)

    def _draw_mode_text(self, editor: Editor, screen_w: int) -> None:
        label = MODE_NAMES.get(editor.mode, "")
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
