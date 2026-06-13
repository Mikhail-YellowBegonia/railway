from __future__ import annotations

import math

import pygame

from model.rail_network import RailNetwork
from model.vec3 import Vec3
from view.camera import Camera
from controller.editor import EditMode, Editor

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
COLOR_SELECTED = (100, 255, 100)
COLOR_HOVER_EDGE = (255, 200, 80)
COLOR_TEXT = (200, 200, 200)

MODE_NAMES: dict[EditMode, str] = {
    EditMode.PLACE: "PLACE (P)",
    EditMode.CONNECT: "CONNECT (C)",
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

    def draw_overlay(self, network: RailNetwork, editor: Editor) -> None:
        w = self.surface.get_width()
        h = self.surface.get_height()
        cam = self.camera

        if editor.hovered_edge_id is not None:
            edge = network.edges.get(editor.hovered_edge_id)
            if edge and not edge.is_arc:
                node_a = network.nodes[edge.node_a_id]
                node_b = network.nodes[edge.node_b_id]
                x1, y1 = cam.world_to_screen(node_a.position.x, node_a.position.y, w, h)
                x2, y2 = cam.world_to_screen(node_b.position.x, node_b.position.y, w, h)
                pygame.draw.line(self.surface, COLOR_HOVER_EDGE, (x1, y1), (x2, y2), 3)

        if editor.selected_node_id is not None:
            node = network.nodes.get(editor.selected_node_id)
            if node:
                cx, cy = cam.world_to_screen(node.position.x, node.position.y, w, h)
                pygame.draw.circle(self.surface, COLOR_SELECTED, (int(cx), int(cy)), 10, 2)

        if editor.hovered_node_id is not None:
            node = network.nodes.get(editor.hovered_node_id)
            if node and editor.hovered_node_id != editor.selected_node_id:
                cx, cy = cam.world_to_screen(node.position.x, node.position.y, w, h)
                pygame.draw.circle(self.surface, COLOR_HOVER, (int(cx), int(cy)), 10, 2)

        self._draw_mode_text(editor, w)

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
