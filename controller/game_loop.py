from __future__ import annotations

import sys

import pygame

from model.rail_network import RailNetwork
from model.geojson_loader import load_geojson
from model.vec3 import Vec3
from view.camera import Camera
from view.renderer import Renderer
from controller.editor import Editor, EditMode

WINDOW_W = 1024
WINDOW_H = 768


class GameLoop:
    def __init__(self, geo_path: str) -> None:
        pygame.init()
        self.surface = pygame.display.set_mode((WINDOW_W, WINDOW_H), pygame.RESIZABLE)
        pygame.display.set_caption("Railway")
        self.clock = pygame.time.Clock()
        self.camera = Camera()
        self.renderer = Renderer(self.surface, self.camera)
        self.network: RailNetwork = load_geojson(geo_path)
        self.editor = Editor(self.network)
        self.running = True

    def run(self) -> None:
        while self.running:
            for event in pygame.event.get():
                self._handle_event(event)

            mouse_world = self._mouse_world_pos()
            self.editor.update_hover(mouse_world)

            self.renderer.clear()
            self.renderer.draw_network(self.network)
            self.renderer.draw_overlay(self.network, self.editor)
            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()
        sys.exit()

    def _handle_event(self, event: pygame.event.Event) -> None:
        if event.type == pygame.QUIT:
            self.running = False
            return

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                self.running = False
            elif event.key == pygame.K_p:
                self.editor.set_mode(EditMode.PLACE)
            elif event.key == pygame.K_c:
                self.editor.set_mode(EditMode.CONNECT)
            elif event.key == pygame.K_d:
                self.editor.set_mode(EditMode.DELETE)
            return

        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                world_pos = self._mouse_world_pos()
                self.editor.handle_click(world_pos)
            elif event.button in (4, 5):
                self.camera.handle_event(event)
            elif event.button == 2:
                self.camera._dragging = True
                self.camera._last_mouse = pygame.Vector2(event.pos)
            return

        if event.type == pygame.MOUSEBUTTONUP:
            if event.button == 2:
                self.camera._dragging = False
            return

        if event.type == pygame.MOUSEMOTION:
            if event.buttons[1]:  # middle button drag
                current = pygame.Vector2(event.pos)
                delta = current - self.camera._last_mouse
                self.camera.pan_x += delta.x / self.camera.scale
                self.camera.pan_y -= delta.y / self.camera.scale
                self.camera._last_mouse = current

    def _mouse_world_pos(self) -> Vec3:
        mx, my = pygame.mouse.get_pos()
        wx, wy = self.camera.screen_to_world(
            mx, my,
            self.surface.get_width(),
            self.surface.get_height(),
        )
        return Vec3(wx, wy, 0.0)


def run_game(geo_path: str) -> None:
    GameLoop(geo_path).run()
