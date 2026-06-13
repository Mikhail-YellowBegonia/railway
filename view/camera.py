from __future__ import annotations

import pygame


class Camera:
    def __init__(self) -> None:
        self.pan_x: float = 0.0
        self.pan_y: float = 0.0
        self.scale: float = 40.0  # pixels per world unit
        self._dragging = False
        self._last_mouse = pygame.Vector2(0, 0)

    def world_to_screen(self, wx: float, wy: float, screen_w: int, screen_h: int) -> tuple[float, float]:
        sx = screen_w / 2 + (wx + self.pan_x) * self.scale
        sy = screen_h / 2 - (wy + self.pan_y) * self.scale
        return sx, sy

    def screen_to_world(self, sx: float, sy: float, screen_w: int, screen_h: int) -> tuple[float, float]:
        wx = (sx - screen_w / 2) / self.scale - self.pan_x
        wy = -(sy - screen_h / 2) / self.scale - self.pan_y
        return wx, wy

    def handle_event(self, event: pygame.event.Event) -> None:
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                self._dragging = True
                self._last_mouse = pygame.Vector2(event.pos)
            elif event.button == 4:
                self.scale *= 1.2
            elif event.button == 5:
                self.scale /= 1.2

        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                self._dragging = False

        elif event.type == pygame.MOUSEMOTION:
            if self._dragging:
                current = pygame.Vector2(event.pos)
                delta = current - self._last_mouse
                self.pan_x += delta.x / self.scale
                self.pan_y -= delta.y / self.scale
                self._last_mouse = current
