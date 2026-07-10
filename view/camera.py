from __future__ import annotations

import pygame


class Camera:
    def __init__(self) -> None:
        self.pan_x: float = 0.0
        self.pan_y: float = 0.0
        self.scale: float = 40.0  # 像素/世界单位；约定 1 世界单位 = 1 米（公制，见 docs/editor.md §12.1）
        self._dragging = False
        self._last_mouse = pygame.Vector2(0, 0)

        # 相机跟随状态
        self.follow_enabled = False  # 自动跟随开关
        self.follow_smoothing = 0.1  # 平滑因子（0=瞬移，1=不动），默认 0.1

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

    def set_center_smooth(self, world_x: float, world_y: float) -> None:
        """平滑跟随目标世界坐标（线性插值）。

        相机中心逐渐移向 (world_x, world_y)，插值因子由 follow_smoothing 控制。
        用于列车跟随等场景，避免瞬移抖动。

        注意：pan_x/pan_y 是相机偏移，世界原点在屏幕中心时 pan=(0,0)。
        要让 (world_x, world_y) 显示在屏幕中心，设 pan=(-world_x, -world_y)。
        """
        target_pan_x = -world_x
        target_pan_y = -world_y

        # 线性插值（lerp）
        self.pan_x += (target_pan_x - self.pan_x) * self.follow_smoothing
        self.pan_y += (target_pan_y - self.pan_y) * self.follow_smoothing
