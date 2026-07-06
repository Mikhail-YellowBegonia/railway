from __future__ import annotations

import math

import pygame

from view.camera import Camera

# 网格颜色：次级（细/暗）与主级（粗/亮）。主级 = 次级间距 × MAJOR_EVERY。
COLOR_GRID_MINOR = (45, 45, 45)
COLOR_GRID_MAJOR = (60, 60, 60)
COLOR_AXIS = (70, 90, 70)  # 世界原点坐标轴（x=0 / y=0），略偏绿以区分

# 次级网格的目标屏幕像素间距：实际间距会跳到最接近它的档位。
TARGET_MINOR_PX = 32.0
# 主级每隔多少条次级线出现一次（1/5/10/50/100 序列 → 每 5 条一主级）。
MAJOR_EVERY = 5

# 档位尾数序列（1/2/5 × 10ⁿ 米）。含用户列举的 1/5/10/50/100。
_MANTISSA = (1.0, 2.0, 5.0)


def _pick_minor_spacing(scale: float) -> float:
    """按缩放选一个使屏幕间距最接近 TARGET_MINOR_PX 的次级世界间距（米）。

    scale 为像素/米。理想世界间距 = TARGET_MINOR_PX / scale，取 1/2/5×10ⁿ
    中屏幕像素最接近目标的一档，保持不同缩放下网格密度稳定。
    """
    if scale <= 0.0:
        return 1.0

    ideal = TARGET_MINOR_PX / scale
    exp = math.floor(math.log10(ideal))

    best_spacing = 1.0
    best_err = float("inf")
    # 遍历相邻两个数量级的尾数，取屏幕像素误差最小者
    for e in (exp, exp + 1):
        base = 10.0**e
        for m in _MANTISSA:
            spacing = m * base
            px = spacing * scale
            err = abs(px - TARGET_MINOR_PX)
            if err < best_err:
                best_err = err
                best_spacing = spacing
    return best_spacing


def draw_grid(surface: pygame.Surface, camera: Camera) -> None:
    """绘制自适应档位网格背景（数学坐标系，Y 向上）。

    在 clear() 之后、draw_network() 之前调用。仅遍历当前视口可见的网格线，
    避免全世界扫描。次级线暗、主级线亮，x=0 / y=0 轴线单独着色。
    """
    w = surface.get_width()
    h = surface.get_height()
    scale = camera.scale
    if scale <= 0.0:
        return

    minor = _pick_minor_spacing(scale)
    major = minor * MAJOR_EVERY

    # 视口世界坐标范围（四角反投影；数学坐标 Y 向上 → 屏幕顶为 max_y）
    left_x, top_y = camera.screen_to_world(0, 0, w, h)
    right_x, bottom_y = camera.screen_to_world(w, h, w, h)
    min_x, max_x = min(left_x, right_x), max(left_x, right_x)
    min_y, max_y = min(top_y, bottom_y), max(top_y, bottom_y)

    _draw_lines_x(surface, camera, w, h, min_x, max_x, minor, major)
    _draw_lines_y(surface, camera, w, h, min_y, max_y, minor, major)


def _is_multiple(value: float, spacing: float) -> bool:
    """value 是否落在 spacing 的整数倍上（含浮点容差）。用于判定主级/轴线。"""
    ratio = value / spacing
    return abs(ratio - round(ratio)) < 1e-6


def _draw_lines_x(
    surface: pygame.Surface,
    camera: Camera,
    w: int,
    h: int,
    min_x: float,
    max_x: float,
    minor: float,
    major: float,
) -> None:
    """绘制竖直网格线（沿 x 方向逐条）。"""
    start = math.ceil(min_x / minor)
    end = math.floor(max_x / minor)
    for i in range(start, end + 1):
        wx = i * minor
        sx, _ = camera.world_to_screen(wx, 0.0, w, h)
        if _is_multiple(wx, major) and abs(wx) < 1e-6:
            color = COLOR_AXIS
        elif _is_multiple(wx, major):
            color = COLOR_GRID_MAJOR
        else:
            color = COLOR_GRID_MINOR
        pygame.draw.line(surface, color, (sx, 0), (sx, h))


def _draw_lines_y(
    surface: pygame.Surface,
    camera: Camera,
    w: int,
    h: int,
    min_y: float,
    max_y: float,
    minor: float,
    major: float,
) -> None:
    """绘制水平网格线（沿 y 方向逐条）。"""
    start = math.ceil(min_y / minor)
    end = math.floor(max_y / minor)
    for i in range(start, end + 1):
        wy = i * minor
        _, sy = camera.world_to_screen(0.0, wy, w, h)
        if _is_multiple(wy, major) and abs(wy) < 1e-6:
            color = COLOR_AXIS
        elif _is_multiple(wy, major):
            color = COLOR_GRID_MAJOR
        else:
            color = COLOR_GRID_MINOR
        pygame.draw.line(surface, color, (0, sy), (w, sy))
