from __future__ import annotations

import pygame

from controller.ui_state import ActionHint, FeedbackLevel, FeedbackMessage


_TOAST_COLORS = {
    FeedbackLevel.INFO: (90, 145, 210),
    FeedbackLevel.SUCCESS: (70, 175, 105),
    FeedbackLevel.WARNING: (205, 150, 55),
    FeedbackLevel.ERROR: (205, 75, 75),
}


def draw_context_bar(
    surface: pygame.Surface,
    font: pygame.font.Font,
    hints: tuple[ActionHint, ...],
) -> None:
    """Draw the current context's actions from the centralized action registry."""
    height = font.get_linesize() + 10
    y = surface.get_height() - height
    layer = pygame.Surface((surface.get_width(), height), pygame.SRCALPHA)
    layer.fill((12, 15, 22, 215))
    x = 12
    for hint in hints:
        key = font.render(hint.binding, True, (255, 220, 105))
        label = font.render(f" {hint.label}   ", True, (205, 215, 230))
        if x + key.get_width() + label.get_width() > surface.get_width() - 12:
            break
        layer.blit(key, (x, 5))
        x += key.get_width()
        layer.blit(label, (x, 5))
        x += label.get_width()
    surface.blit(layer, (0, y))


def draw_feedback(
    surface: pygame.Surface,
    font: pygame.font.Font,
    message: FeedbackMessage | None,
) -> None:
    """Draw one short player-facing result without mirroring the terminal log."""
    if message is None:
        return
    max_text_width = min(680, surface.get_width() - 60)
    lines = _wrap_text(font, message.text, max_text_width)
    line_h = font.get_linesize()
    pad_x, pad_y = 12, 8
    box_w = max(font.size(line)[0] for line in lines) + pad_x * 2
    box_h = line_h * len(lines) + pad_y * 2
    x = (surface.get_width() - box_w) // 2
    y = 44
    layer = pygame.Surface((box_w, box_h), pygame.SRCALPHA)
    layer.fill((18, 21, 29, 230))
    pygame.draw.rect(layer, _TOAST_COLORS[message.level], layer.get_rect(), 2)
    for index, line in enumerate(lines):
        layer.blit(
            font.render(line, True, (235, 240, 248)),
            (pad_x, pad_y + index * line_h),
        )
    surface.blit(layer, (x, y))


def _wrap_text(font: pygame.font.Font, text: str, max_width: int) -> list[str]:
    lines: list[str] = []
    current = ""
    for char in text:
        candidate = current + char
        if current and font.size(candidate)[0] > max_width:
            lines.append(current)
            current = char
        else:
            current = candidate
    if current or not lines:
        lines.append(current)
    return lines
