from __future__ import annotations

import pygame
import pygame_gui
from pygame_gui.elements import UIButton, UIPanel

from controller.editor import EditMode


class GameGUI:
    """pygame_gui-backed screen-space controls layered above the world canvas."""

    _BUTTONS = (
        ("mode.idle", "Select", "Return to selection / idle mode"),
        ("mode.build", "Build", "Build track (B)"),
        ("mode.delete", "Delete", "Delete track or trains (D)"),
        ("mode.signal", "Signal", "Place directional signals (H)"),
        ("mode.poi", "POI", "Create and inspect POIs (J)"),
        ("mode.play", "Play", "Select trains and issue orders (P)"),
    )
    _MODE_ACTIONS = {
        EditMode.IDLE: "mode.idle",
        EditMode.BUILD: "mode.build",
        EditMode.DELETE: "mode.delete",
        EditMode.SIGNAL: "mode.signal",
        EditMode.POI: "mode.poi",
        EditMode.PLAY: "mode.play",
    }

    def __init__(self, resolution: tuple[int, int]) -> None:
        self.manager = pygame_gui.UIManager(resolution)
        self._resolution = resolution
        self._panel_width = 452
        self._panel_height = 44
        self._panel = UIPanel(
            pygame.Rect(0, 0, self._panel_width, self._panel_height),
            manager=self.manager,
            object_id="#mode_toolbar",
        )
        self._buttons: dict[str, UIButton] = {}
        button_width = 72
        for index, (action_id, label, tooltip) in enumerate(self._BUTTONS):
            button = UIButton(
                pygame.Rect(6 + index * (button_width + 2), 5, button_width, 30),
                label,
                manager=self.manager,
                container=self._panel,
                tool_tip_text=tooltip,
                object_id=f"#{action_id.replace('.', '_')}",
            )
            self._buttons[action_id] = button
        self._layout()

    def process_event(self, event: pygame.event.Event) -> tuple[str | None, bool]:
        """Return a stable action id and whether pygame_gui consumed the event."""
        consumed = bool(self.manager.process_events(event))
        if event.type == pygame_gui.UI_BUTTON_PRESSED:
            for action_id, button in self._buttons.items():
                if event.ui_element is button:
                    return action_id, True
        return None, consumed

    def set_resolution(self, resolution: tuple[int, int]) -> None:
        if resolution == self._resolution:
            return
        self._resolution = resolution
        self.manager.set_window_resolution(resolution)
        self._layout()

    def set_active_mode(self, mode: EditMode) -> None:
        active_action = self._MODE_ACTIONS[mode]
        for action_id, button in self._buttons.items():
            if action_id == active_action:
                button.select()
            else:
                button.unselect()

    def update(self, time_delta: float) -> None:
        self.manager.update(time_delta)

    def draw(self, surface: pygame.Surface) -> None:
        self.manager.draw_ui(surface)

    def _layout(self) -> None:
        x = max(8, self._resolution[0] - self._panel_width - 8)
        self._panel.set_relative_position((x, 8))
