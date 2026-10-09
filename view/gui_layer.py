from __future__ import annotations

import pygame
import pygame_gui
from pygame_gui.elements import UIButton, UILabel, UIPanel

from controller.editor import EditMode
from controller.consist_builder import ConsistBuilder, WagonPreset, preset_metadata


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
        self._builder_panel = UIPanel(
            pygame.Rect(0, 0, 1160, 820),
            manager=self.manager,
            object_id="#consist_builder",
        )
        self._builder_title = UILabel(
            pygame.Rect(12, 8, 796, 30), "Consist Builder",
            manager=self.manager, container=self._builder_panel,
            object_id="#consist_builder_title",
        )
        self._catalog_panel = UIPanel(
            pygame.Rect(12, 44, 300, 340),
            manager=self.manager, container=self._builder_panel,
            object_id="#consist_catalog_panel",
        )
        UILabel(
            pygame.Rect(8, 6, 232, 28), "Vehicle Catalog",
            manager=self.manager, container=self._catalog_panel,
        )
        self._catalog_buttons = {
            WagonPreset.POWERED_CONTROL: UIButton(
                pygame.Rect(10, 44, 280, 54), "Powered Control Car",
                manager=self.manager, container=self._catalog_panel,
                object_id="#consist_catalog_powered",
            ),
            WagonPreset.COACH: UIButton(
                pygame.Rect(10, 108, 280, 54), "Ordinary Coach",
                manager=self.manager, container=self._catalog_panel,
                object_id="#consist_catalog_coach",
            ),
        }
        self._add_button = UIButton(
            pygame.Rect(10, 198, 280, 42), "Add Selected",
            manager=self.manager, container=self._catalog_panel,
            object_id="#consist_add",
        )
        self._double_add_button = UIButton(
            pygame.Rect(10, 246, 280, 42), "Add Control + Coach",
            manager=self.manager, container=self._catalog_panel,
            object_id="#consist_add_double",
        )

        self._metadata_panel = UIPanel(
            pygame.Rect(324, 44, 824, 340),
            manager=self.manager, container=self._builder_panel,
            object_id="#consist_metadata_panel",
        )
        UILabel(
            pygame.Rect(8, 6, 518, 28), "Vehicle Metadata",
            manager=self.manager, container=self._metadata_panel,
        )
        self._metadata_labels = [
            UILabel(
                pygame.Rect(16, 44 + index * 42, 790, 34), "",
                manager=self.manager, container=self._metadata_panel,
                object_id=f"#consist_metadata_{index}",
            )
            for index in range(5)
        ]

        self._preview_panel = UIPanel(
            pygame.Rect(12, 400, 1136, 400),
            manager=self.manager, container=self._builder_panel,
            object_id="#consist_preview_panel",
        )
        UILabel(
            pygame.Rect(8, 6, 1120, 28), "Consist Preview & Info",
            manager=self.manager, container=self._preview_panel,
        )
        self._builder_info = UILabel(
            pygame.Rect(12, 36, 1110, 28), "",
            manager=self.manager, container=self._preview_panel,
            object_id="#consist_builder_info",
        )
        self._wagon_buttons: list[UIButton] = []
        self._delete_button = UIButton(
            pygame.Rect(12, 300, 120, 42), "Delete",
            manager=self.manager, container=self._preview_panel,
            object_id="#consist_delete",
        )
        self._move_left_button = UIButton(
            pygame.Rect(146, 300, 64, 42), "←",
            manager=self.manager, container=self._preview_panel,
            object_id="#consist_move_left",
        )
        self._move_right_button = UIButton(
            pygame.Rect(220, 300, 64, 42), "→",
            manager=self.manager, container=self._preview_panel,
            object_id="#consist_move_right",
        )
        self._reverse_button = UIButton(
            pygame.Rect(294, 300, 126, 42), "Reverse",
            manager=self.manager, container=self._preview_panel,
            object_id="#consist_reverse",
        )
        self._cancel_button = UIButton(
            pygame.Rect(900, 300, 110, 42), "Cancel",
            manager=self.manager, container=self._preview_panel,
            object_id="#consist_cancel",
        )
        self._complete_button = UIButton(
            pygame.Rect(1020, 300, 110, 42), "Complete",
            manager=self.manager, container=self._preview_panel,
            object_id="#consist_complete",
        )
        self._builder_static_actions = {
            self._add_button: "consist.add",
            self._double_add_button: "consist.add.double",
            self._delete_button: "consist.delete",
            self._move_left_button: "consist.move.left",
            self._move_right_button: "consist.move.right",
            self._reverse_button: "consist.reverse",
            self._cancel_button: "consist.cancel",
            self._complete_button: "consist.complete",
        }
        self._builder_visible = False
        self._builder_scroll = 0
        self._builder_panel.hide()

        # PLAY overlays share one registry and one visibility owner.  They are
        # intentionally screen-space panels; world input remains the controller's
        # responsibility and is blocked while an overlay is active.
        self._overlay_panels = {
            "consist_builder": self._builder_panel,
            "train_info": self._make_train_info_panel(),
            "schedule": self._make_schedule_panel(),
        }
        self._overlay_visible: str | None = None
        self._schedule_scroll = 0
        self._layout()

    def _make_train_info_panel(self) -> UIPanel:
        panel = UIPanel(
            pygame.Rect(0, 0, 540, 430), manager=self.manager,
            object_id="#train_info_panel",
        )
        UILabel(
            pygame.Rect(14, 12, 500, 32), "Train Information",
            manager=self.manager, container=panel,
        )
        self._train_info_labels = [
            UILabel(
                pygame.Rect(18, 58 + index * 32, 500, 28), "",
                manager=self.manager, container=panel,
            )
            for index in range(8)
        ]
        self._train_info_schedule = UIButton(
            pygame.Rect(18, 330, 220, 42), "Open Schedule (P)",
            manager=self.manager, container=panel,
        )
        self._train_info_close = UIButton(
            pygame.Rect(390, 330, 120, 42), "Close (I)",
            manager=self.manager, container=panel,
        )
        panel.hide()
        return panel

    def _make_schedule_panel(self) -> UIPanel:
        panel = UIPanel(
            pygame.Rect(0, 0, 640, 500), manager=self.manager,
            object_id="#schedule_panel",
        )
        UILabel(
            pygame.Rect(14, 12, 600, 32), "Dispatch Schedule",
            manager=self.manager, container=panel,
        )
        self._schedule_rows = [
            UIButton(
                pygame.Rect(18, 52 + index * 28, 600, 26), "",
                manager=self.manager, container=panel,
            )
            for index in range(10)
        ]
        self._schedule_priority_down = UIButton(
            pygame.Rect(18, 405, 52, 38), "-",
            manager=self.manager, container=panel,
        )
        self._schedule_priority_up = UIButton(
            pygame.Rect(76, 405, 52, 38), "+",
            manager=self.manager, container=panel,
        )
        self._schedule_move_up = UIButton(
            pygame.Rect(18, 350, 72, 38), "Up",
            manager=self.manager, container=panel,
        )
        self._schedule_move_down = UIButton(
            pygame.Rect(96, 350, 72, 38), "Down",
            manager=self.manager, container=panel,
        )
        self._schedule_delete_item = UIButton(
            pygame.Rect(174, 350, 110, 38), "Delete (X)",
            manager=self.manager, container=panel,
        )
        self._schedule_toggle_repeat = UIButton(
            pygame.Rect(290, 350, 138, 38), "Toggle Loop",
            manager=self.manager, container=panel,
        )
        self._schedule_delete_plan = UIButton(
            pygame.Rect(434, 350, 118, 38), "Delete Plan",
            manager=self.manager, container=panel,
        )
        self._schedule_edit = UIButton(
            pygame.Rect(18, 398, 160, 38), "Edit Selected",
            manager=self.manager, container=panel,
        )
        self._schedule_close = UIButton(
            pygame.Rect(490, 398, 120, 38), "Close",
            manager=self.manager, container=panel,
        )
        panel.hide()
        return panel

    def _set_overlay(self, name: str | None) -> None:
        for panel_name, panel in self._overlay_panels.items():
            if panel_name == name:
                panel.show()
            else:
                panel.hide()
        self._overlay_visible = name

    @property
    def overlay_visible(self) -> str | None:
        return self._overlay_visible

    def hide_overlays(self) -> None:
        self._set_overlay(None)

    def show_train_info(self, lines: tuple[str, ...]) -> None:
        self._set_overlay("train_info")
        for label, text in zip(self._train_info_labels, lines, strict=False):
            label.set_text(text)
        for label in self._train_info_labels[len(lines):]:
            label.set_text("")

    def show_schedule(self, lines: tuple[str, ...], *, can_edit: bool = True) -> None:
        self._set_overlay("schedule")
        self._overlay_panels["schedule"].hide()
        for row, text in zip(self._schedule_rows, lines, strict=False):
            row.set_text(text)
        for row in self._schedule_rows[len(lines):]:
            row.set_text("")
        if can_edit:
            self._schedule_priority_down.enable()
            self._schedule_priority_up.enable()
            for button in (self._schedule_move_up, self._schedule_move_down,
                           self._schedule_delete_item, self._schedule_toggle_repeat,
                           self._schedule_delete_plan, self._schedule_edit):
                button.enable()
        else:
            self._schedule_priority_down.disable()
            self._schedule_priority_up.disable()
            for button in (self._schedule_move_up, self._schedule_move_down,
                           self._schedule_delete_item, self._schedule_toggle_repeat,
                           self._schedule_delete_plan, self._schedule_edit):
                button.disable()

    def draw_schedule_overlay(
        self, surface: pygame.Surface, font: pygame.font.Font,
        lines: tuple[str, ...], selected_index: int = 0,
    ) -> None:
        """Draw the schedule as a compact text list, not padded GUI buttons."""
        if self._overlay_visible != "schedule":
            return
        w, h = surface.get_size()
        panel_w = min(620, max(470, w // 3))
        panel_h = min(h - 100, 650)
        x = w - panel_w - 24
        y = 52
        bg = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)
        bg.fill((15, 20, 29, 235))
        surface.blit(bg, (x, y))
        pygame.draw.rect(surface, (100, 135, 175), (x, y, panel_w, panel_h), 2)
        title = font.render(lines[0] if lines else "调度计划", True, (240, 245, 250))
        surface.blit(title, (x + 18, y + 14))
        top = y + 52
        bottom = y + panel_h - 116
        line_h = font.get_linesize() + 6
        visible = max(1, int((bottom - top) / line_h))
        body_start = 4 if len(lines) > 4 else 1
        body = lines[body_start:]
        max_scroll = max(0, len(body) - visible)
        self._schedule_scroll = min(self._schedule_scroll, max_scroll)
        for row, text in enumerate(body[self._schedule_scroll:self._schedule_scroll + visible]):
            index = row + self._schedule_scroll
            selected = index == selected_index
            color = (255, 220, 100) if selected else (205, 215, 228)
            if selected:
                pygame.draw.rect(surface, (48, 62, 82),
                                 (x + 10, top + row * line_h - 2, panel_w - 20, line_h))
            surface.blit(font.render(text, True, color), (x + 20, top + row * line_h))
        button_y = y + panel_h - 88
        labels = (("↑", "schedule.move.up"), ("↓", "schedule.move.down"),
                  ("删除", "schedule.delete.item"), ("循环/单次", "schedule.toggle.repeat"))
        button_w = (panel_w - 50) // 4
        for i, (label, _action) in enumerate(labels):
            rect = pygame.Rect(x + 10 + i * (button_w + 10), button_y, button_w, 34)
            pygame.draw.rect(surface, (45, 56, 72), rect)
            pygame.draw.rect(surface, (100, 130, 165), rect, 1)
            text = font.render(label, True, (220, 228, 238))
            surface.blit(text, text.get_rect(center=rect.center))
        hint = font.render("P 编辑参数/插入 · O/Esc 关闭", True, (155, 175, 198))
        surface.blit(hint, (x + 18, y + panel_h - 42))

    def schedule_pointer_action(self, event: pygame.event.Event, lines: tuple[str, ...]) -> str | None:
        if self._overlay_visible != "schedule":
            return None
        w, h = event.pos if hasattr(event, "pos") else (0, 0)
        sw, sh = self._resolution
        panel_w = min(620, max(470, sw // 3))
        panel_h = min(sh - 100, 650)
        x, y = sw - panel_w - 24, 52
        if event.type == pygame.MOUSEWHEEL:
            if not (x <= w <= x + panel_w and y <= h <= y + panel_h):
                return None
            self._schedule_scroll = max(0, self._schedule_scroll - event.y)
            return "schedule.refresh"
        if event.type != pygame.MOUSEBUTTONDOWN or not (x <= w <= x + panel_w and y <= h <= y + panel_h):
            return None
        if h < y + 52:
            return "schedule.close"
        line_h = pygame.font.Font(None, 20).get_linesize() + 6
        body_start = 4 if len(lines) > 4 else 1
        row = int((h - (y + 52)) // line_h) + self._schedule_scroll
        body_len = max(0, len(lines) - body_start)
        if 0 <= row < body_len and w < x + panel_w - 18:
            return f"schedule.select.{row + body_start}"
        button_y = y + panel_h - 88
        button_w = (panel_w - 50) // 4
        actions = ("schedule.move.up", "schedule.move.down",
                   "schedule.delete.item", "schedule.toggle.repeat")
        for i, action in enumerate(actions):
            rect = pygame.Rect(x + 10 + i * (button_w + 10), button_y, button_w, 34)
            if rect.collidepoint((w, h)):
                return action
        return None

    def process_event(self, event: pygame.event.Event) -> tuple[str | None, bool]:
        """Return a stable action id and whether pygame_gui consumed the event."""
        consumed = bool(self.manager.process_events(event))
        if event.type == pygame_gui.UI_BUTTON_PRESSED:
            for action_id, button in self._buttons.items():
                if event.ui_element is button:
                    return action_id, True
            for preset, button in self._catalog_buttons.items():
                if event.ui_element is button:
                    return f"consist.catalog.{preset.value}", True
            action_id = self._builder_static_actions.get(event.ui_element)
            if action_id is not None:
                return action_id, True
            overlay_actions = {
                self._train_info_schedule: "schedule.open",
                self._train_info_close: "overlay.close",
                self._schedule_priority_down: "schedule.priority.down",
                self._schedule_priority_up: "schedule.priority.up",
                self._schedule_move_up: "schedule.move.up",
                self._schedule_move_down: "schedule.move.down",
                self._schedule_delete_item: "schedule.delete.item",
                self._schedule_toggle_repeat: "schedule.toggle.repeat",
                self._schedule_delete_plan: "schedule.delete.plan",
                self._schedule_edit: "schedule.edit",
                self._schedule_close: "overlay.close",
            }
            action_id = overlay_actions.get(event.ui_element)
            if action_id is not None:
                return action_id, True
            for index, button in enumerate(self._wagon_buttons):
                if event.ui_element is button:
                    return f"consist.select.{index}", True
            for index, button in enumerate(self._schedule_rows):
                if event.ui_element is button:
                    return f"schedule.select.{index}", True
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

    def show_consist_builder(self, builder: ConsistBuilder) -> None:
        self._builder_visible = True
        self._set_overlay("consist_builder")
        self.refresh_consist_builder(builder)

    def hide_consist_builder(self) -> None:
        self._builder_visible = False
        if self._overlay_visible == "consist_builder":
            self._set_overlay(None)
        else:
            self._builder_panel.hide()

    def refresh_consist_builder(self, builder: ConsistBuilder) -> None:
        if not self._builder_visible:
            return
        self._builder_title.set_text(f"Consist Builder - Depot {builder.depot_name}")
        for preset, button in self._catalog_buttons.items():
            if preset == builder.selected_preset:
                button.select()
            else:
                button.unselect()
        for label, text in zip(
            self._metadata_labels, preset_metadata(builder.selected_preset), strict=True,
        ):
            label.set_text(text)

        for button in self._wagon_buttons:
            button.kill()
        self._wagon_buttons.clear()
        # Wagon selection is handled by the self-drawn map-like preview below;
        # pygame_gui buttons here caused clipping and made long consists shrink.

        direction = "Forward" if builder.direction > 0 else "Reverse"
        capacity = "Ready" if builder.can_complete else "Over Depot capacity"
        self._builder_info.set_text(
            f"{len(builder.consist.wagons)} cars - Length {builder.consist.total_length:.0f} m / "
            f"Depot {builder.depot_length:.0f} m - {direction} - {capacity}"
        )
        if len(builder.consist.wagons) <= 1:
            self._delete_button.disable()
        else:
            self._delete_button.enable()
        if builder.selected_wagon_index <= 0:
            self._move_left_button.disable()
        else:
            self._move_left_button.enable()
        if builder.selected_wagon_index >= len(builder.consist.wagons) - 1:
            self._move_right_button.disable()
        else:
            self._move_right_button.enable()
        if builder.can_complete:
            self._complete_button.enable()
        else:
            self._complete_button.disable()

    def consist_builder_pointer_action(self, event: pygame.event.Event, builder: ConsistBuilder) -> str | None:
        """Handle preview-only pointer gestures outside pygame_gui widgets."""
        if not self._builder_visible or self._overlay_visible != "consist_builder":
            return None
        if event.type not in (pygame.MOUSEBUTTONDOWN, pygame.MOUSEWHEEL):
            return None
        if event.type == pygame.MOUSEWHEEL:
            self._builder_scroll = max(0, self._builder_scroll - int(event.y) * 48)
            return "consist.preview.scroll"
        if event.button == 3:
            preview = self._preview_panel.get_abs_rect()
            area = pygame.Rect(preview.x + 24, preview.y + 70, preview.width - 48, 180)
            x = area.left + 18 - self._builder_scroll
            for index, wagon in enumerate(builder.consist.wagons):
                width = max(64, min(150, int(wagon.length * 3.2)))
                rect = pygame.Rect(x, area.centery - 26, width, 52)
                if rect.collidepoint(event.pos):
                    builder.select_wagon(index)
                    return "consist.flip"
                x += width + 12
            return "consist.flip"
        if event.button == 1:
            preview = self._preview_panel.get_abs_rect()
            area = pygame.Rect(preview.x + 24, preview.y + 70, preview.width - 48, 180)
            x = area.left + 18 - self._builder_scroll
            for index, wagon in enumerate(builder.consist.wagons):
                width = max(64, min(150, int(wagon.length * 3.2)))
                if pygame.Rect(x, area.centery - 26, width, 52).collidepoint(event.pos):
                    builder.select_wagon(index)
                    return f"consist.select.{index}"
                x += width + 12
        return None

    def draw_consist_preview(self, surface: pygame.Surface, builder: ConsistBuilder) -> None:
        """Draw a map-like, labelled consist preview over the legacy panel."""
        if not self._builder_visible or self._overlay_visible != "consist_builder":
            return
        preview = self._preview_panel.get_abs_rect()
        area = pygame.Rect(preview.x + 24, preview.y + 70, preview.width - 48, 180)
        pygame.draw.rect(surface, (20, 25, 34), area)
        pygame.draw.rect(surface, (100, 120, 140), area, 1)
        pygame.draw.line(surface, (110, 110, 110),
                         (area.left + 10, area.centery),
                         (area.right - 10, area.centery), 2)
        x = area.left + 18 - self._builder_scroll
        y = area.centery
        font = pygame.font.Font(None, 18)
        for index, wagon in enumerate(builder.consist.wagons):
            width = max(64, min(150, int(wagon.length * 3.2)))
            if x + width > area.right - 10:
                break
            rect = pygame.Rect(x, y - 26, width, 52)
            color = (75, 170, 205) if wagon.have_control else (150, 155, 165)
            if index == builder.selected_wagon_index:
                pygame.draw.rect(surface, (255, 210, 70), rect.inflate(6, 6), 2)
            pygame.draw.rect(surface, color, rect, 2)
            pygame.draw.line(surface, color, (x, y), (x + width, y), 1)
            # Physical orientation marker: triangle points in wagon orientation.
            if wagon.orientation > 0:
                points = [(x + width - 10, y), (x + width - 22, y - 8), (x + width - 22, y + 8)]
            else:
                points = [(x + 10, y), (x + 22, y - 8), (x + 22, y + 8)]
            pygame.draw.polygon(surface, (255, 235, 100), points)
            label = "Control" if wagon.have_control else "Coach"
            text = font.render(f"{index + 1} {label}", True, (235, 240, 245))
            surface.blit(text, (x + 6, rect.top + 6))
            x += width + 12

    def _layout(self) -> None:
        x = max(8, self._resolution[0] - self._panel_width - 8)
        self._panel.set_relative_position((x, 8))
        builder_x = max(0, (self._resolution[0] - 1160) // 2)
        builder_y = max(0, (self._resolution[1] - 820) // 2)
        self._builder_panel.set_relative_position((builder_x, builder_y))
        info_panel = self._overlay_panels["train_info"]
        info_panel.set_relative_position((
            max(0, (self._resolution[0] - 540) // 2),
            max(0, (self._resolution[1] - 430) // 2),
        ))
        schedule_panel = self._overlay_panels["schedule"]
        schedule_panel.set_relative_position((
            max(0, (self._resolution[0] - 640) // 2),
            max(0, (self._resolution[1] - 500) // 2),
        ))
