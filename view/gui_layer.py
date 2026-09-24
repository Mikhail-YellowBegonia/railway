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
            pygame.Rect(0, 0, 820, 570),
            manager=self.manager,
            object_id="#consist_builder",
        )
        self._builder_title = UILabel(
            pygame.Rect(12, 8, 796, 30), "Consist Builder",
            manager=self.manager, container=self._builder_panel,
            object_id="#consist_builder_title",
        )
        self._catalog_panel = UIPanel(
            pygame.Rect(12, 44, 250, 260),
            manager=self.manager, container=self._builder_panel,
            object_id="#consist_catalog_panel",
        )
        UILabel(
            pygame.Rect(8, 6, 232, 28), "Vehicle Catalog",
            manager=self.manager, container=self._catalog_panel,
        )
        self._catalog_buttons = {
            WagonPreset.POWERED_CONTROL: UIButton(
                pygame.Rect(10, 44, 226, 54), "Powered Control Car",
                manager=self.manager, container=self._catalog_panel,
                object_id="#consist_catalog_powered",
            ),
            WagonPreset.COACH: UIButton(
                pygame.Rect(10, 108, 226, 54), "Ordinary Coach",
                manager=self.manager, container=self._catalog_panel,
                object_id="#consist_catalog_coach",
            ),
        }
        self._add_button = UIButton(
            pygame.Rect(10, 198, 226, 42), "Add Selected",
            manager=self.manager, container=self._catalog_panel,
            object_id="#consist_add",
        )

        self._metadata_panel = UIPanel(
            pygame.Rect(272, 44, 536, 260),
            manager=self.manager, container=self._builder_panel,
            object_id="#consist_metadata_panel",
        )
        UILabel(
            pygame.Rect(8, 6, 518, 28), "Vehicle Metadata",
            manager=self.manager, container=self._metadata_panel,
        )
        self._metadata_labels = [
            UILabel(
                pygame.Rect(16, 44 + index * 34, 500, 30), "",
                manager=self.manager, container=self._metadata_panel,
                object_id=f"#consist_metadata_{index}",
            )
            for index in range(5)
        ]

        self._preview_panel = UIPanel(
            pygame.Rect(12, 314, 796, 244),
            manager=self.manager, container=self._builder_panel,
            object_id="#consist_preview_panel",
        )
        UILabel(
            pygame.Rect(8, 6, 778, 28), "Consist Preview & Info",
            manager=self.manager, container=self._preview_panel,
        )
        self._builder_info = UILabel(
            pygame.Rect(12, 36, 770, 28), "",
            manager=self.manager, container=self._preview_panel,
            object_id="#consist_builder_info",
        )
        self._wagon_buttons: list[UIButton] = []
        self._delete_button = UIButton(
            pygame.Rect(12, 142, 92, 38), "Delete",
            manager=self.manager, container=self._preview_panel,
            object_id="#consist_delete",
        )
        self._move_left_button = UIButton(
            pygame.Rect(112, 142, 54, 38), "←",
            manager=self.manager, container=self._preview_panel,
            object_id="#consist_move_left",
        )
        self._move_right_button = UIButton(
            pygame.Rect(172, 142, 54, 38), "→",
            manager=self.manager, container=self._preview_panel,
            object_id="#consist_move_right",
        )
        self._reverse_button = UIButton(
            pygame.Rect(236, 142, 116, 38), "Reverse",
            manager=self.manager, container=self._preview_panel,
            object_id="#consist_reverse",
        )
        self._cancel_button = UIButton(
            pygame.Rect(584, 142, 92, 38), "Cancel",
            manager=self.manager, container=self._preview_panel,
            object_id="#consist_cancel",
        )
        self._complete_button = UIButton(
            pygame.Rect(684, 142, 92, 38), "Complete",
            manager=self.manager, container=self._preview_panel,
            object_id="#consist_complete",
        )
        self._builder_static_actions = {
            self._add_button: "consist.add",
            self._delete_button: "consist.delete",
            self._move_left_button: "consist.move.left",
            self._move_right_button: "consist.move.right",
            self._reverse_button: "consist.reverse",
            self._cancel_button: "consist.cancel",
            self._complete_button: "consist.complete",
        }
        self._builder_visible = False
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
        x = 12
        wagon_count = max(1, len(builder.consist.wagons))
        wagon_width = max(52, min(112, (764 - 6 * (wagon_count - 1)) // wagon_count))
        for index, wagon in enumerate(builder.consist.wagons):
            role = "Powered" if wagon.have_control and wagon.is_powered else "Coach"
            button = UIButton(
                pygame.Rect(x, 76, wagon_width, 50), role,
                manager=self.manager, container=self._preview_panel,
                object_id=f"#consist_wagon_{index}",
            )
            if index == builder.selected_wagon_index:
                button.select()
            self._wagon_buttons.append(button)
            x += wagon_width + 6

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

    def _layout(self) -> None:
        x = max(8, self._resolution[0] - self._panel_width - 8)
        self._panel.set_relative_position((x, 8))
        builder_x = max(0, (self._resolution[0] - 820) // 2)
        builder_y = max(0, (self._resolution[1] - 570) // 2)
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
