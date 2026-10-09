"""Single-owner lifecycle for future screen- and world-space drags."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class DragOwner(str, Enum):
    SCREEN_WIDGET = "screen_widget"
    WORLD_TOOL = "world_tool"
    CAMERA = "camera"


@dataclass
class DragSession:
    owner: DragOwner | None = None
    action_id: str | None = None
    start_pos: tuple[float, float] | None = None
    current_pos: tuple[float, float] | None = None

    @property
    def active(self) -> bool:
        return self.owner is not None

    def begin(
        self, owner: DragOwner, action_id: str,
        position: tuple[float, float],
    ) -> bool:
        if self.active:
            return False
        self.owner = owner
        self.action_id = action_id
        self.start_pos = position
        self.current_pos = position
        return True
    def update(self, owner: DragOwner, position: tuple[float, float]) -> bool:
        if self.owner is not owner:
            return False
        self.current_pos = position
        return True

    def release(self, owner: DragOwner, position: tuple[float, float]) -> str | None:
        if self.owner is not owner:
            return None
        action_id = self.action_id
        self.current_pos = position
        self.cancel(owner)
        return action_id

    def cancel(self, owner: DragOwner | None = None) -> bool:
        if not self.active or (owner is not None and self.owner is not owner):
            return False
        self.owner = None
        self.action_id = None
        self.start_pos = None
        self.current_pos = None
        return True
