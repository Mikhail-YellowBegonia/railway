"""Functional consist-construction state, independent from pygame_gui widgets."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from model.wagon import Consist, Wagon, create_simple_car


class WagonPreset(str, Enum):
    POWERED_CONTROL = "powered_control"
    COACH = "coach"


@dataclass
class ConsistBuilder:
    depot_id: str
    depot_name: str
    depot_length: float
    consist: Consist
    direction: int = 1
    selected_preset: WagonPreset = WagonPreset.POWERED_CONTROL
    selected_wagon_index: int = 0

    @classmethod
    def start(cls, depot_id: str, depot_name: str, depot_length: float) -> "ConsistBuilder":
        return cls(
            depot_id=depot_id,
            depot_name=depot_name,
            depot_length=depot_length,
            consist=Consist(wagons=[_create_preset(WagonPreset.POWERED_CONTROL)]),
        )

    @property
    def can_complete(self) -> bool:
        return bool(self.consist.wagons) and self.consist.total_length <= self.depot_length + 1e-9

    @property
    def selected_wagon(self) -> Wagon | None:
        if not self.consist.wagons:
            return None
        self.selected_wagon_index = min(
            max(self.selected_wagon_index, 0), len(self.consist.wagons) - 1,
        )
        return self.consist.wagons[self.selected_wagon_index]

    def select_preset(self, preset: WagonPreset) -> None:
        self.selected_preset = preset

    def select_wagon(self, index: int) -> None:
        if 0 <= index < len(self.consist.wagons):
            self.selected_wagon_index = index

    def add_wagon(self) -> Wagon:
        wagon = _create_preset(self.selected_preset)
        self.consist.wagons.append(wagon)
        self.selected_wagon_index = len(self.consist.wagons) - 1
        return wagon

    def delete_selected(self) -> bool:
        if len(self.consist.wagons) <= 1:
            return False
        self.consist.wagons.pop(self.selected_wagon_index)
        self.selected_wagon_index = min(
            self.selected_wagon_index, len(self.consist.wagons) - 1,
        )
        return True

    def move_selected(self, delta: int) -> bool:
        target = self.selected_wagon_index + delta
        if not (0 <= target < len(self.consist.wagons)):
            return False
        wagons = self.consist.wagons
        wagons[self.selected_wagon_index], wagons[target] = (
            wagons[target], wagons[self.selected_wagon_index],
        )
        self.selected_wagon_index = target
        return True

    def reverse(self) -> None:
        self.consist.wagons.reverse()
        for wagon in self.consist.wagons:
            wagon.reverse_relative_to_consist()
        self.direction *= -1
        self.selected_wagon_index = len(self.consist.wagons) - 1 - self.selected_wagon_index


def preset_metadata(preset: WagonPreset) -> tuple[str, ...]:
    if preset == WagonPreset.POWERED_CONTROL:
        return (
            "Powered control car",
            "Length: 20 m",
            "Mass: 50 t",
            "Rated power: 3000 kW",
            "Control capability: yes",
        )
    return (
        "Ordinary coach",
        "Length: 20 m",
        "Mass: 50 t",
        "Rated power: none",
        "Control capability: no",
    )


def _create_preset(preset: WagonPreset) -> Wagon:
    if preset == WagonPreset.POWERED_CONTROL:
        return create_simple_car(
            length=20.0, mass=50.0, P_rated=3000.0, have_control=True,
        )
    return create_simple_car(
        length=20.0, mass=50.0, P_rated=None, have_control=False,
    )
