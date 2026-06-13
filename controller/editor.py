from __future__ import annotations

from enum import Enum, auto

from model.rail_network import RailNetwork
from model.vec3 import Vec3

SNAP_THRESHOLD = 0.3


class EditMode(Enum):
    PLACE = auto()
    CONNECT = auto()
    DELETE = auto()


class Editor:
    def __init__(self, network: RailNetwork) -> None:
        self.network = network
        self.mode: EditMode = EditMode.PLACE
        self.hovered_node_id: int | None = None
        self.hovered_edge_id: int | None = None
        self.selected_node_id: int | None = None

    def update_hover(self, world_pos: Vec3) -> None:
        self.hovered_node_id = self.network.node_id_at(world_pos, SNAP_THRESHOLD)

        if self.mode == EditMode.DELETE:
            if self.hovered_node_id is not None:
                self.hovered_edge_id = None
            else:
                self.hovered_edge_id = self.network.edge_id_at(world_pos, SNAP_THRESHOLD)
        else:
            self.hovered_edge_id = None

    def handle_click(self, world_pos: Vec3) -> None:
        if self.mode == EditMode.PLACE:
            self._click_place(world_pos)
        elif self.mode == EditMode.CONNECT:
            self._click_connect(world_pos)
        elif self.mode == EditMode.DELETE:
            self._click_delete(world_pos)

    def set_mode(self, mode: EditMode) -> None:
        self.mode = mode
        self.selected_node_id = None

    def _click_place(self, pos: Vec3) -> None:
        node = self.network.add_node(pos)
        self.hovered_node_id = node.node_id

    def _click_connect(self, pos: Vec3) -> None:
        target_id = self.network.node_id_at(pos, SNAP_THRESHOLD)

        if self.selected_node_id is None:
            if target_id is not None:
                self.selected_node_id = target_id
        else:
            src_id = self.selected_node_id
            self.selected_node_id = None

            if target_id is None:
                new_node = self.network.add_node(pos)
                target_id = new_node.node_id

            if src_id != target_id:
                a = self.network.nodes[src_id]
                b = self.network.nodes[target_id]
                self.network.add_edge(a, b)

    def _click_delete(self, pos: Vec3) -> None:
        if self.hovered_node_id is not None:
            self.network.remove_node(self.hovered_node_id)
            self.hovered_node_id = None
        elif self.hovered_edge_id is not None:
            self.network.remove_edge(self.hovered_edge_id)
            self.hovered_edge_id = None
