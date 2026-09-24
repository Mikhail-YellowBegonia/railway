"""Pure-ish train creation from a Depot POI."""
from __future__ import annotations

from model.depot import ordered_depot_path
from model.occupancy import OccupancyState
from model.poi import POI, POIKind
from model.rail_network import RailNetwork
from model.train_entity import TrainEntity, TrainState
from model.train_physics import DEFAULT_PHYSICS, TrainPhysics
from model.wagon import Consist


def spawn_train_at_depot(
    network: RailNetwork,
    depot: POI,
    consist: Consist,
    *,
    direction: int = 1,
    physics: TrainPhysics = DEFAULT_PHYSICS,
) -> TrainEntity:
    """Create a parked train fully laid out inside ``depot``.

    The train's front coupler is placed at the selected Depot endpoint and its
    tail coupler is placed ``consist.total_length`` metres behind it. The
    initial kinematics therefore sees distinct wagon positions instead of all
    wagons sharing a single node.
    """
    if depot.kind != POIKind.DEPOT:
        raise ValueError("列车只能从 Depot 生成")
    if not consist.wagons:
        raise ValueError("编组不能为空")
    path = ordered_depot_path(depot, network, direction)
    depot_length_m = path.total_cost
    consist_length = consist.total_length
    if consist_length > depot_length_m + 1e-9:
        raise ValueError(
            f"编组过长：{consist_length:.1f}m > Depot 长度 {depot_length_m:.1f}m"
        )

    # RigidWagonKinematics consumes the first wagon's front bogie position as
    # its path coordinate. The front coupler sits ahead of that bogie by this
    # offset, so place the coupler at the Depot endpoint without clipping it.
    first = consist.wagons[0]
    head_bogie_offset = (
        first.logical_front_bogie_pos - first.logical_front_coupler_pos
    )
    head_bogie_abs_s = depot_length_m - head_bogie_offset
    tail_coupler_abs_s = depot_length_m - consist_length

    # Keep occupied_offset inside the first retained edge. The occupied list
    # may still include the unoccupied suffix of the final edge; this is the
    # existing conservative edge-granularity semantics used by dispatch.
    prefix_length = 0.0
    start_index = 0
    for index, (edge_id, _edge_direction) in enumerate(path.edges):
        edge_length = network.edges[edge_id].length
        if tail_coupler_abs_s <= prefix_length + edge_length + 1e-9:
            start_index = index
            break
        prefix_length += edge_length
    occupied = path.edges[start_index:]
    occupied_offset = max(0.0, tail_coupler_abs_s - prefix_length)
    front_bogie_s = head_bogie_abs_s - tail_coupler_abs_s
    occupancy = OccupancyState(
        occupied=list(occupied),
        occupied_offset=occupied_offset,
        s=front_bogie_s,
        route=[],
    )
    state = TrainState(
        occupancy=occupancy,
        remaining_to_goal=0.0,
        v=0.0,
        consist=consist,
    )
    return TrainEntity(state, network, physics)
