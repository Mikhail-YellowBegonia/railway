from __future__ import annotations

from dataclasses import dataclass

from model.pathfinding import DirectedEdge, Path
from model.rail_network import RailNetwork


@dataclass
class OccupancyState:
    """列车车身占用轨迹的滑动窗口状态。

    occupied: 车身当前覆盖的有向边序列（tail → head 顺序），长度始终
              >= consist_length（否则说明 route 已耗尽，clamp 到终点）。
    occupied_offset: 车尾（tail）在 occupied[0] 内的局部弧长偏移（米）。
    s: 车头（head）在 occupied 路径上的绝对弧长（从 occupied[0] 起点算，米）。
    route: 待走的有向边（寻路结果中，head 尚未走到的部分）。为空 = 无更多指令。
    """
    occupied: list[DirectedEdge]
    occupied_offset: float
    s: float
    route: list[DirectedEdge]


def advance_occupied_path(
    network: RailNetwork,
    state: OccupancyState,
    delta_s: float,
    consist_length: float,
) -> tuple[OccupancyState, bool]:
    """推进车头弧长 delta_s，消费 route，滑动裁剪 occupied 头部。

    不做任何道岔判断——走哪条边由 route 提前给定，这里只处理边界越过
    （occupied 追加）和窗口裁剪（occupied 头部弹出）。

    返回:
        (新状态, reached_end)
        reached_end=True 表示 route 已耗尽且到达终点（s 被 clamp）。
    """
    occupied = list(state.occupied)
    route = list(state.route)
    s = state.s + delta_s
    occupied_offset = state.occupied_offset
    reached_end = False

    def edge_len(directed: DirectedEdge) -> float:
        return network.edges[directed[0]].length

    total_len = sum(edge_len(d) for d in occupied)

    # head 越过 occupied 末端：从 route 消费新边
    while s > total_len + 1e-9:
        if not route:
            s = total_len
            reached_end = True
            break
        next_edge = route.pop(0)
        occupied.append(next_edge)
        total_len += edge_len(next_edge)

    # 滑动裁剪：从 occupied 头部移除已经甩出车尾的边
    tail_s = s - consist_length
    while len(occupied) > 1:
        first_end_s = edge_len(occupied[0]) - occupied_offset
        if tail_s > first_end_s + 1e-9:
            occupied.pop(0)
            s -= first_end_s
            tail_s -= first_end_s
            occupied_offset = 0.0
        else:
            break

    new_state = OccupancyState(
        occupied=occupied,
        occupied_offset=occupied_offset,
        s=s,
        route=route,
    )
    return new_state, reached_end


def occupied_as_path(state: OccupancyState, network: RailNetwork) -> Path:
    """把 occupied 边列表转换为 Path（供 PathKinematics/渲染使用）。"""
    total_cost = sum(network.edges[e[0]].length for e in state.occupied)
    return Path(edges=list(state.occupied), total_cost=total_cost)
