"""会话持久化（roadmap #2）：列车状态的序列化/反序列化。

对应 docs/session_persistence.md（2026-09 定稿）。纯模型模块，无 view/controller/
pygame 依赖。

核心难点与方案：
- edge_id 每次加载都会重新分配（RailNetwork._next_edge_id），因此
  OccupancyState.occupied/route 与 TrainState.goal 里的 DirectedEdge
  `(edge_id, direction)` **不能直存 id**——改为存"这条边从 a 端走到 b 端"
  的两端节点坐标 `{"a": [x,y,z], "b": [x,y,z]}`，加载时按坐标反查节点、再找
  两者之间的边，并据"存的是从 a→b 的方向"恢复真实 direction。这与信号持久化
  （geojson_writer/loader 的 "signals" 字段）是同一套坐标匹配语义。
- wagon_id 是 uuid，折返/连挂/解挂都保留原 id，天然稳定，直存。
- controller / authority_remaining 是运行时派生状态，不落盘：停放列车加载后
  controller=None；行驶中列车（route 非空）用 assign_route 重建 controller。
- split_sibling / couple_approach_partner 是运行时对象引用，不落盘：
  split_sibling 在 GameLoop 加载后按"共享至少一条 occupied 边"动态重建
  （与豁免前置条件语义等价），couple_approach_partner 由列车续行/重寻路重建
  （见 docs/session_persistence.md §6 决策 2/3）。

失败容错（决策 4）：单列车反查失败（某条有向边坐标找不到节点/边）静默跳过该
列车，其余正常载入。
"""
from __future__ import annotations

from typing import TYPE_CHECKING

from model.vec3 import Vec3

if TYPE_CHECKING:
    from model.rail_network import RailNetwork
    from model.train_entity import TrainEntity

# 坐标匹配容差，与 geojson_loader 的 load_geojson/load_signals 默认一致。
EPSILON = 0.01

# 物理模型类型标记（当前 GameLoop 只用 RealisticElectric，存标记兜底未来多模型）。
PHYSICS_REALISTIC_ELECTRIC = "realistic_electric"


def _vec3_list(v: Vec3) -> list[float]:
    return [v.x, v.y, v.z]


def _coord(v: list[float] | tuple[float, ...]) -> Vec3:
    if len(v) >= 3:
        return Vec3(v[0], v[1], v[2])
    if len(v) == 2:
        return Vec3(v[0], v[1], 0.0)
    return Vec3(0.0, 0.0, 0.0)


def _directed_to_coords(
    network: "RailNetwork", edge_id: int, direction: int,
) -> dict | None:
    """DirectedEdge (edge_id, direction) → {"a": 该端坐标, "b": 另一端坐标}。

    a 是"从哪端出发"：direction=+1 时 a = node_a；direction=-1 时 a = node_b。
    这样 a→b 顺序隐含行进方向，加载时据此反推 direction。
    """
    edge = network.edges.get(edge_id)
    if edge is None:
        return None
    na = network.nodes[edge.node_a_id].position
    nb = network.nodes[edge.node_b_id].position
    if direction > 0:
        return {"a": _vec3_list(na), "b": _vec3_list(nb)}
    return {"a": _vec3_list(nb), "b": _vec3_list(na)}


def _coords_to_directed(
    network: "RailNetwork", rec: dict,
) -> tuple[int, int] | None:
    """{"a","b"} 坐标 → DirectedEdge (edge_id, direction)，找不到返回 None。

    direction = +1 若该边几何 node_a 的坐标 ≈ a（即存的是"从 node_a 出发"），
    否则 -1。与 _directed_to_coords 互逆。
    """
    a = _coord(rec["a"])
    b = _coord(rec["b"])
    node_a_id = network.node_id_at(a, EPSILON)
    node_b_id = network.node_id_at(b, EPSILON)
    if node_a_id is None or node_b_id is None:
        return None
    node_a = network.nodes.get(node_a_id)
    if node_a is None:
        return None
    edge_id = None
    for eid in node_a.incident_edge_ids:
        edge = network.edges[eid]
        if {edge.node_a_id, edge.node_b_id} == {node_a_id, node_b_id}:
            edge_id = eid
            break
    if edge_id is None:
        return None
    edge = network.edges[edge_id]
    direction = 1 if edge.node_a_id == node_a_id else -1
    return edge_id, direction


def serialize_trains(trains: list["TrainEntity"]) -> list[dict]:
    """TrainEntity 列表 → JSON 对象列表。见 docs/session_persistence.md §2。"""
    result: list[dict] = []
    for train in trains:
        occ = train.state.occupancy
        wagons = []
        for w in train.state.consist.wagons:
            wagons.append({
                "wagon_id": w.wagon_id,
                "length": w.length,
                "mass": w.mass,
                "P_rated": w.P_rated,
                "coupler_1_pos": w.coupler_1_pos,
                "coupler_2_pos": w.coupler_2_pos,
                "bogies": [
                    {
                        "geometric_role": b.geometric_role.value,
                        "pos": b.pos,
                        "load_share": b.load_share,
                        "axle_count": b.axle_count,
                    }
                    for b in w.bogies
                ],
            })

        occupied = [
            _directed_to_coords(train.network, eid, d)
            for eid, d in occ.occupied
        ]
        route = [
            _directed_to_coords(train.network, eid, d)
            for eid, d in occ.route
        ]
        # 任一有向边反查失败（理论上不应发生——occupied/route 引用的边必在
        # 网络里），整列跳过（决策 4 容错）。
        if any(x is None for x in occupied + route):
            continue

        goal = None
        if train.state.goal is not None:
            geid, gt, gdir = train.state.goal
            gcoord = _directed_to_coords(train.network, geid, gdir)
            if gcoord is None:
                goal = None  # goal 边失效则当作无指令（容错）
            else:
                goal = {"edge": gcoord, "t": gt, "direction": gdir}

        result.append({
            "consist": wagons,
            "v": train.state.v,
            "v_target": train.v_target,
            "occupied": occupied,
            "occupied_offset": occ.occupied_offset,
            "s": occ.s,
            "route": route,
            "remaining_to_goal": train.state.remaining_to_goal,
            "goal": goal,
            "physics": PHYSICS_REALISTIC_ELECTRIC,
        })
    return result


def deserialize_trains(
    data: list[dict], network: "RailNetwork",
) -> list["TrainEntity"]:
    """JSON 对象列表 → TrainEntity 列表。反查失败的单列静默跳过（决策 4）。

    还原规则：
    - 停放（route 空）：controller=None，直接由 TrainEntity 构造（is_parked）。
    - 行驶中（route 非空）：构造后调 assign_route(route, remaining, goal) 重建
      controller；state.v 已写入。
    """
    from model.wagon import WagonConfig, Consist, BogieConfig, GeometricRole
    from model.occupancy import OccupancyState
    from model.train_entity import TrainEntity, TrainState
    from model.train_physics import RealisticElectric

    trains: list["TrainEntity"] = []
    for rec in data:
        # --- 反查有向边 ---
        occupied = []
        for r in rec.get("occupied", []):
            d = _coords_to_directed(network, r)
            if d is None:
                break
            occupied.append(d)
        else:
            pass
        if len(occupied) != len(rec.get("occupied", [])):
            continue  # 某条 occupied 边失效，跳过整列（决策 4）

        route = []
        for r in rec.get("route", []):
            d = _coords_to_directed(network, r)
            if d is None:
                break
            route.append(d)
        if len(route) != len(rec.get("route", [])):
            continue

        goal = None
        if rec.get("goal") is not None:
            g = rec["goal"]
            gd = _coords_to_directed(network, g["edge"])
            if gd is not None:
                goal = (gd[0], g["t"], g["direction"])

        # --- 重建车厢 ---
        wagons = []
        for w in rec.get("consist", []):
            bogies = [
                BogieConfig(
                    geometric_role=GeometricRole(b["geometric_role"]),
                    pos=b["pos"],
                    load_share=b.get("load_share", 0.5),
                    axle_count=b.get("axle_count", 2),
                )
                for b in w.get("bogies", [])
            ]
            wagons.append(WagonConfig(
                length=w["length"],
                mass=w["mass"],
                P_rated=w.get("P_rated"),
                coupler_1_pos=w.get("coupler_1_pos", 0.0),
                coupler_2_pos=w.get("coupler_2_pos", w["length"]),
                bogies=bogies,
                wagon_id=w["wagon_id"],
            ))
        if not wagons:
            continue

        consist = Consist(wagons=wagons)
        occ = OccupancyState(
            occupied=occupied,
            occupied_offset=rec.get("occupied_offset", 0.0),
            s=rec.get("s", 0.0),
            route=route,
        )
        state = TrainState(
            occupancy=occ,
            remaining_to_goal=rec.get("remaining_to_goal", 0.0),
            v=rec.get("v", 0.0),
            consist=consist,
            goal=goal,
        )
        train = TrainEntity(state, network, RealisticElectric())
        train.v_target = rec.get("v_target", 0.0)
        # 行驶中（有 route）：重建 controller
        if route:
            train.assign_route(route, max(0.0, rec.get("remaining_to_goal", 0.0)), goal)
        trains.append(train)
    return trains


def rebuild_split_siblings(trains: list["TrainEntity"]) -> None:
    """按"共享至少一条 occupied 边"动态重建 split_sibling 互指（决策 2）。

    与 model/dispatch.py::_consist_maneuver_exempt 的 split_sibling 豁免前置
    条件（共享边）语义等价：解挂后仍共享边的两段会重建互指、恢复分离驶离的
    信号豁免；已驶离（不再共享边）的两段本就不需要豁免。
    """
    n = len(trains)
    for i in range(n):
        a_edges = {eid for eid, _d in trains[i].state.occupancy.occupied}
        if not a_edges:
            continue
        for j in range(i + 1, n):
            b_edges = {eid for eid, _d in trains[j].state.occupancy.occupied}
            if a_edges & b_edges:
                trains[i].split_sibling = trains[j]
                trains[j].split_sibling = trains[i]


def rebuild_couple_partners(
    trains: list["TrainEntity"], network: RailNetwork,
) -> None:
    """会话重载后一次性重建 couple_approach_partner（决策 2 的连挂侧）。

    couple_approach_partner 是运行时对象引用、不落盘。带**未完成指令**的列车
    （controller 或 route 仍存在）若 goal 几何点落在另一停放列车端头车钩
    1m 内 = 保存时正驶向该车尾钩的连挂指令 → 重建配对。否则 dispatch 的调车
    全放行分支失效，重载后这类车会在信号前永久 holding（2026-09 bug2 现场：
    用户 3 边 2 信号最小复现，#2 驶向 #1 车尾走到 node 前卡死）。

    只在 load 时做一次、且只匹配"进行中指令"——不做每帧几何猜测，避免普通
    寻路碰巧把 goal 设在别人车钩旁的第三方列车被误豁免（回归安全网
    tests/test_couple_ui.py）。
    """
    tol = 1.0  # 车钩容差，与 COUPLE_DIST 一致
    parked = {
        t: t.kinematics.get_end_coupler_data(t.state.s)
        for t in trains if t.is_parked()
    }
    for t in trains:
        if t.is_parked() or t.couple_approach_partner is not None:
            continue
        goal = t.state.goal
        if goal is None:
            continue
        goal_pos = _edge_point(network, goal[0], goal[1])
        for other, (head_data, tail_data) in parked.items():
            for cp_pos, _eid, _t in (head_data, tail_data):
                if (cp_pos - goal_pos).length() <= tol:
                    t.couple_approach_partner = other
                    break
            if t.couple_approach_partner is not None:
                break


def _edge_point(network: RailNetwork, edge_id: int, t: float):
    """Edge 上参数 t（node_a→node_b）处的世界坐标（直/弧），goal 几何判定用。"""
    edge = network.edges[edge_id]
    na = network.nodes[edge.node_a_id].position
    nb = network.nodes[edge.node_b_id].position
    if not edge.is_arc:
        return na + (nb - na) * t
    from model.geom_utils import rotate_around_axis
    return edge.arc_center + rotate_around_axis(
        edge.arc_start_dir, edge.arc_normal, edge.arc_angle_rad * t
    ) * edge.arc_radius
