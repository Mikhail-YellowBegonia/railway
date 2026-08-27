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
    s: 车头（head）弧长，**相对 occupied_offset**（0 = 恰好在偏移点，
       与 RigidWagonKinematics.initial_offset 的语义一致）。
       绝对弧长（从 occupied[0] 真实起点算）= s + occupied_offset。
    route: 待走的有向边（寻路结果中，head 尚未走到的部分）。为空 = 无更多指令。
    """
    occupied: list[DirectedEdge]
    occupied_offset: float
    s: float
    route: list[DirectedEdge]


def _is_reversal_pair(a: DirectedEdge, b: DirectedEdge) -> bool:
    """判断 a→b 是否构成折返对（同 edge_id，方向相反）。

    这是 Dijkstra 在死端插入的反向同边（见 pathfinding.neighbors 的
    allow_reversal 分支），route 里出现这样的相邻对，代表列车需要先停车、
    原地翻转，才能继续往下走——不是可以直接穿越的普通边界。
    """
    return a[0] == b[0] and a[1] == -b[1]


def advance_occupied_path(
    network: RailNetwork,
    state: OccupancyState,
    delta_s: float,
    consist_length: float,
) -> tuple[OccupancyState, bool, bool]:
    """推进车头弧长 delta_s，消费 route，滑动裁剪 occupied 头部。

    坐标约定：state.s 是相对 state.occupied_offset 的坐标（0 = 车尾偏移点，
    与 RigidWagonKinematics.initial_offset 语义一致）。内部换算成绝对坐标
    （x=0 在 occupied[0] 当前真实起点）做边界比较和裁剪，最后再换算回来。

    不做任何道岔判断——走哪条边由 route 提前给定，这里只处理边界越过
    （occupied 追加）和窗口裁剪（occupied 头部弹出）。

    折返点处理：若即将从 route 消费的下一条边与当前 occupied 末端构成
    折返对（_is_reversal_pair），则**不消费它**，把车头精确 clamp 在边界
    处，丢弃这条折返标记边（它只是 Dijkstra 的记账产物，reverse_in_place
    会让 occupied 自然产生等价的反向边，不需要重复），并返回
    needs_reversal=True。当帧剩余的 delta_s 直接丢弃（简化：折返视为瞬间
    停车，不做减速曲线，后续如需要可在上层补）。

    返回:
        (新状态, reached_end, needs_reversal)
        reached_end=True 表示 route 已耗尽且到达终点（s 被 clamp）。
        needs_reversal=True 表示车头已停在折返点，调用方需在停车状态下
        调用 TrainEntity.reverse_in_place()，再用返回的 state.route 继续
        （折返标记边已被弹出，route 已经是"折返后应继续走"的剩余部分）。
        两者不会同时为 True。
    """
    occupied = list(state.occupied)
    route = list(state.route)

    def edge_len(directed: DirectedEdge) -> float:
        return network.edges[directed[0]].length

    # 换算到绝对坐标：head 的绝对弧长
    abs_head = state.s + state.occupied_offset + delta_s
    total_len = sum(edge_len(d) for d in occupied)
    reached_end = False
    needs_reversal = False

    # head 越过 occupied 末端：从 route 消费新边
    while abs_head > total_len + 1e-9:
        if not route:
            abs_head = total_len
            reached_end = True
            break
        if occupied and _is_reversal_pair(occupied[-1], route[0]):
            # 折返点：停在当前边界，丢弃折返标记边，不再继续本帧推进
            route.pop(0)
            abs_head = total_len
            needs_reversal = True
            break
        next_edge = route.pop(0)
        occupied.append(next_edge)
        total_len += edge_len(next_edge)

    # 滑动裁剪：从 occupied 头部移除已经甩出车尾的边（绝对坐标原点跟着右移）
    abs_tail = abs_head - consist_length
    while len(occupied) > 1:
        first_len = edge_len(occupied[0])
        if abs_tail > first_len + 1e-9:
            occupied.pop(0)
            abs_head -= first_len
            abs_tail -= first_len
        else:
            break

    # 换算回 (occupied_offset, s) 表示：offset = tail 的绝对位置（clamp >= 0）
    new_offset = max(0.0, abs_tail)
    new_s = abs_head - new_offset

    new_state = OccupancyState(
        occupied=occupied,
        occupied_offset=new_offset,
        s=new_s,
        route=route,
    )
    return new_state, reached_end, needs_reversal


def truncate_to_segment(
    state: OccupancyState,
    segment_edge_ids: set[int],
    network: RailNetwork,
) -> OccupancyState:
    """丢弃 occupied 中不属于 simple_segment 的前缀（turnout 背面的边）。

    折返只应发生在 endpoint，车身占用范围理应完全落在该 endpoint 对应的
    simple_segment 内（前提：pathfinding.neighbors 已经用 consist_length
    过滤掉了 segment 长度不足的折返选项，见 Step3 阶段 B）。但 occupied
    是按整条边的粒度滑动裁剪的，可能因为窗口余量而多留了一段 turnout
    背面的边（即上一次巡航路径的残留），这段必须在折返前丢弃，否则
    reverse_occupancy 会把 turnout 背面的边也整体反转，导致车头折返后
    继续沿着错误的方向走上原路径（已复现：这正是最初死循环 bug 的成因，
    不是几何插值问题，是 occupied 里混入了不该反转的边）。

    安全性：只丢弃"绝对确定在车身之外"的前缀——occupied_offset 已验证
    是真实车尾位置的低估值（occupied_offset <= 真实车尾弧长），所以只要
    丢弃前缀总长 <= occupied_offset，就不可能切入车身本体。若不属于
    segment 的前缀比 occupied_offset 还长（理论上不该发生，因为 pathfinding
    已保证 segment 长度 >= consist_length），则保守地只丢弃到 occupied_offset
    为止，不冒险继续丢，宁可保留多余的边也不截断进车身。
    """
    occupied = list(state.occupied)
    drop_count = 0
    dropped_length = 0.0

    for directed in occupied:
        if directed[0] in segment_edge_ids:
            break
        edge_len = network.edges[directed[0]].length
        if dropped_length + edge_len > state.occupied_offset + 1e-6:
            # 再丢这条边会切入车身本体范围，停止（保守，见上方文档）
            break
        dropped_length += edge_len
        drop_count += 1

    if drop_count == 0:
        return state

    new_occupied = occupied[drop_count:]
    if not new_occupied:
        # 极端情况：整条 occupied 都不在 segment 内且已耗尽安全边界，不截断
        return state

    new_offset = max(0.0, state.occupied_offset - dropped_length)
    return OccupancyState(
        occupied=new_occupied,
        occupied_offset=new_offset,
        s=state.s,
        route=state.route,
    )


def occupied_as_path(state: OccupancyState, network: RailNetwork) -> Path:
    """把 occupied 边列表转换为 Path（供 PathKinematics/渲染使用）。"""
    total_cost = sum(network.edges[e[0]].length for e in state.occupied)
    return Path(edges=list(state.occupied), total_cost=total_cost)


def reverse_occupancy(
    network: RailNetwork,
    state: OccupancyState,
    real_tail_bogie_abs_s: float,
    consist_length: float,
) -> OccupancyState:
    """原地折返：整体反转 occupied 边序列的行进方向，车头车尾互换。

    这不是"转向架反弹"——车身物理占用的边集合完全不变，只是重新定义
    哪一端是车头、哪一端是车尾（对应 OpenTTD 的 flip-in-place 语义）。
    route 由调用方负责清空（折返后旧指令失效，需要基于新车头重新寻路）。

    real_tail_bogie_abs_s: 车身真实车尾——末节车厢**后转向架**（不是车钩）
    的绝对弧长坐标（在 occupied[0] 起点为 0 的坐标系下）。**必须**是转向架
    坐标，因为 get_all_wagon_poses 消费的 front_bogie_s 参数本身是转向架
    弧长，用车钩弧长做镜像基准会因转向架相对车钩的内缩产生系统偏差（已
    验证：偏差量级等于 bogies[0].pos，即转向架到车钩的距离）。
    调用方用 RigidWagonKinematics.real_tail_bogie_abs_s() 获取，不能用
    state.occupied_offset 代替——occupied_offset 只是滑动窗口裁剪用的粗略
    近似（consist_length 之差，忽略了转向架相对车钩的内缩），用它做镜像
    基准会让折返后车厢位置和折返前不一致（已用真实车厢位置验证过这个
    偏差，量级与转向架内缩相关，通常几米）。

    推导（world_dir(d, x) = 方向 d 下局部坐标 x 对应的世界坐标，
    total_len = occupied 总弧长；方向反转的通用镜像关系：
    world(-d, x) = world(d, total_len - x)）：

    折返定义：新头的世界位置 = 旧尾（真实）的世界位置。代入镜像关系：
    - new_abs_head = total_len - real_tail_bogie_abs_s
    - new_occupied_offset：沿用 advance_occupied_path 同样的近似规则
      max(0, new_abs_head - consist_length)，保持两处"窗口尾部定义"一致
    """
    def edge_len(directed: DirectedEdge) -> float:
        return network.edges[directed[0]].length

    total_len = sum(edge_len(d) for d in state.occupied)
    new_occupied = [(eid, -d) for eid, d in reversed(state.occupied)]
    new_abs_head = total_len - real_tail_bogie_abs_s
    new_offset = max(0.0, new_abs_head - consist_length)
    new_s = new_abs_head - new_offset

    return OccupancyState(
        occupied=new_occupied,
        occupied_offset=new_offset,
        s=new_s,
        route=[],
    )
