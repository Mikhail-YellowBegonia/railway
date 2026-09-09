"""编组连挂/解挂的几何配对判定（纯逻辑，无 pygame/view 依赖）。

对应 docs/consist_ui.md（F 阶段完整 UI 规格，2026-09 定稿）。数据层原语
（TrainEntity.decouple_at / couple_with，Consist.split_at / merged_with）
已经正确且不受影响——本模块只规约 controller 层的"哪列车的哪个端头可以跟
哪列车的哪个端头连挂 / 谁离谁最近"，供 GameLoop 交互与端到端测试共用。

关键语义（与 specs §5 一致）：
- 物理配对只允许"反向端"贴一起：A 车尾钩 ↔ B 车头钩，或 A 车头钩 ↔ B 车尾钩。
  车头对车头 / 车尾对车尾一律拒绝。
- 连挂要求两车都停放（is_parked），数据层 couple_with/decouple_at 自带断言，
  这里不重复抛错，只返回判定结果，由调用方决定提示。

朝向判定的历史（重要，勿回退）：
早期实现用"两车首节车厢 heading 点积 > 0.9"判断"同向"，2026-09 在 manual_track
90° 弧（edge 29，半径 170m）上实测到误杀：两列**同向**停在弧上、相距约 87m、
寻路可达，但首节朝向随弧角分叉（点积 0.863 < 0.9）被误判"朝向不一致"。
根因是朝向参照端选错：连挂"追尾"接触的是 A 车头 + B **车尾**，首节朝向只代表
各自车头端，弧上首尾朝向随弧角分叉，长编组更甚。

修复（两层）：
1. 朝向判据改为**接触端各自的切线方向**（end_heading：head 端取首节车厢
   heading，tail 端取末节车厢 heading）——已贴住（<1m）时两端头位置几乎重合，
   切线天然一致，不会误杀；反向重叠（头对头贴住）时接触端切线相反，正确拒绝。
2. `game_loop._couple_to_hovered` 的"驶向分支"不再做整车朝向预检——可达性
   交给 find_path_from_point（支持折返 allow_reversal=True）。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from model.train_entity import TrainEntity

# 端头车钩连挂判定的距离/朝向阈值（世界米制）。与既有 _try_couple 一致。
COUPLE_DIST = 1.0
HEADING_DOT = 0.9


@dataclass(frozen=True)
class CoupleMatch:
    """一次可执行的连挂配对：谁挂到谁车尾。

    merged_head: 连挂后成为车头（wagons[0]）的列车实体
    merged_rear: 被挂到车尾（追加到车尾）的列车实体
    pair_dist:    两个端头车钩之间的距离（米，判定时的实测值）
    """

    merged_head: "TrainEntity"
    merged_rear: "TrainEntity"
    pair_dist: float


def end_coupler_pos(train: "TrainEntity", which: str):
    """返回 train 端头车钩的 (世界坐标, edge_id, t)。

    which: 'head' = 车头前钩（首节 coupler_1 端），'tail' = 车尾后钩
    （末节 coupler_2 端）。返回值来自 RigidWagonKinematics.get_end_coupler_data，
    顺序是 [head, tail]。
    """
    data = train.kinematics.get_end_coupler_data(train.state.s)
    return data[0] if which == "head" else data[1]


def end_heading(train: "TrainEntity", which: str):
    """端头车钩处的切线方向（近似为该端车厢的连线 heading）。

    which='head' → 首节车厢 heading（车头端）。
    which='tail' → 末节车厢 heading（车尾端）。
    连挂的朝向一致性必须按"接触端"各取各的：A头↔B尾 用 A首节·B末节，
    A尾↔B头 用 A末节·B首节（docs/consist_ui.md §5.1）。
    """
    poses = train.kinematics.get_all_wagon_poses(train.state.s)
    if not poses:
        return None
    return poses[0].heading if which == "head" else poses[-1].heading


def head_hook_offset(train: "TrainEntity") -> float:
    """车头前车钩到首节车头转向架的弧长差（米）。

    get_end_coupler_data 里车头钩位置 = 头转向架位姿 + heading * head_offset
    （head_offset = bogies[0].pos - coupler_1_pos）。下达"驶向对方车尾"的指令时，
    目标 (edge_id, t) 是转向架停车点；要让**车钩**精确停在对方尾钩处，需提前
    head_offset 停车（见 controller/game_loop.py::_issue_goal_order 的
    stop_before_m 参数）。
    """
    wagon = train.state.consist.wagons[0]
    return wagon.bogies[0].pos - wagon.coupler_1_pos


def _ends_aligned(a: "TrainEntity", a_end: str, b: "TrainEntity", b_end: str) -> bool:
    """接触端切线方向是否一致（点积 > HEADING_DOT）。按端各取各的 heading。"""
    ha = end_heading(a, a_end)
    hb = end_heading(b, b_end)
    if ha is None or hb is None:
        return False
    return ha.dot(hb) >= HEADING_DOT


# 连挂驶向的最大"就近"距离（米）：两车端头相距超过此值视为"不相邻"，
# 不下达驶向连挂指令（避免长途寻路/绕大圈/折返边，见 docs/consist_ui.md §5.5）。
# 语义是"就近微调对齐"，不是"长途寻路"。
DRIVE_COUPLE_MAX_DIST = 300.0


def drive_couple_goal(
    train_a: "TrainEntity",
    target_train: "TrainEntity",
) -> tuple[int, float] | None:
    """判定 train_a 能否"前进驶向 target_train 车尾"完成连挂，返回目标 (edge_id, t)。

    就近对接语义（2026-09 用户拍板，docs/consist_ui.md §5.2）：连挂是"就近微调
    对齐"，不是长途寻路。本仓库**没有倒车能力**（`advance_occupied_path` 只沿
    route 正向推进，`reverse_in_place` 是原地掉头不是物理倒车），因此只支持
    情形：A 车头朝前、B 尾钩在 A 前方、同向、就近。

    判定依据（纯几何，不依赖寻路）：
    - 两车都停放；
    - 接触端切线一致（A 头端 · B 尾端，_ends_aligned）；
    - A 头钩 → B 尾钩 的直线距离在 (0, DRIVE_COUPLE_MAX_DIST] 内（就近）；
    - **B 尾钩在 A 前进方向的前半平面**：A 头钩指向 B 尾钩的方向 与 A 头端
      切线 heading 的夹角 < 90°。这一条是区分"前方可对接" vs "身后需倒车"
      的关键——身后时夹角 ≈180° 被拒，避免寻路绕大圈/折返边（2026-09 bug）。
      弧线上成立的前提是弧角 < 180°（弦方向总在起点切线前半平面），
      manual_track 最大弧 90°，安全。

    返回 None = 不可"前进对接"（已越过需倒车 / 不相邻 / 朝向不符 / 未停），
    调用方提示用户。返回 (edge_id, t) = B 尾钩所在 edge/t，调用方用
    `_issue_goal_order(..., stop_before_m=head_hook_offset(A))` 下达。
    """
    if not train_a.is_parked() or not target_train.is_parked():
        return None

    a_head = end_coupler_pos(train_a, "head")
    b_tail = end_coupler_pos(target_train, "tail")

    # 接触端切线一致性（A 头端 · B 尾端）。
    if not _ends_aligned(train_a, "head", target_train, "tail"):
        return None

    # 就近 + 前方判定。
    delta = b_tail[0] - a_head[0]
    dist = delta.length()
    if not (0.0 < dist <= DRIVE_COUPLE_MAX_DIST):
        return None
    a_heading = end_heading(train_a, "head")
    if a_heading is None:
        return None
    # 沿 A 前进方向的分量：> 0 表示 B 尾钩在 A 前方。
    along = delta.dot(a_heading)
    if along <= 0.0:
        return None

    return (b_tail[1], b_tail[2])


def find_couple_pair(
    train_a: "TrainEntity",
    others: list["TrainEntity"],
    max_dist: float = COUPLE_DIST,
) -> CoupleMatch | None:
    """在 others 中找可与 train_a 连挂的最近配对。

    规则（specs §5.1/§5.3）：
    - 只检查合法"反向端"组合：A 尾 ↔ B 头、A 头 ↔ B 尾。
    - 两端头距离 < max_dist，且接触端切线方向一致（_ends_aligned）。
    - 多个候选满足时返回端头距离最近的一对（并列按 others 顺序）。
    - train_a 必须停放（is_parked），否则返回 None（调用方负责提示）。

    返回 CoupleMatch 或 None；不修改任何状态，纯判定。
    """
    if not train_a.is_parked():
        return None

    a_head = end_coupler_pos(train_a, "head")
    a_tail = end_coupler_pos(train_a, "tail")

    best: CoupleMatch | None = None
    for other in others:
        if other is train_a or not other.is_parked():
            continue
        b_head = end_coupler_pos(other, "head")
        b_tail = end_coupler_pos(other, "tail")

        # A 尾 ↔ B 头：A 在前面、B 跟在后面 → A 当头，B 挂 A 尾。
        # 接触端 = A 车尾端 · B 车头端。
        d1 = (a_tail[0] - b_head[0]).length()
        if d1 < max_dist and _ends_aligned(train_a, "tail", other, "head"):
            if best is None or d1 < best.pair_dist:
                best = CoupleMatch(merged_head=train_a, merged_rear=other, pair_dist=d1)

        # A 头 ↔ B 尾：B 在前面、A 跟在后面 → B 当头，A 挂 B 尾。
        # 接触端 = A 车头端 · B 车尾端。
        d2 = (a_head[0] - b_tail[0]).length()
        if d2 < max_dist and _ends_aligned(train_a, "head", other, "tail"):
            if best is None or d2 < best.pair_dist:
                best = CoupleMatch(merged_head=other, merged_rear=train_a, pair_dist=d2)

    return best


def try_couple_to(
    train_a: "TrainEntity",
    target_train: "TrainEntity",
    target_end: str,
    max_dist: float = COUPLE_DIST,
) -> CoupleMatch | None:
    """K 键手动连挂：只尝试与悬停端头（target_train 的 head/tail）这一对配对。

    规则（specs §5.1 反向端配对）：
    - target_end='head' → 只允许 A 车尾钩 ↔ B 车头钩（A 在前面被 B 追尾）。
      接触端 = A 车尾端 · B 车头端。
    - target_end='tail' → 只允许 A 车头钩 ↔ B 车尾钩（A 追尾 B）。
      接触端 = A 车头端 · B 车尾端。
    两端头距离 < max_dist 且接触端切线一致才返回 CoupleMatch。

    与 find_couple_pair 的区别：后者在全部 others 里找最近的一对（停车事件自动
    连挂用）；这里绑定到玩家悬停的明确目标（不歧义，specs §5.3）。

    两车都必须停放；不修改任何状态，纯判定。
    """
    if not train_a.is_parked() or not target_train.is_parked():
        return None

    if target_end == "head":
        b_head = end_coupler_pos(target_train, "head")
        a_tail = end_coupler_pos(train_a, "tail")
        d = (a_tail[0] - b_head[0]).length()
        if d < max_dist and _ends_aligned(train_a, "tail", target_train, "head"):
            return CoupleMatch(merged_head=train_a, merged_rear=target_train, pair_dist=d)
    else:  # target_end == 'tail'
        b_tail = end_coupler_pos(target_train, "tail")
        a_head = end_coupler_pos(train_a, "head")
        d = (a_head[0] - b_tail[0]).length()
        if d < max_dist and _ends_aligned(train_a, "head", target_train, "tail"):
            return CoupleMatch(merged_head=target_train, merged_rear=train_a, pair_dist=d)
    return None
