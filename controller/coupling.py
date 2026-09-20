"""编组连挂/解挂的几何配对判定（纯逻辑，无 pygame/view 依赖）。

对应 docs/consist_ui.md（F 阶段完整 UI 规格，2026-09 定稿）。数据层原语
（TrainEntity.decouple_at / couple_with，Consist.split_at / merged_with）
已经正确且不受影响——本模块只规约 controller 层的"哪列车的哪个端头可以跟
哪列车的哪个端头连挂 / 谁离谁最近"，供 GameLoop 交互与端到端测试共用。

关键语义（与 specs §5 一致）：
- 四种暴露端组合都可形成机械连接。头头/尾尾会先归一化其中一列的**逻辑**
  首尾，再按“前段尾钩 ↔ 后段头钩”合并；车厢对象自身的物理朝向不变。
- 连挂要求两车都停放（is_parked），数据层 couple_with/decouple_at 自带断言，
  这里不重复抛错，只返回判定结果，由调用方决定提示。

朝向判定的历史（重要，勿回退）：
早期实现用"两车首节车厢 heading 点积 > 0.9"判断"同向"，2026-09 在 manual_track
90° 弧（edge 29，半径 170m）上实测到误杀：两列**同向**停在弧上、相距约 87m、
寻路可达，但首节朝向随弧角分叉（点积 0.863 < 0.9）被误判"朝向不一致"。
根因是朝向参照端选错：连挂"追尾"接触的是 A 车头 + B **车尾**，首节朝向只代表
各自车头端，弧上首尾朝向随弧角分叉，长编组更甚。

修复（三层，判据按使用场景分家，勿回退）：
1. **已贴住（<1m）判定**用接触端各自的切线方向（end_heading：head 端取首节
   车厢 heading，tail 端取末节车厢 heading）——贴住时两端头位置几乎重合，切线
   天然共线，不会误杀；取点积绝对值，因此头对头也可连挂。
2. **驶向连挂（未贴住）预判**不再要求两列车逻辑同向，只扫描本车前进半平面内
   最近的目标暴露端，并保留护栏距离；**可达性交给 find_path_from_point**
   （支持折返 allow_reversal=True，隔弧/隔节点/隔岔路都能驶向）。见
   `drive_couple_goal` 与 docs/consist_ui.md §5.2（2026-09 bug1 放宽记录）。
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
    reverse_head: bool = False
    reverse_rear: bool = False


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
    return wagon.logical_front_bogie_pos - wagon.logical_front_coupler_pos


def _ends_aligned(a: "TrainEntity", a_end: str, b: "TrainEntity", b_end: str) -> bool:
    """接触端是否共线；同向和反向车厢都可形成有效机械连接。"""
    ha = end_heading(a, a_end)
    hb = end_heading(b, b_end)
    if ha is None or hb is None:
        return False
    return abs(ha.dot(hb)) >= HEADING_DOT


def couple_match_for_ends(
    train_a: "TrainEntity",
    a_end: str,
    train_b: "TrainEntity",
    b_end: str,
    max_dist: float = COUPLE_DIST,
) -> CoupleMatch | None:
    """把任意两个接触端归一化成“逻辑前段尾钩 ↔ 逻辑后段头钩”。

    头头或尾尾连接会在合并前逻辑折返其中一列。折返同时翻转车厢相对编组的
    orientation，因此不会改变任一车厢的世界位置或物理朝向。
    """
    if train_a is train_b or not train_a.is_parked() or not train_b.is_parked():
        return None
    a_data = end_coupler_pos(train_a, a_end)
    b_data = end_coupler_pos(train_b, b_end)
    distance = (a_data[0] - b_data[0]).length()
    if distance >= max_dist or not _ends_aligned(train_a, a_end, train_b, b_end):
        return None

    if a_end == "tail" and b_end == "head":
        return CoupleMatch(train_a, train_b, distance)
    if a_end == "head" and b_end == "tail":
        return CoupleMatch(train_b, train_a, distance)
    if a_end == "head" and b_end == "head":
        return CoupleMatch(train_b, train_a, distance, reverse_head=True)
    if a_end == "tail" and b_end == "tail":
        return CoupleMatch(train_a, train_b, distance, reverse_rear=True)
    return None


# 连挂驶向的护栏距离（米）：A 头钩 → B 尾钩 直线距离超过此值不下达。这只是
# 防"点到天边另一辆列车就开一整张图"的误触护栏——**真实可达性交给寻路**
# （_issue_goal_order → find_path_from_point，支持折返/绕行），不再做"就近微调"
# 级别的几何预判（2026-09 bug1 用户拍板放宽，见 docs/consist_ui.md §5.2）。
DRIVE_COUPLE_MAX_DIST = 2000.0


def drive_couple_goal(
    train_a: "TrainEntity",
    target_train: "TrainEntity",
) -> tuple[int, float] | None:
    """选择 train_a 前进方向上最近的目标暴露端，返回目标 (edge_id, t)。

    本仓库**没有倒车能力**（`advance_occupied_path` 只沿 route 正向推进，
    `reverse_in_place` 是原地逻辑换向不是物理倒车）。目标可以同向或反向停放：
    头对头接触后可通过逻辑方向归一化连接，车厢物理朝向保持不变。

    2026-09 bug1 放宽记录（用户拍板"先放宽就近对接门槛"）：旧版两处误杀——
    1) 端头切线一致判据（`_ends_aligned`，HEADING_DOT=0.9 ≈ 25.8°）：同向两车
    在**同一条弧上相距较远**时，端头切线随弧角分叉（如 170m 半径弧相距 87m →
    夹角 29° → dot 0.875），合法追尾被拒。贴住判据用它没问题（贴住时端头几乎
    同点、切线天然一致），但驶向预判不能用。
    2) 300m 直线距离上限：合法远距驶向被拒。
    放宽后：
    - **同向判据用整列车列方向**（头钩−尾钩连线近似，弧上同向几乎不随弧角
      分叉）：(A 头钩−A 尾钩)·(B 头钩−B 尾钩) > 0。直轨对顶/逆行（B 车头朝
      A 来向）→ 点积 < 0 被拒（那种布局 A 开过去也接不上，需倒车/掉头）。
    - 距离上限 300 → DRIVE_COUPLE_MAX_DIST（护栏，防误触天边列车）。
    - 保留"B 尾钩在 A 前进方向的前半平面"判据：防"A 已越过 B 尾"时寻路绕大圈/
      折返边（2026-09 绕行 bug）。弧线上前提是弧角 < 180°（manual_track 最大
      弧 90°，安全；大弧/S 弯的极端误杀留给寻路不可达兜底提示）。

    返回 None = 不可"前进对接"（已越过 / 逆行对顶 / 相距超护栏 / 未停），
    调用方提示用户。返回 (edge_id, t) = B 尾钩所在 edge/t，调用方用
    `_issue_goal_order(..., stop_before_m=head_hook_offset(A))` 下达。
    """
    if not train_a.is_parked() or not target_train.is_parked():
        return None

    a_head = end_coupler_pos(train_a, "head")
    a_tail = end_coupler_pos(train_a, "tail")
    b_tail = end_coupler_pos(target_train, "tail")
    b_head = end_coupler_pos(target_train, "head")

    a_forward = a_head[0] - a_tail[0]
    candidates: list[tuple[float, int, float]] = []
    for endpoint in (b_head, b_tail):
        delta = endpoint[0] - a_head[0]
        distance = delta.length()
        if (0.0 < distance <= DRIVE_COUPLE_MAX_DIST
                and delta.dot(a_forward) > 0.0):
            candidates.append((distance, endpoint[1], endpoint[2]))
    if not candidates:
        return None
    _distance, edge_id, t = min(candidates)
    return edge_id, t


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

        for a_end, b_end in (
            ("tail", "head"), ("head", "tail"),
            ("head", "head"), ("tail", "tail"),
        ):
            match = couple_match_for_ends(train_a, a_end, other, b_end, max_dist)
            if match is not None and (best is None or match.pair_dist < best.pair_dist):
                best = match

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

    if target_end not in ("head", "tail"):
        return None
    # 明确目标端时，尝试本车两个暴露端；计划连挂会直接指定本车 head。
    best = None
    for own_end in ("head", "tail"):
        match = couple_match_for_ends(
            train_a, own_end, target_train, target_end, max_dist,
        )
        if match is not None and (best is None or match.pair_dist < best.pair_dist):
            best = match
    return best
