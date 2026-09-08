"""编组连挂/解挂的几何配对判定（纯逻辑，无 pygame/view 依赖）。

对应 docs/consist_ui.md（F 阶段完整 UI 规格，2026-09 定稿）。数据层原语
（TrainEntity.decouple_at / couple_with，Consist.split_at / merged_with）
已经正确且不受影响——本模块只规约 controller 层的"哪列车的哪个端头可以跟
哪列车的哪个端头连挂 / 谁离谁最近"，供 GameLoop 交互与端到端测试共用。

关键语义（与 specs §5 一致）：
- 物理配对只允许"反向端"贴一起：A 车尾钩 ↔ B 车头钩，或 A 车头钩 ↔ B 车尾钩。
  车头对车头 / 车尾对车尾一律拒绝（heading 点积 > HEADING_DOT 已隐含此约束）。
- 连挂要求两车都停放（is_parked），数据层 couple_with/decouple_at 自带断言，
  这里不重复抛错，只返回判定结果，由调用方决定提示。
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


def _train_heading(train: "TrainEntity"):
    """首节车厢几何朝向（连线方向），供朝向一致性检查。"""
    poses = train.kinematics.get_all_wagon_poses(train.state.s)
    if not poses:
        return None
    return poses[0].heading


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


def find_couple_pair(
    train_a: "TrainEntity",
    others: list["TrainEntity"],
    max_dist: float = COUPLE_DIST,
) -> CoupleMatch | None:
    """在 others 中找可与 train_a 连挂的最近配对。

    规则（specs §5.1/§5.3）：
    - 只检查合法"反向端"组合：A 尾 ↔ B 头、A 头 ↔ B 尾。
    - 两端头距离 < max_dist 且两车 heading 点积 > HEADING_DOT。
    - 多个候选满足时返回端头距离最近的一对（并列按 others 顺序）。
    - train_a 必须停放（is_parked），否则返回 None（调用方负责提示）。

    返回 CoupleMatch 或 None；不修改任何状态，纯判定。
    """
    if not train_a.is_parked():
        return None

    a_head = end_coupler_pos(train_a, "head")
    a_tail = end_coupler_pos(train_a, "tail")
    a_heading = _train_heading(train_a)

    best: CoupleMatch | None = None
    for other in others:
        if other is train_a or not other.is_parked():
            continue
        b_head = end_coupler_pos(other, "head")
        b_tail = end_coupler_pos(other, "tail")
        o_heading = _train_heading(other)
        if a_heading is None or o_heading is None:
            continue
        if a_heading.dot(o_heading) < HEADING_DOT:
            continue

        # A 尾 ↔ B 头：A 在前面、B 跟在后面 → A 当头，B 挂 A 尾
        d1 = (a_tail[0] - b_head[0]).length()
        # A 头 ↔ B 尾：B 在前面、A 跟在后面 → B 当头，A 挂 B 尾
        d2 = (a_head[0] - b_tail[0]).length()

        if d1 <= d2:
            d = d1
            if d < max_dist and (best is None or d < best.pair_dist):
                best = CoupleMatch(merged_head=train_a, merged_rear=other, pair_dist=d)
        else:
            d = d2
            if d < max_dist and (best is None or d < best.pair_dist):
                best = CoupleMatch(merged_head=other, merged_rear=train_a, pair_dist=d)

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
    - target_end='tail' → 只允许 A 车头钩 ↔ B 车尾钩（A 追尾 B）。
    两端头距离 < max_dist 且 heading 点积 > HEADING_DOT 才返回 CoupleMatch。

    与 find_couple_pair 的区别：后者在全部 others 里找最近的一对（停车事件自动
    连挂用）；这里绑定到玩家悬停的明确目标（不歧义，specs §5.3）。

    两车都必须停放；不修改任何状态，纯判定。
    """
    if not train_a.is_parked() or not target_train.is_parked():
        return None

    a_head = end_coupler_pos(train_a, "head")
    a_tail = end_coupler_pos(train_a, "tail")
    a_heading = _train_heading(train_a)
    o_heading = _train_heading(target_train)
    if a_heading is None or o_heading is None:
        return None
    if a_heading.dot(o_heading) < HEADING_DOT:
        return None

    if target_end == "head":
        b_head = end_coupler_pos(target_train, "head")
        d = (a_tail[0] - b_head[0]).length()
        if d < max_dist:
            return CoupleMatch(merged_head=train_a, merged_rear=target_train, pair_dist=d)
    else:  # target_end == 'tail'
        b_tail = end_coupler_pos(target_train, "tail")
        d = (a_head[0] - b_tail[0]).length()
        if d < max_dist:
            return CoupleMatch(merged_head=target_train, merged_rear=train_a, pair_dist=d)
    return None
