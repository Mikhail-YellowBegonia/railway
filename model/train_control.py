"""Explicit runtime ownership of a train's movement command."""
from __future__ import annotations

from enum import Enum


class TrainControlAuthority(str, Enum):
    NONE = "none"
    PLAN = "plan"
    PLAN_PAUSED = "plan_paused"
    MANUAL_TAKEOVER = "manual_takeover"


def authority_label(authority: TrainControlAuthority) -> str:
    return {
        TrainControlAuthority.NONE: "无运行指令",
        TrainControlAuthority.PLAN: "调度计划",
        TrainControlAuthority.PLAN_PAUSED: "计划已暂停",
        TrainControlAuthority.MANUAL_TAKEOVER: "临时手动接管",
    }[authority]
