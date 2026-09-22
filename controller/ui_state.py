from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto


class UIContext(Enum):
    """Input/help context, ordered from the most specific overlay to workspace modes."""

    IDLE = auto()
    BUILD_READY = auto()
    BUILD_ACTIVE = auto()
    DELETE = auto()
    SIGNAL = auto()
    POI = auto()
    PLAY = auto()
    PLAN_EDIT = auto()
    CONSIST_PANEL = auto()
    SCHEDULE_MENU = auto()


class FeedbackLevel(Enum):
    INFO = auto()
    SUCCESS = auto()
    WARNING = auto()
    ERROR = auto()


@dataclass(frozen=True)
class ActionHint:
    action_id: str
    binding: str
    label: str


@dataclass(frozen=True)
class FeedbackMessage:
    text: str
    level: FeedbackLevel
    expires_at_ms: int


_CONTEXT_HINTS: dict[UIContext, tuple[ActionHint, ...]] = {
    UIContext.IDLE: (
        ActionHint("mode.build", "B", "建造"), ActionHint("mode.delete", "D", "删除"),
        ActionHint("mode.signal", "H", "信号"), ActionHint("mode.poi", "J", "POI"),
        ActionHint("mode.play", "P", "游玩"),
    ),
    UIContext.BUILD_READY: (
        ActionHint("build.start", "左键", "起点"), ActionHint("build.snap", "G/L/A", "吸附"),
        ActionHint("build.parallel_snap", "Ctrl+P", "平行吸附"),
        ActionHint("mode.cancel", "Esc", "退出"),
    ),
    UIContext.BUILD_ACTIVE: (
        ActionHint("build.commit", "左键", "确认"),
        ActionHint("build.cancel", "右键/Esc", "取消"),
        ActionHint("build.force_straight", "Shift", "强制直线"),
        ActionHint("build.case2t", "Alt", "Case 2T"),
    ),
    UIContext.DELETE: (
        ActionHint("delete.hovered", "左键", "删除悬停对象"),
        ActionHint("mode.cancel", "Esc", "退出"),
    ),
    UIContext.SIGNAL: (
        ActionHint("signal.toggle", "左键", "放置/切换信号"),
        ActionHint("mode.cancel", "H/Esc", "退出"),
    ),
    UIContext.POI: (
        ActionHint("poi.toggle_member", "左键", "选择成员"),
        ActionHint("poi.create", "Enter", "创建"),
        ActionHint("station.create", "T", "创建 Station"),
        ActionHint("poi.undo_member", "Backspace", "撤销成员"),
        ActionHint("poi.delete_hovered", "Delete", "删除悬停 POI"),
        ActionHint("mode.cancel", "J/Esc", "退出"),
    ),
    UIContext.PLAY: (
        ActionHint("play.select", "左键", "选择/放置"),
        ActionHint("play.issue_order", "右键", "下达目的地"),
        ActionHint("plan.edit", "P", "编辑计划"), ActionHint("consist.open", "I", "编组"),
        ActionHint("schedule.open", "O", "计划菜单"),
        ActionHint("train.stop_or_pause", "Space", "停车/暂停"),
    ),
    UIContext.PLAN_EDIT: (
        ActionHint("plan.add_anchor", "左键", "锚点/目标"),
        ActionHint("plan.confirm_item", "Enter", "确认条目"),
        ActionHint("plan.add_command", "K/R/W", "连挂/折返/等待"),
        ActionHint("plan.undo", "Backspace", "撤销"),
        ActionHint("plan.finish", "P/Esc", "完成编辑"),
    ),
    UIContext.CONSIST_PANEL: (
        ActionHint("consist.select_wagon", "1-9", "选择车厢"),
        ActionHint("consist.priority", "[/]", "优先级"),
        ActionHint("consist.toggle_control", "C", "控制车"),
        ActionHint("consist.close", "I", "关闭"),
    ),
    UIContext.SCHEDULE_MENU: (
        ActionHint("plan.pause", "Space", "暂停/继续"),
        ActionHint("plan.edit", "P", "编辑计划"),
        ActionHint("schedule.close", "O/Esc", "关闭"),
    ),
}


def action_hints(context: UIContext) -> tuple[ActionHint, ...]:
    return _CONTEXT_HINTS[context]


class FeedbackCenter:
    """Short-lived player feedback; detailed diagnostics remain in the terminal."""

    def __init__(self) -> None:
        self._message: FeedbackMessage | None = None

    def post(
        self,
        text: str,
        now_ms: int,
        level: FeedbackLevel = FeedbackLevel.INFO,
        duration_ms: int | None = None,
    ) -> None:
        if duration_ms is None:
            duration_ms = 4500 if level in (FeedbackLevel.WARNING, FeedbackLevel.ERROR) else 2800
        self._message = FeedbackMessage(text, level, now_ms + duration_ms)

    def current(self, now_ms: int) -> FeedbackMessage | None:
        if self._message is not None and now_ms >= self._message.expires_at_ms:
            self._message = None
        return self._message

    def clear(self) -> None:
        self._message = None
