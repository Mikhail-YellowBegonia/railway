"""UI 上下文、短时反馈、提示绘制与 Ctrl+P 路由回归。"""
from __future__ import annotations

import os
from types import SimpleNamespace

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame
import pygame_gui

from controller.editor import EditMode
from controller.game_loop import GameLoop
from controller.ui_state import (
    FeedbackCenter,
    FeedbackLevel,
    UIContext,
    action_hints,
)
from view.ui_overlay import draw_context_bar, draw_feedback
from view.gui_layer import GameGUI


# ① 动作提示按上下文集中管理；BUILD 不再与 PLAY 共用裸 P。
build_bindings = {hint.binding: hint.label for hint in action_hints(UIContext.BUILD_READY)}
play_bindings = {hint.binding: hint.label for hint in action_hints(UIContext.PLAY)}
assert build_bindings["Ctrl+P"] == "平行吸附"
assert "P" not in build_bindings
assert play_bindings["P"] == "编辑计划"
for context in UIContext:
    ids = [hint.action_id for hint in action_hints(context)]
    assert all(ids) and len(ids) == len(set(ids))
print("✅ ① BUILD 平行吸附与 PLAY/计划编辑不再争用裸 P")


# ② 短时反馈按时间失效，警告级别可独立传递给 view。
feedback = FeedbackCenter()
feedback.post("测试警告", 1000, FeedbackLevel.WARNING, duration_ms=500)
message = feedback.current(1499)
assert message is not None and message.text == "测试警告"
assert message.level == FeedbackLevel.WARNING
assert feedback.current(1500) is None
print("✅ ② 玩家短时反馈按生命周期清理")


# ③ Ctrl+P 的真实键盘路由只切换 BUILD 平行吸附，不进入 PLAY。
loop = GameLoop.__new__(GameLoop)
loop.inspect_train = None
loop.feedback = FeedbackCenter()
loop.editor = SimpleNamespace(mode=EditMode.BUILD, parallel_snap_enabled=False)
event = pygame.event.Event(pygame.KEYDOWN, key=pygame.K_p, mod=pygame.KMOD_CTRL)
loop._handle_keydown(event)
assert loop.editor.mode == EditMode.BUILD
assert loop.editor.parallel_snap_enabled is True
print("✅ ③ Ctrl+P 在 BUILD 中只切换平行吸附")


# ④ 自绘的轻量状态栏/短时消息可在 dummy surface 上工作；未来可由 pygame_gui 替换。
pygame.init()
surface = pygame.Surface((800, 240), pygame.SRCALPHA)
font = pygame.font.Font(None, 20)
draw_context_bar(surface, font, action_hints(UIContext.POI))
feedback.post("POI 已创建", 2000, FeedbackLevel.SUCCESS)
draw_feedback(surface, font, feedback.current(2000))
assert surface.get_bounding_rect().width > 0
pygame.quit()
print("✅ ④ 上下文栏与短时反馈可独立渲染")


# ⑤ pygame_gui 工具栏产生稳定 action id，并能随分辨率更新后继续绘制。
pygame.init()
surface = pygame.display.set_mode((900, 300), pygame.RESIZABLE)
gui = GameGUI(surface.get_size())
button = gui._buttons["mode.poi"]
mouse_down = pygame.event.Event(
    pygame.MOUSEBUTTONDOWN,
    button=1,
    pos=button.rect.center,
)
_action_id, mouse_consumed = gui.process_event(mouse_down)
assert mouse_consumed
event = pygame.event.Event(
    pygame_gui.UI_BUTTON_PRESSED,
    ui_element=button,
    ui_object_id="#mode_poi",
)
action_id, consumed = gui.process_event(event)
assert consumed and action_id == "mode.poi"
gui.set_active_mode(EditMode.POI)
assert button.is_selected
gui.set_resolution((1000, 400))
gui.update(1 / 60)
surface.fill((0, 0, 0))
gui.draw(surface)
assert surface.get_bounding_rect().width > 0
pygame.quit()
print("✅ ⑤ pygame_gui 模式工具栏事件、选中态、缩放与绘制通过")
