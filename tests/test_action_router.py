"""A0.3 modal keyboard and widget action-id convergence."""
import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import pygame

from controller.action_router import action_for_key


pygame.init()


def key(value):
    return pygame.event.Event(pygame.KEYDOWN, {"key": value, "mod": 0})


assert action_for_key(key(pygame.K_l), "consist_builder") == "consist.add.powered_control"
assert action_for_key(key(pygame.K_BACKSPACE), "consist_builder") == "consist.delete"
assert action_for_key(key(pygame.K_RETURN), "consist_builder") == "consist.complete"
assert action_for_key(key(pygame.K_p), "train_info") == "schedule.open"
assert action_for_key(key(pygame.K_x), "schedule") == "schedule.delete.item"
assert action_for_key(key(pygame.K_o), "schedule") == "schedule.close"
assert action_for_key(key(pygame.K_p), "schedule") == "schedule.edit"
assert action_for_key(key(pygame.K_x), "plan_edit") == "plan.delete.item"
assert action_for_key(pygame.event.Event(pygame.KEYDOWN, {"key": pygame.K_x, "mod": pygame.KMOD_CTRL}), "plan_edit") == "plan.clear"
assert action_for_key(key(pygame.K_ESCAPE), "plan_edit") == "plan.finish"
assert action_for_key(key(pygame.K_RETURN), "poi") == "poi.confirm"
assert action_for_key(key(pygame.K_t), "poi") == "station.toggle"
assert action_for_key(key(pygame.K_x), "poi") == "poi.delete.hovered"
assert action_for_key(key(pygame.K_b), "workspace") == "mode.build"
assert action_for_key(key(pygame.K_p), "workspace") == "mode.play"
assert action_for_key(key(pygame.K_p), "workspace_play") is None
assert action_for_key(key(pygame.K_f), "workspace_play") == "mode.play"
assert action_for_key(key(pygame.K_b), "schedule") is None
assert action_for_key(pygame.event.Event(pygame.MOUSEBUTTONDOWN), "schedule") is None

print("✅ 输入 action 路由：编组/列车信息/计划选单键盘与 widget action id 对齐")
