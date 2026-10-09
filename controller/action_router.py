"""Shared keyboard-to-action routing for screen-space modal contexts.

Widgets already emit stable action ids through ``GameGUI``.  This module gives
keyboard shortcuts the same ids for the first A0.3 consumers, so modal input
does not duplicate business operations in ``GameLoop``.
"""
from __future__ import annotations

import pygame


def action_for_key(event: pygame.event.Event, context: str) -> str | None:
    """Return one stable action id for a key in ``context`` or ``None``.

    Contexts are deliberately string-based to keep this boundary independent of
    the larger workspace state machine and easy to exercise without a window.
    """
    if event.type != pygame.KEYDOWN:
        return None
    key = event.key
    if key == pygame.K_F3:
        return "debug.route.toggle"
    if context == "consist_builder":
        return {
            pygame.K_l: "consist.add.powered_control",
            pygame.K_c: "consist.add.coach",
            pygame.K_d: "consist.add.double",
            pygame.K_BACKSPACE: "consist.delete",
            pygame.K_r: "consist.reverse",
            pygame.K_LEFT: "consist.move.left",
            pygame.K_RIGHT: "consist.move.right",
            pygame.K_ESCAPE: "consist.cancel",
            pygame.K_RETURN: "consist.complete",
            pygame.K_KP_ENTER: "consist.complete",
            pygame.K_LEFT: "consist.move.left",
            pygame.K_RIGHT: "consist.move.right",
        }.get(key)
    if context == "train_info":
        return {
            pygame.K_i: "overlay.close",
            pygame.K_p: "schedule.open",
            pygame.K_ESCAPE: "overlay.close",
        }.get(key)
    if context == "schedule":
        if key == pygame.K_x and getattr(event, "mod", 0) & pygame.KMOD_CTRL:
            return "schedule.delete.plan"
        return {
            pygame.K_p: "schedule.edit",
            pygame.K_LEFTBRACKET: "schedule.move.up",
            pygame.K_RIGHTBRACKET: "schedule.move.down",
            pygame.K_x: "schedule.delete.item",
            pygame.K_l: "schedule.toggle.repeat",
            pygame.K_ESCAPE: "schedule.close",
            pygame.K_o: "schedule.close",
        }.get(key)
    if context == "plan_edit":
        if key in (pygame.K_RETURN, pygame.K_KP_ENTER):
            return "plan.confirm"
        if key == pygame.K_BACKSPACE:
            return "plan.undo"
        if key == pygame.K_x:
            return "plan.clear" if getattr(event, "mod", 0) & pygame.KMOD_CTRL else "plan.delete.item"
        if key == pygame.K_w:
            return "plan.add.wait_couple"
        if key == pygame.K_k:
            return "plan.add.couple"
        if key == pygame.K_r:
            return "plan.add.reverse"
        if key in (pygame.K_p, pygame.K_ESCAPE):
            return "plan.finish"
        return None
    if context == "poi":
        if key in (pygame.K_RETURN, pygame.K_KP_ENTER):
            return "poi.confirm"
        return {
            pygame.K_BACKSPACE: "poi.undo",
            pygame.K_x: "poi.delete.hovered",
            pygame.K_t: "station.toggle",
            pygame.K_v: "poi.type.depot",
            pygame.K_n: "poi.name.start",
        }.get(key)
    if context in ("workspace", "workspace_play"):
        if getattr(event, "mod", 0) & pygame.KMOD_CTRL:
            return None
        common = {
            pygame.K_b: "mode.build",
            pygame.K_d: "mode.delete",
            pygame.K_h: "mode.signal",
            pygame.K_j: "mode.poi",
            pygame.K_f: "mode.play",
        }
        if context == "workspace":
            common[pygame.K_p] = "mode.play"
        return common.get(key)
    return None
