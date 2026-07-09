from __future__ import annotations

import sys

import pygame

from model.rail_network import RailNetwork
from model.geojson_loader import load_geojson
from model.vec3 import Vec3
from view.camera import Camera
from view.renderer import Renderer
from controller.editor import Editor, EditMode, BuildState

WINDOW_W = 1024
WINDOW_H = 768

# 平移触发集：IDLE 模式下左/右/中键都可平移；其它模式仅中键
PAN_BUTTONS_IDLE = {1, 2, 3}
PAN_BUTTONS_OTHER = {2}


class GameLoop:
    def __init__(self, geo_path: str) -> None:
        pygame.init()
        self.surface = pygame.display.set_mode((WINDOW_W, WINDOW_H), pygame.RESIZABLE)
        pygame.display.set_caption("Railway")
        self.clock = pygame.time.Clock()
        self.camera = Camera()
        self.renderer = Renderer(self.surface, self.camera)
        self.network: RailNetwork = load_geojson(geo_path)
        self.editor = Editor(self.network)
        self.running = True

        # 平移状态（受模式影响触发集）
        self._pan_button: int | None = None  # 当前正在按的平移按钮（None 表示未平移）
        self._pan_last_mouse: pygame.Vector2 = pygame.Vector2(0, 0)
        # 用于区分"按下=平移"还是"按下未拖动=点击"（仅 IDLE 模式相关，其它模式无歧义）
        self._pan_button_down_pos: pygame.Vector2 = pygame.Vector2(0, 0)
        self._pan_moved: bool = False

    def run(self) -> None:
        while self.running:
            for event in pygame.event.get():
                self._handle_event(event)

            # 每帧同步键盘修饰键状态到 Editor（force_straight 等）
            self._sync_modifiers()

            mouse_world = self._mouse_world_pos()
            self.editor.update_hover(mouse_world)

            self.renderer.clear()
            self.renderer.draw_grid()
            self.renderer.draw_network(self.network, self.editor)
            self.renderer.draw_overlay(self.network, self.editor, mouse_world)
            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()
        sys.exit()

    # ===== 输入事件分发 =====

    def _handle_event(self, event: pygame.event.Event) -> None:
        if event.type == pygame.QUIT:
            self.running = False
            return

        if event.type == pygame.KEYDOWN:
            self._handle_keydown(event)
            return

        if event.type == pygame.MOUSEBUTTONDOWN:
            self._handle_mouse_down(event)
            return

        if event.type == pygame.MOUSEBUTTONUP:
            self._handle_mouse_up(event)
            return

        if event.type == pygame.MOUSEMOTION:
            self._handle_mouse_motion(event)
            return

    def _handle_keydown(self, event: pygame.event.Event) -> None:
        if event.key == pygame.K_ESCAPE:
            self.editor.handle_cancel()
        elif event.key == pygame.K_b:
            self.editor.set_mode(EditMode.BUILD)
        elif event.key == pygame.K_d:
            self.editor.set_mode(EditMode.DELETE)
        elif event.key == pygame.K_q:
            self.running = False
        elif event.key == pygame.K_g:
            # G 键切换格点吸附（见 docs/snapping.md）
            # 与 L/A 互斥:开启格点时关闭长度/角度
            self.editor.grid_snap_enabled = not self.editor.grid_snap_enabled
            if self.editor.grid_snap_enabled:
                self.editor.length_snap_enabled = False
                self.editor.angle_snap_enabled = False
        elif event.key == pygame.K_l:
            # L 键切换长度吸附(所有直线建造)；与 A/G 互斥
            self.editor.length_snap_enabled = not self.editor.length_snap_enabled
            if self.editor.length_snap_enabled:
                self.editor.angle_snap_enabled = False
                self.editor.grid_snap_enabled = False
        elif event.key == pygame.K_a:
            # A 键切换角度吸附(仅单弧建造)；与 L/G 互斥
            self.editor.angle_snap_enabled = not self.editor.angle_snap_enabled
            if self.editor.angle_snap_enabled:
                self.editor.length_snap_enabled = False
                self.editor.grid_snap_enabled = False
        elif event.key == pygame.K_p:
            # P 键切换平行吸附(Simple/Complex Case)；独立开关
            self.editor.parallel_snap_enabled = not self.editor.parallel_snap_enabled
        elif event.key == pygame.K_s:
            # S 键保存当前路网到 manual_track.geojson（临时持久化功能）
            from model.geojson_writer import write_geojson
            write_geojson(self.editor.network, "manual_track.geojson")
            print(f"已保存 {len(self.editor.network.edges)} 条边到 manual_track.geojson")

    def _handle_mouse_down(self, event: pygame.event.Event) -> None:
        # 滚轮先处理（不参与平移逻辑）
        if event.button in (4, 5):
            self.camera.handle_event(event)
            return

        # 右键在 BUILD_ACTIVE 中等同于 Esc（取消当前建造），不参与平移。
        # 见 docs/editor.md §10.4。其它情况（IDLE 等）右键继续走平移分支。
        if (
            event.button == 3
            and self.editor.mode == EditMode.BUILD
            and self.editor.build_state == BuildState.ACTIVE
        ):
            self.editor.handle_cancel()
            return

        pan_buttons = self._pan_buttons_for_mode()
        if event.button in pan_buttons:
            # 进入"待定平移"状态：先记录按下，移动后才算真正平移
            self._pan_button = event.button
            self._pan_last_mouse = pygame.Vector2(event.pos)
            self._pan_button_down_pos = pygame.Vector2(event.pos)
            self._pan_moved = False
            return

        # 非平移按钮：交给编辑器（左键 in BUILD/DELETE）
        if event.button == 1:
            world_pos = self._mouse_world_pos()
            self.editor.handle_click(world_pos)

    def _handle_mouse_up(self, event: pygame.event.Event) -> None:
        if self._pan_button is not None and event.button == self._pan_button:
            self._pan_button = None
            self._pan_moved = False

    def _handle_mouse_motion(self, event: pygame.event.Event) -> None:
        if self._pan_button is None:
            return
        # 跟踪是否已发生拖动（用于"按下未拖动"的潜在 click 判定）
        current = pygame.Vector2(event.pos)
        if (current - self._pan_button_down_pos).length() > 2:
            self._pan_moved = True
        delta = current - self._pan_last_mouse
        self.camera.pan_x += delta.x / self.camera.scale
        self.camera.pan_y -= delta.y / self.camera.scale
        self._pan_last_mouse = current

    # ===== 辅助 =====

    def _pan_buttons_for_mode(self) -> set[int]:
        """根据当前编辑模式决定哪些鼠标按钮触发平移。"""
        if self.editor.mode == EditMode.IDLE:
            return PAN_BUTTONS_IDLE
        return PAN_BUTTONS_OTHER

    def _sync_modifiers(self) -> None:
        """每帧把键盘修饰状态同步到 Editor。"""
        keys = pygame.key.get_pressed()
        # 强制直线：按住 LSHIFT 时，Case 2 改走 Case 1
        self.editor.force_straight = bool(keys[pygame.K_LSHIFT])
        # 强制 Case 2T（§10.5）：按住 LALT 时，M2 路径吸附直边 → 单切线弧
        # LSHIFT 优先（force_straight 的判定在 _compute_plan 中早于 case2t）
        self.editor.force_case2t = bool(keys[pygame.K_LALT])
        # 同步缩放：吸附阈值以屏幕像素为基准，需按当前 scale 换算世界阈值
        self.editor.pixel_scale = self.camera.scale

    def _mouse_world_pos(self) -> Vec3:
        mx, my = pygame.mouse.get_pos()
        wx, wy = self.camera.screen_to_world(
            mx, my,
            self.surface.get_width(),
            self.surface.get_height(),
        )
        return Vec3(wx, wy, 0.0)


def run_game(geo_path: str) -> None:
    GameLoop(geo_path).run()
