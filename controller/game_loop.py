from __future__ import annotations

import sys
import os

import pygame

from model.rail_network import RailNetwork
from model.geojson_loader import load_geojson
from model.vec3 import Vec3
from view.camera import Camera
from view.renderer import Renderer
from controller.editor import Editor, EditMode, BuildState

WINDOW_W = 1024
WINDOW_H = 768

# 固定存档文件名：S 键保存到此，启动时若存在则优先加载（方便反复启停调试）。
SAVE_PATH = "manual_track.geojson"

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
        # 优先加载固定存档，不存在则加载传入的默认地图
        if os.path.exists(SAVE_PATH):
            self.network = load_geojson(SAVE_PATH)
            print(f"已加载存档 {SAVE_PATH}（{len(self.network.edges)} 条边）")
        else:
            self.network = load_geojson(geo_path)
        self.editor = Editor(self.network)
        self.running = True

        # 平移状态（受模式影响触发集）
        self._pan_button: int | None = None  # 当前正在按的平移按钮（None 表示未平移）
        self._pan_last_mouse: pygame.Vector2 = pygame.Vector2(0, 0)
        # 用于区分"按下=平移"还是"按下未拖动=点击"（仅 IDLE 模式相关，其它模式无歧义）
        self._pan_button_down_pos: pygame.Vector2 = pygame.Vector2(0, 0)
        self._pan_moved: bool = False

        # 空间索引可视化开关（I 键切换）
        self.debug_show_tiles = False

        # 寻路测试叠加态（F 键切换）：不参与编辑器状态机，独立于 EditMode。
        # 依次点选两个吸附到的节点求最短路径，第三次点击重置。
        self.pathtest_enabled = False
        self.pathtest_start_node: int | None = None
        self.pathtest_goal_node: int | None = None
        self.pathtest_path = None  # model.pathfinding.Path | None

        # Debug 列车（需求 D + 物理层）：算出 Path 后自动启动
        self.debug_train_active = False
        self.debug_train_kinematics = None  # model.kinematics.PathKinematics | None
        self.debug_train_physics = None     # model.train_physics.TrainPhysics | None
        # 列车状态（由 GameLoop 管理，物理层只计算加速度）
        self.debug_train_s = 0.0  # 弧长（米）
        self.debug_train_v = 0.0  # 速度（m/s）
        self.debug_train_a = 0.0  # 加速度（m/s²）

    def run(self) -> None:
        while self.running:
            for event in pygame.event.get():
                self._handle_event(event)

            # 每帧同步键盘修饰键状态到 Editor（force_straight 等）
            self._sync_modifiers()

            # 更新 debug 列车（物理层 + 状态积分）
            if self.debug_train_active and self.debug_train_physics is not None:
                # 方向键控制油门/制动（临时交互，AI 调度前）
                keys = pygame.key.get_pressed()
                if keys[pygame.K_UP]:
                    throttle, brake = 1.0, 0.0
                elif keys[pygame.K_DOWN]:
                    throttle, brake = 0.0, 1.0
                else:
                    throttle, brake = 0.0, 0.0  # 惰行

                dt = self.clock.get_time() / 1000.0

                # 物理层：计算加速度（无状态）
                self.debug_train_a = self.debug_train_physics.compute_acceleration(
                    self.debug_train_v, throttle, brake
                )

                # 状态积分：v 和 s 的欧拉更新
                self.debug_train_v += self.debug_train_a * dt
                self.debug_train_v = max(0.0, min(self.debug_train_v, self.debug_train_physics.v_max))
                self.debug_train_s += self.debug_train_v * dt
                self.debug_train_s = max(0.0, min(self.debug_train_s, self.debug_train_kinematics.total_length))

                # 到达终点停止
                if self.debug_train_s >= self.debug_train_kinematics.total_length:
                    self.debug_train_v = 0.0
                    self.debug_train_a = 0.0
                    self.debug_train_active = False

            mouse_world = self._mouse_world_pos()
            self.editor.update_hover(mouse_world)

            self.renderer.clear()
            self.renderer.draw_grid()
            # 空间索引可视化（I 键切换，在 draw_network 之前绘制，避免遮挡轨道）
            if self.debug_show_tiles:
                from view.renderer import draw_spatial_index_debug
                draw_spatial_index_debug(
                    self.renderer.surface,
                    self.camera,
                    self.network,
                    cursor_world_pos=mouse_world,
                    query_radius=30.0,  # 可调整，匹配吸附阈值
                )
            self.renderer.draw_network(self.network, self.editor)
            self.renderer.draw_overlay(self.network, self.editor, mouse_world)
            # 寻路测试可视化（F 键叠加态）
            if self.pathtest_enabled:
                from view.renderer import draw_pathfinding_debug, draw_debug_train
                draw_pathfinding_debug(
                    self.renderer.surface,
                    self.camera,
                    self.network,
                    self.renderer._font,
                    self.pathtest_start_node,
                    self.pathtest_goal_node,
                    self.pathtest_path,
                )
                # Debug 列车（需求 D）
                if self.debug_train_active and self.debug_train_physics:
                    draw_debug_train(
                        self.renderer.surface,
                        self.camera,
                        self.debug_train_kinematics,
                        self.debug_train_s,
                    )
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
            # S 键保存当前路网到固定存档（启动时会优先加载它）
            from model.geojson_writer import write_geojson
            write_geojson(self.editor.network, SAVE_PATH)
            print(f"已保存 {len(self.editor.network.edges)} 条边到 {SAVE_PATH}")
        elif event.key == pygame.K_i:
            # I 键切换空间索引可视化（debug 用）
            self.debug_show_tiles = not self.debug_show_tiles
            status = "开启" if self.debug_show_tiles else "关闭"
            print(f"空间索引可视化: {status}")
        elif event.key == pygame.K_f:
            # F 键切换寻路测试叠加态（debug 用）
            self.pathtest_enabled = not self.pathtest_enabled
            self._reset_pathtest()
            status = "开启" if self.pathtest_enabled else "关闭"
            print(f"寻路测试: {status}（点选起点、终点两个节点）")

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

        # 寻路测试态：左键专用于选点，优先于平移拦截（否则 IDLE 下左键会被平移分支吃掉）
        if event.button == 1 and self.pathtest_enabled:
            self._pathtest_click(self._mouse_world_pos())
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

    # ===== 寻路测试（F 键叠加态，debug） =====

    def _reset_pathtest(self) -> None:
        self.pathtest_start_node = None
        self.pathtest_goal_node = None
        self.pathtest_path = None
        # 重置时停止 debug 列车
        self.debug_train_active = False
        self.debug_train_kinematics = None
        self.debug_train_physics = None
        self.debug_train_s = 0.0
        self.debug_train_v = 0.0
        self.debug_train_a = 0.0

    def _snap_node_at(self, world_pos: Vec3) -> int | None:
        """复用编辑器吸附系统，返回点击命中的节点 ID（未命中节点返回 None）。"""
        snap = self.editor._snap(world_pos)
        return snap.snapped_node_id

    def _pathtest_click(self, world_pos: Vec3) -> None:
        from model.pathfinding import find_path_between_nodes

        # 已有完整结果 → 第三次点击重置，重新开始
        if self.pathtest_start_node is not None and self.pathtest_goal_node is not None:
            self._reset_pathtest()

        node_id = self._snap_node_at(world_pos)
        if node_id is None:
            print("寻路测试：未点中节点（请点选轨道节点）")
            return

        if self.pathtest_start_node is None:
            self.pathtest_start_node = node_id
            print(f"寻路测试：起点 = 节点 {node_id}")
            return

        # 选终点并求路径
        self.pathtest_goal_node = node_id
        path = find_path_between_nodes(
            self.network, self.pathtest_start_node, self.pathtest_goal_node
        )
        self.pathtest_path = path
        if path is None:
            print(
                f"寻路测试：节点 {self.pathtest_start_node} → {node_id} 不可达"
            )
            # 不可达，停止 debug 列车
            self.debug_train_active = False
            self.debug_train_kinematics = None
            self.debug_train_physics = None
        else:
            seq = " → ".join(f"e{eid}({'+' if d > 0 else '-'})" for eid, d in path.edges)
            print(
                f"寻路测试：节点 {self.pathtest_start_node} → {node_id} "
                f"共 {len(path.edges)} 段，总长 {path.total_cost:.2f}\n  {seq}"
            )
            # 自动启动 debug 列车（物理层 + 状态初始化）
            from model.kinematics import PathKinematics
            from model.train_physics import SimplePhysics

            self.debug_train_kinematics = PathKinematics(self.network, path)
            self.debug_train_physics = SimplePhysics()  # 无状态，只需创建实例
            self.debug_train_s = 0.0
            self.debug_train_v = 0.0
            self.debug_train_a = 0.0
            self.debug_train_active = True
            print(f"  → Debug 列车已启动，路径总长 {self.debug_train_kinematics.total_length:.2f} m")
            print(f"  → 控制：方向键 ↑ 加速，↓ 制动")


def run_game(geo_path: str) -> None:
    GameLoop(geo_path).run()
