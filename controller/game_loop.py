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

SAVE_PATH = "manual_track.geojson"

PAN_BUTTONS_IDLE = {1, 2, 3}
PAN_BUTTONS_OTHER = {2}

# 控制台回显缓冲（最近 N 条，显示在游戏右下角）
_console_messages: list[str] = []


class _TeeLogger:
    """把 stdout 写入同时追加到消息缓冲，保留最近 max 条。"""
    MAX = 60

    def __init__(self, lines: list[str], orig) -> None:
        self._lines = lines
        self._orig = orig
        self._buf = ""

    def write(self, s: str) -> None:
        self._orig.write(s)
        self._buf += s
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            if line.strip():
                self._lines.append(line)
                if len(self._lines) > self.MAX:
                    self._lines.pop(0)

    def flush(self) -> None:
        self._orig.flush()


class GameLoop:
    def __init__(self, geo_path: str) -> None:
        sys.stdout = _TeeLogger(_console_messages, sys.stdout)
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

        # PLAY 模式状态
        self.train_path = None                     # 当前可视化路径（焦点列车）
        self.train_path_virtual_points = []        # 寻路虚拟节点位置（调试用）

        # 放置列车的两步流程
        self.train_placement_node_id = None        # 第一步：选中的放置节点
        self.train_placement_consist = None        # 待放置的车组

        # 列车列表 + 焦点（E 阶段）
        self.trains: list["TrainEntity"] = []
        self.active_train: "TrainEntity | None" = None
        self.train_v_target: float = 0.0           # 焦点列车的巡航速度（m/s）
        self.inspect_train: "TrainEntity | None" = None  # 编组面板目标（I 键切换）
        # 车钩悬停状态：(列车, coupler_idx) 或 None
        self._hovered_coupler: tuple["TrainEntity", int] | None = None

    def run(self) -> None:
        while self.running:
            for event in pygame.event.get():
                self._handle_event(event)

            # 每帧同步键盘修饰键状态到 Editor（force_straight 等）
            self._sync_modifiers()

            # 更新列车（物理层 + 状态积分）
            if self.active_train is not None and self.active_train.is_moving():
                dt = self.clock.get_time() / 1000.0

                # ↑/↓ 调整目标速度
                keys = pygame.key.get_pressed()
                V_MAX = 30.0
                V_STEP = 5.0
                if keys[pygame.K_UP]:
                    self.train_v_target = min(V_MAX, self.train_v_target + V_STEP * dt)
                elif keys[pygame.K_DOWN]:
                    self.train_v_target = max(0.0, self.train_v_target - V_STEP * dt)
                self.active_train.v_target = self.train_v_target

                self.active_train.update(dt, self.train_v_target)

                # 相机跟随列车
                if self.camera.follow_enabled and self.active_train is not None:
                    wagon_poses = self.active_train.kinematics.get_all_wagon_poses(self.active_train.state.s)
                    if wagon_poses:
                        lead_pose = wagon_poses[0]
                        self.camera.set_center_smooth(lead_pose.position.x, lead_pose.position.y)

            # 非焦点列车各自按其上次速度目标继续运行
            dt = self.clock.get_time() / 1000.0
            for t in self.trains:
                if t is not self.active_train and t.is_moving():
                    t.update(dt, t.v_target)

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
            # PLAY 模式可视化：路径预览按需从 occupancy+route 现拼，不缓存
            if (self.editor.mode == EditMode.PLAY and self.active_train is not None
                    and self.active_train.state.occupancy.route):
                from view.renderer import draw_pathfinding_debug
                from model.pathfinding import Path as _Path
                occ = self.active_train.state.occupancy
                preview_path = _Path(
                    edges=list(occ.occupied) + list(occ.route),
                    total_cost=sum(self.network.edges[e[0]].length
                                  for e in occ.occupied + occ.route),
                )
                draw_pathfinding_debug(
                    self.renderer.surface, self.camera, self.network,
                    self.renderer._font, None, None, preview_path,
                )
                # 绘制虚拟节点（黄色圆圈，调试用）
                for vpos in self.train_path_virtual_points:
                    sx, sy = self.camera.world_to_screen(
                        vpos.x, vpos.y,
                        self.renderer.surface.get_width(),
                        self.renderer.surface.get_height()
                    )
                    pygame.draw.circle(
                        self.renderer.surface,
                        (255, 255, 0),  # 黄色
                        (int(sx), int(sy)),
                        6,  # 半径
                        2,  # 线宽（空心圆）
                    )
            # 放置虚影：显示所有可能方向的列车预览（半透明灰色）
            if self.train_placement_node_id is not None and self.train_placement_consist is not None:
                from view.renderer import draw_debug_train
                from model.rigid_kinematics import RigidWagonKinematics
                from model.pathfinding import Path, _directed_from

                placement_node = self.network.nodes[self.train_placement_node_id]
                for eid in placement_node.incident_edge_ids:
                    # 为每条关联边创建临时路径和运动学
                    park_directed = _directed_from(self.network, eid, self.train_placement_node_id)
                    temp_path = Path(edges=[park_directed], total_cost=self.network.edges[eid].length)

                    try:
                        temp_kin = RigidWagonKinematics(
                            self.network, temp_path, self.train_placement_consist
                        )
                        # 绘制半透明虚影（灰色，occupied=False 会用橙色，这里需要自定义颜色）
                        # 暂时用橙色表示虚影，后续可以添加颜色参数
                        draw_debug_train(
                            self.renderer.surface,
                            self.camera,
                            temp_kin,
                            0.0,
                            occupied=False,  # 橙色虚影
                        )
                    except Exception:
                        # 如果路径太短无法容纳列车，跳过
                        pass

            # 非焦点列车（蓝色线框）
            from view.renderer import draw_debug_train as _draw_train
            COLOR_INACTIVE = (80, 140, 220)
            for t in self.trains:
                if t is not self.active_train:
                    _draw_train(self.renderer.surface, self.camera,
                                t.kinematics, t.state.s, color=COLOR_INACTIVE)

            if self.active_train is not None:
                from view.renderer import draw_debug_train, draw_train_hud
                # 焦点列车（红色）
                draw_debug_train(
                    self.renderer.surface,
                    self.camera,
                    self.active_train.kinematics,
                    self.active_train.state.s,
                    occupied=True,
                )
                # 制动区判断（HUD 显示用）
                _v = self.active_train.state.v
                _a_b = abs(self.active_train.physics.compute_acceleration(
                    _v, 0.0, 1.0, self.active_train.state.consist))
                _in_braking = False
                if self.active_train.controller is not None:
                    _d_stop = (_v ** 2) / max(1e-6, 2 * _a_b) \
                              * self.active_train.controller.safety_margin
                    _in_braking = (self.active_train.kinematics.total_length
                                   - self.active_train.state.s) <= _d_stop
                draw_train_hud(
                    self.renderer.surface,
                    self.renderer._font,
                    self.active_train.state.s,
                    self.active_train.state.v,
                    self.active_train.last_a,
                    self.active_train.kinematics.total_length,
                    self.train_v_target,
                    braking=_in_braking,
                )
            # 右下角控制台回显
            from view.renderer import (draw_console_log, draw_train_tooltip,
                                       draw_consist_panel, draw_coupler_highlight)
            draw_console_log(self.renderer.surface, self.renderer._font, _console_messages)
            # PLAY 模式：车钩悬停检测 + tooltip
            if self.editor.mode == EditMode.PLAY:
                self._hovered_coupler = self._hit_test_coupler(mouse_world)
                if self._hovered_coupler is not None:
                    _, cp_idx = self._hovered_coupler
                    cp_train = self._hovered_coupler[0]
                    couplers = cp_train.kinematics.get_coupler_positions(cp_train.state.s)
                    if cp_idx < len(couplers):
                        w_s = self.renderer.surface.get_width()
                        h_s = self.renderer.surface.get_height()
                        draw_coupler_highlight(
                            self.renderer.surface, self.camera,
                            couplers[cp_idx], w_s, h_s,
                        )
                else:
                    hovered = self._hit_test_train(mouse_world)
                    if hovered is not None:
                        idx = self.trains.index(hovered) + 1
                        mx, my = pygame.mouse.get_pos()
                        draw_train_tooltip(
                            self.renderer.surface, self.renderer._font,
                            (mx, my), idx, hovered.state.v,
                        )
            # 编组面板（I 键触发）
            if self.inspect_train is not None and self.inspect_train in self.trains:
                idx = self.trains.index(self.inspect_train) + 1
                draw_consist_panel(
                    self.renderer.surface, self.renderer._font,
                    idx, self.inspect_train,
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
            if self.editor.mode == EditMode.PLAY:
                if self.active_train is not None or self.train_placement_node_id is not None:
                    # 先取消焦点/放置流程
                    self.active_train = None
                    self.train_placement_node_id = None
                    self.train_placement_consist = None
                    self.train_path = None
                else:
                    # 再按一次退出 PLAY
                    self.editor.set_mode(EditMode.IDLE)
            else:
                self.editor.handle_cancel()
        elif event.key == pygame.K_p or event.key == pygame.K_f:
            # P（正式）/ F（兼容旧习惯）切换 PLAY 模式
            if self.editor.mode == EditMode.PLAY:
                self.editor.set_mode(EditMode.IDLE)
                self.train_path = None
                print("PLAY 模式：关闭")
            else:
                self.editor.set_mode(EditMode.PLAY)
                print("PLAY 模式：开启（左键选中列车/放置，右键下达指令）")
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
            if self.editor.mode == EditMode.PLAY:
                # PLAY 模式：I 键切换焦点列车编组面板
                if self.active_train is not None:
                    self.inspect_train = None if self.inspect_train is self.active_train \
                                               else self.active_train
            else:
                # 其他模式：I 键切换空间索引可视化（debug 用）
                self.debug_show_tiles = not self.debug_show_tiles
                status = "开启" if self.debug_show_tiles else "关闭"
                print(f"空间索引可视化: {status}")
        elif event.key == pygame.K_c:
            # C 键切换相机跟随（A1）
            self.camera.follow_enabled = not self.camera.follow_enabled
            status = "开启" if self.camera.follow_enabled else "关闭"
            print(f"相机跟随: {status}")
        elif event.key == pygame.K_k:
            # K 键：解挂（临时调试，固定在第一节后分割）
            # ponytail: 临时交互，F 阶段替换为完整 UI
            self._debug_decouple()
        elif event.key == pygame.K_SPACE:
            # 空格键：列车紧急停止（emergency_stop）
            if self.active_train is not None:
                self.active_train.emergency_stop()
                self.train_v_target = 0.0
                print("列车紧急停止")
    def _handle_mouse_down(self, event: pygame.event.Event) -> None:
        # 滚轮先处理（不参与平移逻辑）
        if event.button in (4, 5):
            self.camera.handle_event(event)
            return

        # 右键在 BUILD_ACTIVE 中等同于 Esc（取消当前建造），不参与平移。
        if (
            event.button == 3
            and self.editor.mode == EditMode.BUILD
            and self.editor.build_state == BuildState.ACTIVE
        ):
            self.editor.handle_cancel()
            return

        # PLAY 模式：左键=选择/放置，右键=下达寻路指令；两者都不触发平移
        if self.editor.mode == EditMode.PLAY:
            if event.button == 1:
                self._play_left_click(self._mouse_world_pos())
                return
            if event.button == 3:
                self._play_right_click(self._mouse_world_pos())
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

    # ===== PLAY 模式（P 键叠加态）=====

    def _debug_decouple(self) -> None:
        """K 键：悬停车钩 → 解挂；未悬停 → couple 条件检查。"""
        if self.editor.mode != EditMode.PLAY:
            return

        # 悬停在车钩上：执行解挂
        if self._hovered_coupler is not None:
            train, coupler_idx = self._hovered_coupler
            if not train.is_parked():
                print("PLAY: 请先停车再解挂（空格键）")
                return
            try:
                front, rear = train.decouple_at(coupler_idx)
            except Exception as e:
                print(f"PLAY: 解挂失败 — {e}")
                return
            idx = self.trains.index(train)
            self.trains[idx:idx+1] = [front, rear]
            self.active_train = front
            self.train_path = None
            self.inspect_train = None
            self._hovered_coupler = None
            fn, rn = len(front.state.consist.wagons), len(rear.state.consist.wagons)
            print(f"PLAY: Decouple #{idx+1} | {fn+rn}→{fn}+{rn}")
            return

        # 未悬停：尝试 couple
        self._try_couple()

    def _hit_test_train(self, world_pos: Vec3) -> "TrainEntity | None":
        """返回点击命中的列车，未命中返回 None。判定：任意 wagon 中心 < 5m。"""
        HIT_RADIUS = 5.0
        for train in self.trains:
            poses = train.kinematics.get_all_wagon_poses(train.state.s)
            for pose in poses:
                if (pose.position - world_pos).length() < HIT_RADIUS:
                    return train
        return None

    def _hit_test_coupler(self, world_pos: Vec3) -> tuple["TrainEntity", int] | None:
        """返回鼠标命中的 (列车, coupler_idx)，未命中返回 None。命中半径 1.5m。"""
        HIT_RADIUS = 1.5
        for train in self.trains:
            couplers = train.kinematics.get_coupler_positions(train.state.s)
            for i, cp in enumerate(couplers):
                if (cp - world_pos).length() < HIT_RADIUS:
                    return train, i
        return None

    def _try_couple(self) -> None:
        """检查焦点列车端头车钩是否与其他列车端头车钩足够近，满足则连挂。

        条件：两车都停车，端头车钩距离 < 1m，朝向一致（heading 点积 > 0.9）。
        """
        COUPLE_DIST = 1.0
        HEADING_DOT  = 0.9

        front = self.active_train
        if front is None:
            print("PLAY: 未选中列车")
            return
        if not front.is_parked():
            print("PLAY: 请先停车再连挂（空格键）")
            return

        front_head_data, front_tail_data = \
            front.kinematics.get_end_coupler_data(front.state.s)

        for other in self.trains:
            if other is front or not other.is_parked():
                continue
            other_head_data, other_tail_data = \
                other.kinematics.get_end_coupler_data(other.state.s)

            # 检查所有端头组合：自身车尾 ↔ 对方车头（最常见），或自身车头 ↔ 对方车尾
            for (a_pos, _ae, _at), (b_pos, _be, _bt), a_is_tail in (
                (front_tail_data, other_head_data, True),
                (front_head_data, other_tail_data, False),
            ):
                if (a_pos - b_pos).length() > COUPLE_DIST:
                    continue
                # 朝向检查：两车 heading 点积
                front_poses = front.kinematics.get_all_wagon_poses(front.state.s)
                other_poses = other.kinematics.get_all_wagon_poses(other.state.s)
                if not front_poses or not other_poses:
                    continue
                fh = front_poses[0].heading
                oh = other_poses[0].heading
                if fh.dot(oh) < HEADING_DOT:
                    continue

                # 满足条件：连挂
                if a_is_tail:
                    new_train = front.couple_with(other)
                    label = f"#{self.trains.index(front)+1}+#{self.trains.index(other)+1}"
                else:
                    new_train = other.couple_with(front)
                    label = f"#{self.trains.index(other)+1}+#{self.trains.index(front)+1}"

                self.trains.remove(front)
                self.trains.remove(other)
                self.trains.append(new_train)
                self.active_train = new_train
                self.train_path = None
                self.inspect_train = None
                n = len(new_train.state.consist.wagons)
                print(f"PLAY: Couple {label} → {n} 节")
                return

        print("PLAY: 未找到可连挂的列车（需停车且端头车钩 < 1m）")

    def _play_left_click(self, world_pos: Vec3) -> None:
        """左键：切换焦点列车 / 放置新列车（两步流程）。"""
        from model.pathfinding import Path, _directed_from
        from model.train_physics import RealisticElectric
        from model.wagon import create_simple_wagon, Consist
        from model.train_entity import TrainEntity, TrainState

        # 优先：命中已有列车 → 切换焦点
        hit = self._hit_test_train(world_pos)
        if hit is not None:
            if self.active_train is not hit:
                self.active_train = hit
                self.train_v_target = hit.v_target
                self.train_path = None
                idx = self.trains.index(hit) + 1
                print(f"PLAY: 选中列车 #{idx}")
            return

        # 放置流程第一步：选节点
        if self.train_placement_node_id is None:
            node_id = self._snap_node_at(world_pos)
            if node_id is None:
                print("PLAY: 左键点选节点放置新列车，右键下达寻路指令")
                return
            node = self.network.nodes.get(node_id)
            if not node or not node.incident_edge_ids:
                print(f"PLAY: 节点 {node_id} 无关联边，无法放置")
                return
            locomotive = create_simple_wagon(length=20.0, mass=50.0, P_rated=3000.0)
            coach1     = create_simple_wagon(length=18.0, mass=45.0, P_rated=None)
            coach2     = create_simple_wagon(length=22.0, mass=60.0, P_rated=None)
            self.train_placement_consist = Consist(wagons=[locomotive, coach1, coach2])
            self.train_placement_node_id = node_id
            print(f"PLAY: 节点 {node_id} 已选，左键点相邻节点指定朝向")
            return

        # 放置流程第二步：选朝向
        target_node_id = self._snap_node_at(world_pos)
        if target_node_id is None:
            print("PLAY: 请点选相邻节点指定朝向")
            return
        placement_node = self.network.nodes[self.train_placement_node_id]
        edge_id = None
        for eid in placement_node.incident_edge_ids:
            edge = self.network.edges[eid]
            if edge.node_a_id == target_node_id or edge.node_b_id == target_node_id:
                edge_id = eid
                break
        if edge_id is None:
            print(f"PLAY: 节点 {target_node_id} 与放置节点不相邻")
            return
        from model.occupancy import OccupancyState
        park_directed = _directed_from(self.network, edge_id, self.train_placement_node_id)
        park_occ = OccupancyState(occupied=[park_directed], occupied_offset=0.0, s=0.0, route=[])
        state = TrainState(occupancy=park_occ, remaining_to_goal=0.0, v=0.0,
                           consist=self.train_placement_consist)
        new_train = TrainEntity(state, self.network, RealisticElectric())
        self.trains.append(new_train)
        self.active_train = new_train
        self.train_v_target = 0.0
        self.camera.follow_enabled = True
        self.train_placement_node_id = None
        self.train_placement_consist = None
        idx = self.trains.index(new_train) + 1
        print(f"PLAY: 列车 #{idx} 已放置，右键点击目标位置出发")

    def _play_right_click(self, world_pos: Vec3) -> None:
        """右键：对焦点列车下达寻路指令。"""
        if self.active_train is None:
            print("PLAY: 未选中列车，左键先选中或放置")
            return
        self._issue_path_order(world_pos)

    def _find_path_any_goal_direction(
        self, start_edge_id, start_t, start_direction, goal_edge_id, goal_t,
        *, allow_reversal, debug, consist_length,
    ):
        """玩家点击目标点时不指定到达方向，分别尝试 +1/-1，取代价更低者。

        Step 2：find_path_from_point 现在要求显式 goal_direction（消除到达
        方向歧义），但 PLAY 模式下玩家右键点选轨道并不表达"以哪个方向进站"
        的意图，所以在这一层统一枚举两个方向。

        consist_length 必须传真实列车长度（Step 3 阶段 B）：决定折返在
        哪些 endpoint 可行——simple_segment 长度不足的死端会被寻路层直接
        排除，避免"折返后车尾越过 turnout"或"死端间无限振荡"的问题。

        返回 (path, start_offset, end_offset, goal_direction) 或 None——
        goal_direction 需要一并带出，供 assign_route 记录 state.goal
        （中途折返重新寻路时要用同一个到达方向，不能再重新枚举一次）。
        """
        from model.pathfinding import find_path_from_point
        best = None
        best_gd = None
        for gd in (1, -1):
            result = find_path_from_point(
                self.network, start_edge_id, start_t, start_direction,
                goal_edge_id, goal_t, gd,
                allow_reversal=allow_reversal, consist_length=consist_length, debug=debug,
            )
            if result is not None and (best is None or result[0].total_cost < best[0].total_cost):
                best = result
                best_gd = gd
        if best is None:
            return None
        return (*best, best_gd)

    def _apply_route_result(
        self, train, path, start_offset: float, end_offset: float,
        goal_edge_id: int, goal_t: float, goal_direction: int,
    ) -> None:
        """将 find_path_from_point 的结果转换为 route + remaining_to_goal，下达给 train。

        occupancy 本身不动（车身位置/历史不受影响），只设置待走的 route。
        goal 三元组一并记录到 state.goal，供 route 中途出现折返点时
        （TrainEntity._do_auto_reversal）从新车头位置重新寻路，见 Step 3。
        """
        head_directed = train.head_directed_edge()
        if path.edges and path.edges[0] == head_directed:
            route = path.edges[1:]
        else:
            # 首边不匹配代表当前就需要原地折返才能出发；直接走通用的
            # 折返重寻路路径：先清空 route 触发不了 needs_reversal（那是
            # advance_occupied_path 的中途检测），这里改为显式调用
            # reverse_in_place 后再走一次 assign_route。
            if train.is_parked():
                train.reverse_in_place()
                head_directed = train.head_directed_edge()
                route = path.edges[1:] if (path.edges and path.edges[0] == head_directed) else path.edges
            else:
                print("PLAY: 需要折返才能出发，请先停车")
                return
        remaining_to_goal = path.total_cost - start_offset - end_offset
        train.assign_route(route, max(0.0, remaining_to_goal), (goal_edge_id, goal_t, goal_direction))

    def _issue_path_order(self, world_pos: Vec3) -> None:
        """对 active_train 下达寻路指令（Edge 途中或 Node）。

        右键目标会先检查是否吸附到其他列车的端头车钩（5m 范围内）。
        """

        # Couple-2：检查是否吸附到其他列车端头车钩
        COUPLE_SNAP = 5.0
        snapped_goal: tuple[int, float] | None = None
        for other in self.trains:
            if other is self.active_train:
                continue
            head_data, tail_data = other.kinematics.get_end_coupler_data(other.state.s)
            for cp_pos, cp_edge_id, cp_t in (head_data, tail_data):
                if (cp_pos - world_pos).length() < COUPLE_SNAP:
                    snapped_goal = (cp_edge_id, cp_t)
                    print(f"PLAY: 吸附到列车 #{self.trains.index(other)+1} 车钩 "
                          f"(edge {cp_edge_id} t={cp_t:.2f})")
                    break
            if snapped_goal:
                break

        train = self.active_train
        start_edge_id, start_t = train.current_edge_and_t()
        start_direction = train.current_direction()

        def _edge_pos(edge, t):
            na = self.network.nodes[edge.node_a_id].position
            nb = self.network.nodes[edge.node_b_id].position
            if not edge.is_arc:
                return na + (nb - na) * t
            from model.geom_utils import rotate_around_axis
            return edge.arc_center + rotate_around_axis(
                edge.arc_start_dir, edge.arc_normal, edge.arc_angle_rad * t
            ) * edge.arc_radius

        goal = snapped_goal if snapped_goal is not None else self._snap_edge_at(world_pos)
        if goal is not None:
            goal_edge_id, goal_t = goal
            result = self._find_path_any_goal_direction(
                start_edge_id, start_t, start_direction, goal_edge_id, goal_t,
                allow_reversal=True, debug=True,
                consist_length=train.state.consist.total_length,
            )
            if result is None:
                print(f"PLAY: 不可达 (edge {start_edge_id} → edge {goal_edge_id})")
                return
            path, start_offset, end_offset, goal_direction = result
            se = self.network.edges[start_edge_id]
            ge = self.network.edges[goal_edge_id]
            self.train_path_virtual_points = [_edge_pos(se, start_t), _edge_pos(ge, goal_t)]
            self._apply_route_result(train, path, start_offset, end_offset,
                                     goal_edge_id, goal_t, goal_direction)
            self.train_path = None  # 可视化路径按需从 occupancy+route 重建，不再缓存
            self.train_v_target = 0.0
            idx = self.trains.index(train) + 1
            print(f"PLAY: 列车 #{idx} → edge {goal_edge_id} t={goal_t:.2f}，"
                  f"剩余 {train.state.remaining_to_goal:.0f} m")
            return

        node_id = self._snap_node_at(world_pos)
        if node_id is None:
            print("PLAY: 右键请靠近节点或轨道")
            return
        goal_node = self.network.nodes.get(node_id)
        if goal_node is None:
            return
        goal_edge_id = next(iter(goal_node.incident_edge_ids))
        goal_edge = self.network.edges[goal_edge_id]
        goal_t = 0.0 if goal_edge.node_a_id == node_id else 1.0
        result = self._find_path_any_goal_direction(
            start_edge_id, start_t, start_direction, goal_edge_id, goal_t,
            allow_reversal=True, debug=True,
            consist_length=train.state.consist.total_length,
        )
        if result is None:
            print(f"PLAY: 节点 {node_id} 不可达")
            return
        path, start_offset, end_offset, goal_direction = result
        se = self.network.edges[start_edge_id]
        ge = self.network.edges[goal_edge_id]
        self.train_path_virtual_points = [_edge_pos(se, start_t), _edge_pos(ge, goal_t)]
        self._apply_route_result(train, path, start_offset, end_offset,
                                 goal_edge_id, goal_t, goal_direction)
        self.train_path = None
        self.train_v_target = 0.0
        idx = self.trains.index(train) + 1
        seq = " → ".join(f"e{eid}({'+' if d > 0 else '-'})" for eid, d in path.edges)
        print(f"PLAY: 列车 #{idx} → 节点 {node_id}，剩余 {train.state.remaining_to_goal:.0f} m\n  {seq}")

    def _snap_node_at(self, world_pos: Vec3) -> int | None:
        """返回点击命中的节点 ID，未命中返回 None。"""
        snap = self.editor._snap(world_pos)
        return snap.snapped_node_id

    def _snap_edge_at(self, world_pos: Vec3) -> tuple[int, float] | None:
        """返回点击命中的 (edge_id, t) ∈ [0,1]，未命中返回 None。"""
        snap = self.editor._snap(world_pos)
        if snap.snapped_edge_id is not None and snap.snapped_edge_t is not None:
            return snap.snapped_edge_id, snap.snapped_edge_t
        return None

    def _train_mode_click(self, world_pos: Vec3) -> None:
        pass  # ponytail: 旧入口，已由 _play_left_click/_play_right_click 替代，保留避免引用断裂



def run_game(geo_path: str) -> None:
    GameLoop(geo_path).run()
