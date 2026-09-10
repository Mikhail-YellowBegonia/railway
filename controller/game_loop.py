from __future__ import annotations

import sys
import os

import pygame

from model.rail_network import RailNetwork
from model.geojson_loader import load_geojson, load_signals
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
            self.signals = load_signals(SAVE_PATH, self.network)
        else:
            self.network = load_geojson(geo_path)
            from model.signal import SignalTable
            self.signals = SignalTable()
        self.editor = Editor(self.network)
        self.running = True

        # roadmap #2 会话持久化：列车列表在下方（trains 声明处）初始化，因为
        # 反查需要 network 就绪。这里先留个占位标记，实际加载见 __init__ 末尾。
        self._loaded_trains: list["TrainEntity"] | None = None

        # Step 3：固定闭塞管理（block 划分 + 占用推导信号颜色，每帧 rebuild，
        # 见 model/block.py 顶部关于重算频率的说明）
        from model.block import BlockManager
        self.block_manager = BlockManager()

        # Step 6：信号-运动调度（授权边界 + 预约推进 + 信号前停车等待），
        # 见 model/dispatch.py。每帧对每列车 tick 一次。
        from model.dispatch import TrainDispatcher
        self.dispatcher = TrainDispatcher(self.network, self.signals, self.block_manager)

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
        # roadmap #2 会话持久化：从存档还原列车（含行驶中列车的 route/goal/剩余
        # 距离），并重建 split_sibling 互指（解挂两段共享边的信号豁免）。旧存档
        # 无 "trains" 字段 → 空列表。
        self.trains: list["TrainEntity"] = self._load_saved_trains()
        self.active_train: "TrainEntity | None" = None
        self.train_v_target: float = 0.0           # 焦点列车的巡航速度（m/s）
        self.inspect_train: "TrainEntity | None" = None  # 编组面板目标（I 键切换）
        # 车钩悬停状态（docs/consist_ui.md F 阶段完整 UI）：
        # None = 未悬停；否则 (列车, kind, key)——
        #   kind='internal', key=int(i)  → 第 i 个节间车钩（decouple_at(i) 的解挂点）
        #   kind='end',      key='head'/'tail' → 端头车钩（连挂点，见 coupling.py）
        self._hovered_coupler: tuple["TrainEntity", str, int | str] | None = None
        # self.signals（DirectedEdge -> SignalState 旁挂表）已在上方加载
        # 网络时同步初始化/还原，不在这里重复创建。

    def _load_saved_trains(self) -> list["TrainEntity"]:
        """从存档还原列车（roadmap #2 会话持久化，见 docs/session_persistence.md）。

        在 __init__ 里、network 就绪后调用。旧存档无 "trains" 字段 → 空列表。
        还原后重建 split_sibling 互指（解挂两段共享边的信号豁免）。
        """
        if not os.path.exists(SAVE_PATH):
            return []
        from model.geojson_loader import load_trains
        from model.session import rebuild_split_siblings, rebuild_couple_partners
        trains = load_trains(SAVE_PATH, self.network)
        rebuild_split_siblings(trains)
        # 连挂驶向配对按 goal 几何一次性重建（运行时引用不落盘；否则重载后
        # 带连挂指令的 holding 车无豁免、永久卡在信号前，2026-09 bug2 现场）
        rebuild_couple_partners(trains, self.network)
        if trains:
            print(f"已还原 {len(trains)} 列列车")
        return trains

    def run(self) -> None:
        while self.running:
            for event in pygame.event.get():
                self._handle_event(event)

            # 每帧同步键盘修饰键状态到 Editor（force_straight 等）
            self._sync_modifiers()

            # 更新列车（Step 6：由调度器做授权+预约推进+物理积分；停放列车
            # 在 tick 内直接 no-op，信号前等待的列车也在 tick 内尝试续约恢复）
            dt = self.clock.get_time() / 1000.0

            # ↑/↓ 调整焦点列车目标速度（无论行驶还是信号前等待都可预设）
            if self.active_train is not None:
                keys = pygame.key.get_pressed()
                V_MAX = 30.0
                V_STEP = 5.0
                if keys[pygame.K_UP]:
                    self.train_v_target = min(V_MAX, self.train_v_target + V_STEP * dt)
                elif keys[pygame.K_DOWN]:
                    self.train_v_target = max(0.0, self.train_v_target - V_STEP * dt)
                self.active_train.v_target = self.train_v_target

            # 停车事件自动连挂（docs/consist_ui.md §5.2，2026-09 定稿）：先记录
            # 本帧开始时正在行驶的列车，tick 后凡是从"行驶中"变"停放"的列车，
            # 触发一次自动连挂扫描（停在别的停放列车端头车钩 1m 内且同向即挂）。
            # 已停放许久的列车不在 moving_before 里，不会被反复扫描。
            moving_before = {id(t) for t in self.trains if t.is_moving()}
            for t in self.trains:
                self.dispatcher.tick(t, dt, t.v_target, self.trains)
            just_stopped = [t for t in self.trains
                            if id(t) in moving_before and t.is_parked()]
            for t in just_stopped:
                if t in self.trains:
                    self._auto_couple_on_stop(t)

            # 相机跟随列车
            if self.camera.follow_enabled and self.active_train is not None:
                wagon_poses = self.active_train.kinematics.get_all_wagon_poses(self.active_train.state.s)
                if wagon_poses:
                    lead_pose = wagon_poses[0]
                    self.camera.set_center_smooth(lead_pose.position.x, lead_pose.position.y)

            mouse_world = self._mouse_world_pos()
            self.editor.update_hover(mouse_world)

            # 清理 DELETE/合并等网络变更留下的悬空信号引用，再重算 block
            # （见 model/signal.py::prune_missing 顶部说明——Editor 不知道
            # SignalTable 的存在，靠这里兜底同步，不侵入 Editor 的删除逻辑）
            self.signals.prune_missing(self.network)
            # Step 3：每帧重算 block 划分；Step 4：释放已驶离的预约
            self.block_manager.rebuild(self.network, self.signals)
            self.block_manager.tick_reservations(self.trains)
            signal_colors = self.block_manager.compute_colors(
                self.trains, self.network, self.signals,
            )

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
            self.renderer.draw_overlay(
                self.network, self.editor, mouse_world, self.signals, signal_colors,
            )
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
                                       draw_consist_panel, draw_coupler_highlight,
                                       draw_end_coupler_highlight, draw_text_tooltip)
            draw_console_log(self.renderer.surface, self.renderer._font, _console_messages)
            # PLAY 模式：车钩悬停检测 + 高亮 + tooltip（docs/consist_ui.md §4/§5）
            if self.editor.mode == EditMode.PLAY:
                self._hovered_coupler = self._hit_test_coupler(mouse_world)
                if self._hovered_coupler is not None:
                    train, kind, key = self._hovered_coupler
                    w_s = self.renderer.surface.get_width()
                    h_s = self.renderer.surface.get_height()
                    mx, my = pygame.mouse.get_pos()
                    if kind == "internal":
                        couplers = train.kinematics.get_coupler_positions(train.state.s)
                        cp_idx = int(key)
                        if cp_idx < len(couplers):
                            draw_coupler_highlight(
                                self.renderer.surface, self.camera,
                                couplers[cp_idx], w_s, h_s,
                            )
                        # 解挂 tooltip：N1+N2 预告（停放可解挂；行驶中提示停车）
                        n1 = cp_idx + 1
                        n2 = len(train.state.consist.wagons) - n1
                        if train.is_parked():
                            txt = f"解挂 → {n1}+{n2} 节 · K/空格/回车 确认"
                        else:
                            txt = f"解挂 → {n1}+{n2} 节 · 请先停车"
                        draw_text_tooltip(
                            self.renderer.surface, self.renderer._font,
                            (mx, my), txt,
                        )
                    else:  # kind == "end"
                        head_data, tail_data = \
                            train.kinematics.get_end_coupler_data(train.state.s)
                        pos = head_data[0] if key == "head" else tail_data[0]
                        draw_end_coupler_highlight(
                            self.renderer.surface, self.camera, pos, w_s, h_s,
                        )
                        # 连挂 tooltip：区分"自己端头"与"可连挂目标"
                        other_idx = self.trains.index(train) + 1
                        which = "车头" if key == "head" else "车尾"
                        if train is self.active_train:
                            txt = f"本车{which}（连挂请悬停其它列车端头）"
                        else:
                            txt = f"连挂目标 #{other_idx}（{which}）· K 确认"
                            if not train.is_parked():
                                txt += " · 对方未停车"
                            elif self.active_train is not None and \
                                    not self.active_train.is_parked():
                                txt += " · 请先停车再连挂"
                        draw_text_tooltip(
                            self.renderer.surface, self.renderer._font,
                            (mx, my), txt,
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
        elif event.key == pygame.K_h:
            # H 键：SIGNAL 模式（信号机放置/切换，纯手动，Step 2）
            if self.editor.mode == EditMode.SIGNAL:
                self.editor.set_mode(EditMode.IDLE)
                print("SIGNAL 模式：关闭")
            else:
                self.editor.set_mode(EditMode.SIGNAL)
                print("SIGNAL 模式：开启（左键点节点附近某条边的方向放置/切换信号）")
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
            # S 键保存当前路网 + 信号 + 列车到固定存档（启动时会优先加载它）。
            # roadmap #2 会话持久化：列车状态（位置/速度/编组/route/goal/v_target）
            # 一并落盘，见 docs/session_persistence.md。
            from model.geojson_writer import write_geojson
            write_geojson(self.editor.network, SAVE_PATH,
                          signals=self.signals, trains=self.trains)
            print(f"已保存 {len(self.editor.network.edges)} 条边 + "
                  f"{len(self.signals.all_signals())} 个信号 + "
                  f"{len(self.trains)} 列列车到 {SAVE_PATH}")
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
        elif event.key == pygame.K_k or event.key == pygame.K_KP_ENTER \
                or event.key == pygame.K_RETURN:
            # 编组操作确认键（docs/consist_ui.md §4/§5，2026-09 定稿）：
            #   K / 空格 / 回车 = 悬停内部车钩 → 解挂确认；
            #   K（仅 K）        = 悬停其它列车端头车钩 → 连挂确认。
            # 空格/回车只在"悬停内部车钩"时有编组语义，其余情况退回既有行为
            # （空格 = 紧急停止，回车 = 无操作），避免占用常用键。
            if self._hovered_coupler is not None:
                train, kind, key = self._hovered_coupler
                if kind == "internal":
                    self._decouple_at_hovered(train, int(key))
                elif event.key == pygame.K_k:
                    self._couple_to_hovered(train, key)
            elif event.key == pygame.K_k and self.editor.mode == EditMode.PLAY:
                print("PLAY: 悬停列车内部车钩按 K 解挂，悬停其它列车端头车钩按 K 连挂")
        elif event.key == pygame.K_r:
            # R 键（PLAY 模式）：选中列车原地折返——只切换前进方向（逻辑），
            # 不改编组顺序；任意位置可用，但要求列车停放、车身所在段无道岔。
            # 2026-09 用户要求（服务"玩家手动要求列车折返"场景）。
            if self.editor.mode != EditMode.PLAY:
                print("PLAY: R 折返只在 PLAY 模式可用（P 键进入）")
            elif self.active_train is None:
                print("PLAY: 未选中列车，左键先选中再按 R 折返")
            elif self.active_train.is_moving():
                print("PLAY: 请先让列车完全停车（空格急停）再按 R 折返")
            else:
                t = self.active_train
                if t.reverse_in_place():
                    print(f"PLAY: 列车 #{self.trains.index(t)+1} 已原地折返"
                          f"（只换前进方向，编组顺序不变）")
                # 失败原因由 reverse_in_place 打印（跨道岔等）
        elif event.key == pygame.K_SPACE:
            # 空格键：PLAY 模式且悬停内部车钩(停放) = 解挂确认；否则 = 列车紧急停止
            # （行驶中按空格仍应能紧急停车，不能被悬停语义吞掉；非 PLAY 模式
            # 残留的 _hovered_coupler 不应占用空格，避免 BUILD 里误吞急停）
            if (self.editor.mode == EditMode.PLAY
                    and self._hovered_coupler is not None):
                train, kind, key = self._hovered_coupler
                if kind == "internal" and train.is_parked():
                    self._decouple_at_hovered(train, int(key))
                    return
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

        # SIGNAL 模式：左键=放置/切换信号，不触发平移
        if self.editor.mode == EditMode.SIGNAL:
            if event.button == 1:
                self._signal_left_click(self._mouse_world_pos())
                return

        # DELETE 模式：左键优先删除命中的列车（2026-09 用户要求，测试需要清理
        # 错放的列车）；未命中列车才交编辑器删轨道。不触发平移。
        if self.editor.mode == EditMode.DELETE and event.button == 1:
            world_pos = self._mouse_world_pos()
            hit = self._hit_test_train(world_pos)
            if hit is not None:
                self._delete_train(hit)
                return
            # 拒绝删除被列车占用/预约的轨道（2026-09 用户报"删有列车的轨道
            # 崩溃"；TF2/OpenTTD 同样拒绝删除有车/有进路的区段）。列车
            # occupied/route 里的 edge_id 一旦被删即成悬空引用，下一帧
            # dispatcher.tick / kinematics 直接 KeyError 崩溃。
            reason = self._rail_delete_blocked_reason()
            if reason is not None:
                print(f"DELETE 拒绝：{reason}（先把列车开走或删除列车）")
                return
            # 未命中列车、轨道也没被占用 → 落到编辑器删轨道（下方 handle_click）

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

    def _decouple_at_hovered(self, train: "TrainEntity", coupler_idx: int) -> None:
        """解挂确认（docs/consist_ui.md §4）：悬停内部车钩 + K/空格/回车。

        只处理停放列车；行驶中提示先停车（保持现状不执行）。
        """
        if self.editor.mode != EditMode.PLAY:
            return
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
        # 单节无动力提示（docs/consist_ui.md §4.2：允许 + 提示）
        for label, seg in (("前段", front), ("后段", rear)):
            if len(seg.state.consist.wagons) == 1 \
                    and not seg.state.consist.wagons[0].is_powered:
                print(f"PLAY: 提示 {label} 为单节无动力车厢，无法自行行驶")

    def _couple_to_hovered(self, target_train: "TrainEntity", target_end: str) -> None:
        """K 键手动连挂确认（docs/consist_ui.md §5）：悬停其它列车端头车钩 + K。

        三情形（§5.2，就近对接语义，2026-09 用户拍板；2026-09 bug1 放宽门槛）：
        - 已贴住（<1m、同向、双停）→ 立即 couple；
        - 可前进驶向对接（两车同向停放、B 尾钩在 A 车头前方、未超护栏距离）→
          下达驶向指令（目标 = B 尾钩，可达性由寻路判定，允许隔弧/隔节点/
          隔岔路的远距驶向），停车事件自动连挂接管（§5.2/§9-4 定稿）；
        - 其余（已越过目标需倒车 / 逆行对顶 / 未停）→ 提示，不下达。
        K 键与右键吸附车钩**语义等价**（§9 定稿），目标端 head/tail 都接受——
        具体能否对接由 drive_couple_goal 的宽松栅栏 + 寻路统一决定，不再区分端头。
        """
        if self.editor.mode != EditMode.PLAY:
            return
        front = self.active_train
        if front is None:
            print("PLAY: 未选中列车，左键先选中或放置")
            return
        if target_train is front:
            print("PLAY: 悬停的是本车端头，请悬停其它列车的端头再按 K")
            return

        from controller.coupling import try_couple_to, drive_couple_goal, head_hook_offset

        # 情形 1：已贴住 → 立即连挂。悬停端只是玩家"意图"，但贴住可能是另一端
        # （例如解挂后两段紧贴，玩家悬停任意端都该能撤销解挂）。两端都试一次，
        # 任一贴住（<1m 且接触端切线一致）即连挂——修复"显然可连却被提示方向
        # 不对"的 bug（2026-09：旧实现只试悬停端，另一端贴住时误落 drive 分支）。
        match = try_couple_to(front, target_train, target_end)
        if match is None and target_end == "head":
            match = try_couple_to(front, target_train, "tail")
        elif match is None:
            match = try_couple_to(front, target_train, "head")
        if match is not None:
            self._merge_couple(match)
            return

        # 情形 2/3：未贴住 → 驶向连挂判定（仅前进驶向，无倒车）
        if not front.is_parked():
            print("PLAY: 请先停车再连挂（空格键），或右键下达行驶指令")
            return
        if not target_train.is_parked():
            print("PLAY: 对方列车未停车，无法连挂")
            return

        goal = drive_couple_goal(front, target_train)
        if goal is None:
            print(f"PLAY: 无法连挂——两车需**同向停放**、本车车头朝前且在对方"
                  f"车尾的**后方**（相距过远或已越过对方时先手动行驶到其后方，"
                  f"本版本不支持倒车连挂）")
            return
        _edge_id, _t = goal
        # stop_before = 本车车钩与头转向架的间距：停车点是头转向架，但连挂要让
        # **车钩** 停在对方尾钩处，须提前该距离停车（_apply_route_result 说明）。
        self._issue_goal_order(front, _edge_id, _t,
                               f"连挂 #{self.trains.index(target_train)+1} 车尾",
                               stop_before_m=head_hook_offset(front),
                               goal_direction=target_train.current_direction(),
                               ignore_signals=True)
        # 信号豁免（docs/consist_ui.md §5.5）：驶向连挂目标时，调度层对本车
        # 全放行（dispatch._resolve_couple_target → 调车分支），允许开进对方
        # 占用的受保护区间甚至逆单向；授权仍 clamp 在目标尾钩（stop_before +
        # update 的 min(remaining, authority) 保证不压线）。
        front.couple_approach_partner = target_train
        print(f"PLAY: 已下达驶向 #{self.trains.index(target_train)+1} "
              f"车尾指令，到位后自动连挂")

    def _auto_couple_on_stop(self, stopped: "TrainEntity") -> None:
        """停车事件自动连挂（docs/consist_ui.md §5.2/§9-4，2026-09 定稿）。

        在 run() 每帧检测到某列车刚由行驶变停放时调用一次：扫描全部其它
        停放列车，端头车钩 <1m 且同向即自动 couple（距离最近优先）。普通
        右键寻路只要停在别的列车端头车钩旁就会自动挂上，无需再按 K。
        """
        from controller.coupling import find_couple_pair
        match = find_couple_pair(stopped, self.trains)
        if match is not None:
            self._merge_couple(match, auto=True)

    def _merge_couple(self, match, auto: bool = False) -> None:
        """执行 CoupleMatch 并更新 trains 列表（手动/自动共用）。"""
        merged_head, merged_rear, dist = match.merged_head, match.merged_rear, match.pair_dist
        if merged_head not in self.trains or merged_rear not in self.trains:
            return  # 已被其它连挂消耗
        label = f"#{self.trains.index(merged_head)+1}+#{self.trains.index(merged_rear)+1}"
        new_train = merged_head.couple_with(merged_rear)
        self.trains.remove(merged_head)
        self.trains.remove(merged_rear)
        self.trains.append(new_train)
        self.active_train = new_train
        self.train_path = None
        self.inspect_train = None
        self._hovered_coupler = None
        n = len(new_train.state.consist.wagons)
        tag = "自动" if auto else "手动"
        print(f"PLAY: Couple {tag} {label} → {n} 节")

    def _hit_test_train(self, world_pos: Vec3) -> "TrainEntity | None":
        """返回点击命中的列车，未命中返回 None。判定：任意 wagon 中心 < 5m。"""
        HIT_RADIUS = 5.0
        for train in self.trains:
            poses = train.kinematics.get_all_wagon_poses(train.state.s)
            for pose in poses:
                if (pose.position - world_pos).length() < HIT_RADIUS:
                    return train
        return None

    def _delete_train(self, train: "TrainEntity") -> None:
        """DELETE 模式点击列车 → 删除该列车（2026-09 用户要求，测试清理用）。

        列车从 trains 列表移除；其预约由 tick_reservations 下一帧按"不在列表"
        自动清理，split_sibling 伙伴的互指引用随对象一起释放（豁免自然失效）。
        若删除的是焦点列车，清空焦点/悬停/编组面板状态。
        """
        idx = self.trains.index(train) + 1
        self.trains.remove(train)
        if self.active_train is train:
            self.active_train = None
            self.train_path = None
        if self.inspect_train is train:
            self.inspect_train = None
        if self._hovered_coupler is not None and self._hovered_coupler[0] is train:
            self._hovered_coupler = None
        print(f"PLAY: 已删除列车 #{idx}")

    def _train_locked_edges(self) -> set[int]:
        """所有列车当前占用（occupied）或已预约/待走（route）的 edge_id 集合。"""
        locked: set[int] = set()
        for t in self.trains:
            locked |= {eid for eid, _d in t.state.occupancy.occupied}
            locked |= {eid for eid, _d in t.state.occupancy.route}
        return locked

    def _rail_delete_blocked_reason(self) -> str | None:
        """DELETE 点击轨道前的安全检查：命中对象是否被列车占用/预约。

        返回拒绝原因字符串（应拒绝删除），或 None（可安全删除）。与
        Transport Fever 2 / OpenTTD 一致：有车或有进路的区段不允许拆除。
        被删的 edge_id 会留在列车 occupied/route 里成悬空引用，下一帧
        dispatcher.tick（network.edges[eid]）或 kinematics 重建即崩溃
        （2026-09 用户实测）。
        """
        locked = self._train_locked_edges()
        if not locked:
            return None
        # editor._delete 的命中优先级是 Node > Edge，这里按同一优先级检查：
        # 删节点走"合并两条边"（同样会移除旧 edge_id/新建边），也要拦。
        nid = self.editor.hovered_node_id
        if nid is not None:
            node = self.network.nodes.get(nid)
            if node is not None and (set(node.incident_edge_ids) & locked):
                return f"节点 {nid} 关联的轨道正被列车占用或预约"
        eid = self.editor.hovered_edge_id
        if eid is not None and eid in locked:
            return f"轨道 edge {eid} 正被列车占用或预约"
        return None

    def _hit_test_coupler(self, _world_pos: Vec3,
                          screen_pos: tuple[int, int] | None = None) \
            -> tuple["TrainEntity", str, int | str] | None:
        """返回鼠标命中的车钩 (列车, kind, key)，未命中返回 None。

        docs/consist_ui.md §2/§6：命中半径改**屏幕像素基准**（COUPLER_HIT_PX），
        缩放看全局时车钩仍可点——不再用世界米数 1.5 m（缩小时会点不中）。
        - kind='internal'：key=int(coupler_idx)，节间车钩，解挂点（decouple_at(i)）
        - kind='end'：key='head'/'tail'，端头车钩，连挂目标
        同一屏幕位置同时命中多个候选时按屏幕距离取最近。

        screen_pos：测试用显式屏幕坐标（None = 当前鼠标位置）。
        """
        COUPLER_HIT_PX = 14.0
        mx, my = screen_pos if screen_pos is not None else pygame.mouse.get_pos()
        w_s = self.renderer.surface.get_width()
        h_s = self.renderer.surface.get_height()

        best: tuple[float, "TrainEntity", str, int | str] | None = None

        for train in self.trains:
            # 内部车钩（N-1 个节间点）
            for i, cp in enumerate(train.kinematics.get_coupler_positions(train.state.s)):
                sx, sy = self.camera.world_to_screen(cp.x, cp.y, w_s, h_s)
                d = ((sx - mx) ** 2 + (sy - my) ** 2) ** 0.5
                if d < COUPLER_HIT_PX and (best is None or d < best[0]):
                    best = (d, train, "internal", i)
            # 端头车钩（head/tail）
            head_data, tail_data = train.kinematics.get_end_coupler_data(train.state.s)
            for key, data in (("head", head_data), ("tail", tail_data)):
                cp = data[0]
                sx, sy = self.camera.world_to_screen(cp.x, cp.y, w_s, h_s)
                d = ((sx - mx) ** 2 + (sy - my) ** 2) ** 0.5
                if d < COUPLER_HIT_PX and (best is None or d < best[0]):
                    best = (d, train, "end", key)

        if best is None:
            return None
        return (best[1], best[2], best[3])

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
            # 默认编组：3 节相同车厢、全部有动力（便于测试，2026-09 用户要求）。
            # 同长同质量同功率，解挂/连挂后任一段都能独立行驶，测试对称性好。
            wagon1 = create_simple_wagon(length=20.0, mass=50.0, P_rated=3000.0)
            wagon2 = create_simple_wagon(length=20.0, mass=50.0, P_rated=3000.0)
            wagon3 = create_simple_wagon(length=20.0, mass=50.0, P_rated=3000.0)
            self.train_placement_consist = Consist(wagons=[wagon1, wagon2, wagon3])
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
        fixed_goal_direction: int | None = None,
        ignore_signals: bool = False,
    ):
        """玩家点击目标点时不指定到达方向，分别尝试 +1/-1，取代价更低者。

        Step 2：find_path_from_point 现在要求显式 goal_direction（消除到达
        方向歧义），但 PLAY 模式下玩家右键点选轨道并不表达"以哪个方向进站"
        的意图，所以在这一层统一枚举两个方向。

        fixed_goal_direction: 连挂驶向用（2026-09 bug2）：到达方向必须使
        车头在目标钩位处与目标列车同向——枚举 -1 时可能找到"绕行后从反方向
        接近目标车尾"的路径（A 冲到 B 尾旁却因朝向不符连不上）。传 B 的
        current_direction() 时只尝试该方向，绕行反向接近自然判不可达。其余
        场景（右键普通寻路）不传，保持枚举。

        ignore_signals: 连挂驶向用（2026-09 bug2 现场，用户裁决"调车忽略
        一切限制、只看轨道拓扑"）：连挂驶向要把车开到对方占用的受保护区间
        甚至逆单向信号（相对信号把区间禁成单行时，正向追尾会因 One-Way
        被判不可达/被迫绕行）。此时寻路过滤用 None（纯拓扑 + turn_allowed，
        不看 One-Way 背面禁行）——调车引导语义，普通寻路不受影响。

        consist_length 必须传真实列车长度（Step 3 阶段 B）：决定折返在
        哪些 endpoint 可行——simple_segment 长度不足的死端会被寻路层直接
        排除，避免"折返后车尾越过 turnout"或"死端间无限振荡"的问题。

        find_path_from_point 不再修改网络（起点/终点都不分割），两次调用
        可以直接复用其中一次的结果，不需要"先探路再提交"的两阶段流程。

        Step 5（远场/近场拆分，2026-09）：这是**唯一跑 Dijkstra 的地方**
        （远场寻路），默认只用 `SignalTable.passable_topology_only` 过滤——
        只看拓扑级的单向硬性限制（One-Way PBS 背面永久禁止），不看闭塞
        占用/预约状态。判定可达性、算出的是"理论上能不能走到"和"完整
        预期路径"，不该因为"眼前有别的车"就判不可达（红灯不代表这里
        真的走不了，只代表暂时被占用）。近场调度（预约 + 授权 + 信号前
        停车等待）现在完全由 `model/dispatch.py::TrainDispatcher` 每帧处理
        （Step 6），不重新跑 Dijkstra、也不在这里处理。因此这里不再需要
        `requesting_train` 参数。

        返回 (path, start_offset, end_offset, goal_direction) 或 None
        （枚举方向都不可达）。
        """
        from model.pathfinding import find_path_from_point
        passable_fn = (None if ignore_signals
                       else self.signals.passable_topology_only)
        best = None
        best_gd = None
        for gd in (fixed_goal_direction,) if fixed_goal_direction is not None \
                else (1, -1):
            result = find_path_from_point(
                self.network, start_edge_id, start_t, start_direction,
                goal_edge_id, goal_t, gd,
                passable_fn=passable_fn,
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
        stop_before_m: float = 0.0,
    ) -> None:
        """将 find_path_from_point 的结果转换为 route + remaining_to_goal，下达给 train。

        occupancy 本身不动（车身位置/历史不受影响），只设置待走的 route。
        goal 三元组一并记录到 state.goal，供 route 中途出现折返点时
        （TrainEntity._do_auto_reversal）从新车头位置重新寻路，见 Step 3。

        stop_before_m: 车头转向架在到达几何目标点前提前停车的距离（米）。连挂
        驶向用——停车点是"车头转向架"，但连挂要求**车钩**贴住对方尾钩；车钩在
        转向架前方 head_offset（20m 车 ≈2.5m），所以驶向连挂目标须提前
        head_offset 停车（docs/consist_ui.md §5.2，2026-09 修正：旧实现把转向架
        开到目标点，车钩越过 2.5m 永远够不上 <1m 判定）。
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
                if not train.reverse_in_place():
                    print("PLAY: 无法起步行车方向（原地折返被拒，见上方原因）")
                    return
                head_directed = train.head_directed_edge()
                route = path.edges[1:] if (path.edges and path.edges[0] == head_directed) else path.edges
            else:
                print("PLAY: 需要折返才能出发，请先停车")
                return
        remaining_to_goal = (path.total_cost - start_offset - end_offset
                             - stop_before_m)
        # 记录本次指令的停车提前量（连挂驶向用）；折返（_do_auto_reversal）
        # 后按几何重算 remaining 时要减去同一个提前量（2026-09 bug2 修复）。
        train._stop_before_m = stop_before_m
        train.assign_route(route, max(0.0, remaining_to_goal),
                           (goal_edge_id, goal_t, goal_direction))

    def _issue_goal_order(
        self, train, goal_edge_id: int, goal_t: float, label: str,
        stop_before_m: float = 0.0,
        goal_direction: int | None = None,
        ignore_signals: bool = False,
    ) -> None:
        """对指定列车下达"驶向 (goal_edge_id, goal_t)"的远场寻路指令。

        由 _issue_path_order（右键）与 _couple_to_hovered（K 键连挂驶向）共用。
        远场寻路（Dijkstra，默认只看拓扑 + One-Way PBS 反方向硬性禁止）选方向
        -> 下达完整远场 route。

        goal_direction: 连挂驶向用——固定到达方向为目标列车 current_direction()
        （否则枚举 ±1 时，单向信号迫使绕行会让车从**反方向**接近目标车尾，
        贴上了却因朝向不符连不上）。普通右键寻路不传（保持枚举）。

        ignore_signals: 连挂驶向用（2026-09 用户裁决"调车忽略一切限制"）——
        寻路过滤用纯拓扑（忽略 One-Way 背面禁行），配合 dispatch 的调车全放行
        分支，让驶向连挂能开进对方占用的受保护区间甚至逆单向。普通寻路不传。

        find_path_from_point 不再修改网络（起点/终点都不分割），一次调用即可
        拿到可下达的干净路径，不再需要"探路+提交"两阶段流程。

        Step 6（信号接入运动控制，2026-09）：这里不再做任何预约/拒绝——远场
        寻路只回答"理论上可达吗"，判定可达性靠 `passable_topology_only`（拓扑
        单向限制），与闭塞占用/预约无关（红灯不代表不可达，只代表暂时被占）。
        列车拿到完整远场 route 后，由 `TrainDispatcher` 每帧推进预约：前方红灯
        时开到信号前停车等待、绿灯时自动续约恢复，对应 OpenTTD"接受指令、信号
        前等待"的语义。这里只负责可达性判定 + 下达 route。
        """
        def _edge_pos(edge, t):
            na = self.network.nodes[edge.node_a_id].position
            nb = self.network.nodes[edge.node_b_id].position
            if not edge.is_arc:
                return na + (nb - na) * t
            from model.geom_utils import rotate_around_axis
            return edge.arc_center + rotate_around_axis(
                edge.arc_start_dir, edge.arc_normal, edge.arc_angle_rad * t
            ) * edge.arc_radius

        start_edge_id, start_t = train.current_edge_and_t()
        start_direction = train.current_direction()
        consist_length = train.state.consist.total_length
        result = self._find_path_any_goal_direction(
            start_edge_id, start_t, start_direction, goal_edge_id, goal_t,
            allow_reversal=True, debug=True, consist_length=consist_length,
            fixed_goal_direction=goal_direction,
            ignore_signals=ignore_signals,
        )
        if result is None:
            print(f"PLAY: 不可达 ({label})")
            return
        path, start_offset, end_offset, goal_direction = result

        se = self.network.edges[start_edge_id]
        ge = self.network.edges[goal_edge_id]
        self.train_path_virtual_points = [_edge_pos(se, start_t), _edge_pos(ge, goal_t)]
        self._apply_route_result(train, path, start_offset, end_offset,
                                 goal_edge_id, goal_t, goal_direction,
                                 stop_before_m=stop_before_m)
        self.train_path = None  # 可视化路径按需从 occupancy+route 重建，不再缓存
        # 不下达后清零巡航：真实 GUI 每帧把 self.train_v_target 写回
        # active_train.v_target（run()），旧代码在这里清零会把玩家已用 ↑
        # 建立的巡航抹掉——"停车 ↑ 设速→右键设目的地"或"行驶中右键改向"
        # 后列车刹停不动（2026-09 bug2，橙色路径可见但车不走）。手动驾驶
        # 语义不变：停放车巡航=0 下达后仍需按 ↑ 起步；急停（空格）才清零。
        idx = self.trains.index(train) + 1
        cruise = train.v_target
        cruise_hint = (f"，按 ↑ 起步" if cruise <= 0.0
                       else f"，保持 {cruise:.1f} m/s 巡航")
        print(f"PLAY: 列车 #{idx} → {label}，剩余 {train.state.remaining_to_goal:.0f} m"
              f"{cruise_hint}")

    def _issue_path_order(self, world_pos: Vec3) -> None:
        """对 active_train 下达寻路指令（Edge 途中或 Node）。

        右键目标会先检查是否吸附到其他列车的端头车钩（5m 范围内）——
        吸附命中即下达驶向该车钩的指令，到位后由 _auto_couple_on_stop
        （停车事件自动连挂，docs/consist_ui.md §5.2/§9-4）自动连挂。
        """

        # Couple-2：检查是否吸附到其他列车端头车钩（5m 范围内）。
        # 与 K 键语义等价（2026-09 定稿）：吸附命中后走 drive_couple_goal 宽松栅栏
        # 判定（同向 + 前方 + 护栏距离），可达性交给寻路，不再是"无条件把车头开到
        # 任意端头坐标"（那会在目标在身后时绕大圈/折返边，见 docs/consist_ui.md §5.2）。
        COUPLE_SNAP = 5.0
        snapped_target = None  # 被吸附端头车钩所属的列车（连挂豁免用，§5.5）
        for other in self.trains:
            if other is self.active_train:
                continue
            head_data, tail_data = other.kinematics.get_end_coupler_data(other.state.s)
            for cp_pos, _cp_edge_id, _cp_t in (head_data, tail_data):
                if (cp_pos - world_pos).length() < COUPLE_SNAP:
                    snapped_target = other
                    break
            if snapped_target:
                break

        train = self.active_train
        if snapped_target is not None:
            # 驶向连挂判定（与 K 键一致）
            from controller.coupling import drive_couple_goal, head_hook_offset
            goal = drive_couple_goal(train, snapped_target)
            if goal is None:
                print(f"PLAY: 无法连挂——两车需同向停放、本车车头朝前且在对方"
                      f"车尾的后方（不支持倒车连挂）")
                return
            goal_edge_id, goal_t = goal
            self._issue_goal_order(train, goal_edge_id, goal_t,
                                   f"连挂 #{self.trains.index(snapped_target)+1} 车尾",
                                   stop_before_m=head_hook_offset(train),
                                   goal_direction=snapped_target.current_direction(),
                                   ignore_signals=True)
            train.couple_approach_partner = snapped_target
            print(f"PLAY: 吸附到列车 #{self.trains.index(snapped_target)+1} 车钩，"
                  f"到位后自动连挂")
            return

        goal = self._snap_edge_at(world_pos)
        if goal is not None:
            goal_edge_id, goal_t = goal
            self._issue_goal_order(train, goal_edge_id, goal_t,
                                   f"edge {goal_edge_id} t={goal_t:.2f}")
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
        self._issue_goal_order(train, goal_edge_id, goal_t, f"节点 {node_id}")

    def _signal_left_click(self, world_pos: Vec3) -> None:
        """SIGNAL 模式左键：在最近节点上放置一个方向的信号。

        点击落在 Edge 中途时先 split_edge_at（新节点度数恒为 2，见
        CLAUDE.md「图拓扑只因基础设施变化而改变」——放置信号本身就是
        基础设施变化，允许分割）；否则吸附到最近节点。槛位消解交给
        RailNetwork.resolve_directed_edge_by_click（纯几何，不依赖
        view/controller）。

        Step 3：颜色改为由闭塞占用自动推导（BlockManager），玩家不再
        能手动切换红绿——点击已存在的信号槛位只打印提示，不做任何
        状态改动（Step 2 的 toggle 交互已废弃，见 model/signal.py 顶部
        说明）。放置/移除信号会改变闭塞划分，需要 BlockManager 重算，
        当前实现是每帧无条件 rebuild（见 model/block.py），这里不用
        手动触发。
        """
        node_id = self._snap_node_at(world_pos)
        if node_id is None:
            edge_hit = self._snap_edge_at(world_pos)
            if edge_hit is None:
                print("SIGNAL: 请靠近节点或轨道点击")
                return
            edge_id, t = edge_hit
            node_id = self.network.split_edge_at(edge_id, t)
            if node_id is None:
                print("SIGNAL: 该位置无法放置信号（截断失败）")
                return

        best_edge_id = self.network.resolve_directed_edge_by_click(node_id, world_pos)
        if best_edge_id is None:
            print(f"SIGNAL: 节点 {node_id} 没有可用方向")
            return

        from model.pathfinding import _directed_from
        directed = _directed_from(self.network, best_edge_id, node_id)
        if self.signals.has_signal(directed):
            print(f"SIGNAL: 节点 {node_id} 方向 edge {best_edge_id} 已有信号"
                  f"（颜色由占用状态自动决定，不支持手动切换）")
        elif self.signals.is_blocked_backside(directed):
            print(f"SIGNAL: 节点 {node_id} 方向 edge {best_edge_id} 是反向信号的背面，"
                  f"不能在此放置（One-Way PBS，先点另一侧删除或换方向）")
        else:
            self.signals.place(directed)
            print(f"SIGNAL: 节点 {node_id} 方向 edge {best_edge_id} 新建信号")

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
