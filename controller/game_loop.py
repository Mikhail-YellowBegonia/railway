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

        # 列车模式（F 键切换）：
        #   第一次点击 = 在节点处放置列车（停放状态）
        #   后续点击   = 选择目标（节点或 Edge 途中），自动寻路并出发
        self.train_mode_enabled = False
        self.train_path = None                     # 当前可视化路径
        self.train_path_virtual_points = []        # 寻路虚拟节点位置（调试用）

        # 放置列车的两步流程
        self.train_placement_node_id = None        # 第一步：选中的放置节点
        self.train_placement_consist = None        # 待放置的车组

        # 列车实体（持久存在，停放时 controller=None）
        self.train: "TrainEntity | None" = None
        self.train_v_target: float = 0.0           # 玩家设定的巡航速度（m/s）

    def run(self) -> None:
        while self.running:
            for event in pygame.event.get():
                self._handle_event(event)

            # 每帧同步键盘修饰键状态到 Editor（force_straight 等）
            self._sync_modifiers()

            # 更新列车（物理层 + 状态积分）
            if self.train is not None and self.train.is_moving():
                dt = self.clock.get_time() / 1000.0

                # ↑/↓ 调整目标速度
                keys = pygame.key.get_pressed()
                V_MAX = 30.0
                V_STEP = 5.0
                if keys[pygame.K_UP]:
                    self.train_v_target = min(V_MAX, self.train_v_target + V_STEP * dt)
                elif keys[pygame.K_DOWN]:
                    self.train_v_target = max(0.0, self.train_v_target - V_STEP * dt)

                self.train.update(dt, self.train_v_target)

                # 相机跟随列车
                if self.camera.follow_enabled and self.train is not None:
                    wagon_poses = self.train.kinematics.get_all_wagon_poses(self.train.state.s)
                    if wagon_poses:
                        lead_pose = wagon_poses[0]
                        self.camera.set_center_smooth(lead_pose.position.x, lead_pose.position.y)

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
            # 列车模式可视化
            if self.train_mode_enabled and self.train_path is not None:
                from view.renderer import draw_pathfinding_debug, draw_debug_train
                # 路径预览（橙色）：从起点到终点的完整路径
                draw_pathfinding_debug(
                    self.renderer.surface,
                    self.camera,
                    self.network,
                    self.renderer._font,
                    None,
                    None,
                    self.train_path,
                )
                # 路径上的预览列车（橙色，s=0 起点）
                # 只在停放时显示（出发后隐藏，避免与红色实时列车混淆）
                from model.rigid_kinematics import RigidWagonKinematics
                if self.train is not None and self.train.is_parked():
                    preview_kin = RigidWagonKinematics(
                        self.network, self.train_path, self.train.state.consist
                    )
                    draw_debug_train(
                        self.renderer.surface,
                        self.camera,
                        preview_kin,
                        0.0,
                        occupied=False,
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

            if self.train is not None:
                from view.renderer import draw_debug_train, draw_train_hud
                # 实时占位列车（红色）
                draw_debug_train(
                    self.renderer.surface,
                    self.camera,
                    self.train.kinematics,
                    self.train.state.s,
                    occupied=True,
                )
                # 制动区判断（HUD 显示用）
                _v = self.train.state.v
                _a_b = abs(self.train.physics.compute_acceleration(
                    _v, 0.0, 1.0, self.train.state.consist))
                _in_braking = False
                if self.train.controller is not None:
                    _d_stop = (_v ** 2) / max(1e-6, 2 * _a_b) \
                              * self.train.controller.safety_margin
                    _in_braking = (self.train.kinematics.total_length
                                   - self.train.state.s) <= _d_stop
                draw_train_hud(
                    self.renderer.surface,
                    self.renderer._font,
                    self.train.state.s,
                    self.train.state.v,
                    self.train.last_a,
                    self.train.kinematics.total_length,
                    self.train_v_target,
                    braking=_in_braking,
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
            # F 键切换列车模式
            self.train_mode_enabled = not self.train_mode_enabled
            if not self.train_mode_enabled:
                self.train_path = None
            status = "开启" if self.train_mode_enabled else "关闭"
            print(f"列车模式: {status}（第一次点击放置列车，后续点击选择目标）")
        elif event.key == pygame.K_c:
            # C 键切换相机跟随（A1）
            self.camera.follow_enabled = not self.camera.follow_enabled
            status = "开启" if self.camera.follow_enabled else "关闭"
            print(f"相机跟随: {status}")
        elif event.key == pygame.K_SPACE:
            # 空格键：列车紧急停止（emergency_stop）
            if self.train is not None:
                self.train.emergency_stop()
                self.train_v_target = 0.0
                print("列车紧急停止")
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

        # 列车模式：左键专用于选点，优先于平移拦截
        if event.button == 1 and self.train_mode_enabled:
            self._train_mode_click(self._mouse_world_pos())
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

    # ===== 列车模式（F 键叠加态）=====

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
        from model.pathfinding import find_path_from_point, find_path_between_nodes
        from model.pathfinding import Path, _directed_from
        from model.train_physics import RealisticElectric
        from model.wagon import create_simple_wagon, Consist
        from model.train_entity import TrainEntity, TrainState

        # 第一步：选择放置节点
        if self.train is None and self.train_placement_node_id is None:
            node_id = self._snap_node_at(world_pos)
            if node_id is None:
                print("列车模式：请点选节点放置列车")
                return

            node = self.network.nodes.get(node_id)
            if not node or not node.incident_edge_ids:
                print(f"列车模式：节点 {node_id} 无关联边，无法放置")
                return

            # 保存放置节点和车组，进入"选择朝向"状态
            locomotive = create_simple_wagon(length=20.0, mass=50.0, P_rated=3000.0)
            coach1     = create_simple_wagon(length=18.0, mass=45.0, P_rated=None)
            coach2     = create_simple_wagon(length=22.0, mass=60.0, P_rated=None)
            self.train_placement_consist = Consist(wagons=[locomotive, coach1, coach2])
            self.train_placement_node_id = node_id
            print(f"列车模式：已选择节点 {node_id}，点击相邻节点指定朝向")
            return

        # 第二步：选择朝向（点击相邻节点）
        if self.train is None and self.train_placement_node_id is not None:
            target_node_id = self._snap_node_at(world_pos)
            if target_node_id is None:
                print("列车模式：请点选相邻节点指定朝向")
                return

            # 检查两节点是否相邻
            placement_node = self.network.nodes[self.train_placement_node_id]
            edge_id = None
            for eid in placement_node.incident_edge_ids:
                edge = self.network.edges[eid]
                if edge.node_a_id == target_node_id or edge.node_b_id == target_node_id:
                    edge_id = eid
                    break

            if edge_id is None:
                print(f"列车模式：节点 {target_node_id} 与放置节点 {self.train_placement_node_id} 不相邻")
                return

            # 创建朝向目标节点的有向边
            park_directed = _directed_from(self.network, edge_id, self.train_placement_node_id)
            park_path = Path(edges=[park_directed],
                             total_cost=self.network.edges[edge_id].length)

            state = TrainState(path=park_path, s=0.0, v=0.0, consist=self.train_placement_consist)
            physics = RealisticElectric()
            self.train = TrainEntity(state, self.network, physics)
            self.train_v_target = 0.0
            self.camera.follow_enabled = True

            # 清除放置状态
            self.train_placement_node_id = None
            self.train_placement_consist = None
            print(f"列车模式：列车已放置，朝向节点 {target_node_id}，点击目标位置出发")
            return

        # 后续点击：从当前位置寻路到目标（支持 Edge 途中）
        start_edge_id, start_t = self.train.current_edge_and_t()
        start_direction = self.train.current_direction()

        # 提取车尾覆盖路径（用于拼接到新路径前，保持车尾连续）
        tail_path, initial_offset_tail, s_head_in_tail = self.train.tail_coverage_path()

        # 判断是否允许折返：列车停在终点时允许，行驶中途不允许
        # 终点判定：速度接近 0 且 s 接近路径末端
        at_end_of_path = (
            abs(self.train.state.v) < 0.1 and
            self.train.kinematics.total_length - self.train.state.s < 5.0  # 距离终点 < 5m
        )
        allow_reversal = at_end_of_path

        # 优先尝试 snap 到 Edge 途中
        goal = self._snap_edge_at(world_pos)
        if goal is not None:
            goal_edge_id, goal_t = goal
            result = find_path_from_point(
                self.network,
                start_edge_id, start_t,
                goal_edge_id, goal_t,
                start_direction=start_direction,
                allow_reversal=allow_reversal,
            )
            if result is None:
                print(f"列车模式：不可达（edge {start_edge_id} t={start_t:.2f} → edge {goal_edge_id} t={goal_t:.2f}）")
                return
            path, start_offset, end_offset = result

            # 记录虚拟节点位置（调试可视化）
            start_edge = self.network.edges[start_edge_id]
            goal_edge = self.network.edges[goal_edge_id]

            # 手动插值计算虚拟节点位置
            def edge_position_at(edge, t):
                node_a = self.network.nodes[edge.node_a_id]
                node_b = self.network.nodes[edge.node_b_id]
                if not edge.is_arc:
                    return node_a.position + (node_b.position - node_a.position) * t
                # 圆弧
                angle_at_t = edge.arc_angle_rad * t
                from model.geom_utils import rotate_around_axis
                rotated_dir = rotate_around_axis(edge.arc_start_dir, edge.arc_normal, angle_at_t)
                return edge.arc_center + rotated_dir * edge.arc_radius

            start_virtual_pos = edge_position_at(start_edge, start_t)
            goal_virtual_pos = edge_position_at(goal_edge, goal_t)
            self.train_path_virtual_points = [start_virtual_pos, goal_virtual_pos]

            # 拼接车尾路径到新路径前（保证覆盖整列车身）
            from model.pathfinding import Path

            # 去重：如果 tail_path 的最后一条 Edge 和 path 的第一条 Edge 完全相同（edge_id + direction），跳过 path 的第一条
            if (tail_path.edges and path.edges and
                tail_path.edges[-1] == path.edges[0]):  # 比较 (edge_id, direction) 元组
                # Edge 完全重复，只保留一份
                full_path = Path(
                    edges=tail_path.edges + path.edges[1:],
                    total_cost=tail_path.total_cost + path.total_cost - self.network.edges[path.edges[0][0]].length
                )
            else:
                full_path = Path(
                    edges=tail_path.edges + path.edges,
                    total_cost=tail_path.total_cost + path.total_cost
                )

            # full_start_offset = 车尾在完整路径首段的偏移
            full_start_offset = initial_offset_tail

            self.train_path = full_path
            self.train.assign_path(full_path, full_start_offset, end_offset)
            # 手动调整 state.s 为车头在新路径上的位置
            self.train.state.s = s_head_in_tail
            self.train_v_target = 0.0
            print(
                f"列车模式：→ edge {goal_edge_id} t={goal_t:.2f}，"
                f"共 {len(full_path.edges)} 段，路径长 {full_path.total_cost:.1f} m"
            )
            print(f"  → 按住 ↑ 加速，↓ 减速，空格 紧急停止")
            return

        # snap 未命中 Edge，回退到 Node
        node_id = self._snap_node_at(world_pos)
        if node_id is None:
            print("列车模式：未点中轨道，请靠近节点或轨道点击")
            return

        goal_node = self.network.nodes.get(node_id)
        if goal_node is None:
            return
        # 以目标节点所在任意边的端点 t=0 作为目标
        goal_edge_id = next(iter(goal_node.incident_edge_ids))
        goal_edge = self.network.edges[goal_edge_id]
        goal_t = 0.0 if goal_edge.node_a_id == node_id else 1.0

        result = find_path_from_point(
            self.network,
            start_edge_id, start_t,
            goal_edge_id, goal_t,
            start_direction=start_direction,
            allow_reversal=allow_reversal,
        )
        if result is None:
            print(f"列车模式：节点 {node_id} 不可达")
            return
        path, start_offset, end_offset = result

        # 记录虚拟节点位置（调试可视化）
        start_edge = self.network.edges[start_edge_id]
        goal_edge = self.network.edges[goal_edge_id]

        # 手动插值计算虚拟节点位置
        def edge_position_at(edge, t):
            node_a = self.network.nodes[edge.node_a_id]
            node_b = self.network.nodes[edge.node_b_id]
            if not edge.is_arc:
                return node_a.position + (node_b.position - node_a.position) * t
            # 圆弧
            angle_at_t = edge.arc_angle_rad * t
            from model.geom_utils import rotate_around_axis
            rotated_dir = rotate_around_axis(edge.arc_start_dir, edge.arc_normal, angle_at_t)
            return edge.arc_center + rotated_dir * edge.arc_radius

        start_virtual_pos = edge_position_at(start_edge, start_t)
        goal_virtual_pos = edge_position_at(goal_edge, goal_t)
        self.train_path_virtual_points = [start_virtual_pos, goal_virtual_pos]

        # 拼接车尾路径到新路径前
        from model.pathfinding import Path

        # 去重：如果 tail_path 的最后一条 Edge 和 path 的第一条 Edge 完全相同（edge_id + direction），跳过 path 的第一条
        if (tail_path.edges and path.edges and
            tail_path.edges[-1] == path.edges[0]):  # 比较 (edge_id, direction) 元组
            full_path = Path(
                edges=tail_path.edges + path.edges[1:],
                total_cost=tail_path.total_cost + path.total_cost - self.network.edges[path.edges[0][0]].length
            )
        else:
            full_path = Path(
                edges=tail_path.edges + path.edges,
                total_cost=tail_path.total_cost + path.total_cost
            )

        full_start_offset = initial_offset_tail

        self.train_path = full_path
        self.train.assign_path(full_path, full_start_offset, end_offset)
        # 手动调整 state.s 为车头在新路径上的位置
        self.train.state.s = s_head_in_tail
        self.train_v_target = 0.0
        seq = " → ".join(f"e{eid}({'+' if d > 0 else '-'})" for eid, d in full_path.edges)
        print(
            f"列车模式：→ 节点 {node_id}，共 {len(full_path.edges)} 段，"
            f"总长 {full_path.total_cost:.1f} m\n  {seq}"
        )
        print(f"  → 按住 ↑ 加速，↓ 减速，空格 紧急停止")


def run_game(geo_path: str) -> None:
    GameLoop(geo_path).run()
