from __future__ import annotations

from enum import Enum, auto

from model.geom_utils import (
    can_merge_arcs,
    can_merge_straight,
    is_biarc_collinear_straight,
    is_t1_consistent_with_target,
    merged_arc_b_point,
    project_along_direction,
    project_on_segment,
    solve_biarc,
    solve_case2_arc,
    solve_case2_composite,
    solve_case2t,
)
from model.rail_network import RailNetwork
from model.vec3 import Vec3
from controller.snap import SnapSystem, SnapResult
from controller.build_plan import ConstructionPlan, PreviewGeometry

# 世界阈值基准值（米）：等于默认缩放下的像素阈值换算结果，保留作文档/参考语义。
# 实际吸附改用下方像素基准 + 每帧换算，不再直接使用这两个常量。
SNAP_THRESHOLD = 0.3
WARNING_THRESHOLD = 0.5
# 进阶吸附参数(snapping.md):长度增量 100m,角度增量 30°,固定常量
LENGTH_SNAP_INCREMENT = 100.0  # 米
ANGLE_SNAP_INCREMENT = 30.0    # 度
# 每帧按 camera.scale 换算成世界阈值：world = px / scale。
# 选值使默认缩放 scale=40 下换算结果 = 上面的世界常量（12/40=0.3, 20/40=0.5），
# 从而默认手感与旧实现一致，仅在缩放时正确跟随。
SNAP_THRESHOLD_PX = 12.0
WARNING_THRESHOLD_PX = 20.0
DEFAULT_PIXEL_SCALE = 40.0  # 像素/世界单位；与 Camera 初始 scale 对齐
MAX_ARC_RADIUS = 500.0  # 弧半径超过此值时退化为沿 T1 的直线（Q3）


class EditMode(Enum):
    """EditMode 顶层模式 = IDLE / BUILD / DELETE.

    DELETE 操作单位是 Edge，自动清理孤立 Node。
    """
    IDLE = auto()     # 空闲：仅视图操作
    BUILD = auto()    # 建造轨道
    DELETE = auto()   # 删除轨道（边为单位）


class BuildState(Enum):
    """BUILD 模式的子状态"""
    IDLE = auto()     # 等待第一次点击
    ACTIVE = auto()   # 已选定 M1，等待 M2 确认


class Editor:
    def __init__(self, network: RailNetwork) -> None:
        self.network = network
        self.mode: EditMode = EditMode.IDLE
        self.snap_system = SnapSystem()

        # 当前像素/世界缩放（由 GameLoop 每帧从 camera.scale 同步）。
        # 用于把屏幕像素阈值换算为世界阈值，使吸附半径随缩放跟随。
        self.pixel_scale: float = DEFAULT_PIXEL_SCALE

        # BUILD 模式状态
        self.build_state: BuildState = BuildState.IDLE
        self.build_m1: Vec3 | None = None              # M1 世界坐标
        self.build_m1_node_id: int | None = None       # M1 吸附的节点 ID（None 表示空白）
        self.build_t1_candidates: list[Vec3] = []      # M1 处的所有候选切线（空 = 无切线约束 / Case 1）
        # M1 路径吸附时记录待截断的边（commit 时执行截断，得到新中间节点）
        self.build_m1_edge_id: int | None = None
        self.build_m1_edge_t: float | None = None

        # 悬停状态（用于 DELETE 和视觉反馈）
        self.hovered_node_id: int | None = None
        self.hovered_edge_id: int | None = None

        # 预览（BUILD_ACTIVE 时有效）
        self.preview: PreviewGeometry | None = None

        # 警告状态（BUILD_IDLE 吸附关闭时邻近元素警告）
        self.show_warning: bool = False

        # 修饰键标志：由 GameLoop 每帧同步
        # force_straight=True 时强制走 Case 1 直线（即便 T1 有候选）。临时调试用，参见 docs §10.2
        self.force_straight: bool = False
        # force_case2t=True 时（§10.5）：M2 路径吸附直边 + 有 T1 → 调用 Case 2T，
        # 接入点由算法决定（舍弃精确 M2，改为从 M2 所在边的方向线求切圆）。
        # 触发键：LALT（与 LSHIFT 互斥），按住期间临时启用。
        self.force_case2t: bool = False

        # 进阶吸附开关：由 GameLoop 每帧同步（功能键切换，见 docs/snapping.md）
        self.grid_snap_enabled: bool = False  # G 键切换格点吸附
        self.length_snap_enabled: bool = False  # L 键切换长度吸附(仅直线建造)
        self.angle_snap_enabled: bool = False  # A 键切换角度吸附(仅单弧建造)
        self.parallel_snap_enabled: bool = False  # P 键切换平行吸附

    def update_hover(self, world_pos: Vec3) -> None:
        """每帧更新：处理鼠标悬停 + 预览 + 警告"""
        # 吸附检测
        snap = self._snap(world_pos)

        # 更新悬停状态（用于 DELETE 模式和高亮）
        self.hovered_node_id = snap.snapped_node_id
        if self.mode == EditMode.DELETE:
            # DELETE 模式：操作单位是边。点吸附到节点时不算 hover edge，
            # 否则用路径吸附结果作为 hover edge。
            if snap.snapped_node_id is not None:
                self.hovered_edge_id = None
            elif snap.snapped_edge_id is not None:
                self.hovered_edge_id = snap.snapped_edge_id
            else:
                self.hovered_edge_id = None
        else:
            self.hovered_edge_id = None

        # BUILD_ACTIVE: 更新预览
        if self.mode == EditMode.BUILD and self.build_state == BuildState.ACTIVE:
            self._update_preview(snap)

        # BUILD_IDLE: 检查警告（所有吸附 Provider 关闭 + 邻近元素）
        if self.mode == EditMode.BUILD and self.build_state == BuildState.IDLE:
            if not self.snap_system.any_enabled() and self._has_nearby_element(world_pos):
                self.show_warning = True
            else:
                self.show_warning = False
        else:
            self.show_warning = False

    def handle_click(self, world_pos: Vec3) -> None:
        """处理左键点击"""
        if self.mode == EditMode.IDLE:
            return  # IDLE 模式下左键无效

        snap = self._snap(world_pos)

        if self.mode == EditMode.BUILD:
            if self.build_state == BuildState.IDLE:
                self._start_build(world_pos, snap)
            else:  # BuildState.ACTIVE
                self._commit_build(world_pos, snap)

        elif self.mode == EditMode.DELETE:
            self._delete()

    def _snap(self, world_pos: Vec3) -> SnapResult:
        """统一的吸附入口：BUILD_ACTIVE 时传入 M1 作为切线方向参考。

        吸附阈值以屏幕像素为基准，按当前缩放换算为世界阈值传入吸附系统。
        格点吸附的 enabled 状态由外部功能键控制。
        平行吸附每帧更新参考点(网络变化或开关切换时)。
        """
        # 同步格点吸附状态与缩放
        self.snap_system.grid_snap.enabled = self.grid_snap_enabled
        self.snap_system.grid_snap.pixel_scale = self.pixel_scale

        # 同步平行吸附状态与缩放,并更新参考点
        self.snap_system.parallel_snap.enabled = self.parallel_snap_enabled
        self.snap_system.parallel_snap.pixel_scale = self.pixel_scale
        if self.parallel_snap_enabled:
            self.snap_system.parallel_snap.update_reference_points(self.network)

        reference = (
            self.build_m1
            if (self.mode == EditMode.BUILD and self.build_state == BuildState.ACTIVE)
            else None
        )
        return self.snap_system.snap(
            world_pos, self.network, reference, self._world_snap_threshold()
        )

    def _world_snap_threshold(self) -> float:
        """屏幕像素吸附阈值换算成当前世界阈值（米）。"""
        scale = self.pixel_scale if self.pixel_scale > 1e-6 else DEFAULT_PIXEL_SCALE
        return SNAP_THRESHOLD_PX / scale

    def _world_warning_threshold(self) -> float:
        """屏幕像素警告阈值换算成当前世界阈值（米）。"""
        scale = self.pixel_scale if self.pixel_scale > 1e-6 else DEFAULT_PIXEL_SCALE
        return WARNING_THRESHOLD_PX / scale

    def handle_cancel(self) -> None:
        """处理 Esc 键"""
        if self.mode == EditMode.BUILD and self.build_state == BuildState.ACTIVE:
            # 取消当前建造
            self._reset_build_active()
        else:
            # 回到 IDLE 模式
            self.set_mode(EditMode.IDLE)

    def set_mode(self, mode: EditMode) -> None:
        """切换顶层模式"""
        self.mode = mode
        # 重置 BUILD 状态
        self._reset_build_active()
        self.show_warning = False

    def _reset_build_active(self) -> None:
        """清空 BUILD_ACTIVE 状态，回到 BUILD_IDLE。"""
        self.build_state = BuildState.IDLE
        self.build_m1 = None
        self.build_m1_node_id = None
        self.build_t1_candidates = []
        self.build_m1_edge_id = None
        self.build_m1_edge_t = None
        self.preview = None

    # ===== BUILD 逻辑 =====

    def _start_build(self, world_pos: Vec3, snap: SnapResult) -> None:
        """开始建造：记录 M1，进入 BUILD_ACTIVE。

        拒绝条件：所有吸附 Provider 关闭 且 附近有既有元素。
        """
        # 1. 建造前检查
        if not self.snap_system.any_enabled() and self._has_nearby_element(world_pos):
            self.show_warning = True
            return  # 拒绝；保持 BUILD_IDLE

        # 2. 确定 M1
        if snap.snapped:
            self.build_m1 = snap.position
            self.build_m1_node_id = snap.snapped_node_id
            self.build_t1_candidates = list(snap.tangent_candidates)
            # 路径吸附：记下要截断的边和参数（commit 时才真正分裂）
            self.build_m1_edge_id = snap.snapped_edge_id
            self.build_m1_edge_t = snap.snapped_edge_t
        else:
            self.build_m1 = world_pos
            self.build_m1_node_id = None
            self.build_t1_candidates = []
            self.build_m1_edge_id = None
            self.build_m1_edge_t = None

        # 3. 进入 BUILD_ACTIVE
        self.build_state = BuildState.ACTIVE
        self.preview = None  # 预览在 update_hover 中每帧更新

    def _commit_build(self, world_pos: Vec3, snap: SnapResult) -> None:
        """提交建造：从 M1 到 M2 建造轨道。

        所有拒绝一律保持 BUILD_ACTIVE 状态，由用户重新选 M2 或 Esc 取消。

        截断处理（Step 6）：
        - M1/M2 吸附到既有边内部时，commit 时分裂原边产生新中间节点
        - 同边双截断（M1 与 M2 在同一条边上）→ 拒绝（语义模糊，按用户决议）
        """
        # 1. 确定 M2
        if snap.snapped:
            m2 = snap.position
            m2_node_id = snap.snapped_node_id
            t2_candidates = list(snap.tangent_candidates)
            m2_edge_id = snap.snapped_edge_id
            m2_edge_t = snap.snapped_edge_t
            m2_edge_dir = snap.edge_direction  # 直边方向（Case 2T 用）
        else:
            m2 = world_pos
            m2_node_id = None
            t2_candidates = []
            m2_edge_id = None
            m2_edge_t = None
            m2_edge_dir = None

        # 进阶吸附:在 snap 之后、compute_plan 之前调整 M2(长度/角度吸附)
        m2 = self._adjust_m2_for_snapping(m2, self.build_m1, self.build_t1_candidates)

        # 同边双截断 → 拒绝
        if (
            self.build_m1_edge_id is not None
            and m2_edge_id is not None
            and self.build_m1_edge_id == m2_edge_id
        ):
            return

        # 2. 计算建造计划
        plan = self._compute_plan(
            m1=self.build_m1,
            m2=m2,
            t1_candidates=self.build_t1_candidates,
            t2_candidates=t2_candidates,
            m1_node_id=self.build_m1_node_id,
            m2_node_id=m2_node_id,
            m2_edge_id=m2_edge_id,
            m2_edge_dir=m2_edge_dir,
        )

        if not plan.valid:
            return  # 拒绝；保持 BUILD_ACTIVE

        # 3. 截断：先 M1 端，再 M2 端（两端独立，顺序无关）
        if self.build_m1_edge_id is not None and self.build_m1_edge_t is not None:
            new_mid = self.network.split_edge_at(
                self.build_m1_edge_id, self.build_m1_edge_t
            )
            if new_mid is None:
                return  # 截断失败：拒绝
            plan.node_a_id = new_mid

        # M2 端截断：Case 2T（case=5）用 plan 回算的 t；其余用 snap 的 t
        if plan.case == 5 and plan.m2_split_edge_id is not None and plan.m2_split_t is not None:
            new_mid = self.network.split_edge_at(plan.m2_split_edge_id, plan.m2_split_t)
            if new_mid is None:
                return
            plan.node_b_id = new_mid
        elif m2_edge_id is not None and m2_edge_t is not None:
            new_mid = self.network.split_edge_at(m2_edge_id, m2_edge_t)
            if new_mid is None:
                return
            plan.node_b_id = new_mid

        # 4. 应用计划到网络
        self._apply_plan(plan)

        # 5. 回到 BUILD_IDLE
        self._reset_build_active()

    def _compute_plan(
        self,
        m1: Vec3,
        m2: Vec3,
        t1_candidates: list[Vec3],
        t2_candidates: list[Vec3],
        m1_node_id: int | None,
        m2_node_id: int | None,
        m2_edge_id: int | None = None,
        m2_edge_dir: Vec3 | None = None,
    ) -> ConstructionPlan:
        """计算建造计划。

        分支（按优先级）：
        - 无 T1 候选 → Case 1（直接连 M1→M2）
        - force_straight=True → 沿 T1 投影直线（§10.2）
        - force_case2t=True + M2 路径吸附直边 → Case 5（§10.5 单切线弧）
        - 有 T1 候选无 T2 候选 → 选最佳 T1，求 Case 2 单弧（半径过大退化/复合）
        - T1 + T2 都有候选 → Case 3 Biarc（失败拒绝）
        - Q2/Q5 拒绝条件适用于所有有 T1 的分支
        """
        # M1 == M2 → 拒绝
        if m1.distance_to(m2) < 1e-6:
            return ConstructionPlan(case=1, m1=m1, m2=m2, valid=False)

        # Case 1：无切线约束（自由直线）
        if not t1_candidates:
            return ConstructionPlan(
                case=1,
                m1=m1,
                m2=m2,
                node_a_id=m1_node_id,
                node_b_id=m2_node_id,
                edge_geometry=[],
                valid=True,
            )

        # 有 T1 候选：先选最佳
        d = m2 - m1
        d_len = d.length()
        if d_len < 1e-9:
            return ConstructionPlan(case=2, m1=m1, m2=m2, valid=False)
        d_hat = d * (1.0 / d_len)

        best_t1: Vec3 | None = None
        best_dot = -float("inf")
        for cand in t1_candidates:
            cn = cand.normalize()
            score = cn.dot(d_hat)
            if score > best_dot:
                best_dot = score
                best_t1 = cn

        if best_t1 is None or best_dot < 1e-6:
            # Q5 检查推迟到 Case 2T 之后（Case 2T 不依赖 m2 方向）
            pass

        # 强制直线（LSHIFT，§10.2）：优先级最高，跳过所有弧 / Biarc / Case 2T 分支
        # （但仍需 best_t1 有效）
        if self.force_straight and best_t1 is not None and best_dot >= 1e-6:
            # Q2 也对 force_straight 适用
            if is_t1_consistent_with_target(best_t1, m1, m2):
                return self._degenerate_to_straight(m1, best_t1, m2, m1_node_id)
            return ConstructionPlan(case=2, m1=m1, m2=m2, valid=False)

        # Case 2T（§10.5）：force_case2t + M2 路径吸附到直边 → 单切线弧
        # 接入点由算法决定，m2 仅作"二选一"的偏好；跳过 Q2/Q5 的 m2 方向检查
        if (
            self.force_case2t
            and m2_edge_id is not None
            and m2_edge_dir is not None
            and m2_node_id is None
            and best_t1 is not None
        ):
            plan_2t = self._try_case2t(
                m1, best_t1, m2, m2_edge_id, m2_edge_dir, m1_node_id
            )
            if plan_2t is not None:
                return plan_2t
            return ConstructionPlan(case=5, m1=m1, m2=m2, valid=False)

        # 正常分支前的 Q5 / Q2 检查
        if best_t1 is None or best_dot < 1e-6:
            return ConstructionPlan(case=2, m1=m1, m2=m2, valid=False)

        if not is_t1_consistent_with_target(best_t1, m1, m2):
            return ConstructionPlan(case=2, m1=m1, m2=m2, valid=False)

        if t2_candidates:
            # Case 3 候选路径：枚举所有 T2 候选，挑能解出且 R 最大者
            plan_3 = self._try_case3_biarc(
                m1, best_t1, m2, t2_candidates, m1_node_id, m2_node_id
            )
            if plan_3 is not None:
                return plan_3
            # 失败 → 直接拒绝（按用户决议：不向 Case 2 降级）
            return ConstructionPlan(case=3, m1=m1, m2=m2, valid=False)

        # 无 T2 → 走 Case 2 单弧
        result = solve_case2_arc(m1, best_t1, m2)
        if result is None:
            # 弧不可解（M2 几乎在 T1 延长线上）→ 退化为沿 T1 的直线
            return self._degenerate_to_straight(m1, best_t1, m2, m1_node_id)

        center, b_point, arc_normal, radius = result

        # Q3: 半径过大 → 尝试"弧+直线"复合（§10.3）
        # 失败时回退到沿 T1 的纯直线（保留 §4.5 旧行为作为兜底）
        if radius > MAX_ARC_RADIUS:
            composite = self._try_case2_composite(m1, best_t1, m2, m1_node_id, m2_node_id)
            if composite is not None:
                return composite
            return self._degenerate_to_straight(m1, best_t1, m2, m1_node_id)

        return ConstructionPlan(
            case=2,
            m1=m1,
            m2=m2,
            node_a_id=m1_node_id,
            node_b_id=m2_node_id,
            edge_geometry=[b_point],  # 弧用 B 点表示
            valid=True,
        )

    def _try_case3_biarc(
        self,
        m1: Vec3,
        t1: Vec3,
        m2: Vec3,
        t2_candidates: list[Vec3],
        m1_node_id: int | None,
        m2_node_id: int | None,
    ) -> ConstructionPlan | None:
        """尝试 Case 3 Biarc。枚举所有 T2 候选，每个候选求 biarc 解，
        返回 R 最大（曲率最小）的合法 Plan，或 None（全部不可解 / 半径超限）。

        T1 和 T2 候选的语义都是"远离自身节点的另一端，朝外延伸"。
        biarc 数学约定 T2 是"沿 M_mid → M2 进入 M2 的方向"，所以传入
        solve_biarc 时取 -t2。共线退化 fast-path 同样用取负后的 t2 判定。
        """
        best_plan: ConstructionPlan | None = None
        best_r = -1.0

        for t2_raw in t2_candidates:
            t2_for_biarc = t2_raw.normalize() * -1.0

            # Fast-path: T1 与 -T2 同向 且 d∥T1 → Case 1 直线（最高优先级）
            if is_biarc_collinear_straight(t1, t2_for_biarc, m1, m2):
                return ConstructionPlan(
                    case=1,
                    m1=m1,
                    m2=m2,
                    node_a_id=m1_node_id,
                    node_b_id=m2_node_id,
                    edge_geometry=[],
                    valid=True,
                )

            result = solve_biarc(m1, t1, m2, t2_for_biarc)
            if result is None:
                continue
            m_mid, b1, b2, an1, an2, radius = result

            if radius > MAX_ARC_RADIUS:
                continue

            if radius > best_r:
                best_r = radius
                best_plan = ConstructionPlan(
                    case=3,
                    m1=m1,
                    m2=m2,
                    node_a_id=m1_node_id,
                    node_b_id=m2_node_id,
                    edge_geometry=[],
                    biarc_mid=m_mid,
                    biarc_geom_1=[b1],
                    biarc_geom_2=[b2],
                    valid=True,
                )

        return best_plan

    def _try_case2t(
        self,
        m1: Vec3,
        t1: Vec3,
        m2_mouse: Vec3,
        m2_edge_id: int,
        edge_dir: Vec3,
        m1_node_id: int | None,
    ) -> ConstructionPlan | None:
        """Case 2T 单切线弧（§10.5）：求一段弧 *切于 (M1, T1)* 且 *切于直线 L*。

        - L 是 M2_mouse 所在直边的方向线（P0 = edge.node_a, T2_dir = edge_dir）
        - M2 的精确接入点由算法决定（舍弃 m2_mouse），存入 plan.m2_split_t
        - 接入点必须落在边的有效范围内（t ∈ [0,1]），返回纯弧 case=5
        - **接入点越界**(t < 0 或 t > 1)时,输出"弧+直线"复合 case=4:
          弧(M1→切点P)+ 直线(P→M2_mouse投影),补齐延长线段

        返回 case=5(纯弧)或 case=4(弧+直线复合)，或 None（不可解）。
        """
        edge = self.network.edges.get(m2_edge_id)
        if edge is None or edge.is_arc:
            return None  # 仅直边适用

        node_a = self.network.nodes[edge.node_a_id]
        node_b = self.network.nodes[edge.node_b_id]
        p0 = node_a.position

        result = solve_case2t(m1, t1, m2_mouse, p0, edge_dir, MAX_ARC_RADIUS)
        if result is None:
            return None

        m2_actual, b_point, _arc_normal, _radius = result

        # 接入点在边上的参数 t
        t_actual, _proj, _dist = project_on_segment(m2_actual, p0, node_b.position)

        # 情况1:切点在边的有效范围内(0 < t < 1) → 纯弧 case=5
        if 1e-6 < t_actual < 1.0 - 1e-6:
            return ConstructionPlan(
                case=5,
                m1=m1,
                m2=m2_actual,
                node_a_id=m1_node_id,
                node_b_id=None,
                edge_geometry=[b_point],
                m2_split_edge_id=m2_edge_id,
                m2_split_t=t_actual,
                valid=True,
            )

        # 情况2:切点越界(在延长线上) → 弧+直线复合 case=4
        # M2 取原始鼠标在边上的投影(用户期望的接入点)
        t_mouse, m2_proj, _dist_mouse = project_on_segment(m2_mouse, p0, node_b.position)
        # 若投影也越界(鼠标远离边),取最近端点
        if t_mouse < 0.0:
            m2_final = p0
        elif t_mouse > 1.0:
            m2_final = node_b.position
        else:
            m2_final = m2_proj

        return ConstructionPlan(
            case=4,
            m1=m1,
            m2=m2_final,
            node_a_id=m1_node_id,
            node_b_id=None,  # M2 在边内部,需截断产生新节点
            edge_geometry=[],
            composite_mid=m2_actual,  # 切点P(中间点)
            composite_arc_geom=[b_point],  # 弧 M1→P
            composite_tail_geom=[],  # 直线 P→M2
            m2_split_edge_id=m2_edge_id,  # M2 需截断边
            m2_split_t=max(0.0, min(1.0, t_mouse)),  # 截断参数
            valid=True,
        )

    def _try_case2_composite(
        self,
        m1: Vec3,
        t1: Vec3,
        m2: Vec3,
        m1_node_id: int | None,
        m2_node_id: int | None,
    ) -> ConstructionPlan | None:
        """Case 2 半径超限时尝试"弧+直线"复合（§10.3）。

        - 弧段：固定半径 = MAX_ARC_RADIUS，从 M1 切于 T1
        - 直线段：从弧终点切线连续地延伸到 M2
        - 输出 case=4，应用时拆为两条 Edge + 中间 Node

        无解（M2 在固定半径圆内 / 弧角超 π / 退化）→ 返回 None，由调用方降级。
        """
        result = solve_case2_composite(m1, t1, m2, MAX_ARC_RADIUS)
        if result is None:
            return None
        p_mid, b_point, _arc_normal, _tail_dir, _radius = result
        return ConstructionPlan(
            case=4,
            m1=m1,
            m2=m2,
            node_a_id=m1_node_id,
            node_b_id=m2_node_id,
            edge_geometry=[],
            composite_mid=p_mid,
            composite_arc_geom=[b_point],
            composite_tail_geom=[],
            valid=True,
        )

    def _degenerate_to_straight(
        self,
        m1: Vec3,
        t1: Vec3,
        m2: Vec3,
        m1_node_id: int | None,
    ) -> ConstructionPlan:
        """Case 2 退化为沿 T1 的直线：终点是 M2 在 (M1, T1) 上的投影。

        如此保证起点切线连续，避免 M1 处出现折角。
        end_node 一律新建（不复用 m2 处的吸附节点，因为目标点已偏移）。
        """
        m2_eff = project_along_direction(m1, t1, m2)
        if m1.distance_to(m2_eff) < 1e-6:
            return ConstructionPlan(case=1, m1=m1, m2=m2_eff, valid=False)
        return ConstructionPlan(
            case=1,
            m1=m1,
            m2=m2_eff,
            node_a_id=m1_node_id,
            node_b_id=None,  # 退化情形：终点是新位置，不复用 M2 的节点
            edge_geometry=[],
            valid=True,
        )

    def _apply_plan(self, plan: ConstructionPlan) -> None:
        """将建造计划应用到网络"""
        # 获取或创建端点节点
        if plan.node_a_id is not None:
            node_a = self.network.nodes[plan.node_a_id]
        else:
            node_a = self.network.add_node(plan.m1)

        if plan.node_b_id is not None:
            node_b = self.network.nodes[plan.node_b_id]
        else:
            node_b = self.network.add_node(plan.m2)

        # 避免自环
        if node_a.node_id == node_b.node_id:
            return

        if plan.case == 3:
            # Biarc 方案 A：两条 Edge + 中间 Node
            if plan.biarc_mid is None or plan.biarc_geom_1 is None or plan.biarc_geom_2 is None:
                return

            # 零长度弧退化检测(环线闭合等场景):中间点与某端点重合 → 退化为单边
            # 零长度段:用另一段的几何;若两段都零长度(M1==M2),直接拒绝(已被自环检查阻止)
            if plan.biarc_mid.distance_to(plan.m1) < 1e-6:
                # 弧1退化,仅添加弧2(mid→M2,实际 M1→M2)
                self.network.add_edge(node_a, node_b, plan.biarc_geom_2)
                return
            if plan.biarc_mid.distance_to(plan.m2) < 1e-6:
                # 弧2退化,仅添加弧1(M1→mid,实际 M1→M2)
                self.network.add_edge(node_a, node_b, plan.biarc_geom_1)
                return

            # 正常 Biarc:两段非零长度
            mid_node = self.network.add_node(plan.biarc_mid)
            try:
                self.network.add_edge(node_a, mid_node, plan.biarc_geom_1)
                self.network.add_edge(mid_node, node_b, plan.biarc_geom_2)
            except ValueError:
                # 数学解算失败兜底：清理已添加的资源（保守处理）
                self.network.remove_node(mid_node.node_id)
            return

        if plan.case == 4:
            # 弧 + 直线复合（§10.3）：两条 Edge + 中间 Node
            if (
                plan.composite_mid is None
                or plan.composite_arc_geom is None
                or plan.composite_tail_geom is None
            ):
                return
            mid_node = self.network.add_node(plan.composite_mid)
            try:
                self.network.add_edge(node_a, mid_node, plan.composite_arc_geom)
                self.network.add_edge(mid_node, node_b, plan.composite_tail_geom)
            except ValueError:
                self.network.remove_node(mid_node.node_id)
            return

        # Case 1 / Case 2：单条 Edge
        self.network.add_edge(node_a, node_b, plan.edge_geometry)

    def _update_preview(self, snap: SnapResult) -> None:
        """更新 BUILD_ACTIVE 的预览几何"""
        if self.build_m1 is None:
            self.preview = None
            return

        m2 = snap.position
        t2_candidates = list(snap.tangent_candidates) if snap.snapped else []

        # 进阶吸附:在 snap 之后、compute_plan 之前调整 M2(长度/角度吸附)
        m2 = self._adjust_m2_for_snapping(m2, self.build_m1, self.build_t1_candidates)

        # 计算预览计划（和提交时相同逻辑）
        plan = self._compute_plan(
            m1=self.build_m1,
            m2=m2,
            t1_candidates=self.build_t1_candidates,
            t2_candidates=t2_candidates,
            m1_node_id=self.build_m1_node_id,
            m2_node_id=snap.snapped_node_id,
            m2_edge_id=snap.snapped_edge_id,
            m2_edge_dir=snap.edge_direction,
        )

        # Case 5（Case 2T）的预览：plan.m2 已经是算法回算的接入点
        case2t_entry = plan.m2 if plan.case == 5 and plan.valid else None

        self.preview = PreviewGeometry(
            m1=plan.m1,
            m2=plan.m2,
            case=plan.case,
            edge_geometry=plan.edge_geometry,
            valid=plan.valid,
            biarc_mid=plan.biarc_mid,
            biarc_geom_1=plan.biarc_geom_1,
            biarc_geom_2=plan.biarc_geom_2,
            composite_mid=plan.composite_mid,
            composite_arc_geom=plan.composite_arc_geom,
            composite_tail_geom=plan.composite_tail_geom,
            case2t_entry=case2t_entry,
        )

    # ===== DELETE 逻辑 =====

    def _delete(self) -> None:
        """DELETE 模式点击：

        - 命中节点（仅 connection_count == 2）→ 尝试合并两侧边为一条新边；不可合并则静默忽略
        - 命中边 → 删除该边，并连带清理变成孤立的 Node
        - 否则 → 无操作

        悬停优先级（在 update_hover 中已固定）：节点 > 边。
        底线：操作完成后没有孤立节点、没有无头边。
        """
        # 命中节点（仅尝试合并 connection==2 的中间节点）
        if self.hovered_node_id is not None:
            self._try_merge_at_node(self.hovered_node_id)
            return

        # 命中边
        if self.hovered_edge_id is None:
            return

        edge = self.network.edges.get(self.hovered_edge_id)
        if edge is None:
            self.hovered_edge_id = None
            return

        node_a_id = edge.node_a_id
        node_b_id = edge.node_b_id

        self.network.remove_edge(self.hovered_edge_id)
        self.hovered_edge_id = None

        for nid in (node_a_id, node_b_id):
            node = self.network.nodes.get(nid)
            if node is not None and node.connection_count() == 0:
                self.network.remove_node(nid)

    def _try_merge_at_node(self, node_id: int) -> None:
        """尝试合并 connection_count == 2 节点两侧的边。

        失败条件（任一满足都静默忽略，不修改网络）：
        - 节点不存在 / 连接数不为 2
        - 两条边一直一弧（混合）
        - 两条直线不共线（夹角 > 阈值）
        - 两条弧不同心 / 不同半径 / 不同向（normal 反向）/ 切线不连续
        - 合并后的弧 B 点求解失败
        """
        node = self.network.nodes.get(node_id)
        if node is None or node.connection_count() != 2:
            return

        edge_ids = list(node.incident_edge_ids)
        e1 = self.network.edges[edge_ids[0]]
        e2 = self.network.edges[edge_ids[1]]

        # 找出两侧的"远端"节点（不是当前 node 的那一端）
        other_a_id = e1.node_a_id if e1.node_a_id != node_id else e1.node_b_id
        other_b_id = e2.node_a_id if e2.node_a_id != node_id else e2.node_b_id

        if other_a_id == other_b_id:
            # 两条边连接相同的远端 → 这是个"双重边/环"，合并后会自环，拒绝
            return

        a_pos = self.network.nodes[other_a_id].position
        b_pos = self.network.nodes[other_b_id].position
        mid_pos = node.position

        # 类型一致性检查
        if e1.is_arc != e2.is_arc:
            return  # 一直一弧不可合并

        new_geometry: list[Vec3]
        if not e1.is_arc:
            # 两条直线
            if not can_merge_straight(a_pos, mid_pos, b_pos):
                return
            new_geometry = []
        else:
            # 两条弧
            if not can_merge_arcs(e1, e2, mid_pos, a_pos, b_pos):
                return
            new_b = merged_arc_b_point(e1, e2, a_pos, b_pos)
            if new_b is None:
                return
            new_geometry = [new_b]

        # 删除中间节点 + 两条原边，再以原远端节点和新几何建一条边
        # 顺序：先删边、再删节点（避免连带删除把远端也清理掉），再 add_edge
        self.network.remove_edge(e1.edge_id)
        self.network.remove_edge(e2.edge_id)
        self.network.remove_node(node_id)
        self.hovered_node_id = None
        self.hovered_edge_id = None

        try:
            self.network.add_edge(
                self.network.nodes[other_a_id],
                self.network.nodes[other_b_id],
                new_geometry,
            )
        except ValueError:
            # add_edge 在弧几何无效时抛出（理论上前面检查后不应发生）。静默吞掉。
            return

    # ===== 辅助函数 =====

    def _has_nearby_element(self, pos: Vec3) -> bool:
        """检查位置附近是否有节点或边（用于警告）"""
        warn = self._world_warning_threshold()
        if self.network.node_id_at(pos, warn) is not None:
            return True
        if self.network.edge_id_at(pos, warn) is not None:
            return True
        return False

    # ===== 进阶吸附:M2 调整器(snapping.md) =====

    def _adjust_m2_for_snapping(
        self, m2: Vec3, m1: Vec3, t1_candidates: list[Vec3]
    ) -> Vec3:
        """在 snap 之后、compute_plan 之前调整 M2,应用长度/角度吸附。

        长度吸附(L 键):所有直线建造,吸附路径总长为 100m 倍数。
        角度吸附(A 键):仅 Case 2 单弧,吸附 angle(T1, M2-M1) 为 30° 倍数。
        L+A 同时开启时互斥,后开的生效(实际由 game_loop 保证互斥,这里仅防御)。
        """
        # L+A 互斥判定(防御性,正常由外层保证)
        if self.length_snap_enabled and self.angle_snap_enabled:
            # 两个都开了,只用长度(或可记录最后按的键,这里简化为长度优先)
            return self._adjust_m2_length(m2, m1, t1_candidates)

        if self.length_snap_enabled:
            return self._adjust_m2_length(m2, m1, t1_candidates)

        if self.angle_snap_enabled:
            return self._adjust_m2_angle(m2, m1, t1_candidates)

        return m2  # 无吸附,原样返回

    def _adjust_m2_length(
        self, m2: Vec3, m1: Vec3, t1_candidates: list[Vec3]
    ) -> Vec3:
        """长度吸附:调整 M2 使路径长度≈LENGTH_SNAP_INCREMENT 倍数。

        - 无切线(Case 1):调整 |M2-M1| 到最近 100m 倍数,保持方向
        - 有切线(force_straight / Case 2 退化):调整 M2 在 (M1, T1) 射线上的投影
        """
        d = m2 - m1
        dist = d.length()
        if dist < 1e-9:
            return m2  # M1==M2,无法调整

        if not t1_candidates:
            # Case 1 自由直线:量化 |M2-M1| 到最近 100m 倍数
            target = round(dist / LENGTH_SNAP_INCREMENT) * LENGTH_SNAP_INCREMENT
            if target < 1e-6:
                target = LENGTH_SNAP_INCREMENT  # 避免量化到 0
            return m1 + d.normalize() * target

        # 有切线:沿 T1 射线量化投影长度(选最佳 T1,同 _compute_plan)
        best_t1 = self._select_best_t1(t1_candidates, m1, m2)
        if best_t1 is None:
            return m2

        proj_len = d.dot(best_t1)
        if proj_len < 1e-6:
            # 投影长度≤0,M2 在 T1 反向,不调整(或强制到最小正值?)
            return m2

        target_len = round(proj_len / LENGTH_SNAP_INCREMENT) * LENGTH_SNAP_INCREMENT
        if target_len < 1e-6:
            target_len = LENGTH_SNAP_INCREMENT
        return m1 + best_t1 * target_len

    def _adjust_m2_angle(
        self, m2: Vec3, m1: Vec3, t1_candidates: list[Vec3]
    ) -> Vec3:
        """角度吸附:调整 M2 使 angle(T1, M2-M1) ≈ ANGLE_SNAP_INCREMENT 倍数。

        仅 Case 2(有 T1 无 T2)时有意义。M2 在以 M1 为中心、当前半径的圆上旋转,
        量化到 30° 档位。
        """
        if not t1_candidates:
            return m2  # 无切线,角度吸附无意义

        best_t1 = self._select_best_t1(t1_candidates, m1, m2)
        if best_t1 is None:
            return m2

        d = m2 - m1
        radius = d.length()
        if radius < 1e-9:
            return m2

        # 当前角度(T1 到 d 的有符号角度,XY 平面)
        import math
        cos_a = best_t1.dot(d) / radius
        cos_a = max(-1.0, min(1.0, cos_a))
        # 用叉积 z 分量判定符号
        cross_z = best_t1.x * d.y - best_t1.y * d.x
        sin_a = cross_z / radius
        current_deg = math.degrees(math.atan2(sin_a, cos_a))

        # 量化到最近的 30° 倍数
        target_deg = round(current_deg / ANGLE_SNAP_INCREMENT) * ANGLE_SNAP_INCREMENT
        target_rad = math.radians(target_deg)

        # 绕 M1 旋转 best_t1 到目标角度
        cos_t = math.cos(target_rad)
        sin_t = math.sin(target_rad)
        # 2D 旋转矩阵
        new_x = best_t1.x * cos_t - best_t1.y * sin_t
        new_y = best_t1.x * sin_t + best_t1.y * cos_t
        return m1 + Vec3(new_x, new_y, 0.0) * radius

    def _select_best_t1(
        self, t1_candidates: list[Vec3], m1: Vec3, m2: Vec3
    ) -> Vec3 | None:
        """从 T1 候选中选与 (M2-M1) 夹角最小者,复用 _compute_plan 的逻辑。"""
        d = m2 - m1
        d_len = d.length()
        if d_len < 1e-9:
            return None
        d_hat = d.normalize()

        best_t1: Vec3 | None = None
        best_dot = -float("inf")
        for cand in t1_candidates:
            cn = cand.normalize()
            score = cn.dot(d_hat)
            if score > best_dot:
                best_dot = score
                best_t1 = cn

        if best_t1 is None or best_dot < 1e-6:
            return None
        return best_t1
