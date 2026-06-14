from __future__ import annotations

from enum import Enum, auto

from model.geom_utils import (
    is_t1_consistent_with_target,
    project_along_direction,
    solve_case2_arc,
)
from model.rail_network import RailNetwork
from model.vec3 import Vec3
from controller.snap import SnapSystem, SnapResult
from controller.build_plan import ConstructionPlan, PreviewGeometry

SNAP_THRESHOLD = 0.3
WARNING_THRESHOLD = 0.5
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

        # BUILD 模式状态
        self.build_state: BuildState = BuildState.IDLE
        self.build_m1: Vec3 | None = None              # M1 世界坐标
        self.build_m1_node_id: int | None = None       # M1 吸附的节点 ID（None 表示空白）
        self.build_t1_candidates: list[Vec3] = []      # M1 处的所有候选切线（空 = 无切线约束 / Case 1）

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
        """统一的吸附入口：BUILD_ACTIVE 时传入 M1 作为切线方向参考。"""
        reference = (
            self.build_m1
            if (self.mode == EditMode.BUILD and self.build_state == BuildState.ACTIVE)
            else None
        )
        return self.snap_system.snap(world_pos, self.network, reference)

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
        else:
            self.build_m1 = world_pos
            self.build_m1_node_id = None
            self.build_t1_candidates = []

        # 3. 进入 BUILD_ACTIVE
        self.build_state = BuildState.ACTIVE
        self.preview = None  # 预览在 update_hover 中每帧更新

    def _commit_build(self, world_pos: Vec3, snap: SnapResult) -> None:
        """提交建造：从 M1 到 M2 建造轨道。

        所有拒绝一律保持 BUILD_ACTIVE 状态，由用户重新选 M2 或 Esc 取消。
        """
        # 1. 确定 M2
        if snap.snapped:
            m2 = snap.position
            m2_node_id = snap.snapped_node_id
        else:
            m2 = world_pos
            m2_node_id = None

        # 2. 计算建造计划（注意：T2 暂忽略，等 Case 3 Biarc 实现）
        plan = self._compute_plan(
            m1=self.build_m1,
            m2=m2,
            t1_candidates=self.build_t1_candidates,
            m1_node_id=self.build_m1_node_id,
            m2_node_id=m2_node_id,
        )

        if not plan.valid:
            return  # 拒绝；保持 BUILD_ACTIVE

        # 3. TODO: 截断（M2 在既有边中间）
        # 4. 应用计划到网络
        self._apply_plan(plan)

        # 5. 回到 BUILD_IDLE
        self._reset_build_active()

    def _compute_plan(
        self,
        m1: Vec3,
        m2: Vec3,
        t1_candidates: list[Vec3],
        m1_node_id: int | None,
        m2_node_id: int | None,
    ) -> ConstructionPlan:
        """计算建造计划。

        分支：
        - 无 T1 候选 → Case 1（直线）
        - force_straight=True → Case 1（强制直线，临时调试，docs §10.2）
        - 有 T1 候选 → 按 (M2-M1) 选最佳 T1 → Case 2（弧 / 退化为沿 T1 直线）
        """
        # M1 == M2 → 拒绝
        if m1.distance_to(m2) < 1e-6:
            return ConstructionPlan(case=1, m1=m1, m2=m2, valid=False)

        # Case 1：无切线约束 或 强制直线
        if not t1_candidates or self.force_straight:
            return ConstructionPlan(
                case=1,
                m1=m1,
                m2=m2,
                node_a_id=m1_node_id,
                node_b_id=m2_node_id,
                edge_geometry=[],
                valid=True,
            )

        # Case 2：选择最佳 T1（与 M2-M1 夹角最小者）
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
            # Q5: 所有候选与 d 钝角（或垂直）→ 拒绝
            return ConstructionPlan(case=2, m1=m1, m2=m2, valid=False)

        # Q2 决议：T1 反向（M2 在 T1 背后）→ 拒绝
        if not is_t1_consistent_with_target(best_t1, m1, m2):
            return ConstructionPlan(case=2, m1=m1, m2=m2, valid=False)

        # 求弧
        result = solve_case2_arc(m1, best_t1, m2)
        if result is None:
            # 弧不可解（M2 几乎在 T1 延长线上）→ 退化为沿 T1 的直线
            return self._degenerate_to_straight(m1, best_t1, m2, m1_node_id)

        center, b_point, arc_normal, radius = result

        # Q3 决议：半径过大 → 退化为沿 T1 的直线
        if radius > MAX_ARC_RADIUS:
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

        # 创建边
        self.network.add_edge(node_a, node_b, plan.edge_geometry)

    def _update_preview(self, snap: SnapResult) -> None:
        """更新 BUILD_ACTIVE 的预览几何"""
        if self.build_m1 is None:
            self.preview = None
            return

        m2 = snap.position

        # 计算预览计划（和提交时相同逻辑）
        plan = self._compute_plan(
            m1=self.build_m1,
            m2=m2,
            t1_candidates=self.build_t1_candidates,
            m1_node_id=self.build_m1_node_id,
            m2_node_id=snap.snapped_node_id,
        )

        self.preview = PreviewGeometry(
            m1=plan.m1,
            m2=plan.m2,
            case=plan.case,
            edge_geometry=plan.edge_geometry,
            valid=plan.valid,
        )

    # ===== DELETE 逻辑 =====

    def _delete(self) -> None:
        """删除悬停的 Edge，并连带清理变成孤立的 Node。

        DELETE 模式以 Edge 为操作单位（用户期待与线要素互动而非节点）。
        底线：删除后保证没有孤立节点（connection_count == 0）、没有无头边。
        """
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

        # 连带清理：若任一端点变为孤立（connection_count == 0），删除之。
        for nid in (node_a_id, node_b_id):
            node = self.network.nodes.get(nid)
            if node is not None and node.connection_count() == 0:
                self.network.remove_node(nid)

    # ===== 辅助函数 =====

    def _has_nearby_element(self, pos: Vec3) -> bool:
        """检查位置附近是否有节点或边（用于警告）"""
        if self.network.node_id_at(pos, WARNING_THRESHOLD) is not None:
            return True
        if self.network.edge_id_at(pos, WARNING_THRESHOLD) is not None:
            return True
        return False
