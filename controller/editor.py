from __future__ import annotations

from enum import Enum, auto

from model.rail_network import RailNetwork
from model.vec3 import Vec3
from controller.snap import SnapSystem, SnapResult
from controller.build_plan import ConstructionPlan, PreviewGeometry

SNAP_THRESHOLD = 0.3
WARNING_THRESHOLD = 0.5


class EditMode(Enum):
    """顶层编辑模式"""
    IDLE = auto()     # 空闲：仅视图操作
    BUILD = auto()    # 建造轨道
    DELETE = auto()   # 删除节点或边


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
        self.build_t1: Vec3 | None = None              # M1 处的切线方向

        # 悬停状态（用于 DELETE 和视觉反馈）
        self.hovered_node_id: int | None = None
        self.hovered_edge_id: int | None = None

        # 预览（BUILD_ACTIVE 时有效）
        self.preview: PreviewGeometry | None = None

        # 警告状态（BUILD_IDLE 吸附关闭时邻近元素警告）
        self.show_warning: bool = False

    def update_hover(self, world_pos: Vec3) -> None:
        """每帧更新：处理鼠标悬停 + 预览 + 警告"""
        # 吸附检测
        snap = self.snap_system.snap(world_pos, self.network)

        # 更新悬停状态（用于 DELETE 模式和高亮）
        self.hovered_node_id = snap.snapped_node_id
        if self.mode == EditMode.DELETE:
            if self.hovered_node_id is not None:
                self.hovered_edge_id = None
            else:
                self.hovered_edge_id = self.network.edge_id_at(world_pos, SNAP_THRESHOLD)
        else:
            self.hovered_edge_id = None

        # BUILD_ACTIVE: 更新预览
        if self.mode == EditMode.BUILD and self.build_state == BuildState.ACTIVE:
            self._update_preview(snap)

        # BUILD_IDLE: 检查警告（吸附关闭 + 邻近元素）
        if self.mode == EditMode.BUILD and self.build_state == BuildState.IDLE:
            snap_enabled = self.snap_system.point_snap.enabled
            if not snap_enabled and self._has_nearby_element(world_pos):
                self.show_warning = True
            else:
                self.show_warning = False
        else:
            self.show_warning = False

    def handle_click(self, world_pos: Vec3) -> None:
        """处理左键点击"""
        if self.mode == EditMode.IDLE:
            return  # IDLE 模式下左键无效

        snap = self.snap_system.snap(world_pos, self.network)

        if self.mode == EditMode.BUILD:
            if self.build_state == BuildState.IDLE:
                self._start_build(world_pos, snap)
            else:  # BuildState.ACTIVE
                self._commit_build(world_pos, snap)

        elif self.mode == EditMode.DELETE:
            self._delete(snap)

    def handle_cancel(self) -> None:
        """处理 Esc 键"""
        if self.mode == EditMode.BUILD and self.build_state == BuildState.ACTIVE:
            # 取消当前建造
            self.build_state = BuildState.IDLE
            self.build_m1 = None
            self.build_m1_node_id = None
            self.build_t1 = None
            self.preview = None
        else:
            # 回到 IDLE 模式
            self.set_mode(EditMode.IDLE)

    def set_mode(self, mode: EditMode) -> None:
        """切换顶层模式"""
        self.mode = mode
        # 重置 BUILD 状态
        self.build_state = BuildState.IDLE
        self.build_m1 = None
        self.build_m1_node_id = None
        self.build_t1 = None
        self.preview = None
        self.show_warning = False

    # ===== BUILD 逻辑 =====

    def _start_build(self, world_pos: Vec3, snap: SnapResult) -> None:
        """开始建造：记录 M1，进入 BUILD_ACTIVE"""
        # 1. 建造前检查：吸附关闭且附近有元素 → 拒绝
        snap_enabled = self.snap_system.point_snap.enabled
        if not snap_enabled and self._has_nearby_element(world_pos):
            self.show_warning = True
            return

        # 2. 确定 M1
        if snap.snapped:
            self.build_m1 = snap.position
            self.build_m1_node_id = snap.snapped_node_id
            self.build_t1 = snap.tangent  # 可能为 None（Case 1）或有值（Case 2/3）
        else:
            self.build_m1 = world_pos
            self.build_m1_node_id = None
            self.build_t1 = None

        # 3. 进入 BUILD_ACTIVE
        self.build_state = BuildState.ACTIVE
        self.preview = None  # 预览在 update_hover 中每帧更新

    def _commit_build(self, world_pos: Vec3, snap: SnapResult) -> None:
        """提交建造：从 M1 到 M2 建造轨道"""
        # 1. 确定 M2
        if snap.snapped:
            m2 = snap.position
            m2_node_id = snap.snapped_node_id
            t2 = snap.tangent
        else:
            m2 = world_pos
            m2_node_id = None
            t2 = None

        # 2. 计算建造计划
        plan = self._compute_plan(
            m1=self.build_m1,
            m2=m2,
            t1=self.build_t1,
            t2=t2,
            m1_node_id=self.build_m1_node_id,
            m2_node_id=m2_node_id,
        )

        if not plan.valid:
            # 计划无效（例如 M1 == M2），拒绝
            return

        # 3. TODO: 如果需要截断（M2 在既有边中间），先执行截断
        # if plan.split_edge_id is not None:
        #     self._split_edge(plan.split_edge_id, plan.split_at_t)

        # 4. 应用计划到网络
        self._apply_plan(plan)

        # 5. 回到 BUILD_IDLE
        self.build_state = BuildState.IDLE
        self.build_m1 = None
        self.build_m1_node_id = None
        self.build_t1 = None
        self.preview = None

    def _compute_plan(
        self,
        m1: Vec3,
        m2: Vec3,
        t1: Vec3 | None,
        t2: Vec3 | None,
        m1_node_id: int | None,
        m2_node_id: int | None,
    ) -> ConstructionPlan:
        """计算建造计划（Step 1 仅实现 Case 1 直线）"""
        # 检查 M1 == M2
        if m1.distance_to(m2) < 1e-6:
            return ConstructionPlan(case=1, m1=m1, m2=m2, valid=False)

        # Step 1: 只实现 Case 1（无切线或忽略切线）
        # TODO: Case 2 (单切线) 和 Case 3 (双切线) 在后续步骤实现
        return ConstructionPlan(
            case=1,
            m1=m1,
            m2=m2,
            node_a_id=m1_node_id,
            node_b_id=m2_node_id,
            edge_geometry=[],  # 直线
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
        t2 = snap.tangent

        # 计算预览计划（和提交时相同逻辑）
        plan = self._compute_plan(
            m1=self.build_m1,
            m2=m2,
            t1=self.build_t1,
            t2=t2,
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

    def _delete(self, snap: SnapResult) -> None:
        """删除悬停的节点或边"""
        if self.hovered_node_id is not None:
            self.network.remove_node(self.hovered_node_id)
            self.hovered_node_id = None
        elif self.hovered_edge_id is not None:
            self.network.remove_edge(self.hovered_edge_id)
            self.hovered_edge_id = None

    # ===== 辅助函数 =====

    def _has_nearby_element(self, pos: Vec3) -> bool:
        """检查位置附近是否有节点或边（用于警告）"""
        if self.network.node_id_at(pos, WARNING_THRESHOLD) is not None:
            return True
        if self.network.edge_id_at(pos, WARNING_THRESHOLD) is not None:
            return True
        return False
