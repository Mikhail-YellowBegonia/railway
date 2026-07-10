from __future__ import annotations

from model.kinematics import PathKinematics, Pose
from model.wagon import WagonConfig, solve_rear_bogie_s
from model.pathfinding import Path
from model.rail_network import RailNetwork
from model.vec3 import Vec3


class RigidWagonKinematics:
    """刚体车厢运动学（D3）：考虑转向架间割线距离约束。

    替代质点模型 PathKinematics，输入前转向架弧长 s，输出车厢位姿。
    车厢位姿定义（图形渲染用）：
    - 位置 = 两转向架连线中点
    - 朝向 = 连线方向（前 → 后 Bogie）

    内部用 D2 求解器计算后转向架弧长，避免"挤扁"现象。
    """

    def __init__(
        self,
        network: RailNetwork,
        path: Path,
        wagon_config: WagonConfig,
    ) -> None:
        self.network = network
        self.path = path
        self.wagon_config = wagon_config
        # 复用质点运动学查询单点位姿
        self._path_kin = PathKinematics(network, path)

    @property
    def total_length(self) -> float:
        """路径总长（米），与质点模型一致。"""
        return self._path_kin.total_length

    def get_wagon_pose(self, front_bogie_s: float) -> Pose:
        """查询车厢位姿（给定前转向架弧长）。

        参数:
            front_bogie_s: 前转向架在路径上的弧长（米）

        返回:
            车厢 Pose（图形原点 = 两 Bogie 中点，朝向 = 连线方向）
        """
        # D2: 求解后转向架弧长
        rear_bogie_s = solve_rear_bogie_s(
            front_bogie_s,
            self.wagon_config.bogie_spacing,
            self._path_kin,
        )

        # 查询两转向架的位姿
        front_pose = self._path_kin.pose_at(front_bogie_s)
        rear_pose = self._path_kin.pose_at(rear_bogie_s)

        # 车厢位置 = 两 Bogie 连线中点
        wagon_pos = (front_pose.position + rear_pose.position) * 0.5

        # 车厢朝向 = 连线方向（前 → 后）
        line_vec = front_pose.position - rear_pose.position
        if line_vec.length() < 1e-9:
            # 极端情况：两 Bogie 重合（转向架间距为 0），用前 Bogie 朝向
            wagon_heading = front_pose.heading
        else:
            wagon_heading = line_vec.normalize()

        return Pose(position=wagon_pos, heading=wagon_heading)

    def get_bogie_poses(self, front_bogie_s: float) -> tuple[Pose, Pose]:
        """查询两个转向架的位姿（debug/可视化用）。

        返回:
            (前转向架 Pose, 后转向架 Pose)
        """
        rear_bogie_s = solve_rear_bogie_s(
            front_bogie_s,
            self.wagon_config.bogie_spacing,
            self._path_kin,
        )
        front_pose = self._path_kin.pose_at(front_bogie_s)
        rear_pose = self._path_kin.pose_at(rear_bogie_s)
        return (front_pose, rear_pose)
