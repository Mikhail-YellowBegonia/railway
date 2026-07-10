from __future__ import annotations

from model.kinematics import PathKinematics, Pose
from model.wagon import Consist, solve_rear_bogie_s
from model.pathfinding import Path
from model.rail_network import RailNetwork
from model.vec3 import Vec3


class RigidWagonKinematics:
    """刚体车厢运动学（D3 + D6）：支持多节编组的刚体约束计算。

    输入首节车厢前转向架弧长 s，链式求解所有车厢的转向架和位姿。
    车厢位姿定义（图形渲染用）：
    - 位置 = 两转向架连线中点
    - 朝向 = 连线方向（前 → 后 Bogie）

    多节编组：
    - 第 i 节后转向架 = 第 i+1 节前转向架（车钩连接假设为刚性）
    - 性能：O(2n)，n = 车厢数，链式调用 solve_rear_bogie_s
    """

    def __init__(
        self,
        network: RailNetwork,
        path: Path,
        consist: Consist,
    ) -> None:
        self.network = network
        self.path = path
        self.consist = consist
        self._path_kin = PathKinematics(network, path)

    @property
    def total_length(self) -> float:
        """路径总长（米），与质点模型一致。"""
        return self._path_kin.total_length

    def get_all_wagon_poses(self, front_bogie_s: float) -> list[Pose]:
        """查询所有车厢位姿（给定首节前转向架弧长）。

        参数:
            front_bogie_s: 首节车厢前转向架在路径上的弧长（米）

        返回:
            所有车厢 Pose 列表（顺序同 consist.wagons）
        """
        poses = []
        current_s = front_bogie_s

        for i, wagon in enumerate(self.consist.wagons):
            front_s = current_s
            rear_s = solve_rear_bogie_s(front_s, wagon.bogie_spacing, self._path_kin)

            front_pose = self._path_kin.pose_at(front_s)
            rear_pose = self._path_kin.pose_at(rear_s)

            wagon_pos = (front_pose.position + rear_pose.position) * 0.5
            line_vec = front_pose.position - rear_pose.position
            if line_vec.length() < 1e-9:
                wagon_heading = front_pose.heading
            else:
                wagon_heading = line_vec.normalize()

            poses.append(Pose(position=wagon_pos, heading=wagon_heading))

            # 计算下一节前转向架 s（车钩间隙）
            if i < len(self.consist.wagons) - 1:
                next_wagon = self.consist.wagons[i + 1]
                gap = (wagon.coupler_2_pos - wagon.bogies[1].pos) + \
                      (next_wagon.bogies[0].pos - next_wagon.coupler_1_pos)
                current_s = solve_rear_bogie_s(rear_s, gap, self._path_kin)
            else:
                current_s = rear_s

        return poses

    def get_all_bogie_poses(self, front_bogie_s: float) -> list[tuple[Pose, Pose]]:
        """查询所有转向架位姿（可视化用）。

        返回:
            [(前 Bogie Pose, 后 Bogie Pose), ...] 列表（顺序同 consist.wagons）
        """
        bogie_pairs = []
        current_s = front_bogie_s

        for i, wagon in enumerate(self.consist.wagons):
            front_s = current_s
            rear_s = solve_rear_bogie_s(front_s, wagon.bogie_spacing, self._path_kin)
            front_pose = self._path_kin.pose_at(front_s)
            rear_pose = self._path_kin.pose_at(rear_s)
            bogie_pairs.append((front_pose, rear_pose))

            # 计算下一节前转向架 s（车钩间隙）
            if i < len(self.consist.wagons) - 1:
                next_wagon = self.consist.wagons[i + 1]
                gap = (wagon.coupler_2_pos - wagon.bogies[1].pos) + \
                      (next_wagon.bogies[0].pos - next_wagon.coupler_1_pos)
                current_s = solve_rear_bogie_s(rear_s, gap, self._path_kin)
            else:
                current_s = rear_s

        return bogie_pairs
