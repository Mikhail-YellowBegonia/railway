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

    参数:
        initial_offset: 起点偏移（米），列车从路径上该弧长处出发（默认 0）
        end_offset: 终点裁剪（米），路径末尾截去该长度作为有效终点（默认 0）
    """

    def __init__(
        self,
        network: RailNetwork,
        path: Path,
        consist: Consist,
        initial_offset: float = 0.0,
        end_offset: float = 0.0,
    ) -> None:
        self.network = network
        self.path = path
        self.consist = consist
        self.initial_offset = initial_offset
        self.end_offset = end_offset
        self._path_kin = PathKinematics(network, path)

    @property
    def total_length(self) -> float:
        """可行驶路径长度（米），已扣除起点偏移和终点裁剪。"""
        return self._path_kin.total_length - self.initial_offset - self.end_offset

    def get_all_wagon_poses(self, front_bogie_s: float) -> list[Pose]:
        """查询所有车厢位姿（给定首节前转向架弧长）。

        参数:
            front_bogie_s: 可行驶区间内的弧长（米），0 = 起点偏移处

        返回:
            所有车厢 Pose 列表（顺序同 consist.wagons）
        """
        poses = []
        current_s = front_bogie_s + self.initial_offset

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
        current_s = front_bogie_s + self.initial_offset

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

    def get_coupler_positions(self, front_bogie_s: float) -> list[Vec3]:
        """返回所有节间连接点的世界坐标（N-1 个，N=车厢数）。coupler_idx i 对应 decouple_at(i)。"""
        return [pos for pos, _eid, _t in self.get_coupler_data(front_bogie_s)]

    def get_coupler_data(self, front_bogie_s: float) -> list[tuple[Vec3, int, float]]:
        """返回所有节间连接点的 (世界坐标, edge_id, t)（N-1 个）。

        供寻路终点吸附使用：edge_id + t 直接作为 find_path_from_point 的目标。
        """
        wagons = self.consist.wagons
        if len(wagons) < 2:
            return []

        result = []
        current_s = front_bogie_s + self.initial_offset

        for i, wagon in enumerate(wagons):
            rear_s = solve_rear_bogie_s(current_s, wagon.bogie_spacing, self._path_kin)

            if i < len(wagons) - 1:
                next_wagon = wagons[i + 1]
                rear_coupler_offset = wagon.coupler_2_pos - wagon.bogies[1].pos
                rear_coupler_s = solve_rear_bogie_s(rear_s, rear_coupler_offset, self._path_kin)
                front_coupler_offset = next_wagon.bogies[0].pos - next_wagon.coupler_1_pos
                gap = rear_coupler_offset + front_coupler_offset
                next_front_s = solve_rear_bogie_s(rear_s, gap, self._path_kin)
                front_coupler_s = solve_rear_bogie_s(rear_s, front_coupler_offset, self._path_kin)

                mid_s = (rear_coupler_s + front_coupler_s) * 0.5
                pos = self._path_kin.pose_at(mid_s).position
                edge_id, t = self._path_kin.edge_at(mid_s)
                result.append((pos, edge_id, t))
                current_s = next_front_s
            else:
                current_s = rear_s

        return result

    def real_tail_bogie_abs_s(self, front_bogie_s: float) -> float:
        """真实车尾——末节车厢**后转向架**的绝对弧长坐标（_path_kin 坐标系）。

        链式求解得到，比 occupied_offset（滑动窗口近似，= abs_head -
        consist_length）精确——后者忽略了转向架相对车钩的内缩，两者可能
        相差数米。折返（reverse_occupancy）必须用这个值做镜像基准，
        不能用 occupied_offset，也不能用车钩弧长（get_end_coupler_data
        返回的是车钩，get_all_wagon_poses 消费的输入是转向架弧长，两者
        不是同一个参考点，混用会导致折返后车厢位置错位）。
        """
        wagons = self.consist.wagons
        abs_s = front_bogie_s + self.initial_offset

        current_s = abs_s
        for i, wagon in enumerate(wagons):
            rear_s = solve_rear_bogie_s(current_s, wagon.bogie_spacing, self._path_kin)
            if i < len(wagons) - 1:
                next_wagon = wagons[i + 1]
                gap = (wagon.coupler_2_pos - wagon.bogies[1].pos) + \
                      (next_wagon.bogies[0].pos - next_wagon.coupler_1_pos)
                current_s = solve_rear_bogie_s(rear_s, gap, self._path_kin)
            else:
                current_s = rear_s  # 末节后转向架
        return current_s

    def real_tail_abs_s(self, front_bogie_s: float) -> float:
        """真实车尾（末节车厢后车钩）的绝对弧长坐标（_path_kin 坐标系）。

        用于 get_end_coupler_data 等"车钩位置"场景。折返镜像基准请用
        real_tail_bogie_abs_s（转向架，不是车钩）。
        """
        tail_bogie_s = self.real_tail_bogie_abs_s(front_bogie_s)
        tail_wagon = self.consist.wagons[-1]
        tail_coupler_offset = tail_wagon.coupler_2_pos - tail_wagon.bogies[1].pos
        return solve_rear_bogie_s(tail_bogie_s, tail_coupler_offset, self._path_kin)

    def get_end_coupler_data(self, front_bogie_s: float) -> tuple[
        tuple[Vec3, int, float],
        tuple[Vec3, int, float],
    ]:
        """返回首尾端头车钩的 (世界坐标, edge_id, t)。

        用于 couple 条件检查：
        - [0] = 车头前车钩（首节 coupler_1_pos 端）
        - [1] = 车尾后车钩（末节 coupler_2_pos 端）
        """
        wagons = self.consist.wagons
        abs_s = front_bogie_s + self.initial_offset

        # 车头前车钩：首节前转向架向前偏移 bogie[0].pos - coupler_1_pos
        head_wagon = wagons[0]
        head_offset = head_wagon.bogies[0].pos - head_wagon.coupler_1_pos
        # 车头方向是正向（abs_s 本身是前转向架），前车钩在前转向架更前方
        # 用 pose_at 的 heading 外推
        head_pose = self._path_kin.pose_at(abs_s)
        head_pos = head_pose.position + head_pose.heading * head_offset
        head_edge_id, head_t = self._path_kin.edge_at(abs_s)

        tail_s = self.real_tail_abs_s(front_bogie_s)
        tail_pos = self._path_kin.pose_at(tail_s).position
        tail_edge_id, tail_t = self._path_kin.edge_at(tail_s)

        return (
            (head_pos, head_edge_id, head_t),
            (tail_pos, tail_edge_id, tail_t),
        )
