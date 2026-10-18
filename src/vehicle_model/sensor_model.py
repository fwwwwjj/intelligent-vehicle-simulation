"""
车载感知传感器数字建模 (Sensor Digital Model)。
包含激光雷达 (LiDAR) 与前视相机 (Camera) 的有效视场角 (FOV)、最大感知测距及高斯测量噪声建模。
"""

from typing import List
import numpy as np
from src.common.state import VehicleState, ObstacleState


class SensorModel:
    """
    虚拟感知传感器数字模型
    """
    def __init__(
        self,
        max_range: float = 80.0,
        fov_deg: float = 360.0,
        pos_noise_std: float = 0.05,
        vel_noise_std: float = 0.10
    ):
        self.max_range = max_range
        self.fov_rad = np.radians(fov_deg)
        self.pos_noise_std = pos_noise_std
        self.vel_noise_std = vel_noise_std

    def filter_perceived_obstacles(
        self,
        ego_state: VehicleState,
        all_obstacles: List[ObstacleState]
    ) -> List[ObstacleState]:
        """
        基于视场几何范围及距离衰减筛选可见目标，并施加高斯感知噪声
        :param ego_state: 自车当前全局状态
        :param all_obstacles: 环境真值所有障碍物
        :return: 感知层输出的带噪目标列表
        """
        perceived = []
        for obs in all_obstacles:
            dx = obs.x - ego_state.x
            dy = obs.y - ego_state.y
            dist = float(np.hypot(dx, dy))

            # 距离范围滤除
            if dist > self.max_range:
                continue

            # 视场角 FOV 滤除
            rel_angle = np.arctan2(dy, dx) - ego_state.heading
            rel_angle = np.arctan2(np.sin(rel_angle), np.cos(rel_angle))
            if np.abs(rel_angle) > (self.fov_rad / 2.0):
                continue

            # 叠加高斯噪声模拟真实传感器感知特性
            noisy_obs = ObstacleState(
                id=obs.id,
                obstacle_type=obs.obstacle_type,
                x=obs.x + np.random.normal(0, self.pos_noise_std),
                y=obs.y + np.random.normal(0, self.pos_noise_std),
                length=obs.length,
                width=obs.width,
                heading=obs.heading,
                vx=obs.vx + np.random.normal(0, self.vel_noise_std),
                vy=obs.vy + np.random.normal(0, self.vel_noise_std),
                predicted_trajectory=obs.predicted_trajectory
            )
            perceived.append(noisy_obs)

        return perceived
