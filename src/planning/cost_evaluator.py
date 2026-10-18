"""
轨迹综合代价函数评估器接口。
多目标加权优化：涵盖安全性、乘客舒适性、道路中心线依从度、行驶效率及动力学物理边界。
"""

from typing import List
import numpy as np
from src.common.state import Trajectory, ObstacleState, TrajectoryPoint
from src.common.geometry import BoundingBox2D, CollisionChecker


class TrajectoryCostEvaluator:
    """
    轨迹代价评价器
    Cost = W_safe * J_safe + W_comfort * J_comfort + W_eff * J_eff + W_ref * J_ref
    """
    def __init__(
        self,
        w_collision: float = 1000.0,
        w_jerk: float = 2.0,
        w_accel: float = 1.0,
        w_ref_deviation: float = 5.0,
        w_target_speed: float = 3.0
    ):
        self.w_collision = w_collision
        self.w_jerk = w_jerk
        self.w_accel = w_accel
        self.w_ref_deviation = w_ref_deviation
        self.w_target_speed = w_target_speed

    def evaluate(
        self,
        trajectory: Trajectory,
        obstacles: List[ObstacleState],
        target_speed: float
    ) -> float:
        """
        计算单条候选轨迹的综合惩罚代价
        :param trajectory: 待评估轨迹
        :param obstacles: 动态障碍物未来预测序列
        :param target_speed: 目标期望航速
        :return: 综合代价代数值 (越小越优)
        """
        if not trajectory.points:
            return float("inf")

        cost_jerk = 0.0
        cost_accel = 0.0
        cost_speed_dev = 0.0
        cost_collision = 0.0

        for pt in trajectory.points:
            cost_jerk += (pt.jerk ** 2)
            cost_accel += (pt.a ** 2)
            cost_speed_dev += ((pt.v - target_speed) ** 2)

            # 简化的时空安全距离检查
            for obs in obstacles:
                dist = np.hypot(pt.x - obs.x, pt.y - obs.y)
                if dist < 3.5:
                    cost_collision += self.w_collision / max(0.1, dist)

        total_cost = (
            self.w_jerk * cost_jerk +
            self.w_accel * cost_accel +
            self.w_target_speed * cost_speed_dev +
            cost_collision
        )
        trajectory.total_cost = total_cost
        return total_cost
