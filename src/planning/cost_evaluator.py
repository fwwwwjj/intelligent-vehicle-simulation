"""
轨迹综合代价函数评估器。
多目标加权优化：安全性、乘客舒适性、道路中心线依从度、行驶效率及动力学物理边界。

与旧实现的关键区别：
1. 障碍物按轨迹点的时间戳做恒定速度投影，而不是用“当前时刻”的位置；
2. 使用 OBB + SAT 做真实碰撞检测，并把碰撞轨迹标记为 invalid；
3. 同时累计最小净距离，避免”贴着障碍物但不重叠“的轨迹拿低代价。
"""

from typing import List

import numpy as np

from src.common.state import Trajectory, ObstacleState
from src.common.geometry import BoundingBox2D, CollisionChecker


class TrajectoryCostEvaluator:
    """
    轨迹代价评价器
    Cost = W_safe * J_safe + W_comfort * J_comfort + W_eff * J_eff + W_ref * J_ref
    """

    def __init__(
        self,
        w_collision: float = 1_000_000.0,
        w_jerk: float = 2.0,
        w_lateral_jerk: float = 3.0,
        w_accel: float = 1.0,
        w_ref_deviation: float = 3.0,
        w_target_speed: float = 3.0,
        w_clearance: float = 50.0,
        w_lateral_accel: float = 1.0,
        ego_length: float = 4.8,
        ego_width: float = 1.9,
        collision_buffer: float = 0.2,
        max_accel: float = 3.5,
        max_decel: float = 7.0,
    ):
        self.w_collision = w_collision
        self.w_jerk = w_jerk
        self.w_lateral_jerk = w_lateral_jerk
        self.w_accel = w_accel
        self.w_ref_deviation = w_ref_deviation
        self.w_target_speed = w_target_speed
        self.w_clearance = w_clearance
        self.w_lateral_accel = w_lateral_accel
        self.ego_length = ego_length
        self.ego_width = ego_width
        self.collision_buffer = collision_buffer
        self.max_accel = max_accel
        self.max_decel = max_decel

    def evaluate(
        self,
        trajectory: Trajectory,
        obstacles: List[ObstacleState],
        target_speed: float,
        dt: float = 0.1,
    ) -> float:
        """
        计算单条候选轨迹的综合惩罚代价。
        :param trajectory: 待评估轨迹（点按 t 递增）
        :param obstacles: 动态障碍物列表（使用当前状态 + 恒定速度预测）
        :param target_speed: 目标期望速度 (m/s)
        :param dt: 轨迹采样间隔 (s)，用于把逐点代价近似为积分
        :return: 综合代价（越小越优）
        """
        if not trajectory.points:
            trajectory.valid = False
            return float("inf")

        dt = max(1e-3, float(dt))
        cost_jerk = 0.0
        cost_accel = 0.0
        cost_speed_dev = 0.0
        cost_collision = 0.0
        cost_clearance = 0.0
        cost_lateral_accel = 0.0
        cost_lateral_jerk = 0.0
        valid = True

        for pt in trajectory.points:
            # 动力学/运动学可行性
            if pt.v < -0.1 or pt.a > self.max_accel + 1e-6 or pt.a < -self.max_decel - 1e-6:
                valid = False
            if not np.isfinite(pt.x) or not np.isfinite(pt.y):
                valid = False
                break

            cost_jerk += (pt.jerk ** 2) * dt
            cost_accel += (pt.a ** 2) * dt
            cost_speed_dev += ((pt.v - target_speed) ** 2) * dt
            # 横向加速度惩罚：抑制“急打方向”的候选轨迹
            a_lat = (pt.v ** 2) * pt.kappa
            cost_lateral_accel += (a_lat ** 2) * dt

        # Lateral jerk estimated in Frenet path coordinates. Curvature-rate
        # captures steering transients and suppresses sharp S-shaped nudges.
        for i in range(1, len(trajectory.points)):
            prev, pt = trajectory.points[i - 1], trajectory.points[i]
            delta_t = max(1e-3, pt.t - prev.t)
            lat_acc_prev = prev.v ** 2 * prev.kappa
            lat_acc = pt.v ** 2 * pt.kappa
            lateral_jerk = (lat_acc - lat_acc_prev) / delta_t
            cost_lateral_jerk += lateral_jerk ** 2 * delta_t

            if not obstacles:
                continue

            ego_box = BoundingBox2D(pt.x, pt.y, self.ego_length, self.ego_width, pt.theta)
            for obs in obstacles:
                # 恒定速度预测到轨迹点时刻
                ox = obs.x + obs.vx * pt.t
                oy = obs.y + obs.vy * pt.t
                # 粗筛：中心距超过外接圆半径直接跳过，降低 SAT 调用量
                if np.hypot(pt.x - ox, pt.y - oy) > 0.5 * (
                    np.hypot(self.ego_length, self.ego_width)
                    + np.hypot(obs.length, obs.width)
                ) + 1.0:
                    continue

                obs_box = BoundingBox2D(ox, oy, obs.length, obs.width, obs.heading)
                if CollisionChecker.check_obb_collision(
                    ego_box, obs_box, buffer=self.collision_buffer
                ):
                    cost_collision += self.w_collision
                    valid = False
                    continue

                distance = CollisionChecker.min_distance_between_boxes(ego_box, obs_box)
                if distance < 3.0:
                    cost_clearance += self.w_clearance / max(0.2, distance) * dt

        total_cost = (
            cost_collision
            + self.w_jerk * cost_jerk
            + self.w_accel * cost_accel
            + self.w_target_speed * cost_speed_dev
            + self.w_lateral_accel * cost_lateral_accel
            + self.w_lateral_jerk * cost_lateral_jerk
            + cost_clearance
        )
        trajectory.total_cost = float(total_cost)
        trajectory.valid = valid
        return float(total_cost)
