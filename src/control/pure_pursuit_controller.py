"""
经典纯跟踪横向控制 + PID 纵向控制基线 (Pure Pursuit Controller)。

作为运动控制消融实验中的经典基线：横向采用几何纯跟踪 (Pure Pursuit)，
沿规划轨迹按固定前视时间取预瞄点，由车辆运动学关系
``delta = arctan(2 * L * sin(alpha) / Ld)`` 计算前轮转角；纵向复用
:class:`~src.control.pid_controller.LongitudinalPIDController` 跟踪轨迹末端
目标速度，并叠加轨迹前馈加速度。

与 LQR/MPC 相比，该控制器不含动力学模型、不做优化求解，只做几何跟踪，
用于量化先进控制器的贡献。
"""

from typing import List

import numpy as np

from src.common.state import VehicleState, Trajectory, TrajectoryPoint
from src.control.base_controller import BaseController, ControlCommand
from src.control.pid_controller import LongitudinalPIDController


class PurePursuitController(BaseController):
    """
    几何纯跟踪 (Pure Pursuit) 控制器。

    - 横向：前视时间 ``lookahead_time`` 秒处的轨迹点作为预瞄点，按运动学
      自行车模型计算期望前轮转角，并做转角速率限制与幅值限幅；
    - 纵向：PID 跟踪轨迹末端目标速度 + 轨迹前馈加速度，做加加速度限制与
      加速度限幅。
    """

    def __init__(
        self,
        wheelbase: float = 2.85,
        max_steer: float = 0.65,
        max_steer_rate: float = 0.50,
        max_accel: float = 3.5,
        max_decel: float = 7.0,
        max_jerk: float = 4.0,
        lookahead_time: float = 0.8,
        min_lookahead: float = 2.0,
        speed_kp: float = 0.8,
        speed_ki: float = 0.1,
    ):
        self.wheelbase = float(wheelbase)
        self.max_steer = float(max_steer)
        self.max_steer_rate = float(max_steer_rate)
        self.max_accel = float(max_accel)
        self.max_decel = float(max_decel)
        self.max_jerk = float(max_jerk)
        self.lookahead_time = float(lookahead_time)
        self.min_lookahead = float(min_lookahead)
        self.speed_kp = float(speed_kp)
        self.speed_ki = float(speed_ki)

        self.longitudinal_controller = LongitudinalPIDController(
            kp=self.speed_kp,
            ki=self.speed_ki,
            kd=0.0,
            max_accel=self.max_accel,
            max_decel=self.max_decel,
        )

        self._prev_steer = 0.0
        self._prev_accel = 0.0

    # ------------------------------------------------------------------
    # 轨迹时间索引
    # ------------------------------------------------------------------
    @staticmethod
    def _point_at_time(trajectory: Trajectory, t_query: float) -> TrajectoryPoint:
        """在按 ``t`` 递增的轨迹上做线性插值，返回 ``t_query`` 处的轨迹点。"""
        points: List[TrajectoryPoint] = trajectory.points
        if len(points) == 1:
            return points[0]

        times = [p.t for p in points]
        if t_query <= times[0]:
            return points[0]
        if t_query >= times[-1]:
            return points[-1]

        # 二分查找包含 t_query 的相邻轨迹段
        lo, hi = 0, len(times) - 1
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if times[mid] <= t_query:
                lo = mid
            else:
                hi = mid

        p0, p1 = points[lo], points[hi]
        span = times[hi] - times[lo]
        if span <= 1e-9:
            return p0
        ratio = float((t_query - times[lo]) / span)

        dtheta = float(p1.theta - p0.theta)
        dtheta = float(np.arctan2(np.sin(dtheta), np.cos(dtheta)))
        theta = float(p0.theta + ratio * dtheta)
        theta = float(np.arctan2(np.sin(theta), np.cos(theta)))

        return TrajectoryPoint(
            x=float(p0.x + ratio * (p1.x - p0.x)),
            y=float(p0.y + ratio * (p1.y - p0.y)),
            theta=theta,
            kappa=float(p0.kappa + ratio * (p1.kappa - p0.kappa)),
            dkappa=float(p0.dkappa + ratio * (p1.dkappa - p0.dkappa)),
            v=float(p0.v + ratio * (p1.v - p0.v)),
            a=float(p0.a + ratio * (p1.a - p0.a)),
            jerk=float(p0.jerk + ratio * (p1.jerk - p0.jerk)),
            t=float(t_query),
            s=float(p0.s + ratio * (p1.s - p0.s)),
        )

    # ------------------------------------------------------------------
    # 主控制律
    # ------------------------------------------------------------------
    def compute_control(
        self,
        current_state: VehicleState,
        target_trajectory: Trajectory,
        dt: float,
    ) -> ControlCommand:
        """
        计算前轮转角与纵向加速度指令。

        :param current_state: 当前车辆状态
        :param target_trajectory: 规划层下发的时空轨迹
        :param dt: 控制周期 (s)
        :return: 控制指令
        """
        step_dt = max(float(dt), 1e-3)

        if not target_trajectory.points:
            self.longitudinal_controller.reset()
            self._prev_steer = 0.0
            self._prev_accel = -2.0
            return ControlCommand(acceleration=-2.0, brake=0.4)

        last_point = target_trajectory.points[-1]
        t_lookahead = max(0.0, min(self.lookahead_time, float(last_point.t)))
        target_pt = self._point_at_time(target_trajectory, t_lookahead)

        # 1. 预瞄点转换到车身坐标系
        dx_world = float(target_pt.x - current_state.x)
        dy_world = float(target_pt.y - current_state.y)
        cos_h = float(np.cos(current_state.heading))
        sin_h = float(np.sin(current_state.heading))
        dx_body = cos_h * dx_world + sin_h * dy_world
        dy_body = -sin_h * dx_world + cos_h * dy_world

        # 2. 纯跟踪几何关系：Ld 至少取 min_lookahead，避免预瞄点过近导致转角奇异。
        lookahead_dist = max(float(np.hypot(dx_body, dy_body)), self.min_lookahead)
        alpha = float(np.arctan2(dy_body, dx_body))
        steer_target = float(
            np.arctan2(2.0 * self.wheelbase * np.sin(alpha), lookahead_dist)
        )

        # 3. 转角速率限制 + 幅值限幅
        max_delta = self.max_steer_rate * step_dt
        steer_cmd = float(
            np.clip(
                steer_target,
                self._prev_steer - max_delta,
                self._prev_steer + max_delta,
            )
        )
        steer_cmd = float(np.clip(steer_cmd, -self.max_steer, self.max_steer))
        steering_rate = (steer_cmd - self._prev_steer) / step_dt
        self._prev_steer = steer_cmd

        # 4. 纵向：PID 跟踪轨迹末端目标速度 + 轨迹前馈加速度
        v_ref = max(0.0, float(last_point.v))
        a_ff = float(last_point.a)
        accel_cmd = float(
            self.longitudinal_controller.compute(
                target_speed=v_ref,
                current_speed=float(current_state.velocity),
                dt=step_dt,
            )
        ) + a_ff

        # 5. 加加速度限制与加速度限幅
        max_delta_a = self.max_jerk * step_dt
        accel_cmd = float(
            np.clip(
                accel_cmd,
                self._prev_accel - max_delta_a,
                self._prev_accel + max_delta_a,
            )
        )
        accel_cmd = float(np.clip(accel_cmd, -self.max_decel, self.max_accel))
        self._prev_accel = accel_cmd

        throttle = float(np.clip(max(0.0, accel_cmd) / max(1e-6, self.max_accel), 0.0, 1.0))
        brake = float(np.clip(max(0.0, -accel_cmd) / max(1e-6, self.max_decel), 0.0, 1.0))

        return ControlCommand(
            steering_angle=steer_cmd,
            acceleration=accel_cmd,
            throttle=throttle,
            brake=brake,
            steering_rate=float(steering_rate),
        )

    def reset(self) -> None:
        """重置控制器内部状态（新场景开始时调用）。"""
        self._prev_steer = 0.0
        self._prev_accel = 0.0
        self.longitudinal_controller.reset()


__all__ = ["PurePursuitController"]
