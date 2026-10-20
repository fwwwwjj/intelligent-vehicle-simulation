"""
LTV-MPC 轨迹跟踪控制器单元测试。

测试重点不是“和某个商用求解器逐位一致”，而是：
1. 无轨迹时安全制动；
2. 目标速度变化能反映到纵向指令；
3. 弯道曲率能产生正确符号的转向前馈；
4. 执行机构/速率约束不被突破；
5. 短闭环能收敛到目标速度。
"""

import unittest

import numpy as np

from src.common.state import Trajectory, TrajectoryPoint, VehicleState
from src.control.mpc_controller import MPCController


def _straight_trajectory(speed: float = 10.0, horizon: float = 5.0, dt: float = 0.1) -> Trajectory:
    points = []
    for k in range(int(round(horizon / dt)) + 1):
        t = k * dt
        points.append(
            TrajectoryPoint(
                x=speed * t,
                y=0.0,
                theta=0.0,
                kappa=0.0,
                v=speed,
                a=0.0,
                t=t,
                s=speed * t,
            )
        )
    return Trajectory(points=points)


def _curved_trajectory(speed: float = 10.0, kappa: float = 0.02) -> Trajectory:
    points = []
    for k in range(51):
        t = 0.1 * k
        s = speed * t
        points.append(
            TrajectoryPoint(
                x=s,
                y=0.5 * kappa * s * s,
                theta=kappa * s,
                kappa=kappa,
                v=speed,
                a=0.0,
                t=t,
                s=s,
            )
        )
    return Trajectory(points=points)


class TestMPCController(unittest.TestCase):

    def test_empty_trajectory_brakes(self):
        controller = MPCController()
        state = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=10.0, vx=10.0)
        command = controller.compute_control(state, Trajectory(points=[]), dt=0.05)
        self.assertLess(command.acceleration, 0.0)
        self.assertGreater(command.brake, 0.0)

    def test_lower_target_speed_commands_deceleration(self):
        controller = MPCController()
        state = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=15.0, vx=15.0)
        command = controller.compute_control(state, _straight_trajectory(10.0), dt=0.05)
        self.assertLess(command.acceleration, 0.0)
        self.assertGreaterEqual(command.acceleration, -controller.max_decel)
        self.assertTrue(np.isfinite(command.steering_angle))
        self.assertTrue(controller.last_solve_info.get("success", False))
        self.assertEqual(controller.last_solve_info.get("status"), "solved")

    def test_curve_produces_positive_steering(self):
        controller = MPCController()
        state = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=10.0, vx=10.0)
        command = controller.compute_control(state, _curved_trajectory(), dt=0.05)
        # 左转曲率为正，前轮转角应为正（ISO 8855 左正）
        self.assertGreater(command.steering_angle, 0.0)

    def test_actuator_and_rate_limits(self):
        controller = MPCController()
        state = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=12.0, vx=12.0)
        trajectory = _straight_trajectory(5.0)
        previous_steer = 0.0
        previous_accel = 0.0
        for _ in range(10):
            command = controller.compute_control(state, trajectory, dt=0.05)
            self.assertLessEqual(abs(command.steering_angle), controller.max_steer + 1e-9)
            self.assertLessEqual(command.acceleration, controller.max_accel + 1e-9)
            self.assertGreaterEqual(command.acceleration, -controller.max_decel - 1e-9)
            self.assertLessEqual(
                abs(command.steering_angle - previous_steer),
                controller.max_steer_rate * 0.05 + 1e-6,
            )
            self.assertLessEqual(
                abs(command.acceleration - previous_accel),
                controller.max_jerk * 0.05 + 1e-6,
            )
            previous_steer = command.steering_angle
            previous_accel = command.acceleration

    def test_closed_loop_speed_tracking(self):
        controller = MPCController()
        state = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=14.0, vx=14.0)
        dt = 0.05
        for _ in range(40):
            command = controller.compute_control(state, _straight_trajectory(10.0), dt)
            speed = max(0.0, state.vx + command.acceleration * dt)
            state = VehicleState(
                x=state.x + speed * dt,
                y=state.y,
                heading=state.heading,
                velocity=speed,
                vx=speed,
                acceleration=command.acceleration,
                steer_angle=command.steering_angle,
            )
        self.assertLess(state.vx, 14.0)
        self.assertGreater(state.vx, 8.0)


if __name__ == "__main__":
    unittest.main()
