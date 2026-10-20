"""
控制算法单元测试 (LQR / PID)
"""

import unittest
import numpy as np
from src.control.pid_controller import LongitudinalPIDController
from src.control.lqr_controller import LQRController
from src.common.state import VehicleState, Trajectory, TrajectoryPoint


class TestControllers(unittest.TestCase):

    def test_pid_speed_control(self):
        pid = LongitudinalPIDController(kp=1.0, ki=0.1, kd=0.01)
        accel = pid.compute(target_speed=20.0, current_speed=15.0, dt=0.1)
        self.assertGreater(accel, 0.0)

        accel_brake = pid.compute(target_speed=10.0, current_speed=15.0, dt=0.1)
        self.assertLess(accel_brake, 0.0)

    def test_lqr_controller_execution(self):
        lqr = LQRController()
        state = VehicleState(x=0.0, y=0.5, heading=0.0, velocity=15.0, vx=15.0)
        traj = Trajectory(points=[TrajectoryPoint(x=0.0, y=0.0, theta=0.0, v=15.0)])
        cmd = lqr.compute_control(state, traj, dt=0.05)
        self.assertIsInstance(cmd.steering_angle, float)

    def test_lqr_understeer_gradient_and_feedforward(self):
        # 验证车辆稳态不足转向梯度 Kv 计算及其高低速前馈差异
        lqr = LQRController()
        self.assertGreater(lqr.understeer_gradient, 0.0)

        # 弯道轨迹 (kappa = 0.03)
        pts = [
            TrajectoryPoint(x=s, y=0.0, theta=0.03 * s, kappa=0.03, v=15.0, t=s / 15.0)
            for s in np.linspace(0, 30, 20)
        ]
        traj = Trajectory(points=pts)

        # 低速 5 m/s 与高速 20 m/s 前馈转角比较
        state_low = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=5.0, vx=5.0)
        lqr.reset()
        cmd_low = lqr.compute_control(state_low, traj, dt=0.05)

        state_high = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=20.0, vx=20.0)
        lqr.reset()
        cmd_high = lqr.compute_control(state_high, traj, dt=0.05)

        # 带有 Kv * v^2 补偿的前馈，在高速下提供的转向角补偿应大于低速
        self.assertGreater(cmd_high.steering_angle, cmd_low.steering_angle)

    def test_lqr_lateral_adhesion_protection(self):
        lqr = LQRController(max_steer_rate=1.0, friction_coefficient=0.8)
        pts = [
            TrajectoryPoint(x=s, y=0.0, theta=0.02 * s, kappa=0.02, v=15.0, t=s / 15.0)
            for s in np.linspace(0, 30, 20)
        ]
        traj = Trajectory(points=pts)

        # 正常状态
        state_calm = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=15.0, vx=15.0, ay=0.0, yaw_rate=0.0)
        lqr.reset()
        cmd_calm = lqr.compute_control(state_calm, traj, dt=0.05)

        # 极端高侧偏工况 (接近 0.8 * 9.81 = 7.848 m/s^2)
        state_aggr = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=15.0, vx=15.0, ay=7.0, yaw_rate=0.45)
        lqr.reset()
        cmd_aggr = lqr.compute_control(state_aggr, traj, dt=0.05)

        # 附着软保护触发，转向速率和单步转向增量被平滑约束
        self.assertLess(abs(cmd_aggr.steering_rate), abs(cmd_calm.steering_rate) + 0.1)


if __name__ == "__main__":
    unittest.main()
