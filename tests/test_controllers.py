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


if __name__ == "__main__":
    unittest.main()
