"""
Pacejka 魔术公式与 2-DOF 动力学主回路测试。

覆盖：
1. 零侧偏零侧向力、峰值/饱和量级；
2. 纵向载荷转移；
3. ``tire_model="pacejka"`` 的动力学步进稳定且前进；
4. 线性轮胎分支仍保持原有行为。
"""

import unittest

import numpy as np

from src.common.state import VehicleState
from src.vehicle_model.dynamics import DynamicBicycleModel
from src.vehicle_model.tire import PacejkaTireModel


class TestPacejkaTire(unittest.TestCase):

    def test_zero_slip_zero_force(self):
        tire = PacejkaTireModel()
        self.assertAlmostEqual(tire.lateral_force(0.0, 4000.0), 0.0)

    def test_force_saturates_near_peak(self):
        tire = PacejkaTireModel(mu=0.9)
        fz = 4000.0
        peak = tire.lateral_force(0.12, fz)
        extreme = tire.lateral_force(1.2, fz)
        self.assertGreater(abs(peak), 0.5 * 0.9 * fz)
        self.assertLess(abs(extreme), 1.05 * 0.9 * fz)
        self.assertGreater(abs(extreme), 0.3 * 0.9 * fz)

    def test_combined_force_respects_friction_ellipse(self):
        tire = PacejkaTireModel(mu=0.9)
        fz = 4000.0
        pure = tire.lateral_force(0.12, fz)
        combined = tire.combined_lateral_force(0.12, fz, longitudinal_force=0.8 * 0.9 * fz)
        self.assertLess(abs(combined), abs(pure))
        self.assertLessEqual(abs(combined), 0.9 * fz + 1e-6)

    def test_normal_load_transfer(self):
        model = DynamicBicycleModel(tire_model="pacejka")
        front_accel, rear_accel = model.normal_loads(2.0)
        front_brake, rear_brake = model.normal_loads(-4.0)
        self.assertLess(front_accel, front_brake)
        self.assertGreater(rear_accel, rear_brake)
        self.assertGreater(front_accel, 0.0)
        self.assertGreater(rear_accel, 0.0)

    def test_pacejka_dynamic_step_is_stable_and_moves_forward(self):
        model = DynamicBicycleModel(tire_model="pacejka", friction_coefficient=0.9)
        state = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=15.0, vx=15.0)
        next_state = model.step(state, accel_cmd=1.0, steer_cmd=0.01, dt=0.05)
        self.assertGreater(next_state.vx, state.vx)
        self.assertGreater(next_state.x, state.x)
        self.assertTrue(np.isfinite(next_state.vy))
        self.assertTrue(np.isfinite(next_state.yaw_rate))

    def test_linear_branch_unchanged(self):
        model = DynamicBicycleModel(tire_model="linear")
        state = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=10.0, vx=10.0)
        next_state = model.step(state, accel_cmd=2.0, steer_cmd=0.0, dt=0.1)
        self.assertAlmostEqual(next_state.x, 1.02, places=6)
        self.assertAlmostEqual(next_state.y, 0.0, places=6)
        self.assertAlmostEqual(next_state.vx, 10.2, places=6)


if __name__ == "__main__":
    unittest.main()
