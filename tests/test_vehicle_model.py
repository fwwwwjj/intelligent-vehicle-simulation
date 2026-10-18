"""
车辆运动学与动力学机理模型测试
"""

import unittest
import numpy as np
from src.common.state import VehicleState
from src.vehicle_model.kinematics import KinematicBicycleModel
from src.vehicle_model.dynamics import DynamicBicycleModel


class TestVehicleModel(unittest.TestCase):

    def test_kinematic_bicycle_straight(self):
        model = KinematicBicycleModel()
        state = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=10.0)
        next_state = model.step(state, accel_cmd=0.0, steer_cmd=0.0, dt=0.1)

        self.assertTrue(np.isclose(next_state.x, 1.0))
        self.assertTrue(np.isclose(next_state.y, 0.0))
        self.assertTrue(np.isclose(next_state.heading, 0.0))
        self.assertTrue(np.isclose(next_state.velocity, 10.0))

    def test_dynamic_bicycle_acceleration(self):
        model = DynamicBicycleModel()
        state = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=10.0, vx=10.0)
        next_state = model.step(state, accel_cmd=2.0, steer_cmd=0.0, dt=0.1)

        self.assertGreater(next_state.vx, state.vx)
        self.assertGreater(next_state.x, state.x)


if __name__ == "__main__":
    unittest.main()
