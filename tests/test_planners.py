"""
规划算法与多项式轨迹生成测试
"""

import unittest
import numpy as np
from src.planning.frenet_lattice import QuinticPolynomial, FrenetLatticePlanner
from src.common.state import VehicleState
from src.decision.base_decision import DecisionOutput, DrivingIntent


class TestPlanners(unittest.TestCase):

    def test_quintic_polynomial_boundary_conditions(self):
        poly = QuinticPolynomial(xs=0.0, vxs=0.0, axs=0.0, xe=10.0, vxe=0.0, axe=0.0, T=5.0)

        # 验证起点边界条件
        self.assertTrue(np.isclose(poly.calc_point(0.0), 0.0))
        self.assertTrue(np.isclose(poly.calc_first_derivative(0.0), 0.0))
        self.assertTrue(np.isclose(poly.calc_second_derivative(0.0), 0.0))

        # 验证终点边界条件
        self.assertTrue(np.isclose(poly.calc_point(5.0), 10.0))
        self.assertTrue(np.isclose(poly.calc_first_derivative(5.0), 0.0))
        self.assertTrue(np.isclose(poly.calc_second_derivative(5.0), 0.0))


if __name__ == "__main__":
    unittest.main()
