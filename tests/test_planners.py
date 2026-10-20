"""
规划算法与多项式轨迹生成测试
"""

import unittest
import numpy as np
from src.planning.frenet_lattice import QuinticPolynomial, FrenetLatticePlanner
from src.common.state import VehicleState, ObstacleState, ObstacleType
from src.decision.base_decision import DecisionOutput, DrivingIntent
from src.common.frenet_frame import ReferencePoint


class TestPlanners(unittest.TestCase):

    @staticmethod
    def _straight_reference(length=100):
        return [ReferencePoint(x=float(s), y=0.0, theta=0.0, s=float(s)) for s in range(length + 1)]

    def test_adaptive_lateral_targets_include_nudge_sides(self):
        planner = FrenetLatticePlanner()
        ref = self._straight_reference()
        ego = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=8.0)
        obstacle = ObstacleState(
            id="static", obstacle_type=ObstacleType.STATIC, x=24.0, y=0.0,
            length=1.0, width=1.0, heading=0.0,
        )
        planned = planner.plan(
            ego, ref, [obstacle],
            DecisionOutput(DrivingIntent.YIELD, "lane", 4.0),
            plan_time_horizon=5.0, dt=0.1,
        )
        self.assertTrue(planned.points)
        self.assertTrue(any(abs(point.y) >= 0.8 for point in planned.points))

    def test_target_sampling_is_bounded(self):
        planner = FrenetLatticePlanner()
        ref = self._straight_reference()
        ego_frenet = planner._fallback_reference_line
        from src.common.frenet_frame import FrenetPoint
        targets = planner._adaptive_lateral_targets(
            FrenetPoint(s=0.0, l=0.0), ref, [], 0.0
        )
        self.assertLessEqual(len(targets), 7)
        self.assertTrue(all(-1.775 <= target <= 1.775 for target in targets))

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
