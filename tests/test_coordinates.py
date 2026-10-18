"""
坐标转换与几何基础单元测试
"""

import unittest
import numpy as np
from src.common.frenet_frame import ReferencePoint, FrenetPoint, FrenetConverter
from src.common.geometry import BoundingBox2D, CollisionChecker
from src.common.state import VehicleState


class TestCoordinates(unittest.TestCase):

    def test_frenet_converter_basic(self):
        ref_pt = ReferencePoint(x=10.0, y=0.0, theta=0.0, kappa=0.0, s=10.0)
        f_pt = FrenetPoint(s=10.0, s_d=15.0, s_dd=0.0, l=1.5, l_prime=0.0)
        c_pt = FrenetConverter.frenet_to_cartesian(f_pt, ref_pt)

        self.assertTrue(np.isclose(c_pt.x, 10.0))
        self.assertTrue(np.isclose(c_pt.y, 1.5))
        self.assertTrue(np.isclose(c_pt.theta, 0.0))

    def test_cartesian_to_frenet_projection(self):
        ref_line = [
            ReferencePoint(x=float(x), y=0.0, theta=0.0, kappa=0.0, s=float(x))
            for x in range(0, 100, 5)
        ]
        state = VehicleState(x=20.0, y=1.75, heading=0.0, velocity=15.0)
        frenet_pt = FrenetConverter.cartesian_to_frenet(state, ref_line)

        self.assertTrue(np.isclose(frenet_pt.s, 20.0, atol=0.5))
        self.assertTrue(np.isclose(frenet_pt.l, 1.75, atol=0.1))
        self.assertTrue(np.isclose(frenet_pt.s_d, 15.0, atol=0.1))

    def test_obb_bounding_box_corners(self):
        box = BoundingBox2D(center_x=0.0, center_y=0.0, length=4.0, width=2.0, heading=0.0)
        corners = box.get_corners()
        self.assertEqual(len(corners), 4)
        self.assertTrue(np.isclose(corners[0][0], 2.0))
        self.assertTrue(np.isclose(corners[0][1], 1.0))

    def test_sat_collision_detection(self):
        # 1. 明确重叠的两个包围盒
        box1 = BoundingBox2D(center_x=0.0, center_y=0.0, length=4.8, width=1.9, heading=0.0)
        box2 = BoundingBox2D(center_x=2.0, center_y=0.5, length=4.8, width=1.9, heading=0.0)
        self.assertTrue(CollisionChecker.check_obb_collision(box1, box2))

        # 2. 距离较远无重叠的两个包围盒
        box3 = BoundingBox2D(center_x=10.0, center_y=0.0, length=4.8, width=1.9, heading=0.0)
        self.assertFalse(CollisionChecker.check_obb_collision(box1, box3))

        # 3. 错位车道 (横向间距 3.75m)
        box_adjacent = BoundingBox2D(center_x=0.0, center_y=3.75, length=4.8, width=1.9, heading=0.0)
        self.assertFalse(CollisionChecker.check_obb_collision(box1, box_adjacent))


if __name__ == "__main__":
    unittest.main()
