"""
IDM 参数标定与系统辨识工具链单元测试。
"""

import unittest
import numpy as np

from src.environment.obstacle_manager import IntelligentDriverModel
from src.environment.calibration import (
    IDMCalibrator,
    extract_following_pairs_from_arrays,
)


class TestIDMCalibration(unittest.TestCase):

    def test_extract_following_pairs(self):
        # 构造简单跟驰时序
        dt = 0.1
        t = np.arange(0, 5.0, dt)
        lead_x = 50.0 + 20.0 * t
        lead_v = np.full_like(t, 20.0)
        self_x = 10.0 + 19.0 * t
        self_v = np.full_like(t, 19.0)

        pairs = extract_following_pairs_from_arrays(
            time_series_self_x=self_x,
            time_series_self_v=self_v,
            time_series_lead_x=lead_x,
            time_series_lead_v=lead_v,
            dt=dt,
            veh_length_sum=9.6,
        )

        self.assertGreater(len(pairs), 0)
        self.assertIn("current_speed", pairs[0])
        self.assertIn("observed_accel", pairs[0])
        self.assertAlmostEqual(pairs[0]["current_speed"], 19.0)

    def test_calibrate_idm_reduces_rmse(self):
        # 使用真实真实已知 IDM 生成合成跟驰数据，再检验辨识器能否降低误差
        ground_truth_idm = IntelligentDriverModel(
            v0=25.0, s0=3.0, T=1.5, a_max=2.0, b_desired=2.0
        )
        samples = []
        for v in [15.0, 20.0, 24.0]:
            for gap in [15.0, 30.0, 50.0]:
                for lead_v in [18.0, 22.0, 25.0]:
                    accel = ground_truth_idm.compute_acceleration(v, lead_v, gap)
                    samples.append({
                        "current_speed": v,
                        "lead_speed": lead_v,
                        "gap": gap,
                        "observed_accel": accel,
                    })

        calibrator = IDMCalibrator()
        # 故意给一个偏差较大的初值
        initial_guess = {
            "v0": 35.0,
            "s0": 6.0,
            "T": 2.5,
            "a_max": 3.5,
            "b_desired": 3.5,
        }
        res = calibrator.calibrate(samples, initial_params=initial_guess)

        self.assertTrue(res["success"])
        self.assertLess(res["final_rmse"], res["initial_rmse"])
        self.assertLess(res["final_rmse"], 0.2)  # 拟合误差应显著降低至接近零


if __name__ == "__main__":
    unittest.main()
