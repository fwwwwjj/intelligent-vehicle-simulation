"""
车载感知数字模型的回归测试。

感知层是整个闭环里**唯一的随机源**（其余模块均为确定性计算），因此这里锁定三件事：
1. FOV 与最大测距的过滤边界（看到 / 看不到的判定）；
2. 同一 seed 必须逐位可复现（这是 30-seed 基准可复现的前提）；
3. 噪声统计量级与配置一致（均值≈0、标准差≈配置值），避免"噪声写错量级"导致
   闭环指标悄悄漂移。

全部为纯计算，不启动仿真，耗时可忽略。
"""

import unittest

import numpy as np

from src.common.state import ObstacleState, ObstacleType, VehicleState
from src.vehicle_model.sensor_model import SensorModel


def _ego(x: float = 0.0, y: float = 0.0, heading: float = 0.0) -> VehicleState:
    return VehicleState(x=x, y=y, heading=heading, velocity=10.0, vx=10.0)


def _car(obstacle_id: str, x: float, y: float = 0.0) -> ObstacleState:
    return ObstacleState(
        id=obstacle_id,
        obstacle_type=ObstacleType.CAR,
        x=x,
        y=y,
        length=4.8,
        width=1.9,
        heading=0.0,
        vx=10.0,
    )


class TestSensorFovAndRange(unittest.TestCase):
    def test_obstacle_behind_is_filtered_by_front_fov(self):
        sensor = SensorModel(max_range=80.0, fov_deg=90.0, pos_noise_std=0.0, vel_noise_std=0.0)
        ahead = _car("ahead", 30.0, 0.0)
        behind = _car("behind", -30.0, 0.0)

        seen = sensor.filter_perceived_obstacles(_ego(), [ahead, behind])
        self.assertEqual([obs.id for obs in seen], ["ahead"])

    def test_360_fov_sees_both_sides(self):
        sensor = SensorModel(max_range=80.0, fov_deg=360.0, pos_noise_std=0.0, vel_noise_std=0.0)
        ahead = _car("ahead", 30.0, 0.0)
        behind = _car("behind", -30.0, 0.0)

        seen = sensor.filter_perceived_obstacles(_ego(), [ahead, behind])
        self.assertEqual(sorted(obs.id for obs in seen), ["ahead", "behind"])

    def test_obstacle_beyond_max_range_is_filtered(self):
        sensor = SensorModel(max_range=80.0, fov_deg=360.0, pos_noise_std=0.0, vel_noise_std=0.0)
        near = _car("near", 79.0, 0.0)
        far = _car("far", 81.0, 0.0)

        seen = sensor.filter_perceived_obstacles(_ego(), [near, far])
        self.assertEqual([obs.id for obs in seen], ["near"])

    def test_fov_rotates_with_ego_heading(self):
        # 自车朝向 +90°，正前方应是 +y 方向；此时 x 轴上的车进入侧方而应被 90° FOV 排除
        sensor = SensorModel(max_range=80.0, fov_deg=90.0, pos_noise_std=0.0, vel_noise_std=0.0)
        lateral = _car("lateral", 30.0, 0.0)
        forward = _car("forward", 0.0, 30.0)

        seen = sensor.filter_perceived_obstacles(
            _ego(heading=np.pi / 2.0), [lateral, forward]
        )
        self.assertEqual([obs.id for obs in seen], ["forward"])


class TestSensorNoise(unittest.TestCase):
    def test_same_seed_is_bit_exact(self):
        def run(seed: int):
            sensor = SensorModel(
                max_range=80.0, fov_deg=360.0,
                pos_noise_std=0.05, vel_noise_std=0.10,
                rng=np.random.default_rng(seed),
            )
            out = []
            for _ in range(20):
                obs = sensor.filter_perceived_obstacles(_ego(), [_car("c", 25.0, 1.0)])[0]
                out.append((obs.x, obs.y, obs.vx, obs.vy))
            return np.asarray(out)

        self.assertTrue(np.array_equal(run(42), run(42)))
        self.assertFalse(np.array_equal(run(42), run(43)))

    def test_noise_magnitude_matches_config(self):
        sensor = SensorModel(
            max_range=200.0, fov_deg=360.0,
            pos_noise_std=0.05, vel_noise_std=0.10,
            rng=np.random.default_rng(7),
        )
        xs, vs = [], []
        for _ in range(2000):
            obs = sensor.filter_perceived_obstacles(_ego(), [_car("c", 50.0, 0.0)])[0]
            xs.append(obs.x - 50.0)
            vs.append(obs.vx - 10.0)

        self.assertAlmostEqual(float(np.mean(xs)), 0.0, delta=0.01)
        self.assertAlmostEqual(float(np.std(xs)), 0.05, delta=0.01)
        self.assertAlmostEqual(float(np.mean(vs)), 0.0, delta=0.02)
        self.assertAlmostEqual(float(np.std(vs)), 0.10, delta=0.02)

    def test_zero_noise_returns_ground_truth(self):
        sensor = SensorModel(max_range=80.0, fov_deg=360.0, pos_noise_std=0.0, vel_noise_std=0.0)
        obs = sensor.filter_perceived_obstacles(_ego(), [_car("c", 42.0, 3.75)])[0]
        self.assertEqual((obs.x, obs.y, obs.vx), (42.0, 3.75, 10.0))


if __name__ == "__main__":
    unittest.main()
