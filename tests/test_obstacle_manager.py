"""动态障碍物跟驰与短期预测测试。"""

import unittest

from src.common.state import ObstacleState, ObstacleType
from src.environment.obstacle_manager import IntelligentDriverModel, ObstacleManager


def _car(obstacle_id: str, x: float, speed: float) -> ObstacleState:
    return ObstacleState(
        id=obstacle_id,
        obstacle_type=ObstacleType.CAR,
        x=x,
        y=0.0,
        length=4.8,
        width=1.9,
        heading=0.0,
        vx=speed,
    )


class TestObstaclePrediction(unittest.TestCase):
    def test_scheduled_lane_change_starts_at_configured_time(self):
        manager = ObstacleManager()
        car = _car("scripted", 0.0, 10.0)
        car.y = 3.75
        manager.add_obstacle(car)
        manager.schedule_lane_change("scripted", target_y=0.0, start_time=0.5, duration=1.0)

        for _ in range(5):
            manager.step(0.1)
        self.assertAlmostEqual(car.y, 3.75)
        for _ in range(10):
            manager.step(0.1)
        self.assertAlmostEqual(car.y, 0.0, places=6)
        self.assertAlmostEqual(car.vy, 0.0, places=6)

    def test_idm_acceleration_is_used_in_prediction(self):
        manager = ObstacleManager()
        car = _car("idm", 0.0, 10.0)
        manager.add_obstacle(car, IntelligentDriverModel(v0=20.0))

        manager.step(0.1)

        point = next(p for p in car.predicted_trajectory if abs(p.t - 1.0) < 1e-9)
        constant_speed_x = car.x + car.vx * point.t
        self.assertGreater(point.x, constant_speed_x)
        self.assertGreater(point.v, car.vx)

    def test_braking_prediction_stops_at_finite_distance(self):
        manager = ObstacleManager()
        follower = _car("follower", 0.0, 20.0)
        stopped_lead = _car("lead", 20.0, 0.0)
        manager.add_obstacle(follower, IntelligentDriverModel(v0=30.0))
        manager.add_obstacle(stopped_lead)

        manager.step(0.1)

        prediction = follower.predicted_trajectory
        self.assertLess(prediction[0].x - follower.x, follower.vx * prediction[0].t)
        self.assertAlmostEqual(prediction[-1].x, prediction[-2].x)
        self.assertGreaterEqual(prediction[-1].v, 0.0)

    def test_mobil_lane_change_execution(self):
        # 慢前车在原车道 (y=0)，跟驰车在启用 MOBIL 后换道至目标车道 (y=3.75)
        manager = ObstacleManager(enable_mobil=True, lane_width=3.75)
        slow_lead = _car("slow_lead", 30.0, 5.0)
        follower = _car("follower", 10.0, 15.0)
        manager.add_obstacle(slow_lead)
        manager.add_obstacle(follower, IntelligentDriverModel(v0=25.0, s0=3.0, T=1.5, a_max=2.0, b_desired=2.0))

        # 演化 3 秒，检查是否成功发起并完成向 y=3.75 换道
        for _ in range(60):
            manager.step(0.05)

        self.assertGreater(follower.y, 2.0)
        self.assertAlmostEqual(follower.y, 3.75, delta=0.5)


class TestMobilLaneBounds(unittest.TestCase):
    """MOBIL 必须只在真实车道之间换道，不能靠 y ± lane_width 推算到路外。

    回归背景：2 车道公路（y=0 / 3.75）上，身处 y=3.75 的车向上换道的候选
    曾被算成 y=7.5，即"越出道路"，且换道完成后会再次评估，可连续越界。
    """

    def test_candidate_centers_exclude_offroad_lane(self):
        manager = ObstacleManager(enable_mobil=True, lane_width=3.75, lane_centers=[0.0, 3.75])
        car = _car("car", 0.0, 6.0)
        car.y = 3.75
        self.assertEqual(manager._candidate_lane_centers(car), [0.0])

    def test_mobil_never_targets_nonexistent_lane(self):
        manager = ObstacleManager(enable_mobil=True, lane_width=3.75, lane_centers=[0.0, 3.75])
        car = _car("car", 0.0, 6.0)
        car.y = 3.75
        blocker = _car("blocker", 12.0, 0.0)
        blocker.y = 3.75
        manager.add_obstacle(car, IntelligentDriverModel(v0=25.0))
        manager.add_obstacle(blocker)

        # current_accel 取强负值，确保激励准则成立、进入"选目标车道"分支
        target_y = manager._evaluate_mobil_lane_change("car", car, -6.0)
        self.assertIn(target_y, (None, 0.0))
        self.assertNotEqual(target_y, 7.5)

    def test_no_lane_table_falls_back_without_crash(self):
        manager = ObstacleManager(enable_mobil=True, lane_width=3.75)
        car = _car("car", 0.0, 6.0)
        # 无车道表时退回旧的 ±lane_width 推算，且仍过滤 y<0 一侧
        self.assertEqual(manager._candidate_lane_centers(car), [3.75])


if __name__ == "__main__":
    unittest.main()
