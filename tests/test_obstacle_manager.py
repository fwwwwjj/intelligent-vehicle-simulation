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


if __name__ == "__main__":
    unittest.main()
