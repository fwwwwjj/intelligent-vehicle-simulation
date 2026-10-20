"""
贝叶斯动态博弈决策器 (Bayesian Game-Theoretic Decision Maker) 单元测试。
检验对手意图在线估计、连续动作空间速度协同以及期望效用最大化决策机制。
"""

import unittest
from src.common.state import VehicleState, ObstacleState, ObstacleType
from src.decision.base_decision import DrivingIntent
from src.decision.game_theoretic import GameTheoreticDecisionMaker
from src.environment.road_network import Lane, RoadNetwork


def _build_test_road_network() -> RoadNetwork:
    network = RoadNetwork()
    # 主线 main_lane_0 (y=0)
    pts_main = [(float(x), 0.0) for x in range(0, 301, 2)]
    network.add_lane(Lane(lane_id="main_lane_0", width=3.75, center_points=pts_main))
    # 匝道 ramp_0 (y=-3.75)
    pts_ramp = [(float(x), -3.75) for x in range(0, 201, 2)]
    network.add_lane(Lane(lane_id="ramp_0", width=3.75, center_points=pts_ramp))
    for lane_id in network.lanes:
        network.generate_reference_line(lane_id, sample_step=1.0)
    return network


class TestGameTheoreticDecision(unittest.TestCase):

    def setUp(self):
        self.road_network = _build_test_road_network()
        self.decision_maker = GameTheoreticDecisionMaker(
            road_network=self.road_network,
            desired_speed=20.0,
            ego_length=4.8,
            ego_width=1.9,
        )

    def test_opponent_intent_inference_braking(self):
        """测试背景车减速时，礼让置信度 P(yield) 显著升高。"""
        obs = ObstacleState(
            id="obs_foll",
            obstacle_type=ObstacleType.CAR,
            x=20.0,
            y=0.0,
            vx=22.0,
        )
        # 背景车持续减速 (22 -> 21 -> 20 -> 18)
        self.decision_maker.estimate_opponent_intent(obs)
        obs.vx = 21.0
        self.decision_maker.estimate_opponent_intent(obs)
        obs.vx = 19.5
        self.decision_maker.estimate_opponent_intent(obs)
        obs.vx = 18.0
        p_yield = self.decision_maker.estimate_opponent_intent(obs)

        # 减速表明有礼让意图，P(yield) 应该超过 0.65
        self.assertGreater(p_yield, 0.65)

    def test_opponent_intent_inference_accelerating(self):
        """测试背景车加速冲抢时，礼让置信度 P(yield) 显著降低。"""
        obs = ObstacleState(
            id="obs_foll",
            obstacle_type=ObstacleType.CAR,
            x=20.0,
            y=0.0,
            vx=20.0,
        )
        # 背景车持续加速 (20 -> 22 -> 24 -> 26)
        self.decision_maker.estimate_opponent_intent(obs)
        obs.vx = 22.0
        self.decision_maker.estimate_opponent_intent(obs)
        obs.vx = 24.5
        self.decision_maker.estimate_opponent_intent(obs)
        obs.vx = 26.0
        p_yield = self.decision_maker.estimate_opponent_intent(obs)

        # 加速表明强行通过，P(yield) 应该低于 0.35
        self.assertLess(p_yield, 0.35)

    def test_accel_merge_selected_when_front_gap_is_large(self):
        """当前方间隙充裕而后方有距离时，决策器倾向于选择加速合流 (ACCEL_MERGE)。"""
        ego = VehicleState(x=50.0, y=-3.75, velocity=15.0, heading=0.0)
        # 主线前车在 120m 处 (净距很大)，后车在 30m 处 (间隙足够)
        lead = ObstacleState(id="lead", obstacle_type=ObstacleType.CAR, x=130.0, y=0.0, vx=18.0)
        follower = ObstacleState(id="foll", obstacle_type=ObstacleType.CAR, x=20.0, y=0.0, vx=17.0)

        decision = self.decision_maker.make_decision(ego, [lead, follower], current_lane_id="ramp_0")
        self.assertEqual(decision.intent, DrivingIntent.MERGE)
        self.assertEqual(decision.target_lane_id, "main_lane_0")
        # 期望速度应该高于当前自车速度 (主动提速合流)
        self.assertGreater(decision.target_speed, ego.velocity)
        self.assertIn("ACCEL_MERGE", decision.reasoning_log)

    def test_critical_rear_danger_forces_yield(self):
        """当后方车辆过近 (净距 < 3.5m) 且正在逼近时，强制让行保障安全。"""
        ego = VehicleState(x=50.0, y=-3.75, velocity=15.0, heading=0.0)
        # 后车紧贴 (x=48m, 车长 4.8m，净距已小于 3m) 且速度比自车快
        danger_follower = ObstacleState(
            id="foll_danger", obstacle_type=ObstacleType.CAR, x=48.0, y=0.0, vx=18.0
        )
        decision = self.decision_maker.make_decision(ego, [danger_follower], current_lane_id="ramp_0")
        self.assertEqual(decision.intent, DrivingIntent.YIELD)
        self.assertEqual(decision.target_lane_id, "ramp_0")
        self.assertIn("critical_rear=True", decision.reasoning_log)


if __name__ == "__main__":
    unittest.main()
