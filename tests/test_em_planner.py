"""
EM/QP 规划器单元测试。

覆盖：
1. 无碰撞直线场景的速度/轨迹完整性；
2. 前方静止车辆场景的速度 QP 停车与终端速度保护；
3. 弯道参考线下的路径 QP 依从度；
4. 装配层能按 ``planner="em"`` 构建 EMPlanner。
"""

import unittest

import numpy as np

from src.common.state import VehicleState
from src.decision.base_decision import DecisionOutput, DrivingIntent
from src.environment.road_network import Lane, RoadNetwork
from src.environment.scenario_loader import (
    build_emergency_avoidance_scenario,
    build_highway_merge_scenario,
)
from src.planning.em_planner import EMPlanner
from src.runner import build_planner


def _straight_network(length: float = 200.0) -> RoadNetwork:
    network = RoadNetwork()
    network.add_lane(
        Lane(
            lane_id="main_lane_0",
            center_points=[(float(x), 0.0) for x in range(0, int(length) + 1, 2)],
        )
    )
    network.generate_reference_line("main_lane_0", sample_step=1.0)
    return network


class TestEMPlanner(unittest.TestCase):

    def test_straight_cruise_trajectory_is_complete(self):
        network = _straight_network()
        reference = network.get_reference_line("main_lane_0")
        ego = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=12.0, vx=12.0)
        decision = DecisionOutput(
            intent=DrivingIntent.CRUISE,
            target_lane_id="main_lane_0",
            target_speed=12.0,
        )
        planner = EMPlanner()
        trajectory = planner.plan(ego, reference, [], decision, plan_time_horizon=5.0, dt=0.1)

        self.assertTrue(trajectory.points)
        self.assertTrue(trajectory.valid)
        self.assertTrue(planner.last_path_result.success)
        self.assertTrue(planner.last_speed_result.success)
        times = [p.t for p in trajectory.points]
        stations = [p.s for p in trajectory.points]
        self.assertTrue(np.all(np.diff(times) > 0.0))
        self.assertTrue(np.all(np.diff(stations) >= -1e-6))
        self.assertAlmostEqual(trajectory.points[-1].v, 12.0, delta=1.0)
        self.assertGreater(len(trajectory.points), 10)

    def test_static_obstacle_yields_terminal_speed_guard(self):
        scenario = build_emergency_avoidance_scenario()
        reference = scenario.road_network.get_reference_line("main_lane_0")
        obstacles = scenario.obstacle_manager.get_all_obstacles()
        decision = DecisionOutput(
            intent=DrivingIntent.CRUISE,
            target_lane_id="main_lane_0",
            target_speed=22.22,
        )
        planner = EMPlanner()
        trajectory = planner.plan(
            scenario.ego_state,
            reference,
            obstacles,
            decision,
            plan_time_horizon=5.0,
            dt=0.1,
        )

        self.assertTrue(planner.last_speed_result.success)
        self.assertTrue(trajectory.valid)
        # 静止障碍物位于 s=90 m；自车中心应停在其前方，且终端速度被压到很低
        self.assertLess(trajectory.points[-1].s, 90.0)
        self.assertLess(trajectory.points[-1].v, 2.0)
        self.assertGreaterEqual(
            min(p.a for p in trajectory.points), -planner.max_decel - 1e-6
        )

    def test_curved_reference_is_tracked(self):
        radius = 100.0
        network = RoadNetwork()
        points = [
            (radius * np.sin(theta), radius * (1.0 - np.cos(theta)))
            for theta in np.linspace(0.0, 1.3, 120)
        ]
        network.add_lane(
            Lane(lane_id="arc", center_points=[(float(x), float(y)) for x, y in points])
        )
        reference = network.generate_reference_line("arc", sample_step=0.5)
        ego = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=15.0, vx=15.0)
        decision = DecisionOutput(
            intent=DrivingIntent.CRUISE,
            target_lane_id="arc",
            target_speed=15.0,
        )
        planner = EMPlanner()
        trajectory = planner.plan(ego, reference, [], decision, plan_time_horizon=5.0, dt=0.1)

        last = trajectory.points[-1]
        expected_y = radius - np.sqrt(max(0.0, radius * radius - last.x * last.x))
        self.assertLess(abs(last.y - expected_y), 0.5)
        self.assertGreater(max(abs(p.kappa) for p in trajectory.points), 1e-4)

    def test_runner_builds_em_planner(self):
        scenario = build_highway_merge_scenario()
        planner = build_planner(scenario, planner="em")
        self.assertIsInstance(planner, EMPlanner)


if __name__ == "__main__":
    unittest.main()
