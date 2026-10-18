"""
闭环仿真、真实指标与场景加载的回归测试。

这些测试覆盖早期骨架最容易“看起来能跑、实际上没闭环”的四类问题：
1. 控制器是否真正跟踪目标速度；
2. 指标是否能检测碰撞，而不是永远 success=1；
3. 规划器是否使用真实参考线并做碰撞校验；
4. 同一 seed 是否可复现。
"""

import unittest

import numpy as np

from src.common.state import ObstacleState, ObstacleType, VehicleState
from src.control.lqr_controller import LQRController
from src.decision.base_decision import DecisionOutput, DrivingIntent
from src.decision.rule_based_fsm import HierarchicalFSMDecisionMaker
from src.environment.obstacle_manager import ObstacleManager
from src.environment.road_network import Lane, RoadNetwork
from src.environment.scenario_loader import build_highway_merge_scenario
from src.planning.frenet_lattice import FrenetLatticePlanner
from src.runner import build_engine
from src.simulator import SimulationEngine
from src.vehicle_model.kinematics import KinematicBicycleModel


def _straight_network(length: float = 200.0, lanes=("main_lane_0",)) -> RoadNetwork:
    network = RoadNetwork()
    for idx, lane_id in enumerate(lanes):
        network.add_lane(
            Lane(
                lane_id=lane_id,
                center_points=[(float(x), float(idx * 3.75)) for x in range(0, int(length) + 1, 2)],
            )
        )
        network.generate_reference_line(lane_id, sample_step=1.0)
    return network


class TestClosedLoop(unittest.TestCase):

    def test_builtin_scenario_has_three_lanes(self):
        scenario = build_highway_merge_scenario()
        self.assertIn("ramp_0", scenario.road_network.lanes)
        self.assertIn("main_lane_0", scenario.road_network.lanes)
        self.assertIn("main_lane_1", scenario.road_network.lanes)
        self.assertGreater(len(scenario.road_network.get_reference_line("main_lane_0")), 10)
        self.assertEqual(scenario.current_lane_id, "ramp_0")

    def test_controller_tracks_lower_target_speed(self):
        controller = LQRController()
        model = KinematicBicycleModel()
        state = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=15.0, vx=15.0)
        from src.common.state import Trajectory, TrajectoryPoint
        traj = Trajectory(points=[
            TrajectoryPoint(x=10.0 * i, y=0.0, theta=0.0, kappa=0.0, v=10.0, a=0.0, t=0.1 * i)
            for i in range(51)
        ])
        for _ in range(100):
            cmd = controller.compute_control(state, traj, 0.05)
            state = model.step(state, cmd.acceleration, cmd.steering_angle, 0.05)
        self.assertLess(state.velocity, 15.0)
        self.assertGreater(state.velocity, 9.0)

    def test_planner_uses_curved_reference_line(self):
        radius = 100.0
        network = RoadNetwork()
        points = [
            (radius * np.sin(theta), radius * (1.0 - np.cos(theta)))
            for theta in np.linspace(0.0, 1.0, 60)
        ]
        network.add_lane(Lane(lane_id="arc", center_points=[(float(x), float(y)) for x, y in points]))
        ref = network.generate_reference_line("arc", sample_step=0.5)

        ego = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=15.0, vx=15.0)
        decision = DecisionOutput(
            intent=DrivingIntent.CRUISE,
            target_lane_id="arc",
            target_speed=15.0,
        )
        traj = FrenetLatticePlanner().plan(ego, ref, [], decision, dt=0.1)
        self.assertTrue(traj.points)
        last = traj.points[-1]
        # 轨迹终点应落在圆弧参考线上（允许离散误差）
        expected_y = radius - np.sqrt(max(0.0, radius**2 - last.x**2))
        self.assertLess(abs(last.y - expected_y), 0.5)
        self.assertGreater(max(abs(p.kappa) for p in traj.points), 1e-4)
        self.assertGreater(max(abs(p.theta) for p in traj.points), 1e-3)

    def test_planner_marks_collision_trajectory_invalid(self):
        network = _straight_network()
        ref = network.get_reference_line("main_lane_0")
        ego = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=15.0, vx=15.0)
        obstacle = ObstacleState(
            id="static",
            obstacle_type=ObstacleType.STATIC,
            x=12.0,
            y=0.0,
            length=4.8,
            width=1.9,
            heading=0.0,
        )
        decision = DecisionOutput(
            intent=DrivingIntent.CRUISE,
            target_lane_id="main_lane_0",
            target_speed=15.0,
        )
        traj = FrenetLatticePlanner().plan(ego, ref, [obstacle], decision, dt=0.1)
        self.assertFalse(traj.valid)
        self.assertGreaterEqual(traj.total_cost, 1_000_000.0)

    def test_engine_detects_collision_and_reports_failure(self):
        network = _straight_network()
        manager = ObstacleManager()
        manager.add_obstacle(ObstacleState(
            id="blocker",
            obstacle_type=ObstacleType.STATIC,
            x=2.0,
            y=0.0,
            length=4.8,
            width=1.9,
            heading=0.0,
        ))
        ego = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=10.0, vx=10.0)
        engine = SimulationEngine(
            road_network=network,
            obstacle_manager=manager,
            decision_maker=HierarchicalFSMDecisionMaker(road_network=network),
            planner=FrenetLatticePlanner(),
            controller=LQRController(),
            ego_initial_state=ego,
            enable_visualization=False,
            dt=0.05,
            seed=42,
            vehicle_params={},
            sensor_params={},
            initial_lane_id="main_lane_0",
        )
        record = engine.step()
        self.assertTrue(record.is_collision)
        report = engine.evaluator.compute_summary_report()
        self.assertEqual(report["success_rate"], 0.0)
        self.assertGreater(report["collision_frames"], 0.0)

    def test_seed_reproducibility(self):
        scenario = build_highway_merge_scenario()
        engine_a = build_engine(scenario, decision="fsm", seed=123, enable_visualization=False)
        scenario_b = build_highway_merge_scenario()
        engine_b = build_engine(scenario_b, decision="fsm", seed=123, enable_visualization=False)
        for _ in range(40):
            engine_a.step()
            engine_b.step()
        self.assertAlmostEqual(engine_a.ego_state.x, engine_b.ego_state.x, places=9)
        self.assertAlmostEqual(engine_a.ego_state.y, engine_b.ego_state.y, places=9)
        self.assertAlmostEqual(engine_a.ego_state.velocity, engine_b.ego_state.velocity, places=9)
        self.assertEqual(len(engine_a.evaluator.records), len(engine_b.evaluator.records))


if __name__ == "__main__":
    unittest.main()
