"""任务级 success 与横向约束回归测试。

这些测试锁定两个论文口径：
1. ``success`` 不等于“没撞车”，还必须完成场景任务、不得驶出道路；
2. LQR 的转角上限必须真正服从 ``max_lateral_accel``，不能被 0.05 rad 下限抬高。

测试保持低功耗：只跑 3 个核心场景的短时闭环，全部使用单线程 BLAS + nice。
"""

import math
import unittest
from unittest.mock import patch

from src.common.state import Trajectory, TrajectoryPoint, VehicleState
from src.control.lqr_controller import LQRController
from src.environment.scenario_loader import load_named_scenario
from src.runner import build_engine


class TestTaskLevelSuccess(unittest.TestCase):

    def test_highway_merge_fsm_completes_task(self):
        scenario = load_named_scenario("highway_merge")
        engine = build_engine(scenario, decision="fsm", seed=42, enable_visualization=False)
        report = engine.run(15.0)
        self.assertEqual(report["success_rate"], 1.0)
        self.assertEqual(report["task_completed"], 1.0)
        self.assertEqual(report["road_departure"], 0.0)
        self.assertLess(report["task_completion_time_s"], 15.0)
        self.assertTrue(engine.current_lane_id.startswith("main_lane"))

    def test_game_without_merge_is_not_counted_as_success(self):
        # Game 目前作为探索性模块保留；20 s 内未完成汇入时，任务级 success 必须为 0。
        scenario = load_named_scenario("highway_merge")
        engine = build_engine(scenario, decision="game", seed=42, enable_visualization=False)
        report = engine.run(20.0)
        self.assertEqual(report["task_completed"], 0.0)
        self.assertEqual(report["success_rate"], 0.0)
        self.assertEqual(engine.current_lane_id, "ramp_0")

    def test_intersection_not_completed_is_failure(self):
        # 当前交叉口场景仍为实验性：未完成左转时必须判失败。
        # 安全监督会在 invalid 轨迹上制动，因此这里不强制要求驶出道路；
        # 关键口径是“没完成任务 -> success=0”。
        scenario = load_named_scenario("intersection_unprotected")
        engine = build_engine(scenario, decision="fsm", seed=42, enable_visualization=False)
        report = engine.run(12.0)
        self.assertEqual(report["task_completed"], 0.0)
        self.assertEqual(report["success_rate"], 0.0)
        self.assertGreater(report["planner_invalid_count"], 0.0)

    def test_cut_in_success_requires_safe_gap(self):
        scenario = load_named_scenario("cut_in")
        engine = build_engine(scenario, decision="fsm", seed=42, enable_visualization=False)
        report = engine.run(12.0)
        self.assertEqual(report["success_rate"], 1.0)
        self.assertGreaterEqual(report["min_obstacle_distance_m"], 2.0)
        self.assertEqual(report["road_departure"], 0.0)

    def test_emergency_avoidance_success_requires_stop(self):
        scenario = load_named_scenario("emergency_avoidance")
        engine = build_engine(scenario, decision="fsm", seed=42, enable_visualization=False)
        report = engine.run(20.0)
        self.assertEqual(report["success_rate"], 1.0)
        self.assertLessEqual(engine.ego_state.velocity, 0.5)
        self.assertGreaterEqual(report["min_obstacle_distance_m"], 2.0)

    def test_completed_task_reports_real_completion_time(self):
        """``task_completed == 1`` 时不得再写出 999 哨兵完成时间。

        早期实现只给 highway_merge / 交叉口写 ``task_completion_time``，
        cut_in 与 emergency_avoidance 的完成时刻永远为 None，最终被序列化成
        999，导致同一行数据自相矛盾。
        """
        cases = (("cut_in", 12.0), ("emergency_avoidance", 22.0))
        for name, duration in cases:
            with self.subTest(scenario=name):
                scenario = load_named_scenario(name)
                engine = build_engine(
                    scenario, decision="fsm", seed=42, enable_visualization=False
                )
                report = engine.run(duration)
                self.assertEqual(report["task_completed"], 1.0)
                self.assertEqual(report["success_rate"], 1.0)
                self.assertLess(report["task_completion_time_s"], duration)
                self.assertGreater(report["task_completion_time_s"], 0.0)

    def test_uncompleted_task_keeps_sentinel_completion_time(self):
        """未完成时仍用 999 表示"无完成时刻"，与 completed=0 保持一致。"""
        scenario = load_named_scenario("highway_merge")
        engine = build_engine(scenario, decision="game", seed=42, enable_visualization=False)
        report = engine.run(10.0)
        self.assertEqual(report["task_completed"], 0.0)
        self.assertEqual(report["task_completion_time_s"], 999.0)


class TestRunnerResolvedDefaults(unittest.TestCase):
    """``build_engine`` 必须使用解析后的算法名，而不是原始参数。"""

    def test_none_decision_falls_back_to_fsm(self):
        scenario = load_named_scenario("highway_merge")
        engine = build_engine(scenario, decision=None, enable_visualization=False)
        self.assertEqual(type(engine.decision_maker).__name__, "HierarchicalFSMDecisionMaker")

    def test_none_planner_and_controller_use_scenario_defaults(self):
        scenario = load_named_scenario("cut_in")
        engine = build_engine(
            scenario, decision="fsm", planner=None, controller=None, enable_visualization=False
        )
        self.assertEqual(type(engine.planner).__name__, "FrenetLatticePlanner")
        self.assertEqual(type(engine.controller).__name__, "LQRController")


class TestPlannerSafetyFallback(unittest.TestCase):
    def test_invalid_trajectory_triggers_braking_fallback(self):
        scenario = load_named_scenario("highway_merge")
        engine = build_engine(scenario, decision="fsm", seed=42, enable_visualization=False)
        with patch.object(engine.planner, "plan", return_value=Trajectory(points=[], valid=False)):
            record = engine.step()
        self.assertTrue(record.planner_invalid)
        self.assertLess(engine.last_control.acceleration, 0.0)
        report = engine.evaluator.compute_summary_report()
        self.assertEqual(report["planner_invalid_count"], 1.0)


class TestLateralAccelLimit(unittest.TestCase):

    def test_lqr_steering_limit_respects_lateral_accel_cap(self):
        controller = LQRController(max_lateral_accel=2.5)
        speed = 22.0
        state = VehicleState(
            x=0.0, y=0.0, heading=0.0, velocity=speed, vx=speed, yaw_rate=0.0
        )
        # 给一个很大的横向偏差，迫使反馈转角顶到上限。
        traj = Trajectory(points=[
            TrajectoryPoint(
                x=speed * 0.1 * i, y=5.0, theta=0.0, kappa=0.0, v=speed, t=0.1 * i
            )
            for i in range(51)
        ])
        command = controller.compute_control(state, traj, 0.05)
        lateral_accel = speed ** 2 / controller.wheelbase * math.tan(abs(command.steering_angle))
        self.assertLessEqual(lateral_accel, 2.5 * 1.02 + 1e-6)
        self.assertLessEqual(
            abs(command.steering_angle),
            math.atan(controller.wheelbase * 2.5 / speed ** 2) + 1e-6,
        )


if __name__ == "__main__":
    unittest.main()
