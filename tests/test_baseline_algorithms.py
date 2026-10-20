"""
经典基线算法单元测试 (IDM 决策 / 中心线规划 / 纯跟踪控制)。

覆盖任务要求：
- IDM：同车道前车跟驰、静止前车制动、邻道目标不误判；
- CenterlinePlanner：直线/曲线参考线轨迹有效且 s 单调、横向偏移收敛、空参考线兜底、
  零速/零目标速度不崩溃；
- PurePursuitController：简单曲线轨迹上航向误差收敛、空轨迹触发制动、转角速率限制。
"""

import unittest

import numpy as np

from src.common.frenet_frame import FrenetConverter, ReferencePoint
from src.common.state import (
    ObstacleState,
    ObstacleType,
    Trajectory,
    TrajectoryPoint,
    VehicleState,
)
from src.control.pure_pursuit_controller import PurePursuitController
from src.decision.base_decision import DecisionOutput, DrivingIntent
from src.decision.idm_decision import IDMDecisionMaker
from src.planning.centerline_planner import CenterlinePlanner
from src.vehicle_model.kinematics import KinematicBicycleModel


class _StraightRoadNetwork:
    """最小道路网络桩：直线场景下车道中心 y 恒为 0。"""

    def get_lane_center_y(self, lane_id: str):
        return 0.0


def _straight_reference(length: float = 150.0, step: float = 1.0):
    """沿 +x 方向、y=0 的直线参考线。"""
    ss = np.arange(0.0, length + step, step)
    return [
        ReferencePoint(
            x=float(s), y=0.0, theta=0.0, kappa=0.0, dkappa=0.0, s=float(s)
        )
        for s in ss
    ]


def _arc_reference(radius: float = 50.0, length: float = 150.0, step: float = 1.0):
    """圆心位于 (0, radius) 的逆时针圆弧参考线。"""
    ss = np.arange(0.0, length + step, step)
    points = []
    for s in ss:
        theta = float(s) / radius
        points.append(
            ReferencePoint(
                x=float(radius * np.sin(theta)),
                y=float(radius * (1.0 - np.cos(theta))),
                theta=float(theta),
                kappa=float(1.0 / radius),
                dkappa=0.0,
                s=float(s),
            )
        )
    return points


def _arc_trajectory(
    radius: float,
    speed: float,
    phase: float,
    duration: float = 2.0,
    dt: float = 0.1,
) -> Trajectory:
    """以 phase 为起点、沿圆弧匀速前进的时间参数化轨迹。"""
    steps = int(round(duration / dt)) + 1
    points = []
    for k in range(steps):
        t = k * dt
        phi = phase + speed * t / radius
        points.append(
            TrajectoryPoint(
                x=float(radius * np.sin(phi)),
                y=float(radius * (1.0 - np.cos(phi))),
                theta=float(phi),
                kappa=float(1.0 / radius),
                dkappa=0.0,
                v=float(speed),
                a=0.0,
                jerk=0.0,
                t=float(t),
                s=float(speed * t),
            )
        )
    return Trajectory(points=points)


class TestIDMDecisionMaker(unittest.TestCase):
    """IDM 决策基线：本车道跟驰/巡航，不做换道。"""

    def setUp(self):
        self.decision_maker = IDMDecisionMaker(road_network=_StraightRoadNetwork())

    @staticmethod
    def _obstacle(obs_id: str, x: float, y: float, vx: float) -> ObstacleState:
        return ObstacleState(
            id=obs_id,
            obstacle_type=ObstacleType.CAR,
            x=x,
            y=y,
            length=4.8,
            width=1.9,
            heading=0.0,
            vx=vx,
        )

    def test_follows_lead_in_current_lane(self):
        ego = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=10.0, vx=10.0)
        lead = self._obstacle("lead", x=20.0, y=0.0, vx=10.0)

        decision = self.decision_maker.make_decision(ego, [lead], "main_lane_0")

        self.assertEqual(decision.intent, DrivingIntent.FOLLOW)
        self.assertEqual(decision.lead_obstacle_id, "lead")
        self.assertEqual(decision.target_lane_id, "main_lane_0")
        self.assertLessEqual(decision.target_speed, self.decision_maker.desired_speed)
        self.assertLess(decision.target_speed, ego.velocity)
        self.assertIn("IDM", decision.reasoning_log)

    def test_brakes_for_stationary_lead(self):
        # 远距离静止前车：IDM 轻微减速但尚未触发 AEB，应保持 FOLLOW 意图
        ego = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=10.0, vx=10.0)
        stopped = self._obstacle("stopped_far", x=40.0, y=0.0, vx=0.0)

        decision = self.decision_maker.make_decision(
            ego, [stopped], "main_lane_0"
        )

        self.assertEqual(decision.intent, DrivingIntent.FOLLOW)
        self.assertEqual(decision.lead_obstacle_id, "stopped_far")
        self.assertLess(decision.target_speed, ego.velocity)
        self.assertGreaterEqual(decision.target_speed, 0.0)

    def test_aeb_override_for_closing_lead(self):
        # 高速接近静止前车：IDM 期望减速度低于 -3 m/s^2，触发 AEB 零速指令
        ego = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=20.0, vx=20.0)
        stopped = self._obstacle("stopped_near", x=40.0, y=0.0, vx=0.0)

        decision = self.decision_maker.make_decision(
            ego, [stopped], "main_lane_0"
        )

        self.assertEqual(decision.intent, DrivingIntent.EMERGENCY_STOP)
        self.assertEqual(decision.lead_obstacle_id, "stopped_near")
        self.assertEqual(decision.target_lane_id, "main_lane_0")
        self.assertAlmostEqual(decision.target_speed, 0.0)
        self.assertIn("AEB", decision.reasoning_log)

    def test_speed_command_horizon_controls_target_speed(self):
        # 同一跟驰场景下，速度指令时间尺度越长，目标速度相对当前速度的偏移越大
        ego = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=10.0, vx=10.0)
        lead = self._obstacle("lead", x=20.0, y=0.0, vx=10.0)

        short_horizon = IDMDecisionMaker(
            road_network=_StraightRoadNetwork(), speed_command_horizon=0.5
        )
        long_horizon = IDMDecisionMaker(
            road_network=_StraightRoadNetwork(), speed_command_horizon=1.0
        )
        decision_short = short_horizon.make_decision(ego, [lead], "main_lane_0")
        decision_long = long_horizon.make_decision(ego, [lead], "main_lane_0")

        # 默认构造应为 1.0 s 时间尺度
        self.assertAlmostEqual(self.decision_maker.speed_command_horizon, 1.0)
        self.assertEqual(decision_short.intent, DrivingIntent.FOLLOW)
        self.assertEqual(decision_long.intent, DrivingIntent.FOLLOW)
        self.assertLess(decision_long.target_speed, decision_short.target_speed)

        # 目标速度须严格等于 clip(v + a_idm * horizon, 0, desired_speed)
        accel = self.decision_maker.idm.compute_acceleration(
            current_speed=ego.velocity,
            lead_speed=lead.vx,
            distance=20.0 - 0.5 * (4.8 + lead.length),
        )
        expected_long = float(np.clip(10.0 + accel * 1.0, 0.0, 22.22))
        self.assertAlmostEqual(decision_long.target_speed, expected_long, places=6)

    def test_ignores_adjacent_lane_obstacle(self):
        ego = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=10.0, vx=10.0)
        adjacent = self._obstacle("adjacent", x=10.0, y=3.75, vx=8.0)

        decision = self.decision_maker.make_decision(
            ego, [adjacent], "main_lane_0"
        )

        self.assertEqual(decision.intent, DrivingIntent.CRUISE)
        self.assertIsNone(decision.lead_obstacle_id)
        self.assertAlmostEqual(decision.target_speed, self.decision_maker.desired_speed)


class TestCenterlinePlanner(unittest.TestCase):
    """中心线规划基线：单条五次多项式轨迹，无搜索/无障碍物感知。"""

    def setUp(self):
        self.planner = CenterlinePlanner()

    @staticmethod
    def _decision(target_speed: float = 12.0, lateral_shift_l: float = 0.0):
        return DecisionOutput(
            intent=DrivingIntent.CRUISE,
            target_lane_id="main_lane_0",
            target_speed=target_speed,
            lateral_shift_l=lateral_shift_l,
        )

    def _assert_finite_and_monotonic(self, trajectory: Trajectory) -> None:
        self.assertTrue(trajectory.valid)
        s_values = [point.s for point in trajectory.points]
        self.assertTrue(np.all(np.diff(s_values) >= -1e-9))
        for point in trajectory.points:
            for attr in ("x", "y", "theta", "kappa", "dkappa", "v", "a", "jerk", "t", "s"):
                self.assertTrue(
                    np.isfinite(getattr(point, attr)),
                    msg=f"non-finite {attr} at t={point.t}",
                )
            self.assertGreaterEqual(point.v, 0.0)

    def test_straight_reference_centerline_tracking(self):
        reference = _straight_reference()
        ego = VehicleState(x=20.0, y=0.0, heading=0.0, velocity=10.0, vx=10.0)

        trajectory = self.planner.plan(
            ego,
            reference,
            [],
            self._decision(target_speed=12.0),
            plan_time_horizon=3.0,
            dt=0.1,
        )

        self._assert_finite_and_monotonic(trajectory)
        self.assertEqual(len(trajectory.points), 31)
        self.assertAlmostEqual(trajectory.points[0].t, 0.0)
        self.assertAlmostEqual(trajectory.points[-1].t, 3.0)
        self.assertAlmostEqual(trajectory.points[0].v, 10.0, places=2)
        self.assertAlmostEqual(trajectory.points[-1].v, 12.0, places=2)
        # 直线参考线上应保持在车道中心（l=0）
        self.assertTrue(np.allclose([p.y for p in trajectory.points], 0.0, atol=1e-6))

    def test_curved_reference_with_lateral_convergence(self):
        radius = 40.0
        reference = _arc_reference(radius=radius)
        s0 = 15.0
        theta0 = s0 / radius
        ego = VehicleState(
            x=radius * np.sin(theta0),
            y=radius * (1.0 - np.cos(theta0)),
            heading=theta0,
            velocity=8.0,
            vx=8.0,
        )

        trajectory = self.planner.plan(
            ego,
            reference,
            [],
            self._decision(target_speed=10.0, lateral_shift_l=0.75),
            plan_time_horizon=3.0,
            dt=0.1,
        )

        self._assert_finite_and_monotonic(trajectory)
        final = trajectory.points[-1]
        ref_final = FrenetConverter.ref_point_at_s(reference, final.s)
        # 终点横向偏移应平滑收敛到目标 0.75 m
        lateral_final = (
            -np.sin(ref_final.theta) * (final.x - ref_final.x)
            + np.cos(ref_final.theta) * (final.y - ref_final.y)
        )
        self.assertAlmostEqual(float(lateral_final), 0.75, delta=0.05)
        # 曲线段曲率应接近参考线曲率 1/R
        self.assertAlmostEqual(final.kappa, 1.0 / radius, delta=0.01)

    def test_initial_lateral_rate_points_toward_target_lane(self):
        # 本规划器每 50 ms 重规划一次；若横向初速度为 0，收敛会 Zeno 式停滞。
        # 因此要求首段轨迹已具有朝目标车道的初始横向速率。
        radius = 40.0
        reference = _arc_reference(radius=radius)
        s0 = 15.0
        theta0 = s0 / radius
        ego = VehicleState(
            x=radius * np.sin(theta0),
            y=radius * (1.0 - np.cos(theta0)),
            heading=theta0,
            velocity=8.0,
            vx=8.0,
        )

        trajectory = self.planner.plan(
            ego,
            reference,
            [],
            self._decision(target_speed=10.0, lateral_shift_l=0.75),
            plan_time_horizon=3.0,
            dt=0.1,
        )

        self._assert_finite_and_monotonic(trajectory)
        first = trajectory.points[0]
        second = trajectory.points[1]
        ref_first = FrenetConverter.ref_point_at_s(reference, first.s)
        ref_second = FrenetConverter.ref_point_at_s(reference, second.s)
        lateral_first = (
            -np.sin(ref_first.theta) * (first.x - ref_first.x)
            + np.cos(ref_first.theta) * (first.y - ref_first.y)
        )
        lateral_second = (
            -np.sin(ref_second.theta) * (second.x - ref_second.x)
            + np.cos(ref_second.theta) * (second.y - ref_second.y)
        )
        initial_lateral_rate = (lateral_second - lateral_first) / max(
            1e-9, second.t - first.t
        )
        self.assertGreater(initial_lateral_rate, 0.0)
        # 期望初始速率约 clip(0.8 * 0.75, -1.2, 1.2) = 0.6 m/s
        self.assertAlmostEqual(initial_lateral_rate, 0.6, delta=0.1)

    def test_empty_reference_line_falls_back_to_straight(self):
        ego = VehicleState(x=3.0, y=1.0, heading=0.3, velocity=5.0, vx=5.0)

        trajectory = self.planner.plan(
            ego,
            [],
            [],
            self._decision(target_speed=8.0),
            plan_time_horizon=2.0,
            dt=0.1,
        )

        self._assert_finite_and_monotonic(trajectory)
        self.assertAlmostEqual(trajectory.points[0].x, ego.x, delta=0.5)
        self.assertAlmostEqual(trajectory.points[0].y, ego.y, delta=0.5)

    def test_zero_speed_and_zero_target_speed_do_not_crash(self):
        reference = _straight_reference()
        ego = VehicleState(x=10.0, y=0.0, heading=0.0, velocity=0.0, vx=0.0)

        trajectory = self.planner.plan(
            ego,
            reference,
            [],
            self._decision(target_speed=0.0),
            plan_time_horizon=2.0,
            dt=0.1,
        )

        self._assert_finite_and_monotonic(trajectory)
        self.assertTrue(len(trajectory.points) > 1)
        self.assertTrue(
            np.allclose([p.v for p in trajectory.points], 0.0, atol=1e-6)
        )


class TestPurePursuitController(unittest.TestCase):
    """纯跟踪控制基线：几何预瞄 + PID 纵向控制。"""

    @staticmethod
    def _phase_of(state: VehicleState, radius: float) -> float:
        """车辆在圆弧参考线上的相位角。"""
        return float(np.arctan2(state.x, radius - state.y))

    def test_empty_trajectory_brakes(self):
        controller = PurePursuitController()
        command = controller.compute_control(
            VehicleState(), Trajectory(points=[]), dt=0.05
        )

        self.assertAlmostEqual(command.acceleration, -2.0)
        self.assertAlmostEqual(command.brake, 0.4)
        self.assertAlmostEqual(command.throttle, 0.0)
        self.assertAlmostEqual(command.steering_angle, 0.0)

    def test_curve_requests_left_steer_with_rate_limit(self):
        radius = 30.0
        speed = 8.0
        phase = 0.10
        controller = PurePursuitController()
        state = VehicleState(
            x=radius * np.sin(phase),
            y=radius * (1.0 - np.cos(phase)),
            heading=0.0,
            velocity=speed,
            vx=speed,
        )
        trajectory = _arc_trajectory(radius=radius, speed=speed, phase=phase)

        command = controller.compute_control(state, trajectory, dt=0.05)

        # 圆弧向左弯曲，且自车航向落后于路径切向，应输出左转（正）转角
        self.assertGreater(command.steering_angle, 0.0)
        self.assertLessEqual(abs(command.steering_angle), controller.max_steer)
        self.assertLessEqual(
            abs(command.steering_rate), controller.max_steer_rate + 1e-9
        )
        self.assertGreaterEqual(command.steering_rate, 0.0)

    def test_reduces_heading_error_on_curved_trajectory(self):
        radius = 40.0
        speed = 8.0
        dt = 0.05
        phase0 = 0.20
        controller = PurePursuitController()
        model = KinematicBicycleModel(wheelbase=controller.wheelbase)

        state = VehicleState(
            x=radius * np.sin(phase0),
            y=radius * (1.0 - np.cos(phase0)),
            heading=0.0,
            velocity=speed,
            vx=speed,
        )

        def heading_error() -> float:
            phase = self._phase_of(state, radius)
            error = float(state.heading - phase)
            return float(np.arctan2(np.sin(error), np.cos(error)))

        initial_error = abs(heading_error())
        self.assertGreater(initial_error, 0.1)

        final_error = initial_error
        for _ in range(40):
            phase = self._phase_of(state, radius)
            trajectory = _arc_trajectory(
                radius=radius, speed=speed, phase=phase, duration=2.0, dt=0.1
            )
            command = controller.compute_control(state, trajectory, dt=dt)
            state = model.step(
                state, command.acceleration, command.steering_angle, dt
            )
            final_error = abs(heading_error())

        self.assertLess(final_error, 0.5 * initial_error)
        self.assertLess(final_error, 0.08)

    def test_longitudinal_brakes_when_reference_slower(self):
        controller = PurePursuitController()
        trajectory = Trajectory(
            points=[
                TrajectoryPoint(
                    x=float(s),
                    y=0.0,
                    theta=0.0,
                    v=5.0,
                    a=0.0,
                    t=float(s) / 5.0,
                    s=float(s),
                )
                for s in np.arange(0.0, 10.0, 0.5)
            ]
        )
        state = VehicleState(x=0.0, y=0.0, heading=0.0, velocity=15.0, vx=15.0)

        command = controller.compute_control(state, trajectory, dt=0.05)

        self.assertLess(command.acceleration, 0.0)
        self.assertGreater(command.brake, 0.0)
        self.assertAlmostEqual(command.throttle, 0.0)

    def test_reset_clears_internal_state(self):
        controller = PurePursuitController()
        trajectory = _arc_trajectory(radius=25.0, speed=10.0, phase=0.0)
        state = VehicleState(
            x=0.0, y=0.0, heading=0.0, velocity=10.0, vx=10.0
        )
        controller.compute_control(state, trajectory, dt=0.05)
        controller.reset()

        self.assertAlmostEqual(controller._prev_steer, 0.0)
        self.assertAlmostEqual(controller._prev_accel, 0.0)
        command = controller.compute_control(state, trajectory, dt=0.05)
        self.assertLessEqual(
            abs(command.steering_angle), controller.max_steer_rate * 0.05 + 1e-9
        )


if __name__ == "__main__":
    unittest.main()
