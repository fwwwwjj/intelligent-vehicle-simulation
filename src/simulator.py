"""
仿真系统主引擎闭环控制核心 (Simulation Engine)。
整合数字环境底座、车载感知数字模型、行为决策模块、轨迹规划优化器、运动控制算法及
车辆物理动力学闭环，并逐步计算可复核的安全性/舒适性/跟踪精度指标。

与早期骨架的关键区别：
- 轨迹规划 dt 与仿真 dt 分离，但控制按轨迹时间索引跟踪；
- KPI 由真实自车/障碍物状态计算，不再写死 lateral_error=0 / min_distance=100；
- 感知随机数由 seed 初始化，保证同一 seed 可复现；
- 可视化模块惰性导入，headless 环境无需 matplotlib。
"""

import time
from typing import Any, Dict, Optional

import numpy as np

from src.common.state import VehicleState, Trajectory, ObstacleState
from src.common.frenet_frame import FrenetConverter, ReferencePoint
from src.common.geometry import BoundingBox2D, CollisionChecker
from src.environment.road_network import RoadNetwork
from src.environment.obstacle_manager import ObstacleManager
from src.vehicle_model.kinematics import KinematicBicycleModel
from src.vehicle_model.dynamics import DynamicBicycleModel
from src.vehicle_model.sensor_model import SensorModel
from src.decision.base_decision import BaseDecisionMaker, DecisionOutput
from src.planning.base_planner import BasePlanner
from src.control.base_controller import BaseController, ControlCommand
from src.evaluation.metrics import PerformanceEvaluator, SimulationMetricsRecord


class SimulationEngine:
    """
    智能车辆行为决策与路径规划仿真主引擎
    """

    def __init__(
        self,
        road_network: RoadNetwork,
        obstacle_manager: ObstacleManager,
        decision_maker: BaseDecisionMaker,
        planner: BasePlanner,
        controller: BaseController,
        ego_initial_state: VehicleState,
        use_dynamic_model: bool = False,
        enable_visualization: bool = True,
        dt: float = 0.05,
        seed: int = 42,
        vehicle_params: Optional[Dict[str, Any]] = None,
        sensor_params: Optional[Dict[str, float]] = None,
        initial_lane_id: str = "main_lane_0",
        plan_dt: float = 0.1,
        plan_horizon: float = 5.0,
        tire_model: Optional[str] = None,
    ):
        self.road_network = road_network
        self.obstacle_manager = obstacle_manager
        self.decision_maker = decision_maker
        self.planner = planner
        self.controller = controller
        self.ego_state = ego_initial_state
        self.tire_model = str(
            tire_model
            or (vehicle_params or {}).get("tire_model", "linear")
        ).lower()
        # Pacejka 轮胎只存在于 2-DOF 动力学模型；显式选择 Pacejka 时自动启用
        self.use_dynamic_model = bool(
            use_dynamic_model or self.tire_model == "pacejka"
        )
        self.enable_visualization = enable_visualization
        self.dt = float(dt)
        self.plan_dt = float(plan_dt)
        self.plan_horizon = float(plan_horizon)
        self.seed = int(seed)

        vp = vehicle_params or {}
        self.vehicle_length = float(vp.get("length", 4.8))
        self.vehicle_width = float(vp.get("width", 1.9))
        self.wheelbase = float(vp.get("wheelbase", 2.85))
        self.max_steer = float(vp.get("max_steer_angle", 0.65))
        self.max_accel = float(vp.get("max_accel", 3.5))
        self.max_decel = float(vp.get("max_decel", 7.0))

        # 物理与感知模型
        self.kinematic_model = KinematicBicycleModel(
            wheelbase=self.wheelbase,
            max_steer=self.max_steer,
            max_accel=self.max_accel,
            max_decel=self.max_decel,
        )
        self.dynamic_model = DynamicBicycleModel(
            mass=float(vp.get("mass", 1600.0)),
            iz=float(vp.get("moment_inertia_z", 2500.0)),
            lf=float(vp.get("lf", 1.35)),
            lr=float(vp.get("lr", 1.50)),
            cf=float(vp.get("cf", 120000.0)),
            cr=float(vp.get("cr", 120000.0)),
            tire_model=self.tire_model,
            pacejka_params=vp.get("pacejka"),
            friction_coefficient=float(vp.get("friction_coefficient", 0.9)),
            cg_height=float(vp.get("cg_height", 0.55)),
        )
        sp = sensor_params or {}
        self.sensor_model = SensorModel(
            max_range=float(sp.get("max_range", 80.0)),
            fov_deg=float(sp.get("fov_deg", 360.0)),
            pos_noise_std=float(sp.get("pos_noise_std", 0.05)),
            vel_noise_std=float(sp.get("vel_noise_std", 0.10)),
            rng=np.random.default_rng(self.seed),
        )

        # 评估器与可视化器
        self.evaluator = PerformanceEvaluator()
        self.visualizer = None
        if self.enable_visualization:
            try:
                from src.visualization.visualizer_2d import Visualizer2D

                self.visualizer = Visualizer2D()
            except Exception as exc:  # pragma: no cover - 取决于图形环境
                print(f"[SimulationEngine] Visualization disabled: {exc}")

        self.current_time = 0.0
        self.current_lane_id = initial_lane_id
        self.last_decision: Optional[DecisionOutput] = None
        self.last_trajectory: Optional[Trajectory] = None
        self.last_control = None

        # 指标微分状态
        self._prev_accel = float(ego_initial_state.acceleration)
        self._prev_x = float(ego_initial_state.x)
        self._prev_y = float(ego_initial_state.y)
        self._distance_traveled = 0.0
        self._step_index = 0

    # ------------------------------------------------------------------
    # 指标计算
    # ------------------------------------------------------------------
    def _tracking_errors(
        self,
        trajectory: Optional[Trajectory],
        reference_line: Optional[list],
    ) -> tuple[float, float]:
        """相对规划轨迹最近点的横向/航向误差；无轨迹时退回参考线。"""
        if trajectory is not None and trajectory.points:
            nearest = min(
                trajectory.points,
                key=lambda p: (p.x - self.ego_state.x) ** 2 + (p.y - self.ego_state.y) ** 2,
            )
            dx = self.ego_state.x - nearest.x
            dy = self.ego_state.y - nearest.y
            lateral = -np.sin(nearest.theta) * dx + np.cos(nearest.theta) * dy
            heading = float(np.arctan2(
                np.sin(self.ego_state.heading - nearest.theta),
                np.cos(self.ego_state.heading - nearest.theta),
            ))
            return float(lateral), heading

        if reference_line:
            frenet = FrenetConverter.cartesian_to_frenet(self.ego_state, reference_line)
            ref_pt = FrenetConverter.ref_point_at_s(reference_line, frenet.s)
            heading = float(np.arctan2(
                np.sin(self.ego_state.heading - ref_pt.theta),
                np.cos(self.ego_state.heading - ref_pt.theta),
            ))
            return float(frenet.l), heading

        return 0.0, 0.0

    def _safety_metrics(self, obstacles: list[ObstacleState]) -> tuple[float, float, int]:
        """
        计算最小 OBB 净距离、最小 TTC 与本步发生碰撞的障碍物数量。
        """
        min_distance = 999.0
        min_ttc = 999.0
        collision_count = 0
        if not obstacles:
            return min_distance, min_ttc, collision_count

        ego_box = BoundingBox2D(
            self.ego_state.x,
            self.ego_state.y,
            self.vehicle_length,
            self.vehicle_width,
            self.ego_state.heading,
        )
        cos_h = float(np.cos(self.ego_state.heading))
        sin_h = float(np.sin(self.ego_state.heading))

        for obs in obstacles:
            obs_box = BoundingBox2D(obs.x, obs.y, obs.length, obs.width, obs.heading)
            if CollisionChecker.check_obb_collision(ego_box, obs_box, buffer=0.0):
                collision_count += 1

            distance = CollisionChecker.min_distance_between_boxes(ego_box, obs_box)
            min_distance = min(min_distance, distance)

            # TTC：仅考虑自车前方、横向重叠的目标
            dx = obs.x - self.ego_state.x
            dy = obs.y - self.ego_state.y
            dx_body = cos_h * dx + sin_h * dy
            dy_body = -sin_h * dx + cos_h * dy
            if dx_body <= 0.0:
                continue
            if abs(dy_body) > 0.5 * (self.vehicle_width + obs.width) + 0.3:
                continue

            rel_speed = self.ego_state.velocity - (obs.vx * cos_h + obs.vy * sin_h)
            gap = dx_body - 0.5 * (self.vehicle_length + obs.length)
            if gap <= 0.0 and rel_speed > 0.0:
                ttc = 0.0
            elif rel_speed > 0.1:
                ttc = max(0.0, gap) / rel_speed
            else:
                continue
            min_ttc = min(min_ttc, ttc)

        return float(min_distance), float(min_ttc), collision_count

    def _is_planner_fallback(self, planned_trajectory: Optional[Trajectory]) -> bool:
        """
        判断当前步规划器是否进入回退逻辑。
        - EM 规划器：若路径 QP 或速度 QP 任一求解失败（或使用兜底），则判定为规划回退；
        - 空轨迹：无论何种规划器，若未能产出有效轨迹点，判定为规划回退；
        - Lattice 规划器：基于采样的确定性多项式筛选，无 QP 求解回退机制，有轨迹时稳定判定为 False。
        """
        if planned_trajectory is None or not planned_trajectory.points:
            return True
        if hasattr(self.planner, "last_path_result") or hasattr(self.planner, "last_speed_result"):
            path_res = getattr(self.planner, "last_path_result", None)
            speed_res = getattr(self.planner, "last_speed_result", None)
            if path_res is None or speed_res is None:
                return True
            if not getattr(path_res, "success", False) or not getattr(speed_res, "success", False):
                return True
            if str(getattr(path_res, "note", "")).startswith("fallback") or str(getattr(speed_res, "note", "")).startswith("fallback"):
                return True
        return False

    def _is_controller_fallback(self, control_cmd: Optional[ControlCommand]) -> bool:
        """
        判断当前步控制器是否进入回退逻辑。
        - MPC 控制器：若未求解、空轨迹、或 ADMM-QP 求解失败进入 PD/P 兜底，判定为控制回退；
        - LQR 控制器：基于代数黎卡提方程解析求解，无 QP 迭代失败回退机制，稳定判定为 False。
        """
        if hasattr(self.controller, "last_solve_info"):
            solve_info = getattr(self.controller, "last_solve_info", None)
            if not isinstance(solve_info, dict) or not solve_info:
                return True
            if solve_info.get("status") == "empty_trajectory":
                return True
            return not bool(solve_info.get("success", False))
        return False

    # ------------------------------------------------------------------
    # 单步闭环
    # ------------------------------------------------------------------
    def step(self) -> SimulationMetricsRecord:
        """
        执行单个仿真控制周期 dt 的完整闭环运算流程。
        """
        # 1. 动态障碍物物理演化（IDM/匀速）
        self.obstacle_manager.step(self.dt)
        ground_truth_obstacles = self.obstacle_manager.get_all_obstacles()

        # 2. 感知模型过滤（FOV + 噪声）
        perceived_obstacles = self.sensor_model.filter_perceived_obstacles(
            self.ego_state, ground_truth_obstacles
        )

        # 3. 行为决策
        t_decision = time.perf_counter()
        decision_output = self.decision_maker.make_decision(
            self.ego_state, perceived_obstacles, self.current_lane_id
        )
        decision_latency_ms = (time.perf_counter() - t_decision) * 1000.0

        # 4. 局部时空轨迹规划（目标车道参考线，缺失时退回当前车道）
        reference_line = self.road_network.get_reference_line(decision_output.target_lane_id)
        if not reference_line:
            reference_line = self.road_network.get_reference_line(self.current_lane_id) or []
        t_plan = time.perf_counter()
        planned_trajectory = self.planner.plan(
            self.ego_state,
            reference_line,
            perceived_obstacles,
            decision_output,
            plan_time_horizon=self.plan_horizon,
            dt=self.plan_dt,
        )
        planning_latency_ms = (time.perf_counter() - t_plan) * 1000.0

        # 5. 底层运动跟踪控制
        t_control = time.perf_counter()
        control_cmd = self.controller.compute_control(
            self.ego_state, planned_trajectory, self.dt
        )
        control_latency_ms = (time.perf_counter() - t_control) * 1000.0

        # 6. 车辆机理模型前向积分
        if self.use_dynamic_model:
            self.ego_state = self.dynamic_model.step(
                self.ego_state, control_cmd.acceleration, control_cmd.steering_angle, self.dt
            )
        else:
            self.ego_state = self.kinematic_model.step(
                self.ego_state, control_cmd.acceleration, control_cmd.steering_angle, self.dt
            )

        # 7. 车道归属更新（换道/汇入后 current_lane_id 会真正变化）
        lane_id = self.road_network.lane_id_at(
            self.ego_state.x, self.ego_state.y, previous_lane_id=self.current_lane_id
        )
        if lane_id is not None:
            self.current_lane_id = lane_id

        # 8. 真实 KPI 计算与回退诊断
        lateral_error, heading_error = self._tracking_errors(planned_trajectory, reference_line)
        min_distance, min_ttc, collision_count = self._safety_metrics(ground_truth_obstacles)
        planner_fallback = self._is_planner_fallback(planned_trajectory)
        controller_fallback = self._is_controller_fallback(control_cmd)

        dx = self.ego_state.x - self._prev_x
        dy = self.ego_state.y - self._prev_y
        self._distance_traveled += float(np.hypot(dx, dy))

        if self._step_index == 0:
            jerk = 0.0
        else:
            jerk = (self.ego_state.acceleration - self._prev_accel) / max(self.dt, 1e-6)

        record = SimulationMetricsRecord(
            timestamp=self.current_time,
            velocity=self.ego_state.velocity,
            acceleration=self.ego_state.acceleration,
            jerk=float(jerk),
            lateral_error=float(lateral_error),
            heading_error=float(heading_error),
            min_obstacle_distance=float(min_distance),
            min_ttc=float(min_ttc),
            planning_latency_ms=float(planning_latency_ms),
            is_collision=collision_count > 0,
            lateral_accel=float(abs(self.ego_state.ay)),
            decision_latency_ms=float(decision_latency_ms),
            control_latency_ms=float(control_latency_ms),
            distance_traveled=float(self._distance_traveled),
            collision_count=int(collision_count),
            planner_fallback=planner_fallback,
            controller_fallback=controller_fallback,
        )
        self.evaluator.record_step(record)

        # 9. 实时画面渲染
        if self.enable_visualization and self.visualizer is not None:
            self.visualizer.render_frame(
                self.ego_state,
                ground_truth_obstacles,
                self.road_network,
                planned_trajectory,
                self.current_time,
            )

        self.last_decision = decision_output
        self.last_trajectory = planned_trajectory
        self.last_control = control_cmd
        self._prev_accel = float(self.ego_state.acceleration)
        self._prev_x = float(self.ego_state.x)
        self._prev_y = float(self.ego_state.y)
        self._step_index += 1
        self.current_time += self.dt
        return record

    def run(self, max_duration: float = 20.0) -> Dict[str, Any]:
        """
        持续运行仿真直至超时，返回真实 KPI 报告。
        """
        steps = max(1, int(round(max_duration / self.dt)))
        for _ in range(steps):
            self.step()
        return self.evaluator.compute_summary_report()
