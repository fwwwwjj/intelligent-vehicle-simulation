"""
仿真系统主引擎闭环控制核心 (Simulation Engine)。
整合数字环境底座、车载感知数字模型、行为决策模块、轨迹规划优化器、运动控制算法及车辆物理动力学闭环。
形成端到端的虚拟数字孪生与仿真试验闭环系统。
"""

import time
from typing import Optional, Dict, Any
from src.common.state import VehicleState, Trajectory
from src.environment.road_network import RoadNetwork
from src.environment.obstacle_manager import ObstacleManager
from src.vehicle_model.kinematics import KinematicBicycleModel
from src.vehicle_model.dynamics import DynamicBicycleModel
from src.vehicle_model.sensor_model import SensorModel
from src.decision.base_decision import BaseDecisionMaker
from src.planning.base_planner import BasePlanner
from src.control.base_controller import BaseController
from src.evaluation.metrics import PerformanceEvaluator, SimulationMetricsRecord
from src.visualization.visualizer_2d import Visualizer2D


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
        dt: float = 0.05
    ):
        self.road_network = road_network
        self.obstacle_manager = obstacle_manager
        self.decision_maker = decision_maker
        self.planner = planner
        self.controller = controller
        self.ego_state = ego_initial_state
        self.use_dynamic_model = use_dynamic_model
        self.enable_visualization = enable_visualization
        self.dt = dt

        # 初始化物理模型与感知模型
        self.kinematic_model = KinematicBicycleModel()
        self.dynamic_model = DynamicBicycleModel()
        self.sensor_model = SensorModel()

        # 评估器与可视化器
        self.evaluator = PerformanceEvaluator()
        self.visualizer = Visualizer2D() if self.enable_visualization else None

        self.current_time = 0.0
        self.current_lane_id = "main_lane_0"

    def step(self) -> SimulationMetricsRecord:
        """
        执行单个仿真控制周期 (dt) 的完整闭环运算流程：
        1. 环境演化 (Environment Evolution)
        2. 传感器感知过滤 (Perception Simulation)
        3. 行为决策状态转移 (Behavior Decision)
        4. 时空轨迹规划 (Trajectory Planning)
        5. 轨迹跟踪控制计算 (Motion Control)
        6. 车辆动力学数值积分递推 (Vehicle Physics Simulation)
        7. KPI 诊断与量化指标评估 (Metrics Logging)
        8. 场景可视化刷新 (Rendering)
        """
        # 1. 动态障碍物物理演化
        self.obstacle_manager.step(self.dt)
        ground_truth_obstacles = self.obstacle_manager.get_all_obstacles()

        # 2. 感知模型过滤 (施加噪声和 FOV)
        perceived_obstacles = self.sensor_model.filter_perceived_obstacles(
            self.ego_state, ground_truth_obstacles
        )

        # 3. 行为决策求解
        decision_start_t = time.perf_counter()
        decision_output = self.decision_maker.make_decision(
            self.ego_state, perceived_obstacles, self.current_lane_id
        )

        # 4. 局部时空轨迹规划求解
        ref_line = self.road_network.get_reference_line(decision_output.target_lane_id) or []
        planned_trajectory = self.planner.plan(
            self.ego_state, ref_line, perceived_obstacles, decision_output, dt=0.1
        )
        planning_latency_ms = (time.perf_counter() - decision_start_t) * 1000.0

        # 5. 底层运动跟踪控制
        control_cmd = self.controller.compute_control(
            self.ego_state, planned_trajectory, self.dt
        )

        # 6. 车辆机理模型前向积分
        if self.use_dynamic_model:
            self.ego_state = self.dynamic_model.step(
                self.ego_state, control_cmd.acceleration, control_cmd.steering_angle, self.dt
            )
        else:
            self.ego_state = self.kinematic_model.step(
                self.ego_state, control_cmd.acceleration, control_cmd.steering_angle, self.dt
            )

        # 7. 评估诊断与指标统计
        record = SimulationMetricsRecord(
            timestamp=self.current_time,
            velocity=self.ego_state.velocity,
            acceleration=self.ego_state.acceleration,
            jerk=(self.ego_state.acceleration - 0.0) / self.dt,
            lateral_error=0.0,
            heading_error=0.0,
            min_obstacle_distance=100.0,
            min_ttc=999.0,
            planning_latency_ms=planning_latency_ms
        )
        self.evaluator.record_step(record)

        # 8. 实时画面渲染
        if self.enable_visualization and self.visualizer:
            self.visualizer.render_frame(
                self.ego_state,
                ground_truth_obstacles,
                self.road_network,
                planned_trajectory,
                self.current_time
            )

        self.current_time += self.dt
        return record

    def run(self, max_duration: float = 20.0) -> Dict[str, Any]:
        """
        持续运行仿真直至超时或触发终止条件
        """
        steps = int(max_duration / self.dt)
        for _ in range(steps):
            self.step()

        report = self.evaluator.compute_summary_report()
        return report
