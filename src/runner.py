"""
仿真装配工厂：把一个 Scenario 组装成可直接运行的 SimulationEngine。

脚本、批量评测和导出工具共用同一套装配逻辑，避免每个入口各写一份
“决策器/规划器/控制器参数”，导致实际运行的系统与配置不一致。
"""

from typing import Optional

from src.control.lqr_controller import LQRController
from src.decision.base_decision import BaseDecisionMaker
from src.decision.game_theoretic import GameTheoreticDecisionMaker
from src.decision.rule_based_fsm import HierarchicalFSMDecisionMaker
from src.environment.scenario_loader import Scenario
from src.planning.frenet_lattice import FrenetLatticePlanner
from src.simulator import SimulationEngine


def build_decision_maker(scenario: Scenario, decision: str) -> BaseDecisionMaker:
    if decision == "game":
        return GameTheoreticDecisionMaker(road_network=scenario.road_network)
    if decision == "fsm":
        return HierarchicalFSMDecisionMaker(road_network=scenario.road_network)
    raise ValueError(f"Unknown decision type: {decision}")


def build_controller(scenario: Scenario) -> LQRController:
    vp = scenario.vehicle_params
    return LQRController(
        wheelbase=vp.get("wheelbase", 2.85),
        cf=vp.get("cf", 120000.0),
        cr=vp.get("cr", 120000.0),
        mass=vp.get("mass", 1600.0),
        iz=vp.get("moment_inertia_z", 2500.0),
        lf=vp.get("lf", 1.35),
        lr=vp.get("lr", 1.50),
        max_steer=vp.get("max_steer_angle", 0.65),
        max_steer_rate=vp.get("max_steer_rate", 0.50),
        max_accel=vp.get("max_accel", 3.5),
        max_decel=vp.get("max_decel", 7.0),
        max_jerk=vp.get("max_jerk", 4.0),
    )


def build_planner(scenario: Scenario) -> FrenetLatticePlanner:
    vp = scenario.vehicle_params
    return FrenetLatticePlanner(
        ego_length=vp.get("length", 4.8),
        ego_width=vp.get("width", 1.9),
        max_accel=vp.get("max_accel", 3.5),
        max_decel=vp.get("max_decel", 7.0),
    )


def build_engine(
    scenario: Scenario,
    decision: str = "fsm",
    seed: Optional[int] = None,
    enable_visualization: bool = False,
    use_dynamic_model: bool = False,
) -> SimulationEngine:
    """按场景配置装配闭环仿真引擎。"""
    return SimulationEngine(
        road_network=scenario.road_network,
        obstacle_manager=scenario.obstacle_manager,
        decision_maker=build_decision_maker(scenario, decision),
        planner=build_planner(scenario),
        controller=build_controller(scenario),
        ego_initial_state=scenario.ego_state,
        use_dynamic_model=use_dynamic_model,
        enable_visualization=enable_visualization,
        dt=scenario.dt,
        seed=scenario.seed if seed is None else seed,
        vehicle_params=scenario.vehicle_params,
        sensor_params=scenario.sensor_params,
        initial_lane_id=scenario.current_lane_id,
    )
