"""
仿真装配工厂：把一个 Scenario 组装成可直接运行的 SimulationEngine。

脚本、批量评测和导出工具共用同一套装配逻辑，避免每个入口各写一份
“决策器/规划器/控制器参数”，导致实际运行的系统与配置不一致。

v0.3 起支持算法可选装配：
- ``planner="lattice" | "em"``；
- ``controller="lqr" | "mpc"``；
- ``tire_model="linear" | "pacejka"``（Pacejka 会自动启用 2-DOF 动力学）。
"""

from typing import Optional

from src.control.base_controller import BaseController
from src.control.lqr_controller import LQRController
from src.control.mpc_controller import MPCController
from src.control.pure_pursuit_controller import PurePursuitController
from src.decision.base_decision import BaseDecisionMaker
from src.decision.game_theoretic import GameTheoreticDecisionMaker
from src.decision.idm_decision import IDMDecisionMaker
from src.decision.rule_based_fsm import HierarchicalFSMDecisionMaker
from src.environment.scenario_loader import Scenario
from src.planning.base_planner import BasePlanner
from src.planning.centerline_planner import CenterlinePlanner
from src.planning.em_planner import EMPlanner
from src.planning.frenet_lattice import FrenetLatticePlanner
from src.simulator import SimulationEngine


def _vehicle_params(scenario: Scenario) -> dict:
    return dict(scenario.vehicle_params or {})


def _lane_half_width(scenario: Scenario, default_lane_width: float = 3.75) -> float:
    """从路网读取真实车道半宽，供规划器限制横向候选范围。"""
    lanes = getattr(scenario.road_network, "lanes", None) or {}
    widths = sorted(float(lane.width) for lane in lanes.values() if float(lane.width) > 0.0)
    lane_width = widths[len(widths) // 2] if widths else float(default_lane_width)
    return 0.5 * lane_width


def build_decision_maker(scenario: Scenario, decision: str) -> BaseDecisionMaker:
    vp = _vehicle_params(scenario)
    ego_length = float(vp.get("length", 4.8))
    ego_width = float(vp.get("width", 1.9))
    if decision == "game":
        return GameTheoreticDecisionMaker(
            road_network=scenario.road_network,
            ego_length=ego_length,
            ego_width=ego_width,
        )
    if decision == "fsm":
        return HierarchicalFSMDecisionMaker(
            road_network=scenario.road_network,
            ego_length=ego_length,
            ego_width=ego_width,
        )
    if decision == "fsm_no_lane_change":
        # 消融：保留规则跟驰/紧急制动，但关闭汇入/换道决策，用于量化战术决策层贡献。
        return HierarchicalFSMDecisionMaker(
            road_network=scenario.road_network,
            ego_length=ego_length,
            ego_width=ego_width,
            enable_lane_change=False,
        )
    if decision == "idm":
        # 经典基线：IDM 自适应巡航 + 本车道保持，不做战术换道。
        return IDMDecisionMaker(
            road_network=scenario.road_network,
            ego_length=ego_length,
            ego_width=ego_width,
        )
    raise ValueError(f"Unknown decision type: {decision}")


def build_controller(
    scenario: Scenario, controller: Optional[str] = None
) -> BaseController:
    """
    按场景配置或显式参数装配控制器。

    :param scenario: 场景上下文
    :param controller: ``"lqr"`` / ``"mpc"``；缺省读取 ``scenario.control_params``
    """
    vp = _vehicle_params(scenario)
    params = dict(scenario.control_params or {})
    name = str(controller or params.get("controller", "lqr")).lower()

    common = dict(
        wheelbase=float(vp.get("wheelbase", 2.85)),
        cf=float(vp.get("cf", 120000.0)),
        cr=float(vp.get("cr", 120000.0)),
        mass=float(vp.get("mass", 1600.0)),
        iz=float(vp.get("moment_inertia_z", 2500.0)),
        lf=float(vp.get("lf", 1.35)),
        lr=float(vp.get("lr", 1.50)),
        max_steer=float(vp.get("max_steer_angle", 0.65)),
        max_steer_rate=float(vp.get("max_steer_rate", 0.50)),
        max_accel=float(vp.get("max_accel", 3.5)),
        max_decel=float(vp.get("max_decel", 7.0)),
        max_jerk=float(vp.get("max_jerk", 4.0)),
    )

    if name == "lqr":
        return LQRController(**common)

    if name == "pure_pursuit":
        return PurePursuitController(
            wheelbase=common["wheelbase"],
            max_steer=common["max_steer"],
            max_steer_rate=common["max_steer_rate"],
            max_accel=common["max_accel"],
            max_decel=common["max_decel"],
            max_jerk=common["max_jerk"],
        )

    if name == "mpc":
        mpc_cfg = dict(params.get("mpc", {}))
        return MPCController(
            prediction_horizon=int(mpc_cfg.get("prediction_horizon", 20)),
            control_horizon=int(mpc_cfg.get("control_horizon", 10)),
            q_lateral=float(mpc_cfg.get("q_lateral", 10.0)),
            q_heading=float(mpc_cfg.get("q_heading", 6.0)),
            q_speed=float(mpc_cfg.get("q_speed", 1.0)),
            r_accel=float(mpc_cfg.get("r_accel", 0.08)),
            r_steer=float(mpc_cfg.get("r_steer", 0.8)),
            r_accel_rate=float(mpc_cfg.get("r_accel_rate", 0.6)),
            r_steer_rate=float(mpc_cfg.get("r_steer_rate", 6.0)),
            lateral_error_limit=float(mpc_cfg.get("lateral_error_limit", 0.9)),
            heading_error_limit=float(mpc_cfg.get("heading_error_limit", 0.0)),
            slack_weight=float(mpc_cfg.get("slack_weight", 2.0e3)),
            slack_linear_weight=float(mpc_cfg.get("slack_linear_weight", 2.0e4)),
            qp_max_iter=int(mpc_cfg.get("qp_max_iter", 2500)),
            **common,
        )

    raise ValueError(f"Unknown controller type: {name}")


def build_planner(scenario: Scenario, planner: Optional[str] = None) -> BasePlanner:
    """
    按场景配置或显式参数装配规划器。

    :param scenario: 场景上下文
    :param planner: ``"lattice"`` / ``"em"``；缺省读取 ``scenario.planning_params``
    """
    vp = _vehicle_params(scenario)
    params = dict(scenario.planning_params or {})
    name = str(planner or params.get("planner", "lattice")).lower()

    if name == "lattice":
        return FrenetLatticePlanner(
            ego_length=float(vp.get("length", 4.8)),
            ego_width=float(vp.get("width", 1.9)),
            max_accel=float(vp.get("max_accel", 3.5)),
            max_decel=float(vp.get("max_decel", 7.0)),
            lane_half_width=_lane_half_width(scenario),
        )

    if name == "centerline":
        return CenterlinePlanner(
            ego_length=float(vp.get("length", 4.8)),
            ego_width=float(vp.get("width", 1.9)),
            max_accel=float(vp.get("max_accel", 3.5)),
            max_decel=float(vp.get("max_decel", 7.0)),
        )

    if name == "em":
        em_cfg = dict(params.get("em", {}))
        return EMPlanner(
            ego_length=float(vp.get("length", 4.8)),
            ego_width=float(vp.get("width", 1.9)),
            lane_width=float(em_cfg.get("lane_width", 3.75)),
            max_accel=float(vp.get("max_accel", 3.5)),
            max_decel=float(vp.get("max_decel", 7.0)),
            max_jerk=float(vp.get("max_jerk", 4.0)),
            path_ds=float(em_cfg.get("path_ds", 2.0)),
            path_length=float(em_cfg.get("path_length", 80.0)),
            speed_dt=float(em_cfg.get("speed_dt", 0.2)),
            speed_horizon=float(em_cfg.get("speed_horizon", 6.0)),
            max_speed=float(em_cfg.get("max_speed", 30.0)),
            safe_gap=float(em_cfg.get("safe_gap", 2.0)),
            lateral_clearance=float(em_cfg.get("lateral_clearance", 0.25)),
            path_weights=em_cfg.get("path_weights"),
            speed_weights=em_cfg.get("speed_weights"),
            qp_max_iter=int(em_cfg.get("qp_max_iter", 2500)),
        )

    raise ValueError(f"Unknown planner type: {name}")


def build_engine(
    scenario: Scenario,
    decision: str = "fsm",
    seed: Optional[int] = None,
    enable_visualization: bool = False,
    use_dynamic_model: bool = False,
    planner: Optional[str] = None,
    controller: Optional[str] = None,
    tire_model: Optional[str] = None,
) -> SimulationEngine:
    """按场景配置与算法选择装配闭环仿真引擎。"""
    vp = _vehicle_params(scenario)
    resolved_tire = str(tire_model or vp.get("tire_model", "linear")).lower()
    resolved_decision = str(decision or "fsm").lower()
    resolved_planner = str(
        planner or (scenario.planning_params or {}).get("planner", "lattice")
    ).lower()
    resolved_controller = str(
        controller or (scenario.control_params or {}).get("controller", "lqr")
    ).lower()
    if resolved_tire == "pacejka":
        use_dynamic_model = True

    experimental_reasons = []
    if resolved_decision == "game":
        experimental_reasons.append("game_decision")
    if resolved_planner == "em":
        experimental_reasons.append("em_planner")
    if resolved_controller == "mpc":
        experimental_reasons.append("mpc_controller")
    if resolved_tire == "pacejka":
        experimental_reasons.append("pacejka_tire")
    if str(scenario.name).lower() == "intersection_unprotected":
        experimental_reasons.append("intersection_unprotected")

    return SimulationEngine(
        road_network=scenario.road_network,
        obstacle_manager=scenario.obstacle_manager,
        # 必须使用上面的 resolved_* （已处理 None 与场景默认值），否则
        # build_engine(decision=None) 会走到 "Unknown decision type" 分支。
        decision_maker=build_decision_maker(scenario, resolved_decision),
        planner=build_planner(scenario, resolved_planner),
        controller=build_controller(scenario, resolved_controller),
        ego_initial_state=scenario.ego_state,
        use_dynamic_model=use_dynamic_model,
        enable_visualization=enable_visualization,
        dt=scenario.dt,
        seed=scenario.seed if seed is None else seed,
        vehicle_params=scenario.vehicle_params,
        sensor_params=scenario.sensor_params,
        initial_lane_id=scenario.current_lane_id,
        tire_model=resolved_tire,
        scenario_name=scenario.name,
        experimental=bool(experimental_reasons),
        experimental_reasons=experimental_reasons,
    )


__all__ = [
    "build_decision_maker",
    "build_controller",
    "build_planner",
    "build_engine",
]
