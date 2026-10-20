"""
场景加载器：把 YAML / JSON 场景描述转换为仿真引擎可直接消费的对象图。

设计目标：
1. ``configs/scenarios/*.yaml`` 不再只是“文档”，而是真正被读取的执行配置；
2. 在没有 PyYAML 的最小环境中，仍可用内置的 ``highway_merge`` 场景做闭环测试；
3. 场景中的车道、障碍物、自车初始状态与 IDM 参数统一进入 RoadNetwork /
   ObstacleManager，避免脚本里再硬编码一套与 YAML 不一致的世界。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

from src.common.state import ObstacleState, ObstacleType, VehicleState
from src.environment.obstacle_manager import IntelligentDriverModel, ObstacleManager
from src.environment.road_network import Lane, RoadNetwork

REPO_ROOT = Path(__file__).resolve().parents[2]


DEFAULT_CONFIG: Dict[str, Any] = {
    "simulation": {
        "time_step": 0.05,
        "max_simulation_time": 40.0,
        "random_seed": 42,
        "logging_level": "INFO",
    },
    "ego_vehicle": {
        "length": 4.8,
        "width": 1.9,
        "wheelbase": 2.85,
        "mass": 1600.0,
        "moment_inertia_z": 2500.0,
        "cf": 120000.0,
        "cr": 120000.0,
        "lf": 1.35,
        "lr": 1.50,
        "max_steer_angle": 0.65,
        "max_steer_rate": 0.50,
        "max_accel": 3.5,
        "max_decel": 7.0,
        "max_jerk": 4.0,
        "tire_model": "linear",
        "friction_coefficient": 0.9,
        "cg_height": 0.55,
        "pacejka": {
            "B": 10.0,
            "C": 1.3,
            "D_scale": 0.95,
            "E": -0.97,
            "load_sensitivity": 0.0,
        },
    },
    "sensor_model": {
        "lidar_range": 80.0,
        "lidar_fov": 360.0,
        "noise_std_position": 0.05,
        "noise_std_velocity": 0.10,
    },
    "planning": {
        "planner": "lattice",
        "em": {
            "path_ds": 2.0,
            "path_length": 80.0,
            "speed_dt": 0.2,
            "speed_horizon": 6.0,
            "max_speed": 30.0,
            "safe_gap": 2.0,
            "lateral_clearance": 0.25,
        },
    },
    "control": {
        "controller": "lqr",
        "mpc": {
            "prediction_horizon": 20,
            "control_horizon": 10,
            "q_lateral": 10.0,
            "q_heading": 6.0,
            "q_speed": 1.0,
            "r_accel": 0.08,
            "r_steer": 0.8,
            "r_accel_rate": 0.6,
            "r_steer_rate": 6.0,
            "lateral_error_limit": 0.9,
        },
    },
    "environment": {
        "lane_width": 3.75,
        "speed_limit": 22.22,
    },
}


HIGHWAY_MERGE_DICT: Dict[str, Any] = {
    "scenario_id": "SCENARIO_HIGHWAY_MERGE_01",
    "description": "主车从匝道加速汇入主线车流，需与主线前后来车协同并平滑合流",
    "map": {
        "type": "highway_on_ramp",
        "main_lanes_count": 2,
        "ramp_length": 600.0,
        "road_length": 600.0,
        "lane_width": 3.75,
        "speed_limit": 22.22,
    },
    "ego_initial_state": {
        "x": 0.0,
        "y": -3.75,
        "v": 15.0,
        "heading": 0.0,
        "lane_id": "ramp_0",
    },
    "obstacles": [
        {
            "id": "obs_lead_main",
            "type": "car",
            "initial_state": {"x": 70.0, "y": 0.0, "v": 20.0, "heading": 0.0, "lane_id": "main_lane_0"},
            "behavior_type": "constant_speed",
        },
        {
            "id": "obs_rear_main",
            "type": "car",
            "initial_state": {"x": -15.0, "y": 0.0, "v": 22.0, "heading": 0.0, "lane_id": "main_lane_0"},
            "behavior_type": "idm",
            "idm_params": {"desired_speed": 24.0, "s0": 4.0, "T": 1.5, "a_max": 2.0, "b_desired": 2.5},
        },
        {
            "id": "obs_fast_left",
            "type": "car",
            "initial_state": {"x": 40.0, "y": 3.75, "v": 28.0, "heading": 0.0, "lane_id": "main_lane_1"},
            "behavior_type": "constant_speed",
        },
    ],
    "traffic": {"enable_mobil": True, "lane_width": 3.75},
}


EMERGENCY_AVOIDANCE_DICT: Dict[str, Any] = {
    "scenario_id": "SCENARIO_EMERGENCY_AVOIDANCE_01",
    "description": "主车巡航时前方车道出现静止车辆，验证跟驰减速与紧急制动闭环",
    "map": {
        "type": "highway_straight",
        "main_lanes_count": 2,
        "road_length": 500.0,
        "ramp_length": 1.0,
        "lane_width": 3.75,
        "speed_limit": 22.22,
    },
    "ego_initial_state": {"x": 0.0, "y": 0.0, "v": 20.0, "heading": 0.0, "lane_id": "main_lane_0"},
    "obstacles": [
        {
            "id": "obs_stopped",
            "type": "car",
            "initial_state": {"x": 90.0, "y": 0.0, "v": 0.0, "heading": 0.0, "lane_id": "main_lane_0"},
            "behavior_type": "constant_speed",
        },
        {
            "id": "obs_left_lane",
            "type": "car",
            "initial_state": {"x": 40.0, "y": 3.75, "v": 25.0, "heading": 0.0, "lane_id": "main_lane_1"},
            "behavior_type": "constant_speed",
        },
    ],
}


CUT_IN_DICT: Dict[str, Any] = {
    "scenario_id": "SCENARIO_HIGHWAY_CUT_IN_01",
    "description": "主车在主线巡航，邻道车辆突然近距离向主车道变道切入，验证减速与避撞博弈",
    "map": {
        "type": "highway_straight",
        "main_lanes_count": 2,
        "road_length": 500.0,
        "ramp_length": 1.0,
        "lane_width": 3.75,
        "speed_limit": 22.22,
    },
    "ego_initial_state": {"x": 0.0, "y": 0.0, "v": 20.0, "heading": 0.0, "lane_id": "main_lane_0"},
    "obstacles": [
        {
            "id": "obs_cut_in",
            "type": "car",
            "initial_state": {"x": 25.0, "y": 3.75, "v": 16.0, "heading": 0.0, "lane_id": "main_lane_1"},
            "behavior_type": "scripted_lane_change",
            "lane_change": {"target_y": 0.0, "start_time": 0.5, "duration": 2.5},
        },
        {
            "id": "obs_lead_far",
            "type": "car",
            "initial_state": {"x": 80.0, "y": 0.0, "v": 22.0, "heading": 0.0, "lane_id": "main_lane_0"},
            "behavior_type": "constant_speed",
        },
    ],
    "traffic": {"enable_mobil": False, "lane_width": 3.75},
}


@dataclass
class Scenario:
    """仿真场景的完整执行上下文。"""
    name: str
    description: str
    road_network: RoadNetwork
    obstacle_manager: ObstacleManager
    ego_state: VehicleState
    current_lane_id: str
    dt: float = 0.05
    vehicle_params: Dict[str, Any] = field(default_factory=dict)
    sensor_params: Dict[str, float] = field(default_factory=dict)
    planning_params: Dict[str, Any] = field(default_factory=dict)
    control_params: Dict[str, Any] = field(default_factory=dict)
    seed: int = 42
    max_duration: float = 40.0


def _deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    result = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def load_default_config(path: Optional[str] = None) -> Dict[str, Any]:
    """读取默认配置；PyYAML 不可用时回退到内置默认值。"""
    config = json.loads(json.dumps(DEFAULT_CONFIG))
    candidate = Path(path) if path else REPO_ROOT / "configs" / "default_config.yaml"
    if not candidate.exists():
        return config
    try:
        import yaml  # 延迟导入：轻量环境不强制依赖 PyYAML
    except Exception:
        return config
    with candidate.open("r", encoding="utf-8") as fh:
        loaded = yaml.safe_load(fh) or {}
    return _deep_merge(config, loaded)


def _parse_obstacle_type(value: str) -> ObstacleType:
    mapping = {
        "car": ObstacleType.CAR,
        "truck": ObstacleType.TRUCK,
        "pedestrian": ObstacleType.PEDESTRIAN,
        "bicycle": ObstacleType.BICYCLE,
        "static": ObstacleType.STATIC,
    }
    return mapping.get(str(value).lower(), ObstacleType.CAR)


def _build_road_network(data: Dict[str, Any], config: Dict[str, Any]) -> RoadNetwork:
    map_cfg = data.get("map", {})
    lane_width = float(map_cfg.get("lane_width", config["environment"]["lane_width"]))
    road_length = float(map_cfg.get("road_length", 400.0))
    main_lanes = int(map_cfg.get("main_lanes_count", 2))
    ramp_length = float(map_cfg.get("ramp_length", 150.0))
    speed_limit = float(map_cfg.get("speed_limit", config["environment"]["speed_limit"]))
    map_type = str(map_cfg.get("type", "")).lower()

    # 只有匝道/汇入类地图才应该生成 ramp_0。早期实现无条件给所有场景
    # 加一条与主线等长的“幽灵匝道”，会让直线场景的多余车道参与最近车道查询。
    has_ramp = (
        bool(map_cfg.get("has_ramp", False))
        or "ramp" in map_type
        or map_type in {"merge", "highway_merge"}
    )

    road = RoadNetwork()

    def add_straight_lane(lane_id: str, y: float, length: float, left: Optional[str], right: Optional[str]) -> None:
        pts = [(float(x), float(y)) for x in range(0, int(length) + 1, 2)]
        road.add_lane(
            Lane(
                lane_id=lane_id,
                width=lane_width,
                speed_limit=speed_limit,
                left_lane_id=left,
                right_lane_id=right,
                center_points=pts,
            )
        )

    # 主线车道：main_lane_0 在 y=0，main_lane_1 在 y=lane_width，以此类推
    for i in range(main_lanes):
        lane_id = f"main_lane_{i}"
        left = f"main_lane_{i + 1}" if i + 1 < main_lanes else None
        if i > 0:
            right = f"main_lane_{i - 1}"
        elif has_ramp:
            right = "ramp_0"
        else:
            right = None
        add_straight_lane(lane_id, i * lane_width, road_length, left, right)

    # 匝道：位于主线右侧（y 为负），仅在汇入类地图中生成
    if has_ramp:
        add_straight_lane("ramp_0", -lane_width, ramp_length, None, "main_lane_0")

    for lane_id in road.lanes:
        road.generate_reference_line(lane_id, sample_step=1.0)
    return road


def _build_from_dict(data: Dict[str, Any], config: Dict[str, Any], name: str = "scenario") -> Scenario:
    road = _build_road_network(data, config)

    ego_cfg = data.get("ego_initial_state", {})
    ego_speed = float(ego_cfg.get("v", ego_cfg.get("velocity", 0.0)))
    ego_heading = float(ego_cfg.get("heading", 0.0))
    ego_heading = math.atan2(math.sin(ego_heading), math.cos(ego_heading))
    ego_state = VehicleState(
        x=float(ego_cfg.get("x", 0.0)),
        y=float(ego_cfg.get("y", 0.0)),
        heading=ego_heading,
        velocity=ego_speed,
        vx=ego_speed,
    )
    current_lane_id = str(ego_cfg.get("lane_id", "main_lane_0"))

    traffic_cfg = data.get("traffic", {})
    lane_width = float(data.get("map", {}).get("lane_width", config["environment"]["lane_width"]))
    manager = ObstacleManager(
        enable_mobil=bool(traffic_cfg.get("enable_mobil", False)),
        lane_width=float(traffic_cfg.get("lane_width", lane_width)),
    )
    for item in data.get("obstacles", []):
        state_cfg = item.get("initial_state", {})
        speed = float(state_cfg.get("v", 0.0))
        heading = float(state_cfg.get("heading", 0.0))
        heading = math.atan2(math.sin(heading), math.cos(heading))
        obstacle = ObstacleState(
            id=str(item.get("id", f"obs_{len(manager.obstacles)}")),
            obstacle_type=_parse_obstacle_type(item.get("type", "car")),
            x=float(state_cfg.get("x", 0.0)),
            y=float(state_cfg.get("y", 0.0)),
            length=float(state_cfg.get("length", 4.8)),
            width=float(state_cfg.get("width", 1.9)),
            heading=heading,
            vx=speed * math.cos(heading),
            vy=speed * math.sin(heading),
        )
        idm = None
        if str(item.get("behavior_type", "")).lower() == "idm":
            idm_cfg = item.get("idm_params", {})
            idm = IntelligentDriverModel(
                v0=float(idm_cfg.get("desired_speed", speed)),
                s0=float(idm_cfg.get("s0", 3.0)),
                T=float(idm_cfg.get("T", 1.5)),
                a_max=float(idm_cfg.get("a_max", 2.0)),
                b_desired=float(idm_cfg.get("b_desired", 2.5)),
            )
        manager.add_obstacle(obstacle, idm)
        if str(item.get("behavior_type", "")).lower() == "scripted_lane_change":
            maneuver = item.get("lane_change", {})
            target_lane = maneuver.get("target_lane_id")
            target_y = maneuver.get("target_y")
            if target_y is None and target_lane in road.lanes:
                target_y = road.get_lane_center_y(target_lane)
            if target_y is None:
                raise ValueError(f"Obstacle {obstacle.id} scripted lane change needs target_y or valid target_lane_id")
            manager.schedule_lane_change(
                obstacle.id,
                target_y=float(target_y),
                start_time=float(maneuver.get("start_time", 0.0)),
                duration=float(maneuver.get("duration", 2.5)),
            )
        manager.generate_predicted_trajectory(obstacle.id, horizon=6.0, dt=0.2)

    sim_cfg = config.get("simulation", {})
    sensor_cfg = config.get("sensor_model", {})
    vehicle_cfg = config.get("ego_vehicle", {})

    return Scenario(
        name=name,
        description=str(data.get("description", "")),
        road_network=road,
        obstacle_manager=manager,
        ego_state=ego_state,
        current_lane_id=current_lane_id,
        dt=float(sim_cfg.get("time_step", 0.05)),
        vehicle_params={
            k: float(v)
            if isinstance(v, (int, float))
            else (dict(v) if isinstance(v, dict) else v)
            for k, v in vehicle_cfg.items()
        },
        sensor_params={
            "max_range": float(sensor_cfg.get("lidar_range", 80.0)),
            "fov_deg": float(sensor_cfg.get("lidar_fov", 360.0)),
            "pos_noise_std": float(sensor_cfg.get("noise_std_position", 0.05)),
            "vel_noise_std": float(sensor_cfg.get("noise_std_velocity", 0.10)),
        },
        planning_params=dict(config.get("planning", {})),
        control_params=dict(config.get("control", {})),
        seed=int(sim_cfg.get("random_seed", 42)),
        max_duration=float(sim_cfg.get("max_simulation_time", 40.0)),
    )


def build_highway_merge_scenario() -> Scenario:
    """不依赖 PyYAML 的内置高速汇入场景（用于测试/最小环境）。"""
    return _build_from_dict(HIGHWAY_MERGE_DICT, DEFAULT_CONFIG, name="highway_merge")


def build_emergency_avoidance_scenario() -> Scenario:
    """不依赖 PyYAML 的前方静止车辆紧急制动场景。"""
    return _build_from_dict(EMERGENCY_AVOIDANCE_DICT, DEFAULT_CONFIG, name="emergency_avoidance")


def build_cut_in_scenario() -> Scenario:
    """不依赖 PyYAML 的邻车切入场景。"""
    return _build_from_dict(CUT_IN_DICT, DEFAULT_CONFIG, name="cut_in")


_BUILTIN_SCENARIO_BUILDERS = {
    "highway_merge": build_highway_merge_scenario,
    "emergency_avoidance": build_emergency_avoidance_scenario,
    "cut_in": build_cut_in_scenario,
}


def load_scenario(path: str, config_path: Optional[str] = None) -> Scenario:
    """
    从 YAML/JSON 文件加载场景。

    :param path: 场景文件路径
    :param config_path: 可选默认配置文件路径
    """
    scenario_path = Path(path)
    if not scenario_path.exists():
        raise FileNotFoundError(f"Scenario file not found: {scenario_path}")

    with scenario_path.open("r", encoding="utf-8") as fh:
        if scenario_path.suffix.lower() == ".json":
            data = json.load(fh)
        else:
            try:
                import yaml  # 延迟导入
            except Exception as exc:  # pragma: no cover - 取决于运行环境
                raise RuntimeError(
                    "PyYAML is required to load YAML scenarios; "
                    "install requirements.txt or use the built-in scenario."
                ) from exc
            data = yaml.safe_load(fh) or {}

    config = load_default_config(config_path)
    return _build_from_dict(data, config, name=scenario_path.stem)


def load_named_scenario(name_or_path: str) -> Scenario:
    """
    按名称或路径加载场景。

    - 传入存在的文件路径：直接加载；
    - 传入 ``highway_merge`` / ``emergency_avoidance`` 且 PyYAML 不可用：回退到内置场景；
    - 其他名称：在 ``configs/scenarios/<name>.yaml`` 中查找。
    """
    candidate = Path(name_or_path)
    if candidate.exists():
        return load_scenario(str(candidate))

    builtin_builder = _BUILTIN_SCENARIO_BUILDERS.get(name_or_path)
    scenario_path = REPO_ROOT / "configs" / "scenarios" / f"{name_or_path}.yaml"
    if scenario_path.exists():
        try:
            return load_scenario(str(scenario_path))
        except RuntimeError:
            # PyYAML 缺失时，所有内置场景都应能回退，而不是只有 highway_merge。
            if builtin_builder is not None:
                return builtin_builder()
            raise
    if builtin_builder is not None:
        return builtin_builder()
    raise FileNotFoundError(
        f"Unknown scenario '{name_or_path}'. "
        f"Available built-in: {sorted(_BUILTIN_SCENARIO_BUILDERS)}; "
        f"expected file: {scenario_path}"
    )
