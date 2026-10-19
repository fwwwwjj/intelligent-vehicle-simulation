"""
仿真环境、道路网络及数据源接口模块。

采用惰性导入，避免仅使用 RoadNetwork 时被迫加载 pandas 数据集解析器。
"""

from typing import Any

__all__ = [
    "Lane",
    "RoadNetwork",
    "ObstacleManager",
    "IntelligentDriverModel",
    "MOBILLaneChangeModel",
    "BaseDatasetLoader",
    "HighDDatasetLoader",
    "InteractionDatasetLoader",
    "OpenScenarioParser",
    "Scenario",
    "load_scenario",
    "load_named_scenario",
    "build_highway_merge_scenario",
    "build_emergency_avoidance_scenario",
    "build_cut_in_scenario",
    "IDMCalibrator",
]

_LAZY = {
    "Lane": ("src.environment.road_network", "Lane"),
    "RoadNetwork": ("src.environment.road_network", "RoadNetwork"),
    "ObstacleManager": ("src.environment.obstacle_manager", "ObstacleManager"),
    "IntelligentDriverModel": ("src.environment.obstacle_manager", "IntelligentDriverModel"),
    "MOBILLaneChangeModel": ("src.environment.obstacle_manager", "MOBILLaneChangeModel"),
    "BaseDatasetLoader": ("src.environment.dataset_loader", "BaseDatasetLoader"),
    "HighDDatasetLoader": ("src.environment.dataset_loader", "HighDDatasetLoader"),
    "InteractionDatasetLoader": ("src.environment.dataset_loader", "InteractionDatasetLoader"),
    "OpenScenarioParser": ("src.environment.dataset_loader", "OpenScenarioParser"),
    "Scenario": ("src.environment.scenario_loader", "Scenario"),
    "load_scenario": ("src.environment.scenario_loader", "load_scenario"),
    "load_named_scenario": ("src.environment.scenario_loader", "load_named_scenario"),
    "build_highway_merge_scenario": ("src.environment.scenario_loader", "build_highway_merge_scenario"),
    "build_emergency_avoidance_scenario": ("src.environment.scenario_loader", "build_emergency_avoidance_scenario"),
    "build_cut_in_scenario": ("src.environment.scenario_loader", "build_cut_in_scenario"),
    "IDMCalibrator": ("src.environment.calibration", "IDMCalibrator"),
}


def __getattr__(name: str) -> Any:
    if name in _LAZY:
        module_name, attr = _LAZY[name]
        from importlib import import_module

        return getattr(import_module(module_name), attr)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
