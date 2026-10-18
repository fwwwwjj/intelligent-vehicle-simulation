"""
面向智能车辆行为决策与路径规划的数字建模及仿真系统
China Simulation Innovation Application Competition (SIAC)

采用惰性导入：仅在真正使用 ``SimulationEngine`` 时才加载完整仿真栈，
避免 ``import src.common.state`` 之类的轻量导入被迫拉起 scipy / matplotlib。
"""

from typing import Any

__all__ = ["SimulationEngine"]


def __getattr__(name: str) -> Any:
    if name == "SimulationEngine":
        from src.simulator import SimulationEngine

        return SimulationEngine
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
