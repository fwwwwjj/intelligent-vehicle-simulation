"""
面向智能车辆行为决策与路径规划的数字建模及仿真系统
China Simulation Innovation Application Competition (SIAC)

采用惰性导入：仅在真正使用 ``SimulationEngine`` 时才加载完整仿真栈，
避免 ``import src.common.state`` 之类的轻量导入被迫拉起 scipy / matplotlib。
"""

from typing import Any

# 与 pyproject.toml 的 [project] version 保持一致；v0.4-core 为当前提交范围。
__version__ = "0.4.0"

__all__ = ["SimulationEngine", "__version__"]


def __getattr__(name: str) -> Any:
    if name == "SimulationEngine":
        from src.simulator import SimulationEngine

        return SimulationEngine
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
