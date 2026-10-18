"""
可视化与结果分析模块。

惰性导入 matplotlib，保证 headless / 无图形依赖环境也能运行闭环仿真。
"""

from typing import Any

__all__ = ["Visualizer2D", "ScientificPlotter"]


def __getattr__(name: str) -> Any:
    if name == "Visualizer2D":
        from src.visualization.visualizer_2d import Visualizer2D

        return Visualizer2D
    if name == "ScientificPlotter":
        from src.visualization.plot_utils import ScientificPlotter

        return ScientificPlotter
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
