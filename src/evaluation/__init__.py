"""
量化评估与性能基准评测模块。

惰性导入 benchmark，避免 simulator -> evaluation.metrics 的导入链
被 benchmark -> simulator 反向依赖打断。
"""

from typing import Any

__all__ = ["SimulationMetricsRecord", "PerformanceEvaluator", "BenchmarkSuite"]


def __getattr__(name: str) -> Any:
    if name == "SimulationMetricsRecord":
        from src.evaluation.metrics import SimulationMetricsRecord

        return SimulationMetricsRecord
    if name == "PerformanceEvaluator":
        from src.evaluation.metrics import PerformanceEvaluator

        return PerformanceEvaluator
    if name == "BenchmarkSuite":
        from src.evaluation.benchmark import BenchmarkSuite

        return BenchmarkSuite
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
