"""
仿真评估量化指标与 KPI 统计分析体系。
完全对标中国仿真创新应用大赛对于“结果演示分析与科学量化评估”的要求。
涵盖安全性 (Safety)、舒适性 (Comfort)、通行效率 (Efficiency) 与实时计算性能 (Real-time Latency)。
"""

from dataclasses import dataclass, field
from typing import List, Dict
import numpy as np


@dataclass
class SimulationMetricsRecord:
    """单步仿真诊断数据快照"""
    timestamp: float
    velocity: float
    acceleration: float
    jerk: float
    lateral_error: float
    heading_error: float
    min_obstacle_distance: float
    min_ttc: float
    planning_latency_ms: float
    is_collision: bool = False


class PerformanceEvaluator:
    """
    仿真指标综合计算器
    """
    def __init__(self):
        self.records: List[SimulationMetricsRecord] = []

    def record_step(self, step_record: SimulationMetricsRecord) -> None:
        """记录单步指标"""
        self.records.append(step_record)

    def compute_summary_report(self) -> Dict[str, float]:
        """
        生成全生命周期综合 KPI 评分报告
        """
        if not self.records:
            return {}

        jerks = [abs(r.jerk) for r in self.records]
        accels = [abs(r.acceleration) for r in self.records]
        lat_errors = [abs(r.lateral_error) for r in self.records]
        dists = [r.min_obstacle_distance for r in self.records]
        ttcs = [r.min_ttc for r in self.records if r.min_ttc > 0 and r.min_ttc < 50.0]
        latencies = [r.planning_latency_ms for r in self.records]
        has_collision = any(r.is_collision for r in self.records)

        return {
            "success_rate": 0.0 if has_collision else 1.0,
            "min_obstacle_distance_m": float(np.min(dists)) if dists else 0.0,
            "min_ttc_s": float(np.min(ttcs)) if ttcs else 999.0,
            "rms_jerk_m_s3": float(np.sqrt(np.mean(np.square(jerks)))) if jerks else 0.0,
            "max_lateral_accel_m_s2": float(np.max(accels)) if accels else 0.0,
            "mean_lateral_error_m": float(np.mean(lat_errors)) if lat_errors else 0.0,
            "max_lateral_error_m": float(np.max(lat_errors)) if lat_errors else 0.0,
            "avg_planning_latency_ms": float(np.mean(latencies)) if latencies else 0.0,
            "max_planning_latency_ms": float(np.max(latencies)) if latencies else 0.0,
        }
