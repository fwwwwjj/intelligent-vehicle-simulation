"""
仿真评估量化指标与 KPI 统计分析体系。
完全对标中国仿真创新应用大赛对于“结果演示分析与科学量化评估”的要求。
涵盖安全性 (Safety)、舒适性 (Comfort)、跟踪精度 (Tracking)、通行效率 (Efficiency)
与实时计算性能 (Real-time Latency)。

所有指标都来自逐步记录的真实状态，不再使用硬编码常量。
"""

from dataclasses import dataclass
from typing import Dict, List

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
    lateral_accel: float = 0.0
    decision_latency_ms: float = 0.0
    control_latency_ms: float = 0.0
    distance_traveled: float = 0.0
    collision_count: int = 0


class PerformanceEvaluator:
    """
    仿真指标综合计算器
    """
    def __init__(self):
        self.records: List[SimulationMetricsRecord] = []

    def record_step(self, step_record: SimulationMetricsRecord) -> None:
        """记录单步指标"""
        self.records.append(step_record)

    def reset(self) -> None:
        self.records.clear()

    def compute_summary_report(self) -> Dict[str, float]:
        """
        生成全生命周期综合 KPI 评分报告。
        """
        if not self.records:
            return {}

        jerks = [abs(r.jerk) for r in self.records]
        accels = [r.acceleration for r in self.records]
        lateral_accels = [abs(r.lateral_accel) for r in self.records]
        lat_errors = [abs(r.lateral_error) for r in self.records]
        heading_errors = [abs(r.heading_error) for r in self.records]
        dists = [r.min_obstacle_distance for r in self.records]
        ttcs = [r.min_ttc for r in self.records if 0.0 < r.min_ttc < 50.0]
        latencies = [r.planning_latency_ms for r in self.records]
        decision_latencies = [r.decision_latency_ms for r in self.records]
        control_latencies = [r.control_latency_ms for r in self.records]
        # 口径区分：collision_count 是“碰撞障碍物数量之和”（可能一帧多车），
        # collision_frames 是“发生碰撞的帧数”（可能一帧多车只计一次）。
        collision_obstacle_count = sum(r.collision_count for r in self.records)
        collision_frame_count = sum(1 for r in self.records if r.is_collision)

        return {
            "success_rate": 0.0 if collision_obstacle_count > 0 else 1.0,
            "collision_count": float(collision_obstacle_count),
            "collision_frames": float(collision_frame_count),
            "collision_rate": float(collision_frame_count / len(self.records)),
            "min_obstacle_distance_m": float(np.min(dists)) if dists else 999.0,
            "min_ttc_s": float(np.min(ttcs)) if ttcs else 999.0,
            "rms_jerk_m_s3": float(np.sqrt(np.mean(np.square(jerks)))),
            "max_accel_m_s2": float(np.max(accels)),
            "max_decel_m_s2": float(np.min(accels)),
            "max_lateral_accel_m_s2": float(np.max(lateral_accels)),
            "mean_lateral_error_m": float(np.mean(lat_errors)),
            "max_lateral_error_m": float(np.max(lat_errors)),
            "mean_heading_error_rad": float(np.mean(heading_errors)),
            "avg_planning_latency_ms": float(np.mean(latencies)),
            "max_planning_latency_ms": float(np.max(latencies)),
            "avg_decision_latency_ms": float(np.mean(decision_latencies)),
            "avg_control_latency_ms": float(np.mean(control_latencies)),
            "distance_traveled_m": float(self.records[-1].distance_traveled),
            "duration_s": float(self.records[-1].timestamp),
        }
