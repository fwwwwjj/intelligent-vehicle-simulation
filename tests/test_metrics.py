"""
KPI 聚合口径测试。

重点锁定两个容易混淆的字段：
- ``collision_count``：逐帧碰撞障碍物数量之和（一帧两车计 2）；
- ``collision_frames``：发生碰撞的帧数（一帧两车仍只计 1）。
"""

import unittest
import csv
import json
import tempfile

import numpy as np

from src.evaluation.metrics import PerformanceEvaluator, SimulationMetricsRecord
from src.evaluation.benchmark import BenchmarkSuite


def _record(
    timestamp: float,
    *,
    acceleration: float = 0.0,
    jerk: float = 0.0,
    min_distance: float = 10.0,
    min_ttc: float = 999.0,
    is_collision: bool = False,
    collision_count: int = 0,
    lateral_error: float = 0.0,
    heading_error: float = 0.0,
    planning_ms: float = 2.0,
) -> SimulationMetricsRecord:
    return SimulationMetricsRecord(
        timestamp=timestamp,
        velocity=10.0,
        acceleration=acceleration,
        jerk=jerk,
        lateral_error=lateral_error,
        heading_error=heading_error,
        min_obstacle_distance=min_distance,
        min_ttc=min_ttc,
        planning_latency_ms=planning_ms,
        is_collision=is_collision,
        collision_count=collision_count,
        decision_latency_ms=0.1,
        control_latency_ms=0.2,
        distance_traveled=timestamp * 10.0,
    )


class TestPerformanceEvaluator(unittest.TestCase):

    def test_collision_count_and_frames_have_distinct_semantics(self):
        evaluator = PerformanceEvaluator()
        evaluator.record_step(_record(
            0.05, acceleration=1.0, jerk=2.0, min_distance=10.0, min_ttc=5.0,
            is_collision=True, collision_count=1, lateral_error=0.1,
        ))
        evaluator.record_step(_record(
            0.10, acceleration=-1.0, jerk=-4.0, min_distance=0.5, min_ttc=0.0,
            is_collision=True, collision_count=2,
        ))
        evaluator.record_step(_record(0.15))

        report = evaluator.compute_summary_report()
        self.assertEqual(report["success_rate"], 0.0)
        self.assertEqual(report["collision_count"], 3.0)
        self.assertEqual(report["collision_frames"], 2.0)
        self.assertAlmostEqual(report["collision_rate"], 2.0 / 3.0)
        self.assertAlmostEqual(report["min_obstacle_distance_m"], 0.5)
        self.assertAlmostEqual(report["min_ttc_s"], 5.0)
        self.assertAlmostEqual(report["rms_jerk_m_s3"], float(np.sqrt(20.0 / 3.0)))
        self.assertAlmostEqual(report["max_accel_m_s2"], 1.0)
        self.assertAlmostEqual(report["max_decel_m_s2"], -1.0)
        self.assertAlmostEqual(report["mean_lateral_error_m"], 0.1 / 3.0)
        self.assertAlmostEqual(report["duration_s"], 0.15)

    def test_no_collision_reports_success(self):
        evaluator = PerformanceEvaluator()
        evaluator.record_step(_record(0.05, min_distance=12.0, min_ttc=8.0))
        evaluator.record_step(_record(0.10, min_distance=6.0, min_ttc=4.0))

        report = evaluator.compute_summary_report()
        self.assertEqual(report["success_rate"], 1.0)
        self.assertEqual(report["collision_count"], 0.0)
        self.assertEqual(report["collision_frames"], 0.0)
        self.assertEqual(report["collision_rate"], 0.0)
        self.assertAlmostEqual(report["min_obstacle_distance_m"], 6.0)
        self.assertAlmostEqual(report["min_ttc_s"], 4.0)

    def test_empty_evaluator_returns_empty_report(self):
        self.assertEqual(PerformanceEvaluator().compute_summary_report(), {})

    def test_reset_clears_records(self):
        evaluator = PerformanceEvaluator()
        evaluator.record_step(_record(0.05))
        evaluator.reset()
        self.assertEqual(evaluator.records, [])
        self.assertEqual(evaluator.compute_summary_report(), {})


class TestBenchmarkSummary(unittest.TestCase):
    def test_group_summary_reports_sample_spread_and_success_interval(self):
        suite = BenchmarkSuite(["merge"], decisions=["fsm"], seeds=[1, 2, 3])
        suite.results = {
            "a": {"scenario": "merge", "decision": "fsm", "seed": 1, "success_rate": 1.0, "collision_count": 0.0, "mean_lateral_error_m": 1.0},
            "b": {"scenario": "merge", "decision": "fsm", "seed": 2, "success_rate": 0.0, "collision_count": 1.0, "mean_lateral_error_m": 3.0},
            "c": {"scenario": "merge", "decision": "fsm", "seed": 3, "success_rate": 1.0, "collision_count": 0.0, "mean_lateral_error_m": 2.0},
        }
        summary = suite.summarize_groups()[0]
        self.assertEqual(summary["sample_count"], 3)
        self.assertEqual(summary["success_count"], 2)
        self.assertAlmostEqual(summary["success_rate_mean"], 2.0 / 3.0)
        self.assertLess(summary["success_rate_ci95_low"], 2.0 / 3.0)
        self.assertGreater(summary["success_rate_ci95_high"], 2.0 / 3.0)
        self.assertAlmostEqual(summary["mean_lateral_error_m_mean"], 2.0)
        self.assertAlmostEqual(summary["mean_lateral_error_m_std"], 1.0)

    def test_save_writes_raw_runs_and_group_summary(self):
        suite = BenchmarkSuite(["merge"], decisions=["fsm"], seeds=[42], duration=2.0)
        suite.results = {
            "merge__fsm__seed42": {
                "scenario": "merge", "decision": "fsm", "seed": 42,
                "success_rate": 1.0, "mean_lateral_error_m": 0.1,
            }
        }
        with tempfile.TemporaryDirectory() as directory:
            json_path, raw_csv, summary_csv = suite.save(directory)
            with open(json_path, encoding="utf-8") as fh:
                payload = json.load(fh)
            self.assertIn("merge__fsm__seed42", payload["runs"])
            self.assertEqual(payload["metadata"]["seeds"], [42])
            with open(raw_csv, encoding="utf-8", newline="") as fh:
                self.assertEqual(len(list(csv.DictReader(fh))), 1)
            with open(summary_csv, encoding="utf-8", newline="") as fh:
                self.assertEqual(len(list(csv.DictReader(fh))), 1)


if __name__ == "__main__":
    unittest.main()
