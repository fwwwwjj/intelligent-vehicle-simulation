#!/usr/bin/env python3
"""
比赛图表与实验对比数据导出脚本。
从真实闭环仿真日志生成车速、横向误差、jerk 曲线，不再使用合成指数曲线。
"""

import argparse
import json
import os
import sys
from dataclasses import asdict
from pathlib import Path

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.environment.scenario_loader import load_named_scenario
from src.runner import build_engine
from src.visualization.plot_utils import ScientificPlotter


def main():
    parser = argparse.ArgumentParser(description="导出真实仿真遥测图表")
    parser.add_argument("--scenario", type=str, default="highway_merge")
    parser.add_argument("--decision", type=str, default="fsm", choices=["fsm", "game"])
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--seed", type=int, default=None)
    args = parser.parse_args()

    scenario = load_named_scenario(args.scenario)
    engine = build_engine(
        scenario,
        decision=args.decision,
        seed=args.seed,
        enable_visualization=False,
    )
    report = engine.run(max_duration=args.duration)
    records = engine.evaluator.records
    if not records:
        raise RuntimeError("No telemetry records generated")

    out_dir = Path("results")
    out_dir.mkdir(parents=True, exist_ok=True)
    times = [r.timestamp for r in records]
    speeds = [r.velocity for r in records]
    lateral_errors = [r.lateral_error for r in records]
    jerks = [r.jerk for r in records]

    output_path = out_dir / "telemetry_analysis.png"
    ScientificPlotter.plot_telemetry_curves(
        times, speeds, lateral_errors, jerks, str(output_path)
    )
    with (out_dir / "telemetry_analysis.json").open("w", encoding="utf-8") as fh:
        json.dump(
            {
                "scenario": scenario.name,
                "decision": args.decision,
                "report": report,
                "records": [asdict(r) for r in records],
            },
            fh,
            ensure_ascii=False,
            indent=2,
        )

    print(f">>> 遥测图表已保存: {output_path}")
    print(f">>> KPI: success={report['success_rate']}, "
          f"minD={report['min_obstacle_distance_m']:.2f}m, "
          f"RMS jerk={report['rms_jerk_m_s3']:.2f}m/s^3, "
          f"avg plan={report['avg_planning_latency_ms']:.1f}ms")


if __name__ == "__main__":
    main()
