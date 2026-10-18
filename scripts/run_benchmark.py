#!/usr/bin/env python3
"""
批量基准场景评测与自动化打分脚本。
对 FSM 与博弈决策、多个随机种子做闭环回归，输出 results/benchmark.json / csv。
"""

import argparse
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.evaluation.benchmark import BenchmarkSuite


def main():
    parser = argparse.ArgumentParser(description="批量基准评测")
    parser.add_argument("--scenarios", nargs="+", default=["highway_merge"], help="场景名称或路径")
    parser.add_argument("--decisions", nargs="+", default=["fsm", "game"], choices=["fsm", "game"])
    parser.add_argument("--seeds", nargs="+", type=int, default=[42, 43, 44])
    parser.add_argument("--duration", type=float, default=20.0)
    args = parser.parse_args()

    print(">>> 启动批量场景基准评测...")
    suite = BenchmarkSuite(
        args.scenarios,
        decisions=args.decisions,
        seeds=args.seeds,
        duration=args.duration,
    )
    results = suite.run_all()
    json_path, csv_path = suite.save("results")

    print("\n---------------- Benchmark Summary ----------------")
    header = f"{'key':40s} {'success':>7s} {'minD(m)':>8s} {'TTC(s)':>8s} {'RMSjerk':>8s} {'latErr':>7s} {'plan(ms)':>9s}"
    print(header)
    for key, row in results.items():
        print(
            f"{key:40s} {row.get('success_rate', 0):7.2f} "
            f"{row.get('min_obstacle_distance_m', 0):8.2f} "
            f"{row.get('min_ttc_s', 0):8.2f} "
            f"{row.get('rms_jerk_m_s3', 0):8.3f} "
            f"{row.get('mean_lateral_error_m', 0):7.3f} "
            f"{row.get('avg_planning_latency_ms', 0):9.2f}"
        )
    print("--------------------------------------------------")
    print(f">>> JSON: {json_path}")
    print(f">>> CSV : {csv_path}")


if __name__ == "__main__":
    main()
