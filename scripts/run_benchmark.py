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
    parser.add_argument("--output-dir", default="results", help="结果输出目录")
    parser.add_argument(
        "--dynamic-model",
        "--dynamic",
        dest="dynamic_model",
        action="store_true",
        help="使用 2-DOF 动力学模型",
    )
    parser.add_argument("--planner", type=str, default=None, choices=["lattice", "em"])
    parser.add_argument("--controller", type=str, default=None, choices=["lqr", "mpc"])
    parser.add_argument("--tire", type=str, default=None, choices=["linear", "pacejka"])
    args = parser.parse_args()

    print(">>> 启动批量场景基准评测...")
    suite = BenchmarkSuite(
        args.scenarios,
        decisions=args.decisions,
        seeds=args.seeds,
        duration=args.duration,
        use_dynamic_model=args.dynamic_model,
        planner=args.planner,
        controller=args.controller,
        tire_model=args.tire,
    )
    results = suite.run_all()
    json_path, csv_path, summary_csv_path = suite.save(args.output_dir)

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
    print("Group summary (per-run success rate, Wilson 95% CI):")
    for row in suite.group_summary:
        print(
            f"{row['scenario']} / {row['decision']}: n={row['sample_count']}, "
            f"success={row['success_rate_mean']:.3f} "
            f"[{row['success_rate_ci95_low']:.3f}, {row['success_rate_ci95_high']:.3f}]"
        )
    print(f">>> JSON: {json_path}")
    print(f">>> CSV : {csv_path}")
    print(f">>> Group summary CSV: {summary_csv_path}")


if __name__ == "__main__":
    main()
