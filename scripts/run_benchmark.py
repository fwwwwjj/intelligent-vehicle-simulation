#!/usr/bin/env python3
"""
批量基准场景评测与自动化打分脚本。
核心基准默认 FSM（Game 为探索性模块，需显式指定），对多个随机种子做闭环回归，输出 results/benchmark.json / csv。
"""

import argparse
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.evaluation.benchmark import BenchmarkSuite


def main():
    parser = argparse.ArgumentParser(description="批量基准评测")
    parser.add_argument("--scenarios", nargs="+", default=["highway_merge"], help="场景名称或路径")
    parser.add_argument(
        "--decisions",
        nargs="+",
        default=["fsm"],
        choices=["fsm", "game"],
        help="核心基准默认 FSM；game 为探索性模块，需显式指定",
    )
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
    parser.add_argument(
        "--allow-experimental",
        action="store_true",
        help="允许 Game/EM/MPC/Pacejka/交叉口进入批量评测（结果标记为实验性）",
    )
    args = parser.parse_args()

    experimental = (
        any(decision == "game" for decision in args.decisions)
        or args.planner == "em"
        or args.controller == "mpc"
        or args.tire == "pacejka"
        or any("intersection" in str(name) for name in args.scenarios)
    )
    if experimental and not args.allow_experimental:
        print("[EXPERIMENTAL] 当前参数包含实验性模块（Game/EM/MPC/Pacejka/交叉口）。")
        print("[EXPERIMENTAL] 这些模块代码保留，但不纳入主结果；请加 --allow-experimental 后显式运行。")
        return 2
    if experimental:
        print("[EXPERIMENTAL] 已允许运行；输出会标记 experimental=true，请勿写入主结果表。")

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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
