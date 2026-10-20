#!/usr/bin/env python3
"""导出 v0.4-core 的核心图表（BEV / 遥测 / 10–30 seeds KPI 分布）。

只使用核心链路：HFSM + Frenet Lattice + LQR + 运动学模型。
运行前需安装 matplotlib：``pip install -r requirements.txt``。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.environment.scenario_loader import load_named_scenario  # noqa: E402
from src.runner import build_engine  # noqa: E402
from src.visualization.visualizer_2d import Visualizer2D  # noqa: E402

SCENARIOS = ("highway_merge", "cut_in", "emergency_avoidance")
LABELS = {
    "highway_merge": "Highway merge",
    "cut_in": "Cut-in",
    "emergency_avoidance": "Emergency braking",
}


def _run(name: str, seed: int, duration: float):
    scenario = load_named_scenario(name)
    engine = build_engine(scenario, decision="fsm", seed=seed, enable_visualization=False)
    report = engine.run(duration)
    return engine, report


def _replay_to(name: str, seed: int, index: int, duration: float):
    scenario = load_named_scenario(name)
    engine = build_engine(scenario, decision="fsm", seed=seed, enable_visualization=False)
    for _ in range(max(0, index) + 1):
        engine.step()
    return engine


def _replay_until(name: str, seed: int, predicate, duration: float):
    scenario = load_named_scenario(name)
    engine = build_engine(scenario, decision="fsm", seed=seed, enable_visualization=False)
    steps = max(1, int(round(duration / engine.dt)))
    for _ in range(steps):
        engine.step()
        if predicate(engine):
            return engine
    return engine


def _save_bev(engine, path: Path, dpi: int = 150) -> None:
    Visualizer2D.save_bev_snapshot(
        engine.ego_state,
        engine.obstacle_manager.get_all_obstacles(),
        engine.road_network,
        engine.last_trajectory,
        engine.current_time,
        str(path),
        dpi=dpi,
    )


def _plot_telemetry(runs, out: Path) -> None:
    fig, axes = plt.subplots(3, 4, figsize=(16, 9), constrained_layout=True)
    fields = (
        ("velocity", "Speed (m/s)"),
        ("acceleration", "Acceleration (m/s²)"),
        ("jerk", "Jerk (m/s³)"),
        ("min_ttc", "TTC (s)"),
    )
    for row, name in enumerate(SCENARIOS):
        records = runs[name][0].evaluator.records
        times = [r.timestamp for r in records]
        for col, (field, title) in enumerate(fields):
            values = [getattr(r, field) for r in records]
            if field == "min_ttc":
                values = [v if 0.0 < v < 50.0 else np.nan for v in values]
            axes[row, col].plot(times, values, linewidth=1.4)
            axes[row, col].grid(alpha=0.25)
            if row == 0:
                axes[row, col].set_title(title)
            if col == 0:
                axes[row, col].set_ylabel(LABELS[name])
            if row == 2:
                axes[row, col].set_xlabel("Time (s)")
    fig.suptitle("Core closed-loop telemetry (FSM + Lattice + LQR, seed 42)", fontsize=13)
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)


def _plot_kpi_distribution(benchmark_path: Path, out: Path) -> None:
    payload = json.loads(benchmark_path.read_text(encoding="utf-8"))
    grouped = {name: [] for name in SCENARIOS}
    for row in payload["runs"].values():
        grouped.setdefault(row["scenario"], []).append(row)

    metrics = (
        ("min_obstacle_distance_m", "Min OBB gap (m)"),
        ("rms_jerk_m_s3", "RMS jerk (m/s³)"),
        ("avg_planning_latency_ms", "Planning latency (ms)"),
    )
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.8), constrained_layout=True)
    for ax, (field, title) in zip(axes, metrics):
        data = [[float(r[field]) for r in grouped.get(name, [])] for name in SCENARIOS]
        try:
            ax.boxplot(data, tick_labels=[LABELS[n] for n in SCENARIOS], showmeans=True)
        except TypeError:  # matplotlib < 3.9
            ax.boxplot(data, labels=[LABELS[n] for n in SCENARIOS], showmeans=True)
        ax.set_title(title)
        ax.grid(alpha=0.25, axis="y")
        ax.tick_params(axis="x", rotation=12)
    sample_counts = {name: len(grouped.get(name, [])) for name in SCENARIOS}
    n_unique = sorted(set(sample_counts.values()))
    n_label = str(n_unique[0]) if len(n_unique) == 1 else str(sample_counts)
    fig.suptitle(f"Core benchmark distribution (FSM, n={n_label} per scenario)", fontsize=13)
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="导出 v0.4-core 核心图表")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--benchmark", type=Path, default=Path("results/benchmark.json"))
    parser.add_argument("--output-dir", type=Path, default=Path("results/competition_pack/figures"))
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    runs = {}
    for name in SCENARIOS:
        engine, report = _run(name, args.seed, args.duration)
        runs[name] = (engine, report)
        print(f"{name}: success={report['success_rate']:.0f} "
              f"minD={report['min_obstacle_distance_m']:.2f} "
              f"jerk={report['rms_jerk_m_s3']:.2f}")

    # BEV snapshots: merge at first main-lane frame, cut-in at minimum gap, emergency at stop.
    merge_engine = _replay_until(
        "highway_merge", args.seed,
        lambda engine: engine.current_lane_id.startswith("main_lane"),
        args.duration,
    )
    _save_bev(merge_engine, args.output_dir / "fig1_highway_merge_bev.png")

    cut_records = runs["cut_in"][0].evaluator.records
    cut_idx = min(range(len(cut_records)), key=lambda i: cut_records[i].min_obstacle_distance)
    _save_bev(_replay_to("cut_in", args.seed, cut_idx, args.duration),
              args.output_dir / "fig2_cut_in_bev.png")

    em_records = runs["emergency_avoidance"][0].evaluator.records
    stop_idx = next((i for i, r in enumerate(em_records) if r.velocity <= 0.2), len(em_records) - 1)
    _save_bev(_replay_to("emergency_avoidance", args.seed, stop_idx, args.duration),
              args.output_dir / "fig3_emergency_avoidance_bev.png")

    _plot_telemetry(runs, args.output_dir / "fig4_core_telemetry.png")
    if args.benchmark.exists():
        _plot_kpi_distribution(args.benchmark, args.output_dir / "fig5_core_kpi_distribution.png")
    else:
        print(f"benchmark not found, skip KPI distribution: {args.benchmark}")

    print(f">>> figures written to {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
