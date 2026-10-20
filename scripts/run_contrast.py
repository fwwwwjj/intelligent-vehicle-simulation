#!/usr/bin/env python3
"""Condition-driven contrast (baseline/ablation) benchmark runner.

Runs the proposed stack and every requested baseline/ablation condition through
the closed-loop simulator, one ``BenchmarkSuite`` per condition, then writes:

* ``contrast.json``         provenance metadata + raw runs + summaries + paired stats
* ``contrast_runs.csv``     one row per (condition, scenario, seed) run
* ``contrast_summary.csv``  per-(condition, scenario) aggregates
* ``contrast_report.md``    Markdown report with paired comparisons

Example (smoke test, 3 seeds, non-experimental conditions only)::

    python3 -B scripts/run_contrast.py \\
        --seeds 42 43 44 \\
        --conditions proposed_fsm_lattice_lqr ablation_planner_centerline \\
        --output-dir /tmp/siac_contrast_smoke

Experimental conditions (Game / EM / MPC / Pacejka) additionally require
``--allow-experimental``; without it the script aborts with exit code 2.
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.evaluation.benchmark import BenchmarkSuite  # noqa: E402
from src.evaluation.contrast import (  # noqa: E402
    CONDITION_BY_NAME,
    CONDITION_NAMES,
    CORE_SCENARIOS,
    DEFAULT_SEEDS,
    PROPOSED_CONDITION,
    build_markdown_report,
    collect_provenance,
    compare_conditions,
    resolve_conditions,
    summarize_runs,
    write_runs_csv,
    write_summary_csv,
)

LOW_POWER_HINTS = (
    "OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 "
    "MKL_NUM_THREADS=1 nice -n 19 python3 scripts/run_contrast.py ..."
)


def _format_seconds(seconds: float) -> str:
    seconds = max(0.0, float(seconds))
    if seconds < 60.0:
        return "{:.1f}s".format(seconds)
    minutes, remainder = divmod(int(round(seconds)), 60)
    return "{}m{:02d}s".format(minutes, remainder)


def parse_args(argv: List[str] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run baseline/ablation contrast experiments (one BenchmarkSuite per condition).",
        epilog=(
            "Low-power hints (optional, not applied automatically): {}\n"
            "Experimental conditions require --allow-experimental.".format(LOW_POWER_HINTS)
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--scenarios",
        nargs="+",
        default=list(CORE_SCENARIOS),
        help="Scenario names or YAML paths (default: %(default)s)",
    )
    parser.add_argument(
        "--seeds",
        nargs="+",
        type=int,
        default=list(DEFAULT_SEEDS),
        help="Random seeds, ignored by deterministic algorithms (default: 42..71)",
    )
    parser.add_argument("--duration", type=float, default=20.0, help="Duration per run in seconds")
    parser.add_argument(
        "--output-dir",
        default="results/contrast",
        help="Directory for contrast.json / CSVs / report (default: %(default)s)",
    )
    parser.add_argument(
        "--conditions",
        nargs="+",
        default=None,
        choices=list(CONDITION_NAMES),
        help="Condition names to run (default: all conditions)",
    )
    parser.add_argument(
        "--allow-experimental",
        action="store_true",
        help="Required when the selection contains experimental conditions (Game/EM/MPC/Pacejka).",
    )
    parser.add_argument(
        "--bootstrap-samples",
        type=int,
        default=2000,
        help="Paired bootstrap resamples for the confidence intervals (default: %(default)s)",
    )
    parser.add_argument(
        "--bootstrap-seed",
        type=int,
        default=0,
        help="Seed for the paired bootstrap RNG (default: %(default)s)",
    )
    return parser.parse_args(argv)


def main(argv: List[str] = None) -> int:
    args = parse_args(argv)

    try:
        selected = resolve_conditions(args.conditions)
    except KeyError as error:
        print("[ERROR] {}".format(error))
        return 2

    experimental_conditions = [condition.name for condition in selected if condition.experimental]
    if experimental_conditions and not args.allow_experimental:
        print("[EXPERIMENTAL] Selection contains experimental conditions: {}".format(
            ", ".join(experimental_conditions)
        ))
        print(
            "[EXPERIMENTAL] These modules stay isolated from the main results. "
            "Re-run with --allow-experimental to include them (rows are marked experimental=true)."
        )
        return 2
    if experimental_conditions:
        print("[EXPERIMENTAL] Running experimental conditions: {}".format(
            ", ".join(experimental_conditions)
        ))

    scenarios = list(args.scenarios)
    seeds = [int(seed) for seed in args.seeds]
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    total_runs = len(selected) * len(scenarios) * len(seeds)
    print(">>> Contrast experiment: {} conditions x {} scenarios x {} seeds = {} runs".format(
        len(selected), len(scenarios), len(seeds), total_runs
    ))
    print(">>> scenarios: {}".format(", ".join(scenarios)))
    print(">>> seeds: {}".format(seeds))

    started = time.time()
    completed_runs = 0
    all_runs: Dict[str, Dict[str, Any]] = {}

    for index, condition in enumerate(selected, start=1):
        condition_started = time.time()
        print(
            "\n[{}/{}] condition {} (decision={}, planner={}, controller={}, "
            "tire={}, dynamic_model={}, experimental={})".format(
                index,
                len(selected),
                condition.name,
                condition.decision,
                condition.planner,
                condition.controller,
                condition.tire_model,
                condition.dynamic_model,
                condition.experimental,
            )
        )
        suite = BenchmarkSuite(
            scenario_configs=scenarios,
            decisions=[condition.decision],
            seeds=seeds,
            duration=args.duration,
            use_dynamic_model=condition.dynamic_model,
            planner=condition.planner,
            controller=condition.controller,
            tire_model=condition.tire_model,
        )
        results = suite.run_all()

        # Benchmark run keys do not include planner/controller, so two
        # conditions can collide (e.g. same decision, different planner).
        # Prefix with the condition name and attach the condition metadata.
        for key, row in results.items():
            unique_key = "{}__{}".format(condition.name, key)
            row["run_key"] = unique_key
            row["condition"] = condition.name
            row["condition_family"] = condition.family
            row["condition_component"] = condition.component
            row["condition_description"] = condition.description
            row["condition_experimental"] = condition.experimental
            all_runs[unique_key] = row
            completed_runs += 1

        elapsed = time.time() - started
        rate = completed_runs / elapsed if elapsed > 0 else 0.0
        eta = (total_runs - completed_runs) / rate if rate > 0 else 0.0
        print(
            "    condition done in {} | runs {}/{} | elapsed {} | ETA {}".format(
                _format_seconds(time.time() - condition_started),
                completed_runs,
                total_runs,
                _format_seconds(elapsed),
                _format_seconds(eta),
            )
        )

    summary = summarize_runs(all_runs)
    comparisons = compare_conditions(
        all_runs,
        baseline=PROPOSED_CONDITION,
        n_boot=args.bootstrap_samples,
        seed=args.bootstrap_seed,
    )
    metadata: Dict[str, Any] = {
        "experiment": "contrast",
        "description": "Baseline/ablation contrast matrix against the proposed FSM + lattice + LQR stack.",
        "timestamp_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "command": " ".join([sys.executable] + list(sys.argv if argv is None else argv)),
        "argv": list(sys.argv if argv is None else argv),
        "scenario_configs": scenarios,
        "seeds": seeds,
        "duration_s": float(args.duration),
        "condition_names": [condition.name for condition in selected],
        "conditions": {condition.name: condition.to_dict() for condition in selected},
        "experimental": bool(experimental_conditions),
        "experimental_conditions": experimental_conditions,
        "n_boot": int(args.bootstrap_samples),
        "bootstrap_seed": int(args.bootstrap_seed),
        "alpha": 0.05,
        "success_rate_definition": (
            "per-run task-level binary success (task completion + no collision + no road departure); "
            "Wilson 95% interval"
        ),
        "metric_std_definition": "sample standard deviation (ddof=1); 0 for a single sample",
        "paired_difference_definition": "condition - proposed, paired on (scenario, seed)",
        "low_power_env_hints": LOW_POWER_HINTS,
        **collect_provenance(scenarios),
    }
    contrast_data: Dict[str, Any] = {
        "metadata": metadata,
        "baseline": PROPOSED_CONDITION,
        "conditions": {condition.name: condition.to_dict() for condition in selected},
        "runs": all_runs,
        "summary": summary,
        "comparisons": comparisons,
    }

    json_path = output_dir / "contrast.json"
    runs_csv_path = output_dir / "contrast_runs.csv"
    summary_csv_path = output_dir / "contrast_summary.csv"
    report_path = output_dir / "contrast_report.md"

    import json

    with json_path.open("w", encoding="utf-8") as handle:
        json.dump(contrast_data, handle, ensure_ascii=False, indent=2)
    write_runs_csv(runs_csv_path, all_runs)
    write_summary_csv(summary_csv_path, summary)
    report_path.write_text(build_markdown_report(contrast_data), encoding="utf-8")

    print("\n---------------- Contrast Summary ----------------")
    header = f"{'condition':42s} {'scenario':22s} {'n':>3s} {'success':>8s} {'95% CI':>18s} {'minD':>7s} {'jerk':>7s} {'plan ms':>8s}"
    print(header)
    by_condition_scenario = summary.get("by_condition_scenario", {})
    for key in sorted(by_condition_scenario):
        entry = by_condition_scenario[key]
        metrics = entry.get("metrics", {})
        min_distance = metrics.get("min_obstacle_distance_m", {}).get("mean")
        jerk = metrics.get("rms_jerk_m_s3", {}).get("mean")
        latency = metrics.get("avg_planning_latency_ms", {}).get("mean")
        print(
            "{:42s} {:22s} {:3d} {:8.3f} [{:6.3f},{:6.3f}] {:7.2f} {:7.3f} {:8.2f}".format(
                str(entry.get("condition")),
                str(entry.get("scenario")),
                int(entry.get("n", 0)),
                float(entry.get("success_rate_mean") or 0.0),
                float(entry.get("success_ci95_low") or 0.0),
                float(entry.get("success_ci95_high") or 0.0),
                float(min_distance) if min_distance is not None else 0.0,
                float(jerk) if jerk is not None else 0.0,
                float(latency) if latency is not None else 0.0,
            )
        )
    print("--------------------------------------------------")
    print(">>> JSON: {}".format(json_path))
    print(">>> Runs CSV: {}".format(runs_csv_path))
    print(">>> Summary CSV: {}".format(summary_csv_path))
    print(">>> Report: {}".format(report_path))
    print(">>> Total wall time: {}".format(_format_seconds(time.time() - started)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
