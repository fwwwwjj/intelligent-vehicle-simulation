#!/usr/bin/env python3
"""Recompute contrast summaries/statistics and regenerate the Markdown report.

Reads ``contrast.json`` written by ``scripts/run_contrast.py``, recomputes the
per-condition/per-scenario summaries and paired comparisons from the raw runs,
saves ``contrast_report.md`` (+ an updated summary CSV) and, when matplotlib is
available, 2–3 diagnostic figures:

* ``contrast_success_rate.png``     success rate by condition/scenario
* ``contrast_safety_comfort.png``   safety vs comfort trade-off
* ``contrast_planning_latency.png`` planning latency per condition

Matplotlib is imported lazily and the script degrades gracefully when it is not
installed (the statistics/report are always produced).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

# Must be set before matplotlib is imported: the default ~/.matplotlib may not be
# writable in the execution environment and would emit cache warnings.
if not os.environ.get("MPLCONFIGDIR"):
    os.environ["MPLCONFIGDIR"] = tempfile.mkdtemp(prefix="siac_mpl_")

from src.evaluation.contrast import (  # noqa: E402
    PROPOSED_CONDITION,
    build_markdown_report,
    compare_conditions,
    summarize_runs,
    write_summary_csv,
)

DEFAULT_INPUT = Path("results/contrast/contrast.json")
DEFAULT_OUTPUT_DIR = Path("results/contrast")


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Regenerate contrast report/figures from contrast.json without re-running simulations.",
        epilog=(
            "Low-power hints (optional, not applied automatically): "
            "OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1 "
            "nice -n 19 python3 scripts/analyze_contrast.py ..."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--input",
        default=str(DEFAULT_INPUT),
        help="Path to contrast.json produced by run_contrast.py (default: %(default)s)",
    )
    parser.add_argument(
        "--output-dir",
        default=str(DEFAULT_OUTPUT_DIR),
        help="Directory for report/figures (default: %(default)s)",
    )
    parser.add_argument(
        "--bootstrap-samples",
        type=int,
        default=None,
        help="Override the number of paired bootstrap resamples stored in the JSON",
    )
    parser.add_argument(
        "--bootstrap-seed",
        type=int,
        default=None,
        help="Override the paired bootstrap RNG seed stored in the JSON",
    )
    parser.add_argument(
        "--no-figures",
        action="store_true",
        help="Skip matplotlib figure generation",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=150,
        help="Figure resolution in dots per inch (default: %(default)s)",
    )
    return parser.parse_args(argv)


def load_contrast(path: Path) -> Dict[str, Any]:
    """Load ``contrast.json`` and fail with a readable message on malformed input."""
    if not path.is_file():
        raise FileNotFoundError(
            "contrast JSON not found: {} (run scripts/run_contrast.py first)".format(path)
        )
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict) or "runs" not in data:
        raise ValueError("{} is not a contrast.json file (missing 'runs')".format(path))
    return data


def recompute(data: Dict[str, Any], n_boot: Optional[int], seed: Optional[int]) -> Dict[str, Any]:
    """Recompute summary + paired comparisons from the stored raw runs."""
    metadata = dict(data.get("metadata") or {})
    resolved_boot = int(n_boot if n_boot is not None else metadata.get("n_boot", 2000))
    resolved_seed = int(seed if seed is not None else metadata.get("bootstrap_seed", 0))
    runs = data.get("runs") or {}
    baseline = str(data.get("baseline") or metadata.get("baseline") or PROPOSED_CONDITION)

    summary = summarize_runs(runs)
    comparisons = compare_conditions(
        runs,
        baseline=baseline,
        n_boot=resolved_boot,
        seed=resolved_seed,
    )
    metadata["n_boot"] = resolved_boot
    metadata["bootstrap_seed"] = resolved_seed
    recomputed = dict(data)
    recomputed["metadata"] = metadata
    recomputed["baseline"] = baseline
    recomputed["summary"] = summary
    recomputed["comparisons"] = comparisons
    return recomputed


def _import_matplotlib() -> Optional[Any]:
    """Import matplotlib's Agg pyplot lazily; return ``None`` when unavailable."""
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        return plt
    except ImportError as error:  # pragma: no cover - environment dependent
        print("[WARN] matplotlib is not available; skipping figures: {}".format(error))
        return None


def _safe_metric_mean(entry: Mapping[str, Any], metric: str) -> Optional[float]:
    metrics = entry.get("metrics")
    if not isinstance(metrics, Mapping):
        return None
    payload = metrics.get(metric)
    if not isinstance(payload, Mapping):
        return None
    value = payload.get("mean")
    try:
        return float(value) if value is not None else None
    except (TypeError, ValueError):
        return None


def plot_success_rate(summary: Mapping[str, Any], output_dir: Path, dpi: int = 150) -> Optional[Path]:
    """Grouped bar chart: success rate by condition, one series per scenario."""
    plt = _import_matplotlib()
    if plt is None:
        return None
    import numpy as np

    by_condition_scenario = summary.get("by_condition_scenario") or {}
    conditions = list(summary.get("conditions") or [])
    scenarios = list(summary.get("scenarios") or [])
    if not by_condition_scenario or not conditions or not scenarios:
        return None

    positions = np.arange(len(conditions), dtype=float)
    width = 0.8 / float(len(scenarios))
    figure, axis = plt.subplots(figsize=(max(8.0, 1.15 * len(conditions) + 4.0), 4.6))
    for index, scenario in enumerate(scenarios):
        rates: List[float] = []
        lower_errors: List[float] = []
        upper_errors: List[float] = []
        bars: List[float] = []
        for condition_index, condition in enumerate(conditions):
            entry = by_condition_scenario.get("{}::{}".format(condition, scenario))
            if not isinstance(entry, Mapping):
                continue
            rate = float(entry.get("success_rate_mean") or 0.0)
            low = float(entry.get("success_ci95_low") or 0.0)
            high = float(entry.get("success_ci95_high") or 0.0)
            rates.append(rate)
            lower_errors.append(max(0.0, rate - low))
            upper_errors.append(max(0.0, high - rate))
            offset = (index - (len(scenarios) - 1) / 2.0) * width
            bars.append(positions[condition_index] + offset)
        if not rates:
            continue
        axis.bar(
            bars,
            rates,
            width=width,
            label=scenario,
            yerr=[lower_errors, upper_errors],
            capsize=2.0,
        )
    axis.set_xticks(positions)
    axis.set_xticklabels(conditions, rotation=30, ha="right", fontsize=8)
    axis.set_ylim(0.0, 1.12)
    axis.set_ylabel("Success rate")
    axis.set_title("Success rate by condition and scenario (Wilson 95% CI)")
    axis.grid(axis="y", alpha=0.3)
    axis.legend(fontsize=8, ncol=len(scenarios))
    figure.tight_layout()
    path = output_dir / "contrast_success_rate.png"
    figure.savefig(path, dpi=dpi)
    plt.close(figure)
    return path


def plot_safety_comfort(summary: Mapping[str, Any], output_dir: Path, dpi: int = 150) -> Optional[Path]:
    """Scatter of min obstacle distance (safety) vs RMS jerk (comfort) per condition."""
    plt = _import_matplotlib()
    if plt is None:
        return None

    by_condition = summary.get("by_condition") or {}
    if not by_condition:
        return None
    family_colors = {
        "proposed": "#1f77b4",
        "baseline": "#2ca02c",
        "ablation": "#d62728",
        "unknown": "#7f7f7f",
    }
    figure, axis = plt.subplots(figsize=(7.5, 5.0))
    plotted = 0
    for name, entry in sorted(by_condition.items()):
        if not isinstance(entry, Mapping):
            continue
        safety = _safe_metric_mean(entry, "min_obstacle_distance_m")
        comfort = _safe_metric_mean(entry, "rms_jerk_m_s3")
        if safety is None or comfort is None:
            continue
        family = str(entry.get("family") or "unknown")
        color = family_colors.get(family, family_colors["unknown"])
        axis.scatter([safety], [comfort], s=70, color=color, edgecolor="black", zorder=3)
        axis.annotate(
            str(name),
            xy=(safety, comfort),
            xytext=(5, 4),
            textcoords="offset points",
            fontsize=6.5,
            color=color,
        )
        plotted += 1
    if plotted == 0:
        plt.close(figure)
        return None
    axis.set_xlabel("Mean min obstacle distance (m) — higher is safer")
    axis.set_ylabel("Mean RMS jerk (m/s^3) — lower is more comfortable")
    axis.set_title("Safety–comfort trade-off by condition (aggregated over scenarios)")
    axis.grid(alpha=0.3)
    handles = [
        plt.Line2D([], [], marker="o", linestyle="", color=color, label=family)
        for family, color in family_colors.items()
        if family != "unknown"
    ]
    axis.legend(handles=handles, fontsize=8)
    figure.tight_layout()
    path = output_dir / "contrast_safety_comfort.png"
    figure.savefig(path, dpi=dpi)
    plt.close(figure)
    return path


def plot_planning_latency(summary: Mapping[str, Any], output_dir: Path, dpi: int = 150) -> Optional[Path]:
    """Horizontal grouped bars: average and p95 planning latency per condition."""
    plt = _import_matplotlib()
    if plt is None:
        return None
    import numpy as np

    by_condition = summary.get("by_condition") or {}
    if not by_condition:
        return None
    entries: List[tuple] = []
    for name, entry in sorted(by_condition.items()):
        if not isinstance(entry, Mapping):
            continue
        average = _safe_metric_mean(entry, "avg_planning_latency_ms")
        if average is None:
            continue
        p95 = _safe_metric_mean(entry, "p95_planning_latency_ms")
        entries.append((str(name), average, p95))
    if not entries:
        return None
    entries.sort(key=lambda item: item[1])
    labels = [item[0] for item in entries]
    averages = [item[1] for item in entries]
    p95_values = [item[2] if item[2] is not None else 0.0 for item in entries]

    y_positions = np.arange(len(entries), dtype=float)
    figure, axis = plt.subplots(figsize=(8.5, max(4.0, 0.45 * len(entries) + 1.5)))
    axis.barh(y_positions - 0.2, averages, height=0.38, label="avg planning latency", color="#1f77b4")
    if any(item[2] is not None for item in entries):
        axis.barh(y_positions + 0.2, p95_values, height=0.38, label="p95 planning latency", color="#ff7f0e")
    axis.set_yticks(y_positions)
    axis.set_yticklabels(labels, fontsize=7.5)
    axis.set_xlabel("Planning latency (ms) — lower is better")
    axis.set_title("Planning latency per condition (mean over runs)")
    axis.grid(axis="x", alpha=0.3)
    axis.legend(fontsize=8)
    figure.tight_layout()
    path = output_dir / "contrast_planning_latency.png"
    figure.savefig(path, dpi=dpi)
    plt.close(figure)
    return path


def make_figures(summary: Mapping[str, Any], output_dir: Path, dpi: int = 150) -> List[Path]:
    """Generate all contrast figures, skipping any that cannot be produced."""
    figures: List[Path] = []
    builders = (
        ("success rate", plot_success_rate),
        ("safety-comfort", plot_safety_comfort),
        ("planning latency", plot_planning_latency),
    )
    for label, builder in builders:
        try:
            path = builder(summary, output_dir, dpi=dpi)
        except Exception as error:  # noqa: BLE001 - a figure must never break the analysis
            print("[WARN] skipped {} figure: {}".format(label, error))
            continue
        if path is not None:
            figures.append(path)
    return figures


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    input_path = Path(args.input)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    try:
        data = load_contrast(input_path)
    except (FileNotFoundError, ValueError, json.JSONDecodeError) as error:
        print("[ERROR] {}".format(error))
        return 2

    recomputed = recompute(data, args.bootstrap_samples, args.bootstrap_seed)
    summary = recomputed["summary"]
    comparisons = recomputed["comparisons"]

    report_path = output_dir / "contrast_report.md"
    report_path.write_text(build_markdown_report(recomputed), encoding="utf-8")
    summary_csv_path = write_summary_csv(output_dir / "contrast_summary.csv", summary)

    print(
        ">>> analyzed {} runs / {} conditions / {} scenarios".format(
            summary.get("n_runs", 0),
            len(summary.get("conditions") or []),
            len(summary.get("scenarios") or []),
        )
    )
    print(">>> paired comparisons: {}".format(comparisons.get("n_comparisons", 0)))
    print(">>> Report: {}".format(report_path))
    print(">>> Summary CSV: {}".format(summary_csv_path))

    if args.no_figures:
        print(">>> Figures skipped (--no-figures)")
        return 0
    figures = make_figures(summary, output_dir, dpi=args.dpi)
    if not figures:
        print("[WARN] no figures generated (matplotlib missing or no summarized metrics)")
    for path in figures:
        print(">>> Figure: {}".format(path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
