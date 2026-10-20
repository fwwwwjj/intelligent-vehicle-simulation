"""
Contrast / ablation experiment definitions, paired statistics and report generation.

This module is deliberately dependency-light: it only needs NumPy (no SciPy, no
pandas) so that the paired statistical analysis can be reproduced in the same
Python 3.9 environment used for the closed-loop simulations.

The design has three separable layers:

1. ``Condition`` + ``CONDITIONS``: the declarative baseline/ablation matrix that
   ``scripts/run_contrast.py`` turns into :class:`src.evaluation.benchmark.BenchmarkSuite`
   invocations (one suite per condition).
2. Pure statistics helpers: Wilson score interval, exact McNemar test, paired
   bootstrap, Wilcoxon signed-rank (normal approximation with tie/zero
   correction), Cliff's delta and Holm-Bonferroni correction.
3. ``summarize_runs`` / ``compare_conditions`` / ``build_markdown_report``:
   aggregate benchmark run rows into per-condition/per-scenario summaries,
   paired comparisons against the proposed condition, and a Markdown report.

Conventions
-----------
* A run row is one dict produced by ``BenchmarkSuite``; this module additionally
  requires a ``condition`` field (added by ``scripts/run_contrast.py``).
* Paired comparisons always pair runs on ``(scenario, seed)``.  The sign
  convention everywhere is ``difference = condition - proposed``: a positive
  difference means the ablation condition reported a *larger* value than the
  proposed full stack.
* For success outcomes, ``b`` counts baseline-only successes (proposed succeeds,
  ablation fails) and ``c`` counts ablation-only successes.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import platform
import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple, Union

import numpy as np

# ``src/evaluation/contrast.py`` -> repository root.
REPO_ROOT = Path(__file__).resolve().parents[2]

# Two-sided 95% normal quantile, kept as a constant so the Wilson interval is
# identical to the one already used by ``BenchmarkSuite.summarize_groups``.
Z_95 = 1.959963984540054

#: Name of the full proposed stack; the reference for every paired comparison.
PROPOSED_CONDITION = "proposed_fsm_lattice_lqr"

#: Default scenarios for the contrast experiment (the three core validation scenes).
CORE_SCENARIOS: Tuple[str, ...] = ("highway_merge", "cut_in", "emergency_avoidance")

#: Default seed range: 42..71 inclusive (30 seeds) matching the core benchmark.
DEFAULT_SEEDS: Tuple[int, ...] = tuple(range(42, 72))


# ---------------------------------------------------------------------------
# Condition matrix
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Condition:
    """One row of the baseline/ablation comparison matrix.

    The ``decision``/``planner``/``controller``/``tire_model``/``dynamic_model``
    fields are passed straight to ``src.runner.build_engine`` (via
    ``BenchmarkSuite``); the remaining fields are experiment metadata that is
    attached to every run row and rendered in the report.
    """

    name: str
    family: str  # proposed | baseline | ablation
    component: str  # full | full_stack | decision | planner | controller | vehicle_model
    description: str
    decision: str
    planner: str
    controller: str
    tire_model: str = "linear"
    dynamic_model: bool = False
    experimental: bool = False

    def to_dict(self) -> Dict[str, Any]:
        """JSON-friendly representation."""
        return asdict(self)


def _condition(
    name: str,
    family: str,
    component: str,
    description: str,
    decision: str,
    planner: str,
    controller: str,
    *,
    tire_model: str = "linear",
    dynamic_model: bool = False,
    experimental: bool = False,
) -> Condition:
    return Condition(
        name=name,
        family=family,
        component=component,
        description=description,
        decision=decision,
        planner=planner,
        controller=controller,
        tire_model=tire_model,
        dynamic_model=dynamic_model,
        experimental=experimental,
    )


#: Ordered comparison matrix.  The proposed stack is always first so it is the
#: natural baseline in reports and logs.
CONDITIONS: Tuple[Condition, ...] = (
    _condition(
        "proposed_fsm_lattice_lqr",
        "proposed",
        "full",
        "Proposed full stack: hierarchical FSM decision + Frenet lattice planner + LQR controller + linear tire model.",
        "fsm",
        "lattice",
        "lqr",
    ),
    _condition(
        "baseline_idm_centerline_pure_pursuit",
        "baseline",
        "full_stack",
        "Classical baseline stack: IDM car-following + centerline tracking + pure-pursuit controller.",
        "idm",
        "centerline",
        "pure_pursuit",
    ),
    _condition(
        "ablation_decision_game",
        "ablation",
        "decision",
        "Decision ablation: replace the rule-based FSM with the game-theoretic decision maker.",
        "game",
        "lattice",
        "lqr",
        experimental=True,
    ),
    _condition(
        "ablation_decision_idm",
        "ablation",
        "decision",
        "Decision ablation: replace the tactical FSM with IDM adaptive cruise (no lane-change intent).",
        "idm",
        "lattice",
        "lqr",
    ),
    _condition(
        "ablation_decision_no_lane_change",
        "ablation",
        "decision",
        "Decision ablation: HFSM with merge/lane-change intent disabled (lane keeping + car following only).",
        "fsm_no_lane_change",
        "lattice",
        "lqr",
    ),
    _condition(
        "ablation_planner_em",
        "ablation",
        "planner",
        "Planner ablation: replace the lattice planner with the expectation-maximisation planner.",
        "fsm",
        "em",
        "lqr",
        experimental=True,
    ),
    _condition(
        "ablation_planner_centerline",
        "ablation",
        "planner",
        "Planner ablation: replace search/optimisation with pure centerline tracking (no obstacle awareness).",
        "fsm",
        "centerline",
        "lqr",
    ),
    _condition(
        "ablation_controller_pure_pursuit",
        "ablation",
        "controller",
        "Controller ablation: replace LQR tracking with a pure-pursuit controller.",
        "fsm",
        "lattice",
        "pure_pursuit",
    ),
    _condition(
        "ablation_controller_mpc",
        "ablation",
        "controller",
        "Controller ablation: replace LQR tracking with the MPC controller.",
        "fsm",
        "lattice",
        "mpc",
        experimental=True,
    ),
    _condition(
        "ablation_model_dynamic_2dof",
        "ablation",
        "vehicle_model",
        "Vehicle-model ablation: closed loop with the 2-DOF dynamic model instead of the kinematic model.",
        "fsm",
        "lattice",
        "lqr",
        dynamic_model=True,
    ),
    _condition(
        "ablation_model_pacejka",
        "ablation",
        "vehicle_model",
        "Vehicle-model ablation: 2-DOF dynamic model with a Pacejka nonlinear tire model.",
        "fsm",
        "lattice",
        "lqr",
        tire_model="pacejka",
        dynamic_model=True,
        experimental=True,
    ),
)

CONDITION_BY_NAME: Dict[str, Condition] = {condition.name: condition for condition in CONDITIONS}

#: Ordered list of condition names.
CONDITION_NAMES: Tuple[str, ...] = tuple(condition.name for condition in CONDITIONS)


def all_conditions() -> List[Condition]:
    """Return a copy of the full condition list."""
    return list(CONDITIONS)


def resolve_conditions(names: Optional[Iterable[str]] = None) -> List[Condition]:
    """Resolve condition names to :class:`Condition` objects.

    ``None`` (or an empty iterable) means "all conditions".  An unknown name
    raises ``KeyError`` so CLI callers can turn it into a usage error.
    """
    if names is None:
        return all_conditions()
    requested = [str(name) for name in names]
    if not requested:
        return all_conditions()
    unknown = [name for name in requested if name not in CONDITION_BY_NAME]
    if unknown:
        raise KeyError(
            "Unknown condition(s): {}. Available: {}".format(
                ", ".join(sorted(unknown)), ", ".join(CONDITION_NAMES)
            )
        )
    # Preserve the canonical matrix order regardless of CLI argument order.
    selected = set(requested)
    return [condition for condition in CONDITIONS if condition.name in selected]


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

#: Continuous metrics aggregated in summaries, in report/CSV order.
SUMMARY_METRICS: Tuple[str, ...] = (
    "min_obstacle_distance_m",
    "min_ttc_s",
    "rms_jerk_m_s3",
    "max_lateral_accel_m_s2",
    "max_lateral_error_m",
    "avg_planning_latency_ms",
    "p95_planning_latency_ms",
    "task_completion_time_s",
    "collision_frames",
    "planner_fallback_rate",
    "controller_fallback_rate",
)

#: Metrics that enter the paired comparison family (Holm-Bonferroni corrected).
COMPARISON_METRICS: Tuple[str, ...] = SUMMARY_METRICS

#: Metrics where a *higher* value is better.  Everything else is "lower is
#: better"; ``build_markdown_report`` uses this to label effect direction.
HIGHER_IS_BETTER: Dict[str, bool] = {
    "min_obstacle_distance_m": True,
    "min_ttc_s": True,
    "task_completion_time_s": False,
    "rms_jerk_m_s3": False,
    "max_lateral_accel_m_s2": False,
    "max_lateral_error_m": False,
    "avg_planning_latency_ms": False,
    "p95_planning_latency_ms": False,
    "collision_frames": False,
    "planner_fallback_rate": False,
    "controller_fallback_rate": False,
}

#: Human-readable metric labels used in the Markdown report.
METRIC_LABELS: Dict[str, str] = {
    "min_obstacle_distance_m": "min obstacle distance (m)",
    "min_ttc_s": "min TTC (s)",
    "rms_jerk_m_s3": "RMS jerk (m/s^3)",
    "max_lateral_accel_m_s2": "max lateral accel (m/s^2)",
    "max_lateral_error_m": "max lateral error (m)",
    "avg_planning_latency_ms": "avg planning latency (ms)",
    "p95_planning_latency_ms": "p95 planning latency (ms)",
    "task_completion_time_s": "completion time (s)",
    "collision_frames": "collision frames",
    "planner_fallback_rate": "planner fallback rate",
    "controller_fallback_rate": "controller fallback rate",
}


def _metric_filter(metric: str, value: float) -> bool:
    """Apply the documented sentinel-value filters before aggregation.

    * ``min_ttc_s``: the evaluator emits ``999.0`` when no finite TTC was
      observed, so only finite values below 50 s are meaningful.
    * ``task_completion_time_s``: ``999.0`` marks runs that never completed the
      task; completion time is therefore conditional on completion.
    """
    if not math.isfinite(value):
        return False
    if metric == "min_ttc_s":
        return value < 50.0
    if metric == "task_completion_time_s":
        return value < 999.0
    return True


def _finite_values(rows: Sequence[Mapping[str, Any]], metric: str) -> np.ndarray:
    values: List[float] = []
    for row in rows:
        raw = row.get(metric)
        if raw is None:
            continue
        try:
            value = float(raw)
        except (TypeError, ValueError):
            continue
        if _metric_filter(metric, value):
            values.append(value)
    return np.asarray(values, dtype=float)


def _finite_or_none(value: Any) -> Optional[float]:
    """Coerce to a JSON-safe float (``None`` for nan/inf/non-numeric)."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return float(number) if math.isfinite(number) else None


def _success_value(row: Mapping[str, Any]) -> bool:
    """Per-run binary success, matching ``BenchmarkSuite`` (success_rate == 1)."""
    raw = row.get("success_rate")
    if raw is None:
        raw = row.get("task_success", 0.0)
    try:
        return float(raw) >= 1.0
    except (TypeError, ValueError):
        return False


# ---------------------------------------------------------------------------
# Core statistics (pure NumPy / stdlib)
# ---------------------------------------------------------------------------


def wilson_interval(successes: int, n: int, z: float = Z_95) -> Tuple[float, float]:
    """Two-sided Wilson score interval for a binomial proportion.

    Returns ``(low, high)`` clipped to ``[0, 1]``.  ``n <= 0`` returns the
    uninformative interval ``(0.0, 1.0)``.
    """
    n = int(n)
    successes = int(successes)
    if n <= 0:
        return (0.0, 1.0)
    if successes < 0 or successes > n:
        raise ValueError("successes must be in [0, n]")
    p = successes / float(n)
    denom = 1.0 + z * z / n
    center = (p + z * z / (2.0 * n)) / denom
    half = z * math.sqrt(p * (1.0 - p) / n + z * z / (4.0 * n * n)) / denom
    return (max(0.0, center - half), min(1.0, center + half))


def _exact_binomial_two_sided(smaller: int, n: int) -> float:
    """Two-sided exact binomial p-value for ``min(b, c)`` out of ``n`` at p=0.5."""
    if n <= 0:
        return 1.0
    smaller = int(smaller)
    n = int(n)
    if smaller < 0:
        smaller = 0
    if smaller > n:
        smaller = n
    tail = sum(math.comb(n, k) for k in range(0, smaller + 1))
    p = 2.0 * tail / (1 << n)
    return float(min(1.0, p))


def mcnemar_exact_test(
    baseline: Union[int, Sequence[Any]],
    condition: Union[int, Sequence[Any]],
) -> Dict[str, Any]:
    """Exact two-sided McNemar test on paired binary outcomes.

    Two call styles are supported:

    * ``mcnemar_exact_test(b, c)`` with two integers: discordant-pair counts,
      where ``b`` = baseline-only successes and ``c`` = condition-only successes.
    * ``mcnemar_exact_test(baseline_outcomes, condition_outcomes)`` with two
      equal-length sequences (booleans / 0-1 floats): the discordant pairs are
      derived element-wise.

    The p-value is the exact two-sided binomial test
    ``2 * P(X <= min(b, c) | X ~ Binomial(b + c, 0.5))`` capped at 1.
    """
    if isinstance(baseline, (int, np.integer)) and isinstance(condition, (int, np.integer)):
        b, c = int(baseline), int(condition)
        if b < 0 or c < 0:
            raise ValueError("discordant counts must be non-negative")
    else:
        left = list(baseline)  # type: ignore[arg-type]
        right = list(condition)  # type: ignore[arg-type]
        if len(left) != len(right):
            raise ValueError("paired outcome sequences must have the same length")
        b = 0
        c = 0
        for base_value, cond_value in zip(left, right):
            base_success = bool(float(base_value) >= 0.5)
            cond_success = bool(float(cond_value) >= 0.5)
            if base_success and not cond_success:
                b += 1
            elif cond_success and not base_success:
                c += 1
    n_discordant = b + c
    p_value = _exact_binomial_two_sided(min(b, c), n_discordant) if n_discordant > 0 else 1.0
    return {
        "b": b,
        "c": c,
        "n_discordant": n_discordant,
        "statistic": min(b, c),
        "p_value": float(p_value),
        "method": "exact_binomial_two_sided",
    }


def _rankdata_average(values: np.ndarray) -> np.ndarray:
    """Rank data 1..n with average ranks for ties (no SciPy)."""
    n = len(values)
    order = np.argsort(values, kind="mergesort")
    ranks = np.empty(n, dtype=float)
    sorted_values = values[order]
    i = 0
    while i < n:
        j = i + 1
        while j < n and sorted_values[j] == sorted_values[i]:
            j += 1
        average_rank = (i + 1 + j) / 2.0  # 1-based ranks i+1 .. j
        ranks[order[i:j]] = average_rank
        i = j
    return ranks


def _tie_group_sizes(values: np.ndarray) -> List[int]:
    if len(values) == 0:
        return []
    sorted_values = np.sort(values, kind="mergesort")
    sizes: List[int] = []
    current = 1
    for index in range(1, len(sorted_values)):
        if sorted_values[index] == sorted_values[index - 1]:
            current += 1
        else:
            sizes.append(current)
            current = 1
    sizes.append(current)
    return [size for size in sizes if size > 1]


def wilcoxon_signed_rank(
    baseline: Sequence[float],
    condition: Sequence[float],
) -> Dict[str, Any]:
    """Two-sided Wilcoxon signed-rank test (normal approximation, tie-corrected).

    ``differences = condition - baseline``; zero differences are dropped (Wilcox
    convention) and tied absolute differences receive average ranks.  The
    variance includes the standard tie correction; the p-value uses the normal
    approximation, which is more than adequate for the 30-seed design.
    """
    base = np.asarray([float(v) for v in baseline], dtype=float)
    cond = np.asarray([float(v) for v in condition], dtype=float)
    if len(base) != len(cond):
        raise ValueError("paired sequences must have the same length")
    finite_mask = np.isfinite(base) & np.isfinite(cond)
    diffs = cond[finite_mask] - base[finite_mask]
    nonzero = diffs[diffs != 0.0]
    n = int(len(nonzero))
    result: Dict[str, Any] = {
        "n_pairs": int(len(diffs)),
        "n_nonzero": n,
        "n_zeros": int(len(diffs) - n),
        "w_plus": 0.0,
        "w_minus": 0.0,
        "z": 0.0,
        "p_value": 1.0,
        "method": "normal_approximation_tie_corrected",
    }
    if n == 0:
        return result

    ranks = _rankdata_average(np.abs(nonzero))
    w_plus = float(np.sum(ranks[nonzero > 0.0]))
    w_minus = float(np.sum(ranks[nonzero < 0.0]))
    mean = n * (n + 1) / 4.0
    variance = n * (n + 1) * (2 * n + 1) / 24.0
    for tie_size in _tie_group_sizes(np.abs(nonzero)):
        variance -= (tie_size ** 3 - tie_size) / 48.0
    if variance <= 0.0:
        z = 0.0
        p_value = 1.0
    else:
        z = (w_plus - mean) / math.sqrt(variance)
        # 2 * (1 - Phi(|z|)) == erfc(|z| / sqrt(2)); erfc avoids cancellation.
        p_value = math.erfc(abs(z) / math.sqrt(2.0))
    result.update(
        {
            "w_plus": w_plus,
            "w_minus": w_minus,
            "z": float(z),
            "p_value": float(min(1.0, p_value)),
        }
    )
    return result


def paired_bootstrap_diff(
    baseline: Sequence[float],
    condition: Sequence[float],
    n_boot: int = 2000,
    seed: int = 0,
    ci_level: float = 0.95,
) -> Dict[str, Any]:
    """Bootstrap the mean paired difference ``condition - baseline``.

    Resamples paired differences with replacement (pairs stay coupled), using a
    dedicated ``numpy.random.default_rng(seed)`` so results are reproducible.
    Returns the point estimate plus a percentile confidence interval.
    """
    base = np.asarray([float(v) for v in baseline], dtype=float)
    cond = np.asarray([float(v) for v in condition], dtype=float)
    if len(base) != len(cond):
        raise ValueError("paired sequences must have the same length")
    diffs = cond - base
    diffs = diffs[np.isfinite(diffs)]
    n = int(len(diffs))
    result: Dict[str, Any] = {
        "n": n,
        "mean_diff": float(np.mean(diffs)) if n else float("nan"),
        "median_diff": float(np.median(diffs)) if n else float("nan"),
        "ci95_low": float("nan"),
        "ci95_high": float("nan"),
        "ci_level": float(ci_level),
        "n_boot": int(n_boot),
        "seed": int(seed),
    }
    if n == 0 or n_boot <= 0:
        return result
    rng = np.random.default_rng(int(seed))
    indices = rng.integers(0, n, size=(int(n_boot), n))
    boot_means = diffs[indices].mean(axis=1)
    alpha = 1.0 - float(ci_level)
    low, high = np.percentile(boot_means, [100.0 * alpha / 2.0, 100.0 * (1.0 - alpha / 2.0)])
    result["ci95_low"] = float(low)
    result["ci95_high"] = float(high)
    return result


def cliffs_delta(baseline: Sequence[float], condition: Sequence[float]) -> float:
    """Cliff's delta effect size for ``condition`` relative to ``baseline``.

    ``delta = P(condition > baseline) - P(condition < baseline)``; range
    ``[-1, 1]``.  Positive values mean the condition tends to report larger
    values; ties contribute zero.  Computed with binary search, O(m log n).
    """
    base = np.asarray([float(v) for v in baseline], dtype=float)
    cond = np.asarray([float(v) for v in condition], dtype=float)
    n = int(len(base))
    m = int(len(cond))
    if n == 0 or m == 0:
        return 0.0
    base_sorted = np.sort(base, kind="mergesort")
    greater = int(np.sum(np.searchsorted(base_sorted, cond, side="left")))
    less = int(np.sum(n - np.searchsorted(base_sorted, cond, side="right")))
    return float((greater - less) / float(n * m))


def cliffs_delta_magnitude(delta: float) -> str:
    """Qualitative Cliff's delta thresholds (Romano et al.): negligible/small/medium/large."""
    magnitude = abs(float(delta))
    if magnitude < 0.147:
        return "negligible"
    if magnitude < 0.330:
        return "small"
    if magnitude < 0.474:
        return "medium"
    return "large"


def holm_bonferroni(p_values: Sequence[float]) -> List[float]:
    """Holm-Bonferroni adjusted p-values, preserving input order.

    Adjusted values are monotonised (step-down) and clipped at 1.0; non-finite
    inputs are treated as ``p = 1``.
    """
    cleaned: List[float] = []
    for value in p_values:
        try:
            p = float(value)
        except (TypeError, ValueError):
            p = 1.0
        cleaned.append(p if math.isfinite(p) else 1.0)
    m = len(cleaned)
    if m == 0:
        return []
    order = sorted(range(m), key=lambda index: cleaned[index])
    adjusted = [1.0] * m
    running = 0.0
    for rank, index in enumerate(order):
        candidate = min(1.0, float(m - rank) * cleaned[index])
        running = max(running, candidate)
        adjusted[index] = running
    return adjusted


# ---------------------------------------------------------------------------
# Summaries
# ---------------------------------------------------------------------------


def _as_run_list(runs: Union[Mapping[str, Any], Iterable[Mapping[str, Any]]]) -> List[Dict[str, Any]]:
    """Accept either ``BenchmarkSuite.results`` (dict) or a list of run rows."""
    if isinstance(runs, Mapping):
        return [dict(row) for row in runs.values() if isinstance(row, Mapping)]
    return [dict(row) for row in runs if isinstance(row, Mapping)]


def _summarize_group(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    n = int(len(rows))
    success_count = sum(1 for row in rows if _success_value(row))
    success_rate = success_count / float(n) if n else float("nan")
    low, high = wilson_interval(success_count, n)
    seeds = sorted(
        int(row["seed"]) for row in rows if row.get("seed") is not None and str(row.get("seed")).lstrip("-").isdigit()
    )
    metrics: Dict[str, Dict[str, Any]] = {}
    for metric in SUMMARY_METRICS:
        values = _finite_values(rows, metric)
        if len(values) == 0:
            metrics[metric] = {"n": 0, "mean": None, "std": None, "min": None, "max": None}
            continue
        mean = float(np.mean(values))
        std = float(np.std(values, ddof=1)) if len(values) > 1 else 0.0
        metrics[metric] = {
            "n": int(len(values)),
            "mean": mean,
            "std": std,
            "min": float(np.min(values)),
            "max": float(np.max(values)),
        }
    return {
        "n": n,
        "seeds": seeds,
        "success_count": int(success_count),
        "success_rate_mean": _finite_or_none(success_rate),
        "success_ci95_low": float(low) if n else None,
        "success_ci95_high": float(high) if n else None,
        "metrics": metrics,
    }


def summarize_runs(
    runs: Union[Mapping[str, Any], Iterable[Mapping[str, Any]]],
    metrics: Sequence[str] = SUMMARY_METRICS,
) -> Dict[str, Any]:
    """Aggregate run rows into per-condition and per-condition/scenario summaries.

    Every run row must carry a ``condition`` and ``scenario`` label (added by
    ``scripts/run_contrast.py``).  For each group we report the sample size,
    per-run binary success count/rate with a Wilson 95% interval, and the
    mean/sample-std of every tracked continuous metric after sentinel filtering
    (``min_ttc_s`` finite < 50 s, completion time < 999 s).
    """
    rows = _as_run_list(runs)
    grouped_condition_scenario: Dict[Tuple[str, str], List[Dict[str, Any]]] = {}
    grouped_condition: Dict[str, List[Dict[str, Any]]] = {}
    grouped_scenario: Dict[str, List[Dict[str, Any]]] = {}
    for row in rows:
        condition = row.get("condition")
        scenario = row.get("scenario")
        if condition is None or scenario is None:
            continue
        condition = str(condition)
        scenario = str(scenario)
        grouped_condition_scenario.setdefault((condition, scenario), []).append(row)
        grouped_condition.setdefault(condition, []).append(row)
        grouped_scenario.setdefault(scenario, []).append(row)

    # ``metrics`` is part of the API but the group helper uses the module-level
    # SUMMARY_METRICS; keep the parameter meaningful for custom callers.
    def aggregate(group: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
        if tuple(metrics) == tuple(SUMMARY_METRICS):
            return _summarize_group(group)
        summary = _summarize_group(group)
        custom: Dict[str, Dict[str, Any]] = {}
        for metric in metrics:
            values = _finite_values(group, metric)
            if len(values) == 0:
                custom[metric] = {"n": 0, "mean": None, "std": None}
            else:
                custom[metric] = {
                    "n": int(len(values)),
                    "mean": float(np.mean(values)),
                    "std": float(np.std(values, ddof=1)) if len(values) > 1 else 0.0,
                }
        summary["metrics"] = custom
        return summary

    by_condition_scenario: Dict[str, Dict[str, Any]] = {}
    for (condition, scenario), group in sorted(grouped_condition_scenario.items()):
        entry = aggregate(group)
        entry.update({"condition": condition, "scenario": scenario})
        condition_meta = CONDITION_BY_NAME.get(condition)
        if condition_meta is not None:
            entry["family"] = condition_meta.family
            entry["component"] = condition_meta.component
        by_condition_scenario["{}::{}".format(condition, scenario)] = entry

    by_condition: Dict[str, Dict[str, Any]] = {}
    for condition, group in sorted(grouped_condition.items()):
        entry = aggregate(group)
        entry["condition"] = condition
        condition_meta = CONDITION_BY_NAME.get(condition)
        if condition_meta is not None:
            entry["family"] = condition_meta.family
            entry["component"] = condition_meta.component
        by_condition[condition] = entry

    by_scenario: Dict[str, Dict[str, Any]] = {}
    for scenario, group in sorted(grouped_scenario.items()):
        entry = aggregate(group)
        entry["scenario"] = scenario
        by_scenario[scenario] = entry

    definitions = {
        metric: {
            "label": METRIC_LABELS.get(metric, metric),
            "higher_is_better": HIGHER_IS_BETTER.get(metric),
            "filter": (
                "finite and < 50 s"
                if metric == "min_ttc_s"
                else ("finite and < 999 s (completed runs only)" if metric == "task_completion_time_s" else "finite")
            ),
        }
        for metric in metrics
    }
    return {
        "n_runs": int(len(rows)),
        "conditions": sorted(grouped_condition),
        "scenarios": sorted(grouped_scenario),
        "metric_definitions": definitions,
        "by_condition_scenario": by_condition_scenario,
        "by_condition": by_condition,
        "by_scenario": by_scenario,
    }


# ---------------------------------------------------------------------------
# Paired comparisons against the proposed condition
# ---------------------------------------------------------------------------


def _run_key(row: Mapping[str, Any]) -> Optional[Tuple[str, str, int]]:
    condition = row.get("condition")
    scenario = row.get("scenario")
    seed = row.get("seed")
    if condition is None or scenario is None or seed is None:
        return None
    try:
        return (str(condition), str(scenario), int(seed))
    except (TypeError, ValueError):
        return None


def compare_conditions(
    runs: Union[Mapping[str, Any], Iterable[Mapping[str, Any]]],
    baseline: str = PROPOSED_CONDITION,
    metrics: Sequence[str] = COMPARISON_METRICS,
    n_boot: int = 2000,
    seed: int = 0,
    alpha: float = 0.05,
) -> Dict[str, Any]:
    """Paired comparisons of every condition against ``baseline``.

    Runs are paired on ``(scenario, seed)``.  The success outcome uses an exact
    McNemar test and reports discordant pair counts; every continuous metric
    reports the paired mean difference, a seeded paired-bootstrap 95% CI,
    a Wilcoxon signed-rank p-value and Cliff's delta.  P-values within one
    condition's family (success + metrics) are Holm-Bonferroni corrected.

    The sign convention is ``condition - baseline`` throughout.
    """
    rows = _as_run_list(runs)
    index: Dict[Tuple[str, str, int], Dict[str, Any]] = {}
    for row in rows:
        key = _run_key(row)
        if key is not None:
            index[key] = row

    available_conditions = sorted({condition for (condition, _, _) in index})
    baseline_keys = {(scenario, run_seed) for (condition, scenario, run_seed) in index if condition == baseline}

    comparisons: List[Dict[str, Any]] = []
    for condition in available_conditions:
        if condition == baseline:
            continue
        condition_keys = {(scenario, run_seed) for (name, scenario, run_seed) in index if name == condition}
        pairs = sorted(condition_keys & baseline_keys)
        if not pairs:
            # No matched (scenario, seed) pairs -> this condition cannot be
            # compared against the proposed stack; omit it from the report.
            continue
        base_rows = [index[(baseline, scenario, run_seed)] for scenario, run_seed in pairs]
        cond_rows = [index[(condition, scenario, run_seed)] for scenario, run_seed in pairs]
        condition_meta = CONDITION_BY_NAME.get(condition)

        # ---- success (paired binary outcome) ----
        base_success = [1.0 if _success_value(row) else 0.0 for row in base_rows]
        cond_success = [1.0 if _success_value(row) else 0.0 for row in cond_rows]
        mcnemar = mcnemar_exact_test(base_success, cond_success)
        success_payload: Dict[str, Any] = {
            "n_pairs": int(len(pairs)),
            "success_count_baseline": int(sum(base_success)),
            "success_count_condition": int(sum(cond_success)),
            "success_rate_baseline": _finite_or_none(np.mean(base_success)) if base_success else None,
            "success_rate_condition": _finite_or_none(np.mean(cond_success)) if cond_success else None,
            "risk_difference": (
                _finite_or_none(float(np.mean(cond_success)) - float(np.mean(base_success)))
                if base_success
                else None
            ),
            "discordant_baseline_only": mcnemar["b"],
            "discordant_condition_only": mcnemar["c"],
            "concordant_success": int(
                sum(1 for b_value, c_value in zip(base_success, cond_success) if b_value and c_value)
            ),
            "concordant_failure": int(
                sum(1 for b_value, c_value in zip(base_success, cond_success) if not b_value and not c_value)
            ),
            "mcnemar_p_value": mcnemar["p_value"],
            "mcnemar_p_holm": None,
            "significant_holm": False,
        }

        # ---- continuous metrics ----
        metric_payload: Dict[str, Dict[str, Any]] = {}
        metric_p_values: List[float] = []
        for metric in metrics:
            paired_base: List[float] = []
            paired_cond: List[float] = []
            for base_row, cond_row in zip(base_rows, cond_rows):
                base_value = base_row.get(metric)
                cond_value = cond_row.get(metric)
                if base_value is None or cond_value is None:
                    continue
                try:
                    base_float = float(base_value)
                    cond_float = float(cond_value)
                except (TypeError, ValueError):
                    continue
                if _metric_filter(metric, base_float) and _metric_filter(metric, cond_float):
                    paired_base.append(base_float)
                    paired_cond.append(cond_float)
            n_pairs_metric = len(paired_base)
            payload: Dict[str, Any] = {
                "n_pairs": int(n_pairs_metric),
                "mean_baseline": _finite_or_none(np.mean(paired_base)) if n_pairs_metric else None,
                "mean_condition": _finite_or_none(np.mean(paired_cond)) if n_pairs_metric else None,
                "mean_diff": None,
                "median_diff": None,
                "ci95_low": None,
                "ci95_high": None,
                "wilcoxon_p_value": None,
                "wilcoxon_p_holm": None,
                "cliffs_delta": None,
                "effect_magnitude": None,
                "significant_holm": False,
            }
            if n_pairs_metric > 0:
                bootstrap = paired_bootstrap_diff(
                    paired_base, paired_cond, n_boot=n_boot, seed=seed
                )
                wilcoxon = wilcoxon_signed_rank(paired_base, paired_cond)
                delta = cliffs_delta(paired_base, paired_cond)
                payload.update(
                    {
                        "mean_diff": _finite_or_none(bootstrap["mean_diff"]),
                        "median_diff": _finite_or_none(bootstrap["median_diff"]),
                        "ci95_low": _finite_or_none(bootstrap["ci95_low"]),
                        "ci95_high": _finite_or_none(bootstrap["ci95_high"]),
                        "wilcoxon_p_value": _finite_or_none(wilcoxon["p_value"]),
                        "cliffs_delta": _finite_or_none(delta),
                        "effect_magnitude": cliffs_delta_magnitude(delta),
                        "n_nonzero": int(wilcoxon["n_nonzero"]),
                    }
                )
                if n_pairs_metric >= 2 and payload["wilcoxon_p_value"] is not None:
                    metric_p_values.append(float(payload["wilcoxon_p_value"]))
            metric_payload[metric] = payload

        # ---- Holm-Bonferroni within this condition's test family ----
        # The family label stores the metric's position in ``metrics`` so the
        # adjusted p-values map back correctly even when a metric is skipped
        # (e.g. completion time with no completed runs).
        family = {("success", -1): float(mcnemar["p_value"])}
        for metric_index, metric in enumerate(metrics):
            p_value = metric_payload[metric]["wilcoxon_p_value"]
            if p_value is not None and metric_payload[metric]["n_pairs"] >= 2:
                family[("metric", metric_index)] = float(p_value)
        labels = list(family.keys())
        adjusted = holm_bonferroni([family[label] for label in labels])
        for label, adjusted_p in zip(labels, adjusted):
            if label[0] == "success":
                success_payload["mcnemar_p_holm"] = _finite_or_none(adjusted_p)
                success_payload["significant_holm"] = bool(adjusted_p < alpha)
            else:
                metric = metrics[label[1]]
                metric_payload[metric]["wilcoxon_p_holm"] = _finite_or_none(adjusted_p)
                metric_payload[metric]["significant_holm"] = bool(adjusted_p < alpha)

        comparisons.append(
            {
                "condition": condition,
                "family": condition_meta.family if condition_meta else "unknown",
                "component": condition_meta.component if condition_meta else "unknown",
                "experimental": bool(condition_meta.experimental) if condition_meta else False,
                "n_pairs": int(len(pairs)),
                "scenarios": sorted({scenario for scenario, _ in pairs}),
                "seeds": sorted({run_seed for _, run_seed in pairs}),
                "success": success_payload,
                "metrics": metric_payload,
            }
        )

    return {
        "baseline": baseline,
        "alpha": float(alpha),
        "n_boot": int(n_boot),
        "bootstrap_seed": int(seed),
        "n_conditions": len(available_conditions),
        "n_comparisons": len(comparisons),
        "definitions": {
            "paired_unit": "one run per (scenario, seed); only pairs present in both conditions are used",
            "difference": "condition - baseline",
            "success_test": "exact two-sided McNemar/binomial test on discordant pairs",
            "metric_test": "two-sided Wilcoxon signed-rank, normal approximation with tie/zero correction",
            "bootstrap": "paired percentile bootstrap of the mean difference, seeded RNG",
            "effect_size": "Cliff's delta = P(condition > baseline) - P(condition < baseline)",
            "multiple_comparisons": "Holm-Bonferroni within each condition family (success + metrics)",
        },
        "comparisons": comparisons,
    }


# ---------------------------------------------------------------------------
# Provenance / metadata helpers
# ---------------------------------------------------------------------------


def _git_output(args: Sequence[str], cwd: Path) -> Optional[str]:
    try:
        completed = subprocess.run(
            ["git"] + list(args),
            check=True,
            capture_output=True,
            text=True,
            cwd=str(cwd),
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    return completed.stdout.strip()


def collect_provenance(
    scenario_configs: Sequence[str],
    repo_root: Optional[Path] = None,
) -> Dict[str, Any]:
    """Capture reproducibility metadata (git revision, dirty flag, scenario hashes)."""
    root = Path(repo_root) if repo_root is not None else REPO_ROOT
    commit = _git_output(["rev-parse", "HEAD"], root)
    # Generated artifacts under results/ are not code dirt (same rule as benchmark.py).
    status = _git_output(["status", "--porcelain", "--", ".", ":(exclude)results"], root)
    dirty = None if status is None else bool(status)
    scenario_sha256: Dict[str, str] = {}
    for scenario in scenario_configs:
        candidate = Path(str(scenario))
        if not candidate.is_file():
            candidate = root / "configs" / "scenarios" / "{}.yaml".format(scenario)
        if candidate.is_file():
            scenario_sha256[str(scenario)] = hashlib.sha256(candidate.read_bytes()).hexdigest()
    return {
        "git_commit": commit,
        "git_dirty": dirty,
        "scenario_sha256": scenario_sha256,
        "python_version": platform.python_version(),
    }


# ---------------------------------------------------------------------------
# CSV / JSON-friendly flattening
# ---------------------------------------------------------------------------


def _csv_value(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return "{:.10g}".format(value)
    if isinstance(value, (list, tuple, dict)):
        return json.dumps(value, ensure_ascii=False)
    return value


def runs_to_rows(
    runs: Union[Mapping[str, Any], Iterable[Mapping[str, Any]]],
) -> List[Dict[str, Any]]:
    """Flatten run rows for CSV output (stable first-seen field order)."""
    rows = _as_run_list(runs)
    flattened: List[Dict[str, Any]] = []
    for row in rows:
        flattened.append({key: _csv_value(value) for key, value in row.items()})
    return flattened


def write_runs_csv(
    path: Union[str, Path],
    runs: Union[Mapping[str, Any], Iterable[Mapping[str, Any]]],
) -> Path:
    """Write one CSV row per run; union of all row keys (first-seen order)."""
    rows = runs_to_rows(runs)
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fieldnames: List[str] = []
    seen = set()
    for row in rows:
        for key in row:
            if key not in seen:
                seen.add(key)
                fieldnames.append(key)
    with destination.open("w", encoding="utf-8", newline="") as handle:
        if not fieldnames:
            handle.write("")
            return destination
        writer = csv.DictWriter(handle, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    return destination


def summary_to_rows(summary: Mapping[str, Any]) -> List[Dict[str, Any]]:
    """Flatten a ``summarize_runs`` result into per-(condition, scenario) CSV rows."""
    rows: List[Dict[str, Any]] = []
    by_condition_scenario = summary.get("by_condition_scenario", {}) if isinstance(summary, Mapping) else {}
    if not isinstance(by_condition_scenario, Mapping):
        return rows
    for key, entry in by_condition_scenario.items():
        if not isinstance(entry, Mapping):
            continue
        row: Dict[str, Any] = {
            "condition": entry.get("condition"),
            "family": entry.get("family"),
            "component": entry.get("component"),
            "scenario": entry.get("scenario"),
            "n": entry.get("n"),
            "success_count": entry.get("success_count"),
            "success_rate_mean": entry.get("success_rate_mean"),
            "success_ci95_low": entry.get("success_ci95_low"),
            "success_ci95_high": entry.get("success_ci95_high"),
        }
        metrics = entry.get("metrics", {})
        if isinstance(metrics, Mapping):
            for metric in SUMMARY_METRICS:
                payload = metrics.get(metric)
                if not isinstance(payload, Mapping):
                    continue
                row["{}_mean".format(metric)] = payload.get("mean")
                row["{}_std".format(metric)] = payload.get("std")
                row["{}_n".format(metric)] = payload.get("n")
        rows.append({key_: _csv_value(value_) for key_, value_ in row.items()})
    return rows


def write_summary_csv(path: Union[str, Path], summary: Mapping[str, Any]) -> Path:
    """Write per-(condition, scenario) aggregate statistics as CSV."""
    rows = summary_to_rows(summary)
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fieldnames: List[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with destination.open("w", encoding="utf-8", newline="") as handle:
        if not fieldnames:
            handle.write("")
            return destination
        writer = csv.DictWriter(handle, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    return destination


# ---------------------------------------------------------------------------
# Markdown report
# ---------------------------------------------------------------------------


def _fmt(value: Any, digits: int = 3) -> str:
    """Format a number for a Markdown table; ``—`` for missing values."""
    if value is None:
        return "—"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(number):
        return "—"
    return "{:.{}f}".format(number, digits)


def _fmt_signed(value: Any, digits: int = 3) -> str:
    if value is None:
        return "—"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(number):
        return "—"
    return "{:+.{digits}f}".format(number, digits=digits)


def _md_cell(value: Any) -> str:
    """Escape a value for a single Markdown table cell."""
    text = "—" if value is None else str(value)
    return text.replace("|", "\\|").replace("\n", " ")


def _md_table(headers: Sequence[str], rows: Sequence[Sequence[Any]]) -> str:
    if not rows:
        return "_No rows._\n"
    lines = [
        "| " + " | ".join(_md_cell(header) for header in headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(_md_cell(cell) for cell in row) + " |")
    return "\n".join(lines) + "\n"


def _seed_span(seeds: Sequence[int]) -> str:
    unique = sorted({int(seed) for seed in seeds})
    if not unique:
        return "none"
    if len(unique) == 1:
        return str(unique[0])
    if unique == list(range(unique[0], unique[-1] + 1)):
        return "{}–{} ({} seeds)".format(unique[0], unique[-1], len(unique))
    return ", ".join(str(seed) for seed in unique)


def build_markdown_report(contrast_data: Mapping[str, Any]) -> str:
    """Render the full contrast experiment report as Markdown.

    ``contrast_data`` is the structure written to ``contrast.json``
    (``metadata``, ``conditions``, ``runs``, ``summary``, ``comparisons``).
    Missing ``summary``/``comparisons`` sections are recomputed from ``runs``
    when possible so the report can be regenerated from raw runs.
    """
    data: Dict[str, Any] = dict(contrast_data)
    metadata: Mapping[str, Any] = data.get("metadata", {}) if isinstance(data.get("metadata"), Mapping) else {}
    summary: Mapping[str, Any] = data.get("summary", {}) if isinstance(data.get("summary"), Mapping) else {}
    comparisons: Mapping[str, Any] = (
        data.get("comparisons", {}) if isinstance(data.get("comparisons"), Mapping) else {}
    )
    runs = data.get("runs")

    if (not summary or "by_condition_scenario" not in summary) and runs is not None:
        summary = summarize_runs(runs)
    if (not comparisons or "comparisons" not in comparisons) and runs is not None:
        comparisons = compare_conditions(
            runs,
            baseline=str(data.get("baseline", PROPOSED_CONDITION)),
            n_boot=int(metadata.get("n_boot", 2000)),
            seed=int(metadata.get("bootstrap_seed", 0)),
        )
    baseline_name = str(comparisons.get("baseline", PROPOSED_CONDITION))

    lines: List[str] = []
    lines.append("# Contrast Experiment Report")
    lines.append("")
    lines.append(
        "Paired baseline/ablation analysis for the proposed stack "
        "(`{}`). All numbers are produced from closed-loop simulation runs; "
        "no metric is hard-coded.".format(baseline_name)
    )
    lines.append("")

    # ---- metadata ----
    lines.append("## Metadata")
    lines.append("")
    timestamp = metadata.get("timestamp_utc") or metadata.get("timestamp") or "unknown"
    meta_rows = [
        ("Timestamp (UTC)", timestamp),
        ("Git commit", metadata.get("git_commit") or "unknown"),
        ("Git dirty (excluding results/)", metadata.get("git_dirty")),
        ("Python version", metadata.get("python_version") or platform.python_version()),
        ("Duration per run (s)", metadata.get("duration_s")),
        ("Scenarios", ", ".join(str(item) for item in metadata.get("scenario_configs", [])) or "unknown"),
        ("Seeds", _seed_span(metadata.get("seeds", []))),
        ("Bootstrap resamples", comparisons.get("n_boot", metadata.get("n_boot"))),
        ("Bootstrap seed", comparisons.get("bootstrap_seed", metadata.get("bootstrap_seed"))),
        ("Multiple-comparison alpha", comparisons.get("alpha", 0.05)),
    ]
    lines.append(_md_table(["Field", "Value"], meta_rows))
    lines.append("")

    # ---- condition matrix ----
    lines.append("## Condition Definitions")
    lines.append("")
    condition_rows: List[List[Any]] = []
    conditions = data.get("conditions", {})
    if isinstance(conditions, Mapping):
        condition_iter = list(conditions.values())
    else:
        condition_iter = list(conditions) if isinstance(conditions, (list, tuple)) else []
    if not condition_iter:
        condition_iter = [condition.to_dict() for condition in CONDITIONS]
    for condition in condition_iter:
        if not isinstance(condition, Mapping):
            continue
        condition_rows.append(
            [
                condition.get("name"),
                condition.get("family"),
                condition.get("component"),
                condition.get("decision"),
                condition.get("planner"),
                condition.get("controller"),
                condition.get("tire_model"),
                condition.get("dynamic_model"),
                condition.get("experimental"),
                condition.get("description"),
            ]
        )
    lines.append(
        _md_table(
            [
                "Condition",
                "Family",
                "Component",
                "Decision",
                "Planner",
                "Controller",
                "Tire",
                "Dynamic",
                "Experimental",
                "Description",
            ],
            condition_rows,
        )
    )
    lines.append("")

    # ---- success summary ----
    lines.append("## Success Rate by Condition and Scenario")
    lines.append("")
    lines.append(
        "Per-run success is binary (task completion and no collision and no road "
        "departure). Intervals are Wilson score 95% CIs."
    )
    lines.append("")
    success_rows: List[List[Any]] = []
    by_condition_scenario = summary.get("by_condition_scenario", {}) if summary else {}
    if isinstance(by_condition_scenario, Mapping):
        for key in sorted(by_condition_scenario):
            entry = by_condition_scenario[key]
            if not isinstance(entry, Mapping):
                continue
            success_rows.append(
                [
                    entry.get("condition"),
                    entry.get("scenario"),
                    entry.get("n"),
                    entry.get("success_count"),
                    _fmt(entry.get("success_rate_mean")),
                    "[{}, {}]".format(
                        _fmt(entry.get("success_ci95_low")), _fmt(entry.get("success_ci95_high"))
                    ),
                ]
            )
    lines.append(_md_table(["Condition", "Scenario", "n", "Successes", "Rate", "95% CI"], success_rows))
    lines.append("")

    # ---- metric summary ----
    lines.append("## Metric Summary by Condition and Scenario")
    lines.append("")
    lines.append(
        "Values are mean ± sample standard deviation (ddof=1). "
        "`min_ttc_s` uses only finite values below 50 s; completion time only "
        "completed runs (< 999 s sentinel)."
    )
    lines.append("")
    comfort_rows: List[List[Any]] = []
    latency_rows: List[List[Any]] = []
    if isinstance(by_condition_scenario, Mapping):
        for key in sorted(by_condition_scenario):
            entry = by_condition_scenario[key]
            if not isinstance(entry, Mapping):
                continue
            metric_stats = entry.get("metrics", {})
            if not isinstance(metric_stats, Mapping):
                continue

            def mean_std(metric: str) -> str:
                payload = metric_stats.get(metric)
                if not isinstance(payload, Mapping) or payload.get("mean") is None:
                    return "—"
                return "{} ± {}".format(_fmt(payload.get("mean")), _fmt(payload.get("std")))

            comfort_rows.append(
                [
                    entry.get("condition"),
                    entry.get("scenario"),
                    mean_std("min_obstacle_distance_m"),
                    mean_std("min_ttc_s"),
                    mean_std("rms_jerk_m_s3"),
                    mean_std("max_lateral_accel_m_s2"),
                    mean_std("max_lateral_error_m"),
                ]
            )
            latency_rows.append(
                [
                    entry.get("condition"),
                    entry.get("scenario"),
                    mean_std("avg_planning_latency_ms"),
                    mean_std("p95_planning_latency_ms"),
                    mean_std("task_completion_time_s"),
                    mean_std("collision_frames"),
                    mean_std("planner_fallback_rate"),
                    mean_std("controller_fallback_rate"),
                ]
            )
    lines.append(
        _md_table(
            [
                "Condition",
                "Scenario",
                "min distance (m)",
                "min TTC (s)",
                "RMS jerk (m/s^3)",
                "max lat accel (m/s^2)",
                "max lat error (m)",
            ],
            comfort_rows,
        )
    )
    lines.append("")
    lines.append(
        _md_table(
            [
                "Condition",
                "Scenario",
                "avg plan latency (ms)",
                "p95 plan latency (ms)",
                "completion time (s)",
                "collision frames",
                "planner fallback rate",
                "controller fallback rate",
            ],
            latency_rows,
        )
    )
    lines.append("")

    # ---- comparisons ----
    lines.append("## Paired Comparisons vs Proposed")
    lines.append("")
    lines.append(
        "Pairs are matched on `(scenario, seed)`; the difference is always "
        "`condition - proposed`. Positive differences mean the ablation "
        "condition reported a larger value. `p` values are Holm-Bonferroni "
        "adjusted within each condition family (success + metrics); the "
        "`significant` column marks `p_holm < alpha`."
    )
    lines.append("")
    comparison_list = comparisons.get("comparisons", []) if isinstance(comparisons, Mapping) else []
    success_compare_rows: List[List[Any]] = []
    metric_compare_rows: List[List[Any]] = []
    for comparison in comparison_list:
        if not isinstance(comparison, Mapping):
            continue
        condition = comparison.get("condition")
        success = comparison.get("success", {})
        if isinstance(success, Mapping):
            success_compare_rows.append(
                [
                    condition,
                    comparison.get("n_pairs"),
                    _fmt(success.get("success_rate_baseline")),
                    _fmt(success.get("success_rate_condition")),
                    _fmt_signed(success.get("risk_difference")),
                    success.get("discordant_baseline_only"),
                    success.get("discordant_condition_only"),
                    _fmt(success.get("mcnemar_p_value"), 4),
                    _fmt(success.get("mcnemar_p_holm"), 4),
                    "yes" if success.get("significant_holm") else "no",
                ]
            )
        metric_stats = comparison.get("metrics", {})
        if isinstance(metric_stats, Mapping):
            for metric in COMPARISON_METRICS:
                payload = metric_stats.get(metric)
                if not isinstance(payload, Mapping) or payload.get("n_pairs", 0) == 0:
                    continue
                interval = "—"
                if payload.get("ci95_low") is not None and payload.get("ci95_high") is not None:
                    interval = "[{}, {}]".format(
                        _fmt_signed(payload.get("ci95_low")), _fmt_signed(payload.get("ci95_high"))
                    )
                metric_compare_rows.append(
                    [
                        condition,
                        METRIC_LABELS.get(metric, metric),
                        payload.get("n_pairs"),
                        _fmt(payload.get("mean_baseline")),
                        _fmt(payload.get("mean_condition")),
                        _fmt_signed(payload.get("mean_diff")),
                        interval,
                        _fmt(payload.get("wilcoxon_p_value"), 4),
                        _fmt(payload.get("wilcoxon_p_holm"), 4),
                        _fmt_signed(payload.get("cliffs_delta"), 3),
                        payload.get("effect_magnitude") or "—",
                        "yes" if payload.get("significant_holm") else "no",
                    ]
                )
    lines.append(
        _md_table(
            [
                "Condition",
                "pairs",
                "proposed success",
                "condition success",
                "risk diff",
                "b (proposed only)",
                "c (condition only)",
                "McNemar p",
                "p (Holm)",
                "significant",
            ],
            success_compare_rows,
        )
    )
    lines.append("")
    lines.append(
        _md_table(
            [
                "Condition",
                "Metric",
                "pairs",
                "proposed mean",
                "condition mean",
                "mean diff",
                "95% CI",
                "Wilcoxon p",
                "p (Holm)",
                "Cliff's delta",
                "effect",
                "significant",
            ],
            metric_compare_rows,
        )
    )
    lines.append("")

    # ---- interpretation ----
    lines.append("## Interpretation Notes")
    lines.append("")
    if not comparison_list:
        lines.append(
            "- No paired comparisons were available (no common `(scenario, seed)` "
            "pairs with the proposed condition)."
        )
    for comparison in comparison_list:
        if not isinstance(comparison, Mapping):
            continue
        condition = comparison.get("condition")
        notes: List[str] = []
        success = comparison.get("success", {})
        if isinstance(success, Mapping) and success.get("success_rate_baseline") is not None:
            risk_diff = success.get("risk_difference") or 0.0
            direction = "higher" if risk_diff > 0 else ("lower" if risk_diff < 0 else "equal")
            verdict = "improvement" if risk_diff > 0 else ("degradation" if risk_diff < 0 else "no change")
            if success.get("significant_holm"):
                notes.append(
                    "success rate is significantly {} than proposed (risk diff {}) — {}".format(
                        direction, _fmt_signed(risk_diff), verdict
                    )
                )
            else:
                notes.append(
                    "success rate difference (risk diff {}) is not significant after Holm correction".format(
                        _fmt_signed(risk_diff)
                    )
                )
        metric_stats = comparison.get("metrics", {})
        significant_metrics: List[str] = []
        if isinstance(metric_stats, Mapping):
            for metric in COMPARISON_METRICS:
                payload = metric_stats.get(metric)
                if not isinstance(payload, Mapping) or not payload.get("significant_holm"):
                    continue
                diff = payload.get("mean_diff")
                if diff is None:
                    continue
                higher_better = HIGHER_IS_BETTER.get(metric, False)
                better_when_positive = bool(higher_better)
                improved = (float(diff) > 0) == better_when_positive
                significant_metrics.append(
                    "{} {} (mean diff {}, Holm p={}, Cliff's delta={})".format(
                        METRIC_LABELS.get(metric, metric),
                        "improved" if improved else "worsened",
                        _fmt_signed(diff),
                        _fmt(payload.get("wilcoxon_p_holm"), 4),
                        _fmt_signed(payload.get("cliffs_delta"), 3),
                    )
                )
        if significant_metrics:
            notes.append("Holm-significant metric changes: " + "; ".join(significant_metrics))
        elif not notes:
            notes.append("no significant differences after Holm correction")
        experimental = " (experimental condition)" if comparison.get("experimental") else ""
        lines.append("- **{}{}**: {}".format(condition, experimental, ". ".join(notes) + "."))
    lines.append("")
    lines.append(
        "Direction reminders: higher `min_obstacle_distance_m` and `min_ttc_s` are "
        "safer; lower `rms_jerk_m_s3`, `max_lateral_accel_m_s2`, "
        "`max_lateral_error_m`, planning latency, collision frames and fallback "
        "rates are better. Completion time is conditional on completing the task."
    )
    lines.append("")

    # ---- method notes ----
    lines.append("## Method Notes")
    lines.append("")
    lines.append(
        "- **Success interval**: Wilson score interval at 95%, computed from the "
        "per-run binary success count."
    )
    lines.append(
        "- **McNemar exact test**: two-sided exact binomial test on discordant "
        "pairs with p=0.5; `b` = proposed succeeds / condition fails, "
        "`c` = proposed fails / condition succeeds."
    )
    lines.append(
        "- **Paired bootstrap**: resamples matched differences "
        "`condition - proposed` with replacement (default 2000 resamples), "
        "percentile 95% CI, seeded for reproducibility."
    )
    lines.append(
        "- **Wilcoxon signed-rank**: two-sided, normal approximation with tie and "
        "zero-difference correction; used as the metric significance test."
    )
    lines.append(
        "- **Cliff's delta**: non-parametric effect size "
        "`P(condition > proposed) - P(condition < proposed)`; magnitudes follow "
        "Romano et al. (<0.147 negligible, <0.33 small, <0.474 medium, else large)."
    )
    lines.append(
        "- **Multiple comparisons**: Holm-Bonferroni (step-down) within each "
        "condition family; reported adjusted p-values are monotone and capped at 1."
    )
    lines.append("")
    return "\n".join(lines)


__all__ = [
    "CONDITIONS",
    "CONDITION_BY_NAME",
    "CONDITION_NAMES",
    "CORE_SCENARIOS",
    "DEFAULT_SEEDS",
    "COMPARISON_METRICS",
    "SUMMARY_METRICS",
    "Condition",
    "PROPOSED_CONDITION",
    "all_conditions",
    "resolve_conditions",
    "wilson_interval",
    "mcnemar_exact_test",
    "wilcoxon_signed_rank",
    "paired_bootstrap_diff",
    "cliffs_delta",
    "cliffs_delta_magnitude",
    "holm_bonferroni",
    "summarize_runs",
    "compare_conditions",
    "build_markdown_report",
    "collect_provenance",
    "runs_to_rows",
    "summary_to_rows",
    "write_runs_csv",
    "write_summary_csv",
]
