"""Unit tests for the contrast experiment statistics, aggregation and report.

These tests deliberately avoid closed-loop simulations: every statistic is
checked against hand-computed/known values or against exact definitions, and
aggregation/report tests use small synthetic run rows.  The runner itself is
covered by an argument-guard test and by the 3-seed smoke test outside CI.
"""

import importlib.util
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

from src.evaluation.contrast import (
    COMPARISON_METRICS,
    CONDITION_BY_NAME,
    CONDITIONS,
    DEFAULT_SEEDS,
    METRIC_LABELS,
    PROPOSED_CONDITION,
    SUMMARY_METRICS,
    build_markdown_report,
    cliffs_delta,
    cliffs_delta_magnitude,
    compare_conditions,
    holm_bonferroni,
    mcnemar_exact_test,
    paired_bootstrap_diff,
    resolve_conditions,
    summarize_runs,
    wilcoxon_signed_rank,
    wilson_interval,
    write_runs_csv,
    write_summary_csv,
)

PROPOSED = PROPOSED_CONDITION
ABLATION = "ablation_planner_centerline"


def _make_row(condition, scenario, seed, *, success=1.0, **metrics):
    row = {
        "condition": condition,
        "scenario": scenario,
        "seed": seed,
        "success_rate": float(success),
    }
    baseline_metrics = {
        "min_obstacle_distance_m": 3.0,
        "min_ttc_s": 4.0,
        "rms_jerk_m_s3": 2.0,
        "max_lateral_accel_m_s2": 1.0,
        "max_lateral_error_m": 0.05,
        "avg_planning_latency_ms": 2.0,
        "p95_planning_latency_ms": 3.0,
        "task_completion_time_s": 10.0,
        "collision_frames": 0.0,
        "planner_fallback_rate": 0.0,
        "controller_fallback_rate": 0.0,
    }
    baseline_metrics.update(metrics)
    row.update(baseline_metrics)
    return row


def _synthetic_runs():
    """Two scenarios x three seeds x three conditions, with a known success pattern."""
    rows = []
    for condition in (PROPOSED, ABLATION, "ablation_controller_pure_pursuit"):
        for scenario in ("highway_merge", "cut_in"):
            for seed in (42, 43, 44):
                offset = (seed - 42) * 0.1
                condition_offset = 0.0 if condition == PROPOSED else 1.0
                success = 1.0
                if condition != PROPOSED and seed == 44:
                    success = 0.0
                rows.append(
                    _make_row(
                        condition,
                        scenario,
                        seed,
                        success=success,
                        min_obstacle_distance_m=3.0 + offset + condition_offset,
                        min_ttc_s=2.0 + offset + condition_offset,
                        rms_jerk_m_s3=1.5 + offset + condition_offset,
                        max_lateral_accel_m_s2=1.0 + offset + condition_offset,
                        max_lateral_error_m=0.05 + offset + condition_offset,
                        avg_planning_latency_ms=2.0 + offset + condition_offset,
                        p95_planning_latency_ms=3.0 + offset + condition_offset,
                        task_completion_time_s=10.0 + offset + condition_offset,
                    )
                )
    return rows


class TestWilsonInterval(unittest.TestCase):
    def test_known_values(self):
        low, high = wilson_interval(5, 10)
        self.assertAlmostEqual(low, 0.236593, places=5)
        self.assertAlmostEqual(high, 0.763407, places=5)

        low, high = wilson_interval(10, 10)
        self.assertAlmostEqual(low, 0.722467, places=5)
        self.assertAlmostEqual(high, 1.0, places=5)

        low, high = wilson_interval(0, 10)
        self.assertAlmostEqual(low, 0.0, places=5)
        self.assertAlmostEqual(high, 0.277533, places=5)

    def test_zero_samples_is_uninformative(self):
        self.assertEqual(wilson_interval(0, 0), (0.0, 1.0))

    def test_symmetry(self):
        low_two, high_two = wilson_interval(2, 10)
        low_eight, high_eight = wilson_interval(8, 10)
        self.assertAlmostEqual(low_two, 1.0 - high_eight, places=10)
        self.assertAlmostEqual(high_two, 1.0 - low_eight, places=10)

    def test_invalid_counts(self):
        with self.assertRaises(ValueError):
            wilson_interval(11, 10)


class TestMcNemarExact(unittest.TestCase):
    def test_known_counts(self):
        # 2 * P(X <= 2 | n=12, p=0.5) = 2 * (1 + 12 + 66) / 4096
        result = mcnemar_exact_test(10, 2)
        self.assertEqual(result["b"], 10)
        self.assertEqual(result["c"], 2)
        self.assertEqual(result["n_discordant"], 12)
        self.assertAlmostEqual(result["p_value"], 158.0 / 4096.0, places=12)

        # 2 * P(X <= 1 | n=9, p=0.5) = 2 * (1 + 9) / 512
        self.assertAlmostEqual(mcnemar_exact_test(8, 1)["p_value"], 20.0 / 512.0, places=12)
        self.assertEqual(mcnemar_exact_test(0, 0)["p_value"], 1.0)
        self.assertEqual(mcnemar_exact_test(5, 5)["p_value"], 1.0)

    def test_sequence_input(self):
        # baseline success where proposed == 1, condition success where condition == 1
        base = [1, 1, 0, 0]
        cond = [0, 1, 0, 1]
        result = mcnemar_exact_test(base, cond)
        self.assertEqual(result["b"], 1)
        self.assertEqual(result["c"], 1)
        self.assertEqual(result["p_value"], 1.0)

    def test_length_mismatch(self):
        with self.assertRaises(ValueError):
            mcnemar_exact_test([1, 0], [1])


class TestPairedBootstrap(unittest.TestCase):
    def test_identical_pairs_have_zero_interval(self):
        result = paired_bootstrap_diff([1.0, 2.0, 3.0], [1.0, 2.0, 3.0], n_boot=500, seed=7)
        self.assertEqual(result["mean_diff"], 0.0)
        self.assertEqual(result["ci95_low"], 0.0)
        self.assertEqual(result["ci95_high"], 0.0)

    def test_constant_shift(self):
        result = paired_bootstrap_diff([1.0, 2.0, 3.0], [0.0, 1.0, 2.0], n_boot=500, seed=0)
        self.assertAlmostEqual(result["mean_diff"], -1.0)
        self.assertAlmostEqual(result["ci95_low"], -1.0)
        self.assertAlmostEqual(result["ci95_high"], -1.0)

    def test_seed_is_reproducible_and_interval_contains_estimate(self):
        base = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
        cond = [1.2, 1.8, 3.4, 3.6, 5.5, 6.1, 6.8, 8.3]
        first = paired_bootstrap_diff(base, cond, n_boot=1000, seed=3)
        second = paired_bootstrap_diff(base, cond, n_boot=1000, seed=3)
        self.assertEqual(first["ci95_low"], second["ci95_low"])
        self.assertEqual(first["ci95_high"], second["ci95_high"])
        self.assertLessEqual(first["ci95_low"], first["mean_diff"])
        self.assertGreaterEqual(first["ci95_high"], first["mean_diff"])

    def test_empty_and_mismatched(self):
        empty = paired_bootstrap_diff([], [], n_boot=10)
        self.assertEqual(empty["n"], 0)
        self.assertTrue(math.isnan(empty["mean_diff"]))
        with self.assertRaises(ValueError):
            paired_bootstrap_diff([1.0], [1.0, 2.0])


class TestCliffsDelta(unittest.TestCase):
    def test_complete_separation(self):
        self.assertAlmostEqual(cliffs_delta([1, 2, 3], [4, 5, 6]), 1.0)
        self.assertAlmostEqual(cliffs_delta([4, 5, 6], [1, 2, 3]), -1.0)
        self.assertEqual(cliffs_delta([1, 2, 3], [1, 2, 3]), 0.0)

    def test_partial_overlap(self):
        # condition=[2,3,4] vs baseline=[1,2,3]: greater=1+2+3=6, less=1+0+0=1 -> 5/9
        self.assertAlmostEqual(cliffs_delta([1, 2, 3], [2, 3, 4]), 5.0 / 9.0)

    def test_empty_input(self):
        self.assertEqual(cliffs_delta([], [1.0]), 0.0)

    def test_magnitude_thresholds(self):
        self.assertEqual(cliffs_delta_magnitude(0.10), "negligible")
        self.assertEqual(cliffs_delta_magnitude(0.20), "small")
        self.assertEqual(cliffs_delta_magnitude(0.40), "medium")
        self.assertEqual(cliffs_delta_magnitude(0.80), "large")
        self.assertEqual(cliffs_delta_magnitude(-0.80), "large")


class TestWilcoxonSignedRank(unittest.TestCase):
    def test_all_positive_differences_are_significant(self):
        base = list(range(1, 11))
        cond = [value + 1.0 for value in base]
        result = wilcoxon_signed_rank(base, cond)
        self.assertEqual(result["n_nonzero"], 10)
        self.assertEqual(result["n_zeros"], 0)
        self.assertGreater(result["w_plus"], result["w_minus"])
        self.assertLess(result["p_value"], 0.01)
        self.assertGreater(result["p_value"], 0.0)

    def test_sign_symmetry(self):
        base = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        cond = [1.5, 2.4, 3.6, 4.1, 5.8, 6.2]
        forward = wilcoxon_signed_rank(base, cond)
        backward = wilcoxon_signed_rank(cond, base)
        self.assertAlmostEqual(forward["p_value"], backward["p_value"], places=12)

    def test_zero_differences(self):
        result = wilcoxon_signed_rank([1.0, 2.0], [1.0, 2.0])
        self.assertEqual(result["n_nonzero"], 0)
        self.assertEqual(result["p_value"], 1.0)

    def test_length_mismatch(self):
        with self.assertRaises(ValueError):
            wilcoxon_signed_rank([1.0], [1.0, 2.0])


class TestHolmBonferroni(unittest.TestCase):
    def test_known_values(self):
        self.assertEqual(
            holm_bonferroni([0.01, 0.02, 0.03, 0.04]),
            [0.04, 0.06, 0.06, 0.06],
        )
        self.assertEqual(
            holm_bonferroni([0.01, 0.04, 0.03]),
            [0.03, 0.06, 0.06],
        )

    def test_clipping_and_empty(self):
        self.assertEqual(holm_bonferroni([0.9, 0.95, 0.99]), [1.0, 1.0, 1.0])
        self.assertEqual(holm_bonferroni([]), [])

    def test_non_finite_is_treated_as_one(self):
        adjusted = holm_bonferroni([0.01, float("nan"), 0.02])
        self.assertEqual(adjusted[1], 1.0)
        self.assertLessEqual(adjusted[0], 1.0)


class TestSummaryAggregation(unittest.TestCase):
    def test_per_condition_scenario_aggregates(self):
        rows = [
            _make_row(
                PROPOSED,
                "highway_merge",
                42,
                min_obstacle_distance_m=2.0,
                min_ttc_s=999.0,
                task_completion_time_s=10.0,
                collision_frames=1.0,
            ),
            _make_row(
                PROPOSED,
                "highway_merge",
                43,
                min_obstacle_distance_m=4.0,
                min_ttc_s=3.0,
                task_completion_time_s=999.0,
                collision_frames=0.0,
                success=0.0,
            ),
            _make_row(
                PROPOSED,
                "highway_merge",
                44,
                min_obstacle_distance_m=6.0,
                min_ttc_s=4.0,
                task_completion_time_s=20.0,
                collision_frames=2.0,
            ),
        ]
        summary = summarize_runs(rows)
        self.assertEqual(summary["n_runs"], 3)
        group = summary["by_condition_scenario"]["{}::highway_merge".format(PROPOSED)]
        self.assertEqual(group["n"], 3)
        self.assertEqual(group["success_count"], 2)
        self.assertAlmostEqual(group["success_rate_mean"], 2.0 / 3.0)
        low, high = wilson_interval(2, 3)
        self.assertAlmostEqual(group["success_ci95_low"], low)
        self.assertAlmostEqual(group["success_ci95_high"], high)
        self.assertEqual(group["metrics"]["min_obstacle_distance_m"]["mean"], 4.0)
        self.assertAlmostEqual(group["metrics"]["min_obstacle_distance_m"]["std"], 2.0)
        # min_ttc_s sentinel 999 is filtered out.
        self.assertEqual(group["metrics"]["min_ttc_s"]["n"], 2)
        self.assertAlmostEqual(group["metrics"]["min_ttc_s"]["mean"], 3.5)
        # completion time 999 (did not finish) is filtered out.
        self.assertEqual(group["metrics"]["task_completion_time_s"]["n"], 2)
        self.assertAlmostEqual(group["metrics"]["task_completion_time_s"]["mean"], 15.0)
        self.assertEqual(group["metrics"]["collision_frames"]["mean"], 1.0)

        pooled = summary["by_condition"][PROPOSED]
        self.assertEqual(pooled["n"], 3)
        self.assertEqual(pooled["condition"], PROPOSED)

    def test_accepts_benchmark_results_dict_and_empties(self):
        rows = {"run-a": _make_row(PROPOSED, "highway_merge", 42)}
        summary = summarize_runs(rows)
        self.assertEqual(summary["n_runs"], 1)
        self.assertEqual(summary["by_condition_scenario"]["{}::highway_merge".format(PROPOSED)]["n"], 1)

        empty = summarize_runs([])
        self.assertEqual(empty["n_runs"], 0)
        self.assertEqual(empty["by_condition_scenario"], {})

    def test_metric_metadata_and_tracked_metrics(self):
        summary = summarize_runs([_make_row(PROPOSED, "highway_merge", 42)])
        group = summary["by_condition_scenario"]["{}::highway_merge".format(PROPOSED)]
        for metric in SUMMARY_METRICS:
            self.assertIn(metric, group["metrics"])
        self.assertIn("min_ttc_s", summary["metric_definitions"])
        self.assertTrue(summary["metric_definitions"]["min_obstacle_distance_m"]["higher_is_better"])


class TestCompareConditions(unittest.TestCase):
    def test_pairs_on_common_scenario_and_seed(self):
        runs = _synthetic_runs()
        # An extra seed only present for the proposed condition must not be paired.
        runs.append(_make_row(PROPOSED, "highway_merge", 45))
        result = compare_conditions(runs, n_boot=200, seed=0)
        self.assertEqual(result["baseline"], PROPOSED)
        by_name = {entry["condition"]: entry for entry in result["comparisons"]}
        comparison = by_name[ABLATION]
        self.assertEqual(comparison["n_pairs"], 6)  # 2 scenarios x 3 common seeds
        self.assertEqual(comparison["scenarios"], ["cut_in", "highway_merge"])

        success = comparison["success"]
        # Both conditions succeed on seeds 42/43; the ablation fails seed 44.
        self.assertEqual(success["discordant_baseline_only"], 2)
        self.assertEqual(success["discordant_condition_only"], 0)
        self.assertAlmostEqual(success["risk_difference"], -2.0 / 6.0)
        self.assertIsNotNone(success["mcnemar_p_holm"])

        jerk = comparison["metrics"]["rms_jerk_m_s3"]
        self.assertEqual(jerk["n_pairs"], 6)
        self.assertAlmostEqual(jerk["mean_diff"], 1.0)
        self.assertAlmostEqual(jerk["ci95_low"], 1.0)
        self.assertAlmostEqual(jerk["ci95_high"], 1.0)
        self.assertAlmostEqual(jerk["cliffs_delta"], 1.0)
        self.assertEqual(jerk["effect_magnitude"], "large")
        self.assertIsNotNone(jerk["wilcoxon_p_holm"])

    def test_missing_baseline_yields_no_comparisons(self):
        runs = [_make_row(ABLATION, "highway_merge", 42)]
        result = compare_conditions(runs)
        self.assertEqual(result["comparisons"], [])

    def test_metric_filters_are_respected(self):
        runs = []
        for seed in (42, 43):
            runs.append(_make_row(PROPOSED, "highway_merge", seed, min_ttc_s=999.0, task_completion_time_s=999.0))
            runs.append(_make_row(ABLATION, "highway_merge", seed, min_ttc_s=999.0, task_completion_time_s=999.0))
        result = compare_conditions(runs, n_boot=50, seed=0)
        comparison = result["comparisons"][0]
        # All filtered values are sentinels -> no paired metric samples.
        self.assertEqual(comparison["metrics"]["min_ttc_s"]["n_pairs"], 0)
        self.assertEqual(comparison["metrics"]["min_ttc_s"]["mean_diff"], None)
        self.assertEqual(comparison["metrics"]["task_completion_time_s"]["n_pairs"], 0)
        # A skipped metric must not shift the Holm-adjusted p-values of the
        # remaining metrics.
        self.assertIsNotNone(comparison["metrics"]["controller_fallback_rate"]["wilcoxon_p_holm"])
        self.assertEqual(comparison["metrics"]["controller_fallback_rate"]["wilcoxon_p_holm"], 1.0)


class TestReport(unittest.TestCase):
    def _report(self):
        runs = _synthetic_runs()
        data = {
            "metadata": {
                "timestamp_utc": "2026-10-20T00:00:00Z",
                "git_commit": "abcdef0",
                "git_dirty": False,
                "python_version": "3.9.6",
                "scenario_configs": ["highway_merge", "cut_in"],
                "seeds": [42, 43, 44],
                "duration_s": 20.0,
                "n_boot": 200,
                "bootstrap_seed": 0,
            },
            "baseline": PROPOSED,
            "conditions": {condition.name: condition.to_dict() for condition in CONDITIONS},
            "runs": {str(index): row for index, row in enumerate(runs)},
        }
        data["summary"] = summarize_runs(runs)
        data["comparisons"] = compare_conditions(runs, n_boot=200, seed=0)
        return build_markdown_report(data)

    def test_contains_expected_sections(self):
        report = self._report()
        for section in (
            "# Contrast Experiment Report",
            "## Metadata",
            "## Condition Definitions",
            "## Success Rate by Condition and Scenario",
            "## Metric Summary by Condition and Scenario",
            "## Paired Comparisons vs Proposed",
            "## Interpretation Notes",
            "## Method Notes",
        ):
            self.assertIn(section, report)
        self.assertIn("McNemar", report)
        self.assertIn("Cliff's delta", report)
        self.assertIn(PROPOSED, report)
        self.assertIn(ABLATION, report)
        self.assertIn(METRIC_LABELS["min_obstacle_distance_m"], report)

    def test_recomputes_from_raw_runs(self):
        runs = _synthetic_runs()
        data = {
            "metadata": {"seeds": [42, 43, 44], "scenario_configs": ["highway_merge", "cut_in"]},
            "conditions": {condition.name: condition.to_dict() for condition in CONDITIONS},
            "runs": {str(index): row for index, row in enumerate(runs)},
        }
        report = build_markdown_report(data)
        self.assertIn("## Paired Comparisons vs Proposed", report)
        self.assertIn("ablation_controller_pure_pursuit", report)

    def test_single_condition_report_is_graceful(self):
        runs = [_make_row(PROPOSED, "highway_merge", seed) for seed in (42, 43, 44)]
        report = build_markdown_report({"metadata": {}, "runs": runs, "conditions": CONDITIONS[0].to_dict()})
        self.assertIn("No paired comparisons were available", report)


class TestCsvHelpers(unittest.TestCase):
    def test_write_runs_and_summary_csv(self):
        runs = _synthetic_runs()[:3]
        summary = summarize_runs(runs)
        with tempfile.TemporaryDirectory() as directory:
            runs_path = write_runs_csv(Path(directory) / "runs.csv", runs)
            summary_path = write_summary_csv(Path(directory) / "summary.csv", summary)
            self.assertTrue(runs_path.is_file())
            self.assertTrue(summary_path.is_file())
            header = runs_path.read_text(encoding="utf-8").splitlines()[0]
            self.assertIn("condition", header)
            self.assertIn("scenario", header)
            summary_header = summary_path.read_text(encoding="utf-8").splitlines()[0]
            self.assertIn("success_rate_mean", summary_header)
            self.assertIn("min_obstacle_distance_m_mean", summary_header)


class TestConditions(unittest.TestCase):
    def test_matrix_matches_agreed_design(self):
        names = [condition.name for condition in CONDITIONS]
        self.assertEqual(len(names), 11)
        self.assertEqual(len(set(names)), 11)
        self.assertEqual(names[0], PROPOSED)
        self.assertEqual(list(CONDITION_BY_NAME), names)
        self.assertEqual(len(DEFAULT_SEEDS), 30)
        self.assertEqual(DEFAULT_SEEDS[0], 42)
        self.assertEqual(DEFAULT_SEEDS[-1], 71)

        experimental = {condition.name for condition in CONDITIONS if condition.experimental}
        self.assertEqual(
            experimental,
            {
                "ablation_decision_game",
                "ablation_planner_em",
                "ablation_controller_mpc",
                "ablation_model_pacejka",
            },
        )
        self.assertEqual(CONDITION_BY_NAME["baseline_idm_centerline_pure_pursuit"].decision, "idm")
        self.assertEqual(CONDITION_BY_NAME["baseline_idm_centerline_pure_pursuit"].planner, "centerline")
        self.assertEqual(
            CONDITION_BY_NAME["baseline_idm_centerline_pure_pursuit"].controller, "pure_pursuit"
        )
        pacejka = CONDITION_BY_NAME["ablation_model_pacejka"]
        self.assertTrue(pacejka.dynamic_model)
        self.assertEqual(pacejka.tire_model, "pacejka")

    def test_resolve_conditions(self):
        self.assertEqual(len(resolve_conditions()), 11)
        self.assertEqual(len(resolve_conditions([])), 11)
        resolved = resolve_conditions([ABLATION])
        self.assertEqual([condition.name for condition in resolved], [ABLATION])
        with self.assertRaises(KeyError):
            resolve_conditions(["does_not_exist"])

    def test_comparison_metrics_tracked_in_summary(self):
        self.assertEqual(tuple(SUMMARY_METRICS), tuple(COMPARISON_METRICS))
        self.assertIn("planner_fallback_rate", SUMMARY_METRICS)
        self.assertIn("controller_fallback_rate", SUMMARY_METRICS)


class TestScripts(unittest.TestCase):
    def test_run_contrast_requires_allow_experimental(self):
        command = [
            sys.executable,
            "-B",
            str(ROOT / "scripts" / "run_contrast.py"),
            "--conditions",
            "ablation_decision_game",
            "--scenarios",
            "highway_merge",
            "--seeds",
            "42",
            "--duration",
            "0.1",
        ]
        completed = subprocess.run(command, capture_output=True, text=True, cwd=str(ROOT), timeout=120)
        self.assertEqual(completed.returncode, 2, completed.stderr)
        self.assertIn("--allow-experimental", completed.stdout)

    def test_analyze_recompute_roundtrip(self):
        spec = importlib.util.spec_from_file_location(
            "analyze_contrast_under_test", ROOT / "scripts" / "analyze_contrast.py"
        )
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)

        runs = _synthetic_runs()
        payload = {
            "metadata": {"n_boot": 100, "bootstrap_seed": 0, "scenario_configs": ["highway_merge"]},
            "baseline": PROPOSED,
            "conditions": {condition.name: condition.to_dict() for condition in CONDITIONS},
            "runs": {str(index): row for index, row in enumerate(runs)},
        }
        with tempfile.TemporaryDirectory() as directory:
            input_path = Path(directory) / "contrast.json"
            input_path.write_text(json.dumps(payload), encoding="utf-8")
            loaded = module.load_contrast(input_path)
            recomputed = module.recompute(loaded, n_boot=50, seed=1)
            summary = recomputed["summary"]
            comparisons = recomputed["comparisons"]
            self.assertEqual(summary["n_runs"], len(runs))
            self.assertEqual(comparisons["n_boot"], 50)
            self.assertEqual(comparisons["bootstrap_seed"], 1)
            self.assertEqual(recomputed["metadata"]["n_boot"], 50)


if __name__ == "__main__":
    unittest.main()
