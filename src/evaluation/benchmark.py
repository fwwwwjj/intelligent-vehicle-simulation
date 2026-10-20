"""
场景批量自动化基准测试套件 (Benchmark Suite)。
用于算法鲁棒性验证、不同决策算法（FSM vs Game-theory）横向对比及大赛结果图表导出。

所有结果来自真实闭环仿真，支持同一 seed 复现。
"""

import csv
import hashlib
import json
import math
import platform
from pathlib import Path
import subprocess
from typing import Any, Dict, Iterable, List, Optional, Tuple

import numpy as np


class BenchmarkSuite:
    """
    自动化压力测试与批量场景回归评测器
    """

    def __init__(
        self,
        scenario_configs: List[str],
        decisions: Iterable[str] = ("fsm", "game"),
        seeds: Iterable[int] = (42,),
        duration: float = 20.0,
        use_dynamic_model: bool = False,
        planner: Optional[str] = None,
        controller: Optional[str] = None,
        tire_model: Optional[str] = None,
    ):
        self.scenario_configs = scenario_configs
        self.decisions = tuple(decisions)
        self.seeds = tuple(seeds)
        self.duration = float(duration)
        self.use_dynamic_model = use_dynamic_model
        self.planner = planner
        self.controller = controller
        self.tire_model = tire_model
        self.results: Dict[str, Dict[str, Any]] = {}
        self.group_summary: List[Dict[str, Any]] = []

    def _run_once(self, scenario_name: str, decision: str, seed: int) -> Dict[str, Any]:
        # 延迟导入，避免 evaluation -> benchmark -> simulator 的循环依赖
        from src.environment.scenario_loader import load_named_scenario
        from src.runner import build_engine

        scenario = load_named_scenario(scenario_name)
        engine = build_engine(
            scenario,
            decision=decision,
            seed=seed,
            enable_visualization=False,
            use_dynamic_model=self.use_dynamic_model,
            planner=self.planner,
            controller=self.controller,
            tire_model=self.tire_model,
        )
        report = engine.run(max_duration=self.duration)
        return {
            "scenario": scenario_name,
            "decision": decision,
            "seed": seed,
            "planner": self.planner or "config",
            "controller": self.controller or "config",
            "tire_model": self.tire_model or "config",
            "dynamic_model": bool(engine.use_dynamic_model),
            **report,
        }

    def run_all(self) -> Dict[str, Dict[str, Any]]:
        """执行全部基准场景仿真与打分。"""
        self.results = {}
        for scenario_name in self.scenario_configs:
            for decision in self.decisions:
                for seed in self.seeds:
                    key = f"{scenario_name}__{decision}__seed{seed}"
                    print(f">>> running {key} ...")
                    self.results[key] = self._run_once(scenario_name, decision, seed)
        self.group_summary = self.summarize_groups()
        return self.results

    def summarize_groups(self) -> List[Dict[str, Any]]:
        """Summarize repeated runs by scenario and decision; raw runs remain untouched."""
        grouped: Dict[Tuple[str, str], List[Dict[str, Any]]] = {}
        for row in self.results.values():
            grouped.setdefault((str(row["scenario"]), str(row["decision"])), []).append(row)

        summaries: List[Dict[str, Any]] = []
        for (scenario, decision), rows in sorted(grouped.items()):
            metric_names = sorted({
                key for row in rows for key, value in row.items()
                if key not in {"scenario", "decision", "seed"}
                and isinstance(value, (int, float)) and math.isfinite(float(value))
            })
            summary: Dict[str, Any] = {
                "scenario": scenario,
                "decision": decision,
                "sample_count": len(rows),
                "seeds": [int(row["seed"]) for row in rows],
                "success_count": sum(int(float(row.get("success_rate", 0.0)) >= 1.0) for row in rows),
                "success_rate_mean": float(np.mean([float(row.get("success_rate", 0.0)) for row in rows])),
            }
            # Wilson score interval for the per-run binary success outcome.
            n = len(rows)
            p = summary["success_rate_mean"]
            z = 1.959963984540054
            denom = 1.0 + z * z / n
            center = (p + z * z / (2.0 * n)) / denom
            half = z * math.sqrt(p * (1.0 - p) / n + z * z / (4.0 * n * n)) / denom
            summary["success_rate_ci95_low"] = max(0.0, center - half)
            summary["success_rate_ci95_high"] = min(1.0, center + half)

            for metric in metric_names:
                values = np.asarray([float(row[metric]) for row in rows], dtype=float)
                summary[f"{metric}_mean"] = float(np.mean(values))
                summary[f"{metric}_std"] = float(np.std(values, ddof=1)) if n > 1 else 0.0
            summaries.append(summary)
        return summaries

    def _provenance_metadata(self) -> Dict[str, Any]:
        """Capture the checked-out revision and hashes of scenario files when available."""
        try:
            commit = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                check=True,
                capture_output=True,
                text=True,
                cwd=Path(__file__).resolve().parents[2],
            ).stdout.strip()
        except (OSError, subprocess.CalledProcessError):
            commit = None

        scenario_fingerprints = {}
        repo_root = Path(__file__).resolve().parents[2]
        for scenario in self.scenario_configs:
            candidate = Path(scenario)
            if not candidate.is_file():
                candidate = repo_root / "configs" / "scenarios" / f"{scenario}.yaml"
            if candidate.is_file():
                scenario_fingerprints[str(scenario)] = hashlib.sha256(candidate.read_bytes()).hexdigest()
        return {"git_commit": commit, "scenario_sha256": scenario_fingerprints}

    def to_rows(self) -> List[Dict[str, Any]]:
        """把结果展平为适合 CSV 的行。"""
        return [{"key": key, **value} for key, value in self.results.items()]

    def save(self, output_dir: str = "results") -> Tuple[Path, Path, Path]:
        """Save traceable raw runs and grouped statistics as JSON and CSV files."""
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        json_path = out / "benchmark.json"
        csv_path = out / "benchmark.csv"
        summary_csv_path = out / "benchmark_summary.csv"
        if not self.group_summary:
            self.group_summary = self.summarize_groups()
        with json_path.open("w", encoding="utf-8") as fh:
            json.dump({
                "metadata": {
                    "scenario_configs": self.scenario_configs,
                    "decisions": list(self.decisions),
                    "seeds": list(self.seeds),
                    "duration_s": self.duration,
                    "use_dynamic_model": self.use_dynamic_model,
                    "python_version": platform.python_version(),
                    **self._provenance_metadata(),
                    "success_rate_definition": "mean of per-run binary success_rate; Wilson 95% interval",
                    "metric_std_definition": "sample standard deviation (ddof=1); 0 for a single run",
                },
                "runs": self.results,
                "group_summary": self.group_summary,
            }, fh, ensure_ascii=False, indent=2)
        rows = self.to_rows()
        if rows:
            with csv_path.open("w", encoding="utf-8", newline="") as fh:
                writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
                writer.writeheader()
                writer.writerows(rows)
        if self.group_summary:
            with summary_csv_path.open("w", encoding="utf-8", newline="") as fh:
                writer = csv.DictWriter(fh, fieldnames=list(self.group_summary[0].keys()))
                writer.writeheader()
                writer.writerows(self.group_summary)
        return json_path, csv_path, summary_csv_path
