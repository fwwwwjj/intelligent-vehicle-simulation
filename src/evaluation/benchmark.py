"""
场景批量自动化基准测试套件 (Benchmark Suite)。
用于算法鲁棒性验证、不同决策算法（FSM vs Game-theory）横向对比及大赛结果图表导出。

所有结果来自真实闭环仿真，支持同一 seed 复现。
"""

import csv
import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple


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
    ):
        self.scenario_configs = scenario_configs
        self.decisions = tuple(decisions)
        self.seeds = tuple(seeds)
        self.duration = float(duration)
        self.use_dynamic_model = use_dynamic_model
        self.results: Dict[str, Dict[str, Any]] = {}

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
        )
        report = engine.run(max_duration=self.duration)
        return {
            "scenario": scenario_name,
            "decision": decision,
            "seed": seed,
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
        return self.results

    def to_rows(self) -> List[Dict[str, Any]]:
        """把结果展平为适合 CSV 的行。"""
        return [{"key": key, **value} for key, value in self.results.items()]

    def save(self, output_dir: str = "results") -> Tuple[Path, Path]:
        """保存 JSON 与 CSV，返回两个文件路径。"""
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        json_path = out / "benchmark.json"
        csv_path = out / "benchmark.csv"
        with json_path.open("w", encoding="utf-8") as fh:
            json.dump(self.results, fh, ensure_ascii=False, indent=2)
        rows = self.to_rows()
        if rows:
            with csv_path.open("w", encoding="utf-8", newline="") as fh:
                writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
                writer.writeheader()
                writer.writerows(rows)
        return json_path, csv_path
