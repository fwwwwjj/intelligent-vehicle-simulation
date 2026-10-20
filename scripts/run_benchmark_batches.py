#!/usr/bin/env python3
"""分批运行核心 benchmark，并在批间休眠，避免机器持续发热。

示例：
    python3 scripts/run_benchmark_batches.py \
      --scenarios highway_merge cut_in emergency_avoidance \
      --seeds 52 53 54 55 56 57 58 59 60 61 \
      --batch-size 5 --duration 20 --sleep 45

行为：
- 每批调用 scripts/run_benchmark.py 写入 results/batches/batch_*；
- 批间 sleep；
- 最后把已有 results/benchmark.json 与新批次合并，重写 benchmark.json/csv/summary.csv。
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.evaluation.benchmark import BenchmarkSuite  # noqa: E402


def _run_batch(args, batch_seeds, batch_dir: Path) -> None:
    cmd = [
        sys.executable, "-B", "scripts/run_benchmark.py",
        "--scenarios", *args.scenarios,
        "--decisions", *args.decisions,
        "--seeds", *[str(seed) for seed in batch_seeds],
        "--duration", str(args.duration),
        "--output-dir", str(batch_dir),
    ]
    env = os.environ.copy()
    for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "MKL_NUM_THREADS"):
        env[key] = "1"
    print(f">>> batch seeds={batch_seeds} -> {batch_dir}")
    subprocess.run(cmd, check=True, env=env)


def main() -> int:
    parser = argparse.ArgumentParser(description="分批运行核心 benchmark 并合并结果")
    parser.add_argument("--scenarios", nargs="+", default=["highway_merge", "cut_in", "emergency_avoidance"])
    parser.add_argument("--decisions", nargs="+", default=["fsm"], choices=["fsm", "game"])
    parser.add_argument("--seeds", nargs="+", type=int, required=True)
    parser.add_argument("--batch-size", type=int, default=5)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--sleep", type=float, default=45.0, help="批间休眠秒数")
    parser.add_argument("--output-dir", type=Path, default=Path("results"))
    parser.add_argument("--base", type=Path, default=Path("results/benchmark.json"), help="已有结果，用于合并")
    parser.add_argument("--no-base", action="store_true", help="不合并已有结果")
    args = parser.parse_args()

    seeds = list(dict.fromkeys(int(seed) for seed in args.seeds))
    batch_size = max(1, int(args.batch_size))
    batches = [seeds[i:i + batch_size] for i in range(0, len(seeds), batch_size)]
    batch_root = args.output_dir / "batches"
    batch_root.mkdir(parents=True, exist_ok=True)

    merged = {}
    if not args.no_base and args.base.exists():
        base = json.loads(args.base.read_text(encoding="utf-8"))
        merged.update(base.get("runs", {}))
        print(f">>> loaded base runs: {len(merged)} from {args.base}")

    for index, batch_seeds in enumerate(batches):
        batch_dir = batch_root / f"batch_{batch_seeds[0]}_{batch_seeds[-1]}"
        _run_batch(args, batch_seeds, batch_dir)
        payload = json.loads((batch_dir / "benchmark.json").read_text(encoding="utf-8"))
        for key, row in payload["runs"].items():
            if key in merged:
                print(f"[WARN] duplicate run {key}; overwrite with batch result")
            merged[key] = row
        if index < len(batches) - 1 and args.sleep > 0:
            print(f">>> sleeping {args.sleep:.0f}s to avoid sustained heat ...")
            time.sleep(args.sleep)

    if not merged:
        print("No runs collected.")
        return 2

    all_seeds = sorted({int(row["seed"]) for row in merged.values()})
    suite = BenchmarkSuite(args.scenarios, decisions=args.decisions, seeds=all_seeds, duration=args.duration)
    suite.results = merged
    suite.group_summary = suite.summarize_groups()
    json_path, csv_path, summary_path = suite.save(str(args.output_dir))
    print(f">>> merged {len(merged)} runs")
    print(f">>> JSON: {json_path}")
    print(f">>> CSV : {csv_path}")
    print(f">>> Group summary CSV: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
