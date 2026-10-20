#!/usr/bin/env python3
"""最小可信版：从 highD 真实跟驰轨迹标定 IDM 参数。

用法：
    python3 scripts/calibrate_highd.py --data-root data/raw/highD --recording 01
    python3 scripts/calibrate_highd.py --tracks data/raw/highD/01_tracks.csv

输出：
    results/highd_calibration_<recording>.json

注意：
- 仓库不附带真实 highD；raw data 受许可限制，不要提交。
- ``data/sample_highd`` 是合成样例，只能验证脚本链路，不能写成“真实标定”。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.environment.dataset_loader import HighDDatasetLoader  # noqa: E402
from src.environment.highd_tools import (  # noqa: E402
    calibrate_highd_samples,
    extract_highd_following_samples,
    highd_data_kind,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="highD IDM 参数标定（最小可信版）")
    parser.add_argument("--data-root", default="data/raw/highD", help="highD 数据目录")
    parser.add_argument("--recording", default="01", help="recording 前缀，如 01")
    parser.add_argument("--tracks", default=None, help="直接指定 *_tracks.csv")
    parser.add_argument("--max-samples", type=int, default=3000)
    parser.add_argument("--train-ratio", type=float, default=0.7)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    data_root = Path(args.data_root)
    tracks = Path(args.tracks) if args.tracks else data_root
    if args.tracks and not tracks.exists():
        print(f"[highD] tracks file not found: {tracks}")
        return 2
    if not args.tracks and not data_root.exists():
        print(f"[highD] data root not found: {data_root}")
        print("请把 highD 的 01_tracks.csv / 01_recordingMeta.csv / 01_tracksMeta.csv 放到该目录。")
        return 2

    loader = HighDDatasetLoader(str(data_root))
    try:
        meta = loader.load_scenario(str(tracks), scenario_id=None if args.tracks else args.recording)
    except Exception as exc:
        print(f"[highD] load failed: {exc}")
        return 2

    samples = extract_highd_following_samples(loader, max_samples=args.max_samples)
    if len(samples) < 10:
        print(f"[highD] only {len(samples)} following samples; check recording/time window/precedingId.")
        return 2

    result = calibrate_highd_samples(samples, train_ratio=args.train_ratio)
    kind = highd_data_kind(meta["tracks_file"])
    payload = {
        "dataset_name": "highD",
        "data_kind": kind,
        "recording_id": meta["recording_id"],
        "tracks_file": meta["tracks_file"],
        "frame_rate": meta["frame_rate"],
        "loaded_frames": meta["loaded_frames"],
        "lane_width_m": meta["lane_width_m"],
        "max_samples_arg": args.max_samples,
        **result,
    }
    output = args.output or Path("results") / f"highd_calibration_{meta['recording_id']}.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"[highD] recording={meta['recording_id']} data_kind={kind} samples={result['sample_count']}")
    print(f"[highD] train={result['train_count']} validation={result['validation_count']}")
    print("[highD] calibrated params:")
    for key, value in result["calibrated_params"].items():
        print(f"  {key:12s}: {value:.4f}")
    print(f"[highD] RMSE train: initial={result['initial_rmse_train']:.4f} "
          f"calibrated={result['calibrated_rmse_train']:.4f}")
    if result["validation_count"]:
        print(f"[highD] RMSE validation: default={result['default_rmse_validation']:.4f} "
              f"calibrated={result['calibrated_rmse_validation']:.4f}")
    if kind != "highd_recording":
        print("[highD][WARN] 当前是合成样例，不能作为真实数据标定结论。")
    print(f">>> JSON: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
