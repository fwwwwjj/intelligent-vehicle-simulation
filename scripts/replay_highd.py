#!/usr/bin/env python3
"""最小可信版：highD 真实轨迹开环回放 + BEV 截图/GIF。

用法：
    python3 scripts/replay_highd.py --data-root data/raw/highD --recording 01 \
      --start-time 0 --duration 10 --step 0.5 --max-frames 12

只做开环回放：车辆位置直接来自 highD，不跑决策/规划/控制，也不做闭环交互。
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

from src.common.state import VehicleState  # noqa: E402
from src.environment.dataset_loader import HighDDatasetLoader  # noqa: E402
from src.environment.highd_tools import (  # noqa: E402
    build_road_network_from_highd,
    highd_data_kind,
)
from src.visualization.visualizer_2d import Visualizer2D  # noqa: E402


def _ego_from_obstacle(obstacle) -> VehicleState:
    return VehicleState(
        x=obstacle.x,
        y=obstacle.y,
        heading=obstacle.heading,
        velocity=float(np.hypot(obstacle.vx, obstacle.vy)),
        vx=obstacle.vx,
        vy=obstacle.vy,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="highD 开环回放（最小可信版）")
    parser.add_argument("--data-root", default="data/raw/highD")
    parser.add_argument("--recording", default="01")
    parser.add_argument("--tracks", default=None)
    parser.add_argument("--start-time", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--step", type=float, default=0.5)
    parser.add_argument("--ego-id", type=int, default=None)
    parser.add_argument("--max-frames", type=int, default=12)
    parser.add_argument("--output-dir", type=Path, default=Path("results/highd_replay"))
    parser.add_argument("--gif", action="store_true", help="用 Pillow 把截图合成 GIF")
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

    road, lane_centers, lane_map = build_road_network_from_highd(loader)
    kind = highd_data_kind(meta["tracks_file"])
    args.output_dir.mkdir(parents=True, exist_ok=True)
    frames = max(1, int(args.max_frames))
    step = max(1.0 / max(loader.frame_rate, 1e-6), float(args.step))
    image_paths = []

    for index in range(frames):
        t = float(args.start_time) + index * step
        obstacles = loader.get_frame_obstacles(t)
        if not obstacles:
            continue
        if args.ego_id is not None:
            ego_obs = next((obs for obs in obstacles if obs.id == str(args.ego_id)), None)
            if ego_obs is None:
                print(f"[highD] ego_id={args.ego_id} not found at t={t:.2f}s")
                continue
        else:
            ego_obs = obstacles[0]
        surrounding = [obs for obs in obstacles if obs.id != ego_obs.id]
        path = args.output_dir / f"frame_{index:03d}_t{t:05.2f}.png"
        Visualizer2D.save_bev_snapshot(
            _ego_from_obstacle(ego_obs),
            surrounding,
            road,
            None,
            t,
            str(path),
            dpi=110,
        )
        if kind != "highd_recording":
            try:
                from PIL import Image, ImageDraw

                image = Image.open(path).convert("RGB")
                draw = ImageDraw.Draw(image)
                draw.rectangle([(0, 0), (image.width, 34)], fill=(180, 20, 20))
                draw.text((12, 9), "SYNTHETIC SAMPLE - not real highD", fill=(255, 255, 255))
                image.save(path)
            except Exception:
                pass
        image_paths.append(path)
        print(f"[highD] frame {index + 1}/{frames} t={t:.2f}s vehicles={len(obstacles)} -> {path.name}")

    if not image_paths:
        print("[highD] no frames rendered; check time window.")
        return 2

    gif_path = None
    if args.gif:
        try:
            from PIL import Image

            images = [Image.open(path).convert("RGB") for path in image_paths]
            gif_path = args.output_dir / f"highd_replay_{meta['recording_id']}.gif"
            images[0].save(
                gif_path,
                save_all=True,
                append_images=images[1:],
                duration=int(step * 1000),
                loop=0,
            )
        except Exception as exc:  # pragma: no cover - optional
            print(f"[highD] GIF skipped: {exc}")

    summary = {
        "dataset_name": "highD",
        "data_kind": kind,
        "mode": "open_loop_replay",
        "recording_id": meta["recording_id"],
        "tracks_file": meta["tracks_file"],
        "frame_rate": meta["frame_rate"],
        "start_time_s": float(args.start_time),
        "duration_s": float(args.duration),
        "step_s": float(step),
        "frames_rendered": len(image_paths),
        "ego_id": args.ego_id,
        "lane_centers": lane_centers,
        "lane_map": lane_map,
        "images": [str(path) for path in image_paths],
        "gif": str(gif_path) if gif_path else None,
        "note": "open-loop replay only; no decision/planning/control closed loop",
    }
    summary_path = args.output_dir / f"highd_replay_{meta['recording_id']}.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f">>> summary: {summary_path}")
    if kind != "highd_recording":
        print("[highD][WARN] 当前是合成样例，只能验证回放链路，不能写成真实数据实验。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
