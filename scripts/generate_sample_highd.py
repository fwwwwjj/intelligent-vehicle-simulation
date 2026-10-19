#!/usr/bin/env python3
"""
生成符合德国 highD 官方 CSV 规范的微型演示数据集。

本脚本用于在没有下载完整 40GB 真实 highD 数据集时，在本地零功耗生成几十 KB
的微型真实格式切片（包含 3 辆轿车与 1 辆卡车在高速公路上的跟驰与超车过程），
支持 HighDDatasetLoader 即插即用解析、时间窗截取与测试回放。
"""

import argparse
import csv
from pathlib import Path


def generate_sample_highd(output_dir: str = "data/sample_highd", prefix: str = "01") -> Path:
    out_path = Path(output_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    # 1. 录像元数据 (recordingMeta.csv)
    # 包含帧率 (25fps)、车道标线 (lower: 0.0;3.75;7.5, upper: 7.5;11.25)
    recording_meta_file = out_path / f"{prefix}_recordingMeta.csv"
    with recording_meta_file.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "id", "frameRate", "locationId", "speedLimit", "month", "year",
            "startFrame", "endFrame", "duration", "totalDrivenDistance",
            "totalDrivenTime", "numVehicles", "numCars", "numTrucks",
            "lowerLaneMarkings", "upperLaneMarkings",
        ])
        writer.writerow([
            1, 25.0, 1, 120.0, 10, 2017,
            0, 100, 4.0, 500.0,
            16.0, 4, 3, 1,
            "0.0;3.75;7.5", "7.5;11.25;15.0",
        ])

    # 2. 车辆轨迹元数据 (tracksMeta.csv)
    # 车辆类型 (Car/Truck)、尺寸、初始帧与最终帧
    tracks_meta_file = out_path / f"{prefix}_tracksMeta.csv"
    with tracks_meta_file.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "id", "width", "height", "class", "initialFrame", "finalFrame",
            "numFrames", "drivingDirection",
        ])
        writer.writerow([1, 1.9, 4.8, "Car", 0, 100, 101, 1])
        writer.writerow([2, 2.0, 5.0, "Car", 0, 100, 101, 1])
        writer.writerow([3, 2.5, 12.0, "Truck", 0, 100, 101, 1])
        writer.writerow([4, 1.8, 4.6, "Car", 0, 100, 101, 1])

    # 3. 逐帧时空轨迹数据 (tracks.csv)
    # 100 帧（4 秒），步长 dt = 0.04s
    tracks_file = out_path / f"{prefix}_tracks.csv"
    with tracks_file.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "frame", "id", "x", "y", "width", "height",
            "xVelocity", "yVelocity", "xAcceleration", "yAcceleration",
            "laneId", "precedingId", "followingId",
        ])

        dt = 1.0 / 25.0
        # 初始状态定义: [x, y, vx, vy, ax, ay, lane_id, width, height]
        vehicles = {
            1: {"x": 50.0, "y": 1.875, "vx": 20.0, "vy": 0.0, "ax": 0.0, "ay": 0.0, "lane": 2, "w": 1.9, "h": 4.8, "prec": 0, "foll": 2},
            2: {"x": 10.0, "y": 1.875, "vx": 22.0, "vy": 0.0, "ax": -0.2, "ay": 0.0, "lane": 2, "w": 2.0, "h": 5.0, "prec": 1, "foll": 0},
            3: {"x": 80.0, "y": 5.625, "vx": 18.0, "vy": 0.0, "ax": 0.0, "ay": 0.0, "lane": 3, "w": 2.5, "h": 12.0, "prec": 0, "foll": 4},
            4: {"x": 20.0, "y": 5.625, "vx": 24.0, "vy": 0.0, "ax": 0.1, "ay": 0.0, "lane": 3, "w": 1.8, "h": 4.6, "prec": 3, "foll": 0},
        }

        for frame in range(101):
            for veh_id, s in vehicles.items():
                writer.writerow([
                    frame,
                    veh_id,
                    round(s["x"], 3),
                    round(s["y"], 3),
                    s["w"],
                    s["h"],
                    round(s["vx"], 3),
                    round(s["vy"], 3),
                    round(s["ax"], 3),
                    round(s["ay"], 3),
                    s["lane"],
                    s["prec"],
                    s["foll"],
                ])
                # 状态物理步进
                s["vx"] += s["ax"] * dt
                s["vy"] += s["ay"] * dt
                s["x"] += s["vx"] * dt
                s["y"] += s["vy"] * dt

    return out_path


def main():
    parser = argparse.ArgumentParser(description="生成微型 highD 格式演示数据")
    parser.add_argument("--output_dir", type=str, default="data/sample_highd", help="输出路径")
    parser.add_argument("--prefix", type=str, default="01", help="录像编号前缀")
    args = parser.parse_args()

    target = generate_sample_highd(args.output_dir, args.prefix)
    print(f"[OK] Micro highD sample dataset generated successfully at: {target.resolve()}")


if __name__ == "__main__":
    main()
