"""highD 最小可信版工具：跟驰样本提取、IDM 标定、路网构建。

定位：只做“真实数据 → 跟驰样本 → IDM 参数”和“真实轨迹 → 直线路网 → 开环回放”
两件事；不做全量生产标定，也不做闭环交互回放。

当前仓库不附带真实 highD 数据；``data/sample_highd`` 是合成样例，仅用于
验证脚本链路。真实数据请按官方许可放到 ``data/raw/highD/``（已被 gitignore）。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from src.environment.calibration import IDMCalibrator
from src.environment.dataset_loader import HighDDatasetLoader
from src.environment.obstacle_manager import IntelligentDriverModel
from src.environment.road_network import Lane, RoadNetwork


def highd_data_kind(path: str | Path) -> str:
    """区分合成样例与真实 highD recording，避免把样例写成真实标定。"""
    return "synthetic_sample" if "sample_highd" in str(path) else "highd_recording"


def extract_highd_following_samples(
    loader: HighDDatasetLoader,
    max_samples: int = 5000,
    min_gap: float = 1.0,
    min_speed: float = 1.0,
) -> List[Dict[str, float]]:
    """从 highD tracks 中提取同车道跟驰样本。

    使用 highD 的 ``precedingId`` 直接找前车；gap 用两车纵向 x 差减去半车长。
    加速度优先用 ``xAcceleration``，缺失时用同一 track 的 xVelocity 差分。
    """
    if loader.tracks_data is None:
        raise RuntimeError("Call HighDDatasetLoader.load_scenario() first")

    df = loader.tracks_data
    required = {"frame", "id", "x", "height", "xVelocity", "precedingId"}
    missing = required.difference(df.columns)
    if missing:
        raise ValueError(f"highD tracks missing columns for calibration: {sorted(missing)}")

    work = df[["frame", "id", "x", "height", "xVelocity", "precedingId"]].copy()
    work["precedingId"] = work["precedingId"].fillna(0).astype("int64")

    if "xAcceleration" in df.columns:
        work["observed_accel"] = df["xAcceleration"].to_numpy(dtype=float)
    else:
        work["observed_accel"] = (
            df.groupby("id", sort=False)["xVelocity"].diff().to_numpy(dtype=float)
            * float(loader.frame_rate)
        )
        work["observed_accel"] = work["observed_accel"].fillna(0.0)

    lead = df[["frame", "id", "x", "height", "xVelocity"]].copy()
    lead.columns = ["frame", "precedingId", "lead_x", "lead_height", "lead_speed"]
    merged = work.merge(lead, on=["frame", "precedingId"], how="inner")
    if merged.empty:
        return []

    length_self = merged["height"].fillna(4.8).to_numpy(dtype=float)
    length_lead = merged["lead_height"].fillna(4.8).to_numpy(dtype=float)
    gap = merged["lead_x"].to_numpy(dtype=float) - merged["x"].to_numpy(dtype=float) - 0.5 * (
        length_self + length_lead
    )
    current_speed = merged["xVelocity"].to_numpy(dtype=float)
    lead_speed = merged["lead_speed"].to_numpy(dtype=float)
    accel = merged["observed_accel"].to_numpy(dtype=float)

    mask = (
        np.isfinite(gap)
        & np.isfinite(current_speed)
        & np.isfinite(lead_speed)
        & np.isfinite(accel)
        & (gap >= float(min_gap))
        & (current_speed >= float(min_speed))
        & (np.abs(accel) <= 10.0)
    )
    if not np.any(mask):
        return []

    samples = [
        {
            "current_speed": float(current_speed[i]),
            "lead_speed": float(lead_speed[i]),
            "gap": float(gap[i]),
            "observed_accel": float(accel[i]),
        }
        for i in np.nonzero(mask)[0]
    ]
    if max_samples > 0 and len(samples) > max_samples:
        indices = np.linspace(0, len(samples) - 1, int(max_samples)).astype(int)
        samples = [samples[int(i)] for i in indices]
    return samples


def split_samples(
    samples: List[Dict[str, float]], train_ratio: float = 0.7
) -> Tuple[List[Dict[str, float]], List[Dict[str, float]]]:
    """按时间顺序切分训练/验证集；样本很少时验证集为空。"""
    if len(samples) < 10:
        return list(samples), []
    cut = int(round(len(samples) * float(np.clip(train_ratio, 0.1, 0.9))))
    cut = max(1, min(len(samples) - 1, cut))
    return list(samples[:cut]), list(samples[cut:])


def calibrate_highd_samples(
    samples: List[Dict[str, float]],
    train_ratio: float = 0.7,
    calibrator: Optional[IDMCalibrator] = None,
) -> Dict[str, Any]:
    """在 highD 跟驰样本上标定 IDM，并给出训练/验证 RMSE。"""
    if not samples:
        raise ValueError("No following samples; check recording/precedingId/time window.")
    calibrator = calibrator or IDMCalibrator()
    train, validation = split_samples(samples, train_ratio=train_ratio)
    result = calibrator.calibrate(train)

    calibrated_idm = IntelligentDriverModel(**result["calibrated_params"])
    default_idm = IntelligentDriverModel()
    calibrated_rmse_val = (
        calibrator.compute_rmse(calibrated_idm, validation) if validation else None
    )
    default_rmse_val = (
        calibrator.compute_rmse(default_idm, validation) if validation else None
    )
    return {
        "sample_count": len(samples),
        "train_count": len(train),
        "validation_count": len(validation),
        "calibrated_params": result["calibrated_params"],
        "initial_rmse_train": result["initial_rmse"],
        "calibrated_rmse_train": result["final_rmse"],
        "default_rmse_validation": default_rmse_val,
        "calibrated_rmse_validation": calibrated_rmse_val,
        "success": result["success"],
        "train_ratio": float(train_ratio),
    }


def build_road_network_from_highd(
    loader: HighDDatasetLoader,
    x_padding: float = 20.0,
    sample_step: float = 2.0,
) -> Tuple[RoadNetwork, List[float], Dict[int, str]]:
    """把 highD 的直线高速场景近似成项目 RoadNetwork。

    - 车道中心优先由 recordingMeta 的 lower/upper lane markings 生成；
    - 没有 markings 时，用各 laneId 的 y 中位数；
    - highD laneId 映射到最近的 ``main_lane_*``，仅用于可视化/开环回放。
    """
    if loader.tracks_data is None:
        raise RuntimeError("Call HighDDatasetLoader.load_scenario() first")
    df = loader.tracks_data
    x_min = float(df["x"].min()) - float(x_padding)
    x_max = float(df["x"].max()) + float(x_padding)

    lane_centers: List[float] = []
    lower = list(loader.lane_markings.get("lower", []))
    upper = list(loader.lane_markings.get("upper", []))
    boundaries = sorted(set(float(v) for v in lower + upper))
    if len(boundaries) >= 2:
        lane_centers = [(boundaries[i] + boundaries[i + 1]) / 2.0 for i in range(len(boundaries) - 1)]

    lane_map: Dict[int, str] = {}
    if not lane_centers and "laneId" in df.columns:
        medians = (
            df.dropna(subset=["laneId"])
            .groupby("laneId")["y"]
            .median()
            .sort_values()
        )
        lane_centers = [float(v) for v in medians.tolist()]
        for lane_id, y in medians.items():
            lane_map[int(lane_id)] = f"main_lane_{lane_centers.index(float(y))}"
    if not lane_centers:
        lane_centers = [0.0]

    if not lane_map and "laneId" in df.columns:
        for lane_id, group in df.dropna(subset=["laneId"]).groupby("laneId"):
            median_y = float(group["y"].median())
            nearest = int(np.argmin([abs(median_y - c) for c in lane_centers]))
            lane_map[int(lane_id)] = f"main_lane_{nearest}"

    road = RoadNetwork()
    step = max(0.5, float(sample_step))
    xs = np.arange(x_min, x_max + step, step)
    for index, center_y in enumerate(lane_centers):
        road.add_lane(
            Lane(
                lane_id=f"main_lane_{index}",
                width=float(loader.lane_width_m or 3.75),
                speed_limit=33.33,
                center_points=[(float(x), float(center_y)) for x in xs],
            )
        )
    for lane_id in road.lanes:
        road.generate_reference_line(lane_id, sample_step=1.0)
    return road, lane_centers, lane_map
