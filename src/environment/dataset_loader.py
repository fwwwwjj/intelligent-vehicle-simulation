"""
真实道路轨迹数据集与场景描述文件加载器。

当前状态（v0.3）：
- ``HighDDatasetLoader`` 已能读取 highD 格式的 ``*_tracks.csv``、
  ``*_recordingMeta.csv``、``*_tracksMeta.csv``，并按时间戳查询某一帧的
  障碍物状态；仓库不附带 highD 原始数据，也**没有**完成 IDM 参数标定。
- INTERACTION / NGSIM / OpenSCENARIO 仍为接口预留或占位实现。

因此文档中可以写“highD 格式解析器已实现”，但不能写“已用 highD 标定”
或“已使用真实数据完成闭环实验”。
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import pandas as pd

from src.common.state import ObstacleState, ObstacleType


_HIGHD_REQUIRED_COLUMNS = (
    "frame",
    "id",
    "x",
    "y",
    "width",
    "height",
    "xVelocity",
    "yVelocity",
)
_HIGHD_OPTIONAL_COLUMNS = (
    "xAcceleration",
    "yAcceleration",
    "laneId",
    "precedingId",
    "followingId",
)


class BaseDatasetLoader(ABC):
    """数据集加载器抽象基类。"""

    @abstractmethod
    def load_scenario(self, file_path: str, scenario_id: Optional[str] = None) -> Dict[str, Any]:
        """
        加载场景并解析为仿真环境可执行的状态序列字典。

        :param file_path: 数据集文件路径（文件或目录）
        :param scenario_id: 场景切片标识
        :return: 数据集元数据字典
        """
        raise NotImplementedError

    @abstractmethod
    def get_frame_obstacles(self, timestamp: float) -> List[ObstacleState]:
        """按时间戳提取当前帧的所有障碍物真实物理量。"""
        raise NotImplementedError


class HighDDatasetLoader(BaseDatasetLoader):
    """
    highD 格式轨迹解析器。

    支持两种输入方式：

    1. 传入单个 ``*_tracks.csv`` 文件；
    2. 传入包含多个 recording 的目录，此时通过 ``scenario_id``（文件前缀，
       例如 ``01``）选择 recording；目录下只有一个 recording 时可以省略。

    坐标口径：直接保留 highD 原始 ``x/y``，不做重定心、旋转或 y 轴翻转。
    ``height`` 按 highD 约定解释为车辆纵向长度，``width`` 为横向宽度。
    调用方若要把轨迹映射到 ``RoadNetwork``，需要自行完成坐标对齐。

    内存提示：highD 单个 ``tracks.csv`` 可能包含数十万行，完整读取会占用
    较多内存。离线分析时建议用 ``max_rows`` 截断，或先用 pandas 按帧/车辆
    过滤后再交给本 loader。
    """

    def __init__(
        self,
        data_root: str = "data/raw/highD",
        vehicle_class_map: Optional[Dict[str, ObstacleType]] = None,
    ):
        self.data_root = Path(data_root)
        self._class_map: Dict[str, ObstacleType] = {
            "car": ObstacleType.CAR,
            "truck": ObstacleType.TRUCK,
            "pedestrian": ObstacleType.PEDESTRIAN,
            "bicycle": ObstacleType.BICYCLE,
        }
        if vehicle_class_map:
            self._class_map.update({str(k).lower(): v for k, v in vehicle_class_map.items()})

        self.tracks_data: Optional[pd.DataFrame] = None
        self.tracks_meta: Optional[pd.DataFrame] = None
        self.recording_meta: Dict[str, Any] = {}
        self.frame_rate: float = 25.0
        self.start_frame: int = 0
        self.end_frame: int = 0
        self.lane_markings: Dict[str, List[float]] = {"lower": [], "upper": []}
        self.lane_width_m: Optional[float] = None
        self.load_warnings: List[str] = []

        self._recording_id: Optional[str] = None
        self._indexed_tracks: Optional[pd.DataFrame] = None
        self._frame_numbers: np.ndarray = np.array([], dtype=int)
        self._track_dims: Dict[int, Tuple[float, float]] = {}
        self._track_class: Dict[int, str] = {}

    # ------------------------------------------------------------------
    # 路径与元数据
    # ------------------------------------------------------------------
    @staticmethod
    def _prefix_from_tracks_path(path: Path) -> str:
        name = path.name
        if name.endswith("_tracks.csv"):
            return name[: -len("_tracks.csv")]
        return path.stem

    def _resolve_paths(
        self,
        file_path: str,
        scenario_id: Optional[str],
    ) -> Tuple[Path, Optional[Path], Optional[Path]]:
        """把文件/目录输入解析为 (tracks, recordingMeta, tracksMeta) 三个路径。"""
        p = Path(file_path)
        if p.is_dir():
            candidates = sorted(p.glob("*_tracks.csv")) or sorted(p.glob("*tracks.csv"))
            if not candidates:
                raise FileNotFoundError(
                    f"No highD '*_tracks.csv' found under {p}. "
                    "Expected e.g. 01_tracks.csv / 01_recordingMeta.csv / 01_tracksMeta.csv."
                )
            if scenario_id is not None:
                wanted = str(scenario_id)
                narrowed = [
                    c for c in candidates
                    if c.name.startswith(wanted) or wanted in c.name
                ]
                if narrowed:
                    candidates = narrowed
            if len(candidates) > 1:
                raise ValueError(
                    f"Multiple highD recordings found under {p}: "
                    f"{[c.name for c in candidates]}. Pass scenario_id to choose one."
                )
            tracks_path = candidates[0]
        else:
            tracks_path = p

        if not tracks_path.exists():
            raise FileNotFoundError(f"highD tracks file not found: {tracks_path}")

        prefix = self._prefix_from_tracks_path(tracks_path)
        recording_meta = tracks_path.with_name(f"{prefix}_recordingMeta.csv")
        tracks_meta = tracks_path.with_name(f"{prefix}_tracksMeta.csv")

        # 兼容直接传入名字就叫 tracks.csv / recordingMeta.csv 的简化目录。
        if not recording_meta.exists():
            alt = tracks_path.with_name("recordingMeta.csv")
            if alt.exists():
                recording_meta = alt
        if not tracks_meta.exists():
            alt = tracks_path.with_name("tracksMeta.csv")
            if alt.exists():
                tracks_meta = alt

        return (
            tracks_path,
            recording_meta if recording_meta.exists() else None,
            tracks_meta if tracks_meta.exists() else None,
        )

    @staticmethod
    def _meta_get(meta: Dict[str, Any], *names: str) -> Any:
        lowered = {str(k).lower(): v for k, v in meta.items()}
        for name in names:
            value = lowered.get(name.lower())
            if value is not None:
                return value
        return None

    @staticmethod
    def _as_int(value: Any) -> Optional[int]:
        if value is None:
            return None
        try:
            if pd.isna(value):
                return None
            return int(float(value))
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _parse_markings(value: Any) -> List[float]:
        if value is None:
            return []
        try:
            if pd.isna(value):
                return []
        except (TypeError, ValueError):
            pass
        text = str(value).strip()
        if not text or text.lower() in {"nan", "none"}:
            return []
        result: List[float] = []
        for part in text.replace(",", ";").split(";"):
            part = part.strip()
            if not part:
                continue
            try:
                result.append(float(part))
            except ValueError:
                continue
        return result

    def _load_recording_meta(self, path: Optional[Path]) -> None:
        self.recording_meta = {}
        if path is None:
            return
        try:
            meta_df = pd.read_csv(path, encoding="utf-8-sig")
        except Exception as exc:  # pragma: no cover - 取决于异常文件
            self.load_warnings.append(f"无法读取 {path.name}: {exc}")
            return
        if meta_df.empty:
            self.load_warnings.append(f"{path.name} 为空文件")
            return
        # 正常 highD 是逗号分隔；保留一次分号兜底，避免不同导出工具差异。
        if len(meta_df.columns) == 1:
            try:
                meta_df = pd.read_csv(path, sep=";", encoding="utf-8-sig")
            except Exception:
                pass
        row = meta_df.iloc[0].to_dict()
        self.recording_meta = {
            str(key): (value.item() if isinstance(value, np.generic) else value)
            for key, value in row.items()
        }

        frame_rate = (
            self._meta_get(self.recording_meta, "frameRate", "frame_rate")
            if self.recording_meta else None
        )
        if frame_rate is not None:
            try:
                self.frame_rate = float(frame_rate)
            except (TypeError, ValueError):
                self.load_warnings.append(f"frameRate={frame_rate!r} 无法解析，回退 25.0")

        lower = self._parse_markings(
            self._meta_get(self.recording_meta, "lowerLaneMarkings", "lower_lane_markings")
        )
        upper = self._parse_markings(
            self._meta_get(self.recording_meta, "upperLaneMarkings", "upper_lane_markings")
        )
        self.lane_markings = {"lower": lower, "upper": upper}
        all_markings = sorted(set(lower + upper))
        if len(all_markings) >= 2:
            diffs = np.diff(np.asarray(all_markings, dtype=float))
            diffs = diffs[diffs > 1e-6]
            if diffs.size:
                self.lane_width_m = float(np.median(diffs))

    def _load_tracks_meta(self, path: Optional[Path]) -> None:
        self.tracks_meta = None
        self._track_dims = {}
        self._track_class = {}
        if path is None:
            return
        try:
            meta_df = pd.read_csv(path, encoding="utf-8-sig")
        except Exception as exc:  # pragma: no cover - 取决于异常文件
            self.load_warnings.append(f"无法读取 {path.name}: {exc}")
            return
        if "id" not in meta_df.columns:
            self.load_warnings.append(f"{path.name} 缺少 id 列，忽略车辆类别/尺寸元数据")
            return

        self.tracks_meta = meta_df
        for _, row in meta_df.iterrows():
            try:
                track_id = int(row["id"])
            except (TypeError, ValueError):
                continue
            vehicle_class = str(row.get("class", "Car")).strip().lower()
            self._track_class[track_id] = vehicle_class
            width = row.get("width")
            height = row.get("height")
            if pd.notna(width) and pd.notna(height):
                try:
                    # highD 的 height 是车身纵向长度，width 是横向宽度
                    self._track_dims[track_id] = (float(height), float(width))
                except (TypeError, ValueError):
                    pass

    # ------------------------------------------------------------------
    # 轨迹读取
    # ------------------------------------------------------------------
    def load_scenario(
        self,
        file_path: str,
        scenario_id: Optional[str] = None,
        max_rows: Optional[int] = None,
    ) -> Dict[str, Any]:
        """
        读取 highD 格式 recording，并建立按帧查询的索引。

        :param file_path: ``*_tracks.csv`` 路径，或包含 recording 的目录
        :param scenario_id: 目录中多个 recording 的前缀，例如 ``01``
        :param max_rows: 可选，只读取前 N 行（用于大文件冒烟/调试）
        :return: 数据集元数据；后续可调用 ``get_frame_obstacles``
        """
        tracks_path, recording_meta_path, tracks_meta_path = self._resolve_paths(
            file_path, scenario_id
        )
        self.load_warnings = []
        self._load_recording_meta(recording_meta_path)
        self._load_tracks_meta(tracks_meta_path)

        header = list(pd.read_csv(tracks_path, nrows=0, encoding="utf-8-sig").columns)
        missing = [col for col in _HIGHD_REQUIRED_COLUMNS if col not in header]
        if missing:
            raise ValueError(
                f"highD tracks file {tracks_path.name} is missing required columns: "
                f"{missing}. Available: {header}"
            )

        usecols = [
            col for col in (*_HIGHD_REQUIRED_COLUMNS, *_HIGHD_OPTIONAL_COLUMNS)
            if col in header
        ]
        dtype: Dict[str, str] = {"frame": "int64", "id": "int64"}
        for col in (
            "x",
            "y",
            "width",
            "height",
            "xVelocity",
            "yVelocity",
            "xAcceleration",
            "yAcceleration",
        ):
            if col in usecols:
                dtype[col] = "float64"

        tracks = pd.read_csv(
            tracks_path,
            usecols=usecols,
            dtype=dtype,
            encoding="utf-8-sig",
            nrows=max_rows,
        )
        if tracks.empty:
            raise ValueError(f"highD tracks file {tracks_path.name} contains no rows")

        tracks = tracks.sort_values(["frame", "id"]).reset_index(drop=True)
        self.tracks_data = tracks
        self._indexed_tracks = tracks.set_index("frame", drop=False).sort_index()
        self._frame_numbers = np.sort(tracks["frame"].unique().astype(int))
        self._recording_id = self._prefix_from_tracks_path(tracks_path)

        data_start = int(tracks["frame"].min())
        data_end = int(tracks["frame"].max())
        meta_start = self._as_int(
            self._meta_get(self.recording_meta, "startFrame", "start_frame")
        )
        meta_end = self._as_int(
            self._meta_get(self.recording_meta, "endFrame", "end_frame")
        )
        self.start_frame = meta_start if meta_start is not None else data_start
        self.end_frame = meta_end if meta_end is not None else data_end
        if self.end_frame < self.start_frame:
            self.start_frame, self.end_frame = self.end_frame, self.start_frame

        total_frames = int(tracks["frame"].nunique())
        return {
            "dataset_name": "highD",
            "recording_id": self._recording_id,
            "tracks_file": str(tracks_path),
            "recording_meta_file": str(recording_meta_path) if recording_meta_path else None,
            "tracks_meta_file": str(tracks_meta_path) if tracks_meta_path else None,
            "frame_rate": float(self.frame_rate),
            "start_frame": int(self.start_frame),
            "end_frame": int(self.end_frame),
            "duration_s": float((self.end_frame - self.start_frame) / max(self.frame_rate, 1e-6)),
            "loaded_frames": total_frames,
            "loaded_rows": int(len(tracks)),
            "total_tracks": int(tracks["id"].nunique()),
            "lane_width_m": self.lane_width_m,
            "lane_markings": {key: list(value) for key, value in self.lane_markings.items()},
            "columns": list(tracks.columns),
            "coordinate_note": (
                "highD raw coordinates are preserved (no recentring/rotation); "
                "height=vehicle length, width=vehicle width. Lane mapping is caller-owned."
            ),
            "warnings": list(self.load_warnings),
            "calibrated": False,
        }

    def _require_loaded(self) -> None:
        if self.tracks_data is None or self._indexed_tracks is None:
            raise RuntimeError("Call load_scenario() before querying highD frames")

    def _frame_for_time(self, timestamp: float) -> int:
        """把相对时间映射到最近的可用 highD 帧号。"""
        self._require_loaded()
        if self._frame_numbers.size == 0:
            raise RuntimeError("No frames loaded")
        target = self.start_frame + int(round(float(timestamp) * self.frame_rate))
        idx = int(np.searchsorted(self._frame_numbers, target))
        if idx <= 0:
            return int(self._frame_numbers[0])
        if idx >= self._frame_numbers.size:
            return int(self._frame_numbers[-1])
        before = int(self._frame_numbers[idx - 1])
        after = int(self._frame_numbers[idx])
        return before if abs(target - before) <= abs(after - target) else after

    def _row_to_obstacle(self, row: pd.Series) -> ObstacleState:
        track_id = int(row["id"])
        vx = float(row["xVelocity"])
        vy = float(row["yVelocity"])
        speed = math.hypot(vx, vy)
        heading = math.atan2(vy, vx) if speed > 0.1 else 0.0

        dims = self._track_dims.get(track_id)
        if dims is None:
            height = row.get("height", 4.8)
            width = row.get("width", 1.9)
            length_m = float(height) if pd.notna(height) else 4.8
            width_m = float(width) if pd.notna(width) else 1.9
        else:
            length_m, width_m = dims

        vehicle_class = self._track_class.get(track_id, "car")
        obstacle_type = self._class_map.get(vehicle_class, ObstacleType.CAR)
        return ObstacleState(
            id=str(track_id),
            obstacle_type=obstacle_type,
            x=float(row["x"]),
            y=float(row["y"]),
            length=length_m,
            width=width_m,
            heading=float(heading),
            vx=vx,
            vy=vy,
        )

    def get_frame_obstacles(self, timestamp: float) -> List[ObstacleState]:
        """返回相对时间 ``timestamp`` 秒最近的 highD 帧障碍物列表。"""
        self._require_loaded()
        frame = self._frame_for_time(timestamp)
        assert self._indexed_tracks is not None
        rows = self._indexed_tracks.loc[frame]
        if isinstance(rows, pd.Series):
            rows = rows.to_frame().T
        obstacles = [self._row_to_obstacle(row) for _, row in rows.iterrows()]
        obstacles.sort(key=lambda obs: (obs.x, obs.id))
        return obstacles

    def get_track_history(self, track_id: int) -> pd.DataFrame:
        """返回单个 highD 车辆的完整轨迹（按帧排序）。"""
        self._require_loaded()
        assert self.tracks_data is not None
        history = self.tracks_data[self.tracks_data["id"] == int(track_id)]
        if history.empty:
            raise KeyError(f"highD track id {track_id} not found")
        return history.copy().reset_index(drop=True)

    def _row_to_dict(self, row: pd.Series) -> Dict[str, Any]:
        """把一行 highD 轨迹转成 JSON 友好的字典。"""
        track_id = int(row["id"])
        dims = self._track_dims.get(track_id)
        if dims is None:
            height = row.get("height", 4.8)
            width = row.get("width", 1.9)
            length_m = float(height) if pd.notna(height) else 4.8
            width_m = float(width) if pd.notna(width) else 1.9
        else:
            length_m, width_m = dims
        vx = float(row["xVelocity"])
        vy = float(row["yVelocity"])
        lane_id = row.get("laneId")
        return {
            "id": track_id,
            "frame": int(row["frame"]),
            "x": float(row["x"]),
            "y": float(row["y"]),
            "vx": vx,
            "vy": vy,
            "heading": float(math.atan2(vy, vx)) if math.hypot(vx, vy) > 0.1 else 0.0,
            "length": length_m,
            "width": width_m,
            "lane_id": int(lane_id) if pd.notna(lane_id) else None,
        }

    def extract_window(
        self,
        start_time: float,
        duration: float,
        ego_id: Optional[int] = None,
        radius: Optional[float] = None,
    ) -> Dict[str, Any]:
        """
        截取一个时间窗，可选地以某辆自车为中心做半径过滤。

        返回结构可直接 ``json.dump``，适合从 highD 中挑选汇入/换道片段后再
        转成项目场景。该函数只做数据切片，不做坐标对齐或 IDM 标定。
        """
        self._require_loaded()
        if duration <= 0.0:
            raise ValueError("duration must be positive")
        assert self.tracks_data is not None

        start_frame = self._frame_for_time(start_time)
        end_frame = self._frame_for_time(start_time + duration)
        if end_frame < start_frame:
            start_frame, end_frame = end_frame, start_frame
        subset = self.tracks_data[
            (self.tracks_data["frame"] >= start_frame)
            & (self.tracks_data["frame"] <= end_frame)
        ]
        if subset.empty:
            raise ValueError("No highD frames in the requested time window")

        frames: List[Dict[str, Any]] = []
        if ego_id is None:
            for frame, group in subset.groupby("frame", sort=True):
                frames.append({
                    "frame": int(frame),
                    "t": float((int(frame) - self.start_frame) / max(self.frame_rate, 1e-6)),
                    "tracks": [self._row_to_dict(row) for _, row in group.iterrows()],
                })
        else:
            ego_rows = subset[subset["id"] == int(ego_id)]
            if ego_rows.empty:
                raise KeyError(f"ego track id {ego_id} not found in the requested window")
            ego_by_frame = {int(row["frame"]): row for _, row in ego_rows.iterrows()}
            for frame in sorted(ego_by_frame):
                ego_row = ego_by_frame[frame]
                group = subset[subset["frame"] == frame]
                nearby: List[Dict[str, Any]] = []
                for _, row in group.iterrows():
                    if int(row["id"]) == int(ego_id):
                        continue
                    if radius is not None:
                        distance = math.hypot(
                            float(row["x"]) - float(ego_row["x"]),
                            float(row["y"]) - float(ego_row["y"]),
                        )
                        if distance > float(radius):
                            continue
                    nearby.append(self._row_to_dict(row))
                nearby.sort(key=lambda item: (item["x"], item["id"]))
                frames.append({
                    "frame": int(frame),
                    "t": float((int(frame) - self.start_frame) / max(self.frame_rate, 1e-6)),
                    "ego": self._row_to_dict(ego_row),
                    "obstacles": nearby,
                })

        return {
            "dataset_name": "highD",
            "recording_id": self._recording_id,
            "frame_rate": float(self.frame_rate),
            "start_time_s": float(start_time),
            "duration_s": float(duration),
            "start_frame": int(start_frame),
            "end_frame": int(end_frame),
            "ego_id": int(ego_id) if ego_id is not None else None,
            "radius_m": float(radius) if radius is not None else None,
            "frames": frames,
        }


class InteractionDatasetLoader(BaseDatasetLoader):
    """
    INTERACTION 数据集解析器**接口骨架**（未读取实际数据文件）。
    """

    def __init__(self, data_root: str = "data/raw/INTERACTION"):
        self.data_root = data_root

    def load_scenario(self, file_path: str, scenario_id: Optional[str] = None) -> Dict[str, Any]:
        return {
            "dataset_name": "INTERACTION",
            "type": "roundabout_or_intersection",
            "scenario_id": scenario_id,
            "calibrated": False,
        }

    def get_frame_obstacles(self, timestamp: float) -> List[ObstacleState]:
        return []


class OpenScenarioParser:
    """
    符合 ASAM OpenSCENARIO 2.0 / 1.x 国际标准的仿真测试场景描述解析器。
    """

    @staticmethod
    def parse_xosc(file_path: str) -> Dict[str, Any]:
        """解析 XML 场景逻辑文件（当前仅占位，未实现 XML 解析）。"""
        return {
            "format": "OpenSCENARIO",
            "path": file_path,
            "status": "interface_only",
        }
