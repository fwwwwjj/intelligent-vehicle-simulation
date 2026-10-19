"""
highD 格式 CSV 解析器测试。

测试数据全部在临时目录内生成，不依赖真实 highD 数据文件，也不会把
数据文件提交到仓库（.gitignore 已忽略 data/raw 与 *.csv）。
"""

import csv
import tempfile
import unittest
from pathlib import Path

try:
    import pandas as pd  # noqa: F401
    _HAS_PANDAS = True
except Exception:  # pragma: no cover - 取决于运行环境
    _HAS_PANDAS = False

if _HAS_PANDAS:
    from src.common.state import ObstacleType
    from src.environment.dataset_loader import HighDDatasetLoader


_TRACK_HEADER = [
    "frame", "id", "x", "y", "width", "height",
    "xVelocity", "yVelocity", "xAcceleration", "yAcceleration",
    "laneId", "precedingId", "followingId",
]

_TRACK_ROWS = [
    [0, 1, 0.0, 0.0, 1.9, 4.8, 20.0, 0.0, 0.0, 0.0, 2, 0, 2],
    [0, 2, -20.0, 3.75, 2.0, 6.0, 15.0, 0.0, 0.0, 0.0, 3, 0, 0],
    [25, 1, 20.0, 0.0, 1.9, 4.8, 20.0, 0.0, 0.0, 0.0, 2, 0, 2],
    [25, 2, -5.0, 3.75, 2.0, 6.0, 15.0, 0.0, 0.0, 0.0, 3, 0, 0],
    [50, 1, 40.0, 0.0, 1.9, 4.8, 20.0, 0.0, 0.0, 0.0, 2, 0, 2],
    [50, 2, 10.0, 3.75, 2.0, 6.0, 15.0, 0.0, 0.0, 0.0, 3, 0, 0],
]


def _write_csv(path: Path, header, rows) -> None:
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(header)
        writer.writerows(rows)


@unittest.skipUnless(_HAS_PANDAS, "pandas is required for highD loader tests")
class TestHighDDatasetLoader(unittest.TestCase):

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        self._write_recording(self.root, prefix="01")

    def tearDown(self):
        self._tmp.cleanup()

    @staticmethod
    def _write_recording(root: Path, prefix: str = "01", omit_column: str = "") -> None:
        meta = (
            "id,frameRate,locationId,speedLimit,month,year,startFrame,endFrame,"
            "duration,totalDrivenDistance,totalDrivenTime,numVehicles,numCars,"
            "numTrucks,lowerLaneMarkings,upperLaneMarkings\n"
            "1,25,1,120,10,2017,0,50,2.0,100,2.0,2,1,1,"
            "0.0;3.75;7.5,7.5;11.25\n"
        )
        (root / f"{prefix}_recordingMeta.csv").write_text(meta, encoding="utf-8")

        tracks_meta = (
            "id,width,height,class,initialFrame,finalFrame,numFrames,drivingDirection\n"
            "1,1.9,4.8,Car,0,50,3,1\n"
            "2,2.0,6.0,Truck,0,50,3,1\n"
        )
        (root / f"{prefix}_tracksMeta.csv").write_text(tracks_meta, encoding="utf-8")

        header = [col for col in _TRACK_HEADER if col != omit_column]
        rows = []
        for row in _TRACK_ROWS:
            values = dict(zip(_TRACK_HEADER, row))
            rows.append([values[col] for col in header])
        _write_csv(root / f"{prefix}_tracks.csv", header, rows)

    def test_load_scenario_metadata(self):
        loader = HighDDatasetLoader(str(self.root))
        meta = loader.load_scenario(str(self.root / "01_tracks.csv"))

        self.assertEqual(meta["dataset_name"], "highD")
        self.assertEqual(meta["frame_rate"], 25.0)
        self.assertEqual(meta["total_tracks"], 2)
        self.assertEqual(meta["loaded_frames"], 3)
        self.assertAlmostEqual(meta["lane_width_m"], 3.75)
        self.assertFalse(meta["calibrated"])
        self.assertEqual(meta["warnings"], [])

    def test_directory_resolution_with_scenario_id(self):
        loader = HighDDatasetLoader(str(self.root))
        meta = loader.load_scenario(str(self.root), scenario_id="01")
        self.assertEqual(meta["recording_id"], "01")

    def test_get_frame_obstacles_by_timestamp(self):
        loader = HighDDatasetLoader(str(self.root))
        loader.load_scenario(str(self.root / "01_tracks.csv"))

        obstacles = loader.get_frame_obstacles(1.0)  # 帧 25
        by_id = {obs.id: obs for obs in obstacles}
        self.assertEqual(len(by_id), 2)
        self.assertAlmostEqual(by_id["1"].x, 20.0)
        self.assertAlmostEqual(by_id["2"].x, -5.0)
        self.assertEqual(by_id["2"].obstacle_type, ObstacleType.TRUCK)
        self.assertAlmostEqual(by_id["2"].length, 6.0)
        self.assertAlmostEqual(by_id["2"].width, 2.0)

        # 超出末尾时钳制到最后一帧；负时间钳制到第一帧
        end_obs = {obs.id: obs for obs in loader.get_frame_obstacles(99.0)}
        start_obs = {obs.id: obs for obs in loader.get_frame_obstacles(-1.0)}
        self.assertAlmostEqual(end_obs["1"].x, 40.0)
        self.assertAlmostEqual(start_obs["1"].x, 0.0)

    def test_extract_window_with_ego_and_radius(self):
        loader = HighDDatasetLoader(str(self.root))
        loader.load_scenario(str(self.root / "01_tracks.csv"))

        window = loader.extract_window(0.0, 1.0, ego_id=1, radius=30.0)
        self.assertEqual(len(window["frames"]), 2)
        self.assertEqual(window["frames"][0]["ego"]["id"], 1)
        self.assertAlmostEqual(window["frames"][1]["ego"]["x"], 20.0)
        self.assertEqual([obs["id"] for obs in window["frames"][1]["obstacles"]], [2])

        # 半径收紧后，目标车道那辆车不再进入局部窗口
        narrow = loader.extract_window(0.9, 0.1, ego_id=1, radius=5.0)
        self.assertEqual(narrow["frames"][0]["obstacles"], [])

    def test_get_track_history_and_missing_id(self):
        loader = HighDDatasetLoader(str(self.root))
        loader.load_scenario(str(self.root / "01_tracks.csv"))

        history = loader.get_track_history(2)
        self.assertEqual(len(history), 3)
        self.assertEqual(list(history["frame"]), [0, 25, 50])
        with self.assertRaises(KeyError):
            loader.get_track_history(999)

    def test_missing_required_column_raises(self):
        self._write_recording(self.root, prefix="02", omit_column="xVelocity")
        loader = HighDDatasetLoader(str(self.root))
        with self.assertRaises(ValueError):
            loader.load_scenario(str(self.root / "02_tracks.csv"))

    def test_load_sample_highd_directory(self):
        sample_path = Path("data/sample_highd")
        if not (sample_path / "01_tracks.csv").exists():
            from scripts.generate_sample_highd import generate_sample_highd
            generate_sample_highd(str(sample_path), "01")

        loader = HighDDatasetLoader(str(sample_path))
        meta = loader.load_scenario(str(sample_path / "01_tracks.csv"))
        self.assertEqual(meta["dataset_name"], "highD")
        self.assertGreaterEqual(meta["total_tracks"], 4)
        obstacles = loader.get_frame_obstacles(0.0)
        self.assertGreaterEqual(len(obstacles), 4)


if __name__ == "__main__":
    unittest.main()
