"""highD 最小可信版工具链测试（使用仓库内合成微型样例）。"""

import unittest
from pathlib import Path

from src.environment.dataset_loader import HighDDatasetLoader
from src.environment.highd_tools import (
    build_road_network_from_highd,
    calibrate_highd_samples,
    extract_highd_following_samples,
    highd_data_kind,
)


SAMPLE = Path(__file__).resolve().parents[1] / "data" / "sample_highd" / "01_tracks.csv"


@unittest.skipUnless(SAMPLE.exists(), "sample highD files are required")
class TestHighDMinimalTools(unittest.TestCase):

    def setUp(self):
        self.loader = HighDDatasetLoader(str(SAMPLE.parent))
        self.meta = self.loader.load_scenario(str(SAMPLE))

    def test_sample_is_labelled_synthetic(self):
        self.assertEqual(highd_data_kind(self.meta["tracks_file"]), "synthetic_sample")

    def test_extract_following_samples(self):
        samples = extract_highd_following_samples(self.loader, max_samples=100)
        self.assertGreater(len(samples), 0)
        self.assertLessEqual(len(samples), 100)
        for item in samples[:5]:
            self.assertGreater(item["gap"], 0.0)
            self.assertGreater(item["current_speed"], 0.0)

    def test_calibration_returns_params_and_rmse(self):
        samples = extract_highd_following_samples(self.loader, max_samples=200)
        result = calibrate_highd_samples(samples, train_ratio=0.7)
        self.assertGreater(result["sample_count"], 0)
        self.assertIn("v0", result["calibrated_params"])
        self.assertGreaterEqual(result["calibrated_rmse_train"], 0.0)

    def test_build_road_network_from_markings(self):
        road, centers, mapping = build_road_network_from_highd(self.loader)
        self.assertGreaterEqual(len(centers), 2)
        self.assertGreaterEqual(len(road.lanes), 2)
        self.assertTrue(mapping)
        obstacles = self.loader.get_frame_obstacles(0.0)
        self.assertGreater(len(obstacles), 0)


if __name__ == "__main__":
    unittest.main()
