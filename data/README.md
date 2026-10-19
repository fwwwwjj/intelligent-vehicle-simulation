# 真实数据集目录

本目录用于放置**本地**真实轨迹数据。仓库不附带任何 highD / INTERACTION /
NGSIM 原始数据，`data/raw/` 已被 `.gitignore` 忽略，请勿把数据文件提交到仓库。

## highD

从 [highD 官网](https://www.highd-dataset.com/) 下载后，按 recording 放置：

```text
data/raw/highD/
├── 01_recordingMeta.csv
├── 01_tracks.csv
├── 01_tracksMeta.csv
├── 02_recordingMeta.csv
├── 02_tracks.csv
└── ...
```

读取示例：

```python
from src.environment.dataset_loader import HighDDatasetLoader

loader = HighDDatasetLoader("data/raw/highD")
meta = loader.load_scenario("data/raw/highD/01_tracks.csv")
obstacles = loader.get_frame_obstacles(1.0)  # 相对时间 1.0 s 的最近帧
window = loader.extract_window(0.0, 10.0, ego_id=123, radius=80.0)
```

说明：

- 解析器保留 highD 原始 `x/y` 坐标，不做重定心、旋转或 y 轴翻转；
- highD 的 `height` 按车身纵向长度解释，`width` 为横向宽度；
- 数据只用于本地分析/标定，参赛材料中引用时请同时标注数据集版本、片段 ID、
  时间区间与许可协议；
- 当前尚未用全量真实 highD 做生产环境标定，文档中不能虚报“已完成实车数据标定”。

## 微型演示数据集生成与参数辨识

若本地暂无几十 GB 完整数据集，可一键生成仅几十 KB 的轻量微型演示数据集：

```bash
python3 scripts/generate_sample_highd.py --output_dir data/sample_highd
```

读取演示数据并执行微观交通流参数标定：

```python
from src.environment.dataset_loader import HighDDatasetLoader
from src.environment.calibration import IDMCalibrator, extract_following_pairs_from_arrays

# 1. 加载微型切片
loader = HighDDatasetLoader("data/sample_highd")
meta = loader.load_scenario("data/sample_highd/01_tracks.csv")
obstacles = loader.get_frame_obstacles(0.0)

# 2. 系统辨识与参数标定工具链
calibrator = IDMCalibrator()
# 从提取的跟驰样本中优化 IDM 参数 (v0, s0, T, a_max, b_desired)
```
