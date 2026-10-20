# 第一部分：问题来源与数据来源说明

## 1. 问题来源与工程背景

### 1.1 自动驾驶复杂场景下的决策与规划瓶颈
随着高级别智能网联汽车（SAE L3/L4 级）向规模化量产与商业化落地演进，智能车辆面临的核心挑战已经由结构化道路简单跟驰转变为**高动态、强博弈、长尾高危（Corner Cases）复杂交通环境下的行为决策与实时动态避障**。
- **高速公路强交互汇入 (Highway On-Ramp Merging)**：主线高流速与匝道有限加速区交织，要求主车在极短时空窗内与主线多车进行间隙博弈与速度协同；
- **城市无保护复杂路口 (Unprotected Urban Intersections)**：人机混行、遮挡盲区大、轨迹交织，要求决策具备前瞻意图推理与风险评估能力；
- **长尾极端切入与紧急避障 (Cut-In & Emergency Avoidance)**：传统基于简单硬编码规则的状态机难以应对复杂拓扑与动态多目标约束，易导致“规划求解失效”、“急刹频发”或“安全碰撞事故”。

### 1.2 物理实车道路测试的高成本与高风险痛点
- 实车路测面临安全风险不可控、危险极端工况难构造、偶发失效难复现等突出难题；
- 《中国制造2025》及《智能网联汽车技术路线图 2.0》明确要求：**建立基于数字孪生与高保真虚拟仿真的全栈验证测试平台**。数字仿真建模已成为智能网联汽车研发验证环节不可或缺的核心工具。

---

## 2. 数据来源与当前接入状态

> **必须如实说明**：当前仓库**没有随附** highD / INTERACTION / NGSIM 的原始数据文件。**highD 格式解析 + 最小可信版脚本已就绪，真实 recording 结果待补**。`src/environment/dataset_loader.py` 的 **highD 分支已实现 highD 格式 CSV 读取**（`*_tracks.csv` / `*_recordingMeta.csv` / `*_tracksMeta.csv`，支持按时间戳取帧和窗口切片）；INTERACTION / NGSIM 仍是接口骨架。以下数据集是后续迭代计划接入的目标，不应在参赛材料中表述为“已用于标定”。

### 2.1 计划接入的真实道路数据集

| 数据集 | 来源与特征 | 计划用途 | 当前状态 |
| :--- | :--- | :--- | :--- |
| **highD** | 德国高速公路无人机俯拍，定位误差小于 10 cm，包含自然超车、汇入与换道 | 标定 IDM 跟驰参数；选取汇入片段作为开环回放 | ⚠️ highD 格式解析 + 最小可信版脚本已就绪，真实 recording 结果待补 |
| **INTERACTION** | 伯克利等联合发布，含环岛、无保护路口、匝道并线等强交互场景 | 博弈决策的对抗场景与评测基准 | 接口预留，未接入数据 |
| **NGSIM** | 美国 FHWA 的 US-101 / I-80 高密度轨迹 | 大流量交通流重放与极限合流测试 | 接口预留，未接入数据 |

highD 数据需自行从官方渠道下载并放到 `data/raw/highD/`（该目录已被 `.gitignore` 忽略）。典型用法：

```python
from src.environment.dataset_loader import HighDDatasetLoader

loader = HighDDatasetLoader("data/raw/highD")
meta = loader.load_scenario("data/raw/highD/01_tracks.csv")
obstacles = loader.get_frame_obstacles(1.0)          # 相对时间 1.0 s 的最近帧
window = loader.extract_window(0.0, 10.0, ego_id=123, radius=80.0)
```

> 解析器保留 highD 原始 x/y 坐标，不做重定心/旋转；`height` 视为车身长度。把 highD 轨迹接入 `RoadNetwork` 前仍需做坐标对齐与场景切片选择。

最小可信版 CLI（真实数据放到 `data/raw/highD/` 后）：

```bash
python3 scripts/calibrate_highd.py --data-root data/raw/highD --recording 01
python3 scripts/replay_highd.py --data-root data/raw/highD --recording 01 \
  --start-time 0 --duration 10 --step 0.5 --max-frames 12 --gif
```

`data/sample_highd` 是合成样例，输出会标记 `synthetic_sample`，只能用于自检。详见 [10_highd_calibration_replay.md](10_highd_calibration_replay.md)。

**当前实验数据来源**：
1. `configs/scenarios/*.yaml` 中的典型场景（高速汇入 `highway_merge`、前方静止紧急避撞 `emergency_avoidance`、近距离激进切入 `cut_in`），用于覆盖 ODD 核心功能安全与控制边界验证；
2. `scripts/generate_sample_highd.py` 提供的微型 highD 演示切片（`data/sample_highd/`），用于端到端验证真实数据集加载器 `HighDDatasetLoader` 的解析、时间窗截取与时空查询能力；
3. `src/environment/calibration.py` 提供的 IDM 跟驰参数系统辨识与标定流水线，支持在有实车跟驰时空轨迹时自动拟合微观交通流参数。

### 2.2 场景描述与数字格式

| 格式/接口 | 当前实现 | 说明 |
| :--- | :--- | :--- |
| 项目 YAML 场景 | ✅ 已实现 | `src/environment/scenario_loader.py` 读取 map / ego / obstacles / IDM 参数 |
| JSON 场景 | ✅ 已实现 | 同一 loader 支持 `.json`，便于程序化生成 |
| ASAM OpenSCENARIO 2.0 / 1.x | ⏳ 接口预留 | `dataset_loader.py` 仅有 `parse_xosc` 占位，未解析 XML |
| OpenDRIVE / Lanelet2 高精地图 | ⏳ 规划中 | 当前 `RoadNetwork` 由直线车道中心点构造，尚未读取标准地图 |

### 2.3 数据可溯性要求

后续接入真实数据时，必须同时提交：
1. 数据集名称、版本、下载地址与许可协议；
2. 使用的片段 ID、时间区间与车辆 ID；
3. 预处理脚本与参数标定记录（写入 `configs/` 或 `scripts/`）；
4. 由脚本自动生成的指标 JSON/CSV，而不是手工填写的表格。

在完成上述工作前，结果表只能标注为“合成场景闭环验证”，不能标注为“真实数据驱动实验”。
