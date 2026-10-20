# 第十部分：highD 最小可信版（标定 + 开环回放）

> 版本：v0.4-core（2026-10-20）
> 定位：用真实 highD 轨迹做两件事：**IDM 跟驰参数标定**与**开环轨迹回放**。
> 不做：全量生产标定、闭环交互回放、换道/MOBIL 标定、城市路口。

---

## 1. 这一步解决什么问题

论文/答辩中，评委常问：

1. 你的 IDM 参数从哪来？
2. 你的场景除了合成数据，有没有真实轨迹校验？

最小可信版的回答是：

- 从 highD 同车道跟驰片段中提取 `(current_speed, lead_speed, gap, observed_accel)` 样本；
- 用 `IDMCalibrator` 拟合 `v0, s0, T, a_max, b_desired`；
- 报告默认参数 vs 标定参数的训练/验证 RMSE；
- 把一段真实轨迹按 20 Hz/5 Hz 回放成 BEV 截图，作为“真实车流回放”证据。

> 它不等于“全量实车标定”，论文中必须如实写“基于 highD 的某个 recording / 时间窗”。

---

## 2. 数据准备

highD 需按官方许可申请；仓库不附带原始数据。放置方式：

```text
data/raw/highD/
├── 01_recordingMeta.csv
├── 01_tracks.csv
└── 01_tracksMeta.csv
```

`data/raw/` 已被 `.gitignore` 忽略，不要把原始数据提交到 GitHub。

仓库内 `data/sample_highd/` 是**合成微型样例**，只用于验证脚本链路，不能作为真实数据结论。

---

## 3. 标定

```bash
python3 scripts/calibrate_highd.py \
  --data-root data/raw/highD \
  --recording 01 \
  --max-samples 3000 \
  --train-ratio 0.7
```

输出：`results/highd_calibration_01.json`

关键字段：

| 字段 | 含义 |
| :--- | :--- |
| `data_kind` | `highd_recording` 或 `synthetic_sample` |
| `sample_count/train_count/validation_count` | 样本与训练/验证划分 |
| `calibrated_params` | `v0, s0, T, a_max, b_desired` |
| `initial_rmse_train` / `calibrated_rmse_train` | 训练集默认/标定 RMSE |
| `default_rmse_validation` / `calibrated_rmse_validation` | 验证集默认/标定 RMSE |

**验收标准**：

- 验证集 RMSE 低于默认参数；
- 参数在合理范围（例如 `v0` 20–40 m/s、`T` 0.8–2.5 s、`a_max` 0.5–3 m/s²）；
- 报告中写明 recording ID、时间窗、样本数、是否合成样例。

---

## 4. 开环回放

```bash
python3 scripts/replay_highd.py \
  --data-root data/raw/highD \
  --recording 01 \
  --start-time 0 \
  --duration 10 \
  --step 0.5 \
  --max-frames 12 \
  --gif
```

输出：

- `results/highd_replay/frame_*.png`；
- `results/highd_replay/highd_replay_01.json`；
- 可选 `highd_replay_01.gif`（需要 Pillow）。

回放是**开环**的：车辆位置直接来自 highD，不跑决策/规划/控制，也不做闭环交互。它证明的是“真实轨迹能进入仿真路网并按时间播放”，不是“真实车流下的闭环控制”。

---

## 5. 合成样例自检（无真实数据时）

```bash
python3 scripts/calibrate_highd.py \
  --data-root data/sample_highd \
  --tracks data/sample_highd/01_tracks.csv \
  --max-samples 500 \
  --output results/highd_calibration_sample.json

python3 scripts/replay_highd.py \
  --data-root data/sample_highd \
  --tracks data/sample_highd/01_tracks.csv \
  --duration 2 --step 0.5 --max-frames 3 \
  --output-dir results/highd_replay_sample
```

自检输出会标记 `data_kind=synthetic_sample`，并打印 WARN。**不能**把该结果写成真实 highD 标定。

---

## 6. 代码位置

| 文件 | 作用 |
| :--- | :--- |
| `src/environment/dataset_loader.py` | highD CSV 解析、按帧查询、时间窗切片 |
| `src/environment/highd_tools.py` | 跟驰样本提取、IDM 标定封装、直线路网构建 |
| `src/environment/calibration.py` | `IDMCalibrator`（最小二乘/无 scipy 回退） |
| `scripts/calibrate_highd.py` | 标定 CLI |
| `scripts/replay_highd.py` | 开环回放 CLI |
| `tests/test_highd_tools.py` | 合成样例上的工具链测试 |

---

## 7. 写进论文的建议

在“数据来源”一节增加：

> 使用 highD recording `XX`（时间窗 `t0–t1`，采样率 25 Hz，许可：highD Dataset License）提取同车道跟驰片段，共 `N` 个样本；按 70/30 划分训练/验证，IDM 参数由 `IDMCalibrator` 拟合。标定后验证集加速度 RMSE 从 `A` 降至 `B`。另取 `t2–t3` 片段做开环回放，用于检查坐标/车道映射与轨迹加载。

**不要写**：

- “基于全量 highD 完成生产级标定”；
- “真实数据闭环验证”（除非真的做了闭环回放）；
- “城市无保护路口真实数据”（highD 是高速公路）。

---

## 8. 局限与后续

- 只做同车道跟驰标定；换道/MOBIL/博弈参数未标定；
- 只做开环回放；未做“自车受控 + 真实车流”闭环；
- 只用 1 个 recording 时，结论只对该片段成立；
- 真实数据受许可限制，不能提交原始 CSV。

后续可扩展：多 recording 标定、按场景切片、闭环 cut-in/following 回放、与合成场景结果对比。
