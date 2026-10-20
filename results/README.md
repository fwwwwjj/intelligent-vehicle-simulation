# 仿真基准评测产物与元数据说明 (Benchmark Results & Provenance)

本目录记录 v0.4-core 的核心基准产物：**HFSM + Frenet Lattice + LQR + 运动学模型**，3 个核心场景 × 10 seeds × 20 s 闭环。所有数据由仓库内命令生成，带有 commit、场景 SHA256、Python 版本与逐次运行记录。

> 复杂算法（Game/EM/MPC/Pacejka/交叉口）不在本目录的主结果中；旧版产物已归档到 `results/archive/2026-10-20_pre_scope/`，不得再作为当前结论引用。范围说明见 [../docs/07_scope_and_future_work.md](../docs/07_scope_and_future_work.md)。

---

## 1. 产物清单

- **`benchmark.json`**：环境元数据（Git commit、`git_dirty`、场景配置 SHA256、Python 版本、Wilson 95% 区间定义）、逐次运行原始记录、分组聚合统计。
- **`benchmark.csv`**：逐运行次明细表，便于导入数据分析工具。
- **`benchmark_summary.csv`**：分组聚合摘要（均值与样本标准差 `ddof=1`）。
- **`archive/2026-10-20_pre_scope/`**：范围收敛前的旧 benchmark 与旧 competition_pack，仅作历史留档。

---

## 2. 评测复现命令（低功耗）

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1 \
  nice -n 19 python3 -B scripts/run_benchmark.py \
  --scenarios highway_merge cut_in emergency_avoidance \
  --decisions fsm \
  --seeds 42 43 44 45 46 47 48 49 50 51 \
  --duration 20.0
```

- 10 seeds 是**轻量基准**，用于确认链路与统计管线；论文目标为每条件 30–50 seeds；
- 建议分批运行，保持单线程 BLAS + `nice -n 19`，避免机器持续高负载；
- 运行后检查 `benchmark.json.metadata.git_commit`、`git_dirty` 与场景 SHA256，确保数字可追溯；
- 本次 `results/` 在 commit `9e227db` 的干净工作区生成：`git_dirty=false`；`results/` 产物本身已从 dirty 判定中排除。

---

## 3. 字段语义与口径

| 字段 | 类型 | 说明 |
| :--- | :--- | :--- |
| `scenario` | string | `highway_merge` / `cut_in` / `emergency_avoidance` |
| `decision` | string | 核心基准固定 `fsm`；`game` 为探索性，不纳入主结果 |
| `planner` | string | 核心为 `lattice`；`em` 为未来工作 |
| `controller` | string | 核心为 `lqr`；`mpc` 为未来工作 |
| `tire_model` | string | 核心为 `linear`；`pacejka` 为未来工作 |
| `success_rate` | float | **任务级成功率**：完成任务 + 无碰撞 + 不驶出道路；0/1 |
| `task_success` | float | 与 `success_rate` 同口径的数值字段，便于分组统计 |
| `task_completed` | float | 是否完成场景目标（`highway_merge` 进入主车道；交叉口进入北向出口） |
| `task_completion_time_s` | float | 首次完成时刻；非离散任务记 999（不解释为 999 s 完成） |
| `road_departure` | float | 是否驶出道路（到最近车道中心线 > 半车道宽 + 0.5 m） |
| `collision_count` | float | 逐帧碰撞障碍物数量之和 |
| `collision_frames` | float | 发生碰撞的帧数 |
| `min_obstacle_distance_m` | float | 全程最小 OBB 净距 (m) |
| `min_ttc_s` | float | 全程最小 TTC (s)；无冲突时 999 |
| `rms_jerk_m_s3` | float | 全程加加速度 RMS (m/s³) |
| `max_lateral_accel_m_s2` | float | 全程最大横向加速度 (m/s²)，用于检查 2.5 m/s² 限制 |
| `mean_lateral_error_m` | float | 平均横向误差 (m) |
| `max_lateral_error_m` | float | 最大横向误差 (m) |
| `p95_planning_latency_ms` | float | 规划时延 p95 (ms)，样本 ≥ 20 时有效 |
| `planner_invalid_count` | float | 规划无效帧数（空轨迹或 `Trajectory.valid=False`）；核心链路用于诊断 |
| `safety_brake_count` | float | 1 s 内即将碰撞时触发最小风险制动的帧数 |
| `planner_fallback_count` | int | EM 规划回退次数；Lattice 核心恒为 0 |
| `controller_fallback_count` | int | MPC 控制回退次数；LQR 核心恒为 0 |

### 3.1 任务成功判据

| 场景 | 成功条件 |
| :--- | :--- |
| `highway_merge` | 20 s 内进入 `main_lane_*`；无碰撞；不驶出道路 |
| `cut_in` | 无碰撞；最小净距 ≥ 2 m；不驶出道路；观察窗口 ≥ 5 s |
| `emergency_avoidance` | 无碰撞；最终速度 ≤ 0.5 m/s；最小净距 ≥ 2 m；不驶出道路 |

---

## 4. 当前结果摘要（10 seeds）

| 场景 | n | 任务成功率 (95% CI) | 最小净距 (m) | 最小 TTC (s) | RMS Jerk (m/s³) | 最大横向误差 (m) | 规划均值 / p95 (ms) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| `highway_merge` | 10 | 1.000 [0.722, 1.000] | 1.84 ± 0.00 | 20.42 ± 0.11 | 2.012 ± 0.022 | 0.060 ± 0.000 | 3.06 / 4.41 |
| `cut_in` | 10 | 1.000 [0.722, 1.000] | 5.53 ± 0.15 | 1.76 ± 0.04 | 1.824 ± 0.041 | 0.073 ± 0.004 | 2.56 / 6.47 |
| `emergency_avoidance` | 10 | 1.000 [0.722, 1.000] | 14.07 ± 0.14 | 2.37 ± 0.00 | 1.803 ± 0.003 | 0.000 ± 0.000 | 2.04 / 2.61 |

> 环境：Python 3.9.6，Apple Silicon，单线程 BLAS，`nice -n 19`。以上数字由当前工作区生成；论文/答辩引用时请同时给出 commit、seed 列表与命令。
>
> 诊断字段：`planner_invalid_count` 均值分别为 highway_merge 0.1、cut_in 11.1、emergency_avoidance 30.3；`safety_brake_count` 均为 0。invalid 主要来自远端碰撞/加速度越界，未触发急刹；下一步重规划继续。
> 本次基准 `git_commit=9e227db`、`git_dirty=false`；若重新生成，`results/` 产物本身不会把 dirty 置 True。

---

## 5. 实验环境与可重现性

- **基准平台**：Apple Silicon (macOS), Python 3.9.6；
- **确定性**：固定 seed 时，环境生成、运动学推演与数值求解无随机扰动；同一 seed 可复现；
- **待补**：CI 自动化、30–50 seeds、真实 highD 标定、图表重新生成；
- **环境提示**：当前基准环境无 PyYAML，场景走内置 dict；安装 `requirements.txt` 后应重跑，确认 YAML 与内置场景一致；
- **旧产物**：`archive/2026-10-20_pre_scope/` 中的 `kpi_table.md` 与图片来自旧代码，不得再引用。
