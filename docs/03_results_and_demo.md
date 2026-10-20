# 第三部分：结果演示分析与说明

> **范围说明（v0.4-core，2026-10-20）**：本部分只报告当前核心链路 **HFSM + Frenet Lattice + LQR + 运动学模型** 在三个核心场景上的可复现结果。博弈决策、EM/QP、LTV-MPC、Pacejka 轮胎与城市无保护路口属于**未来工作/探索性模块**，不纳入本部分的主结果；详见 [07_scope_and_future_work.md](07_scope_and_future_work.md)。

---

## 1. 当前核心场景

| 场景 ID | 场景名称 | 道路与初始条件 | 验证目标 | 状态 |
| :--- | :--- | :--- | :--- | :--- |
| `highway_merge` | 高速公路匝道汇入 | 2 条主线车道（y=0 / 3.75 m）+ 1 条右侧匝道（y=-3.75 m）；主车 15 m/s；主线前车 20 m/s、后车 22 m/s（IDM） | 汇入间隙判断、跟驰/汇入状态切换、横向轨迹跟踪 | 核心 |
| `cut_in` | 邻道激进切入 | 2 车道直线；主车 20 m/s；邻车按配置平滑切入主车道 | 变道减速与避撞响应 | 核心（脚本化场景） |
| `emergency_avoidance` | 前方静止车辆紧急制动 | 2 车道直线；主车 20 m/s；本车道 90 m 处静止车辆；相邻车道 25 m/s 来车 | 跟驰减速、停车保持、最小安全距离 | 核心 |
| `intersection_unprotected` | 城市无保护左转 | 城市十字路口拓扑；R=20 m 左转 | 对向冲突流博弈、路权礼让 | **实验性，未来工作**：当前会驶出道路，不纳入主结果 |

---

## 2. 指标定义与计算口径

所有指标在 `src/simulator.py` 中逐步计算，由 `src/evaluation/metrics.py` 汇总，不使用硬编码常量。

| 指标 | 计算方式 |
| :--- | :--- |
| **任务成功 (task_success)** | 由场景目标 + 安全 + 车道偏离共同判定，见 §2.1；不再是“未碰撞” |
| **任务完成 (task_completed)** | `highway_merge` 进入主车道；`intersection` 进入北向出口；其他场景由终态判据给出 |
| **完成时间 (task_completion_time_s)** | 首次满足任务目标的时刻；非离散任务记 999（不参与均值解释） |
| **驶出道路 (road_departure)** | 到最近车道中心线距离 > 半车道宽 + 0.5 m |
| **碰撞 (Collision)** | 自车 OBB 与障碍物 OBB 做 SAT 检测，`buffer=0`；任一步相交记 `is_collision=True` |
| **最小 OBB 净距** | 两矩形边界的最短距离；相交为 0 |
| **最小 TTC** | 仅统计自车前方、横向重叠目标：`gap / closing_speed`；无接近目标时为 999 |
| **RMS Jerk** | 由实际加速度差分 `Δa/Δt` 统计，非规划轨迹的理论 jerk |
| **横向/航向误差** | 自车到规划轨迹最近点的横向偏差与航向差；无轨迹时退回目标车道参考线 |
| **规划无效帧** | 规划器返回空轨迹或 `Trajectory.valid=False` 时置 True，累计 `planner_invalid_count/rate` |
| **安全制动** | 仅当 invalid 轨迹在 1 s 内存在 OBB 碰撞（或轨迹为空）时触发最大减速度/零转向，累计 `safety_brake_count/rate` |
| **最大横向加速度** | 由实际 `ay = v·yaw_rate` 统计；用于检查 `max_lateral_accel` 是否被突破 |
| **规划/决策/控制时延** | 三段分别计时，不合并 |
| **行驶距离/时长** | 由逐步位移与时间戳累计 |

### 2.1 任务成功判据

| 场景 | 成功条件 |
| :--- | :--- |
| `highway_merge` | 20 s 内进入 `main_lane_*`；无碰撞；不驶出道路 |
| `cut_in` | 无碰撞；最小净距 ≥ 2 m；不驶出道路；观察窗口 ≥ 5 s |
| `emergency_avoidance` | 无碰撞；最终速度 ≤ 0.5 m/s；最小净距 ≥ 2 m；不驶出道路 |

> 这一判据会直接识别“没做事也算成功”的假结果：例如当前 Game 在 20 s 内未汇入，`task_success=0`；交叉口驶出道路，`task_success=0`。

---

## 3. 当前实测结果（核心 FSM + Lattice + LQR）

### 3.1 复现命令（低功耗）

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1 \
  nice -n 19 python3 -B scripts/run_benchmark.py \
  --scenarios highway_merge cut_in emergency_avoidance \
  --decisions fsm \
  --seeds 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 \
  --duration 20.0
```

### 3.2 结果表

环境：Python 3.9.6，Apple Silicon，单线程 BLAS，`nice -n 19`；每组 15 seeds，20 s 闭环；成功率附 Wilson 95% 置信区间。

| 场景 | 决策 | n | 任务成功率 (95% CI) | 最小净距 (m) | 最小 TTC (s) | RMS Jerk (m/s³) | 最大横向误差 (m) | 最大横向加速度 (m/s²) | 任务完成时间 (s) | 规划均值 / p95 (ms) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| `highway_merge` | FSM | 15 | 1.000 [0.796, 1.000] | 1.84 ± 0.01 | 20.42 ± 0.11 | 2.007 ± 0.020 | 0.060 ± 0.000 | 2.500 ± 0.000 | 11.71 ± 0.03 | 3.08 / 4.31 |
| `cut_in` | FSM | 15 | 1.000 [0.796, 1.000] | 5.50 ± 0.15 | 1.75 ± 0.04 | 1.832 ± 0.043 | 0.074 ± 0.004 | 2.510 ± 0.000 | — | 2.66 / 6.75 |
| `emergency_avoidance` | FSM | 15 | 1.000 [0.796, 1.000] | 14.10 ± 0.15 | 2.37 ± 0.00 | 1.803 ± 0.003 | 0.000 ± 0.000 | 0.000 ± 0.000 | — | 2.08 / 2.58 |

> 说明：
> - 15 seeds 是**轻量基准**，用于确认链路和统计管线；论文目标为每条件 30–50 seeds；
> - `task_completion_time_s=999` 表示该场景没有离散“完成时刻”（cut_in/emergency 用终态判据），不是“999 秒完成”；
> - 所有场景 `road_departure=0`、`collision_frames=0`；
> - 规划时延是单机实测，受硬件/负载影响；论文需报告 p50/p95/p99 与硬件环境。
> 诊断字段：`planner_invalid_count` 均值分别为 highway_merge 0.1、cut_in 11.1、emergency_avoidance 30.3；`safety_brake_count` 均为 0。含义是规划器在部分帧把“远端碰撞/加速度越界”候选标记为 invalid，但没有触发 1 s 内即将碰撞的最小风险制动；这些帧仍由 LQR 跟踪并在下一步重规划。

### 3.3 结果分析

1. **高速汇入**：FSM 在约 11.7 s 完成从匝道到 `main_lane_0` 的汇入；最小净距约 1.84 m，说明汇入时与主线后车保持了接近但可控的间距。RMS jerk 约 2.01 m/s³，主要来自汇入前的速度调整与横向换道，后续可通过更长的换道时域/更平滑的横向多项式进一步优化。
2. **邻道 cut-in**：脚本化切入在 0.5–3 s 内完成；最小净距约 5.53 m，最小 TTC 约 1.76 s，说明减速响应及时；RMS jerk 约 1.82 m/s³，主要来自切入后的制动调整。
3. **紧急制动**：车辆在静止障碍物前停住，最小净距约 14.07 m，最终速度 ≤ 0.5 m/s；RMS jerk 约 1.80 m/s³，最大横向加速度为 0，符合直线紧急制动预期。
4. **任务级判据的意义**：如果只报“无碰撞”，上面三个场景都会是 100%，但无法区分“完成任务”和“停在原地/驶出道路”。加入任务成功后，Game 的未汇入、交叉口的驶出道路都会被判 0，避免假成功。

---

## 4. 演示与复现

1. **2D BEV 实时动画**：Matplotlib 渲染道路中心线、规划轨迹、自车与障碍物 OBB，并显示速度/转角 HUD。未使用 Pygame。
   ```bash
   python3 scripts/run_simulation.py --scenario highway_merge --decision fsm --duration 20
   ```
   无图形环境使用 `--no_vis`。

2. **逐帧数据与 KPI**：
   ```bash
   python3 scripts/run_simulation.py --scenario highway_merge --decision fsm --duration 20 --no_vis --save_results
   ```
   输出 `results/telemetry_highway_merge_fsm_seed42.json` 与 `results/kpi_highway_merge_fsm_seed42.json`。

3. **批量对比**：
   ```bash
   OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1 \
     nice -n 19 python3 -B scripts/run_benchmark.py \
     --scenarios highway_merge cut_in emergency_avoidance \
     --decisions fsm --seeds 42 43 44 --duration 20
   ```

4. **核心图表**：`scripts/export_core_figures.py` 生成 3 张 BEV、1 张三场景遥测图、1 张 15-seed KPI 箱线图。
   ```bash
   python3 scripts/export_core_figures.py --seed 42 --duration 20 \
     --output-dir results/competition_pack/figures
   ```
   输出：`fig1_highway_merge_bev.png`、`fig2_cut_in_bev.png`、`fig3_emergency_avoidance_bev.png`、`fig4_core_telemetry.png`、`fig5_core_kpi_distribution.png`。

5. **单场景遥测曲线**：`scripts/export_results.py` 从真实 telemetry 生成速度、横向误差、jerk 曲线。
   ```bash
   python3 scripts/export_results.py --scenario highway_merge --decision fsm --duration 20
   ```
   > 当前环境若未安装 matplotlib，该命令会失败；先执行 `pip install -r requirements.txt`。

---

## 5. 当前局限

- **样本量**：15 seeds 只用于轻量验证；论文需要 30–50 seeds、统计检验与效应量。
- **场景多样性**：三个核心场景都是结构化道路/合成交通；highD 格式解析 + 最小可信版脚本已就绪，真实 recording 结果待补。
- **车辆模型**：核心链路使用运动学自行车模型，不能体现轮胎侧滑/极限工况；2-DOF + Pacejka 属于未来工作。
- **控制**：LQR 的横向加速度已限制在 2.5 m/s² 附近；但高速换道的舒适性仍有优化空间。
- **规划**：Lattice 只做单时域采样；EM/QP 属于未来工作。
- **决策**：核心只使用 FSM；Game 当前未完成汇入，不能作为主结果。
- **时延**：单机实测，未做硬实时保证；论文需报告硬件和 p95/p99。
- **YAML 路径**：当前基准环境未安装 PyYAML，`load_named_scenario` 走内置 dict；安装 `requirements.txt` 后应重跑一次，确认 YAML 与内置场景一致。

---

## 6. 未来工作（不纳入本部分主结果）

1. **博弈决策**：修复 Game 汇入死锁；形式化 players/actions/payoffs；30+ seeds 与 FSM 对比。
2. **EM/QP**：与 Lattice 做公平对比；统计规划时延与回退率。
3. **LTV-MPC**：修复 emergency stop；与 LQR 对比。
4. **Pacejka**：轮胎试验标定；与线性轮胎对比。
5. **城市无保护路口**：局部 Frenet 车道查询、冲突点/路权、曲率限速。
6. **真实数据**：highD 子集标定/回放；INTERACTION/NGSIM 接入。
7. **联合仿真**：OpenSCENARIO/ROS/CARLA。
8. **大规模统计**：30–50 seeds、参数扰动、SOTIF 边界场景。

详见 [07_scope_and_future_work.md](07_scope_and_future_work.md) 与 [06_qp_mpc_em_pacejka.md](06_qp_mpc_em_pacejka.md)。

---

## 7. 真实 highD 最小可信版（数据可信度附录）

当前 3.2 的主结果是合成场景闭环。**highD 格式解析 + 最小可信版脚本已就绪，真实 recording 结果待补**。仓库另外提供 **highD 最小可信版**：

- `scripts/calibrate_highd.py`：从 highD 同车道跟驰片段提取样本，标定 IDM 的 `v0, s0, T, a_max, b_desired`，输出训练/验证 RMSE；
- `scripts/replay_highd.py`：把一段真实 highD 轨迹按时间回放成 BEV 截图/GIF，检查坐标、车道映射与加载链路；
- `src/environment/highd_tools.py`：样本提取、标定封装、直线路网构建。

复现：

```bash
python3 scripts/calibrate_highd.py --data-root data/raw/highD --recording 01
python3 scripts/replay_highd.py --data-root data/raw/highD --recording 01 \
  --start-time 0 --duration 10 --step 0.5 --max-frames 12 --gif
```

> 这是**最小可信版**：只做同车道跟驰标定 + 开环回放，不做闭环交互，也不做全量生产标定。`data/sample_highd` 是合成样例，输出会标记 `synthetic_sample`，不能写成真实数据结论。详见 [10_highd_calibration_replay.md](10_highd_calibration_replay.md)。

在论文中，这一节应放在“数据来源/实验设置”，并如实写出 recording ID、时间窗、样本数、许可；不能把开环回放写成“真实车流闭环验证”。
