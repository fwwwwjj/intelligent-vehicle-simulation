# 第三部分：结果演示分析与说明

## 1. 当前可复现实验场景

当前代码内置三个可执行场景，均由 `src/environment/scenario_loader.py` 加载：

| 场景 ID | 场景名称 | 道路与初始条件 | 验证目标 | 状态 |
| :--- | :--- | :--- | :--- | :--- |
| `highway_merge` | 高速公路匝道汇入 | 2 条主线车道（y=0 / 3.75 m）+ 1 条右侧匝道（y=-3.75 m）；主车 15 m/s；主线前车 20 m/s、后车 22 m/s（IDM） | 汇入间隙判断、跟驰/汇入状态切换、横向轨迹跟踪 | 已实现 |
| `emergency_avoidance` | 前方静止车辆紧急制动 | 2 车道直线；主车 20 m/s；本车道 90 m 处静止车辆；相邻车道 25 m/s 来车 | 跟驰减速、停车保持、最小安全距离 | 已实现 |
| SC-02 | 城市无保护左转 | 双向四车道十字路口 | 对向冲突流博弈与时空避撞 | 规划中 |
| `cut_in` | 合成邻道切入 | 邻车按配置开始时刻与持续时间平滑切入主车道 | 避撞与跟驰响应 | 已实现（脚本化场景） |

> `cut_in` 是按配置生成的合成切入场景，不是数据集回放。城市无保护左转仍为规划内容。

---

## 2. 指标定义与计算口径

所有指标在 `src/simulator.py` 中逐步计算，由 `src/evaluation/metrics.py` 汇总，不再使用硬编码常量。

| 指标 | 计算方式 |
| :--- | :--- |
| **碰撞 (Collision)** | 自车 OBB 与障碍物 OBB 做 SAT 检测，`buffer=0`；任一步相交记 `is_collision=True` |
| **最小 OBB 净距** | 两矩形边界的最短距离；相交为 0 |
| **最小 TTC** | 仅统计自车前方、横向重叠目标：`gap / closing_speed`；无接近目标时为 999 |
| **RMS Jerk** | 由实际加速度差分 `Δa/Δt` 统计，非规划轨迹的理论 jerk |
| **横向/航向误差** | 自车到规划轨迹最近点的横向偏差与航向差；无轨迹时退回目标车道参考线 |
| **规划/决策/控制时延** | 三段分别计时，不再把决策+规划合并成 `planning_latency` |
| **行驶距离/时长** | 由逐步位移与时间戳累计 |

---

## 3. 当前实测结果

基准测试套件支持多场景、多决策算法与多随机种子自动化回归。执行评测会写出逐次记录 `results/benchmark.json`、原始明细 `results/benchmark.csv` 以及分组统计表 `results/benchmark_summary.csv`：

```bash
python3 scripts/run_benchmark.py --scenarios highway_merge cut_in --decisions fsm game --seeds 42 43 44 --duration 20.0
```

实测基准对比表（Python 3.9.6, Apple Silicon, 种子 42/43/44, 各工况运行 20s 闭环，分组成功率附 Wilson 95% 置信区间）：

| 场景 | 决策算法 | 样本数 $n$ | 成功率 (95% CI) | 最小净距 (m) | 最小 TTC (s) | RMS Jerk (m/s³) | 横向误差 (m) | 规划耗时 (ms) |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| `highway_merge` | **FSM** | 3 | 1.000 [0.439, 1.000] | 1.84 ± 0.01 | 20.46 ± 0.10 | 2.013 ± 0.019 | 0.003 ± 0.000 | 2.50 ± 0.01 |
| `highway_merge` | **Game** | 3 | 1.000 [0.439, 1.000] | 1.85 ± 0.00 | 999.00 ± 0.00 | **0.999 ± 0.004** | 0.002 ± 0.000 | 6.19 ± 0.09 |
| `cut_in` | **FSM** | 3 | 1.000 [0.439, 1.000] | 5.54 ± 0.16 | 1.78 ± 0.04 | 1.812 ± 0.053 | 0.002 ± 0.000 | 4.53 ± 0.05 |
| `cut_in` | **Game** | 3 | 1.000 [0.439, 1.000] | 3.64 ± 0.18 | 1.77 ± 0.04 | **1.180 ± 0.003** | 0.002 ± 0.001 | 4.72 ± 0.06 |

### 决策对比分析

1. **高速汇入对抗场景 (`highway_merge`)**：
   - **FSM**：采用确定性静态间隙门限，在匝道始终保持目标车速冲刺，与主线逼近后车形成横向僵持，汇入时伴随较强烈的速度/加速度调整，RMS jerk 达 $2.013\ \mathrm{m/s^3}$；
   - **Game**：构建主车与后随逼近车的二人非零和收益矩阵，识别出强冲突风险后主动触发战术礼让（微调车速让出路权），待后车平稳通过后平滑切入主车道，RMS jerk 降至 $0.999\ \mathrm{m/s^3}$（**舒适性平顺度提升 50.4%**）。
2. **邻道激进切入场景 (`cut_in`)**：
   - **FSM**：在 TTC 突降时触发强阶跃减速，产生瞬时 jerk 脉冲（RMS jerk $1.812\ \mathrm{m/s^3}$）；
   - **Game**：通过效用平滑收益权衡，采取平缓跟驰让行（RMS jerk $1.180\ \mathrm{m/s^3}$，**降低 34.9%**）。

---

## 4. 演示与复现

1. **2D BEV 实时动画**：Matplotlib 渲染道路中心线、规划轨迹、自车与障碍物 OBB，并显示速度/转角 HUD。未使用 Pygame。
   ```bash
   python3 scripts/run_simulation.py --scenario highway_merge --decision fsm --duration 20
   ```
   无图形环境使用 `--no_vis`。

2. **逐帧数据与 KPI**：
   ```bash
   python3 scripts/run_simulation.py --scenario highway_merge --decision game --duration 20 --no_vis --save_results
   ```
   输出 `results/telemetry_highway_merge_game_seed42.json` 与 `results/kpi_*.json`。

3. **批量对比**：
   ```bash
   python3 scripts/run_benchmark.py --scenarios highway_merge emergency_avoidance \
     --decisions fsm game --seeds 42 --duration 20
   ```

4. **科研图表**：`scripts/export_results.py` 从真实 telemetry 生成速度、横向误差、jerk 曲线，不再使用合成指数曲线。
   ```bash
   python3 scripts/export_results.py --scenario highway_merge --decision fsm --duration 20
   ```

---

## 5. 当前局限与下一步实验

- 批量基准评测已支持多场景（`highway_merge`, `cut_in`）与多 seed 自动化统计，并在 `results/` 目录下固化可复现元数据；更多工况与大规模场景集（如高难度激进换道、路口交互）有待进一步扩展。
- 已具备 highD 格式解析、微型演示切片与 IDM 标定工具链，但尚未接入 INTERACTION/NGSIM 与全量生产数据；当前不能称为“全量真实数据驱动标定”。
- 在典型工况下（Python 3.9，FSM+Lattice，20s highway_merge），规划平均时延处于 2–6 ms 区间，p95 时延 ≤ 10 ms（粗筛与标量几何优化后；优化前未粗筛时汇入段峰值曾达 87 ms），单步最大时延控制在 12 ms 内；若需硬实时保证仍需更多平台化实测与编译级加速。
- EM/QP、MPC、Pacejka 已作为可切换算法进入主回路，但目前只完成单元/集成测试与短时冒烟，尚未做多 seed、长时程统计；详见 [docs/06_qp_mpc_em_pacejka.md](06_qp_mpc_em_pacejka.md)。
- MOBIL 换道已进入环境步进（场景 `traffic.enable_mobil` 可控，且已施加侧向重叠安全几何过滤与道路边界约束），但尚未做大规模换道安全性统计。
- INTERACTION/NGSIM 与全量 highD 生产标定仍未接入，当前不能称为“真实数据驱动生产标定”。


---

## 6. 新算法框架验证（单 seed / 短时）

以下命令只用于确认“接口可调用、闭环可运行、约束可检查”，不作为性能排名：

```bash
# EM/QP + MPC + Pacejka 短时闭环
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 \
  nice -n 19 python3 -B scripts/run_simulation.py \
  --scenario emergency_avoidance --decision fsm \
  --planner em --controller mpc --tire pacejka \
  --duration 5 --no_vis

# 定向单元/集成测试（低功耗）
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 \
  nice -n 19 python3 -B -m unittest \
  tests.test_qp_solver tests.test_mpc_controller \
  tests.test_em_planner tests.test_pacejka_tire \
  tests.test_optimized_closed_loop -v
```

验证口径：

- QP 返回 `success` 与 `constraint_violation`，调用方不会把未收敛解静默当作硬约束；
- MPC 的 `last_solve_info` 记录迭代数、残差与回退状态；
- EM 的 `last_path_result` / `last_speed_result` 记录 QP 成功/回退状态；
- Pacejka 分支可通过 `engine.dynamic_model.tire_model == "pacejka"` 检查；
- 所有短时运行仍由逐帧 KPI 记录，不以硬编码数字代替。
