# 第三部分：结果演示分析与说明

## 1. 当前可复现实验场景

当前代码内置四个可执行场景，均由 `src/environment/scenario_loader.py` 加载：

| 场景 ID | 场景名称 | 道路与初始条件 | 验证目标 | 状态 |
| :--- | :--- | :--- | :--- | :--- |
| `highway_merge` | 高速公路匝道汇入 | 2 条主线车道（y=0 / 3.75 m）+ 1 条右侧匝道（y=-3.75 m）；主车 15 m/s；主线前车 20 m/s、后车 22 m/s（IDM） | 汇入间隙判断、跟驰/汇入状态切换、横向轨迹跟踪 | 已实现 |
| `emergency_avoidance` | 前方静止车辆紧急制动 | 2 车道直线；主车 20 m/s；本车道 90 m 处静止车辆；相邻车道 25 m/s 来车 | 跟驰减速、停车保持、最小安全距离 | 已实现 |
| `cut_in` | 合成邻道激进变道切入 | 2 车道直线；主车 20 m/s；邻车按配置平滑切入主车道 | 变道减速与避撞响应 | 已实现（脚本化场景） |
| `intersection_unprotected` | 城市无保护左转 | 城市十字路口拓扑；主车 5.56 m/s；对向直行车 -8.33 m/s 穿过冲突区 | 对向冲突流博弈、路权礼让与大曲率过弯 | 已实现 |

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

仓库当前没有随附基准原始输出，因此本节不列未经追溯的实验数字。EM/QP、MPC、Pacejka 已通过单元/集成测试与短时 CLI 冒烟，但尚未纳入基准结果表，不能作为性能结论。复现实验命令如下，执行后会写出逐次 JSON/CSV 与分组汇总 CSV：

```bash
python3 scripts/run_benchmark.py --scenarios highway_merge emergency_avoidance --decisions fsm game --seeds 42 --duration 20
```

JSON 保留运行参数、逐次结果和按场景/决策器分组的均值与离散程度；分组成功率使用逐次二值结果均值并报告 Wilson 95% 区间。单 seed 只适用于冒烟验证，不能作为统计结论。

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

- 目前没有仓库内保存的多 seed 基准结果，不能据此支撑统计性结论；正式报告应保存运行参数和原始结果，并明确样本数与置信区间。
- `highway_merge` 的 FSM/Game 分流不明显，需要增加“密集主线车流”“后车逼近”“相邻车道间隙不足”等更对抗的参数组合。
- 已具备 highD 格式解析、微型演示切片与 IDM 标定工具链，但尚未接入 INTERACTION/NGSIM 与全量生产数据；当前不能称为“全量真实数据驱动标定”。
- 规划平均时延约 8–9 ms，但单步最大时延仍可能超过 20 ms，硬实时结论需要更多测量与优化。
- EM/QP、MPC、Pacejka 已作为可切换算法进入主回路，但目前只完成单元/集成测试与短时冒烟，尚未做多 seed、长时程统计；详见 [docs/06_qp_mpc_em_pacejka.md](06_qp_mpc_em_pacejka.md)。
- MOBIL 换道已进入环境步进（场景 `traffic.enable_mobil` 可控），但尚未做大规模换道安全性统计。
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
