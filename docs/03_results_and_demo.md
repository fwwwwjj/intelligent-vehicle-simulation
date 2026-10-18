# 第三部分：结果演示分析与说明

## 1. 当前可复现实验场景

当前代码内置两个可执行场景，均由 `src/environment/scenario_loader.py` 加载：

| 场景 ID | 场景名称 | 道路与初始条件 | 验证目标 | 状态 |
| :--- | :--- | :--- | :--- | :--- |
| `highway_merge` | 高速公路匝道汇入 | 2 条主线车道（y=0 / 3.75 m）+ 1 条右侧匝道（y=-3.75 m）；主车 15 m/s；主线前车 20 m/s、后车 22 m/s（IDM） | 汇入间隙判断、跟驰/汇入状态切换、横向轨迹跟踪 | 已实现 |
| `emergency_avoidance` | 前方静止车辆紧急制动 | 2 车道直线；主车 20 m/s；本车道 90 m 处静止车辆；相邻车道 25 m/s 来车 | 跟驰减速、停车保持、最小安全距离 | 已实现 |
| SC-02 | 城市无保护左转 | 双向四车道十字路口 | 对向冲突流博弈与时空避撞 | 规划中 |
| SC-03 | 真实 cut-in | 相邻车道车辆近距离切入 | 极限减速度与紧急避让 | 规划中 |

> 早期文档中的 SC-02 / SC-03 与“三场景对比”属于规划内容，当前尚未提供对应场景文件与数据，不应作为已完成实验结果使用。

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

实验设置：`seed=42`，仿真步长 0.05 s，时长 20 s，共 400 个控制周期；由 `python3 scripts/run_benchmark.py --scenarios highway_merge emergency_avoidance --decisions fsm game --seeds 42 --duration 20` 生成，原始数据在 `results/benchmark.json` / `results/benchmark.csv`。

| 场景 | 决策 | 成功率 | 最小 OBB 净距 | 最小 TTC | RMS Jerk | 平均横向误差 | 平均规划时延 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `highway_merge` | FSM | 1.00 | 1.85 m | 20.42 s | 2.007 m/s³ | 0.003 m | 8.17 ms |
| `highway_merge` | Game | 1.00 | 1.85 m | 20.42 s | 2.007 m/s³ | 0.003 m | 8.24 ms |
| `emergency_avoidance` | FSM | 1.00 | 13.99 m | 2.37 s | 1.804 m/s³ | 0.000 m | 8.72 ms |
| `emergency_avoidance` | Game | 1.00 | 15.33 m | 2.37 s | 1.750 m/s³ | 0.000 m | 8.59 ms |

**结果解读：**

1. 两个场景均无碰撞，最小 OBB 净距为正；`emergency_avoidance` 中主车在静止车辆前停住并保持，说明“决策目标速度 → 规划减速剖面 → 控制制动”链路已经打通。
2. `highway_merge` 中 FSM 与 Game 都等到主线后车通过后再汇入，速度剖面接近，因此指标相同。这说明当前单场景还不足以拉开两种决策器的差异，不能据此宣称“博弈决策显著优于 FSM”。
3. `emergency_avoidance` 中 Game 的停车点比 FSM 远约 1.3 m，RMS Jerk 低约 3%；差异存在但样本只有 1 个 seed，不能作为统计结论。
4. `highway_merge` 的最小 TTC 为 20.42 s，表示全程没有形成明显接近趋势；这不是“TTC 越大越好”的绝对指标，需要结合场景理解。

**与早期文档的差异：**

早期 README/docs/03 中的 99.2%、82.5%、4.6 m、0.86 m/s³、12.5 ms 等数字**没有对应的代码路径或实验日志**，无法由当前仓库复现。本版已将结果表替换为上述可复现实验，并在 README 中保留“当前实现边界”。

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

- 当前只有 1 个 seed、2 个场景；不足以支撑“汇入成功率 99%”这类强结论。下一步应扩展到 30+ seeds、3–5 个场景并给出置信区间。
- `highway_merge` 的 FSM/Game 分流不明显，需要增加“密集主线车流”“后车逼近”“相邻车道间隙不足”等更对抗的参数组合。
- 尚未接入 highD / INTERACTION 真实轨迹；当前结果不能称为“真实数据驱动标定”。
- 规划平均时延约 8–9 ms，但单步最大时延仍可能超过 20 ms，硬实时结论需要更多测量与优化。
- EM/QP、MPC、MOBIL、Pacejka 主回路尚未实现，详见 README “当前实现边界”。
