# 竞赛材料 KPI 表（v0.4-core，15 seeds）

> 范围：**HFSM + Frenet Lattice + LQR + 运动学模型**，三个核心场景。
> 环境：Python 3.9.6，Apple Silicon，单线程 BLAS，`nice -n 19`；每组 15 seeds（42–56），20 s 闭环。
> 任务成功 = 完成任务 + 无碰撞 + 不驶出道路；不再只判断碰撞。

| 场景 | 决策 | n | 任务成功率 (95% CI) | 最小净距 (m) | 最小 TTC (s) | RMS Jerk (m/s³) | 最大横向误差 (m) | 最大横向加速度 (m/s²) | 任务完成时间 (s) | 规划均值 / p95 (ms) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| `highway_merge` | FSM | 15 | 1.000 [0.796, 1.000] | 1.84 ± 0.01 | 20.42 ± 0.11 | 2.007 ± 0.020 | 0.060 ± 0.000 | 2.500 ± 0.000 | 11.71 ± 0.03 | 3.08 / 4.31 |
| `cut_in` | FSM | 15 | 1.000 [0.796, 1.000] | 5.50 ± 0.15 | 1.75 ± 0.04 | 1.832 ± 0.043 | 0.074 ± 0.004 | 2.510 ± 0.000 | — | 2.66 / 6.75 |
| `emergency_avoidance` | FSM | 15 | 1.000 [0.796, 1.000] | 14.10 ± 0.15 | 2.37 ± 0.00 | 1.803 ± 0.003 | 0.000 ± 0.000 | 0.000 ± 0.000 | — | 2.08 / 2.58 |

> 说明：
> - 15 seeds 是轻量基准；论文/答辩目标为每条件 30–50 seeds；
> - `task_completion_time_s=999` 表示非离散任务（cut_in/emergency 用终态判据），不是 999 秒完成；
> - 所有核心场景 `collision_frames=0`、`road_departure=0`；
> - `planner_invalid_count` 均值：highway_merge 0.1、cut_in 11.1、emergency_avoidance 30.3；`safety_brake_count` 均为 0（invalid 主要来自远端碰撞/加速度越界，未触发急刹）；
> - 旧版 `kpi_table.md` 与旧图已归档到 `results/archive/2026-10-20_pre_scope/`，不得再引用；
> - 图表待用当前 HEAD 重新生成（当前环境未安装 matplotlib）。
