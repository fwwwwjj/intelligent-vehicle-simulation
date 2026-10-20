# 仿真基准评测产物与元数据说明 (Benchmark Results & Provenance)

本目录记录中国仿真创新应用大赛闭环仿真评测的核心基准产物。所有数据均由仓库内确定性命令闭环仿真生成，具备完整元数据与同一随机种子可复现性。

---

## 1. 产物清单

- **`benchmark.json`**: 包含环境元数据（Git commit、场景配置 SHA256、Python 版本、Wilson 95% 置信区间定义）、逐运行次原始记录（包含解析后的算法实体名 `planner`、`controller`、`tire_model` 与各维度评测 KPI）以及场景分组聚合统计。
- **`benchmark.csv`**: 扁平化的逐运行次明细表，便于导入数据分析工具或绘制图表。
- **`benchmark_summary.csv`**: 分组聚合摘要表（各场景与决策组合下的均值与样本标准差 $\mathrm{ddof}=1$）。

---

## 2. 评测复现命令

在仓库根目录下运行以下命令即可复现当前评测产物：

```bash
# 执行基准测试：highway_merge 与 cut_in 场景，FSM 与 Game 决策对比，随机种子 42, 43, 44
python3 scripts/run_benchmark.py \
    --scenarios highway_merge cut_in \
    --decisions fsm game \
    --seeds 42 43 44 \
    --duration 20.0 \
    --output-dir results
```

---

## 3. 字段语义与口径说明

| 字段 | 类型 | 说明 |
| :--- | :--- | :--- |
| `scenario` | string | 测试场景标识符（如 `highway_merge`, `cut_in`, `emergency_avoidance`） |
| `decision` | string | 决策算法类型（`fsm`: 分层有限状态机; `game`: 博弈交互决策） |
| `planner` | string | 解析后的轨迹规划算法实体（`lattice` / `em`） |
| `controller` | string | 解析后的车辆控制算法实体（`lqr` / `mpc`） |
| `tire_model` | string | 解析后的轮胎物理模型（`linear` / `pacejka`） |
| `success_rate` | float | 仿真成功率（闭环期间无碰撞且未偏离车道，二值判定） |
| `min_obstacle_distance_m` | float | 全程主车轮廓与所有交通参与者外包络的最小欧氏净距 (m) |
| `min_ttc_s` | float | 全程最小碰撞前向时间 Time-To-Collision (s)；无冲突时记录为 999.00s |
| `rms_jerk_m_s3` | float | 全程加加速度均方根 (m/s³)，衡量舒适性与平顺度 |
| `min_accel_m_s2` | float | 全程最小加速度 (m/s²)，负值代表最大制动减速度 |
| `mean_lateral_error_m` | float | 跟踪目标车道中心线的平均横向绝对误差 (m) |
| `p95_planning_latency_ms` | float | 规划计算时延 95 分位数 (ms)（样本数 $\ge 20$ 时有效） |
| `planner_fallback_count` | int | 规划器回退次数（EM 路径/速度 QP 求解不可行或数值失败时递增） |
| `controller_fallback_count` | int | 控制器回退次数（MPC QP 求解不可行或迭代超限时递增） |

---

## 4. 实验环境与可重现性

- **基准测试平台**: Apple Silicon (macOS), Python 3.9.6, NumPy 1.26.4
- **确定性保证**: 固定随机种子时，环境生成、运动学推演与数值求解全链路无随机扰动，结果比特级复现。
