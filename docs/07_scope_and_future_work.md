# 第七部分：当前核心范围与未来工作

> 版本：v0.4-core（2026-10-20）
> 目的：在时间有限的前提下，把参赛/论文的**主张边界**收敛到一条已经跑通、可验证、可复现的主链；复杂算法和完成度较低的模块统一放入“未来工作”，不再作为当前结果的一部分。

---

## 1. 为什么收敛范围

当前仓库包含很多模块，但“模块存在”不等于“实验有效”。审计发现：

- `game_theoretic.py` 在 `highway_merge` 中 20 s 内没有完成汇入，不能支撑“Game 优于 FSM”的结论；
- `em_planner.py`、`mpc_controller.py`、`tire.py` 的 Pacejka 分支只有单元/短时冒烟，没有多 seed 统计；
- `intersection_unprotected` 目前会驶出道路，不能作为“无保护左转已实现”的证据；
- highD 只有格式解析器和合成微型样例，没有真实数据标定。

因此，v0.4-core 的原则是：

> **核心链路必须“任务完成、指标真实、结果可复现”；其余模块保留代码和接口，但在论文/答辩中只作为未来工作。**

---

## 2. 当前核心交付（纳入论文/答辩主结果）

| 层 | 核心实现 | 代码入口 | 验收方式 |
| :--- | :--- | :--- | :--- |
| 环境 | 数字路网、IDM 跟驰、脚本化 cut-in | `scenario_loader.py`、`obstacle_manager.py` | 3 个场景可加载、seed 可复现 |
| 决策 | 分层有限状态机（HFSM） | `rule_based_fsm.py` | 巡航/跟驰/汇入/紧急停车 |
| 规划 | Frenet Lattice + 五次多项式 + OBB 碰撞校验 | `frenet_lattice.py`、`cost_evaluator.py` | 弯道参考线、碰撞轨迹标记 invalid |
| 控制 | LQR 横向跟踪 + 曲率前馈 + 纵向 PID | `lqr_controller.py`、`pid_controller.py` | 速度跟踪、横向加速度受限 |
| 车辆 | 运动学自行车模型 | `kinematics.py` | 闭环积分稳定 |
| 安全监督 | 记录 invalid 轨迹；1 s 内即将碰撞时最小风险制动 | `simulator.py` | 记录 `planner_invalid` / `safety_brake` |
| 评测 | 碰撞、OBB 净距、TTC、Jerk、横向/航向误差、时延、任务成功 | `simulator.py`、`metrics.py` | 逐帧记录 + 汇总 JSON/CSV |
| 场景 | `highway_merge`、`cut_in`、`emergency_avoidance` | `configs/scenarios/` | 10 seeds 任务成功率 100% |

### 2.1 核心任务成功判据

| 场景 | 成功条件 |
| :--- | :--- |
| `highway_merge` | 20 s 内进入 `main_lane_*`；无碰撞；不驶出道路 |
| `cut_in` | 无碰撞；最小净距 ≥ 2 m；不驶出道路；观察窗口 ≥ 5 s |
| `emergency_avoidance` | 无碰撞；最终速度 ≤ 0.5 m/s；最小净距 ≥ 2 m；不驶出道路 |

> 关键变化：`success` 不再等于“没撞车”，而是“完成任务 + 安全 + 不驶出道路”。这会让 Game/交叉口等未完成任务的运行自动判 0，避免“没做事也算成功”。

---

## 3. 未来工作（代码保留，不纳入主结果）

| 模块 | 当前状态 | 为什么不纳入主结果 | 未来工作 |
| :--- | :--- | :--- | :--- |
| 博弈决策 `game_theoretic.py` | 有收益评估/信念更新原型；高速汇入会长时间礼让，20 s 内不汇入 | 任务不完成，FSM/Game 不可比 | 加 liveness/最大等待时间；形式化 players/actions/payoffs；30+ seeds 重评 |
| EM/QP 规划 `em_planner.py` | 路径 QP + 速度 QP 原型；短时冒烟通过 | 未做多 seed、长时程统计；规划时延 p95 20–35 ms | 与 Lattice 做公平对比；约束/回退统计；实时性优化 |
| LTV-MPC `mpc_controller.py` | 线性化误差模型 + QP；`lattice+MPC` 在 emergency 会撞车 | 安全闭环未通过 | 修 emergency stop；加终端约束/更长时域；与 LQR 对比 |
| Pacejka `tire.py` | 稳态魔术公式 + 载荷转移 | 未做试验标定；与线性轮胎差异未统计 | 轮胎试验数据标定；与 2-DOF 主回路联调 |
| 交叉口 `intersection_unprotected` | 拓扑/场景存在；车辆会冲过路口并驶出道路 | 任务不完成；决策层用中位数 y，看不到对向冲突车 | 局部 Frenet 车道查询；冲突点/路权；曲率限速 |
| MOBIL 换道 | 已接入 `ObstacleManager.step()`，高速汇入启用 | 未做换道安全性统计 | 多场景换道安全/舒适统计 |
| highD 真实标定 | CSV 解析器 + 合成微型样例 | 无真实数据、无标定结果 | 下载 highD 子集；`IDMCalibrator` 标定；回放 |
| OpenSCENARIO/ROS/CARLA | 占位/路线图 | 无中间件代码 | 后续工程化 |

> 这些模块的代码不删除：保留接口、单元测试和切换开关，方便未来继续做；但论文/答辩中必须写成“未来工作/探索性原型”，不能写成“已完成并验证”。

---

## 4. 论文中的写法

### 4.1 可以写的

- 核心链路：IDM 环境 → HFSM → Frenet Lattice → LQR → 运动学模型 → 任务级 KPI；
- 任务级 success 的定义与实现；
- 3 个核心场景、10 seeds 的闭环结果（论文目标 30 seeds）；
- 与“只报碰撞”的评测相比，任务级判据能识别“未汇入/驶出道路”的假成功；
- 复杂算法作为 future work 的路线图。

### 4.2 不能写的

- “Game 比 FSM 舒适性提升 50.4%”；
- “EM/QP、MPC、Pacejka 已完成多 seed 验证”；
- “城市无保护左转已实现”；
- “基于 highD 真实数据标定”；
- “统计显著优于基线”（当前只有 10 seeds，且无基线对比）。

---

## 5. 实验规模与低功耗约定

当前基准（v0.4-core）：

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1 \
  nice -n 19 python3 -B scripts/run_benchmark.py \
  --scenarios highway_merge cut_in emergency_avoidance \
  --decisions fsm \
  --seeds 42 43 44 45 46 47 48 49 50 51 \
  --duration 20.0
```

- 10 seeds 是**轻量基准**，用于确认链路和统计管线；不是论文最终样本量；
- 论文目标：每条件 30–50 seeds；建议在空闲时段分批跑，每批不超过 10–15 seeds，保持单线程 BLAS + `nice`；
- 运行后检查 `results/benchmark.json.metadata` 的 commit/场景 SHA256，确保数字可追溯。

---

## 6. 代码与文档的对应关系

| 文档 | 定位 |
| :--- | :--- |
| `README.md` | 项目总览：核心链路 + 未来工作 |
| `docs/01` | 问题来源与数据来源 |
| `docs/02` | 数字模型：标注哪些是核心、哪些是未来工作 |
| `docs/03` | 核心场景/指标/10-seed 结果 |
| `docs/04` | 应用说明：核心 MIL/SIL + 未来 HIL/ROS/CARLA |
| `docs/05` | 代码体系与参考文献 |
| `docs/06` | 未来工作技术储备：QP/MPC/EM/Pacejka |
| `docs/07` | 本文件：范围与未来工作清单 |
| `docs/08` | 论文草稿：核心链路、任务指标、10-seed 结果与局限 |
| `AI_SYSTEM_SPEC.md` | SSOT：接口/量纲/验收口径 |
