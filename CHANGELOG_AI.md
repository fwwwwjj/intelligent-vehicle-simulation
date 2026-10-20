# AI 修改记录

## 2026-10-20 (文档修正：README 目录/CI 徽标/运动学保真度/cut-in TTC)

- **修改文件**：`README.md`、`docs/02_digital_models.md`、`docs/03_results_and_demo.md`、`docs/08_paper_draft.md`、`docs/09_slides_and_video_outline.md`、`AI_SYSTEM_SPEC.md`、`results/competition_pack/kpi_table.md`、`CHANGELOG_AI.md`。
- **修改内容**：
  1. README 文件树补齐 `docs/08`、`docs/09`、`docs/10`，并同步 scripts 目录说明；
  2. 移除静态 “tests 96 local” 徽标，保留 GitHub Actions CI 徽标，并注明“本地 96 项通过，CI 以 Actions 为准”；
  3. 在 README/docs/02/docs/03/docs/08/AI_SPEC 中增加**运动学模型保真度声明**：不含轮胎侧滑、载荷转移、悬架/制动动力学与传感器/执行器硬件延迟，不是高保真车辆动力学/HIL；
  4. 在 docs/03 §3.4 与 docs/08 §6.1 增加 **cut-in 最小 TTC ≈1.75 s 的专门解释**：属于刻意激进切入、紧安全余量但无碰撞；最小净距 5.47 m、最大制动 5.94 m/s²；不能写成“安全余量充足”；
  5. README 核心结果表增加 TTC 备注；答辩 Q&A 增加对应问题。
- **验证方式及结果**：`git diff --check` 通过；本次仅文档与 KPI 表修改，代码未变，96 项测试状态不变。

## 2026-10-20 (v0.4-core：范围收敛到 FSM + Lattice + LQR，复杂算法转为未来工作)

- **修改文件**：`src/simulator.py`、`src/evaluation/metrics.py`、`src/evaluation/benchmark.py`、`src/environment/road_network.py`、`src/environment/scenario_loader.py`、`src/planning/frenet_lattice.py`、`src/runner.py`、`src/control/lqr_controller.py`、`src/control/mpc_controller.py`、`scripts/run_benchmark.py`、`scripts/export_core_figures.py`（新增）、`scripts/export_competition_assets.py`、`tests/test_task_success.py`（新增）、`tests/test_closed_loop.py`、`.github/workflows/ci.yml`（恢复）、`LICENSE`（新增）、`README.md`、`docs/02`、`docs/03`、`docs/04`、`docs/05`、`docs/06`、`docs/07`（新增）、`docs/08_paper_draft.md`（新增）、`AI_SYSTEM_SPEC.md`、`results/*`、`CHANGELOG_AI.md`。
- **修改内容**：
  1. **范围收敛**：当前核心交付定为 **HFSM + Frenet Lattice + LQR + 运动学模型 + 任务级 KPI + 三个核心场景**（highway_merge / cut_in / emergency_avoidance）。Game、EM/QP、LTV-MPC、Pacejka、城市无保护路口、MOBIL 统计、highD 真实标定、OpenSCENARIO/ROS/CARLA 统一列为未来工作；代码与接口保留，但不纳入主结果。
  2. **任务级 success**：`success_rate` 不再只判断碰撞，而是由场景目标 + 无碰撞 + 不驶出道路共同决定；新增 `task_success`、`task_completed`、`task_completion_time_s`、`road_departure` 字段。这样 Game 未汇入、交叉口驶出道路都会自动判 0，避免“没做事也算成功”。
  3. **横向约束修复**：去掉 LQR/MPC 中 `max(lat_steer_limit, 0.05)` 的转角下限，改为 1e-3 数值保护；22 m/s 下横向加速度不再被抬到约 9 m/s²，而是限制在 2.5 m/s² 附近。
  4. **安全监督**：规划轨迹 invalid 时不再一律急刹；只有轨迹为空或前 1 s 内存在 OBB 碰撞时才进入最小风险制动，记录 `planner_invalid` / `safety_brake`。避免把“远端碰撞/加速度越界”误判成急刹。
  5. **基准与分批 seeds**：3 场景 × FSM，基础 10 seeds（42–51）+ 分批 20 seeds（52–71，4 批 × 5），共 **30 seeds / 90 次运行**，20 s 闭环；新增 `scripts/run_benchmark_batches.py`（批间 sleep，避免持续发热），结果合并写回 `results/benchmark.json/csv/summary.csv`。30 seeds 仍为轻量基准，论文目标 30–50 seeds。
  6. **测试**：新增 `tests/test_task_success.py`（任务成功、未汇入判失败、交叉口未完成判失败、cut-in/紧急制动成功、LQR 横向加速度上限、invalid 轨迹安全兜底）、YAML/内置场景一致性测试、`tests/test_highd_tools.py`（highD 样本/标定/路网）与 `tests/test_experimental_guard.py`（Game/EM/MPC/Pacejka/交叉口隔离）；全量 **96 项测试通过**（约 10–20 s，单线程 BLAS + nice）。
  7. **实验性模块隔离**：`build_engine` 会把 Game/EM/MPC/Pacejka/交叉口标记为 `experimental`；CLI 默认拒绝运行，必须显式加 `--allow-experimental`；benchmark 元数据写入 `experimental/experimental_reasons`，避免与核心结果混表。
  8. **highD 最小可信版**：新增 `src/environment/highd_tools.py`、`scripts/calibrate_highd.py`、`scripts/replay_highd.py` 与 `docs/10_highd_calibration_replay.md`；支持真实 highD 同车道跟驰标定 + 开环回放；合成样例会标记 `synthetic_sample`。
  9. **文档与交付**：README/docs/03/docs/04 删除“Game 50.4% 提升”“交叉口已实现”“MPC 已可替代 LQR”等表述；新增 `docs/07_scope_and_future_work.md`、`docs/08_paper_draft.md`、`docs/10_highd_calibration_replay.md`；docs/06 标注为未来工作技术储备；恢复 `.github/workflows/ci.yml`；新增 `LICENSE`；新增 `scripts/export_core_figures.py` 并生成 5 张核心图。
- **验证方式及结果**：
  - `OMP_NUM_THREADS=1 ... nice -n 19 python3 -B -m unittest discover tests` → 96 项通过（约 10–20 s）；
  - `run_benchmark.py ... --seeds 42..51` + `run_benchmark_batches.py --seeds 52..71` → 90 次运行完成，三场景任务成功率 1.000 [0.886, 1.000]；
  - `calibrate_highd.py` / `replay_highd.py` 在 `data/sample_highd` 上自检通过（输出标记 synthetic_sample）；
  - `export_core_figures.py` → 5 张核心图写入 `results/competition_pack/figures/`；
  - 任务级回归：Game 20 s 未汇入 → `success=0`；交叉口驶出道路 → `success=0`；核心 FSM 三场景 → `success=1`。
- **未完成/边界**：Game 汇入死锁、EM/QP 多 seed 统计、MPC 紧急制动、Pacejka 标定、交叉口、真实 highD 数据运行（脚本已就绪）、30–50 seeds、PPT/视频仍未完成；这些均列入未来工作。

## 2026-10-20 (历史记录，已被 v0.4-core 取代：完善 FSM 与 Game 决策差异度、基准元数据追溯与工程治理)

> 说明：本节的 Game 50.4%/34.9% 对比结论已作废——v0.4-core 审计发现 Game 在 20 s 内未完成汇入，任务不等价，不能作为主结果。

- **修改文件**：`src/decision/game_theoretic.py`、`src/environment/obstacle_manager.py`、`src/environment/scenario_loader.py`、`configs/scenarios/highway_merge.yaml`、`src/evaluation/benchmark.py`、`.gitignore`、`results/README.md`、`requirements.txt`、`.github/workflows/ci.yml`、`docs/03_results_and_demo.md`、`CHANGELOG_AI.md`。
- **修改内容**：
  1. **决策对抗性与博弈区分度**：
     - 在 `highway_merge` 场景中增加左侧邻道阻挡车（`obs_block_left`），构建主线紧凑车流与后车加速逼近的对抗工况；
     - 修复 `ObstacleManager._evaluate_mobil_lane_change` 缺失侧向重叠安全几何过滤与道路边界约束的缺陷，避免背景车盲目变道与驶入匝道；
     - 重构 `GameTheoreticDecisionMaker`：将目标车道后随车（`target_follower`）纳入二人非零和收益矩阵，精准评估切入冲突风险；在选择礼让时主动调节车速创造汇入间隙，待车流通过后平滑切入；
     - 实测基准对比验证：在 `highway_merge` 下 Game 较 FSM 的 RMS jerk 降低 50.4%（0.999 vs 2.013 m/s³）；在 `cut_in` 下降低 34.9%（1.180 vs 1.812 m/s³），两类决策在多 seed 下表现出可解释的显著行为分流。
  2. **基准结果与元数据可追溯治理**：
     - `BenchmarkSuite` 中将 `planner/controller/tire_model` 记录为运行时解析后的算法实体名（`lattice/em`, `lqr/mpc`, `linear/pacejka`）并保留入参；
     - 更新 `.gitignore` 允许跟踪核心基准结果与元数据（`results/benchmark.json`, `results/benchmark.csv`, `results/benchmark_summary.csv` 以及 `data/sample_highd/*.csv`），同时撰写 `results/README.md` 规范字段口径与复现命令；
  3. **依赖锁定与自动化 CI 构建**：
     - 锁定 `requirements.txt` 关键依赖至已在 Python 3.9 环境充分验证的版本上限；
     - 新增 GitHub Actions 工作流 `.github/workflows/ci.yml`，在多 Python 版本下自动执行单元测试与轻量冒烟基准。
- **验证方式及结果**：`python3 -B -m unittest discover tests` 65 项全绿；`git diff --check` 通过；`scripts/run_benchmark.py --scenarios highway_merge cut_in --decisions fsm game --seeds 42 43 44` 运行完毕并更新基准文件。

## 2026-10-20 (优化规划计算几何与碰撞距离粗筛，时延 p95 降至 10ms 内)

- **修改文件**：`src/common/geometry.py`、`src/planning/cost_evaluator.py`、`src/common/frenet_frame.py`、`docs/03_results_and_demo.md`、`CHANGELOG_AI.md`。
- **修改内容**：
  1. 在 `cost_evaluator.py` 中引入外接圆与 AABB 几何双重粗筛，中心距与轴向距离超出 $R_1 + R_2 + 3.0\text{m}$ 时直接跳过昂贵的 OBB SAT 与点线段计算；
  2. 优化 `CollisionChecker.check_obb_collision`，以标量轴向投影计算代替逐点生成顶点小数组，单次 SAT 调用耗时降低约 20 倍；
  3. 重构 `CollisionChecker.min_distance_between_boxes`，消除冗余顶点比对并用纯标量数学计算点线段距离平方，单对包围盒最短净距求解耗时降低 18 倍；
  4. 在 `FrenetConverter.ref_point_at_s` 中引入参考线弧长数组对象缓存，消除高频规划中逐点构造大数组与重复排序开销；
  5. 实测 20s `highway_merge`（400步）规划平均时延从 37.8ms 降至 5.7ms，p95 时延从 88.1ms 压减至 9.7ms（达成 $\le 15\text{ms}$ 验收目标），最大时延从 89.2ms 降至 11.8ms，且三个标准场景 KPI 与碰撞率维持零差异。
- **验证方式及结果**：`python3 -B -m unittest discover tests` 65 项全绿；同一 seed 42 下 20s 运行，规划 p95 为 9.66ms。

## 2026-10-20 (修复回退诊断指标与 P95 时延口径)

- **修改文件**：`src/simulator.py`、`src/evaluation/metrics.py`、`tests/test_metrics.py`、`docs/06_qp_mpc_em_pacejka.md`、`AI_SYSTEM_SPEC.md`、`CHANGELOG_AI.md`。
- **修改内容**：
  1. 在 `SimulationEngine.step()` 中从规划器（`EMPlanner.last_path_result` / `last_speed_result`）与控制器（`MPCController.last_solve_info`）读取真实求解与回退状态，向 `SimulationMetricsRecord` 填写真实的 `planner_fallback` 与 `controller_fallback`；
  2. 明确回退口径：EM 路径/速度 QP 未成功或空轨迹记为规划回退，Lattice 稳定记为 False；MPC QP 未成功或空轨迹记为控制回退，LQR 稳定记为 False；
  3. `PerformanceEvaluator` 中将 p95 时延统计限定在样本数 $\ge 20$ 时输出，避免微型样本产生失真分位数；
  4. 新增 4 项单元测试验证正常场景 0 回退、EM 失败回退计数、MPC 失败回退计数及 p95 样本阈值条件。
- **验证方式及结果**：`python3 -B -m unittest discover tests` 65 项全绿通过；CLI 执行 `scripts/run_simulation.py --controller mpc --duration 5 --no_vis` 正常场景回退率 0，QP 迭代设 0 时回退率 100%。

## 2026-10-19 (QP/MPC/EM/Pacejka 主回路 + 远端 MOBIL/标定整合)

- **修改文件**：`src/common/qp.py`、`src/control/mpc_controller.py`、`src/planning/em_planner.py`、`src/vehicle_model/tire.py`、`src/vehicle_model/dynamics.py`、`src/runner.py`、`src/simulator.py`、`src/environment/scenario_loader.py`、`configs/default_config.yaml`、`scripts/run_simulation.py`、`scripts/run_benchmark.py`、`docs/06_qp_mpc_em_pacejka.md`、新增 5 个测试文件及相关文档。
- **修改内容**：
  1. 新增纯 NumPy ADMM QP 后端，支持盒约束、线性约束、过松弛、自适应 rho、热启动与残差诊断；
  2. 实现 LTV-MPC 轨迹跟踪控制器，包含执行机构/速率硬约束、横向软约束、QP 失败回退与 `last_solve_info` 诊断；
  3. 实现 EM/QP 路径-速度解耦规划器：路径 QP + S-T 图速度 QP + 终端速度保护 + 安全回退；
  4. 将 Pacejka 稳态魔术公式、载荷转移与简化摩擦椭圆接入 2-DOF 动力学主回路，并保留线性轮胎分支；
  5. 打通 `planner=lattice|em`、`controller=lqr|mpc`、`tire_model=linear|pacejka` 的配置/装配/CLI 切换；
  6. 与远端 MOBIL 换道、IDM 加速度外推、`cut_in` 场景、highD 微型切片、IDM 标定工具链及 benchmark 汇总功能合并，解决 `scenario_loader`、`benchmark`、文档与 CLI 冲突；
  7. 改进无 scipy 环境下的 IDM 标定回退搜索（多轮坐标轮换 + 区间收缩），使 `tests/test_calibration.py` 在零外部优化器环境下通过。
- **验证方式及结果**：低功耗模式（单线程 BLAS + `nice`）运行 `python3 -B -m unittest discover tests`，**61 项全部通过（约 1.7 s）**；`git diff --check` 通过；`cut_in` 与 `emergency_avoidance` 各 10 步短时闭环冒烟通过。未运行长时间多 seed benchmark，避免机器持续发热。
- **未完成/边界**：INTERACTION/NGSIM 与全量 highD 生产标定、城市无保护路口、OpenSCENARIO/ROS/CARLA 联仿仍未接入；EM/MPC/Pacejka 仅完成框架、单元/集成测试与短时冒烟，尚未做多 seed 统计。本次为本地提交后与 `origin/main` 的 rebase 整合，推送需远端认证。

## 2026-10-19

- **修改文件**：`src/environment/obstacle_manager.py`、`src/environment/scenario_loader.py`、`src/evaluation/benchmark.py`、`scripts/run_benchmark.py`、`configs/scenarios/highway_merge.yaml`、`configs/scenarios/cut_in.yaml`、`tests/test_closed_loop.py`、`tests/test_obstacle_manager.py`、`tests/test_metrics.py`、`AI_SYSTEM_SPEC.md`、`README.md`、`docs/02_digital_models.md`、`docs/03_results_and_demo.md`、`docs/05_references.md`、`CHANGELOG_AI.md`
- **修改内容**：
  1. 增加场景级 `traffic.enable_mobil` / `lane_width` 配置；`highway_merge` 显式启用 MOBIL，其他场景维持按配置关闭；内置场景与 YAML 行为保持同步。
  2. 增加障碍物脚本化平滑换道调度；`cut_in` 车辆按配置的目标车道、开始时刻和持续时间切入，行为由闭环测试验证。
  3. 基准 JSON 增加执行参数、Python 版本、Git 提交号、场景配置 SHA-256 指纹、逐次原始结果和分组统计；新增原始运行 CSV 与分组汇总 CSV。批量成功率由每次运行 0/1 结果求均值，并提供 Wilson 95% 区间；连续指标报告均值及样本标准差。
  4. 更新 SSOT、README 和技术支撑文档，删除过时/无依据的实现状态和实验数字。
- **验证方式及结果**：GPT-6 Luna 运行 `python3 -m unittest discover tests -v`，39 项全部通过（3.134 秒）；`git diff --check` 通过。执行三场景（`highway_merge`、`emergency_avoidance`、`cut_in`）× FSM/Game × seeds 42/43/44、每次 3.5 s 的轻量 benchmark，18 次运行完成，生成 `results/benchmark.json`、`results/benchmark.csv`、`results/benchmark_summary.csv`。该短时合成场景运行仅用于验证场景和统计管线，不作为性能结论。
- **未完成/边界**：没有接入真实 highD 原始数据；短时、每组 3 个 seed 的 smoke benchmark 不足以支持泛化或竞赛统计结论；MPC、EM/QP 与 Pacejka 主回路仍未实现。本次未提交或推送。

## 2026-10-19 21:26:00

- **修改文件**：`test`（删除）、`src/environment/obstacle_manager.py`、`src/environment/scenario_loader.py`、`src/environment/calibration.py`（新增）、`src/environment/__init__.py`、`scripts/generate_sample_highd.py`（新增）、`configs/scenarios/cut_in.yaml`（新增）、`tests/test_calibration.py`（新增）、`tests/test_obstacle_manager.py`、`tests/test_closed_loop.py`、`tests/test_dataset_loader.py`、`data/README.md`、`docs/01_problem_and_data.md`、`docs/02_digital_models.md`、`AI_SYSTEM_SPEC.md`、`README.md`、`CHANGELOG_AI.md`
- **修改内容**：
  1. 删除根目录下无内容的残留文件 `test`；
  2. 解决真实数据缺乏的工程方案：编写 `scripts/generate_sample_highd.py` 本地一键生成合规的微型 highD 演示切片（~20KB），使 `HighDDatasetLoader` 具备开箱即用的解析与切片查询演示能力；
  3. 实现微观交通流参数辨识工具链 `src/environment/calibration.py`（`IDMCalibrator`），支持根据实车跟驰时空轨迹以最小二乘非线性优化反演拟合 IDM 关键参数（v0, s0, T, a_max, b_desired）；
  4. 完善环境交通流动力学：将 MOBIL 换道模型接入 `ObstacleManager.step()`（由 `enable_mobil=True` 开启，支持三次多项式横向平滑过渡）；
  5. 补充激进变道切入测试场景 `configs/scenarios/cut_in.yaml`，并在 `scenario_loader.py` 中增加零依赖内置回退与测试验证；
  6. 同步更新单元测试套件（测试项由 30 项增至 35 项，全部通过）及大赛技术支撑材料与开发规范。
- **修改原因**：响应用户指令清理无效文件，解决用户目前无真实数据的痛点，完善障碍物 MOBIL 自主换道与典型 ODD 长尾场景覆盖，打通参数标定闭环。
- **验证方式及结果**：`python3 -m unittest discover tests -v`，35 项全部通过（耗时 2.52 秒）；`git diff --check` 无空白冲突；保持低功耗轻量级运行。
- **未完成/受阻**：INTERACTION/NGSIM、EM/QP、MPC、Pacejka 魔术公式及 OpenSCENARIO/ROS/CARLA 联仿仍保持接口预留状态；本次未宣称完成全量真实数据的生产级标定。

## 2026-10-19 16:52:50

- **修改文件**：`src/environment/obstacle_manager.py`、`tests/test_obstacle_manager.py`、`AI_SYSTEM_SPEC.md`、`README.md`、`docs/02_digital_models.md`、`docs/03_results_and_demo.md`、`docs/05_references.md`、`CHANGELOG_AI.md`
- **修改内容**：障碍物预测开始使用最近一次 IDM 计算的纵向加速度进行恒加速度外推，预测速度降至零后保持位置；新增 IDM 加速和制动停车预测测试。同步环境模型说明及测试数量（28 项改为 30 项）。检查发现仓库未包含文档所称的 benchmark JSON/CSV 原始输出，因此移除无法追溯的 KPI 数值表，保留复现命令。
- **修改原因**：避免规划器把正在加速或制动的 IDM 车辆错误地当作恒速目标；保持规范和测试说明与实现一致；不将缺少仓库证据的实验结果继续作为当前可复现结论。
- **验证方式及结果**：`python3 -m unittest discover tests -v`，30 项通过；`git diff --check` 通过。未运行 benchmark，未执行真实数据标定或外部联调。
- **未完成/受阻**：highD 标定、INTERACTION/NGSIM、MOBIL 状态机、EM/QP、MPC、Pacejka 及 OpenSCENARIO/ROS/CARLA 联仿仍未完成；分别受真实数据/场景、接口与状态机未建、外部依赖或联仿环境缺失限制。本次未宣称完成这些工作。
