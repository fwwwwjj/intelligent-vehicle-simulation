# AI 修改记录

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
