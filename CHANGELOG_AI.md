# AI 修改记录

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
