# AI 修改记录

## 2026-10-19 16:52:50

- **修改文件**：`src/environment/obstacle_manager.py`、`tests/test_obstacle_manager.py`、`AI_SYSTEM_SPEC.md`、`README.md`、`docs/02_digital_models.md`、`docs/03_results_and_demo.md`、`docs/05_references.md`、`CHANGELOG_AI.md`
- **修改内容**：障碍物预测开始使用最近一次 IDM 计算的纵向加速度进行恒加速度外推，预测速度降至零后保持位置；新增 IDM 加速和制动停车预测测试。同步环境模型说明及测试数量（28 项改为 30 项）。检查发现仓库未包含文档所称的 benchmark JSON/CSV 原始输出，因此移除无法追溯的 KPI 数值表，保留复现命令。
- **修改原因**：避免规划器把正在加速或制动的 IDM 车辆错误地当作恒速目标；保持规范和测试说明与实现一致；不将缺少仓库证据的实验结果继续作为当前可复现结论。
- **验证方式及结果**：`python3 -m unittest discover tests -v`，30 项通过；`git diff --check` 通过。未运行 benchmark，未执行真实数据标定或外部联调。
- **未完成/受阻**：highD 标定、INTERACTION/NGSIM、MOBIL 状态机、EM/QP、MPC、Pacejka 及 OpenSCENARIO/ROS/CARLA 联仿仍未完成；分别受真实数据/场景、接口与状态机未建、外部依赖或联仿环境缺失限制。本次未宣称完成这些工作。
