# 第四部分：应用说明与工程落地价值

## 1. 当前可交付能力

| 能力 | 当前状态 | 说明 |
| :--- | :--- | :--- |
| 纯 Python 闭环仿真（MIL/SIL） | ✅ 可用 | `SimulationEngine` 串联环境、感知、决策、规划、控制、动力学与 KPI |
| 场景化回归 | ✅ 可用 | YAML/JSON 场景加载，支持 seed 复现与批量 `run_benchmark.py` |
| 2D BEV 演示 | ✅ 可用 | Matplotlib 实时渲染；无图形环境可 `--no_vis` |
| 数据驱动标定 | ⏳ 未完成 | highD / INTERACTION / NGSIM 仅接口预留，未附数据 |
| HIL / CAN / DDS | ⏳ 未实现 | 无总线/实时系统接口代码 |
| ROS / CARLA / SUMO 联合仿真 | ⏳ 未实现 | 无中间件或 Co-Simulation 代码 |
| OpenSCENARIO 解析 | ⏳ 未实现 | 仅 `parse_xosc` 占位 |

> 参赛材料中应把上表如实写入“已完成/在研”，避免把路线图写成既有能力。

---

## 2. 工业与教学应用价值

### 2.1 自动驾驶算法虚拟闭环测试 (MIL / SIL)
- 本系统提供纯 Python 的轻量闭环，可在单台 PC 上重复执行同一 seed 的场景，用于算法回归与失效复现；
- 当前版本已能检测碰撞、最小净距、TTC、横向误差与规划时延，适合做“决策/规划/控制”参数对比；
- 由于尚未接入 HIL 与真实控制器，不能宣称已完成硬件在环验证。

### 2.2 预期功能安全 (SOTIF) 的边界场景生成
- 目标：对背景车速度、跟驰参数、初始间距做扰动，批量生成高危边界场景；
- 当前：`run_benchmark.py` 已支持多 seed 批量运行，但场景参数扰动与万级场景生成尚未实现；
- 后续应结合 `configs/scenarios/` 参数化与批量采样，输出统计分布，而不是单个成功案例。

### 2.3 虚拟仿真实验教学
- 学生可直接观察跟驰、汇入、紧急制动三种行为，以及 LQR/PID 参数变化对跟踪误差与舒适性的影响；
- 通过 `--decision fsm|game` 对比规则决策与收益矩阵决策；
- 通过 `--dynamic` 切换运动学/2-DOF 动力学，观察模型差异。

### 2.4 学术算法原型验证
- 系统提供 `BasePlanner` / `BaseDecisionMaker` / `BaseController` 抽象接口，新增算法只需实现对应接口；
- 但不建议把本系统当作高保真 3D 物理引擎；它更适合验证决策/规划/控制逻辑与 KPI 链路。

---

## 3. 部署方式与系统集成

### 3.1 运行环境
- macOS 12+ / Ubuntu 20.04+ / Windows 10+；
- Python 3.9+；
- 必需依赖：NumPy、PyYAML（场景加载）、Matplotlib（可视化/导出）；SciPy 可选（缺失时 LQR 使用 numpy Riccati 迭代）。

### 3.2 部署步骤
```bash
git clone https://github.com/fwwwwjj/intelligent-vehicle-simulation.git
cd intelligent-vehicle-simulation
pip install -r requirements.txt

# 无图形环境运行
python3 scripts/run_simulation.py --scenario highway_merge --decision fsm --duration 20 --no_vis

# 批量评测
python3 scripts/run_benchmark.py --scenarios highway_merge emergency_avoidance \
  --decisions fsm game --seeds 42 --duration 20
```

### 3.3 外部仿真器与中间件（路线图）

- **ROS / ROS2**：计划提供 `VehicleState` / `Trajectory` 与 `nav_msgs/Path`、`geometry_msgs/Twist` 的转换节点；当前未实现。
- **CARLA / SUMO**：计划以本系统作为决策大脑，通过 Co-Simulation 驱动 3D 场景；当前未实现。
- **HIL**：计划接入 CAN/Ethernet/DDS 与域控制器；当前未实现。

在实现并验证之前，以上内容只能作为“应用展望”，不能作为系统已具备的工程接口。
