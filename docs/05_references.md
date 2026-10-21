# 第五部分：代码体系与参考文献

> **范围说明（v0.4-core）**：当前核心代码链路为 HFSM + Frenet Lattice + LQR + 运动学模型 + 任务级 KPI；Game/EM/MPC/Pacejka/交叉口为未来工作，代码保留但不纳入主结果。详见 [07_scope_and_future_work.md](07_scope_and_future_work.md)。

## 1. 代码系统架构索引

```
intelligent-vehicle-simulation/
├── README.md                   # 项目总览：核心链路 + 未来工作
├── requirements.txt            # 系统核心 Python 科学计算依赖清单
├── LICENSE                     # MIT
├── .gitignore                  # Git 忽略配置
├── configs/                    # 场景拓扑与算法超参数 YAML 配置文件
│   ├── default_config.yaml     # 仿真步长、车辆几何、动力学物理参数
│   └── scenarios/              # 三个核心场景 + 实验性交叉口
├── docs/                       # 大赛技术支撑材料
│   ├── 01_problem_and_data.md  # 问题来源与数据来源说明
│   ├── 02_digital_models.md    # 数字模型（核心/未来工作标注）
│   ├── 03_results_and_demo.md  # 核心 30-seed 结果与分析
│   ├── 04_application.md       # 应用与工程落地
│   ├── 05_references.md        # 代码体系与参考文献
│   ├── 06_qp_mpc_em_pacejka.md # 未来工作技术储备
│   ├── 07_scope_and_future_work.md  # 范围与未来工作清单
│   ├── 08_paper_draft.md        # 论文草稿（v0.4-core）
│   ├── 09_slides_and_video_outline.md  # 答辩 PPT/视频大纲
│   └── 10_highd_calibration_replay.md  # highD 最小可信版（标定/回放）
├── src/                        # 核心仿真引擎与算法接口
│   ├── common/                 # 车辆状态、Frenet 映射、OBB 碰撞、QP 后端
│   ├── environment/            # 道路网络、场景加载、IDM/MOBIL、highD 解析
│   │   ├── road_network.py     # 车道拓扑与真实 s/theta/kappa 参考线
│   │   ├── scenario_loader.py  # YAML/JSON 场景 -> 路网/障碍物/自车
│   │   ├── obstacle_manager.py # IDM 步进与预测轨迹
│   │   └── dataset_loader.py   # highD 格式 CSV 解析；INTERACTION/NGSIM 为接口
│   ├── vehicle_model/          # 运动学（核心）、2-DOF/Pacejka（未来工作）、传感器
│   ├── decision/               # HFSM（核心）、Game（探索性）
│   ├── planning/               # Frenet Lattice（核心）、EM/QP（探索性）
│   ├── control/                # LQR/PID（核心）、MPC（探索性）
│   ├── evaluation/             # 任务级 KPI 与批量 benchmark
│   ├── visualization/          # 2D BEV 与真实遥测图表
│   ├── runner.py               # 场景 -> 决策/规划/控制/引擎装配工厂
│   └── simulator.py            # 端到端闭环仿真主引擎
├── scripts/                    # run_simulation / run_benchmark / batches / export_* / highD 标定回放
└── tests/                      # 157 项单元/闭环测试（含任务成功判据）
```

---

### 1.1 回归测试与结果复现

```bash
# 低功耗测试（当前 157 项；含任务级闭环用例；约 10–20 s，视机器负载）
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1 \
  nice -n 19 python3 -B -m unittest discover tests -v

# 核心场景闭环 + KPI
python3 scripts/run_simulation.py --scenario highway_merge --decision fsm --duration 20 --no_vis

# 核心基准（30 seeds；论文目标 30–50 seeds）
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1 \
  nice -n 19 python3 -B scripts/run_benchmark.py \
  --scenarios highway_merge cut_in emergency_avoidance \
  --decisions fsm --seeds 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64 65 66 67 68 69 70 71 --duration 20
```

---

## 2. 核心参考文献与行业标准规范

### 2.1 规划与决策经典文献
1. **Werling, M., Ziegler, J., Kammel, S., & Thrun, S.** (2010). Optimal trajectory generation for dynamic street scenarios in a Frenet Frame. In *IEEE International Conference on Robotics and Automation (ICRA)*, pp. 987–993. *(奠定了 Frenet 坐标系轨迹采样的理论基石)*
2. **Fan, H., Zhu, F., Liu, C., Zhang, L., Zhuang, L., Li, D., ... & Hu, H.** (2018). Baidu Apollo EM Motion Planner. *arXiv preprint arXiv:1807.08048*. *(工业界标杆路径-速度解耦优化方法)*
3. **Li, L., Zheng, N., & Wang, F. Y.** (2014). A survey of motion planning and control techniques for self-driving vehicles. *IEEE Transactions on Intelligent Vehicles*, 1(1), 23-46.
4. **Schwarting, W., Alonso-Mora, J., & Rus, D.** (2018). Planning and decision-making for autonomous vehicles. *Annual Review of Control, Robotics, and Autonomous Systems*, 1, 187-210.
5. **Treiber, M., Hennecke, A., & Helbing, D.** (2000). Congested traffic states in empirical observations and microscopic simulations. *Physical Review E*, 62(2), 1805. *(智能驾驶员模型 IDM 原理文献)*

### 2.2 车辆动力学与控制权威文献
6. **Pacejka, H. B.** (2005). *Tire and vehicle dynamics*. Elsevier. *(Pacejka 魔术公式经典专著)*
7. **Rajamani, R.** (2011). *Vehicle dynamics and control*. Springer Science & Business Media. *(二自由度动力学与横向跟踪经典教材)*
8. **Snider, J. M.** (2009). Automatic steering methods for autonomous automobile path tracking. *Robotics Institute, Carnegie Mellon University, Tech. Rep. CMU-RI-TR-09-08*.

### 2.3 真实数据集与评测基准
9. **Krajewski, R., Bock, J., Kloeker, L., & Eckstein, L.** (2018). The highD dataset: A drone dataset of naturalistic vehicle trajectories on German highways for validation of highly automated driving systems. In *IEEE 21st International Conference on Intelligent Transportation Systems (ITSC)*, pp. 2118-2125.
10. **Zhan, W., Sun, L., Wang, D., Shi, H., Clausse, A., Naumann, M., ... & Tomizuka, M.** (2019). INTERACTION Dataset: An international, adversarial and cooperative driving dataset in interactive scenarios with semantic maps. *arXiv preprint arXiv:1910.03088*.

### 2.4 国际与国家行业标准
11. **ISO 26262:2018** Road vehicles — Functional safety (道路车辆功能安全).
12. **ISO 21448:2022** Road vehicles — Safety of the intended functionality (预期功能安全 SOTIF).
13. **ASAM OpenSCENARIO & OpenDRIVE** Specification for dynamic and static simulation environment description.
14. **GB/T 40429-2021** 《汽车驾驶自动化分级》国家标准.
