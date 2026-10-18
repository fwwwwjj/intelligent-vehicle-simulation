# 第五部分：代码体系与参考文献

## 1. 代码系统架构索引

```
intelligent-vehicle-simulation/
├── README.md                   # 项目综合介绍、竞赛架构图与快速上手指南
├── requirements.txt            # 系统核心 Python 科学计算依赖清单
├── .gitignore                  # Git 忽略配置
├── configs/                    # 场景拓扑与算法超参数 YAML 配置文件
│   ├── default_config.yaml     # 仿真步长、车辆几何、动力学物理参数
│   └── scenarios/              # 高速汇入、城市交叉口、紧急避障场景配置
├── docs/                       # 中国仿真创新应用大赛官方申报材料规范文档
│   ├── 01_problem_and_data.md  # 问题来源与真实数据集 (highD/INTERACTION) 说明
│   ├── 02_digital_models.md    # 动力学、Frenet映射、博弈决策与优化数学模型
│   ├── 03_results_and_demo.md  # 典型工况量化评估指标 (KPI) 与对比分析
│   ├── 04_application.md       # 工业级软件在环 (SIL) 与虚拟实验教学应用说明
│   └── 05_references.md        # 权威学术参考文献与国际行业标准规范
├── src/                        # 核心仿真引擎与高保真算法接口实现
│   ├── common/                 # 车辆状态、Frenet双向映射、OBB几何碰撞检测
│   ├── environment/            # 数字路网、动态交通参与者 IDM 模型与数据加载器
│   ├── vehicle_model/          # 自行车运动学、2-DOF横向动力学与传感器数字模型
│   ├── decision/               # 分层有限状态机 (HFSM) 与博弈交互决策器
│   ├── planning/               # Frenet Lattice 多项式规划器与 EM/QP 时空优化器
│   ├── control/                # 离散 LQR 横向跟踪 (带曲率前馈) 与纵向双 PID 控制器
│   ├── evaluation/             # 仿真指标量化计算器与批量回归基准评测套件
│   ├── visualization/          # 2D BEV 实时交互动画引擎与科研图表导出工具
│   └── simulator.py            # 端到端闭环仿真主引擎核心控制器
├── scripts/                    # 自动化运行脚本与演示入口
│   ├── run_simulation.py       # 单场景交互式可视化仿真运行入口
│   ├── run_benchmark.py        # 批量基准压力测试与评分脚本
│   └── export_results.py       # 竞赛申报书科研分析图表导出脚本
└── tests/                      # 单元测试与接口集成测试套件
    ├── test_coordinates.py     # 坐标变换准确性校验
    ├── test_vehicle_model.py   # 运动学与动力学积分步进测试
    ├── test_planners.py        # 多项式轨迹连续性检验
    └── test_controllers.py     # LQR 与 PID 控制器稳定性测试
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
