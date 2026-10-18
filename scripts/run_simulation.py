#!/usr/bin/env python3
"""
单场景交互式数字仿真运行脚本
提供基于配置文件的仿真环境加载、闭环控制与动态可视化演示
"""

import sys
import os
import argparse

# 将项目根目录添加至系统路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.common.state import VehicleState, ObstacleState, ObstacleType
from src.environment.road_network import RoadNetwork, Lane
from src.environment.obstacle_manager import ObstacleManager, IntelligentDriverModel
from src.decision.rule_based_fsm import HierarchicalFSMDecisionMaker
from src.decision.game_theoretic import GameTheoreticDecisionMaker
from src.planning.frenet_lattice import FrenetLatticePlanner
from src.control.lqr_controller import LQRController
from src.simulator import SimulationEngine


def main():
    parser = argparse.ArgumentParser(description="智能车辆行为决策与路径规划数字仿真系统")
    parser.add_argument("--scenario", type=str, default="highway_merge", help="场景名称")
    parser.add_argument("--decision", type=str, default="fsm", choices=["fsm", "game"], help="决策算法: fsm 或 game")
    parser.add_argument("--duration", type=float, default=15.0, help="仿真运行时长 (秒)")
    parser.add_argument("--no_vis", action="store_true", help="禁用实时可视化窗口")
    args = parser.parse_args()

    print(f"==================================================")
    print(f"启动智能车辆仿真系统 - 中国仿真创新应用大赛")
    print(f"当前测试场景: {args.scenario}")
    print(f"选用决策模块: {args.decision.upper()}")
    print(f"==================================================")

    # 1. 搭建数字路网
    road_network = RoadNetwork()
    lane_0 = Lane(
        lane_id="main_lane_0",
        center_points=[(float(x), 0.0) for x in range(0, 300, 2)]
    )
    road_network.add_lane(lane_0)
    road_network.generate_reference_line("main_lane_0")

    # 2. 初始化环境障碍物
    obs_manager = ObstacleManager()
    lead_car = ObstacleState(
        id="obs_lead_01",
        obstacle_type=ObstacleType.CAR,
        x=50.0,
        y=0.0,
        length=4.8,
        width=1.9,
        heading=0.0,
        vx=15.0,
        vy=0.0
    )
    obs_manager.add_obstacle(lead_car, IntelligentDriverModel(v0=16.0))

    # 3. 初始化主车状态
    ego_init = VehicleState(
        x=0.0,
        y=0.0,
        heading=0.0,
        velocity=18.0,
        vx=18.0
    )

    # 4. 装配决策与规划算法
    decision_maker = HierarchicalFSMDecisionMaker() if args.decision == "fsm" else GameTheoreticDecisionMaker()
    planner = FrenetLatticePlanner()
    controller = LQRController()

    # 5. 实例化仿真引擎并运行
    engine = SimulationEngine(
        road_network=road_network,
        obstacle_manager=obs_manager,
        decision_maker=decision_maker,
        planner=planner,
        controller=controller,
        ego_initial_state=ego_init,
        enable_visualization=not args.no_vis,
        dt=0.05
    )

    print(">>> 仿真运行中...")
    report = engine.run(max_duration=args.duration)

    print("\n---------------- 仿真评测 KPI 报告 ----------------")
    for k, v in report.items():
        print(f"  {k:30s}: {v:.4f}")
    print("--------------------------------------------------")


if __name__ == "__main__":
    main()
