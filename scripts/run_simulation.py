#!/usr/bin/env python3
"""
单场景闭环仿真运行脚本。
从 configs/scenarios/*.yaml（或内置场景）加载路网、自车与障碍物，
装配核心 FSM、Frenet Lattice、LQR 控制，输出任务级 KPI；Game/EM/MPC/Pacejka 为未来工作接口。
"""

import argparse
import json
import os
import sys
from dataclasses import asdict
from pathlib import Path

# 将项目根目录添加至系统路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.environment.scenario_loader import load_named_scenario, load_scenario
from src.runner import build_engine


def main():
    parser = argparse.ArgumentParser(description="智能车辆行为决策与路径规划数字仿真系统")
    parser.add_argument("--scenario", type=str, default="highway_merge", help="场景名称或 YAML/JSON 路径")
    parser.add_argument("--decision", type=str, default="fsm", choices=["fsm", "game"], help="决策算法；game 为探索性/未来工作")
    parser.add_argument("--duration", type=float, default=20.0, help="仿真运行时长 (秒)")
    parser.add_argument("--no_vis", action="store_true", help="禁用实时可视化窗口")
    parser.add_argument("--seed", type=int, default=None, help="随机种子；默认使用场景配置")
    parser.add_argument("--config", type=str, default=None, help="默认配置文件路径")
    parser.add_argument("--save_results", action="store_true", help="保存逐帧 telemetry JSON 与 KPI")
    parser.add_argument("--dynamic", action="store_true", help="使用 2-DOF 动力学模型（未来工作）")
    parser.add_argument("--planner", type=str, default=None, choices=["lattice", "em"], help="规划器；em 为未来工作")
    parser.add_argument("--controller", type=str, default=None, choices=["lqr", "mpc"], help="控制器；mpc 为未来工作")
    parser.add_argument("--tire", type=str, default=None, choices=["linear", "pacejka"], help="轮胎模型；pacejka 为未来工作")
    parser.add_argument(
        "--allow-experimental",
        action="store_true",
        help="允许运行 Game/EM/MPC/Pacejka/交叉口等实验性模块（结果不纳入主结果）",
    )
    args = parser.parse_args()

    if args.scenario.endswith((".yaml", ".yml", ".json")):
        scenario = load_scenario(args.scenario, config_path=args.config)
    else:
        scenario = load_named_scenario(args.scenario)

    planner_name = args.planner or scenario.planning_params.get("planner", "lattice")
    controller_name = args.controller or scenario.control_params.get("controller", "lqr")
    tire_name = args.tire or scenario.vehicle_params.get("tire_model", "linear")

    print("==================================================")
    print("启动智能车辆仿真系统 - 中国仿真创新应用大赛")
    print(f"场景: {scenario.name}  |  {scenario.description}")
    print(f"决策: {args.decision.upper()}  |  规划: {planner_name.upper()}  |  控制: {controller_name.upper()}")
    print(f"轮胎: {tire_name.upper()}  |  步长: {scenario.dt:.3f}s  |  种子: {args.seed or scenario.seed}")
    print("==================================================")

    engine = build_engine(
        scenario,
        decision=args.decision,
        seed=args.seed,
        enable_visualization=not args.no_vis,
        use_dynamic_model=args.dynamic,
        planner=args.planner,
        controller=args.controller,
        tire_model=args.tire,
    )

    if engine.experimental:
        reasons = ", ".join(engine.experimental_reasons)
        print(f"[EXPERIMENTAL] 当前运行包含实验性模块：{reasons}")
        print("[EXPERIMENTAL] 结果不纳入论文/答辩主结果；需显式加 --allow-experimental 才继续。")
        if not args.allow_experimental:
            return 2

    print(">>> 仿真运行中...")
    report = engine.run(max_duration=args.duration)

    print("\n---------------- 仿真评测 KPI 报告 ----------------")
    for key, value in report.items():
        print(f"  {key:30s}: {value:.4f}")
    print("--------------------------------------------------")

    if args.save_results:
        out_dir = Path("results")
        out_dir.mkdir(parents=True, exist_ok=True)
        suffix = f"{scenario.name}_{args.decision}_seed{args.seed or scenario.seed}"
        telemetry_path = out_dir / f"telemetry_{suffix}.json"
        report_path = out_dir / f"kpi_{suffix}.json"
        with telemetry_path.open("w", encoding="utf-8") as fh:
            json.dump([asdict(record) for record in engine.evaluator.records], fh, ensure_ascii=False, indent=2)
        with report_path.open("w", encoding="utf-8") as fh:
            json.dump(report, fh, ensure_ascii=False, indent=2)
        print(f">>> 逐帧数据: {telemetry_path}")
        print(f">>> KPI 报告: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
