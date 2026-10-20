#!/usr/bin/env python3
"""LEGACY：旧版申报材料图表脚本（包含 Game 对比，已不属于 v0.4-core）。

当前核心范围（FSM + Lattice + LQR）的图表请使用 `scripts/export_results.py` 或后续新版脚本。
如确需运行本脚本，请显式设置环境变量 `SIAC_ALLOW_LEGACY_EXPORT=1`。
"""

from __future__ import annotations

import argparse
import logging
import math
import os
import sys
from pathlib import Path

import matplotlib


def _parse_args():
    parser = argparse.ArgumentParser(
        description="运行高速汇入/紧急避撞闭环仿真，生成大赛申报材料图表和 KPI 表。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--seed", type=int, default=42, help="所有仿真共同使用的随机种子")
    parser.add_argument("--duration", type=float, default=20.0, help="每次仿真时长（秒）")
    parser.add_argument("--output", type=Path, default=None, help="输出目录；默认 results/competition_pack")
    parser.add_argument("--no_vis", action="store_true", help="无头模式，不打开仿真窗口（仍导出静态图）")
    parser.add_argument("--log-level", choices=["DEBUG", "INFO", "WARNING", "ERROR"], default="INFO")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if os.environ.get("SIAC_ALLOW_LEGACY_EXPORT") != "1":
        print("LEGACY 脚本：包含 Game/旧版 KPI，不属于 v0.4-core；设置 SIAC_ALLOW_LEGACY_EXPORT=1 才可运行。")
        return 1
    logging.basicConfig(level=getattr(logging, args.log_level), format="%(asctime)s %(levelname)s %(message)s")
    logger = logging.getLogger("competition_assets")
    project_root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(project_root))
    if args.no_vis:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from src.environment.scenario_loader import load_named_scenario
    from src.runner import build_engine
    from src.visualization.visualizer_2d import Visualizer2D

    available_fonts = {font.name for font in font_manager.fontManager.ttflist}
    for chinese_font in ("Hiragino Sans GB", "Arial Unicode MS", "Noto Sans CJK SC"):
        if chinese_font in available_fonts:
            plt.rcParams["font.sans-serif"] = [chinese_font, "DejaVu Sans"]
            break
    plt.rcParams["axes.unicode_minus"] = False

    out_dir = args.output or project_root / "results" / "competition_pack"
    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    logger.info("输出目录：%s；seed=%d；duration=%.2fs", out_dir, args.seed, args.duration)

    runs = {}
    scenario_names = ("highway_merge", "emergency_avoidance")
    for scenario_name in scenario_names:
        for decision in ("fsm", "game"):
            logger.info("运行 %s / %s 闭环仿真", scenario_name, decision)
            scenario = load_named_scenario(scenario_name)
            engine = build_engine(scenario, decision=decision, seed=args.seed, enable_visualization=False)
            report = engine.run(max_duration=args.duration)
            runs[(scenario_name, decision)] = (engine, report)

    # 汇入场景关键博弈时刻：挑选实际记录中净距最小的一帧，并回放该帧状态。
    merge_engine, _ = runs[("highway_merge", "game")]
    records = merge_engine.evaluator.records
    index = min(range(len(records)), key=lambda i: records[i].min_obstacle_distance) if records else 0
    snapshot = build_engine(load_named_scenario("highway_merge"), decision="game", seed=args.seed, enable_visualization=False)
    for _ in range(index + 1):
        snapshot.step()
    Visualizer2D.save_bev_snapshot(snapshot.ego_state, snapshot.obstacle_manager.get_all_obstacles(),
                                   snapshot.road_network, snapshot.last_trajectory,
                                   snapshot.current_time, str(fig_dir / "fig1_highway_merge_bev.png"))

    # FSM/Game 对比：速度、加速度、Jerk、TTC 四面板共享时间轴。
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), constrained_layout=True)
    panels = [("velocity", "速度 (m/s)"), ("acceleration", "加速度 (m/s^2)"),
              ("jerk", "Jerk (m/s^3)"), ("min_ttc", "TTC (s)")]
    for ax, (field, title) in zip(axes.flat, panels):
        for decision, color in (("fsm", "#2878B5"), ("game", "#D9534F")):
            recs = runs[("highway_merge", decision)][0].evaluator.records
            ax.plot([r.timestamp for r in recs], [min(getattr(r, field), 50.0) if field == "min_ttc" else getattr(r, field) for r in recs],
                    label=decision.upper(), color=color, linewidth=1.6)
        ax.set_title(title); ax.set_xlabel("时间 (s)"); ax.grid(alpha=.25)
    axes[0, 0].legend(frameon=False)
    fig.suptitle("高速汇入：FSM 与 Game 闭环决策对比")
    fig.savefig(fig_dir / "fig2_decision_comparison_curves.png", dpi=220)
    plt.close(fig)

    # 紧急避撞的真实轨迹和代表性停车帧。
    emergency, _ = runs[("emergency_avoidance", "fsm")]
    erecs = emergency.evaluator.records
    stop_index = next((i for i, r in enumerate(erecs) if r.velocity <= 0.2), len(erecs) - 1)
    stop_view = build_engine(load_named_scenario("emergency_avoidance"), decision="fsm", seed=args.seed, enable_visualization=False)
    for _ in range(stop_index + 1):
        stop_view.step()
    emergency_path = fig_dir / "fig3_emergency_avoidance_bev.png"
    Visualizer2D.save_bev_snapshot(stop_view.ego_state, stop_view.obstacle_manager.get_all_obstacles(),
                                   stop_view.road_network, stop_view.last_trajectory,
                                   stop_view.current_time, str(emergency_path))
    # 同一张图保留停车时刻的 BEV，并展示该时刻之前的速度与实际 OBB 净距。
    bev_image = plt.imread(emergency_path)
    fig, (bev_ax, timeline_ax) = plt.subplots(
        2, 1, figsize=(13, 9), gridspec_kw={"height_ratios": [2.1, 1]}, constrained_layout=True
    )
    bev_ax.imshow(bev_image)
    bev_ax.axis("off")
    shown = erecs[:stop_index + 1]
    times = [r.timestamp for r in shown]
    timeline_ax.plot(times, [r.velocity for r in shown], color="#2878B5", label="自车速度 (m/s)")
    timeline_ax.set(xlabel="时间 (s)", ylabel="速度 (m/s)")
    timeline_ax.grid(alpha=.25)
    clearance_ax = timeline_ax.twinx()
    clearance_ax.plot(times, [r.min_obstacle_distance for r in shown], color="#D9534F", label="最小 OBB 净距 (m)")
    clearance_ax.set_ylabel("净距 (m)")
    clearance_ax.set_ylim(bottom=0)
    handles_1, labels_1 = timeline_ax.get_legend_handles_labels()
    handles_2, labels_2 = clearance_ax.get_legend_handles_labels()
    timeline_ax.legend(handles_1 + handles_2, labels_1 + labels_2, loc="upper right", frameon=False)
    fig.savefig(emergency_path, dpi=220)
    plt.close(fig)

    # LQR 跟踪误差/航向偏差收敛曲线（使用同一紧急避撞 FSM run 的逐帧测量）。
    fig, ax = plt.subplots(figsize=(10, 4.8), constrained_layout=True)
    t = [r.timestamp for r in erecs]
    ax.plot(t, [r.lateral_error for r in erecs], label="横向误差 (m)", linewidth=1.8)
    ax.plot(t, [r.heading_error for r in erecs], label="航向角偏差 (rad)", linewidth=1.8)
    ax.axhline(0, color="black", linewidth=.7, alpha=.5)
    ax.set(title="LQR 横向跟踪误差与航向角偏差", xlabel="时间 (s)", ylabel="误差")
    ax.grid(alpha=.25); ax.legend(frameon=False)
    fig.savefig(fig_dir / "fig4_tracking_performance.png", dpi=220)
    plt.close(fig)

    columns = [("collision_rate", "碰撞率", "{:.2%}"), ("min_obstacle_distance_m", "最小净距 (m)", "{:.3f}"),
               ("min_ttc_s", "最小 TTC (s)", "{:.3f}"), ("rms_jerk_m_s3", "RMS Jerk (m/s³)", "{:.3f}"),
               ("max_lateral_error_m", "最大横向误差 (m)", "{:.3f}"),
               ("avg_planning_latency_ms", "平均规划时延 (ms)", "{:.3f}")]
    lines = ["| 场景 | 决策方法 | " + " | ".join(label for _, label, _ in columns) + " |",
             "| :--- | :--- | " + " | ".join(":---:" for _ in columns) + " |"]
    for scenario_name in scenario_names:
        for decision in ("fsm", "game"):
            report = runs[(scenario_name, decision)][1]
            vals = [fmt.format(report.get(key, math.nan)) for key, _, fmt in columns]
            lines.append(f"| {scenario_name} | {decision.upper()} | " + " | ".join(vals) + " |")
    lines.extend(["", f"> 仿真时长：每组 {args.duration:.2f} s；随机种子：{args.seed}。指标由闭环逐帧记录汇总。", 
                  "> TTC=999 表示仿真期间无满足前方且横向重叠条件的接近目标；单 seed 结果用于演示，不代表统计性结论。"])
    table = "\n".join(lines) + "\n"
    (out_dir / "kpi_table.md").write_text(table, encoding="utf-8")
    print(table)
    logger.info("申报材料已导出：%s", out_dir)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception:
        logging.exception("申报材料资产生成失败")
        raise SystemExit(1)
