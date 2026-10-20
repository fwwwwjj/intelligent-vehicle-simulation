"""
仿真实验结果科研图表绘制工具集。
Matplotlib 高分辨率遥测与算法对比图。
"""
from pathlib import Path
from typing import List, Optional

import matplotlib.pyplot as plt
import numpy as np


class ScientificPlotter:
    """科研级实验分析图表绘制器。"""

    @staticmethod
    def plot_telemetry_curves(
        time_series: List[float],
        speeds: List[float],
        lateral_errors: List[float],
        jerks: List[float],
        save_path: str = "results/telemetry_analysis.png",
        ttcs: Optional[List[float]] = None,
        ttc_threshold: float = 2.7,
    ) -> None:
        """Plot speed, lateral error, jerk, and optional ego-to-lead TTC."""
        fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
        axes[0].plot(time_series, np.asarray(speeds) * 3.6, color="tab:blue", linewidth=1.8)
        axes[0].set_ylabel("Speed (km/h)")
        axes[0].set_title("Autonomous Vehicle Dynamic Performance Analysis")
        axes[0].grid(True, linestyle="--", alpha=0.6)
        axes[1].plot(time_series, lateral_errors, color="tab:red", linewidth=1.6)
        axes[1].set_ylabel("Lateral Error (m)")
        axes[1].grid(True, linestyle="--", alpha=0.6)
        axes[2].plot(time_series, jerks, color="tab:green", linewidth=1.5)
        axes[2].set_ylabel("Jerk (m/s³)")
        axes[2].grid(True, linestyle="--", alpha=0.6)
        axes[3].axhline(ttc_threshold, color="red", linestyle="--", linewidth=1.5,
                        label=f"Safety warning ({ttc_threshold:.1f} s)")
        if ttcs is not None and len(ttcs):
            n = min(len(time_series), len(ttcs))
            values = np.asarray(ttcs[:n], dtype=float)
            values[(values >= 50) | (values < 0)] = np.nan
            axes[3].plot(np.asarray(time_series[:n]), values, color="purple", linewidth=1.7,
                         label="Ego-to-lead TTC")
            axes[3].set_ylim(bottom=0)
        else:
            axes[3].text(0.5, 0.5, "TTC data not provided", transform=axes[3].transAxes,
                         ha="center", va="center", color="0.4")
        axes[3].set_ylabel("TTC (s)")
        axes[3].set_xlabel("Time (s)")
        axes[3].legend(loc="best", fontsize=8)
        axes[3].grid(True, linestyle="--", alpha=0.6)
        fig.tight_layout()
        output = Path(save_path)
        output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output, dpi=300, bbox_inches="tight")
        plt.close(fig)

    @staticmethod
    def plot_decision_comparison(fsm_records, game_records, save_path: str) -> None:
        """Compare speed smoothness, jerk, and TTC safety for two algorithms."""
        def value(record, *keys, default=np.nan):
            for key in keys:
                if isinstance(record, dict) and key in record:
                    return record[key]
                if hasattr(record, key):
                    return getattr(record, key)
            return default

        def unpack(records):
            records = list(records or [])
            time = np.asarray([value(r, "timestamp", "time", "t") for r in records], dtype=float)
            speed = np.asarray([value(r, "velocity", "speed", "v") for r in records], dtype=float) * 3.6
            jerk = np.asarray([value(r, "jerk", "longitudinal_jerk") for r in records], dtype=float)
            ttc = np.asarray([value(r, "min_ttc", "ttc", "minimum_ttc") for r in records], dtype=float)
            ttc[(ttc >= 50) | (ttc < 0)] = np.nan
            return time, speed, jerk, ttc

        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, constrained_layout=True)
        for records, label, color in ((fsm_records, "FSM", "#1f77b4"),
                                      (game_records, "Game-theoretic", "#d95f02")):
            time, speed, jerk, ttc = unpack(records)
            axes[0].plot(time, speed, color=color, linewidth=1.8, label=label)
            axes[1].plot(time, jerk, color=color, linewidth=1.6, label=label)
            axes[2].plot(time, ttc, color=color, linewidth=1.7, label=label)
        axes[0].set_ylabel("Speed (km/h)")
        axes[0].set_title("Decision Algorithm Comparison")
        axes[1].set_ylabel("Jerk (m/s³)")
        axes[2].set_ylabel("TTC (s)")
        axes[2].set_xlabel("Time (s)")
        axes[2].axhline(2.7, color="red", linestyle="--", linewidth=1.5,
                        label="Safety warning (2.7 s)")
        for ax in axes:
            ax.grid(True, linestyle="--", alpha=0.55)
            ax.legend(loc="best", fontsize=8, framealpha=0.9)
        output = Path(save_path)
        output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output, dpi=300, bbox_inches="tight")
        plt.close(fig)
