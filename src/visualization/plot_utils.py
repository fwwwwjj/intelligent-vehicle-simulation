"""
仿真实验结果科研图表绘制工具集。
支持速度响应、横向跟踪偏差、加速度与加加速度舒适性曲线绘制。
一键导出符合比赛申报书与技术报告格式的高清矢量图 (PDF/PNG)。
"""

from typing import List, Dict
import matplotlib.pyplot as plt
import numpy as np


class ScientificPlotter:
    """
    科研级实验分析图表绘制器
    """
    @staticmethod
    def plot_telemetry_curves(
        time_series: List[float],
        speeds: List[float],
        lateral_errors: List[float],
        jerks: List[float],
        save_path: str = "results/telemetry_analysis.png"
    ) -> None:
        """
        绘制三合一遥测分析图：车速对比、横向误差、加加速度曲线
        """
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

        # 速度跟踪
        axes[0].plot(time_series, np.array(speeds) * 3.6, "b-", linewidth=2.0)
        axes[0].set_ylabel("Speed (km/h)")
        axes[0].set_title("Autonomous Vehicle Dynamic Performance Analysis")
        axes[0].grid(True, linestyle="--", alpha=0.6)

        # 横向跟踪误差
        axes[1].plot(time_series, lateral_errors, "r-", linewidth=1.8)
        axes[1].set_ylabel("Lateral Error (m)")
        axes[1].grid(True, linestyle="--", alpha=0.6)

        # 舒适性 Jerk
        axes[2].plot(time_series, jerks, "g-", linewidth=1.5)
        axes[2].set_ylabel("Jerk (m/s³)")
        axes[2].set_xlabel("Time (s)")
        axes[2].grid(True, linestyle="--", alpha=0.6)

        plt.tight_layout()
        plt.savefig(save_path, dpi=300)
        plt.close(fig)
