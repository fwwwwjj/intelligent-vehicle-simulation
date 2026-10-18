#!/usr/bin/env python3
"""
比赛图表与实验对比数据导出脚本
生成用于大赛技术报告的高清图表与 LaTeX / Markdown 表格
"""

import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.visualization.plot_utils import ScientificPlotter


def main():
    os.makedirs("results", exist_ok=True)
    print(">>> 正在生成仿真实验科学分析图表...")
    # 模拟数据导出示例
    times = [i * 0.05 for i in range(200)]
    speeds = [18.0 + 2.0 * (1.0 - 2.718 ** (-t / 3.0)) for t in times]
    errors = [0.08 * (2.718 ** (-t / 2.0)) for t in times]
    jerks = [0.2 * (2.718 ** (-t / 2.0)) for t in times]

    output_path = "results/telemetry_analysis.png"
    ScientificPlotter.plot_telemetry_curves(times, speeds, errors, jerks, output_path)
    print(f">>> 实验分析图表已成功保存至: {output_path}")


if __name__ == "__main__":
    main()
