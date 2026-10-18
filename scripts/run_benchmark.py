#!/usr/bin/env python3
"""
批量基准场景评测与自动化打分脚本
"""

import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.evaluation.benchmark import BenchmarkSuite


def main():
    print(">>> 启动批量场景基准评测...")
    scenarios = [
        "configs/scenarios/highway_merge.yaml",
        "configs/scenarios/urban_intersection.yaml",
        "configs/scenarios/emergency_avoidance.yaml"
    ]
    suite = BenchmarkSuite(scenarios)
    results = suite.run_all()
    print(">>> 批量评测完成！")


if __name__ == "__main__":
    main()
