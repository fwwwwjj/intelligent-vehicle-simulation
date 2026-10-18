"""
数字道路网络与拓扑地图接口。
支持多车道定义、中心参考线生成、曲率计算以及拓扑连通关系维护。
"""

from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
import numpy as np
from src.common.frenet_frame import ReferencePoint


@dataclass
class Lane:
    """车道数字模型"""
    lane_id: str                       # 车道唯一标识
    width: float = 3.75                # 车道宽 (m)
    speed_limit: float = 22.22         # 限速 (m/s)
    left_lane_id: Optional[str] = None # 左相邻车道ID (用于换道决策)
    right_lane_id: Optional[str] = None# 右相邻车道ID
    predecessors: List[str] = field(default_factory=list) # 前驱车道列表
    successors: List[str] = field(default_factory=list)   # 后继车道列表
    center_points: List[Tuple[float, float]] = field(default_factory=list) # 中心线几何采样点


class RoadNetwork:
    """
    高精度数字道路网络管理器
    管理车道几何、拓扑以及全局参考线平滑
    """
    def __init__(self):
        self.lanes: Dict[str, Lane] = {}
        self.reference_lines: Dict[str, List[ReferencePoint]] = {}

    def add_lane(self, lane: Lane) -> None:
        """注册新车道"""
        self.lanes[lane.lane_id] = lane

    def generate_reference_line(self, lane_id: str, sample_step: float = 0.5) -> List[ReferencePoint]:
        """
        基于车道中心几何离散点，通过样条插值/优化平滑生成带有 (x, y, theta, kappa, s) 的高阶参考线
        :param lane_id: 车道ID
        :param sample_step: 离散采样步长 (m)
        :return: 连续平滑参考线点序列
        """
        if lane_id not in self.lanes:
            raise KeyError(f"Lane {lane_id} not found in RoadNetwork")

        lane = self.lanes[lane_id]
        ref_line = []
        # 接口实现占位：后续接入B样条或Fem-Pos-Smoother
        accumulated_s = 0.0
        for i, (gx, gy) in enumerate(lane.center_points):
            ref_line.append(ReferencePoint(
                x=gx,
                y=gy,
                theta=0.0,
                kappa=0.0,
                dkappa=0.0,
                s=accumulated_s
            ))
            accumulated_s += sample_step

        self.reference_lines[lane_id] = ref_line
        return ref_line

    def get_reference_line(self, lane_id: str) -> Optional[List[ReferencePoint]]:
        """获取指定车道生成的参考线"""
        return self.reference_lines.get(lane_id, None)
