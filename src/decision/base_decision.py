"""
行为决策抽象基类与决策指令定义。
规范宏观行驶意图 (Driving Intent)、目标车道及纵横向安全走廊约束。
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional
from src.common.state import VehicleState, ObstacleState


class DrivingIntent(Enum):
    """战术行为意图枚举"""
    CRUISE = "cruise"                       # 定速巡航
    FOLLOW = "follow"                       # 前车跟驰
    LANE_CHANGE_LEFT = "lane_change_left"   # 向左变道
    LANE_CHANGE_RIGHT = "lane_change_right" # 向右变道
    MERGE = "merge"                         # 匝道合流/汇入
    YIELD = "yield"                         # 减速避让/减速礼让
    OVERTAKE = "overtake"                   # 变道超车
    EMERGENCY_STOP = "emergency_stop"       # 紧急制动停车


@dataclass
class DecisionOutput:
    """
    决策层下发给局部规划层的战术指令结构
    """
    intent: DrivingIntent                   # 核心行为意图
    target_lane_id: str                     # 目标引导车道ID
    target_speed: float                     # 期望巡航速度 (m/s)
    time_headway: float = 1.5               # 期望车头时距 (s)
    lead_obstacle_id: Optional[str] = None  # 决策关注的关键前导车ID
    lateral_shift_l: float = 0.0            # 目标车道中心偏移量 (m)
    reasoning_log: str = ""                 # 决策解释链，用于比赛评审展示


class BaseDecisionMaker(ABC):
    """行为决策器基类"""

    @abstractmethod
    def make_decision(
        self,
        ego_state: VehicleState,
        obstacles: List[ObstacleState],
        current_lane_id: str
    ) -> DecisionOutput:
        """
        根据环境与主车态势进行高阶行为决策
        :param ego_state: 自车状态
        :param obstacles: 感知过滤后的障碍物列表
        :param current_lane_id: 当前所在车道
        :return: 决策输出指令
        """
        pass
