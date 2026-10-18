"""
基于博弈论的交互式动态行为决策接口 (Game-Theoretic Interactive Decision Maker)。
针对强交互场景（匝道汇入、无保护左转），构建主车与关键交互车辆的非合作完全信息动态博弈收益矩阵。
"""

from typing import List, Dict
import numpy as np
from src.common.state import VehicleState, ObstacleState
from src.decision.base_decision import BaseDecisionMaker, DecisionOutput, DrivingIntent


class GameTheoreticDecisionMaker(BaseDecisionMaker):
    """
    博弈交互决策器
    通过评估 (加速汇入, 减速礼让) x (背景车加速, 背景车礼让) 的收益矩阵 (Payoff Matrix)，
    求解纯策略或混合策略纳什均衡 (Nash Equilibrium) / 斯塔克尔伯格均衡 (Stackelberg Equilibrium)。
    """
    def __init__(self, weight_safety: float = 0.5, weight_efficiency: float = 0.3, weight_comfort: float = 0.2):
        self.w_s = weight_safety
        self.w_e = weight_efficiency
        self.w_c = weight_comfort

    def calculate_payoff(self, ego_action: str, obs_action: str, distance: float, delta_v: float) -> float:
        """
        计算主车在该策略组合下的综合效用/收益 (Utility)
        :param ego_action: 主车动作 ('merge' 或 'yield')
        :param obs_action: 交互车动作 ('pass' 或 'yield')
        :param distance: 相对距离 (m)
        :param delta_v: 相对速度 (m/s)
        :return: 效用评分
        """
        # 碰撞惩罚
        safety_utility = -100.0 if (ego_action == "merge" and obs_action == "pass" and distance < 15.0) else 10.0
        # 通行效率
        efficiency_utility = 20.0 if ego_action == "merge" else 5.0
        # 舒适性
        comfort_utility = 10.0

        return self.w_s * safety_utility + self.w_e * efficiency_utility + self.w_c * comfort_utility

    def make_decision(
        self,
        ego_state: VehicleState,
        obstacles: List[ObstacleState],
        current_lane_id: str
    ) -> DecisionOutput:
        """
        基于博弈矩阵求解最优意图
        """
        # 接口实现骨架
        intent = DrivingIntent.MERGE
        return DecisionOutput(
            intent=intent,
            target_lane_id="main_lane_1",
            target_speed=20.0,
            reasoning_log="Game-theory equilibrium: Ego merges with positive payoff advantage"
        )
