"""
分层有限状态机 (Hierarchical Finite State Machine, HFSM) 行为决策器。
基于规则驱动与安全距离判定 (TTC, Time-To-Collision)，实现经典场景的确定性意图决策。
"""

from typing import List, Optional
import numpy as np
from src.common.state import VehicleState, ObstacleState
from src.decision.base_decision import BaseDecisionMaker, DecisionOutput, DrivingIntent


class HierarchicalFSMDecisionMaker(BaseDecisionMaker):
    """
    分层有限状态机行为决策器实现
    包含巡航、跟驰、超车变道及紧急停车等状态迁移规则
    """
    def __init__(self, desired_speed: float = 22.22, min_ttc_threshold: float = 2.5):
        self.desired_speed = desired_speed
        self.min_ttc_threshold = min_ttc_threshold
        self.current_state = DrivingIntent.CRUISE

    def make_decision(
        self,
        ego_state: VehicleState,
        obstacles: List[ObstacleState],
        current_lane_id: str
    ) -> DecisionOutput:
        """
        状态转移逻辑推导
        """
        # 寻找同车道前方最近障碍物
        lead_obs, min_distance = self._find_lead_vehicle(ego_state, obstacles)

        # 1. 紧急安全防线判定
        if lead_obs is not None:
            rel_speed = ego_state.velocity - lead_obs.vx
            ttc = min_distance / max(1e-2, rel_speed) if rel_speed > 0 else float("inf")

            if ttc < self.min_ttc_threshold and min_distance < 12.0:
                self.current_state = DrivingIntent.EMERGENCY_STOP
                return DecisionOutput(
                    intent=DrivingIntent.EMERGENCY_STOP,
                    target_lane_id=current_lane_id,
                    target_speed=0.0,
                    lead_obstacle_id=lead_obs.id,
                    reasoning_log=f"Trigger Emergency Stop: TTC={ttc:.2f}s < {self.min_ttc_threshold}s"
                )

            # 2. 跟驰或巡航判定
            if min_distance < 35.0:
                self.current_state = DrivingIntent.FOLLOW
                return DecisionOutput(
                    intent=DrivingIntent.FOLLOW,
                    target_lane_id=current_lane_id,
                    target_speed=min(lead_obs.vx, self.desired_speed),
                    lead_obstacle_id=lead_obs.id,
                    reasoning_log=f"Car following mode active. Distance={min_distance:.1f}m"
                )

        # 3. 正常定速巡航
        self.current_state = DrivingIntent.CRUISE
        return DecisionOutput(
            intent=DrivingIntent.CRUISE,
            target_lane_id=current_lane_id,
            target_speed=self.desired_speed,
            reasoning_log="Lane clear, executing cruise control"
        )

    def _find_lead_vehicle(self, ego_state: VehicleState, obstacles: List[ObstacleState]) -> tuple[Optional[ObstacleState], float]:
        """筛选车道正前方的最近前导车"""
        closest_obs = None
        min_dist = float("inf")
        for obs in obstacles:
            dx = obs.x - ego_state.x
            dy = obs.y - ego_state.y
            # 简化的纵横向投影判断
            if dx > 0.0 and abs(dy) < 1.8:
                if dx < min_dist:
                    min_dist = dx
                    closest_obs = obs
        return closest_obs, min_dist
