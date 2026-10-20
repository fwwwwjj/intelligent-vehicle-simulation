"""
分层有限状态机 (Hierarchical Finite State Machine, HFSM) 行为决策器。
基于规则驱动与安全距离判定 (TTC, Time-To-Collision)，实现经典场景的确定性意图决策。

决策器可选地持有 :class:`RoadNetwork` 引用：有了车道中心线信息，才能判断
“前车是否在本车道”“目标车道前后间隙是否足够”，而不是只看全局 dx/dy。
"""

from typing import List, Optional, TYPE_CHECKING
import numpy as np

from src.common.state import VehicleState, ObstacleState
from src.decision.base_decision import BaseDecisionMaker, DecisionOutput, DrivingIntent

if TYPE_CHECKING:  # pragma: no cover
    from src.environment.road_network import RoadNetwork


class HierarchicalFSMDecisionMaker(BaseDecisionMaker):
    """
    分层有限状态机行为决策器实现
    包含巡航、跟驰、汇入变道及紧急停车等状态迁移规则
    """

    def __init__(
        self,
        desired_speed: float = 22.22,
        min_ttc_threshold: float = 2.5,
        road_network: Optional["RoadNetwork"] = None,
        merge_lead_gap: float = 20.0,
        merge_follow_gap: float = 15.0,
        ego_length: float = 4.8,
        ego_width: float = 1.9,
        enable_lane_change: bool = True,
    ):
        self.desired_speed = desired_speed
        self.min_ttc_threshold = min_ttc_threshold
        self.road_network = road_network
        self.merge_lead_gap = merge_lead_gap
        self.merge_follow_gap = merge_follow_gap
        self.ego_length = float(ego_length)
        self.ego_width = float(ego_width)
        # 消融实验开关：关闭后 HFSM 只做本车道巡航/跟驰，不再产生汇入/换道意图。
        self.enable_lane_change = bool(enable_lane_change)
        self.current_state = DrivingIntent.CRUISE

    # ------------------------------------------------------------------
    # 车道级感知
    # ------------------------------------------------------------------
    def _lane_center_y(self, lane_id: str) -> Optional[float]:
        if self.road_network is None:
            return None
        try:
            return self.road_network.get_lane_center_y(lane_id)
        except Exception:
            return None

    def _find_lead(
        self, ego_state: VehicleState, obstacles: List[ObstacleState], lane_id: str
    ) -> tuple[Optional[ObstacleState], float]:
        """筛选指定车道正前方的最近前导车，返回 (障碍物, 净车距)。"""
        center_y = self._lane_center_y(lane_id)
        closest_obs = None
        min_gap = float("inf")
        for obs in obstacles:
            dx = obs.x - ego_state.x
            if dx <= 0.0:
                continue
            lateral = abs(obs.y - center_y) if center_y is not None else abs(obs.y - ego_state.y)
            if lateral > 0.5 * (self.ego_width + obs.width) + 0.4:
                continue
            gap = dx - 0.5 * (self.ego_length + obs.length)
            if gap < min_gap:
                min_gap = gap
                closest_obs = obs
        return closest_obs, min_gap

    def _find_follower(
        self, ego_state: VehicleState, obstacles: List[ObstacleState], lane_id: str
    ) -> tuple[Optional[ObstacleState], float]:
        """筛选指定车道正后方的最近跟驰车，返回 (障碍物, 净车距)。"""
        center_y = self._lane_center_y(lane_id)
        closest_obs = None
        min_gap = float("inf")
        for obs in obstacles:
            dx = ego_state.x - obs.x
            if dx <= 0.0:
                continue
            lateral = abs(obs.y - center_y) if center_y is not None else abs(obs.y - ego_state.y)
            if lateral > 0.5 * (self.ego_width + obs.width) + 0.4:
                continue
            gap = dx - 0.5 * (self.ego_length + obs.length)
            if gap < min_gap:
                min_gap = gap
                closest_obs = obs
        return closest_obs, min_gap

    def _merge_target_lane(self, current_lane_id: str) -> Optional[str]:
        if not self.enable_lane_change:
            return None
        if self.road_network is None:
            return None
        if "ramp" not in current_lane_id:
            return None
        for candidate in ("main_lane_0", "main_lane_1"):
            if candidate in self.road_network.lanes:
                return candidate
        return None

    # ------------------------------------------------------------------
    # 状态机
    # ------------------------------------------------------------------
    def make_decision(
        self,
        ego_state: VehicleState,
        obstacles: List[ObstacleState],
        current_lane_id: str
    ) -> DecisionOutput:
        """状态转移逻辑推导。"""
        lead_obs, lead_gap = self._find_lead(ego_state, obstacles, current_lane_id)

        # 1. 紧急安全防线：TTC 与绝对距离双阈值
        if lead_obs is not None:
            rel_speed = ego_state.velocity - lead_obs.vx
            ttc = lead_gap / max(1e-2, rel_speed) if rel_speed > 0 else float("inf")
            if ttc < self.min_ttc_threshold and lead_gap < 12.0:
                self.current_state = DrivingIntent.EMERGENCY_STOP
                return DecisionOutput(
                    intent=DrivingIntent.EMERGENCY_STOP,
                    target_lane_id=current_lane_id,
                    target_speed=0.0,
                    lead_obstacle_id=lead_obs.id,
                    reasoning_log=f"Emergency stop: gap={lead_gap:.1f}m, TTC={ttc:.2f}s",
                )

        # 2. 匝道汇入：检查目标车道前后间隙
        target_lane = self._merge_target_lane(current_lane_id)
        if target_lane is not None:
            target_lead, target_lead_gap = self._find_lead(ego_state, obstacles, target_lane)
            target_follower, target_follow_gap = self._find_follower(ego_state, obstacles, target_lane)
            # follower_closing > 0 表示后车正在逼近（比自车快）
            follower_closing = (
                target_follower.vx - ego_state.velocity if target_follower is not None else 0.0
            )
            lead_safe = target_lead is None or target_lead_gap > self.merge_lead_gap
            # 即使后车没有在逼近，也必须保留最小净距，避免“贴着后车车头”并入
            follow_safe = (
                target_follower is None
                or target_follow_gap > self.merge_follow_gap
                or (target_follow_gap > 5.0 and follower_closing <= 0.0)
            )
            if lead_safe and follow_safe:
                self.current_state = DrivingIntent.MERGE
                target_speed = self.desired_speed
                if target_lead is not None:
                    target_speed = min(self.desired_speed, max(target_lead.vx, 5.0) + 2.0)
                return DecisionOutput(
                    intent=DrivingIntent.MERGE,
                    target_lane_id=target_lane,
                    target_speed=target_speed,
                    lead_obstacle_id=target_lead.id if target_lead else None,
                    reasoning_log=(
                        f"Merge gap accepted: lead={target_lead_gap:.1f}m, "
                        f"follow={target_follow_gap:.1f}m"
                    ),
                )
            # 间隙不安全：留在匝道，按当前车道前车跟驰
            if lead_obs is not None:
                self.current_state = DrivingIntent.FOLLOW
                return DecisionOutput(
                    intent=DrivingIntent.FOLLOW,
                    target_lane_id=current_lane_id,
                    target_speed=min(lead_obs.vx, self.desired_speed),
                    lead_obstacle_id=lead_obs.id,
                    reasoning_log=(
                        f"Hold ramp: target lane gap unsafe "
                        f"(lead={target_lead_gap:.1f}m, follow={target_follow_gap:.1f}m)"
                    ),
                )

        # 3. 跟驰或巡航：用 IDM 期望车距作为触发阈值，给制动留出足够距离
        if lead_obs is not None:
            rel_speed = max(0.0, ego_state.velocity - lead_obs.vx)
            desired_gap = (
                3.0
                + ego_state.velocity * 1.5
                + ego_state.velocity * rel_speed / (2.0 * np.sqrt(2.0 * 2.5))
            )
            follow_trigger = max(20.0, desired_gap)
        else:
            follow_trigger = 0.0
        if lead_obs is not None and lead_gap < follow_trigger:
            self.current_state = DrivingIntent.FOLLOW
            return DecisionOutput(
                intent=DrivingIntent.FOLLOW,
                target_lane_id=current_lane_id,
                target_speed=min(lead_obs.vx, self.desired_speed),
                lead_obstacle_id=lead_obs.id,
                reasoning_log=f"Follow lead {lead_obs.id}: gap={lead_gap:.1f}m",
            )

        self.current_state = DrivingIntent.CRUISE
        return DecisionOutput(
            intent=DrivingIntent.CRUISE,
            target_lane_id=current_lane_id,
            target_speed=self.desired_speed,
            reasoning_log="Lane clear, executing cruise control",
        )
