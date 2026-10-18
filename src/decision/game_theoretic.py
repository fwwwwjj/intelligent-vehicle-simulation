"""
基于博弈论的交互式动态行为决策接口 (Game-Theoretic Interactive Decision Maker)。
针对强交互场景（匝道汇入、交叉口），构建主车与关键交互车辆的非合作博弈收益矩阵。

当前实现采用可解释的二人非零和收益矩阵：
主车策略 {merge/pass, yield} × 背景车策略 {pass, yield}，
对每个组合计算安全、效率、舒适三项效用，再取最坏情况（min）作为稳健收益，
避免“只看自己收益、忽略对方反应”的伪博弈。
"""

from typing import List, Optional, TYPE_CHECKING
import numpy as np

from src.common.state import VehicleState, ObstacleState
from src.decision.base_decision import BaseDecisionMaker, DecisionOutput, DrivingIntent

if TYPE_CHECKING:  # pragma: no cover
    from src.environment.road_network import RoadNetwork


class GameTheoreticDecisionMaker(BaseDecisionMaker):
    """
    博弈交互决策器
    通过评估 (加速汇入, 减速礼让) x (背景车加速通过, 背景车礼让) 的收益矩阵，
    选择稳健收益更高的战术意图。
    """

    def __init__(
        self,
        weight_safety: float = 0.5,
        weight_efficiency: float = 0.3,
        weight_comfort: float = 0.2,
        desired_speed: float = 22.22,
        road_network: Optional["RoadNetwork"] = None,
    ):
        self.w_s = weight_safety
        self.w_e = weight_efficiency
        self.w_c = weight_comfort
        self.desired_speed = desired_speed
        self.road_network = road_network

    # ------------------------------------------------------------------
    # 收益函数
    # ------------------------------------------------------------------
    def calculate_payoff(
        self,
        ego_action: str,
        obs_action: str,
        distance: float,
        delta_v: float,
    ) -> float:
        """
        计算主车在策略组合下的综合效用。

        :param ego_action: 'merge'（加速汇入）或 'yield'（减速礼让）
        :param obs_action: 'pass'（背景车保持/加速）或 'yield'（背景车礼让）
        :param distance: 与交互车的净纵向距离 (m)
        :param delta_v: 主车相对交互车的速度差 (m/s)
        :return: 效用评分
        """
        # 安全项：汇入且对方不礼让，且距离/相对速度危险
        if ego_action == "merge" and obs_action == "pass" and distance < 15.0 and delta_v > -1.0:
            safety = -100.0
        elif distance < 8.0:
            safety = -40.0
        else:
            safety = 10.0

        # 效率项：汇入通行收益更高
        efficiency = 20.0 if ego_action == "merge" else 5.0

        # 舒适项：减速礼让更平顺
        comfort = 8.0 if ego_action == "merge" else 10.0

        # 距离越近、相对速度越大，安全项惩罚越强
        risk = max(0.0, (20.0 - distance)) * max(0.0, delta_v) * 0.1
        return self.w_s * (safety - risk) + self.w_e * efficiency + self.w_c * comfort

    # ------------------------------------------------------------------
    # 场景查询
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
        center_y = self._lane_center_y(lane_id)
        lead = None
        min_gap = float("inf")
        for obs in obstacles:
            dx = obs.x - ego_state.x
            if dx <= 0:
                continue
            lateral = abs(obs.y - center_y) if center_y is not None else abs(obs.y - ego_state.y)
            if lateral > 0.5 * (1.9 + obs.width) + 0.4:
                continue
            gap = dx - 0.5 * (4.8 + obs.length)
            if gap < min_gap:
                min_gap = gap
                lead = obs
        return lead, min_gap

    def _find_follower(
        self, ego_state: VehicleState, obstacles: List[ObstacleState], lane_id: str
    ) -> tuple[Optional[ObstacleState], float]:
        center_y = self._lane_center_y(lane_id)
        follower = None
        min_gap = float("inf")
        for obs in obstacles:
            dx = ego_state.x - obs.x
            if dx <= 0:
                continue
            lateral = abs(obs.y - center_y) if center_y is not None else abs(obs.y - ego_state.y)
            if lateral > 0.5 * (1.9 + obs.width) + 0.4:
                continue
            gap = dx - 0.5 * (4.8 + obs.length)
            if gap < min_gap:
                min_gap = gap
                follower = obs
        return follower, min_gap

    def _target_lane(self, current_lane_id: str) -> Optional[str]:
        if self.road_network is None or "ramp" not in current_lane_id:
            return None
        for candidate in ("main_lane_0", "main_lane_1"):
            if candidate in self.road_network.lanes:
                return candidate
        return None

    # ------------------------------------------------------------------
    # 决策主入口
    # ------------------------------------------------------------------
    def make_decision(
        self,
        ego_state: VehicleState,
        obstacles: List[ObstacleState],
        current_lane_id: str
    ) -> DecisionOutput:
        """构建收益矩阵并选择稳健最优意图。"""
        current_lead, current_gap = self._find_lead(ego_state, obstacles, current_lane_id)
        target_lane = self._target_lane(current_lane_id)

        if target_lane is None:
            # 非汇入场景：用 IDM 期望车距 + TTC 决定巡航或礼让
            if current_lead is not None and current_lead.vx < 1.0 and current_gap < 25.0:
                return DecisionOutput(
                    intent=DrivingIntent.YIELD,
                    target_lane_id=current_lane_id,
                    target_speed=0.0,
                    lead_obstacle_id=current_lead.id,
                    reasoning_log=f"Stopped lead ahead: gap={current_gap:.1f}m; hold standstill",
                )
            if current_lead is not None:
                rel_speed = ego_state.velocity - current_lead.vx
                ttc = current_gap / max(1e-2, rel_speed) if rel_speed > 0 else float("inf")
                desired_gap = (
                    3.0
                    + ego_state.velocity * 1.5
                    + ego_state.velocity * max(0.0, rel_speed) / (2.0 * np.sqrt(2.0 * 2.5))
                )
                if ttc < 3.0 or current_gap < desired_gap:
                    return DecisionOutput(
                        intent=DrivingIntent.YIELD,
                        target_lane_id=current_lane_id,
                        target_speed=min(current_lead.vx, self.desired_speed),
                        lead_obstacle_id=current_lead.id,
                        reasoning_log=f"Yield to lead: gap={current_gap:.1f}m, TTC={ttc:.2f}s",
                    )
            return DecisionOutput(
                intent=DrivingIntent.CRUISE,
                target_lane_id=current_lane_id,
                target_speed=self.desired_speed,
                reasoning_log="No interaction partner; cruise",
            )

        target_lead, target_lead_gap = self._find_lead(ego_state, obstacles, target_lane)
        target_follower, follower_gap = self._find_follower(ego_state, obstacles, target_lane)
        distance = target_lead_gap if target_lead is not None else 100.0
        delta_v = ego_state.velocity - (target_lead.vx if target_lead is not None else ego_state.velocity)
        # follower_closing > 0 表示后车正在逼近（比自车快）
        follower_closing = (
            target_follower.vx - ego_state.velocity if target_follower is not None else 0.0
        )

        # 主车收益：对背景车两种反应取最坏情况
        u_merge = min(
            self.calculate_payoff("merge", "pass", distance, delta_v),
            self.calculate_payoff("merge", "yield", distance, delta_v),
        )
        u_yield = min(
            self.calculate_payoff("yield", "pass", distance, delta_v),
            self.calculate_payoff("yield", "yield", distance, delta_v),
        )

        # 硬安全约束：目标车道后车过近且正在逼近时，不允许汇入
        rear_unsafe = target_follower is not None and (
            (follower_gap < 20.0 and follower_closing > 1.0) or follower_gap < 5.0
        )
        can_merge = (u_merge > u_yield) and not rear_unsafe

        if can_merge:
            target_speed = self.desired_speed
            if target_lead is not None:
                target_speed = min(self.desired_speed, max(target_lead.vx, 5.0) + 2.0)
            return DecisionOutput(
                intent=DrivingIntent.MERGE,
                target_lane_id=target_lane,
                target_speed=target_speed,
                lead_obstacle_id=target_lead.id if target_lead else None,
                reasoning_log=(
                    f"Nash choice=merge (u_merge={u_merge:.1f} > u_yield={u_yield:.1f}); "
                    f"lead_gap={target_lead_gap:.1f}m, follower_gap={follower_gap:.1f}m"
                ),
            )

        reason = "rear unsafe" if rear_unsafe else f"u_yield={u_yield:.1f} >= u_merge={u_merge:.1f}"
        target_speed = self.desired_speed
        if current_lead is not None:
            target_speed = min(current_lead.vx, self.desired_speed)
        return DecisionOutput(
            intent=DrivingIntent.YIELD,
            target_lane_id=current_lane_id,
            target_speed=target_speed,
            lead_obstacle_id=current_lead.id if current_lead else None,
            reasoning_log=f"Nash choice=yield ({reason})",
        )
