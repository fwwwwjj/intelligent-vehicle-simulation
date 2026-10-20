"""
基于贝叶斯动态博弈论的交互式行为决策器 (Bayesian Game-Theoretic Decision Maker)。
针对强交互场景（匝道汇入、十字路口、切入避让），构建主车与关键交互车辆的动态非合作博弈模型。

核心算法机制：
1. 连续动作空间速度协同博弈 (Speed-Negotiating Game)：
   扩展为带前插加速 (Accel-Merge)、协同并道 (Coop-Merge) 与后插礼让 (Decel-Yield) 的连续速度动作集；
2. 对手意图在线贝叶斯推断 (Online Intent Inference)：
   根据背景交互车辆历史加速度与速度趋势，在线递推估计对手礼让置信度 P(yield) in [0.05, 0.95]；
3. 贝叶斯期望效用最大化 (Bayesian Expected Utility Maximization)：
   综合安全风险、通行效率、控制能耗与平顺性多目标，选择期望收益最高的战术意图；
4. 软化硬边界拦截，保留博弈连续性，同时以物理净距为底线保障零碰撞。
"""

from typing import List, Optional, Dict, TYPE_CHECKING
import numpy as np

from src.common.state import VehicleState, ObstacleState
from src.decision.base_decision import BaseDecisionMaker, DecisionOutput, DrivingIntent

if TYPE_CHECKING:  # pragma: no cover
    from src.environment.road_network import RoadNetwork


class GameTheoreticDecisionMaker(BaseDecisionMaker):
    """
    贝叶斯动态博弈交互决策器
    """

    def __init__(
        self,
        weight_safety: float = 0.5,
        weight_efficiency: float = 0.3,
        weight_comfort: float = 0.2,
        weight_effort: float = 0.1,
        desired_speed: float = 22.22,
        road_network: Optional["RoadNetwork"] = None,
        ego_length: float = 4.8,
        ego_width: float = 1.9,
    ):
        self.w_s = float(weight_safety)
        self.w_e = float(weight_efficiency)
        self.w_c = float(weight_comfort)
        self.w_effort = float(weight_effort)
        self.desired_speed = float(desired_speed)
        self.road_network = road_network
        self.ego_length = float(ego_length)
        self.ego_width = float(ego_width)

        # 对手在线历史跟踪：obs_id -> list of float (最近若干帧的历史纵向速度)
        self._obs_history: Dict[str, List[float]] = {}
        # 对手礼让信念概率：obs_id -> P(yield) in [0.05, 0.95]
        self._yield_beliefs: Dict[str, float] = {}

    # ------------------------------------------------------------------
    # 在线对手意图推断 (Bayesian Intent Inference)
    # ------------------------------------------------------------------
    def estimate_opponent_intent(self, obs: ObstacleState) -> float:
        """
        基于背景车历史时序速度与加速度趋势推断其礼让置信度 P(yield)。

        :param obs: 交互背景车状态
        :return: 估计出的礼让概率 P(yield) in [0.05, 0.95]
        """
        history = self._obs_history.setdefault(obs.id, [])
        history.append(float(obs.vx))
        if len(history) > 6:
            history.pop(0)

        # 初始中立先验 P_0(yield) = 0.5
        prior = self._yield_beliefs.get(obs.id, 0.5)

        if len(history) < 2:
            self._yield_beliefs[obs.id] = prior
            return prior

        # 估算最近窗口内的平均纵向加速度趋势 (假设仿真步长约 0.05~0.1s)
        delta_v = history[-1] - history[0]
        dt_span = max(0.05 * (len(history) - 1), 0.05)
        approx_accel = delta_v / dt_span

        # 若背景车有减速迹象 (accel < -0.3 m/s^2)，说明其展现礼让倾向
        if approx_accel < -0.3:
            likelihood_yield = 0.85
        elif approx_accel > 0.3:
            # 持续加速冲抢，判定为不礼让 (pass)
            likelihood_yield = 0.15
        else:
            likelihood_yield = 0.50

        # 贝叶斯时序递推融合
        posterior = 0.65 * prior + 0.35 * likelihood_yield
        posterior = float(np.clip(posterior, 0.05, 0.95))
        self._yield_beliefs[obs.id] = posterior
        return posterior

    # ------------------------------------------------------------------
    # 细化多目标综合效用函数
    # ------------------------------------------------------------------
    def calculate_payoff(
        self,
        ego_action: str,
        obs_action: str,
        lead_gap: float,
        follower_gap: float,
        delta_v_lead: float,
        delta_v_foll: float,
        accel_effort: float = 0.0,
    ) -> float:
        """
        计算主车在特定联合动作 (ego_action x obs_action) 下的综合多目标效用。

        :param ego_action: 'accel_merge', 'coop_merge', 'decel_yield'
        :param obs_action: 'pass' (背景车强行通过) 或 'yield' (背景车减速礼让)
        :param lead_gap: 目标车道前导车净距 (m)
        :param follower_gap: 目标车道后随车净距 (m)
        :param delta_v_lead: 主车相对于前车的速度差 (v_ego - v_lead)
        :param delta_v_foll: 后随车相对于主车的接近速度 (v_foll - v_ego)
        :param accel_effort: 动作所要求的控制加速度幅度 (m/s^2)
        :return: 效用评分
        """
        # 1. 安全效用 (Safety Utility)
        if ego_action.endswith("merge"):
            # 与前导车的碰撞风险
            risk_lead = max(0.0, 15.0 - lead_gap) * max(0.0, delta_v_lead) * 0.2
            if lead_gap < 5.0:
                risk_lead += 60.0

            # 与后随车的碰撞风险
            if obs_action == "pass":
                # 对方不礼让：若后车正在快速逼近且距离近，施加连续二次惩罚
                if delta_v_foll > 0.0:
                    risk_foll = max(0.0, 20.0 - follower_gap) * delta_v_foll * 0.4
                else:
                    risk_foll = max(0.0, 8.0 - follower_gap) * 0.5
                if follower_gap < 5.0:
                    risk_foll += 80.0
            else:
                # 对方礼让：后车主动拉大车距，后向风险大幅降低
                risk_foll = max(0.0, 6.0 - follower_gap) * 0.3

            safety = 20.0 - (risk_lead + risk_foll)
        else:
            # decel_yield: 主动减速礼让，后车碰撞风险归零，仅受当前车道前车影响
            risk_current = max(0.0, 10.0 - lead_gap) * 0.5
            safety = 15.0 - risk_current

        # 2. 通行效率 (Efficiency Utility)
        if ego_action == "accel_merge":
            efficiency = 28.0
        elif ego_action == "coop_merge":
            efficiency = 20.0
        else:
            efficiency = 5.0

        # 3. 驾驶平顺度与控制能耗 (Comfort & Effort Utility)
        if ego_action == "coop_merge":
            comfort = 10.0
        elif ego_action == "decel_yield":
            comfort = 8.0
        else:
            comfort = 6.0

        effort_penalty = self.w_effort * (accel_effort ** 2)

        return (
            self.w_s * safety
            + self.w_e * efficiency
            + self.w_c * comfort
            - effort_penalty
        )

    # ------------------------------------------------------------------
    # 场景拓扑与障碍物查询
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
            if dx <= 0.0:
                continue
            lateral = abs(obs.y - center_y) if center_y is not None else abs(obs.y - ego_state.y)
            if lateral > 0.5 * (self.ego_width + obs.width) + 0.4:
                continue
            gap = dx - 0.5 * (self.ego_length + obs.length)
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
            if dx <= 0.0:
                continue
            lateral = abs(obs.y - center_y) if center_y is not None else abs(obs.y - ego_state.y)
            if lateral > 0.5 * (self.ego_width + obs.width) + 0.4:
                continue
            gap = dx - 0.5 * (self.ego_length + obs.length)
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
        """
        执行基于贝叶斯期望效用的动态博弈决策流程。
        """
        current_lead, current_gap = self._find_lead(ego_state, obstacles, current_lane_id)
        target_lane = self._target_lane(current_lane_id)

        # --------------------------------------------------------------
        # 1. 非汇入场景（直线巡航、前方静止避让、切入避让）
        # --------------------------------------------------------------
        if target_lane is None:
            # 针对前方静止车辆：平稳降速至停车保持
            if current_lead is not None and current_lead.vx < 1.0 and current_gap < 30.0:
                return DecisionOutput(
                    intent=DrivingIntent.YIELD,
                    target_lane_id=current_lane_id,
                    target_speed=0.0,
                    lead_obstacle_id=current_lead.id,
                    reasoning_log=f"Stationary obstacle ahead: gap={current_gap:.1f}m; smooth stop",
                )

            # 针对正常前导车：动态跟驰与时变安全时距保持
            if current_lead is not None:
                rel_speed = ego_state.velocity - current_lead.vx
                ttc = current_gap / max(1e-2, rel_speed) if rel_speed > 0 else float("inf")
                desired_gap = (
                    3.5
                    + ego_state.velocity * 1.5
                    + ego_state.velocity * max(0.0, rel_speed) / (2.0 * np.sqrt(2.0 * 2.5))
                )
                if ttc < 3.2 or current_gap < desired_gap:
                    # 动态协同调速
                    target_speed = min(max(current_lead.vx, 0.0), self.desired_speed)
                    return DecisionOutput(
                        intent=DrivingIntent.FOLLOW,
                        target_lane_id=current_lane_id,
                        target_speed=target_speed,
                        lead_obstacle_id=current_lead.id,
                        reasoning_log=f"Dynamic follow: gap={current_gap:.1f}m, TTC={ttc:.2f}s",
                    )

            # 自由流巡航
            return DecisionOutput(
                intent=DrivingIntent.CRUISE,
                target_lane_id=current_lane_id,
                target_speed=self.desired_speed,
                reasoning_log="Free flow cruise",
            )

        # --------------------------------------------------------------
        # 2. 强交互汇入场景：连续动作空间速度协同博弈
        # --------------------------------------------------------------
        target_lead, target_lead_gap = self._find_lead(ego_state, obstacles, target_lane)
        target_follower, follower_gap = self._find_follower(ego_state, obstacles, target_lane)

        # 在线估计后随车的礼让概率 P(yield)
        p_yield = self.estimate_opponent_intent(target_follower) if target_follower else 0.5
        p_pass = 1.0 - p_yield

        # 零碰撞绝对底线约束（物理净距小于 3.5m 且后车在逼近时强制禁止变道）
        critical_rear_danger = (
            target_follower is not None
            and follower_gap < 3.5
            and (target_follower.vx - ego_state.velocity) > -0.5
        )

        lead_gap_eval = target_lead_gap if target_lead is not None else 100.0
        follow_gap_eval = follower_gap if target_follower is not None else 100.0

        # 策略 1: ACCEL_MERGE (主动加速冲前间隙汇入)
        lead_v = target_lead.vx if target_lead is not None else self.desired_speed
        target_v_accel = min(self.desired_speed, max(ego_state.velocity + 1.8, lead_v + 1.5))
        effort_accel = max(0.0, target_v_accel - ego_state.velocity)
        delta_v_lead_1 = target_v_accel - lead_v
        delta_v_foll_1 = (target_follower.vx - target_v_accel) if target_follower else 0.0

        u_accel_yield = self.calculate_payoff("accel_merge", "yield", lead_gap_eval, follow_gap_eval, delta_v_lead_1, delta_v_foll_1, effort_accel)
        u_accel_pass = self.calculate_payoff("accel_merge", "pass", lead_gap_eval, follow_gap_eval, delta_v_lead_1, delta_v_foll_1, effort_accel)
        e_u_accel = p_yield * u_accel_yield + p_pass * u_accel_pass

        # 策略 2: COOP_MERGE (协同匀速平稳汇入)
        target_v_coop = min(self.desired_speed, max(lead_v, 8.0))
        effort_coop = abs(target_v_coop - ego_state.velocity)
        delta_v_lead_2 = target_v_coop - lead_v
        delta_v_foll_2 = (target_follower.vx - target_v_coop) if target_follower else 0.0

        u_coop_yield = self.calculate_payoff("coop_merge", "yield", lead_gap_eval, follow_gap_eval, delta_v_lead_2, delta_v_foll_2, effort_coop)
        u_coop_pass = self.calculate_payoff("coop_merge", "pass", lead_gap_eval, follow_gap_eval, delta_v_lead_2, delta_v_foll_2, effort_coop)
        e_u_coop = p_yield * u_coop_yield + p_pass * u_coop_pass

        # 策略 3: DECEL_YIELD (主动减速后插礼让)
        foll_v = target_follower.vx if target_follower is not None else self.desired_speed
        target_v_yield = max(5.0, min(self.desired_speed, foll_v - 2.0))
        if current_lead is not None:
            target_v_yield = min(target_v_yield, current_lead.vx)
        effort_yield = max(0.0, ego_state.velocity - target_v_yield)

        u_yield_yield = self.calculate_payoff("decel_yield", "yield", lead_gap_eval, follow_gap_eval, 0.0, 0.0, effort_yield)
        u_yield_pass = self.calculate_payoff("decel_yield", "pass", lead_gap_eval, follow_gap_eval, 0.0, 0.0, effort_yield)
        e_u_yield = p_yield * u_yield_yield + p_pass * u_yield_pass

        # 贝叶斯期望效用最大化决策选择
        if critical_rear_danger:
            selected_action = "decel_yield"
            selected_speed = target_v_yield
            best_payoff = e_u_yield
        else:
            candidates = [
                ("accel_merge", e_u_accel, target_v_accel),
                ("coop_merge", e_u_coop, target_v_coop),
                ("decel_yield", e_u_yield, target_v_yield),
            ]
            selected_action, best_payoff, selected_speed = max(candidates, key=lambda item: item[1])

        if selected_action.endswith("merge"):
            return DecisionOutput(
                intent=DrivingIntent.MERGE,
                target_lane_id=target_lane,
                target_speed=selected_speed,
                lead_obstacle_id=target_lead.id if target_lead else None,
                reasoning_log=(
                    f"Bayesian Game: {selected_action.upper()} (E[U]={best_payoff:.1f}); "
                    f"P(yield)={p_yield:.2f}; lead_gap={lead_gap_eval:.1f}m, follow_gap={follow_gap_eval:.1f}m"
                ),
            )
        else:
            return DecisionOutput(
                intent=DrivingIntent.YIELD,
                target_lane_id=current_lane_id,
                target_speed=selected_speed,
                lead_obstacle_id=current_lead.id if current_lead else None,
                reasoning_log=(
                    f"Bayesian Game: DECEL_YIELD (critical_rear={critical_rear_danger}); "
                    f"P(yield)={p_yield:.2f}; E[U_yield]={e_u_yield:.1f} vs E[U_merge]={max(e_u_accel, e_u_coop):.1f}"
                ),
            )
