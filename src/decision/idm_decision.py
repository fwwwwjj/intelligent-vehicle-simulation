"""
经典 IDM 自适应巡航 + AEB 决策基线 (Intelligent Driver Model Decision Maker)。

该模块实现基线/消融实验中的“无战术决策”组合：只根据本车道最近前导车执行
IDM (Intelligent Driver Model) 纵向跟驰/定速巡航，并叠加经典 TTC 自动紧急制动
(AEB) 安全层；**不做换道、汇入或博弈**。配合
:class:`~src.planning.centerline_planner.CenterlinePlanner` 与
:class:`~src.control.pure_pursuit_controller.PurePursuitController`，构成
“经典 ACC + 车道保持”基线栈，用于量化战术决策层与优化规划层的贡献。

设计要点：
1. 前导车筛选沿用 FSM 决策器的车道中心线横向容差判据，保证与主系统公平对比；
2. 纵向加速度直接复用环境模块中的 :class:`IntelligentDriverModel`，避免
   基线与交通流模型使用两套不一致的 IDM 参数；
3. 目标速度取 ``clip(v + a * speed_command_horizon, 0, desired_speed)``，
   把 IDM 期望加速度换算为可被下游规划器跟踪的速度指令（默认 1.0 s，
   与 5 s 规划时域下的速度跟踪相容）；
4. 与 FSM 相同的 TTC/减速度安全层：IDM 期望减速度过大或 TTC 过低时直接
   输出 ``EMERGENCY_STOP`` 与零目标速度，避免经典 ACC 因缺少 AEB 而失效。
"""

from typing import TYPE_CHECKING, List, Optional

import numpy as np

from src.common.state import VehicleState, ObstacleState
from src.decision.base_decision import BaseDecisionMaker, DecisionOutput, DrivingIntent
from src.environment.obstacle_manager import IntelligentDriverModel

if TYPE_CHECKING:  # pragma: no cover - 仅用于类型检查，避免循环导入
    from src.environment.road_network import RoadNetwork


class IDMDecisionMaker(BaseDecisionMaker):
    """
    经典 IDM 跟驰/巡航 + AEB 行为决策器（无换道）。

    该决策器始终把 ``target_lane_id`` 设为当前车道，因此是一个纯粹的
    “纵向 ACC + 横向车道保持”基线，不会产生任何战术性换道意图。
    """

    #: AEB 触发阈值：IDM 期望减速度低于该值即进入紧急制动 (m/s^2)
    AEB_DECEL_THRESHOLD = -3.0
    #: AEB 触发阈值：TTC 低于该值即进入紧急制动 (s)
    AEB_TTC_THRESHOLD = 1.5

    def __init__(
        self,
        desired_speed: float = 22.22,
        time_headway: float = 1.5,
        min_gap: float = 3.0,
        a_max: float = 3.5,
        b_desired: float = 2.5,
        ego_length: float = 4.8,
        ego_width: float = 1.9,
        road_network: Optional["RoadNetwork"] = None,
        speed_command_horizon: float = 1.0,
    ):
        self.desired_speed = float(desired_speed)
        self.time_headway = float(time_headway)
        self.min_gap = float(min_gap)
        self.a_max = float(a_max)
        self.b_desired = float(b_desired)
        self.ego_length = float(ego_length)
        self.ego_width = float(ego_width)
        self.road_network = road_network
        # IDM 加速度 -> 目标速度指令的时间尺度。1.0 s 与 5 s 规划时域下的
        # 速度跟踪相匹配；过小（如 0.1 s）会让下游几乎观察不到制动意图。
        self.speed_command_horizon = float(speed_command_horizon)

        # 与环境交通流模型共用 IDM 实现，保证基线与场景车辆行为一致。
        self.idm = IntelligentDriverModel(
            v0=self.desired_speed,
            s0=self.min_gap,
            T=self.time_headway,
            a_max=self.a_max,
            b_desired=self.b_desired,
        )

    # ------------------------------------------------------------------
    # 车道级感知
    # ------------------------------------------------------------------
    def _lane_center_y(self, lane_id: str) -> Optional[float]:
        """查询车道中心线的名义 y 坐标；道路网络缺失/查询失败时返回 None。"""
        if self.road_network is None:
            return None
        try:
            center_y = self.road_network.get_lane_center_y(lane_id)
        except Exception:  # pragma: no cover - 防御自定义道路网络实现
            return None
        if center_y is None:
            return None
        return float(center_y)

    def _find_lead(
        self,
        ego_state: VehicleState,
        obstacles: List[ObstacleState],
        lane_id: str,
    ) -> tuple[Optional[ObstacleState], float]:
        """
        在目标车道正前方筛选最近前导车。

        判据与 FSM 决策器保持一致：
        - 横向：相对车道中心（无中心线信息时退化为自车 y）的偏差不超过
          ``0.5 * (ego_width + obs.width) + 0.4``；
        - 纵向：``dx > 0``，并取净车距 ``gap = dx - 0.5 * (ego_length + obs.length)``
          最小者作为最近前导车。
        """
        center_y = self._lane_center_y(lane_id)
        reference_y = ego_state.y if center_y is None else center_y

        closest_obs: Optional[ObstacleState] = None
        min_gap = float("inf")
        for obs in obstacles:
            dx = obs.x - ego_state.x
            if dx <= 0.0:
                continue
            lateral = abs(obs.y - reference_y)
            if lateral > 0.5 * (self.ego_width + obs.width) + 0.4:
                continue
            gap = dx - 0.5 * (self.ego_length + obs.length)
            if gap < min_gap:
                min_gap = gap
                closest_obs = obs
        return closest_obs, min_gap

    # ------------------------------------------------------------------
    # 决策入口
    # ------------------------------------------------------------------
    def make_decision(
        self,
        ego_state: VehicleState,
        obstacles: List[ObstacleState],
        current_lane_id: str,
    ) -> DecisionOutput:
        """
        IDM 纵向决策：本车道有前导车则跟驰/紧急制动，否则定速巡航。

        始终保持在 ``current_lane_id``，不产生任何换道/汇入意图。
        """
        lead_obs, lead_gap = self._find_lead(ego_state, obstacles, current_lane_id)
        current_speed = max(0.0, float(ego_state.velocity))

        if lead_obs is None:
            # 无前导车：IDM 自由流项会收敛到期望速度，这里直接输出期望巡航速度。
            return DecisionOutput(
                intent=DrivingIntent.CRUISE,
                target_lane_id=current_lane_id,
                target_speed=self.desired_speed,
                time_headway=self.time_headway,
                lead_obstacle_id=None,
                lateral_shift_l=0.0,
                reasoning_log=(
                    "IDM cruise: no lead vehicle in lane "
                    f"'{current_lane_id}', target_speed={self.desired_speed:.2f} m/s"
                ),
            )

        lead_speed = max(0.0, float(lead_obs.vx))
        accel = float(
            self.idm.compute_acceleration(
                current_speed=current_speed,
                lead_speed=lead_speed,
                distance=max(0.1, lead_gap),
            )
        )

        # 经典 TTC/AEB 安全层：与 FSM 的紧急制动判据一致，只在“确实存在
        # 追尾风险”时触发，不作为普通跟驰的替代。
        relative_speed = current_speed - lead_speed
        ttc = (
            lead_gap / max(1e-2, relative_speed)
            if relative_speed > 0.0
            else float("inf")
        )
        if accel < self.AEB_DECEL_THRESHOLD or ttc < self.AEB_TTC_THRESHOLD:
            return DecisionOutput(
                intent=DrivingIntent.EMERGENCY_STOP,
                target_lane_id=current_lane_id,
                target_speed=0.0,
                time_headway=self.time_headway,
                lead_obstacle_id=lead_obs.id,
                lateral_shift_l=0.0,
                reasoning_log=(
                    f"AEB override lead {lead_obs.id}: gap={lead_gap:.2f} m, "
                    f"ego_v={current_speed:.2f} m/s, lead_v={lead_speed:.2f} m/s, "
                    f"a_idm={accel:.2f} m/s^2, TTC={ttc:.2f} s "
                    f"-> target_speed=0.0 (no lane change)"
                ),
            )

        target_speed = float(
            np.clip(
                current_speed + accel * self.speed_command_horizon,
                0.0,
                self.desired_speed,
            )
        )

        return DecisionOutput(
            intent=DrivingIntent.FOLLOW,
            target_lane_id=current_lane_id,
            target_speed=target_speed,
            time_headway=self.time_headway,
            lead_obstacle_id=lead_obs.id,
            lateral_shift_l=0.0,
            reasoning_log=(
                f"IDM follow lead {lead_obs.id}: gap={lead_gap:.2f} m, "
                f"ego_v={current_speed:.2f} m/s, lead_v={lead_speed:.2f} m/s, "
                f"a_idm={accel:.2f} m/s^2, "
                f"target_speed={target_speed:.2f} m/s "
                f"(speed_command_horizon={self.speed_command_horizon:.2f} s, "
                f"no lane change)"
            ),
        )


__all__ = ["IDMDecisionMaker"]
