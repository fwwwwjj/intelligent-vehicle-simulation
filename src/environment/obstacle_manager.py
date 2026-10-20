"""
动态交通参与者管理与时空轨迹预测接口。
支持微观交通流行为演化模型：
1. 智能驾驶员纵向跟驰模型 (IDM, Intelligent Driver Model) —— 已接入 step()；
2. 协同换道模型 (MOBIL, Minimizing Overall Braking Induced by Lane change) ——
   可选状态机，场景可控制是否启用。

当前 ``step()`` 对注册了 IDM 的车辆做跟驰更新，其余车辆保持匀速。预测轨迹使用
最近一次计算的纵向加速度作恒加速度外推，并在预测速度到零时截断位置。
"""

from typing import List, Dict, Optional, Any
import numpy as np
from src.common.state import ObstacleState, ObstacleType, TrajectoryPoint


class IntelligentDriverModel:
    """
    智能驾驶员跟驰模型 (IDM, Intelligent Driver Model)
    计算交通参与者在特定车距与相对速度下的自然纵向加速度响应
    """
    def __init__(self, v0: float = 20.0, s0: float = 3.0, T: float = 1.5, a_max: float = 2.0, b_desired: float = 2.5):
        self.v0 = v0                  # 期望巡航速度 (m/s)
        self.s0 = s0                  # 静止安全车距 (m)
        self.T = T                    # 安全车头时距 (s)
        self.a_max = a_max            # 最大舒适加速度 (m/s^2)
        self.b_desired = b_desired    # 期望舒适减速度 (m/s^2)

    def compute_acceleration(self, current_speed: float, lead_speed: float, distance: float) -> float:
        """
        根据当前车辆与前车的相对状态计算 IDM 期望加速度
        :param current_speed: 本车速度 (m/s)
        :param lead_speed: 前车速度 (m/s)
        :param distance: 实际净车距 (m)
        :return: 加速度 (m/s^2)
        """
        delta_v = current_speed - lead_speed
        # 期望动态车距公式 s*(v, delta_v) = s0 + v*T + (v * delta_v) / (2 * sqrt(a*b))
        s_star = self.s0 + max(0.0, current_speed * self.T + (current_speed * delta_v) / (2.0 * np.sqrt(self.a_max * self.b_desired)))
        if distance <= 0.1:
            return -self.b_desired * 2.5

        accel = self.a_max * (1.0 - (current_speed / max(1e-3, self.v0))**4 - (s_star / distance)**2)
        return float(np.clip(accel, -7.0, self.a_max))


class MOBILLaneChangeModel:
    """
    最小化总体制动换道模型 (MOBIL, Minimizing Overall Braking Induced by Lane change)
    综合考量换道自我激励 (Incentive) 与对他车影响的利他系数 (Politeness Factor)。
    """
    def __init__(self, politeness: float = 0.5, accel_threshold: float = 0.2, b_safe: float = 4.0):
        self.p = politeness                   # 利他系数 p in [0, 1]
        self.a_th = accel_threshold           # 换道加速度增益阈值 (m/s^2)
        self.b_safe = b_safe                 # 目标车道后车安全制动减速度上限 (m/s^2)

    def check_lane_change(
        self,
        accel_self_cur: float,
        accel_self_target: float,
        accel_new_follower_before: float,
        accel_new_follower_after: float,
        accel_old_follower_before: float,
        accel_old_follower_after: float
    ) -> bool:
        """
        MOBIL 换道准则判定
        """
        # 1. 安全准则 (Safety Criterion): 不能造成目标车道后车急刹超出安全阈值
        if accel_new_follower_after < -self.b_safe:
            return False

        # 2. 激励准则 (Incentive Criterion):
        # (a'_c - a_c) + p * ((a'_n - a_n) + (a'_o - a_o)) > Delta_a_th
        self_gain = accel_self_target - accel_self_cur
        followers_gain = (accel_new_follower_after - accel_new_follower_before) + \
                         (accel_old_follower_after - accel_old_follower_before)

        total_gain = self_gain + self.p * followers_gain
        return bool(total_gain > self.a_th)


class ObstacleManager:
    """
    仿真环境障碍物时空状态演化管理器
    负责多障碍物的步进模拟、轨迹预测生成与查询。
    支持 IDM 跟驰与 MOBIL 协同换道模型。
    """
    def __init__(self, enable_mobil: bool = False, lane_width: float = 3.75):
        self.obstacles: Dict[str, ObstacleState] = {}
        self.idm_models: Dict[str, IntelligentDriverModel] = {}
        self._last_accelerations: Dict[str, float] = {}
        self.mobil_model = MOBILLaneChangeModel()
        self.enable_mobil: bool = enable_mobil
        self.lane_width: float = lane_width
        self._lane_change_states: Dict[str, Dict[str, Any]] = {}
        self._scheduled_lane_changes: Dict[str, Dict[str, float]] = {}
        self.current_time: float = 0.0

    def add_obstacle(self, obstacle: ObstacleState, idm: Optional[IntelligentDriverModel] = None) -> None:
        """注册障碍物"""
        self.obstacles[obstacle.id] = obstacle
        if idm is not None:
            self.idm_models[obstacle.id] = idm

    def schedule_lane_change(
        self, obstacle_id: str, target_y: float, start_time: float, duration: float
    ) -> None:
        """Schedule a smooth, deterministic lateral maneuver for a scripted scenario."""
        if obstacle_id not in self.obstacles:
            raise KeyError(f"Unknown obstacle: {obstacle_id}")
        if duration <= 0.0 or start_time < 0.0:
            raise ValueError("start_time must be nonnegative and duration must be positive")
        self._scheduled_lane_changes[obstacle_id] = {
            "target_y": float(target_y),
            "start_time": float(start_time),
            "duration": float(duration),
        }

    def _find_lead_at_y(self, obstacle: ObstacleState, check_y: float) -> Optional[ObstacleState]:
        """在指定横向车道线正前方寻找最近前车。"""
        lead = None
        min_gap = float("inf")
        for other in self.obstacles.values():
            if other.id == obstacle.id:
                continue
            if other.x <= obstacle.x:
                continue
            if abs(other.y - check_y) > 0.5 * (obstacle.width + other.width) + 0.5:
                continue
            gap = other.x - obstacle.x - 0.5 * (obstacle.length + other.length)
            if gap < min_gap:
                min_gap = gap
                lead = other
        return lead

    def _find_follower_at_y(self, obstacle: ObstacleState, check_y: float) -> Optional[ObstacleState]:
        """在指定横向车道线后方寻找最近后车。"""
        follower = None
        min_gap = float("inf")
        for other in self.obstacles.values():
            if other.id == obstacle.id:
                continue
            if other.x >= obstacle.x:
                continue
            if abs(other.y - check_y) > 0.5 * (obstacle.width + other.width) + 0.5:
                continue
            gap = obstacle.x - other.x - 0.5 * (obstacle.length + other.length)
            if gap < min_gap:
                min_gap = gap
                follower = other
        return follower

    def _find_lead(self, obstacle: ObstacleState) -> Optional[ObstacleState]:
        """在当前障碍物同车道正前方寻找最近前车。"""
        return self._find_lead_at_y(obstacle, obstacle.y)

    def _compute_accel_for_pair(
        self,
        follower_id: str,
        follower_speed: float,
        lead_speed: Optional[float],
        gap: Optional[float],
    ) -> float:
        idm = self.idm_models.get(follower_id)
        if idm is None:
            return 0.0
        if lead_speed is None or gap is None:
            free = idm.a_max * (1.0 - (follower_speed / max(1e-3, idm.v0)) ** 4)
            return float(np.clip(free, -7.0, idm.a_max))
        return idm.compute_acceleration(
            current_speed=follower_speed,
            lead_speed=max(0.0, lead_speed),
            distance=max(0.1, gap),
        )

    def _evaluate_mobil_lane_change(self, obs_id: str, obs: ObstacleState, current_accel: float) -> Optional[float]:
        """评估障碍物是否满足 MOBIL 换道条件，返回目标车道 y 坐标或 None。"""
        idm = self.idm_models.get(obs_id)
        if idm is None:
            return None

        # 优先考虑左右车道（限制在主线车道范围内，避免背景车违规向匝道换道）
        candidate_ys = [y for y in (obs.y + self.lane_width, obs.y - self.lane_width) if y >= -1e-3]
        for target_y in candidate_ys:
            # 空间几何间距初检：防止目标车道并排侧撞或侵入盲区安全间距
            has_side_collision = any(
                other.id != obs.id
                and abs(other.y - target_y) <= 0.5 * (obs.width + other.width) + 0.5
                and abs(other.x - obs.x) < 0.5 * (obs.length + other.length) + idm.s0
                for other in self.obstacles.values()
            )
            if has_side_collision:
                continue

            target_lead = self._find_lead_at_y(obs, target_y)
            target_follower = self._find_follower_at_y(obs, target_y)

            # 空间几何间距初检
            if target_lead is not None:
                lead_gap = target_lead.x - obs.x - 0.5 * (obs.length + target_lead.length)
                if lead_gap < idm.s0:
                    continue
            if target_follower is not None:
                foll_gap = obs.x - target_follower.x - 0.5 * (obs.length + target_follower.length)
                if foll_gap < idm.s0:
                    continue

            # 1. 目标车道自车预期加速度
            if target_lead is not None:
                accel_self_target = idm.compute_acceleration(
                    current_speed=obs.vx,
                    lead_speed=max(0.0, target_lead.vx),
                    distance=max(0.1, target_lead.x - obs.x - 0.5 * (obs.length + target_lead.length)),
                )
            else:
                accel_self_target = idm.a_max * (1.0 - (obs.vx / max(1e-3, idm.v0)) ** 4)

            # 2. 目标车道后车前后加速度
            if target_follower is not None and target_follower.id in self.idm_models:
                foll_idm = self.idm_models[target_follower.id]
                foll_cur_lead = target_lead  # 原前车即 target_lead
                foll_cur_gap = (
                    target_lead.x - target_follower.x - 0.5 * (target_follower.length + target_lead.length)
                    if target_lead is not None else None
                )
                accel_new_follower_before = self._compute_accel_for_pair(
                    target_follower.id, target_follower.vx,
                    target_lead.vx if target_lead else None, foll_cur_gap
                )
                new_gap = obs.x - target_follower.x - 0.5 * (obs.length + target_follower.length)
                accel_new_follower_after = foll_idm.compute_acceleration(
                    current_speed=target_follower.vx,
                    lead_speed=max(0.0, obs.vx),
                    distance=max(0.1, new_gap),
                )
            else:
                accel_new_follower_before = 0.0
                accel_new_follower_after = 0.0

            # 3. 原车道后车前后加速度
            cur_follower = self._find_follower_at_y(obs, obs.y)
            cur_lead = self._find_lead(obs)
            if cur_follower is not None and cur_follower.id in self.idm_models:
                foll_idm = self.idm_models[cur_follower.id]
                gap_before = obs.x - cur_follower.x - 0.5 * (cur_follower.length + obs.length)
                accel_old_follower_before = foll_idm.compute_acceleration(
                    current_speed=cur_follower.vx,
                    lead_speed=max(0.0, obs.vx),
                    distance=max(0.1, gap_before),
                )
                if cur_lead is not None:
                    gap_after = cur_lead.x - cur_follower.x - 0.5 * (cur_follower.length + cur_lead.length)
                    accel_old_follower_after = foll_idm.compute_acceleration(
                        current_speed=cur_follower.vx,
                        lead_speed=max(0.0, cur_lead.vx),
                        distance=max(0.1, gap_after),
                    )
                else:
                    accel_old_follower_after = foll_idm.a_max * (1.0 - (cur_follower.vx / max(1e-3, foll_idm.v0)) ** 4)
            else:
                accel_old_follower_before = 0.0
                accel_old_follower_after = 0.0

            if self.mobil_model.check_lane_change(
                accel_self_cur=current_accel,
                accel_self_target=accel_self_target,
                accel_new_follower_before=accel_new_follower_before,
                accel_new_follower_after=accel_new_follower_after,
                accel_old_follower_before=accel_old_follower_before,
                accel_old_follower_after=accel_old_follower_after,
            ):
                return target_y

        return None

    def step(self, dt: float) -> None:
        """
        环境物理演化一个时步 dt。
        注册了 IDM 的交通参与者按跟驰模型更新纵向速度；若启用 MOBIL 则评估换道；其余参与者保持匀速。
        """
        # 1. 计算纵向加速度
        accelerations: Dict[str, float] = {}
        for obs_id, obs in self.obstacles.items():
            idm = self.idm_models.get(obs_id)
            if idm is None:
                accelerations[obs_id] = 0.0
                continue
            lead = self._find_lead(obs)
            if lead is None:
                free_accel = idm.a_max * (1.0 - (obs.vx / max(1e-3, idm.v0)) ** 4)
                accelerations[obs_id] = float(np.clip(free_accel, -7.0, idm.a_max))
            else:
                gap = lead.x - obs.x - 0.5 * (obs.length + lead.length)
                accelerations[obs_id] = idm.compute_acceleration(
                    current_speed=obs.vx,
                    lead_speed=max(0.0, lead.vx),
                    distance=max(0.1, gap),
                )

        # 2. MOBIL 换道状态机更新
        if self.enable_mobil:
            for obs_id, obs in self.obstacles.items():
                if obs_id in self._lane_change_states or obs_id in self._scheduled_lane_changes:
                    continue
                accel = accelerations.get(obs_id, 0.0)
                target_y = self._evaluate_mobil_lane_change(obs_id, obs, accel)
                if target_y is not None:
                    self._lane_change_states[obs_id] = {
                        "target_y": target_y,
                        "start_y": obs.y,
                        "duration": 2.5,
                        "elapsed": 0.0,
                    }

        # Deterministic scenario maneuvers are independent of the MOBIL toggle.
        for obs_id, schedule in list(self._scheduled_lane_changes.items()):
            if obs_id in self._lane_change_states or self.current_time + 1e-12 < schedule["start_time"]:
                continue
            obs = self.obstacles[obs_id]
            self._lane_change_states[obs_id] = {
                "target_y": schedule["target_y"],
                "start_y": obs.y,
                "duration": schedule["duration"],
                "elapsed": 0.0,
            }
            del self._scheduled_lane_changes[obs_id]

        # 3. 积分位姿与横向状态
        for obs_id, obs in self.obstacles.items():
            accel = accelerations.get(obs_id, 0.0)
            self._last_accelerations[obs_id] = accel
            if obs.vx >= 0.0:
                obs.vx = float(max(0.0, obs.vx + accel * dt))
            else:
                obs.vx = float(min(0.0, obs.vx - accel * dt))

            # 横向换道过渡 (平滑三次多项式插值)
            lc_state = self._lane_change_states.get(obs_id)
            if lc_state is not None:
                lc_state["elapsed"] += dt
                p = float(np.clip(lc_state["elapsed"] / max(1e-3, lc_state["duration"]), 0.0, 1.0))
                # 3p^2 - 2p^3 平滑过渡
                y_delta = lc_state["target_y"] - lc_state["start_y"]
                obs.y = float(lc_state["start_y"] + y_delta * (3.0 * p ** 2 - 2.0 * p ** 3))
                # dy/dt = y_delta * (6p - 6p^2) / duration
                obs.vy = float(y_delta * (6.0 * p * (1.0 - p)) / max(1e-3, lc_state["duration"]))
                if p >= 1.0:
                    obs.y = float(lc_state["target_y"])
                    obs.vy = 0.0
                    del self._lane_change_states[obs_id]
            else:
                obs.y += obs.vy * dt

            if abs(obs.vx) > 1e-6 or abs(obs.vy) > 1e-6:
                obs.heading = float(np.arctan2(obs.vy, obs.vx))
            obs.x += obs.vx * dt

            self.generate_predicted_trajectory(obs_id, horizon=6.0, dt=0.2)

        self.current_time += dt

    def generate_predicted_trajectory(self, obstacle_id: str, horizon: float = 5.0, dt: float = 0.2) -> List[TrajectoryPoint]:
        """
        为指定障碍物生成未来预测时空轨迹：纵向用最近一次 IDM 加速度作恒加速度外推，
        横向保持恒速；纵向预测速度降到零后位置不再前进。
        :param obstacle_id: 目标障碍物ID
        :param horizon: 预测时域 (s)
        :param dt: 预测采样步长 (s)
        :return: 预测轨迹点序列
        """
        obs = self.obstacles[obstacle_id]
        pred_pts = []
        steps = int(horizon / dt)
        cur_x, cur_y = obs.x, obs.y
        longitudinal_accel = self._last_accelerations.get(obstacle_id, 0.0)
        direction = 1.0 if obs.vx >= 0.0 else -1.0
        initial_speed = abs(obs.vx)
        stop_distance = (
            initial_speed ** 2 / (2.0 * abs(longitudinal_accel))
            if longitudinal_accel < 0.0 else float("inf")
        )
        stop_time = (
            initial_speed / abs(longitudinal_accel)
            if longitudinal_accel < 0.0 else float("inf")
        )
        for i in range(steps):
            t = (i + 1) * dt
            predicted_speed = max(0.0, initial_speed + longitudinal_accel * t)
            if t >= stop_time:
                displacement = stop_distance
            else:
                displacement = max(
                    0.0, initial_speed * t + 0.5 * longitudinal_accel * t ** 2
                )
            pred_vx = direction * predicted_speed
            pred_pts.append(TrajectoryPoint(
                x=float(cur_x + direction * displacement),
                y=float(cur_y + obs.vy * t),
                theta=(
                    float(np.arctan2(obs.vy, pred_vx))
                    if abs(predicted_speed) > 1e-9 or abs(obs.vy) > 1e-9
                    else float(obs.heading)
                ),
                v=float(np.hypot(predicted_speed, obs.vy)),
                t=float(t),
            ))
        obs.predicted_trajectory = pred_pts
        return pred_pts

    def get_all_obstacles(self) -> List[ObstacleState]:
        """获取当前所有活跃障碍物状态列表"""
        return list(self.obstacles.values())
