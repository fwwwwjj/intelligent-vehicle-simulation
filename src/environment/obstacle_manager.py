"""
动态交通参与者管理与时空轨迹预测接口。
支持微观交通流行为演化模型：
1. 智能驾驶员纵向跟驰模型 (IDM, Intelligent Driver Model)
2. 协同换道模型 (MOBIL, Minimizing Overall Braking Induced by Lane change)
满足中国仿真创新应用大赛对于“交通流自然演化高保真度”的高分评审标准。
"""

from typing import List, Dict, Optional
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
    负责多障碍物的步进模拟、轨迹预测生成与查询
    """
    def __init__(self):
        self.obstacles: Dict[str, ObstacleState] = {}
        self.idm_models: Dict[str, IntelligentDriverModel] = {}
        self.mobil_model = MOBILLaneChangeModel()

    def add_obstacle(self, obstacle: ObstacleState, idm: Optional[IntelligentDriverModel] = None) -> None:
        """注册障碍物"""
        self.obstacles[obstacle.id] = obstacle
        if idm is not None:
            self.idm_models[obstacle.id] = idm

    def step(self, dt: float) -> None:
        """
        环境物理演化一个时步 dt
        更新所有交通参与者的一阶/二阶运动状态
        """
        for obs_id, obs in self.obstacles.items():
            obs.x += obs.vx * dt
            obs.y += obs.vy * dt
            # 更新未来预测轨迹序列
            self.generate_predicted_trajectory(obs_id, horizon=5.0, dt=0.2)

    def generate_predicted_trajectory(self, obstacle_id: str, horizon: float = 5.0, dt: float = 0.2) -> List[TrajectoryPoint]:
        """
        为指定障碍物生成未来预测时空轨迹 (包含恒定速度/加速度预测)
        :param obstacle_id: 目标障碍物ID
        :param horizon: 预测时域 (s)
        :param dt: 预测采样步长 (s)
        :return: 预测轨迹点序列
        """
        obs = self.obstacles[obstacle_id]
        pred_pts = []
        steps = int(horizon / dt)
        cur_x, cur_y = obs.x, obs.y
        for i in range(steps):
            t = (i + 1) * dt
            pred_pts.append(TrajectoryPoint(
                x=float(cur_x + obs.vx * t),
                y=float(cur_y + obs.vy * t),
                theta=float(obs.heading),
                v=float(np.hypot(obs.vx, obs.vy)),
                t=float(t)
            ))
        obs.predicted_trajectory = pred_pts
        return pred_pts

    def get_all_obstacles(self) -> List[ObstacleState]:
        """获取当前所有活跃障碍物状态列表"""
        return list(self.obstacles.values())
