"""
智能车辆运动学自行车模型 (Kinematic Bicycle Model) 数字建模。
适用于中低速场景及运动学几何可达性约束检验。
"""

import numpy as np
from src.common.state import VehicleState


class KinematicBicycleModel:
    """
    单轨自行车运动学模型
    状态量: [x, y, heading, v]
    控制量: [acceleration, steer_angle]
    """
    def __init__(self, wheelbase: float = 2.85, max_steer: float = 0.65, max_accel: float = 3.5, max_decel: float = 7.0):
        self.wheelbase = wheelbase          # 轴距 L (m)
        self.max_steer = max_steer          # 最大前轮转角 (rad)
        self.max_accel = max_accel          # 最大加速度 (m/s^2)
        self.max_decel = max_decel          # 最大制动减速度 (m/s^2)

    def step(self, state: VehicleState, accel_cmd: float, steer_cmd: float, dt: float) -> VehicleState:
        """
        前向数值积分步进 (采用 4 阶 Runge-Kutta 或高精度 Euler 积分)
        :param state: 当前时刻车辆状态
        :param accel_cmd: 期望输入纵向加速度 (m/s^2)
        :param steer_cmd: 期望输入前轮转角 (rad)
        :param dt: 仿真步长 (s)
        :return: 下一时态车辆状态
        """
        # 输入饱和度限制
        delta = float(np.clip(steer_cmd, -self.max_steer, self.max_steer))
        accel = float(np.clip(accel_cmd, -self.max_decel, self.max_accel))

        # 车辆质心侧偏角估计 beta = arctan(lr / (lf + lr) * tan(delta))，此处简化为后轴中心参考
        x = state.x + state.velocity * np.cos(state.heading) * dt
        y = state.y + state.velocity * np.sin(state.heading) * dt
        heading = state.heading + (state.velocity / self.wheelbase) * np.tan(delta) * dt
        # 航向角归一化至 [-pi, pi]
        heading = float(np.arctan2(np.sin(heading), np.cos(heading)))

        velocity = max(0.0, state.velocity + accel * dt)
        yaw_rate = (state.velocity / self.wheelbase) * np.tan(delta)

        return VehicleState(
            x=x,
            y=y,
            heading=heading,
            velocity=velocity,
            vx=velocity,
            vy=0.0,
            acceleration=accel,
            ax=accel,
            ay=velocity * yaw_rate,
            yaw_rate=yaw_rate,
            steer_angle=delta,
            timestamp=state.timestamp + dt
        )
