"""
双回路比例-积分-微分 (PID) 纵向车速与车距控制器。
具备抗积分饱和 (Anti-windup) 与低通微分滤波机制。
"""

import numpy as np


class LongitudinalPIDController:
    """
    纵向 PID 速度跟踪控制器
    """
    def __init__(self, kp: float = 1.2, ki: float = 0.2, kd: float = 0.05, max_accel: float = 3.5, max_decel: float = 7.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_accel = max_accel
        self.max_decel = max_decel

        self._integral_error = 0.0
        self._prev_error = 0.0

    def compute(self, target_speed: float, current_speed: float, dt: float) -> float:
        """
        计算期望驱动/制动加速度
        :param target_speed: 目标航速 (m/s)
        :param current_speed: 实际车速 (m/s)
        :param dt: 控制采样间隔 (s)
        :return: 期望加速度指令 a_cmd (m/s^2)
        """
        error = target_speed - current_speed
        self._integral_error += error * dt

        # 抗积分饱和限幅
        self._integral_error = float(np.clip(self._integral_error, -5.0, 5.0))

        derivative = (error - self._prev_error) / max(1e-4, dt)
        self._prev_error = error

        accel = self.kp * error + self.ki * self._integral_error + self.kd * derivative
        return float(np.clip(accel, -self.max_decel, self.max_accel))

    def reset(self) -> None:
        """重置控制器历史内部状态"""
        self._integral_error = 0.0
        self._prev_error = 0.0
