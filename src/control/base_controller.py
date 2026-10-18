"""
运动控制系统抽象基类与底层执行机构控制指令定义。
规范上层规划轨迹到车辆底层方向盘转角与油门/制动加速度的映射接口。
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional
from src.common.state import VehicleState, Trajectory, TrajectoryPoint


@dataclass
class ControlCommand:
    """底层执行机构控制量指令"""
    steering_angle: float = 0.0         # 期望前轮转角 delta (rad)
    acceleration: float = 0.0           # 期望纵向控制加速度 (m/s^2)
    throttle: float = 0.0               # 油门开度 [0.0, 1.0]
    brake: float = 0.0                  # 制动踏板开度 [0.0, 1.0]
    steering_rate: float = 0.0          # 转向角速度 (rad/s)


class BaseController(ABC):
    """轨迹跟踪控制器基类"""

    @abstractmethod
    def compute_control(
        self,
        current_state: VehicleState,
        target_trajectory: Trajectory,
        dt: float
    ) -> ControlCommand:
        """
        根据当前车辆状态与期望参考轨迹，计算控制输出
        :param current_state: 车辆当前物理状态
        :param target_trajectory: 规划层下发的最新时空轨迹
        :param dt: 控制周期 (s)
        :return: 控制指令
        """
        pass
