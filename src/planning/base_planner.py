"""
路径与轨迹规划器抽象基类。
定义由高阶决策输出、环境感知及主车状态到时空轨迹序列的标准映射接口。
"""

from abc import ABC, abstractmethod
from typing import List
from src.common.state import VehicleState, ObstacleState, Trajectory
from src.common.frenet_frame import ReferencePoint
from src.decision.base_decision import DecisionOutput


class BasePlanner(ABC):
    """轨迹规划器抽象基类"""

    @abstractmethod
    def plan(
        self,
        ego_state: VehicleState,
        reference_line: List[ReferencePoint],
        obstacles: List[ObstacleState],
        decision: DecisionOutput,
        plan_time_horizon: float = 5.0,
        dt: float = 0.1
    ) -> Trajectory:
        """
        执行局部时空轨迹规划
        :param ego_state: 自车当前真实/滤波后状态
        :param reference_line: 当前目标车道平滑参考线
        :param obstacles: 感知层检测到的动态与静态障碍物列表
        :param decision: 行为决策层给出的战术指导
        :param plan_time_horizon: 规划未来时间窗 (s)
        :param dt: 轨迹点时间采样间隔 (s)
        :return: 经过动力学约束校验与无碰撞保证的最优时空轨迹
        """
        pass
