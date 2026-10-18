"""
Frenet 坐标系与笛卡尔坐标系之间的双向变换及微积分导数映射接口。
为道路对齐的局部轨迹规划（Lattice Planner、EM Planner）提供理论基础。
"""

from dataclasses import dataclass
from typing import List, Tuple, Optional
import numpy as np
from src.common.state import TrajectoryPoint, VehicleState


@dataclass
class FrenetPoint:
    """
    Frenet 坐标系下的单点状态
    """
    s: float = 0.0                     # 沿参考线的纵向弧长 (m)
    s_d: float = 0.0                   # 纵向速度 ds/dt (m/s)
    s_dd: float = 0.0                  # 纵向加速度 d^2s/dt^2 (m/s^2)
    s_ddd: float = 0.0                 # 纵向加加速度 d^3s/dt^3 (m/s^3)

    l: float = 0.0                     # 横向相对偏移 (m), 左正右负
    l_d: float = 0.0                   # 横向对时间的导数 dl/dt (m/s)
    l_dd: float = 0.0                  # 横向对时间的二阶导数 d^2l/dt^2 (m/s^2)

    # 路径规划中常用的对弧长 s 的导数
    l_prime: float = 0.0               # dl/ds
    l_prime_prime: float = 0.0         # d^2l/ds^2


@dataclass
class ReferencePoint:
    """参考线离散点"""
    x: float
    y: float
    theta: float
    kappa: float
    dkappa: float = 0.0
    s: float = 0.0


class FrenetConverter:
    """
    Cartesian <-> Frenet 坐标转换器
    严格遵循智能车辆运动学非线性映射关系
    """

    @classmethod
    def cartesian_to_frenet(
        cls,
        state: VehicleState,
        reference_line: List[ReferencePoint]
    ) -> FrenetPoint:
        """
        笛卡尔状态映射至 Frenet 状态。
        :param state: 车辆瞬时状态
        :param reference_line: 参考线序列
        :return: Frenet 状态点
        """
        # 寻找最近参考点
        # 实际计算需进行投影计算 (待实现)
        frenet = FrenetPoint(
            s=0.0,
            s_d=state.vx,
            s_dd=state.ax,
            l=0.0,
            l_prime=0.0,
            l_prime_prime=0.0
        )
        return frenet

    @classmethod
    def frenet_to_cartesian(
        cls,
        frenet_pt: FrenetPoint,
        ref_pt: ReferencePoint
    ) -> TrajectoryPoint:
        """
        Frenet 单点变换回笛卡尔空间轨迹点。
        :param frenet_pt: 局部规划生成的 Frenet 状态
        :param ref_pt: 对应 s 处的参考线点信息
        :return: 全局笛卡尔空间轨迹点
        """
        # 严格计算推导:
        # x = x_r - l * sin(theta_r)
        # y = y_r + l * cos(theta_r)
        x = ref_pt.x - frenet_pt.l * np.sin(ref_pt.theta)
        y = ref_pt.y + frenet_pt.l * np.cos(ref_pt.theta)
        theta = ref_pt.theta + np.arctan2(frenet_pt.l_prime, 1.0 - ref_pt.kappa * frenet_pt.l)

        return TrajectoryPoint(
            x=x,
            y=y,
            theta=theta,
            s=frenet_pt.s,
            v=frenet_pt.s_d,
            a=frenet_pt.s_dd
        )
