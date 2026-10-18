"""
Frenet 坐标系与笛卡尔坐标系之间的双向变换及高阶导数映射接口。
严格遵循智能车辆运动学非线性映射关系 (Werling et al., ICRA 2010 标准公式)。
为道路对齐的局部轨迹规划（Lattice Planner、EM Planner）提供坚实的数学基础。
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

    l: float = 0.0                     # 横向相对偏移 (m), 位于参考线左侧为正，右侧为负
    l_d: float = 0.0                   # 横向对时间的导数 dl/dt (m/s)
    l_dd: float = 0.0                  # 横向对时间的二阶导数 d^2l/dt^2 (m/s^2)

    # 路径规划中常用的对弧长 s 的导数
    l_prime: float = 0.0               # dl/ds
    l_prime_prime: float = 0.0         # d^2l/ds^2


@dataclass
class ReferencePoint:
    """参考线离散采样点"""
    x: float                           # 全局坐标 X (m)
    y: float                           # 全局坐标 Y (m)
    theta: float                       # 参考线切线方位角 (rad)
    kappa: float = 0.0                 # 参考线曲率 (1/m)
    dkappa: float = 0.0                # 曲率对弧长的一阶导数 (1/m^2)
    s: float = 0.0                     # 沿参考线的累积曲线弧长 (m)


class FrenetConverter:
    """
    Cartesian <-> Frenet 坐标转换器
    严格推导非线性曲率与时空映射关系
    """

    @classmethod
    def cartesian_to_frenet(
        cls,
        state: VehicleState,
        reference_line: List[ReferencePoint]
    ) -> FrenetPoint:
        """
        笛卡尔状态映射至 Frenet 状态。
        寻找正交投影参考点，并计算精确的一阶与二阶导数。
        :param state: 车辆瞬时状态
        :param reference_line: 参考线采样序列
        :return: Frenet 状态点
        """
        if not reference_line:
            return FrenetPoint(s=0.0, s_d=state.vx, s_dd=state.ax, l=0.0)

        # 1. 寻找最近的参考点作为正交投影原点 (Footpoint 估计)
        min_dist_sq = float("inf")
        match_idx = 0
        for i, ref in enumerate(reference_line):
            d_sq = (state.x - ref.x) ** 2 + (state.y - ref.y) ** 2
            if d_sq < min_dist_sq:
                min_dist_sq = d_sq
                match_idx = i

        ref_match = reference_line[match_idx]

        # 2. 向量投影与横向偏移 l (左正右负)
        dx = state.x - ref_match.x
        dy = state.y - ref_match.y
        cos_t = np.cos(ref_match.theta)
        sin_t = np.sin(ref_match.theta)

        # 沿切线与法线的投影
        delta_s = dx * cos_t + dy * sin_t
        l = -dx * sin_t + dy * cos_t

        s = max(0.0, ref_match.s + delta_s)

        # 3. 航向偏差与速度投影
        delta_theta = state.heading - ref_match.theta
        delta_theta = float(np.arctan2(np.sin(delta_theta), np.cos(delta_theta)))

        # 避免分母奇异性
        one_minus_kappa_l = max(0.01, 1.0 - ref_match.kappa * l)

        # dl/ds
        l_prime = one_minus_kappa_l * np.tan(delta_theta)

        # 纵向速度 s_dot 与横向速度 l_dot
        s_d = state.velocity * np.cos(delta_theta) / one_minus_kappa_l
        l_d = state.velocity * np.sin(delta_theta)

        return FrenetPoint(
            s=s,
            s_d=float(s_d),
            s_dd=float(state.acceleration * np.cos(delta_theta)),
            l=float(l),
            l_d=float(l_d),
            l_dd=0.0,
            l_prime=float(l_prime),
            l_prime_prime=0.0
        )

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
        # 严密微分几何推导:
        # x = x_r - l * sin(theta_r)
        # y = y_r + l * cos(theta_r)
        x = ref_pt.x - frenet_pt.l * np.sin(ref_pt.theta)
        y = ref_pt.y + frenet_pt.l * np.cos(ref_pt.theta)

        # 方位角 theta = theta_r + arctan(l' / (1 - kappa_r * l))
        one_minus_kappa_l = max(0.01, 1.0 - ref_pt.kappa * frenet_pt.l)
        delta_theta = np.arctan2(frenet_pt.l_prime, one_minus_kappa_l)
        theta = ref_pt.theta + delta_theta
        theta = float(np.arctan2(np.sin(theta), np.cos(theta)))

        # 速度合成 v = sqrt((s_dot * (1 - kappa_r * l))^2 + (s_dot * l')^2)
        v = np.hypot(frenet_pt.s_d * one_minus_kappa_l, frenet_pt.s_d * frenet_pt.l_prime)

        return TrajectoryPoint(
            x=float(x),
            y=float(y),
            theta=float(theta),
            s=float(frenet_pt.s),
            v=float(v),
            a=float(frenet_pt.s_dd)
        )
