"""
Frenet 坐标系与笛卡尔坐标系之间的双向变换及高阶导数映射接口。
严格遵循智能车辆运动学非线性映射关系 (Werling et al., ICRA 2010 标准公式)。
为道路对齐的局部轨迹规划（Lattice Planner、EM Planner）提供坚实的数学基础。

本模块另外提供参考线按弧长插值 :meth:`FrenetConverter.ref_point_at_s`，
避免规划器在各处重复编写易错的参考线查找逻辑。
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


def _wrap_angle(angle: float) -> float:
    """将角度归一化到 [-pi, pi]。"""
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


class FrenetConverter:
    """
    Cartesian <-> Frenet 坐标转换器
    严格推导非线性曲率与时空映射关系
    """

    _ref_line_cache: dict = {}

    @classmethod
    def _get_sorted_s_array(cls, reference_line: List[ReferencePoint]) -> np.ndarray:
        ref_id = id(reference_line)
        cached = cls._ref_line_cache.get(ref_id)
        if cached is not None and cached[0] == len(reference_line):
            return cached[1]
        ss = np.array([p.s for p in reference_line], dtype=float)
        if not np.all(np.diff(ss) >= -1e-9):
            order = np.argsort(ss)
            ss = ss[order]
        if len(cls._ref_line_cache) > 64:
            cls._ref_line_cache.clear()
        cls._ref_line_cache[ref_id] = (len(reference_line), ss)
        return ss

    @classmethod
    def ref_point_at_s(cls, reference_line: List[ReferencePoint], s_query: float) -> ReferencePoint:
        """
        按弧长 ``s_query`` 在参考线上做线性插值。

        :param reference_line: 按 s 递增排列的参考线采样点
        :param s_query: 查询弧长 (m)
        :return: 插值后的参考线点
        """
        if not reference_line:
            raise ValueError("reference_line must not be empty")
        if len(reference_line) == 1:
            return reference_line[0]

        ss = cls._get_sorted_s_array(reference_line)

        s_query = float(np.clip(s_query, ss[0], ss[-1]))
        idx = int(np.searchsorted(ss, s_query))
        if idx <= 0:
            return reference_line[0]
        if idx >= len(reference_line):
            return reference_line[-1]

        s0, s1 = ss[idx - 1], ss[idx]
        p0, p1 = reference_line[idx - 1], reference_line[idx]
        if abs(s1 - s0) < 1e-9:
            return p1
        ratio = (s_query - s0) / (s1 - s0)

        theta0 = p0.theta
        theta1 = p1.theta
        # 处理 ±pi 环绕
        dtheta = _wrap_angle(theta1 - theta0)
        theta = _wrap_angle(theta0 + ratio * dtheta)

        return ReferencePoint(
            x=float(p0.x + ratio * (p1.x - p0.x)),
            y=float(p0.y + ratio * (p1.y - p0.y)),
            theta=float(theta),
            kappa=float(p0.kappa + ratio * (p1.kappa - p0.kappa)),
            dkappa=float(p0.dkappa + ratio * (p1.dkappa - p0.dkappa)),
            s=float(s_query),
        )

    @classmethod
    def cartesian_to_frenet(
        cls,
        state: VehicleState,
        reference_line: List[ReferencePoint]
    ) -> FrenetPoint:
        """
        笛卡尔状态映射至 Frenet 状态。
        先对参考线折线做正交投影 (Footpoint)，再按 Werling 公式计算导数。
        :param state: 车辆瞬时状态
        :param reference_line: 参考线采样序列
        :return: Frenet 状态点
        """
        if not reference_line:
            return FrenetPoint(s=0.0, s_d=state.vx, s_dd=state.ax, l=0.0)

        # 1. 粗筛最近采样点
        min_dist_sq = float("inf")
        match_idx = 0
        for i, ref in enumerate(reference_line):
            d_sq = (state.x - ref.x) ** 2 + (state.y - ref.y) ** 2
            if d_sq < min_dist_sq:
                min_dist_sq = d_sq
                match_idx = i

        # 2. 在相邻线段上做精确正交投影
        best_l = 0.0
        best_s = reference_line[match_idx].s
        best_dist = float("inf")
        for seg_idx in range(max(0, match_idx - 1), min(len(reference_line) - 1, match_idx + 1) + 1):
            p0 = reference_line[seg_idx]
            p1 = reference_line[seg_idx + 1] if seg_idx + 1 < len(reference_line) else None
            if p1 is None:
                continue
            vx = p1.x - p0.x
            vy = p1.y - p0.y
            seg_len_sq = vx * vx + vy * vy
            if seg_len_sq < 1e-12:
                continue
            ratio = ((state.x - p0.x) * vx + (state.y - p0.y) * vy) / seg_len_sq
            ratio = float(np.clip(ratio, 0.0, 1.0))
            px = p0.x + ratio * vx
            py = p0.y + ratio * vy
            seg_theta = np.arctan2(vy, vx)
            dx = state.x - px
            dy = state.y - py
            lateral = -dx * np.sin(seg_theta) + dy * np.cos(seg_theta)
            if abs(lateral) < best_dist:
                best_dist = abs(lateral)
                best_l = float(lateral)
                best_s = float(p0.s + ratio * np.sqrt(seg_len_sq))

        ref_match = cls.ref_point_at_s(reference_line, best_s)
        s = max(0.0, float(best_s))
        l = float(best_l)

        # 3. 航向偏差与速度投影
        delta_theta = _wrap_angle(state.heading - ref_match.theta)
        one_minus_kappa_l = max(0.01, 1.0 - ref_match.kappa * l)

        # dl/ds, ds/dt, dl/dt
        l_prime = one_minus_kappa_l * np.tan(delta_theta)
        s_d = state.velocity * np.cos(delta_theta) / one_minus_kappa_l
        l_d = state.velocity * np.sin(delta_theta)

        # 二阶导：l_dd = a_x sinΔθ + v cosΔθ (r - κ s_d)
        l_dd = state.ax * np.sin(delta_theta) + state.velocity * np.cos(delta_theta) * (
            state.yaw_rate - ref_match.kappa * s_d
        )

        return FrenetPoint(
            s=s,
            s_d=float(s_d),
            s_dd=float(state.ax * np.cos(delta_theta)),
            l=l,
            l_d=float(l_d),
            l_dd=float(l_dd),
            l_prime=float(l_prime),
            l_prime_prime=0.0,
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
        # 位置：沿参考线法向偏移 l
        x = ref_pt.x - frenet_pt.l * np.sin(ref_pt.theta)
        y = ref_pt.y + frenet_pt.l * np.cos(ref_pt.theta)

        # 方位角 theta = theta_r + arctan(l' / (1 - kappa_r * l))
        one_minus_kappa_l = max(0.01, 1.0 - ref_pt.kappa * frenet_pt.l)
        delta_theta = np.arctan2(frenet_pt.l_prime, one_minus_kappa_l)
        theta = _wrap_angle(ref_pt.theta + delta_theta)

        # 速度合成 v = sqrt((s_dot * (1 - kappa_r * l))^2 + (s_dot * l')^2)
        v = float(np.hypot(frenet_pt.s_d * one_minus_kappa_l, frenet_pt.s_d * frenet_pt.l_prime))

        # 曲率映射：kappa = [ (l'' + (kappa' l + kappa l') tanΔθ) cos²Δθ / (1-κl) + κ ] cosΔθ/(1-κl)
        cos_d = np.cos(delta_theta)
        kappa = (
            (
                frenet_pt.l_prime_prime
                + (ref_pt.dkappa * frenet_pt.l + ref_pt.kappa * frenet_pt.l_prime)
                * np.tan(delta_theta)
            )
            * cos_d**2
            / one_minus_kappa_l
            + ref_pt.kappa
        ) * cos_d / one_minus_kappa_l

        return TrajectoryPoint(
            x=float(x),
            y=float(y),
            theta=float(theta),
            kappa=float(kappa),
            dkappa=float(ref_pt.dkappa),
            s=float(frenet_pt.s),
            v=v,
            a=float(frenet_pt.s_dd),
            jerk=float(frenet_pt.s_ddd),
        )
