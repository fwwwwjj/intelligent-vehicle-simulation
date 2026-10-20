"""
经典车道中心线跟踪轨迹规划器 (Centerline Planner)。

作为基线/消融实验中的“无优化、无搜索”规划器：本模块**不做采样、不做代价评估、
不做碰撞校验**，只生成一条时间参数化的平滑轨迹，用于量化优化/搜索型规划器
(Lattice / EM) 的贡献。

轨迹构造：
1. 横向：在 Frenet 坐标系下，用五次多项式把当前横向偏移 ``l`` 平滑收敛到
   目标车道中心 ``decision.lateral_shift_l``（通常为 0）；
2. 纵向：用五次多项式使沿参考线的弧长 ``s(t)`` 从当前车速平滑过渡到
   ``decision.target_speed``；
3. 逐点经 Frenet -> Cartesian 映射，并填充 ``theta / kappa / dkappa / v / a /
   jerk / t / s``，供 LQR / Pure Pursuit 控制器与指标模块使用。

该规划器不含障碍物感知，因此空参考线时退化为沿当前航向的直线参考线，且
输出轨迹恒定 ``valid=True``（碰撞/动力学可行性由下游控制器与安全监督兜底）。
"""

from typing import List

import numpy as np

from src.common.state import VehicleState, ObstacleState, Trajectory, TrajectoryPoint
from src.common.frenet_frame import ReferencePoint, FrenetConverter, FrenetPoint
from src.decision.base_decision import DecisionOutput
from src.planning.base_planner import BasePlanner
from src.planning.frenet_lattice import QuinticPolynomial


class CenterlinePlanner(BasePlanner):
    """
    车道中心线跟踪规划器（经典基线）。

    与 :class:`~src.planning.frenet_lattice.FrenetLatticePlanner` 不同，本规划器
    只输出一条候选轨迹：不搜索横向偏移、不评估障碍物代价、不检查碰撞，
    代表“纯参考线跟踪”的经典规划基线。
    """

    def __init__(
        self,
        ego_length: float = 4.8,
        ego_width: float = 1.9,
        max_accel: float = 3.5,
        max_decel: float = 7.0,
    ):
        self.ego_length = float(ego_length)
        self.ego_width = float(ego_width)
        # 纵向动力学限值保存在构造器中，供接口/参数一致性与后续扩展使用；
        # 中心线规划器本身只跟踪决策目标速度，实际执行限幅由控制器完成。
        self.max_accel = float(max_accel)
        self.max_decel = float(max_decel)

    # ------------------------------------------------------------------
    # 参考线兜底
    # ------------------------------------------------------------------
    @staticmethod
    def _fallback_reference_line(
        ego_state: VehicleState, length: float = 120.0
    ) -> List[ReferencePoint]:
        """没有参考线时，沿当前航向构造一条直线参考线，保证规划器可退化为直线行驶。"""
        ref: List[ReferencePoint] = []
        cos_h = float(np.cos(ego_state.heading))
        sin_h = float(np.sin(ego_state.heading))
        for i in range(int(length) + 1):
            ref.append(
                ReferencePoint(
                    x=float(ego_state.x + i * cos_h),
                    y=float(ego_state.y + i * sin_h),
                    theta=float(ego_state.heading),
                    kappa=0.0,
                    dkappa=0.0,
                    s=float(i),
                )
            )
        return ref

    # ------------------------------------------------------------------
    # 微分几何后处理
    # ------------------------------------------------------------------
    @staticmethod
    def _fill_curvature(points: List[TrajectoryPoint]) -> None:
        """由离散轨迹的 theta(s) 差分计算曲率与曲率变化率，保证 kappa 与几何一致。"""
        n = len(points)
        if n == 0:
            return
        if n == 1:
            points[0].kappa = 0.0
            points[0].dkappa = 0.0
            return

        kappas = np.zeros(n, dtype=float)
        for i in range(n):
            if i == 0:
                ds = points[1].s - points[0].s
                dtheta = points[1].theta - points[0].theta
            elif i == n - 1:
                ds = points[-1].s - points[-2].s
                dtheta = points[-1].theta - points[-2].theta
            else:
                ds = points[i + 1].s - points[i - 1].s
                dtheta = points[i + 1].theta - points[i - 1].theta
            dtheta = float(np.arctan2(np.sin(dtheta), np.cos(dtheta)))
            kappas[i] = dtheta / max(abs(ds), 1e-3)

        s_array = np.array([p.s for p in points], dtype=float)
        # 停车/重合轨迹点上 ds 可能为 0；np.gradient 会除零产生 NaN。
        if n > 2 and np.all(np.diff(s_array) >= 1e-6):
            dkappas = np.gradient(kappas, s_array)
        else:
            dkappas = np.zeros(n, dtype=float)

        for i, pt in enumerate(points):
            pt.kappa = float(kappas[i])
            pt.dkappa = float(dkappas[i])

    # ------------------------------------------------------------------
    # 主入口
    # ------------------------------------------------------------------
    def plan(
        self,
        ego_state: VehicleState,
        reference_line: List[ReferencePoint],
        obstacles: List[ObstacleState],
        decision: DecisionOutput,
        plan_time_horizon: float = 5.0,
        dt: float = 0.1,
    ) -> Trajectory:
        """
        生成一条跟踪目标车道中心线的五次多项式时空轨迹。

        :param ego_state: 自车当前状态
        :param reference_line: 目标车道参考线；为空时沿当前航向直线兜底
        :param obstacles: 感知障碍物（本基线规划器刻意不使用）
        :param decision: 行为决策输出（使用 target_speed 与 lateral_shift_l）
        :param plan_time_horizon: 规划时域 (s)
        :param dt: 轨迹点采样间隔 (s)
        :return: 单条时间参数化轨迹，``valid`` 恒为 True
        """
        # 障碍物感知是 Lattice/EM 规划器的能力，中心线基线刻意忽略 obstacles。
        del obstacles

        if not reference_line:
            reference_line = self._fallback_reference_line(ego_state)

        horizon = float(plan_time_horizon)
        if not np.isfinite(horizon) or horizon <= 0.0:
            horizon = 0.1
        step = float(dt)
        if not np.isfinite(step) or step <= 0.0:
            step = 0.1

        ego_frenet = FrenetConverter.cartesian_to_frenet(ego_state, reference_line)
        v0 = max(0.0, float(ego_frenet.s_d))
        # 部分调用方只填 scalar velocity 而未填 vx（默认 0）；此时用 scalar 速度投影兜底，
        # 避免把“正在行驶”误判为静止起点。
        if v0 <= 1e-6 and abs(float(ego_state.velocity)) > 1e-6:
            ref_at_ego = FrenetConverter.ref_point_at_s(reference_line, ego_frenet.s)
            delta_theta = float(ego_state.heading - ref_at_ego.theta)
            delta_theta = float(np.arctan2(np.sin(delta_theta), np.cos(delta_theta)))
            v0 = max(0.0, float(ego_state.velocity) * float(np.cos(delta_theta)))
        target_speed = max(0.0, float(decision.target_speed))
        target_l = float(decision.lateral_shift_l)

        # 纵向终点用平均速度估计，避免目标车速为 0 时 s_end == s_start 导致轨迹卡死。
        s_end = ego_frenet.s + 0.5 * (v0 + target_speed) * horizon
        if s_end < ego_frenet.s:
            s_end = ego_frenet.s

        # 初速度/初加速度取当前纵向状态；终点速度 = 决策目标速度，终点加速度 = 0。
        poly_s = QuinticPolynomial(
            xs=ego_frenet.s,
            vxs=v0,
            axs=0.0,
            xe=s_end,
            vxe=target_speed,
            axe=0.0,
            T=horizon,
        )
        # 横向：从当前偏移平滑收敛到目标车道中心。
        # 初始横向速率不能取 0：本规划器每 50 ms 重规划一次，若每帧都从静止
        # 横向起步，横向位移会被反复“重置”，形成 Zeno 式收敛停滞（无法完成
        # 换道）。这里与 Lattice 规划器一致，取朝目标的期望横向速率，
        # 使 receding-horizon 下仍能真正收敛到目标车道。
        lateral_error = target_l - float(ego_frenet.l)
        l_d0 = float(np.clip(0.8 * lateral_error, -1.2, 1.2))
        poly_l = QuinticPolynomial(
            xs=float(ego_frenet.l),
            vxs=l_d0,
            axs=0.0,
            xe=target_l,
            vxe=0.0,
            axe=0.0,
            T=horizon,
        )

        steps = max(2, int(round(horizon / step)) + 1)
        points: List[TrajectoryPoint] = []
        prev_s = float(ego_frenet.s)
        for k in range(steps):
            t = min(k * step, horizon)
            s_t = float(poly_s.calc_point(t))
            # 数值安全：极端边界下五次多项式可能轻微回退，强制弧长非减。
            if k > 0 and s_t < prev_s:
                s_t = prev_s
            prev_s = s_t

            s_d = float(poly_s.calc_first_derivative(t))
            s_dd = float(poly_s.calc_second_derivative(t))
            s_ddd = float(poly_s.calc_third_derivative(t))
            l_t = float(poly_l.calc_point(t))
            l_d = float(poly_l.calc_first_derivative(t))
            l_dd = float(poly_l.calc_second_derivative(t))

            # dl/ds：低速/静止时 dl/dt 与 ds/dt 同时趋近 0，需做除零保护。
            if abs(s_d) > 1e-3:
                l_prime = l_d / s_d
            else:
                l_prime = 0.0
            l_prime = float(np.clip(l_prime, -5.0, 5.0))

            ref_pt = FrenetConverter.ref_point_at_s(reference_line, s_t)
            f_pt = FrenetPoint(
                s=s_t,
                s_d=s_d,
                s_dd=s_dd,
                s_ddd=s_ddd,
                l=l_t,
                l_d=l_d,
                l_dd=l_dd,
                l_prime=l_prime,
                l_prime_prime=0.0,
            )
            c_pt = FrenetConverter.frenet_to_cartesian(f_pt, ref_pt)
            c_pt.t = float(t)
            c_pt.s = float(s_t)
            c_pt.a = float(s_dd)
            c_pt.jerk = float(s_ddd)
            points.append(c_pt)

        # 用离散几何重算曲率，保证 kappa/dkappa 与 theta(s) 自洽。
        self._fill_curvature(points)
        return Trajectory(points=points, total_cost=0.0, valid=True)


__all__ = ["CenterlinePlanner"]
