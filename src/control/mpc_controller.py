"""
线性时变模型预测控制 (Linear Time-Varying MPC) 轨迹跟踪控制器。

设计口径
--------
- 预测模型：以规划轨迹为参考的 Frenet 误差动力学
  ``e = [e_y, e_psi, e_v]``，控制量 ``u = [a, delta]``；
- 参考量：按轨迹时间戳插值得到 ``v_ref / kappa_ref / a_ref``，
  曲率通过 ``delta_ref = atan(L * kappa_ref)`` 转成前馈转角；
- 优化：在预测时域内最小化横向/航向/速度误差 + 控制量 + 控制增量，
  同时施加执行机构硬约束与横向/航向软约束（slack）；
- 求解：调用 :mod:`src.common.qp` 的纯 NumPy ADMM QP，不强制依赖
  OSQP/cvxpy；求解失败时退化为带曲率前馈的 PD + 速度 P 控制器，
  并把失败信息记录在 ``last_solve_info`` 中，便于诊断而不是静默失败。

该控制器与 :class:`LQRController` 具有相同的
:class:`~src.control.base_controller.BaseController` 接口，可在
``src.runner.build_controller`` 中通过 ``controller="mpc"`` 切换。
"""

from __future__ import annotations

from typing import Any, Dict, Optional

import numpy as np

from src.common.qp import QPSolution, solve_qp
from src.common.state import Trajectory, TrajectoryPoint, VehicleState
from src.control.base_controller import BaseController, ControlCommand

_TWO_PI = 2.0 * np.pi


def _wrap_angle(angle: float) -> float:
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


def _sample_trajectory(trajectory: Trajectory, t_query: float) -> TrajectoryPoint:
    """按时间戳对轨迹做线性插值（角度做 ±pi 环绕）。"""
    points = trajectory.points
    if len(points) == 1:
        return points[0]

    times = [p.t for p in points]
    if t_query <= times[0]:
        return points[0]
    if t_query >= times[-1]:
        return points[-1]

    lo, hi = 0, len(times) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if times[mid] <= t_query:
            lo = mid
        else:
            hi = mid

    t0, t1 = times[lo], times[hi]
    if abs(t1 - t0) < 1e-9:
        return points[hi]
    ratio = float((t_query - t0) / (t1 - t0))
    p0, p1 = points[lo], points[hi]
    theta = _wrap_angle(p0.theta + ratio * _wrap_angle(p1.theta - p0.theta))
    return TrajectoryPoint(
        x=float(p0.x + ratio * (p1.x - p0.x)),
        y=float(p0.y + ratio * (p1.y - p0.y)),
        theta=theta,
        kappa=float(p0.kappa + ratio * (p1.kappa - p0.kappa)),
        dkappa=float(p0.dkappa + ratio * (p1.dkappa - p0.dkappa)),
        v=float(p0.v + ratio * (p1.v - p0.v)),
        a=float(p0.a + ratio * (p1.a - p0.a)),
        jerk=float(p0.jerk + ratio * (p1.jerk - p0.jerk)),
        t=float(t_query),
        s=float(p0.s + ratio * (p1.s - p0.s)),
    )


class MPCController(BaseController):
    """
    基于 QP 的 LTV-MPC 轨迹跟踪控制器。

    参数中的权重遵循“代价 = 0.5 * V' P V + q' V”的内部形式，调用方只需
    按物理意义给出误差/控制权重即可。
    """

    def __init__(
        self,
        prediction_horizon: int = 20,
        control_horizon: int = 10,
        q_lateral: float = 10.0,
        q_heading: float = 6.0,
        q_speed: float = 1.0,
        r_accel: float = 0.08,
        r_steer: float = 0.8,
        r_accel_rate: float = 0.6,
        r_steer_rate: float = 6.0,
        lateral_error_limit: float = 0.9,
        heading_error_limit: float = 0.0,
        slack_weight: float = 2.0e3,
        slack_linear_weight: float = 2.0e4,
        wheelbase: float = 2.85,
        cf: float = 120000.0,
        cr: float = 120000.0,
        mass: float = 1600.0,
        iz: float = 2500.0,
        lf: float = 1.35,
        lr: float = 1.50,
        max_steer: float = 0.65,
        max_steer_rate: float = 0.50,
        max_accel: float = 3.5,
        max_decel: float = 7.0,
        max_jerk: float = 4.0,
        qp_max_iter: int = 2500,
        qp_eps_abs: float = 2e-5,
        qp_eps_rel: float = 2e-5,
        friction_coefficient: float = 0.9,
        max_lateral_accel: float = 4.0,
    ):
        self.Np = max(1, int(prediction_horizon))
        self.Nc = max(1, min(int(control_horizon), self.Np))
        self.q_lateral = float(q_lateral)
        self.q_heading = float(q_heading)
        self.q_speed = float(q_speed)
        self.r_accel = float(r_accel)
        self.r_steer = float(r_steer)
        self.r_accel_rate = float(r_accel_rate)
        self.r_steer_rate = float(r_steer_rate)
        self.lateral_error_limit = float(lateral_error_limit)
        self.heading_error_limit = float(heading_error_limit)
        self.slack_weight = float(slack_weight)
        self.slack_linear_weight = float(slack_linear_weight)

        self.wheelbase = float(wheelbase)
        self.cf = float(cf)
        self.cr = float(cr)
        self.mass = float(mass)
        self.iz = float(iz)
        self.lf = float(lf)
        self.lr = float(lr)
        self.max_steer = float(max_steer)
        self.max_steer_rate = float(max_steer_rate)
        self.max_accel = float(max_accel)
        self.max_decel = float(max_decel)
        self.max_jerk = float(max_jerk)
        self.friction_coefficient = float(friction_coefficient)
        self.max_lateral_accel = float(max_lateral_accel)

        self.qp_max_iter = int(qp_max_iter)
        self.qp_eps_abs = float(qp_eps_abs)
        self.qp_eps_rel = float(qp_eps_rel)

        self._prev_accel = 0.0
        self._prev_steer = 0.0
        self.last_solve_info: Dict[str, Any] = {}
        self._warm_start_u: Optional[np.ndarray] = None
        self._warm_start_x: Optional[np.ndarray] = None
        self._warm_start_y: Optional[np.ndarray] = None

    # ------------------------------------------------------------------
    # 参考轨迹与当前误差
    # ------------------------------------------------------------------
    @staticmethod
    def _reference_stack(
        trajectory: Trajectory, dt: float, horizon: int
    ) -> list[TrajectoryPoint]:
        return [_sample_trajectory(trajectory, (k + 1) * dt) for k in range(horizon)]

    @staticmethod
    def _current_errors(
        state: VehicleState, ref0: TrajectoryPoint
    ) -> tuple[float, float, float]:
        dx = state.x - ref0.x
        dy = state.y - ref0.y
        cos_t = float(np.cos(ref0.theta))
        sin_t = float(np.sin(ref0.theta))
        lateral = -sin_t * dx + cos_t * dy
        heading = _wrap_angle(state.heading - ref0.theta)
        speed = float(state.vx if abs(state.vx) > 1e-6 else state.velocity)
        speed_error = speed - max(0.0, ref0.v)
        return float(lateral), float(heading), float(speed_error)

    # ------------------------------------------------------------------
    # 预测矩阵与 QP 组装
    # ------------------------------------------------------------------
    def _build_prediction(
        self,
        x0: np.ndarray,
        refs: list[TrajectoryPoint],
    ):
        """返回 A_bar, B_bar, d_bar，使 X = A_bar x0 + B_bar U + d_bar。"""
        n_u = 2 * self.Nc
        a_rows = []
        b_rows = []
        d_rows = []

        A_cur = np.eye(3)
        B_cur = np.zeros((3, n_u))
        d_cur = np.zeros(3)
        for k in range(self.Np):
            ref = refs[k]
            v_ref = max(0.0, float(ref.v))
            kappa_ref = float(ref.kappa)
            a_ref = float(ref.a)
            dt = self._active_dt
            A_k = np.array(
                [
                    [1.0, dt * v_ref, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ]
            )
            B_k = np.array(
                [
                    [0.0, 0.0],
                    [0.0, dt * v_ref / max(self.wheelbase, 1e-3)],
                    [dt, 0.0],
                ]
            )
            d_k = np.array([0.0, -dt * v_ref * kappa_ref, -dt * a_ref])

            A_cur = np.dot(A_k, A_cur)
            B_cur = np.dot(A_k, B_cur)
            d_cur = np.dot(A_k, d_cur) + d_k

            if k < self.Nc:
                B_cur[:, k] += B_k[:, 0]
                B_cur[:, self.Nc + k] += B_k[:, 1]
            else:
                B_cur[:, self.Nc - 1] += B_k[:, 0]
                B_cur[:, 2 * self.Nc - 1] += B_k[:, 1]

            a_rows.append(A_cur.copy())
            b_rows.append(B_cur.copy())
            d_rows.append(d_cur.copy())

        return (
            np.vstack(a_rows),
            np.vstack(b_rows),
            np.concatenate(d_rows),
        )

    def _assemble_qp(
        self,
        const_state: np.ndarray,
        B_bar: np.ndarray,
        refs: list[TrajectoryPoint],
    ) -> tuple[np.ndarray, np.ndarray, Optional[np.ndarray], np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """组装 P, q, A_ineq, b_ineq, lb, ub, u_ref。"""
        state_dim = 3
        n_u = 2 * self.Nc
        n_slack = self.Np
        n_var = n_u + n_slack

        Q = np.diag([self.q_lateral, self.q_heading, self.q_speed])
        Q_bar = np.kron(np.eye(self.Np), Q)
        P_state = 2.0 * np.dot(B_bar.T, np.dot(Q_bar, B_bar))
        q_state = 2.0 * np.dot(B_bar.T, np.dot(Q_bar, const_state))

        # 控制量参考：加速度参考规划值，转角参考曲率前馈
        a_ref = np.zeros(self.Nc)
        delta_ref = np.zeros(self.Nc)
        for k in range(self.Nc):
            ref = refs[min(k, len(refs) - 1)]
            a_ref[k] = float(ref.a)
            delta_ref[k] = float(np.arctan(self.wheelbase * ref.kappa))

        P_u = np.zeros((n_u, n_u))
        q_u = np.zeros(n_u)
        for k in range(self.Nc):
            P_u[k, k] = 2.0 * self.r_accel
            q_u[k] = -2.0 * self.r_accel * a_ref[k]
            P_u[self.Nc + k, self.Nc + k] = 2.0 * self.r_steer
            q_u[self.Nc + k] = -2.0 * self.r_steer * delta_ref[k]

        # 控制增量代价：a_k - a_{k-1}, delta_k - delta_{k-1}
        diff = np.zeros((self.Nc, self.Nc))
        for k in range(self.Nc):
            diff[k, k] = 1.0
            if k > 0:
                diff[k, k - 1] = -1.0
        D_rate = np.zeros((n_u, n_u))
        D_rate[: self.Nc, : self.Nc] = diff
        D_rate[self.Nc :, self.Nc :] = diff
        W_rate = np.diag(
            [self.r_accel_rate] * self.Nc + [self.r_steer_rate] * self.Nc
        )
        d0 = np.zeros(n_u)
        d0[0] = self._prev_accel
        d0[self.Nc] = self._prev_steer
        P_rate = 2.0 * np.dot(D_rate.T, np.dot(W_rate, D_rate))
        q_rate = -2.0 * np.dot(D_rate.T, np.dot(W_rate, d0))

        P = np.zeros((n_var, n_var))
        q = np.zeros(n_var)
        P[:n_u, :n_u] = P_state + P_u + P_rate
        q[:n_u] = q_state + q_u + q_rate
        # slack 变量做列缩放：s_phys = slack_scale * s_tilde，使 Hessian 对角
        # 与约束行系数处于同一量级，避免大罚因子导致 ADMM 条件数恶化。
        slack_scale = 1.0 / np.sqrt(max(2.0 * self.slack_weight, 1e-9))
        P[n_u:, n_u:] = np.eye(n_slack)
        q[n_u:] = self.slack_linear_weight * slack_scale

        # 线性不等式 A_ineq V <= b_ineq
        rows = []
        upper = []
        if self.lateral_error_limit > 0.0 or self.heading_error_limit > 0.0:
            for k in range(self.Np):
                base = state_dim * k
                if self.lateral_error_limit > 0.0:
                    b_y = B_bar[base, :]
                    const_y = float(const_state[base])
                    row = np.zeros(n_var)
                    row[:n_u] = b_y
                    row[n_u + k] = -slack_scale
                    rows.append(row.copy())
                    upper.append(self.lateral_error_limit - const_y)
                    rows.append(-row)
                    upper.append(self.lateral_error_limit + const_y)
                if self.heading_error_limit > 0.0:
                    b_psi = B_bar[base + 1, :]
                    const_psi = float(const_state[base + 1])
                    row = np.zeros(n_var)
                    row[:n_u] = b_psi
                    row[n_u + k] = -slack_scale
                    rows.append(row.copy())
                    upper.append(self.heading_error_limit - const_psi)
                    rows.append(-row)
                    upper.append(self.heading_error_limit + const_psi)

        # 控制增量硬约束（绝对值拆成两条不等式）
        max_da = max(self.max_jerk, 0.0) * self._active_dt
        max_ddelta = max(self.max_steer_rate, 0.0) * self._active_dt
        if max_da > 0.0:
            for k in range(self.Nc):
                row = np.zeros(n_var)
                row[:n_u] = D_rate[k, :]
                rows.append(row.copy())
                upper.append(max_da + d0[k])
                rows.append(-row)
                upper.append(max_da - d0[k])
        if max_ddelta > 0.0:
            for k in range(self.Nc):
                idx = self.Nc + k
                row = np.zeros(n_var)
                row[:n_u] = D_rate[idx, :]
                rows.append(row.copy())
                upper.append(max_ddelta + d0[idx])
                rows.append(-row)
                upper.append(max_ddelta - d0[idx])

        if rows:
            A_ineq = np.vstack(rows)
            b_ineq = np.asarray(upper, dtype=float)
        else:
            A_ineq = None
            b_ineq = np.zeros(0)

        lb = np.full(n_var, -np.inf)
        ub = np.full(n_var, np.inf)
        lb[: self.Nc] = -self.max_decel
        ub[: self.Nc] = self.max_accel
        lb[self.Nc : n_u] = -self.max_steer
        ub[self.Nc : n_u] = self.max_steer
        lb[n_u:] = 0.0

        return P, q, A_ineq, b_ineq, lb, ub, np.concatenate([a_ref, delta_ref])

    # ------------------------------------------------------------------
    # 主控制律
    # ------------------------------------------------------------------
    def compute_control(
        self,
        current_state: VehicleState,
        target_trajectory: Trajectory,
        dt: float,
    ) -> ControlCommand:
        """
        滚动时域求解并返回首个控制量。

        :param current_state: 当前车辆状态
        :param target_trajectory: 规划轨迹（点带 t/v/a/kappa/theta）
        :param dt: 控制周期 (s)
        :return: 前轮转角、期望加速度与油门/制动开度
        """
        self._active_dt = max(1e-3, float(dt))
        if not target_trajectory.points:
            self.last_solve_info = {"status": "empty_trajectory"}
            self._warm_start_u = None
            return ControlCommand(acceleration=-2.0, brake=0.4)

        refs = self._reference_stack(target_trajectory, self._active_dt, self.Np)
        ref0 = _sample_trajectory(target_trajectory, 0.0)
        ey, epsi, ev = self._current_errors(current_state, ref0)
        x0 = np.array([ey, epsi, ev], dtype=float)

        A_bar, B_bar, d_bar = self._build_prediction(x0, refs)
        const_state = np.dot(A_bar, x0) + d_bar
        P, q, A_ineq, b_ineq, lb, ub, u_ref = self._assemble_qp(
            const_state, B_bar, refs
        )

        n_u = 2 * self.Nc
        n_slack = self.Np
        n_var = n_u + n_slack

        x0_qp = None
        y0_qp = None

        if self._warm_start_x is not None and self._warm_start_x.size == n_var:
            # 滚动时域热启动预测序列向前平移 1 步 (Receding-Horizon Warm-Start Shift)
            prev_a = self._warm_start_x[: self.Nc]
            shifted_a = np.empty_like(prev_a)
            shifted_a[:-1] = prev_a[1:]
            shifted_a[-1] = prev_a[-1]

            prev_delta = self._warm_start_x[self.Nc : n_u]
            shifted_delta = np.empty_like(prev_delta)
            shifted_delta[:-1] = prev_delta[1:]
            shifted_delta[-1] = prev_delta[-1]

            prev_s = self._warm_start_x[n_u:]
            shifted_s = np.empty_like(prev_s)
            shifted_s[:-1] = prev_s[1:]
            shifted_s[-1] = 0.0

            x0_qp = np.concatenate([shifted_a, shifted_delta, shifted_s])
        elif self._warm_start_u is not None and self._warm_start_u.size == n_u:
            x0_qp = np.concatenate([self._warm_start_u, np.zeros(self.Np)])

        # 对偶变量热启动匹配：若上一周期的对偶变量维度与本周期一致，直接用于 ADMM 初值
        m_ineq = A_ineq.shape[0] if A_ineq is not None else 0
        m_total = m_ineq + n_var
        if self._warm_start_y is not None and self._warm_start_y.size == m_total:
            y0_qp = self._warm_start_y.copy()

        solution: QPSolution = solve_qp(
            P,
            q,
            A=A_ineq,
            u=b_ineq if A_ineq is not None else None,
            lb=lb,
            ub=ub,
            x_init=x0_qp,
            y_init=y0_qp,
            max_iter=self.qp_max_iter,
            eps_abs=self.qp_eps_abs,
            eps_rel=self.qp_eps_rel,
        )

        qp_ok = bool(
            solution.success
            and np.all(np.isfinite(solution.x))
            and solution.constraint_violation <= 2e-3
        )
        if qp_ok:
            u_seq = solution.x[:n_u].copy()
            accel_cmd = float(u_seq[0])
            steer_cmd = float(u_seq[self.Nc])
            self._warm_start_u = u_seq
            self._warm_start_x = solution.x.copy()
            self._warm_start_y = solution.y.copy() if solution.y is not None else None
        else:
            accel_cmd, steer_cmd = self._fallback_control(
                current_state, ref0, ey, epsi, ev
            )
            self._warm_start_u = None
            self._warm_start_x = None
            self._warm_start_y = None

        # 执行机构硬约束 + 指令速率限制（即使 QP 失败也保证安全）
        max_delta_a = max(self.max_jerk, 0.0) * self._active_dt
        accel_cmd = float(
            np.clip(
                accel_cmd,
                self._prev_accel - max_delta_a,
                self._prev_accel + max_delta_a,
            )
        )
        accel_cmd = float(np.clip(accel_cmd, -self.max_decel, self.max_accel))

        # 轮胎侧向附着裕度监测与转向速率软保护
        vx_now = max(1.0, float(current_state.vx if abs(current_state.vx) > 1e-6 else current_state.velocity))
        curr_ay = float(current_state.ay if hasattr(current_state, "ay") else 0.0)
        kinematic_ay = float(current_state.yaw_rate * vx_now)
        a_lat_est = max(abs(curr_ay), abs(kinematic_ay))

        g = 9.81
        mu_g = max(0.1, self.friction_coefficient * g)
        adhesion_ratio = a_lat_est / mu_g

        # 当侧向附着利用率接近极限时平滑限幅转向速率，避免剧烈打方向导致侧滑失稳
        if adhesion_ratio > 0.65:
            rate_scale = float(np.clip(1.0 - 1.8 * (adhesion_ratio - 0.65), 0.20, 1.0))
        else:
            rate_scale = 1.0

        max_delta_steer = max(self.max_steer_rate, 0.0) * self._active_dt * rate_scale

        if self.max_lateral_accel is not None and self.max_lateral_accel > 0.0:
            safe_a_lat_max = min(self.max_lateral_accel, mu_g * 0.9)
            adhesion_steer_limit = float(np.arctan(self.wheelbase * safe_a_lat_max / (vx_now ** 2)))
            # 与 LQR 一致：数值下限只防止 0 除，不能抬高到 0.05 rad。
            effective_max_steer = min(self.max_steer, max(adhesion_steer_limit, 1e-3))
        else:
            effective_max_steer = self.max_steer

        steer_cmd = float(
            np.clip(
                steer_cmd,
                self._prev_steer - max_delta_steer,
                self._prev_steer + max_delta_steer,
            )
        )
        steer_cmd = float(np.clip(steer_cmd, -effective_max_steer, effective_max_steer))

        steer_rate = (steer_cmd - self._prev_steer) / self._active_dt
        self._prev_accel = accel_cmd
        self._prev_steer = steer_cmd
        self.last_solve_info = {
            "status": solution.status,
            "success": qp_ok,
            "iterations": int(solution.iterations),
            "primal_residual": float(solution.primal_residual),
            "dual_residual": float(solution.dual_residual),
            "constraint_violation": float(solution.constraint_violation),
            "objective": float(solution.objective),
            "horizon": self.Np,
            "control_horizon": self.Nc,
            "u_ref": u_ref.tolist(),
        }

        throttle = max(0.0, accel_cmd) / max(1e-6, self.max_accel)
        brake = max(0.0, -accel_cmd) / max(1e-6, self.max_decel)
        return ControlCommand(
            steering_angle=steer_cmd,
            acceleration=accel_cmd,
            throttle=float(np.clip(throttle, 0.0, 1.0)),
            brake=float(np.clip(brake, 0.0, 1.0)),
            steering_rate=float(steer_rate),
        )

    def _fallback_control(
        self,
        state: VehicleState,
        ref0: TrajectoryPoint,
        ey: float,
        epsi: float,
        ev: float,
    ) -> tuple[float, float]:
        """QP 不可用时的保守回退：曲率前馈 + 横向 PD + 纵向 P。"""
        delta_ref = float(np.arctan(self.wheelbase * ref0.kappa))
        steer = delta_ref - 0.9 * epsi - 0.35 * ey
        accel = 0.8 * (-ev) + float(ref0.a)
        return float(accel), float(steer)

    def reset(self) -> None:
        """重置控制器内部状态（新场景开始时调用）。"""
        self._prev_accel = 0.0
        self._prev_steer = 0.0
        self._warm_start_u = None
        self._warm_start_x = None
        self._warm_start_y = None
        self.last_solve_info = {}


__all__ = ["MPCController"]
