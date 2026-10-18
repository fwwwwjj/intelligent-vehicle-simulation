"""
离散线性二次型调节器 (Discrete Linear Quadratic Regulator, LQR) 横向轨迹跟踪控制器。
具备横向跟踪误差消除、基于曲率的前馈转向补偿 (Curvature Feedforward) 以及
按轨迹时间索引的前视跟踪 (Time-indexed Preview Tracking)。

纵向控制复用 :class:`LongitudinalPIDController`，跟踪规划轨迹末端的期望速度。
scipy 可用时使用 ``scipy.linalg.solve_discrete_are``；不可用时退化为 numpy
Riccati 迭代，保证核心算法在最小依赖环境下仍可测试与复现。
"""

from typing import Optional

import numpy as np

from src.common.state import VehicleState, Trajectory, TrajectoryPoint
from src.control.base_controller import BaseController, ControlCommand
from src.control.pid_controller import LongitudinalPIDController

try:  # pragma: no cover - 取决于运行环境
    from scipy import linalg as _scipy_linalg
except Exception:  # pragma: no cover
    _scipy_linalg = None


def _solve_discrete_are_numpy(
    a: np.ndarray,
    b: np.ndarray,
    q: np.ndarray,
    r: np.ndarray,
    max_iter: int = 2000,
    tol: float = 1e-10,
) -> np.ndarray:
    """用 Riccati 不动点迭代求解离散代数黎卡提方程（scipy 缺失时的回退）。"""
    p = q.copy()
    for _ in range(max_iter):
        btp = b.T @ p
        p_next = a.T @ p @ a - (a.T @ p @ b) @ np.linalg.solve(
            r + btp @ b, btp @ a
        ) + q
        if np.max(np.abs(p_next - p)) < tol:
            return p_next
        p = p_next
    return p


class LQRController(BaseController):
    """
    基于二自由度误差动力学方程的 LQR 横向控制器
    误差状态: e = [e_y, e_y_dot, e_psi, e_psi_dot]^T

    轨迹点按 ``TrajectoryPoint.t`` 递增排列。控制器选取当前时刻向前
    ``lookahead_time`` 秒的预瞄点计算横向/航向误差；纵向则跟踪轨迹末端的
    期望速度，保证决策层的 FOLLOW / EMERGENCY_STOP 能真正作用到车辆上。
    """

    def __init__(
        self,
        q_matrix_diag=(2.0, 0.1, 1.5, 0.1),
        r_weight: float = 1.0,
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
        max_lateral_accel: float = 2.5,
        lookahead_time: float = 0.60,
    ):
        self.Q = np.diag(q_matrix_diag)
        self.R = np.array([[r_weight]])
        self.wheelbase = wheelbase
        self.cf = cf
        self.cr = cr
        self.mass = mass
        self.iz = iz
        self.lf = lf
        self.lr = lr
        self.max_steer = max_steer
        self.max_steer_rate = max_steer_rate
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.max_jerk = max_jerk
        self.max_lateral_accel = max_lateral_accel
        self.lookahead_time = lookahead_time

        self.longitudinal_controller = LongitudinalPIDController(
            kp=0.8, ki=0.1, kd=0.0, max_accel=max_accel, max_decel=max_decel
        )
        self._v_ref_filtered: Optional[float] = None
        self._prev_steer = 0.0
        self._prev_accel = 0.0
        self._dare_cache: dict = {}

    # ------------------------------------------------------------------
    # 轨迹索引
    # ------------------------------------------------------------------
    @staticmethod
    def _point_at_time(trajectory: Trajectory, t_query: float) -> TrajectoryPoint:
        """返回时间最接近 ``t_query`` 的轨迹点（轨迹点按 t 递增）。"""
        points = trajectory.points
        if len(points) == 1:
            return points[0]
        times = [p.t for p in points]
        if t_query <= times[0]:
            return points[0]
        if t_query >= times[-1]:
            return points[-1]
        # 二分查找最近邻
        lo, hi = 0, len(times) - 1
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if times[mid] <= t_query:
                lo = mid
            else:
                hi = mid
        return points[lo] if abs(times[lo] - t_query) <= abs(times[hi] - t_query) else points[hi]

    def _dare_gain(self, a_d: np.ndarray, b_d: np.ndarray) -> np.ndarray:
        key = (round(float(np.linalg.norm(a_d)), 6), round(float(np.linalg.norm(b_d)), 6))
        cached = self._dare_cache.get(key)
        if cached is not None:
            return cached

        if _scipy_linalg is not None:
            try:
                p = _scipy_linalg.solve_discrete_are(a_d, b_d, self.Q, self.R)
            except Exception:
                p = _solve_discrete_are_numpy(a_d, b_d, self.Q, self.R)
        else:
            p = _solve_discrete_are_numpy(a_d, b_d, self.Q, self.R)

        k = np.linalg.solve(self.R + b_d.T @ p @ b_d, b_d.T @ p @ a_d)
        self._dare_cache[key] = k
        if len(self._dare_cache) > 64:
            self._dare_cache.pop(next(iter(self._dare_cache)))
        return k

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
        计算前轮转角与纵向加速度指令。

        :param current_state: 当前车辆状态
        :param target_trajectory: 规划层下发的轨迹（点带 t / theta / kappa / v / a）
        :param dt: 控制周期 (s)
        :return: 控制指令
        """
        if not target_trajectory.points:
            self.longitudinal_controller.reset()
            return ControlCommand(acceleration=-2.0, brake=0.4)

        # 1. 预瞄点选择：按时间索引，而不是永远取 points[0]
        preview_t = max(0.0, min(self.lookahead_time, target_trajectory.points[-1].t))
        target_pt = self._point_at_time(target_trajectory, preview_t)

        # 2. 横向/航向误差
        dx = current_state.x - target_pt.x
        dy = current_state.y - target_pt.y
        cos_t = np.cos(target_pt.theta)
        sin_t = np.sin(target_pt.theta)
        ey = -sin_t * dx + cos_t * dy
        epsi = current_state.heading - target_pt.theta
        epsi = float(np.arctan2(np.sin(epsi), np.cos(epsi)))

        # 3. 误差动力学 (2-DOF) 与 DARE
        vx = max(1.0, current_state.vx)
        a = np.zeros((4, 4))
        a[0, 1] = 1.0
        a[1, 1] = -(self.cf + self.cr) / (self.mass * vx)
        a[1, 2] = (self.cf + self.cr) / self.mass
        a[1, 3] = (-self.lf * self.cf + self.lr * self.cr) / (self.mass * vx)
        a[2, 3] = 1.0
        a[3, 1] = -(self.lf * self.cf - self.lr * self.cr) / (self.iz * vx)
        a[3, 2] = (self.lf * self.cf - self.lr * self.cr) / self.iz
        a[3, 3] = -(self.lf**2 * self.cf + self.lr**2 * self.cr) / (self.iz * vx)

        b = np.zeros((4, 1))
        b[1, 0] = self.cf / self.mass
        b[3, 0] = self.lf * self.cf / self.iz

        a_d = np.eye(4) + a * dt
        b_d = b * dt
        k = self._dare_gain(a_d, b_d)

        # e_y_dot 用“相对路径切向的横向速度”而不是车身 vy：
        # 运动学模型里 vy 恒为 0，若直接使用会让 LQR 丢失横向阻尼。
        ey_dot = current_state.velocity * float(np.sin(epsi))
        epsi_dot = current_state.yaw_rate - target_pt.kappa * current_state.velocity
        error_vec = np.array([ey, ey_dot, epsi, epsi_dot], dtype=float)
        steer_fb = float(-(k @ error_vec)[0])

        # 4. 曲率前馈 + 横向加速度/转向速率限制
        prev_steer = self._prev_steer
        steer_ff = float(target_pt.kappa * self.wheelbase)
        v_now = max(1.0, current_state.velocity)
        # 运动学近似下，a_lat = v^2 * tan(delta) / L，据此反解舒适转向上限
        lat_steer_limit = float(np.arctan(self.wheelbase * self.max_lateral_accel / (v_now ** 2)))
        steer_limit = min(self.max_steer, max(lat_steer_limit, 0.005))
        steer_target = float(np.clip(steer_fb + steer_ff, -steer_limit, steer_limit))
        max_delta = self.max_steer_rate * dt
        steer_cmd = float(
            np.clip(steer_target, prev_steer - max_delta, prev_steer + max_delta)
        )
        steer_rate = (steer_cmd - prev_steer) / max(dt, 1e-6)
        self._prev_steer = steer_cmd

        # 5. 纵向：跟踪轨迹末端目标速度（决策层的期望速度会传导到这里）
        v_ref_raw = float(target_trajectory.points[-1].v)
        # 感知噪声会传导到决策目标速度；低通滤波避免纵向指令逐帧抖动
        if self._v_ref_filtered is None:
            self._v_ref_filtered = v_ref_raw
        else:
            self._v_ref_filtered = 0.85 * self._v_ref_filtered + 0.15 * v_ref_raw
        v_ref = float(self._v_ref_filtered)
        a_ff = float(target_trajectory.points[-1].a)
        if current_state.velocity < 0.1 and v_ref < 0.1:
            # 静止跟停：清除积分，避免停车后持续输出负加速度
            self.longitudinal_controller.reset()
            accel_cmd = 0.0
        else:
            accel_cmd = self.longitudinal_controller.compute(v_ref, current_state.velocity, dt)
            accel_cmd = float(np.clip(accel_cmd + a_ff, -self.max_decel, self.max_accel))
        # 加加速度限制：避免目标速度切换时出现 -100 m/s^3 的指令阶跃
        max_delta_a = self.max_jerk * dt
        accel_cmd = float(
            np.clip(
                accel_cmd,
                self._prev_accel - max_delta_a,
                self._prev_accel + max_delta_a,
            )
        )
        self._prev_accel = accel_cmd

        throttle = max(0.0, accel_cmd) / max(1e-6, self.max_accel)
        brake = max(0.0, -accel_cmd) / max(1e-6, self.max_decel)

        return ControlCommand(
            steering_angle=steer_cmd,
            acceleration=accel_cmd,
            throttle=float(np.clip(throttle, 0.0, 1.0)),
            brake=float(np.clip(brake, 0.0, 1.0)),
            steering_rate=float(steer_rate),
        )

    def reset(self) -> None:
        """重置控制器内部状态（新场景开始时调用）。"""
        self._prev_steer = 0.0
        self._prev_accel = 0.0
        self._v_ref_filtered = None
        self._dare_cache.clear()
        self.longitudinal_controller.reset()
