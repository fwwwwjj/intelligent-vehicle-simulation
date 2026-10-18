"""
离散线性二次型调节器 (Discrete Linear Quadratic Regulator, LQR) 横向轨迹跟踪控制器。
具备横向跟踪误差消除与基于曲率的前馈转向补偿 (Curvature Feedforward)。
"""

import numpy as np
from scipy import linalg
from src.common.state import VehicleState, Trajectory
from src.control.base_controller import BaseController, ControlCommand


class LQRController(BaseController):
    """
    基于二自由度误差动力学方程的 LQR 横向控制器
    误差状态: e = [e_y, e_y_dot, e_psi, e_psi_dot]^T
    """
    def __init__(
        self,
        q_matrix_diag=(2.0, 0.1, 1.5, 0.1),
        r_weight=1.0,
        wheelbase: float = 2.85,
        cf: float = 120000.0,
        cr: float = 120000.0,
        mass: float = 1600.0,
        iz: float = 2500.0,
        lf: float = 1.35,
        lr: float = 1.50
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

    def compute_control(
        self,
        current_state: VehicleState,
        target_trajectory: Trajectory,
        dt: float
    ) -> ControlCommand:
        """
        求解代数黎卡提方程 (DARE)，计算最优状态反馈增益 K，并融合曲率前馈
        """
        if not target_trajectory.points:
            return ControlCommand()

        # 匹配最近参考轨迹点
        target_pt = target_trajectory.points[0]
        dx = current_state.x - target_pt.x
        dy = current_state.y - target_pt.y
        ey = -np.sin(target_pt.theta) * dx + np.cos(target_pt.theta) * dy
        epsi = current_state.heading - target_pt.theta
        epsi = float(np.arctan2(np.sin(epsi), np.cos(epsi)))

        vx = max(1.0, current_state.vx)
        # 连续状态空间矩阵 A, B 离散化
        A = np.zeros((4, 4))
        A[0, 1] = 1.0
        A[1, 1] = -(self.cf + self.cr) / (self.mass * vx)
        A[1, 2] = (self.cf + self.cr) / self.mass
        A[1, 3] = (-self.lf * self.cf + self.lr * self.cr) / (self.mass * vx)
        A[2, 3] = 1.0
        A[3, 1] = -(self.lf * self.cf - self.lr * self.cr) / (self.iz * vx)
        A[3, 2] = (self.lf * self.cf - self.lr * self.cr) / self.iz
        A[3, 3] = -(self.lf**2 * self.cf + self.lr**2 * self.cr) / (self.iz * vx)

        B = np.zeros((4, 1))
        B[1, 0] = self.cf / self.mass
        B[3, 0] = self.lf * self.cf / self.iz

        # 欧拉离散化
        Ad = np.eye(4) + A * dt
        Bd = B * dt

        # 求解离散黎卡提方程 P
        try:
            P = linalg.solve_discrete_are(Ad, Bd, self.Q, self.R)
            K = np.linalg.inv(self.R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)
        except Exception:
            K = np.zeros((1, 4))

        # 反馈控制量 u_fb = -K * e
        error_vec = np.array([ey, current_state.vy, epsi, current_state.yaw_rate])
        steer_fb = float(-K @ error_vec)

        # 曲率前馈转向补偿 delta_ff = kappa * (L - lr * K[2] + ...)
        steer_ff = target_pt.kappa * self.wheelbase
        steer_cmd = steer_fb + steer_ff

        # 纵向速度比例前馈补偿
        accel_cmd = 1.0 * (target_pt.v - current_state.velocity) + target_pt.a

        return ControlCommand(
            steering_angle=float(steer_cmd),
            acceleration=float(accel_cmd)
        )
