"""
Frenet 时空坐标系下的 Lattice 多项式轨迹规划器 (Frenet Lattice Planner)。
经典算法参考：Moritz Werling et al. "Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame", IEEE ICRA 2010.
"""

from typing import List
import numpy as np
from src.common.state import VehicleState, ObstacleState, Trajectory, TrajectoryPoint
from src.common.frenet_frame import ReferencePoint, FrenetConverter, FrenetPoint
from src.decision.base_decision import DecisionOutput, DrivingIntent
from src.planning.base_planner import BasePlanner
from src.planning.cost_evaluator import TrajectoryCostEvaluator


class QuinticPolynomial:
    """五次多项式曲线: x(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5"""
    def __init__(self, xs: float, vxs: float, axs: float, xe: float, vxe: float, axe: float, T: float):
        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([
            [T**3, T**4, T**5],
            [3 * T**2, 4 * T**3, 5 * T**4],
            [6 * T, 12 * T**2, 20 * T**3]
        ])
        b = np.array([
            xe - self.a0 - self.a1 * T - self.a2 * T**2,
            vxe - self.a1 - 2 * self.a2 * T,
            axe - 2 * self.a2
        ])
        try:
            x = np.linalg.solve(A, b)
            self.a3 = x[0]
            self.a4 = x[1]
            self.a5 = x[2]
        except np.linalg.LinAlgError:
            self.a3 = self.a4 = self.a5 = 0.0

    def calc_point(self, t: float) -> float:
        return self.a0 + self.a1 * t + self.a2 * t**2 + self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5

    def calc_first_derivative(self, t: float) -> float:
        return self.a1 + 2 * self.a2 * t + 3 * self.a3 * t**2 + 4 * self.a4 * t**3 + 5 * self.a5 * t**4

    def calc_second_derivative(self, t: float) -> float:
        return 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2 + 20 * self.a5 * t**3

    def calc_third_derivative(self, t: float) -> float:
        return 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t**2


class FrenetLatticePlanner(BasePlanner):
    """
    Frenet Lattice 采样轨迹规划器
    """
    def __init__(self):
        self.cost_evaluator = TrajectoryCostEvaluator()

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
        采样纵向时序与横向偏移，生成候选轨迹簇，评估并筛选全局最优无碰撞且动力学平滑轨迹
        """
        # 1. 自车状态变换至 Frenet 坐标
        ego_frenet = FrenetConverter.cartesian_to_frenet(ego_state, reference_line)

        # 2. 横向与纵向末态离散采样空间 (Sampling Space)
        # 横向目标偏移 l 采样: 居中 0, 左偏, 右偏
        target_l_samples = [decision.lateral_shift_l, decision.lateral_shift_l - 0.5, decision.lateral_shift_l + 0.5]
        # 时间窗采样 T: [3.5s, 4.0s, 4.5s, 5.0s]
        t_samples = [3.5, 4.0, 5.0]

        candidate_trajectories: List[Trajectory] = []

        # 3. 多项式曲线解算与离散化
        for t_horizon in t_samples:
            for l_target in target_l_samples:
                # 生成横向五次多项式
                poly_l = QuinticPolynomial(
                    xs=ego_frenet.l, vxs=ego_frenet.l_d, axs=ego_frenet.l_dd,
                    xe=l_target, vxe=0.0, axe=0.0, T=t_horizon
                )

                # 生成纵向多项式 (巡航模式下以期望速度推进)
                s_end = ego_frenet.s + decision.target_speed * t_horizon
                poly_s = QuinticPolynomial(
                    xs=ego_frenet.s, vxs=ego_frenet.s_d, axs=ego_frenet.s_dd,
                    xe=s_end, vxe=decision.target_speed, axe=0.0, T=t_horizon
                )

                # 合成笛卡尔空间轨迹序列
                points: List[TrajectoryPoint] = []
                steps = int(t_horizon / dt)
                for step_idx in range(steps):
                    t = step_idx * dt
                    s_t = poly_s.calc_point(t)
                    s_d_t = poly_s.calc_first_derivative(t)
                    s_dd_t = poly_s.calc_second_derivative(t)
                    l_t = poly_l.calc_point(t)
                    l_d_t = poly_l.calc_first_derivative(t)

                    f_pt = FrenetPoint(s=s_t, s_d=s_d_t, s_dd=s_dd_t, l=l_t, l_d=l_d_t)
                    # 查找对应参考线点并转换
                    # 简化的直线参考映射示例:
                    ref_dummy = ReferencePoint(x=s_t, y=0.0, theta=0.0, kappa=0.0, s=s_t)
                    c_pt = FrenetConverter.frenet_to_cartesian(f_pt, ref_dummy)
                    c_pt.t = t
                    points.append(c_pt)

                traj = Trajectory(points=points)
                self.cost_evaluator.evaluate(traj, obstacles, decision.target_speed)
                candidate_trajectories.append(traj)

        # 4. 选取代价最低的最优轨迹
        best_trajectory = min(candidate_trajectories, key=lambda t: t.total_cost, default=Trajectory())
        return best_trajectory
