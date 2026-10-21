"""
Frenet 时空坐标系下的 Lattice 多项式轨迹规划器 (Frenet Lattice Planner)。
经典算法参考：Moritz Werling et al. "Optimal Trajectory Generation for Dynamic Street Scenarios
in a Frenet Frame", IEEE ICRA 2010.

本实现相对早期骨架的关键修正：
1. 使用真实参考线（按弧长插值）做 Frenet -> Cartesian，不再用 y=0 的假参考点；
2. 纵向五次多项式以“起点速度 -> 目标速度”的平均速度估计终点，紧急停车不会
   出现 s 不单调或负速度；
3. 每个轨迹点填充 theta / kappa / v / a / jerk，供控制器和代价函数使用；
4. 候选轨迹按时间投影障碍物做 OBB 碰撞校验，碰撞轨迹标记 invalid 并重罚。
"""

from typing import List, Optional, Tuple

import numpy as np

from src.common.state import VehicleState, ObstacleState, Trajectory, TrajectoryPoint
from src.common.frenet_frame import ReferencePoint, FrenetConverter, FrenetPoint
from src.decision.base_decision import DecisionOutput
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
            [6 * T, 12 * T**2, 20 * T**3],
        ])
        b = np.array([
            xe - self.a0 - self.a1 * T - self.a2 * T**2,
            vxe - self.a1 - 2 * self.a2 * T,
            axe - 2 * self.a2,
        ])
        try:
            x = np.linalg.solve(A, b)
            self.a3 = float(x[0])
            self.a4 = float(x[1])
            self.a5 = float(x[2])
        except np.linalg.LinAlgError:
            self.a3 = self.a4 = self.a5 = 0.0
        self.T = float(T)

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

    def __init__(
        self,
        cost_evaluator: Optional[TrajectoryCostEvaluator] = None,
        ego_length: float = 4.8,
        ego_width: float = 1.9,
        max_accel: float = 3.5,
        max_decel: float = 7.0,
        lane_half_width: float = 1.875,
        shoulder_margin: float = 0.30,
    ):
        self.cost_evaluator = cost_evaluator or TrajectoryCostEvaluator(
            ego_length=ego_length,
            ego_width=ego_width,
            max_accel=max_accel,
            max_decel=max_decel,
        )
        self.ego_length = ego_length
        self.ego_width = ego_width
        self.max_accel = max_accel
        self.max_decel = max_decel
        # 真实车道半宽（3.75 m 车道 → 1.875 m）。横向候选必须限制在
        # "半宽 + 路肩"以内，否则规划器会主动生成一条已被判定为"驶出道路"的轨迹。
        self.lane_half_width = float(lane_half_width)
        self.shoulder_margin = float(shoulder_margin)
        self.max_curvature = float(np.tan(0.65) / 2.85)

    # ------------------------------------------------------------------
    # 参考线兜底
    # ------------------------------------------------------------------
    @staticmethod
    def _fallback_reference_line(ego_state: VehicleState, length: float = 120.0) -> List[ReferencePoint]:
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
        # 停车/重合轨迹点上 ds 可能为 0；np.gradient 会除零并产生 NaN 警告。
        # 这种情况下退化为 0，曲率本身仍由上面的差分给出。
        if n > 2 and np.all(np.diff(s_array) >= 1e-6):
            dkappas = np.gradient(kappas, s_array)
        else:
            dkappas = np.zeros(n, dtype=float)

        for i, pt in enumerate(points):
            pt.kappa = float(kappas[i])
            pt.dkappa = float(dkappas[i])

    # ------------------------------------------------------------------
    # 单条候选轨迹
    # ------------------------------------------------------------------
    def _build_candidate(
        self,
        ego_frenet: FrenetPoint,
        reference_line: List[ReferencePoint],
        target_speed: float,
        target_l: float,
        horizon: float,
        dt: float,
    ) -> Optional[Trajectory]:
        v0 = max(0.0, ego_frenet.s_d)
        target_speed = max(0.0, target_speed)
        # 用平均速度估计终点，避免停车目标时 s_end == s_start 导致非单调
        s_end = ego_frenet.s + 0.5 * (v0 + target_speed) * horizon
        if s_end < ego_frenet.s:
            s_end = ego_frenet.s

        poly_s = QuinticPolynomial(
            xs=ego_frenet.s,
            vxs=v0,
            axs=ego_frenet.s_dd,
            xe=s_end,
            vxe=target_speed,
            axe=0.0,
            T=horizon,
        )
        # 横向多项式的初始侧向速度取“朝目标车道的期望换道率”，而不是
        # v*sin(Δθ)。后者会把航向误差当成换道初速度，逐帧重规划形成正反馈；
        # 取 0 又会让每次重规划都从静止起步，5s 内几乎不产生横向位移。
        lateral_error = target_l - ego_frenet.l
        l_d0 = float(np.clip(0.8 * lateral_error, -1.2, 1.2))
        poly_l = QuinticPolynomial(
            xs=ego_frenet.l,
            vxs=l_d0,
            axs=0.0,
            xe=target_l,
            vxe=0.0,
            axe=0.0,
            T=horizon,
        )

        points: List[TrajectoryPoint] = []
        steps = max(2, int(round(horizon / dt)) + 1)
        for k in range(steps):
            t = min(k * dt, horizon)
            s_t = poly_s.calc_point(t)
            s_d = poly_s.calc_first_derivative(t)
            s_dd = poly_s.calc_second_derivative(t)
            s_ddd = poly_s.calc_third_derivative(t)
            l_t = poly_l.calc_point(t)
            l_d = poly_l.calc_first_derivative(t)
            l_dd = poly_l.calc_second_derivative(t)
            l_prime = l_d / max(1e-3, abs(s_d)) if abs(s_d) > 0.5 else 0.0

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

        self._fill_curvature(points)
        return Trajectory(points=points)

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
        """采样横向偏移与纵向时间窗，评估并返回综合代价最低的可行轨迹。"""
        if not reference_line:
            reference_line = self._fallback_reference_line(ego_state)

        ego_frenet = FrenetConverter.cartesian_to_frenet(ego_state, reference_line)
        target_speed = max(0.0, float(decision.target_speed))

        base_l = float(decision.lateral_shift_l)
        # 附近有障碍物时才启用 ±0.5m 横向避让候选，否则只跟踪目标车道中心，
        # 既减少无意义候选，也降低 50Hz 下的规划耗时。
        # 只有“前方 30m 内”的障碍物才值得启用 ±0.5m 横向避让候选；
        # 后方来车由决策层的跟驰/汇入安全条件处理，不应让规划器做无意义绕行。
        nearby_obstacle = any(
            obs.x > ego_state.x - 2.0
            and float(np.hypot(obs.x - ego_state.x, obs.y - ego_state.y)) < 30.0
            for obs in obstacles
        )
        if nearby_obstacle:
            lateral_targets = self._adaptive_lateral_targets(
                ego_frenet, reference_line, obstacles, base_l
            )
        else:
            lateral_targets = [base_l]
        horizons = [max(4.0, plan_time_horizon)]
        # 只把 80m 内的障碍物送入碰撞检查
        relevant_obstacles = [
            obs for obs in obstacles
            if float(np.hypot(obs.x - ego_state.x, obs.y - ego_state.y)) < 80.0
        ]

        candidates: List[Trajectory] = []
        # 当前实现只启用单一规划时域；保留 horizons 列表是为了将来扩展多时域时
        # 候选总数仍受上限约束（横向目标数 × 时域数 ≤ MAX_LATTICE_SIZE）。
        MAX_LATTICE_SIZE = 9
        max_targets = max(1, MAX_LATTICE_SIZE // len(horizons))
        if len(lateral_targets) > max_targets:
            lateral_targets = sorted(lateral_targets, key=lambda value: (abs(value - base_l), value))[:max_targets]
        if nearby_obstacle and len(lateral_targets) > 3:
            # Retain center, the nearest viable side, and one larger nudge.
            sides = [value for value in lateral_targets if value != base_l]
            lateral_targets = [base_l]
            for sign in (-1.0, 1.0):
                side = sorted((v for v in sides if np.sign(v - base_l) == sign), key=lambda v: abs(v-base_l))
                if side:
                    lateral_targets.append(side[0])
                    if len(lateral_targets) < 3 and len(side) > 1:
                        lateral_targets.append(side[-1])
        for horizon in horizons:
            for target_l in lateral_targets:
                traj = self._build_candidate(
                    ego_frenet=ego_frenet,
                    reference_line=reference_line,
                    target_speed=target_speed,
                    target_l=target_l,
                    horizon=horizon,
                    dt=dt,
                )
                if traj is None or not traj.points:
                    continue
                if any(
                    abs(pt.v ** 2 * pt.kappa) > 3.0 + 1e-6
                    or abs(pt.kappa) > self.max_curvature + 1e-6
                    for pt in traj.points
                ):
                    traj.valid = False
                    traj.total_cost = float("inf")
                    candidates.append(traj)
                    continue
                self.cost_evaluator.evaluate(traj, relevant_obstacles, target_speed, dt=dt)
                # 参考线依从度：偏离目标车道中心越远，代价越高（避免无必要地
                # 选择 ±0.5m 的候选轨迹，同时保留避障时的横向自由度）
                lateral_penalty = (
                    self.cost_evaluator.w_ref_deviation
                    * abs(target_l - base_l)
                    * horizon
                )
                traj.total_cost += float(lateral_penalty)
                candidates.append(traj)

        if not candidates:
            return Trajectory(points=[], valid=False)

        valid = [traj for traj in candidates if traj.valid]
        pool = valid if valid else candidates
        return min(pool, key=lambda traj: traj.total_cost)

    def _adaptive_lateral_targets(
        self,
        ego_frenet: FrenetPoint,
        reference_line: List[ReferencePoint],
        obstacles: List[ObstacleState],
        base_l: float,
    ) -> List[float]:
        """Generate a compact set of targets from nearby Frenet free corridors."""
        # 横向参考可能已是车道中心或相邻车道目标。以真实车道半宽 + 路肩为边界，
        # 既能绕行窄障碍物，又不会把候选目标放到道路之外。
        lane_half_width = self.lane_half_width + self.shoulder_margin
        left_limit, right_limit = base_l + lane_half_width, base_l - lane_half_width
        left_clearance, right_clearance = lane_half_width, lane_half_width
        for obs in obstacles:
            op = FrenetConverter.cartesian_to_frenet(
                VehicleState(x=obs.x, y=obs.y, heading=obs.heading,
                             velocity=float(np.hypot(obs.vx, obs.vy))),
                reference_line,
            )
            half_s = 0.5 * obs.length + 0.5 * self.ego_length
            obs_s_min, obs_s_max = op.s - half_s, op.s + half_s
            if obs_s_max < ego_frenet.s - 3.0 or obs_s_min > ego_frenet.s + 50.0:
                continue
            # Obstacle-free corridor width (vehicle clearance is checked by OBB SAT).
            half_l = 0.5 * obs.width + 0.2
            obs_l_min, obs_l_max = op.l - half_l, op.l + half_l
            # A blocker spanning the lane center leaves two possible passages:
            # measure the available width from its outer edge to each road edge.
            if obs_l_max >= base_l:
                left_clearance = min(left_clearance, left_limit - obs_l_max)
            if obs_l_min <= base_l:
                right_clearance = min(right_clearance, obs_l_min - right_limit)

        targets = [base_l]
        # 只有"该侧剩余走廊明显宽于车身"时才生成绕行候选；阈值与车宽挂钩，
        # 不能写成针对某一特定车道宽的固定值（例如 2.2 m），否则换车道宽度后
        # 绕行候选会整体消失或整体越界。
        min_corridor = 0.5 * self.ego_width + 0.30
        if left_clearance > min_corridor:
            targets.extend((base_l + 0.8, base_l + 1.2, base_l + 1.6))
        if right_clearance > min_corridor:
            targets.extend((base_l - 0.8, base_l - 1.2, base_l - 1.6))
        return sorted(set(float(np.clip(v, right_limit + 0.1, left_limit - 0.1)) for v in targets))[:7]
