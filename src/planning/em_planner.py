"""
基于二次规划 (Quadratic Programming, QP) 的 EM 路径-速度解耦规划器。

实现口径参考 Baidu Apollo EM Planner 的 “E-step 决策 / M-step 优化” 思想，
但把问题收敛到竞赛仓库可验证的最小闭环：

1. **E-step（走廊/决策）**：把障碍物投影到 Frenet 路径与 S-T 图，选择从左侧
   或右侧绕行的凸走廊；若两侧都不可行，则不生成横向绕行约束，由速度层在
   障碍物前减速/停车。
2. **M-step（路径 QP）**：在固定站距 ``s`` 上优化横向偏移 ``l(s)``：
   ``min w_ref||l-l_ref||^2 + w1||l'||^2 + w2||l''||^2 + w3||l'''||^2``，
   约束为道路边界、初始位姿与障碍物走廊（带松弛）。
3. **M-step（速度 QP）**：在时间网格上优化速度 ``v(t)``：
   ``min w_v||v-v_ref||^2 + w_a||a||^2 + w_j||j||^2``，
   约束为速度/加速度/加加速度边界，以及 S-T 图给出的
   ``s(t) <= s_obs(t) - safe_gap`` 跟驰/停车约束（带松弛）。
4. **缝合**：把 ``l(s)`` 与 ``s(t)`` 合成时空轨迹，填充
   ``theta/kappa/v/a/jerk``，再用 OBB-SAT 做碰撞校验。

与 Frenet Lattice 的关系：Lattice 是“采样候选 + 代价筛选”，EM 是“凸走廊 +
QP 连续优化”。两者都实现 :class:`~src.planning.base_planner.BasePlanner`，
可在 ``src.runner.build_planner`` 中通过 ``planner="em"`` 切换。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional

import numpy as np

from src.common.frenet_frame import FrenetConverter, FrenetPoint, ReferencePoint
from src.common.qp import solve_qp
from src.common.state import ObstacleState, Trajectory, TrajectoryPoint, VehicleState
from src.decision.base_decision import DecisionOutput
from src.planning.base_planner import BasePlanner
from src.planning.cost_evaluator import TrajectoryCostEvaluator


def _wrap_angle(angle: float) -> float:
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


def _fill_curvature(points: List[TrajectoryPoint]) -> None:
    """由离散 theta(s) 差分计算曲率与曲率变化率。"""
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
        dtheta = _wrap_angle(dtheta)
        kappas[i] = dtheta / max(abs(ds), 1e-3)

    s_array = np.array([p.s for p in points], dtype=float)
    if n > 2 and np.all(np.diff(s_array) >= 1e-6):
        dkappas = np.gradient(kappas, s_array)
    elif n > 2:
        # 停车时轨迹点 s 可能重复；退化到按索引差分，避免 np.gradient 的
        # 重复 x 警告与 NaN 污染 dkappa。
        dkappas = np.gradient(kappas)
    else:
        dkappas = np.zeros(n, dtype=float)
    for i, pt in enumerate(points):
        pt.kappa = float(kappas[i])
        pt.dkappa = float(dkappas[i])


@dataclass
class STProjection:
    """单个障碍物在 S-T 图上的投影。"""

    obstacle_id: str
    times: np.ndarray
    s: np.ndarray
    l: np.ndarray
    active: np.ndarray
    length: float
    width: float
    speed: float


class STGraph:
    """
    S-T 图：把动态障碍物的预测轨迹投影为时空占用。

    ``active`` 表示该时刻障碍物与自车规划路径在横向发生重叠；只有 active 的
    时间点才会进入速度 QP 的跟驰/停车上界。
    """

    def __init__(
        self,
        time_horizon: float = 6.0,
        s_max: float = 200.0,
        dt: float = 0.2,
        lateral_margin: float = 0.25,
    ):
        self.time_horizon = float(time_horizon)
        self.s_max = float(s_max)
        self.dt = max(1e-2, float(dt))
        self.lateral_margin = float(lateral_margin)
        self.projections: List[STProjection] = []

    def time_grid(self) -> np.ndarray:
        n = max(1, int(round(self.time_horizon / self.dt)))
        return np.arange(0.0, n * self.dt + 0.5 * self.dt, self.dt)

    def add_obstacle_projection(
        self,
        obstacle: ObstacleState,
        reference_line: List[ReferencePoint],
        ego_path_lookup: Optional[Callable[[float], float]] = None,
        ego_width: float = 1.9,
        ego_s0: float = 0.0,
    ) -> STProjection:
        """
        把一个障碍物的预测状态投影到 S-T 图。

        :param obstacle: 障碍物当前状态（优先使用 ``predicted_trajectory``，
            缺失时按恒定速度外推）
        :param reference_line: 目标车道参考线
        :param ego_path_lookup: ``s -> l`` 的自车路径横向偏移函数
        :param ego_width: 自车宽度 (m)
        :param ego_s0: 自车当前站距，用于过滤过远目标
        """
        times = self.time_grid()
        s_values = np.full(times.size, np.nan)
        l_values = np.full(times.size, np.nan)
        active = np.zeros(times.size, dtype=bool)

        # 只对“规划起点时位于自车前方”的目标生成跟驰/停车约束。后方来车
        # 即使更快、稍后跑到前面，也不应让 s_ego(t) <= s_rear(t) 这种不等式
        # 进入速度 QP，否则会把可行的汇入问题变成不可行问题。
        initial_state = VehicleState(
            x=float(obstacle.x),
            y=float(obstacle.y),
            heading=float(obstacle.heading),
            velocity=float(np.hypot(obstacle.vx, obstacle.vy)),
            vx=float(obstacle.vx),
            vy=float(obstacle.vy),
        )
        initial_s = FrenetConverter.cartesian_to_frenet(
            initial_state, reference_line
        ).s
        ahead_at_start = initial_s > ego_s0 + 0.5 * ego_width

        pred = obstacle.predicted_trajectory or []
        pred_t = np.array([p.t for p in pred], dtype=float) if pred else np.zeros(0)

        for idx, t in enumerate(times):
            if pred:
                order = int(np.argmin(np.abs(pred_t - t)))
                p = pred[order]
                x, y = float(p.x), float(p.y)
                heading = float(p.theta)
            else:
                x = float(obstacle.x + obstacle.vx * t)
                y = float(obstacle.y + obstacle.vy * t)
                heading = float(obstacle.heading)

            state = VehicleState(
                x=x,
                y=y,
                heading=heading,
                velocity=float(np.hypot(obstacle.vx, obstacle.vy)),
                vx=float(obstacle.vx),
                vy=float(obstacle.vy),
            )
            frenet = FrenetConverter.cartesian_to_frenet(state, reference_line)
            s_values[idx] = frenet.s
            l_values[idx] = frenet.l

            # 只把“前方”障碍物投影为跟驰/停车约束；后方来车不属于本车
            # 纵向让行责任，若误加 s_ego <= s_rear 会让速度 QP 直接不可行。
            if (
                not ahead_at_start
                or frenet.s > ego_s0 + self.s_max
                or frenet.s < ego_s0
            ):
                active[idx] = False
                continue
            path_l = float(ego_path_lookup(frenet.s)) if ego_path_lookup else 0.0
            clearance = 0.5 * (ego_width + obstacle.width) + self.lateral_margin
            active[idx] = bool(abs(frenet.l - path_l) < clearance)

        projection = STProjection(
            obstacle_id=obstacle.id,
            times=times,
            s=s_values,
            l=l_values,
            active=active,
            length=float(obstacle.length),
            width=float(obstacle.width),
            speed=float(np.hypot(obstacle.vx, obstacle.vy)),
        )
        self.projections.append(projection)
        return projection

    def upper_s_bounds(
        self,
        ego_s0: float,
        ego_length: float,
        safe_gap: float = 2.0,
    ) -> np.ndarray:
        """
        返回每个时间点的自车站距上界。

        ``s_ego(t) <= s_obs(t) - 0.5*(L_ego + L_obs) - safe_gap``；
        无 active 障碍物时返回 ``+inf``。
        """
        bounds = np.full(self.time_grid().size, np.inf, dtype=float)
        for projection in self.projections:
            # projection 的时间网格与当前图一致；若不一致则按索引取最小长度
            n = min(bounds.size, projection.times.size)
            for k in range(n):
                if not projection.active[k]:
                    continue
                candidate = (
                    projection.s[k]
                    - 0.5 * (ego_length + projection.length)
                    - safe_gap
                )
                if candidate < bounds[k]:
                    bounds[k] = candidate
        return bounds


@dataclass
class PathResult:
    s: np.ndarray
    l: np.ndarray
    l_ref: float
    cost: float
    success: bool
    iterations: int = 0
    note: str = ""
    constraint_violation: float = 0.0
    status: str = ""
    constraint_violation: float = 0.0
    status: str = ""


@dataclass
class SpeedResult:
    times: np.ndarray
    v: np.ndarray
    s: np.ndarray
    a: np.ndarray
    jerk: np.ndarray
    cost: float
    success: bool
    iterations: int = 0
    note: str = ""
    constraint_violation: float = 0.0
    status: str = ""
    constraint_violation: float = 0.0
    status: str = ""


class EMPlanner(BasePlanner):
    """
    EM 路径-速度解耦 QP 规划器。
    """

    def __init__(
        self,
        ego_length: float = 4.8,
        ego_width: float = 1.9,
        lane_width: float = 3.75,
        max_accel: float = 3.5,
        max_decel: float = 7.0,
        max_jerk: float = 4.0,
        path_ds: float = 2.0,
        path_length: float = 80.0,
        speed_dt: float = 0.2,
        speed_horizon: float = 6.0,
        max_speed: float = 30.0,
        safe_gap: float = 2.0,
        lateral_clearance: float = 0.25,
        path_weights: Optional[dict] = None,
        speed_weights: Optional[dict] = None,
        qp_max_iter: int = 2500,
        qp_eps_abs: float = 2e-5,
        qp_eps_rel: float = 2e-5,
    ):
        self.ego_length = float(ego_length)
        self.ego_width = float(ego_width)
        self.lane_width = float(lane_width)
        self.max_accel = float(max_accel)
        self.max_decel = float(max_decel)
        self.max_jerk = float(max_jerk)
        self.path_ds = max(0.5, float(path_ds))
        self.path_length = max(10.0, float(path_length))
        self.speed_dt = max(0.05, float(speed_dt))
        self.speed_horizon = max(1.0, float(speed_horizon))
        self.max_speed = float(max_speed)
        self.safe_gap = max(0.0, float(safe_gap))
        self.lateral_clearance = max(0.0, float(lateral_clearance))
        self.qp_max_iter = int(qp_max_iter)
        self.qp_eps_abs = float(qp_eps_abs)
        self.qp_eps_rel = float(qp_eps_rel)

        pw = {
            "ref": 1.0,
            "slope": 4.0,
            "curvature": 24.0,
            "jerk": 120.0,
            "start_slope": 30.0,
            "terminal": 8.0,
            "slack": 2.0e3,
            "slack_linear": 2.0e4,
        }
        pw.update(path_weights or {})
        self.path_weights = pw

        sw = {
            "speed": 1.0,
            "accel": 0.2,
            "jerk": 0.08,
            "terminal": 4.0,
            "slack": 2.0e3,
            "slack_linear": 2.0e4,
        }
        sw.update(speed_weights or {})
        self.speed_weights = sw

        self.cost_evaluator = TrajectoryCostEvaluator(
            ego_length=self.ego_length,
            ego_width=self.ego_width,
            max_accel=self.max_accel,
            max_decel=self.max_decel,
        )
        self.last_path_result: Optional[PathResult] = None
        self.last_speed_result: Optional[SpeedResult] = None
        self.last_st_graph: Optional[STGraph] = None

    # ------------------------------------------------------------------
    # 参考线兜底
    # ------------------------------------------------------------------
    @staticmethod
    def _fallback_reference_line(
        ego_state: VehicleState, length: float = 160.0
    ) -> List[ReferencePoint]:
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
    # 路径 QP
    # ------------------------------------------------------------------
    def _path_grid(
        self, ego_frenet: FrenetPoint, reference_line: List[ReferencePoint], horizon: float
    ) -> np.ndarray:
        v = max(0.0, float(ego_frenet.s_d))
        desired = max(self.path_length, v * float(horizon) + 20.0)
        ref_end = float(reference_line[-1].s)
        s_end = min(ref_end, ego_frenet.s + desired)
        span = max(0.0, s_end - ego_frenet.s)
        if span < self.path_ds:
            return np.array([ego_frenet.s, ego_frenet.s + self.path_ds], dtype=float)
        n = max(1, int(np.floor(span / self.path_ds)))
        grid = ego_frenet.s + self.path_ds * np.arange(n + 1)
        if grid[-1] < s_end - 0.25 * self.path_ds:
            grid = np.append(grid, s_end)
        return grid

    @staticmethod
    def _difference_matrix(n: int, order: int) -> np.ndarray:
        """返回 n 个变量的 k 阶前向差分矩阵，形状 (n-order, n)。"""
        if n <= order:
            return np.zeros((0, n), dtype=float)
        return np.diff(np.eye(n, dtype=float), n=order, axis=0)

    def _obstacle_frenet(
        self, obstacle: ObstacleState, reference_line: List[ReferencePoint]
    ) -> Optional[FrenetPoint]:
        state = VehicleState(
            x=obstacle.x,
            y=obstacle.y,
            heading=obstacle.heading,
            velocity=float(np.hypot(obstacle.vx, obstacle.vy)),
            vx=obstacle.vx,
            vy=obstacle.vy,
        )
        try:
            return FrenetConverter.cartesian_to_frenet(state, reference_line)
        except Exception:
            return None

    def _optimize_path(
        self,
        ego_state: VehicleState,
        ego_frenet: FrenetPoint,
        reference_line: List[ReferencePoint],
        obstacles: List[ObstacleState],
        decision: DecisionOutput,
        horizon: float,
    ) -> PathResult:
        s_grid = self._path_grid(ego_frenet, reference_line, horizon)
        n_l = len(s_grid)
        ds = float(np.mean(np.diff(s_grid))) if n_l > 1 else self.path_ds
        ds = max(ds, 1e-3)

        lane_half = 0.5 * self.lane_width
        ego_half = 0.5 * self.ego_width
        l_min = -lane_half + ego_half + 0.10
        l_max = lane_half - ego_half - 0.10
        l_min = min(l_min, float(ego_frenet.l) - 0.05)
        l_max = max(l_max, float(ego_frenet.l) + 0.05)
        if l_max - l_min < 0.2:
            center = 0.5 * (l_min + l_max)
            l_min, l_max = center - 0.1, center + 0.1

        l_ref = float(np.clip(decision.lateral_shift_l, l_min, l_max))

        I = np.eye(n_l, dtype=float)
        D1 = self._difference_matrix(n_l, 1) / ds
        D2 = self._difference_matrix(n_l, 2) / (ds * ds)
        D3 = self._difference_matrix(n_l, 3) / (ds * ds * ds)

        P = 2.0 * self.path_weights["ref"] * I
        q = -2.0 * self.path_weights["ref"] * l_ref * np.ones(n_l)
        if D1.size:
            P += 2.0 * self.path_weights["slope"] * np.dot(D1.T, D1)
        if D2.size:
            P += 2.0 * self.path_weights["curvature"] * np.dot(D2.T, D2)
        if D3.size:
            P += 2.0 * self.path_weights["jerk"] * np.dot(D3.T, D3)

        # 初始斜率软约束：(l_1 - l_0)/ds ≈ l'_0
        start_slope_row = np.zeros(n_l)
        start_slope_row[0] = -1.0 / ds
        if n_l > 1:
            start_slope_row[1] = 1.0 / ds
        P += 2.0 * self.path_weights["start_slope"] * np.outer(
            start_slope_row, start_slope_row
        )
        q += (
            -2.0
            * self.path_weights["start_slope"]
            * start_slope_row
            * float(ego_frenet.l_prime)
        )

        # 终点依从参考横向偏移
        terminal = np.zeros((1, n_l))
        terminal[0, -1] = 1.0
        P += 2.0 * self.path_weights["terminal"] * np.dot(terminal.T, terminal)
        q += -2.0 * self.path_weights["terminal"] * l_ref * terminal[0]

        # 障碍物走廊：为每个障碍物选择一个可绕行侧，若两侧都不可行则跳过
        corridor_rows: List[np.ndarray] = []
        corridor_upper: List[float] = []
        for obstacle in obstacles:
            obs_f = self._obstacle_frenet(obstacle, reference_line)
            if obs_f is None:
                continue
            s_lo = obs_f.s - 0.5 * (self.ego_length + obstacle.length) - 2.0
            s_hi = obs_f.s + 0.5 * (self.ego_length + obstacle.length) + 2.0
            if s_hi < s_grid[0] - 2.0 or s_lo > s_grid[-1] + 2.0:
                continue
            idx = np.where((s_grid >= s_lo) & (s_grid <= s_hi))[0]
            if idx.size == 0:
                continue

            clearance = 0.5 * (self.ego_width + obstacle.width) + self.lateral_clearance
            left_bound = float(obs_f.l + clearance)
            right_bound = float(obs_f.l - clearance)
            left_space = l_max - left_bound
            right_space = right_bound - l_min
            if left_space >= 0.10 and left_space >= right_space:
                for i in idx:
                    row = np.zeros(n_l)
                    row[i] = -1.0
                    corridor_rows.append(row)
                    corridor_upper.append(-left_bound)
            elif right_space >= 0.10:
                for i in idx:
                    row = np.zeros(n_l)
                    row[i] = 1.0
                    corridor_rows.append(row)
                    corridor_upper.append(right_bound)

        n_slack = max(1, len(corridor_rows)) if corridor_rows else 0
        n_var = n_l + n_slack
        P_full = np.zeros((n_var, n_var))
        q_full = np.zeros(n_var)
        P_full[:n_l, :n_l] = P
        q_full[:n_l] = q
        slack_scale = 1.0 / np.sqrt(max(2.0 * self.path_weights["slack"], 1e-9))
        if n_slack:
            P_full[n_l:, n_l:] = np.eye(n_slack)
            q_full[n_l:] = self.path_weights["slack_linear"] * slack_scale
            A_rows = np.zeros((len(corridor_rows), n_var))
            for r, row in enumerate(corridor_rows):
                A_rows[r, :n_l] = row
                A_rows[r, n_l + r] = -slack_scale
            A_corr = A_rows
            b_corr = np.asarray(corridor_upper, dtype=float)
        else:
            A_corr = None
            b_corr = None

        lb = np.full(n_var, -np.inf)
        ub = np.full(n_var, np.inf)
        lb[:n_l] = l_min
        ub[:n_l] = l_max
        # 起点位置硬约束（l_0 必须在道路边界内；上面的界限已扩展包含 l_0）
        lb[0] = float(ego_frenet.l)
        ub[0] = float(ego_frenet.l)
        if n_slack:
            lb[n_l:] = 0.0

        solution = solve_qp(
            P_full,
            q_full,
            A=A_corr,
            u=b_corr,
            lb=lb,
            ub=ub,
            max_iter=self.qp_max_iter,
            eps_abs=self.qp_eps_abs,
            eps_rel=self.qp_eps_rel,
        )
        success = bool(
            solution.success
            and np.all(np.isfinite(solution.x))
            and solution.constraint_violation <= 2e-2
        )

        if success:
            l_values = np.asarray(solution.x[:n_l], dtype=float)
            cost = float(solution.objective)
            note = "qp"
        else:
            # 退化为“从当前横移到目标横移的平滑插值”，保证规划器仍输出可跟踪路径
            ratio = np.linspace(0.0, 1.0, n_l)
            smooth = 3.0 * ratio**2 - 2.0 * ratio**3
            l_values = float(ego_frenet.l) + (l_ref - float(ego_frenet.l)) * smooth
            cost = float("inf")
            note = f"fallback:{solution.status}"

        return PathResult(
            s=s_grid,
            l=l_values,
            l_ref=l_ref,
            cost=cost,
            success=success,
            iterations=int(solution.iterations),
            note=note,
            constraint_violation=float(solution.constraint_violation),
            status=str(solution.status),
        )

    # ------------------------------------------------------------------
    # 速度 QP
    # ------------------------------------------------------------------
    @staticmethod
    def _speed_integration_matrix(n_steps: int, dt: float) -> np.ndarray:
        """返回 (n_steps, n_steps+1) 的梯形积分矩阵，s_k = s_0 + C v。"""
        C = np.zeros((n_steps, n_steps + 1), dtype=float)
        for k in range(1, n_steps + 1):
            row = k - 1
            C[row, 0] = 0.5 * dt
            if k > 1:
                C[row, 1:k] = dt
            C[row, k] = 0.5 * dt
        return C

    def _fallback_speed(
        self,
        v0: float,
        target: float,
        times: np.ndarray,
        s0: float = 0.0,
        upper_bounds: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        QP 失败时的保守速度剖面：按最大加/减速度趋向目标，并在 S-T 上界
        前按 ``v <= sqrt(2*b*(s_bound-s))`` 预留制动距离，避免回退解直接追尾。
        """
        n = times.size
        if n == 0:
            return np.zeros(0)
        v = np.empty(n, dtype=float)
        v[0] = v0
        s = float(s0)
        for k in range(1, n):
            dt = max(1e-6, float(times[k] - times[k - 1]))
            if target >= v[k - 1]:
                candidate = v[k - 1] + self.max_accel * dt
            else:
                candidate = v[k - 1] - self.max_decel * dt
            candidate = float(np.clip(candidate, 0.0, max(target, 0.0) if target >= v[k - 1] else max(v[k - 1], 0.0)))
            if upper_bounds is not None and k < upper_bounds.size and np.isfinite(upper_bounds[k]):
                available = max(0.0, float(upper_bounds[k] - s))
                v_stop = float(np.sqrt(max(0.0, 2.0 * self.max_decel * available)))
                candidate = min(candidate, v_stop)
            candidate = float(
                np.clip(
                    candidate,
                    max(0.0, v[k - 1] - self.max_decel * dt),
                    v[k - 1] + self.max_accel * dt,
                )
            )
            v[k] = candidate
            s += 0.5 * dt * (v[k - 1] + candidate)
        return v

    def _optimize_speed(
        self,
        ego_frenet: FrenetPoint,
        decision: DecisionOutput,
        st_graph: STGraph,
        time_grid: np.ndarray,
        dt: float,
    ) -> SpeedResult:
        n_steps = max(1, time_grid.size - 1)
        times = time_grid[: n_steps + 1]
        v0 = max(0.0, float(ego_frenet.s_d))
        target = max(0.0, float(decision.target_speed))
        v_max = max(self.max_speed, v0 + 5.0, target + 5.0)

        C = self._speed_integration_matrix(n_steps, dt)
        A_acc = np.zeros((n_steps, n_steps + 1), dtype=float)
        for k in range(n_steps):
            A_acc[k, k] = -1.0 / dt
            A_acc[k, k + 1] = 1.0 / dt
        A_jerk = np.diff(A_acc, axis=0) / dt if n_steps > 1 else np.zeros((0, n_steps + 1))

        # 速度参考：巡航/跟驰取决策目标速度；紧急停车目标为 0
        v_ref = np.full(n_steps + 1, target)

        # 若 S-T 上界会在时域内被穿越，则给终端速度一个保守上界。
        # 否则优化器可能“恰好”在时域末端贴住障碍物边界，但末端速度仍很高，
        # 时域一结束就会追尾（这是 horizon 截断的典型漏洞）。
        upper_bounds = st_graph.upper_s_bounds(
            ego_s0=ego_frenet.s,
            ego_length=self.ego_length,
            safe_gap=self.safe_gap,
        )
        finite_steps = np.where(np.isfinite(upper_bounds))[0]
        binding = False
        terminal_cap = float("inf")
        if finite_steps.size:
            free_travel = ego_frenet.s + v0 * times[finite_steps] + 5.0
            binding = bool(np.any(upper_bounds[finite_steps] < free_travel))
        if binding:
            active_speeds = [
                p.speed
                for p in st_graph.projections
                if np.any(p.active)
            ]
            terminal_cap = max(
                0.5,
                (min(active_speeds) if active_speeds else 0.0) - 0.5,
            )
            v_ref[-1] = min(target, terminal_cap)
        terminal_weight = (
            max(self.speed_weights["terminal"], 200.0)
            if binding
            else self.speed_weights["terminal"]
        )

        P = np.zeros((n_steps + 1, n_steps + 1))
        q = np.zeros(n_steps + 1)
        w_speed = self.speed_weights["speed"]
        P += 2.0 * w_speed * np.eye(n_steps + 1)
        q += -2.0 * w_speed * v_ref
        P += 2.0 * self.speed_weights["accel"] * np.dot(A_acc.T, A_acc)
        if A_jerk.size:
            P += 2.0 * self.speed_weights["jerk"] * np.dot(A_jerk.T, A_jerk)
        terminal = np.zeros(n_steps + 1)
        terminal[-1] = 1.0
        P += 2.0 * terminal_weight * np.outer(terminal, terminal)
        q += -2.0 * terminal_weight * float(v_ref[-1]) * terminal

        # 硬约束：加速度/加加速度
        rows = [A_acc, -A_acc]
        rhs = [np.full(n_steps, self.max_accel), np.full(n_steps, self.max_decel)]
        if A_jerk.size:
            rows.extend([A_jerk, -A_jerk])
            rhs.extend(
                [
                    np.full(A_jerk.shape[0], self.max_jerk),
                    np.full(A_jerk.shape[0], self.max_jerk),
                ]
            )

        # S-T 跟驰/停车约束：s_k <= bound_k。
        # 先尝试硬约束（变量更少、收敛更快）；若不可行或数值未收敛，再退回
        # 带松弛的版本，保证“停不下来”时仍能返回最大制动剖面而不是空轨迹。
        # C[k] 对应 t_{k+1} 的站距，因此约束取 upper_bounds[k+1]，避免
        # 把 t_k 的上界错配到 t_{k+1} 的积分行上（会平白收紧一步）。
        st_row_bounds = upper_bounds[1 : n_steps + 1]
        active_steps = np.where(np.isfinite(st_row_bounds))[0]
        slack_scale = 1.0 / np.sqrt(max(2.0 * self.speed_weights["slack"], 1e-9))

        def _solve_speed_qp(use_slack: bool):
            n_st = int(active_steps.size) if use_slack else 0
            n_var = (n_steps + 1) + n_st
            P_full = np.zeros((n_var, n_var))
            q_full = np.zeros(n_var)
            P_full[: n_steps + 1, : n_steps + 1] = P
            q_full[: n_steps + 1] = q
            if n_st:
                P_full[n_steps + 1 :, n_steps + 1 :] = np.eye(n_st)
                q_full[n_steps + 1 :] = (
                    self.speed_weights["slack_linear"] * slack_scale
                )

            A_rows = []
            b_rows = []
            for row, b in zip(rows, rhs):
                if row.size:
                    A_rows.append(
                        np.hstack([row, np.zeros((row.shape[0], n_st))])
                    )
                    b_rows.append(b)
            for j, k in enumerate(active_steps):
                row = np.zeros(n_var)
                row[: n_steps + 1] = C[k]
                if use_slack:
                    row[n_steps + 1 + j] = -slack_scale
                A_rows.append(row.reshape(1, -1))
                b_rows.append(np.array([st_row_bounds[k] - ego_frenet.s]))

            A_ineq = np.vstack(A_rows) if A_rows else None
            b_ineq = np.concatenate(b_rows) if b_rows else None

            lb = np.zeros(n_var)
            ub = np.full(n_var, np.inf)
            ub[: n_steps + 1] = v_max
            if n_st:
                lb[n_steps + 1 :] = 0.0
            lb[0] = v0
            ub[0] = v0
            if binding and v0 - self.max_decel * times[-1] <= terminal_cap:
                ub[n_steps] = min(v_max, terminal_cap)

            sol = solve_qp(
                P_full,
                q_full,
                A=A_ineq,
                u=b_ineq,
                lb=lb,
                ub=ub,
                max_iter=self.qp_max_iter,
                eps_abs=self.qp_eps_abs,
                eps_rel=self.qp_eps_rel,
            )
            ok = bool(
                sol.success
                and np.all(np.isfinite(sol.x))
                and sol.constraint_violation <= 2e-2
            )
            return sol, ok, n_st

        solution, success, _ = _solve_speed_qp(use_slack=False)
        used_soft = False
        if not success and active_steps.size:
            solution, success, _ = _solve_speed_qp(use_slack=True)
            used_soft = True
        if success:
            v = np.asarray(solution.x[: n_steps + 1], dtype=float)
            cost = float(solution.objective)
            note = "qp-soft" if used_soft else "qp-hard"
        else:
            v = self._fallback_speed(
                v0,
                target,
                times,
                s0=float(ego_frenet.s),
                upper_bounds=upper_bounds,
            )
            cost = float("inf")
            note = f"fallback:{solution.status}"

        v = np.clip(v, 0.0, v_max)
        s_offsets = np.dot(C, v) if n_steps > 0 else np.zeros(0)
        s = float(ego_frenet.s) + np.concatenate(([0.0], s_offsets))
        s = np.maximum.accumulate(s)
        a = np.dot(A_acc, v)
        a_full = np.concatenate(([a[0]], a)) if a.size else np.zeros(times.size)
        if times.size > 2:
            jerk_full = np.gradient(a_full, times)
        else:
            jerk_full = np.zeros(times.size)

        return SpeedResult(
            times=times,
            v=v,
            s=s,
            a=a_full[: times.size],
            jerk=jerk_full[: times.size],
            cost=cost,
            success=success,
            iterations=int(solution.iterations),
            note=note,
            constraint_violation=float(solution.constraint_violation),
            status=str(solution.status),
        )

    # ------------------------------------------------------------------
    # 轨迹缝合
    # ------------------------------------------------------------------
    def _assemble_trajectory(
        self,
        reference_line: List[ReferencePoint],
        path: PathResult,
        speed: SpeedResult,
        obstacles: List[ObstacleState],
        decision: DecisionOutput,
        dt: float,
    ) -> Trajectory:
        l_values = np.interp(
            speed.s, path.s, path.l, left=float(path.l[0]), right=float(path.l[-1])
        )
        l_d = np.gradient(l_values, speed.times) if speed.times.size > 1 else np.zeros_like(l_values)
        l_dd = (
            np.gradient(l_d, speed.times) if speed.times.size > 2 else np.zeros_like(l_values)
        )
        l_prime_path = np.gradient(path.l, path.s) if path.s.size > 2 else np.zeros_like(path.l)
        l_prime = np.interp(speed.s, path.s, l_prime_path)
        l_prime_prime_path = (
            np.gradient(l_prime_path, path.s) if path.s.size > 2 else np.zeros_like(path.l)
        )
        l_prime_prime = np.interp(speed.s, path.s, l_prime_prime_path)

        points: List[TrajectoryPoint] = []
        for k in range(speed.times.size):
            s_k = float(speed.s[k])
            ref_pt = FrenetConverter.ref_point_at_s(reference_line, s_k)
            frenet_pt = FrenetPoint(
                s=s_k,
                s_d=float(speed.v[k]),
                s_dd=float(speed.a[k]),
                s_ddd=float(speed.jerk[k]),
                l=float(l_values[k]),
                l_d=float(l_d[k]),
                l_dd=float(l_dd[k]),
                l_prime=float(l_prime[k]),
                l_prime_prime=float(l_prime_prime[k]),
            )
            c_pt = FrenetConverter.frenet_to_cartesian(frenet_pt, ref_pt)
            c_pt.t = float(speed.times[k])
            c_pt.s = s_k
            c_pt.v = float(np.clip(speed.v[k], 0.0, max(self.max_speed, 40.0)))
            c_pt.a = float(np.clip(speed.a[k], -self.max_decel, self.max_accel))
            c_pt.jerk = float(speed.jerk[k])
            points.append(c_pt)

        _fill_curvature(points)
        trajectory = Trajectory(points=points)
        self.cost_evaluator.evaluate(
            trajectory,
            obstacles,
            target_speed=max(0.0, float(decision.target_speed)),
            dt=dt,
        )
        return trajectory

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
        """执行 Path QP + Speed QP 并返回时空轨迹。"""
        if not reference_line:
            reference_line = self._fallback_reference_line(ego_state)

        ego_frenet = FrenetConverter.cartesian_to_frenet(ego_state, reference_line)
        path = self._optimize_path(
            ego_state,
            ego_frenet,
            reference_line,
            obstacles,
            decision,
            horizon=plan_time_horizon,
        )

        speed_dt = max(self.speed_dt, float(dt))
        speed_horizon = max(self.speed_horizon, float(plan_time_horizon))
        st_graph = STGraph(
            time_horizon=speed_horizon,
            s_max=max(150.0, float(ego_frenet.s) + self.path_length),
            dt=speed_dt,
        )
        path_lookup = lambda s: float(np.interp(s, path.s, path.l))
        for obstacle in obstacles:
            st_graph.add_obstacle_projection(
                obstacle,
                reference_line,
                ego_path_lookup=path_lookup,
                ego_width=self.ego_width,
                ego_s0=ego_frenet.s,
            )

        time_grid = st_graph.time_grid()
        speed = self._optimize_speed(
            ego_frenet,
            decision,
            st_graph,
            time_grid,
            dt=speed_dt,
        )

        trajectory = self._assemble_trajectory(
            reference_line,
            path,
            speed,
            obstacles,
            decision,
            dt=speed_dt,
        )
        self.last_path_result = path
        self.last_speed_result = speed
        self.last_st_graph = st_graph
        return trajectory


__all__ = ["EMPlanner", "STGraph", "STProjection", "PathResult", "SpeedResult"]
