"""
轻量级稠密凸二次规划 (Quadratic Programming, QP) 求解器。

MPC 跟踪与 EM 规划共用同一个 QP 后端。为了保持仓库“最小依赖可复现”的
约束，这里不强制依赖 OSQP / cvxpy / scipy，而是实现一个 OSQP 风格的
交替方向乘子法 (ADMM)：

    min  0.5 * x' P x + q' x
    s.t. l <= A x <= u
         lb <= x <= ub

盒约束会被自动展开为 ``A`` 的单位行，因此调用方只需要给出一份线性约束。
求解器支持热启动（``x0`` / ``z0`` / ``y0``），适合滚动时域控制。

数值口径：
- 目标函数中的二次项使用 ``0.5 * x' P x``，若代价写成 ``w * (x-r)^2``，
  则 ``P`` 对角加 ``2w``、``q`` 加 ``-2wr``；
- 约束容差在 ``success`` 与 ``constraint_violation`` 中显式报告，
  调用方不应把未收敛解当作硬约束保证。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional

import numpy as np


@dataclass
class QPSolution:
    """QP 求解结果与收敛诊断。"""

    x: np.ndarray
    success: bool
    iterations: int
    primal_residual: float
    dual_residual: float
    objective: float
    constraint_violation: float
    status: str = "solved"
    z: Optional[np.ndarray] = None
    y: Optional[np.ndarray] = None


def _broadcast_bound(value, size: int, default: float) -> np.ndarray:
    if value is None:
        return np.full(size, default, dtype=float)
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.size == 1:
        return np.full(size, float(array[0]), dtype=float)
    if array.size != size:
        raise ValueError(f"bound size {array.size} does not match rows {size}")
    return array.copy()


def _objective(P: np.ndarray, q: np.ndarray, x: np.ndarray) -> float:
    return float(0.5 * np.dot(x, np.dot(P, x)) + np.dot(q, x))


def solve_qp(
    P: np.ndarray,
    q: np.ndarray,
    A: Optional[np.ndarray] = None,
    l: Optional[Iterable[float]] = None,
    u: Optional[Iterable[float]] = None,
    lb: Optional[Iterable[float]] = None,
    ub: Optional[Iterable[float]] = None,
    x0: Optional[np.ndarray] = None,
    z0: Optional[np.ndarray] = None,
    y0: Optional[np.ndarray] = None,
    max_iter: int = 4000,
    eps_abs: float = 1e-5,
    eps_rel: float = 1e-5,
    rho: float = 1.0,
    sigma: float = 1e-6,
    adaptive_rho: bool = True,
    check_interval: int = 25,
    alpha: float = 1.6,
    x_init: Optional[np.ndarray] = None,
    y_init: Optional[np.ndarray] = None,
) -> QPSolution:
    """
    求解稠密凸 QP。

    :param P: 半正定/正定 Hessian，形状 (n, n)
    :param q: 线性项，形状 (n,)
    :param A: 线性约束矩阵，形状 (m, n)，可选
    :param l: 线性约束下界，标量或长度 m，缺省 -inf
    :param u: 线性约束上界，标量或长度 m，缺省 +inf
    :param lb: 变量下界，标量或长度 n，缺省 -inf
    :param ub: 变量上界，标量或长度 n，缺省 +inf
    :param x0: 原始变量热启动值
    :param z0: 约束变量热启动值（``z = clip(A x)``）
    :param y0: 缩放对偶变量热启动值
    :param max_iter: 最大 ADMM 迭代次数
    :param eps_abs: 绝对残差容差
    :param eps_rel: 相对残差容差
    :param rho: 初始惩罚参数
    :param sigma: 原始变量稳定项，避免 P 半正定时奇异
    :param adaptive_rho: 是否按原始/对偶残差比例自适应调整 rho
    :param check_interval: 自适应 rho 的检查间隔
    :param alpha: ADMM 过松弛系数，1.0 表示不松弛，1.6 为 OSQP 默认值
    :param x_init: 原始变量初值别名 (等效于 x0)
    :param y_init: 对偶变量初值别名 (等效于 y0)
    :return: :class:`QPSolution`
    """
    if x0 is None and x_init is not None:
        x0 = x_init
    if y0 is None and y_init is not None:
        y0 = y_init
    q = np.asarray(q, dtype=float).reshape(-1)
    n = q.size
    if n == 0:
        return QPSolution(
            x=np.zeros(0),
            success=True,
            iterations=0,
            primal_residual=0.0,
            dual_residual=0.0,
            objective=0.0,
            constraint_violation=0.0,
            status="empty",
        )

    P = np.asarray(P, dtype=float)
    if P.shape != (n, n):
        raise ValueError(f"P shape {P.shape} does not match n={n}")
    P = 0.5 * (P + P.T)

    rows = []
    lower = []
    upper = []
    if A is not None:
        A = np.asarray(A, dtype=float)
        if A.ndim == 1:
            A = A.reshape(1, -1)
        if A.shape[1] != n:
            raise ValueError(f"A shape {A.shape} does not match n={n}")
        l_vec = _broadcast_bound(l, A.shape[0], -np.inf)
        u_vec = _broadcast_bound(u, A.shape[0], np.inf)
        rows.append(A)
        lower.append(l_vec)
        upper.append(u_vec)

    if lb is not None or ub is not None:
        identity = np.eye(n)
        rows.append(identity)
        lower.append(_broadcast_bound(lb, n, -np.inf))
        upper.append(_broadcast_bound(ub, n, np.inf))

    if rows:
        A_full = np.vstack(rows)
        l_full = np.concatenate(lower)
        u_full = np.concatenate(upper)
        if np.any(l_full > u_full + 1e-12):
            raise ValueError("inconsistent QP bounds: l > u")
    else:
        A_full = np.zeros((0, n), dtype=float)
        l_full = np.zeros(0, dtype=float)
        u_full = np.zeros(0, dtype=float)

    m = A_full.shape[0]
    if m == 0:
        x = np.linalg.solve(P + sigma * np.eye(n), -q)
        return QPSolution(
            x=x,
            success=True,
            iterations=0,
            primal_residual=0.0,
            dual_residual=0.0,
            objective=_objective(P, q, x),
            constraint_violation=0.0,
            status="unconstrained",
        )

    if x0 is not None and np.asarray(x0).size == n:
        x = np.asarray(x0, dtype=float).reshape(-1).copy()
    else:
        x = np.zeros(n, dtype=float)

    if z0 is not None and np.asarray(z0).size == m:
        z = np.asarray(z0, dtype=float).reshape(-1).copy()
    else:
        z = np.clip(np.dot(A_full, x), l_full, u_full)

    if y0 is not None and np.asarray(y0).size == m:
        y = np.asarray(y0, dtype=float).reshape(-1).copy()
    else:
        y = np.zeros(m, dtype=float)

    identity = np.eye(n)
    AtA = np.dot(A_full.T, A_full)

    def factor_inverse(current_rho: float) -> np.ndarray:
        matrix = P + sigma * identity + current_rho * AtA
        matrix = 0.5 * (matrix + matrix.T)
        try:
            return np.linalg.inv(matrix)
        except np.linalg.LinAlgError:
            # 极少数半正定病态情形：加 jitter 后退化为伪逆
            jitter = 1e-8 * max(1.0, float(np.max(np.abs(np.diag(matrix)))))
            return np.linalg.pinv(matrix + jitter * identity)

    Minv = factor_inverse(rho)
    success = False
    status = "max_iter"
    primal_residual = float("inf")
    dual_residual = float("inf")

    for iteration in range(1, max_iter + 1):
        x_prev = x
        rhs = -q + sigma * x_prev + rho * np.dot(A_full.T, (z - y))
        x = np.dot(Minv, rhs)

        Ax = np.dot(A_full, x)
        z_prev = z
        relaxed = alpha * Ax + (1.0 - alpha) * z_prev
        z = np.clip(relaxed + y, l_full, u_full)
        y = y + relaxed - z

        primal = Ax - z
        dual = rho * np.dot(A_full.T, (z - z_prev))
        primal_residual = float(np.linalg.norm(primal))
        dual_residual = float(np.linalg.norm(dual))

        eps_pri = np.sqrt(m) * eps_abs + eps_rel * max(
            float(np.linalg.norm(Ax)), float(np.linalg.norm(z))
        )
        eps_dual = np.sqrt(n) * eps_abs + eps_rel * max(
            float(np.linalg.norm(np.dot(A_full.T, y))),
            float(np.linalg.norm(np.dot(P, x))),
        )
        if primal_residual <= eps_pri and dual_residual <= eps_dual:
            success = True
            status = "solved"
            break

        if adaptive_rho and iteration % check_interval == 0:
            if primal_residual > 10.0 * dual_residual:
                rho *= 2.0
                y *= 0.5
                Minv = factor_inverse(rho)
            elif dual_residual > 10.0 * primal_residual:
                rho *= 0.5
                y *= 2.0
                Minv = factor_inverse(rho)

    Ax = np.dot(A_full, x)
    violation = 0.0
    finite_low = np.isfinite(l_full)
    finite_high = np.isfinite(u_full)
    if np.any(finite_low):
        violation = max(violation, float(np.max(l_full[finite_low] - Ax[finite_low])))
    if np.any(finite_high):
        violation = max(violation, float(np.max(Ax[finite_high] - u_full[finite_high])))
    violation = max(0.0, violation)

    # 盒约束可以从 x 直接投影；线性约束不做破坏可行性的粗暴截断。
    if m == n and np.allclose(A_full, np.eye(n), atol=0.0, rtol=0.0):
        x = np.clip(x, l_full, u_full)

    return QPSolution(
        x=x,
        success=success,
        iterations=iteration if max_iter > 0 else 0,
        primal_residual=primal_residual,
        dual_residual=dual_residual,
        objective=_objective(P, q, x),
        constraint_violation=violation,
        status=status,
        z=z.copy() if z is not None else None,
        y=y.copy() if y is not None else None,
    )


__all__ = ["QPSolution", "solve_qp"]
