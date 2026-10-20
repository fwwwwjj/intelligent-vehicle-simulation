"""
通用 QP 求解器单元测试。

覆盖：无约束、盒约束、线性等式/不等式、热启动与不一致边界。
这些用例刻意保持小规模，便于在低功耗模式下快速回归。
"""

import unittest

import numpy as np

from src.common.qp import solve_qp


class TestQPSolver(unittest.TestCase):

    def test_unconstrained_matches_linear_solve(self):
        P = np.array([[4.0, 1.0], [1.0, 3.0]])
        q = np.array([1.0, 2.0])
        solution = solve_qp(P, q)
        expected = np.linalg.solve(P, -q)
        np.testing.assert_allclose(solution.x, expected, atol=1e-4)
        self.assertTrue(solution.success)

    def test_box_constraints(self):
        P = np.eye(2)
        q = np.array([-3.0, -4.0])
        solution = solve_qp(P, q, lb=np.array([0.0, 0.0]), ub=np.array([1.0, 1.0]))
        np.testing.assert_allclose(solution.x, np.array([1.0, 1.0]), atol=1e-3)
        self.assertTrue(np.all(solution.x >= -1e-6))
        self.assertTrue(np.all(solution.x <= 1.0 + 1e-6))

    def test_linear_equality_and_inequality(self):
        P = np.eye(2)
        q = np.zeros(2)
        A = np.array([[1.0, 1.0], [1.0, 0.0]])
        l = np.array([1.0, 0.0])
        u = np.array([1.0, 0.4])
        solution = solve_qp(P, q, A=A, l=l, u=u)
        np.testing.assert_allclose(solution.x, np.array([0.4, 0.6]), atol=2e-3)
        self.assertTrue(np.all(np.dot(A, solution.x) >= l - 2e-3))
        self.assertTrue(np.all(np.dot(A, solution.x) <= u + 2e-3))

    def test_warm_start_returns_same_solution(self):
        P = np.array([[2.0, 0.0], [0.0, 2.0]])
        q = np.array([-2.0, -2.0])
        cold = solve_qp(P, q, lb=np.array([0.0, 0.0]), ub=np.array([0.5, 0.5]))
        warm = solve_qp(
            P,
            q,
            lb=np.array([0.0, 0.0]),
            ub=np.array([0.5, 0.5]),
            x0=cold.x,
        )
        np.testing.assert_allclose(warm.x, cold.x, atol=1e-3)

    def test_inconsistent_bounds_raise(self):
        with self.assertRaises(ValueError):
            solve_qp(
                np.eye(1),
                np.zeros(1),
                A=np.array([[1.0]]),
                l=np.array([2.0]),
                u=np.array([1.0]),
            )


if __name__ == "__main__":
    unittest.main()
