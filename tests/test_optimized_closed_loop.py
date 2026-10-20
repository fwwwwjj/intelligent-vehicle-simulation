"""
新算法主回路集成测试（低功耗短时运行）。

不追求长时间统计，只验证：
1. ``build_engine`` 能按 planner/controller/tire 选项装配；
2. EM + MPC + Pacejka 能在真实 Scene 上连续步进且状态有限；
3. KPI 记录仍然逐帧生成。
"""

import unittest

import numpy as np

from src.control.lqr_controller import LQRController
from src.control.mpc_controller import MPCController
from src.environment.scenario_loader import (
    build_emergency_avoidance_scenario,
    build_highway_merge_scenario,
)
from src.planning.em_planner import EMPlanner
from src.planning.frenet_lattice import FrenetLatticePlanner
from src.runner import build_engine


class TestOptimizedClosedLoop(unittest.TestCase):

    def test_em_mpc_closed_loop_short_run(self):
        scenario = build_emergency_avoidance_scenario()
        engine = build_engine(
            scenario,
            decision="fsm",
            seed=42,
            enable_visualization=False,
            planner="em",
            controller="mpc",
        )
        self.assertIsInstance(engine.planner, EMPlanner)
        self.assertIsInstance(engine.controller, MPCController)

        for _ in range(8):
            engine.step()

        state = engine.ego_state
        self.assertTrue(np.isfinite(state.x))
        self.assertTrue(np.isfinite(state.y))
        self.assertTrue(np.isfinite(state.velocity))
        self.assertGreaterEqual(state.velocity, 0.0)
        self.assertEqual(len(engine.evaluator.records), 8)
        record = engine.evaluator.records[-1]
        self.assertGreaterEqual(record.planning_latency_ms, 0.0)
        self.assertGreaterEqual(record.control_latency_ms, 0.0)

    def test_pacejka_selects_dynamic_model(self):
        scenario = build_highway_merge_scenario()
        engine = build_engine(
            scenario,
            decision="fsm",
            seed=7,
            enable_visualization=False,
            planner="lattice",
            controller="lqr",
            tire_model="pacejka",
        )
        self.assertEqual(engine.tire_model, "pacejka")
        self.assertTrue(engine.use_dynamic_model)
        self.assertEqual(engine.dynamic_model.tire_model, "pacejka")
        self.assertIsInstance(engine.planner, FrenetLatticePlanner)
        self.assertIsInstance(engine.controller, LQRController)

        for _ in range(5):
            engine.step()
        self.assertTrue(np.isfinite(engine.ego_state.x))
        self.assertTrue(np.isfinite(engine.ego_state.yaw_rate))


if __name__ == "__main__":
    unittest.main()
