"""实验性模块的代码级隔离测试。

目标：Game/EM/MPC/Pacejka/交叉口代码保留、可显式运行，但默认构建核心引擎时
必须标记为 experimental，避免它们悄悄进入主结果。
"""

import unittest

from src.environment.scenario_loader import load_named_scenario
from src.runner import build_engine


class TestExperimentalGuard(unittest.TestCase):

    def test_core_engine_is_not_experimental(self):
        scenario = load_named_scenario("highway_merge")
        engine = build_engine(scenario, decision="fsm", enable_visualization=False)
        self.assertFalse(engine.experimental)
        self.assertEqual(engine.experimental_reasons, [])

    def test_game_decision_is_marked_experimental(self):
        scenario = load_named_scenario("highway_merge")
        engine = build_engine(scenario, decision="game", enable_visualization=False)
        self.assertTrue(engine.experimental)
        self.assertIn("game_decision", engine.experimental_reasons)

    def test_em_planner_is_marked_experimental(self):
        scenario = load_named_scenario("highway_merge")
        engine = build_engine(scenario, decision="fsm", planner="em", enable_visualization=False)
        self.assertTrue(engine.experimental)
        self.assertIn("em_planner", engine.experimental_reasons)

    def test_mpc_controller_is_marked_experimental(self):
        scenario = load_named_scenario("highway_merge")
        engine = build_engine(scenario, decision="fsm", controller="mpc", enable_visualization=False)
        self.assertTrue(engine.experimental)
        self.assertIn("mpc_controller", engine.experimental_reasons)

    def test_pacejka_tire_is_marked_experimental(self):
        scenario = load_named_scenario("highway_merge")
        engine = build_engine(scenario, decision="fsm", tire_model="pacejka", enable_visualization=False)
        self.assertTrue(engine.experimental)
        self.assertIn("pacejka_tire", engine.experimental_reasons)

    def test_intersection_is_marked_experimental(self):
        scenario = load_named_scenario("intersection_unprotected")
        engine = build_engine(scenario, decision="fsm", enable_visualization=False)
        self.assertTrue(engine.experimental)
        self.assertIn("intersection_unprotected", engine.experimental_reasons)


if __name__ == "__main__":
    unittest.main()
