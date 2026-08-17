from pathlib import Path
import unittest

import numpy as np

from debug_tool.config import DEFAULT_CONFIG, load_tool_config, validate_config
from debug_tool.control import apply_rpp_constraints
from debug_tool.geometry import carrot_point
from debug_tool.pid_lab import simulate_pid_step
from debug_tool.scenarios import SCENARIOS
from debug_tool.simulator import simulate_rpp


class DebugToolTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_tool_config(DEFAULT_CONFIG)

    def test_production_yaml_is_loaded(self):
        self.assertAlmostEqual(self.config.rpp.max_lookahead_dist, 0.9)
        self.assertAlmostEqual(self.config.smoother.max_angular_velocity, 0.75)
        self.assertAlmostEqual(self.config.costmap.cost_scaling_factor, 10.0)

    def test_current_known_configuration_issues_are_detected(self):
        parameters = {issue.parameter for issue in validate_config(self.config)}
        self.assertIn("heading_error_scaling", parameters)
        self.assertIn("lateral_error", parameters)
        self.assertIn("inflation_cost_scaling_factor", parameters)
        self.assertIn("max_angular_vel", parameters)

    def test_carrot_interpolates_circle_intersection(self):
        path = np.array([[0.0, 0.0], [0.5, 0.0], [0.8, 0.0]])
        carrot = carrot_point(path, np.array([0.0, 0.0]), 0.6)
        np.testing.assert_allclose(carrot, [0.6, 0.0], atol=1e-8)

    def test_constraints_never_raise_planner_speed(self):
        result = apply_rpp_constraints(
            0.6, 1.5, 2.0, 0.4, 0.2, 0.3, self.config.rpp
        )
        self.assertLessEqual(max(result.values()), 0.6 + 1e-9)
        sequence = list(result.values())
        self.assertTrue(all(b <= a + 1e-9 for a, b in zip(sequence, sequence[1:])))

    def test_straight_scenario_runs_and_produces_finite_channels(self):
        result = simulate_rpp(SCENARIOS["straight"], self.config)
        self.assertGreater(len(result.time), 10)
        self.assertTrue(np.all(np.isfinite(result.pose)))
        self.assertTrue(np.all(np.isfinite(result.channels["measured_v"])))

    def test_pid_step_response_is_finite_and_tracks_positive_target(self):
        response = simulate_pid_step(self.config.pid)
        self.assertTrue(np.all(np.isfinite(response.measured)))
        self.assertGreater(response.measured[-1], 0.0)
        self.assertLessEqual(response.measured[-1], response.target[-1] + 1e-6)


if __name__ == "__main__":
    unittest.main()
