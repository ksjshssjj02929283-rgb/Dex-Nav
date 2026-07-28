#!/usr/bin/env python3
# Copyright 2026 DEXE Robot Engineer
# SPDX-License-Identifier: Apache-2.0

import math
import os
import random
import sys
import unittest

SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from motion_diagnostics import (  # noqa: E402
    controller_target_dynamics,
    limits_from_task,
    local_linear_derivative,
    summarize_dynamics,
    velocity_dynamics,
)
from gen_test_nav_tasks import gen_task  # noqa: E402


class MotionDiagnosticsTest(unittest.TestCase):

    def test_controller_target_uses_explicit_acceleration_and_recovers_jerk(self):
        samples = [
            [round(i * 0.1, 3), 0.2, -0.2 + 0.5 * i * 0.1]
            for i in range(8)
        ]
        dynamics = controller_target_dynamics(samples, jerk_window_sec=0.3)
        self.assertEqual(
            dynamics["acceleration_source"], "explicit_controller_target")
        self.assertAlmostEqual(dynamics["acceleration"][3], -0.05, places=9)
        self.assertAlmostEqual(dynamics["jerk"][3], 0.5, places=9)

    def test_velocity_dynamics_estimates_constant_acceleration(self):
        samples = [[i * 0.05, 0.3 * i * 0.05, 0.0] for i in range(20)]
        dynamics = velocity_dynamics(
            samples, acceleration_window_sec=0.25, jerk_window_sec=0.3)
        for value in dynamics["acceleration"][3:-3]:
            self.assertAlmostEqual(value, 0.3, places=9)
        for value in dynamics["jerk"][4:-4]:
            self.assertAlmostEqual(value, 0.0, places=9)

    def test_derivative_does_not_bridge_long_sample_gap(self):
        times = [0.0, 0.1, 0.2, 2.0, 2.1, 2.2]
        values = [0.0, 0.1, 0.2, 10.0, 10.1, 10.2]
        derivative = local_linear_derivative(
            times, values, window_sec=0.3, max_gap_sec=0.5)
        for value in (derivative[1], derivative[4]):
            self.assertAlmostEqual(value, 1.0, places=9)
        self.assertTrue(math.isnan(derivative[0]))
        self.assertTrue(math.isnan(derivative[-1]))

    def test_summary_counts_contiguous_limit_events(self):
        dynamics = {
            "time": [0, 1, 2, 3, 4, 5],
            "acceleration": [0.0, 0.6, 0.7, 0.0, -0.6, -0.7],
            "jerk": [0.0, 0.8, 0.9, 0.0, -0.8, -0.9],
        }
        limits = {
            "max_acceleration": 0.5,
            "max_deceleration": 0.5,
            "max_abs_jerk": 0.5,
            "tolerance_ratio": 0.0,
        }
        summary = summarize_dynamics(dynamics, limits)
        self.assertEqual(summary["status"], "exceeded")
        self.assertEqual(summary["acceleration_exceedance_events"], 1)
        self.assertEqual(summary["deceleration_exceedance_events"], 1)
        self.assertEqual(summary["jerk_exceedance_events"], 2)

    def test_old_task_without_limits_remains_compatible(self):
        limits = limits_from_task({})
        self.assertIsNone(limits["max_acceleration"])
        self.assertIsNone(limits["max_deceleration"])
        self.assertIsNone(limits["max_abs_jerk"])
        summary = summarize_dynamics(
            {"time": [0.0], "acceleration": [0.1], "jerk": [math.nan]},
            limits)
        self.assertEqual(summary["status"], "no_limits")

    def test_generated_task_contains_complete_p0_cycle_schema(self):
        task = gen_task("A", "B", "succeeded", 1000.0, random.Random(7))
        self.assertEqual(task["schema_version"], 2)
        cycles = task["speed_plan_cycles"]
        self.assertGreater(len(cycles), 1)
        self.assertEqual(
            [cycle["cycle_id"] for cycle in cycles],
            list(range(1, len(cycles) + 1)))
        self.assertTrue(all("boundary" in cycle for cycle in cycles))
        self.assertTrue(all("terminal" in cycle for cycle in cycles))
        self.assertTrue(all("control_receipt" in cycle for cycle in cycles))
        self.assertEqual(
            task["speed_plan_cycle_summary"]["control_receipt_valid_count"],
            len(cycles))
        self.assertEqual(len(task["planned_path_ids"]), len(task["planned_paths"]))


if __name__ == "__main__":
    unittest.main()
