"""用于离线调参的确定性路径与障碍事件。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class Scenario:
    key: str
    name: str
    path: np.ndarray
    initial_pose: Tuple[float, float, float]
    max_time: float = 30.0
    obstacle_xy: Optional[Tuple[float, float]] = None
    obstacle_radius: float = 0.28
    obstacle_active_time: Tuple[float, float] = (float("inf"), float("inf"))


def _line(start, end, count=100):
    return np.column_stack(
        (np.linspace(start[0], end[0], count), np.linspace(start[1], end[1], count))
    )


def _arc(center, radius, start_angle, end_angle, count=80):
    angles = np.linspace(start_angle, end_angle, count)
    return np.column_stack(
        (center[0] + radius * np.cos(angles), center[1] + radius * np.sin(angles))
    )


def make_scenarios() -> Dict[str, Scenario]:
    straight = _line((0.0, 0.0), (6.0, 0.0), 160)

    curve = np.vstack(
        (
            _line((0.0, 0.0), (1.5, 0.0), 45)[:-1],
            _arc((1.5, 1.0), 1.0, -np.pi / 2, 0.0, 75)[:-1],
            _line((2.5, 1.0), (2.5, 4.0), 90),
        )
    )

    sharp = np.vstack(
        (
            _line((0.0, 0.0), (2.0, 0.0), 70)[:-1],
            _line((2.0, 0.0), (2.0, 3.0), 100),
        )
    )

    x = np.linspace(0.0, 7.0, 220)
    s_curve = np.column_stack((x, 0.65 * np.sin(1.25 * x)))

    ripple_x = np.linspace(0.0, 6.0, 260)
    ripple = np.column_stack((ripple_x, 0.08 * np.sin(7.0 * ripple_x)))

    obstacle_path = _line((0.0, 0.0), (7.0, 0.0), 220)

    return {
        "straight": Scenario("straight", "直线", straight, (0.0, 0.15, 0.0), 20.0),
        "curve_1m": Scenario("curve_1m", "1m半径弯", curve, (0.0, 0.0, 0.0), 30.0),
        "sharp_90": Scenario("sharp_90", "90度折角", sharp, (0.0, 0.0, 0.0), 35.0),
        "s_curve": Scenario("s_curve", "连续S弯", s_curve, (0.0, 0.0, 0.0), 30.0),
        "ripple": Scenario("ripple", "高频小波动", ripple, (0.0, 0.0, 0.0), 25.0),
        "dynamic_obstacle": Scenario(
            "dynamic_obstacle",
            "动态障碍出现后消失",
            obstacle_path,
            (0.0, 0.0, 0.0),
            30.0,
            obstacle_xy=(3.0, 0.0),
            obstacle_radius=0.3,
            obstacle_active_time=(2.5, 8.0),
        ),
    }


SCENARIOS = make_scenarios()
