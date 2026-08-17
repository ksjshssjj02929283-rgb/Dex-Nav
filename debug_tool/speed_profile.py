"""生产速度规划器的轻量滚动时域近似。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .config import SpeedPlannerConfig
from .geometry import cumulative_lengths, path_curvature, project_to_path


@dataclass
class SpeedProfile:
    arc: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    curvature: np.ndarray
    obstacle_index: Optional[int]


def build_speed_profile(
    path: np.ndarray,
    config: SpeedPlannerConfig,
    current_speed: float,
    obstacle_xy: Optional[Tuple[float, float]] = None,
    obstacle_radius: float = 0.0,
) -> SpeedProfile:
    """根据当前实测速度生成可行的速度包络。

    约束顺序与生产代码一致，包括速度上限、障碍、弯道、末端以及前后向运动限制；
    这里使用恒加速度传播，不复现 S 曲线的全部加加速度内部状态。
    """
    arc = cumulative_lengths(path)
    n = len(path)
    velocity = np.full(n, max(config.max_speed, 0.0), dtype=float)
    curvature = path_curvature(path, max(config.curve_compute_dist, 1e-3))

    radius_threshold = max(config.regulated_linear_scaling_min_radius, 1e-9)
    curve_indices = []
    for i, kappa in enumerate(curvature):
        radius = np.inf if abs(kappa) < 1e-9 else 1.0 / abs(kappa)
        if radius <= radius_threshold:
            cap = max(config.min_curvature_speed, config.max_speed * radius / radius_threshold)
            velocity[i] = min(velocity[i], cap)
            curve_indices.append(i)

    if curve_indices:
        for i in curve_indices:
            cap = velocity[i]
            lower = arc[i] - max(config.curve_lead_in_distance, 0.0)
            upper = arc[i] + max(config.curve_lead_out_distance, 0.0)
            mask = (arc >= lower) & (arc <= upper)
            velocity[mask] = np.minimum(velocity[mask], cap)

    obstacle_index: Optional[int] = None
    if obstacle_xy is not None:
        obstacle = np.asarray(obstacle_xy, dtype=float)
        distances = np.linalg.norm(path - obstacle, axis=1)
        candidates = np.flatnonzero(distances <= obstacle_radius)
        if len(candidates):
            obstacle_index = int(candidates[0])
            buffer_start = arc[obstacle_index] - max(config.collision_stop_distance, 0.0)
            velocity[arc >= buffer_start] = 0.0

    if n:
        velocity[-1] = min(velocity[-1], max(config.end_speed, 0.0))
        stop_start = max(0.0, arc[-1] - max(config.stop_distance, 0.0))
        velocity[arc >= stop_start] = np.minimum(
            velocity[arc >= stop_start], max(config.end_speed, 0.0)
        )

    # 先进行反向制动可行性传播，再进行正向加速可行性传播。
    braking = max(abs(config.decel), 1e-6)
    for i in range(n - 2, -1, -1):
        ds = max(arc[i + 1] - arc[i], 0.0)
        reachable = np.sqrt(max(velocity[i + 1] ** 2 + 2.0 * braking * ds, 0.0))
        velocity[i] = min(velocity[i], reachable)
    if n:
        velocity[0] = min(velocity[0], max(abs(current_speed), 0.0))
    acceleration_limit = max(config.accel, 1e-6)
    for i in range(1, n):
        ds = max(arc[i] - arc[i - 1], 0.0)
        reachable = np.sqrt(max(velocity[i - 1] ** 2 + 2.0 * acceleration_limit * ds, 0.0))
        velocity[i] = min(velocity[i], reachable)

    acceleration = np.zeros(n)
    for i in range(n - 1):
        ds = arc[i + 1] - arc[i]
        if ds > 1e-9:
            acceleration[i] = (velocity[i + 1] ** 2 - velocity[i] ** 2) / (2.0 * ds)
    if n > 1:
        acceleration[-1] = acceleration[-2]
    return SpeedProfile(arc, velocity, acceleration, curvature, obstacle_index)


def preview_speed(
    profile: SpeedProfile,
    path: np.ndarray,
    robot_xy: np.ndarray,
    measured_speed: float,
    time_lookahead: float,
    spatial_fallback: float,
) -> Tuple[float, float]:
    projection = project_to_path(path, robot_xy)
    effective_speed = max(abs(measured_speed), 0.05)
    preview_distance = max(effective_speed * max(time_lookahead, 0.0), spatial_fallback * 0.15)
    query_arc = min(projection.arc_length + preview_distance, profile.arc[-1])
    velocity = float(np.interp(query_arc, profile.arc, profile.velocity))
    acceleration = float(np.interp(query_arc, profile.arc, profile.acceleration))
    return velocity, acceleration
