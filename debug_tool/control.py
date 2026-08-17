"""尽量贴近本仓库 RPP 实现的控制器基础计算。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

import numpy as np

from .config import PIDConfig, RPPConfig, SmootherConfig


@dataclass
class PIDState:
    integral: float = 0.0
    previous_error: float = 0.0
    first: bool = True

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = 0.0
        self.first = True

    def step(self, error: float, dt: float, config: PIDConfig) -> float:
        derivative = 0.0 if self.first else (error - self.previous_error) / max(dt, 1e-9)
        self.first = False
        self.integral += error * dt * config.ki
        limit = abs(config.integrator_saturation)
        self.integral = float(np.clip(self.integral, -limit, limit))
        self.previous_error = error
        return config.kp * error + self.integral + config.kd * derivative


@dataclass
class RPPState:
    speed_pid: PIDState
    last_speed_cmd: float = 0.0
    last_speed_valid: bool = False
    rotating: bool = False


@dataclass
class SmootherState:
    linear: float = 0.0
    angular: float = 0.0


def apply_rpp_constraints(
    target_speed: float,
    curvature: float,
    near_curvature: float,
    cross_track_error: float,
    heading_error: float,
    obstacle_distance: float,
    rpp: RPPConfig,
) -> Dict[str, float]:
    values: Dict[str, float] = {"planner": max(target_speed, 0.0)}
    speed = values["planner"]

    curvature_speed = speed
    if rpp.use_regulated_linear_velocity_scaling and abs(curvature) > 1e-9:
        radius = 1.0 / abs(curvature)
        if radius < rpp.regulated_linear_scaling_min_radius:
            curvature_speed *= radius / max(rpp.regulated_linear_scaling_min_radius, 1e-9)
    curvature_speed = max(
        curvature_speed,
        min(speed, max(rpp.regulated_linear_scaling_min_speed, 0.0)),
    )
    speed = min(speed, curvature_speed)
    values["curvature"] = speed

    if (
        rpp.use_cost_regulated_linear_velocity_scaling
        and np.isfinite(obstacle_distance)
        and obstacle_distance < rpp.cost_scaling_dist
    ):
        cost_speed = values["planner"] * rpp.cost_scaling_gain * max(obstacle_distance, 0.0) / max(
            rpp.cost_scaling_dist, 1e-9
        )
        cost_speed = max(cost_speed, min(values["planner"], rpp.regulated_linear_scaling_min_speed))
        speed = min(speed, cost_speed)
    values["obstacle"] = speed

    if cross_track_error > rpp.lateral_error_threshold:
        span = max(rpp.lateral_error_max_dist - rpp.lateral_error_threshold, 1e-3)
        fraction = (cross_track_error - rpp.lateral_error_threshold) / span
        scale = max(rpp.min_lateral_speed_scale, 1.0 - (1.0 - rpp.min_lateral_speed_scale) * fraction)
        speed *= scale
    values["lateral"] = speed

    absolute_heading = abs(heading_error)
    if absolute_heading > rpp.heading_error_scaling_min_angle:
        span = max(rpp.heading_error_scaling_max_angle - rpp.heading_error_scaling_min_angle, 1e-3)
        fraction = (absolute_heading - rpp.heading_error_scaling_min_angle) / span
        scale = max(rpp.min_heading_speed_scale, 1.0 - (1.0 - rpp.min_heading_speed_scale) * fraction)
        speed *= scale
    values["heading"] = speed

    max_curvature = max(abs(curvature), abs(near_curvature))
    demand = rpp.angular_gain * speed * max_curvature
    if max_curvature > 1e-3 and demand > rpp.max_angular_vel:
        speed = rpp.max_angular_vel / max(rpp.angular_gain * max_curvature, 1e-9)
    values["angular_capacity"] = speed
    return values


def speed_pid_step(
    target: float,
    measured: float,
    feedforward_accel: float,
    dt: float,
    rpp: RPPConfig,
    state: RPPState,
) -> float:
    if not state.last_speed_valid or abs(state.last_speed_cmd - measured) > rpp.speed_loop_cmd_reseed_error:
        state.last_speed_cmd = measured
        state.last_speed_valid = True
    pid_config = PIDConfig(
        kp=rpp.speed_loop_kp,
        ki=rpp.speed_loop_ki,
        kd=rpp.speed_loop_kd,
        integrator_saturation=rpp.speed_loop_integrator_saturation,
        max_accel=rpp.max_linear_accel,
        cmd_reseed_error=rpp.speed_loop_cmd_reseed_error,
    )
    acceleration = feedforward_accel + state.speed_pid.step(target - measured, dt, pid_config)
    acceleration = float(np.clip(acceleration, -abs(rpp.max_linear_accel), abs(rpp.max_linear_accel)))
    command = state.last_speed_cmd + acceleration * dt
    command = float(np.clip(command, 0.0, min(max(target, 0.0), rpp.desired_linear_vel)))
    state.last_speed_cmd = command
    return command


def _axis_smooth(current: float, target: float, accel: float, decel: float, dt: float) -> float:
    accelerating = abs(target) >= abs(current) and current * target >= 0.0
    limit = abs(accel if accelerating else decel) * dt
    return current + float(np.clip(target - current, -limit, limit))


def smoother_step(
    target_linear: float,
    target_angular: float,
    dt: float,
    config: SmootherConfig,
    state: SmootherState,
) -> tuple[float, float]:
    target_linear = float(np.clip(target_linear, config.min_linear_velocity, config.max_linear_velocity))
    target_angular = float(np.clip(target_angular, -config.max_angular_velocity, config.max_angular_velocity))
    state.linear = _axis_smooth(
        state.linear, target_linear, config.max_linear_accel, config.max_linear_decel, dt
    )
    state.angular = _axis_smooth(
        state.angular, target_angular, config.max_angular_accel, config.max_angular_decel, dt
    )
    return state.linear, state.angular
