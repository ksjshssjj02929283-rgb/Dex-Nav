"""用于参数分析的差速底盘 RPP 闭环仿真。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from .config import ToolConfig
from .control import (
    PIDState,
    RPPState,
    SmootherState,
    apply_rpp_constraints,
    smoother_step,
    speed_pid_step,
)
from .geometry import carrot_point, project_to_path, to_robot_frame, wrap_angle
from .scenarios import Scenario
from .speed_profile import build_speed_profile, preview_speed


@dataclass
class SimulationOptions:
    plant_linear_tau: float = 0.12
    plant_angular_tau: float = 0.10
    robot_collision_radius: float = 0.35
    goal_tolerance: float = 0.06
    settle_speed: float = 0.06


@dataclass
class SimulationResult:
    scenario: Scenario
    config: ToolConfig
    time: np.ndarray
    pose: np.ndarray
    main_carrot: np.ndarray
    near_carrot: np.ndarray
    channels: Dict[str, np.ndarray]
    states: List[str]
    completed: bool
    collision: bool
    metrics: Dict[str, float]
    events: List[str] = field(default_factory=list)


def _local_suffix(path: np.ndarray, robot_xy: np.ndarray) -> np.ndarray:
    projection = project_to_path(path, robot_xy)
    tail = path[projection.segment + 1 :]
    if len(tail) == 0:
        return np.vstack((projection.point, path[-1]))
    return np.vstack((projection.point, tail))


def _collision_imminent(
    pose: Tuple[float, float, float],
    linear: float,
    angular: float,
    horizon: float,
    max_distance: float,
    obstacle_xy: Optional[Tuple[float, float]],
    combined_radius: float,
) -> bool:
    if obstacle_xy is None or abs(linear) < 1e-6:
        return False
    obstacle = np.asarray(obstacle_xy)
    x, y, yaw = pose
    dt = 0.03
    elapsed = 0.0
    travelled = 0.0
    while elapsed < horizon and travelled < max_distance:
        x += linear * np.cos(yaw) * dt
        y += linear * np.sin(yaw) * dt
        yaw += angular * dt
        elapsed += dt
        travelled += abs(linear) * dt
        if np.linalg.norm(np.array((x, y)) - obstacle) <= combined_radius:
            return True
    return False


def _summary_metrics(
    channels: Dict[str, np.ndarray], states: List[str], completed: bool, collision: bool
) -> Dict[str, float]:
    rotate_entries = sum(
        1 for index, state in enumerate(states)
        if state == "ROTATE" and (index == 0 or states[index - 1] != "ROTATE")
    )
    return {
        "completed": float(completed),
        "collision": float(collision),
        "duration_s": float(channels["time"][-1]) if len(channels["time"]) else 0.0,
        "max_cross_track_error_m": float(np.max(channels["xte"])) if len(channels["xte"]) else 0.0,
        "mean_abs_cross_track_error_m": float(np.mean(np.abs(channels["xte"]))) if len(channels["xte"]) else 0.0,
        "mean_speed_mps": float(np.mean(np.abs(channels["measured_v"]))) if len(channels["measured_v"]) else 0.0,
        "max_abs_angular_speed_rps": float(np.max(np.abs(channels["measured_w"]))) if len(channels["measured_w"]) else 0.0,
        "rotate_entries": float(rotate_entries),
        "minimum_obstacle_clearance_m": float(np.min(channels["obstacle_distance"]))
        if len(channels["obstacle_distance"]) and np.any(np.isfinite(channels["obstacle_distance"]))
        else float("inf"),
    }


def simulate_rpp(
    scenario: Scenario,
    config: ToolConfig,
    options: Optional[SimulationOptions] = None,
) -> SimulationResult:
    options = options or SimulationOptions()
    dt = config.dt
    rpp = config.rpp
    pose = np.asarray(scenario.initial_pose, dtype=float)
    measured_v = 0.0
    measured_w = 0.0
    rpp_state = RPPState(PIDState())
    smoother_state = SmootherState()

    history: Dict[str, List[float]] = {
        key: [] for key in (
            "time", "planner", "curvature_cap", "obstacle_cap", "lateral_cap",
            "heading_cap", "angular_capacity_cap", "controller_v", "smoother_v",
            "measured_v", "controller_w", "smoother_w", "measured_w", "lookahead",
            "curvature", "near_curvature", "heading_error", "xte", "obstacle_distance",
        )
    }
    poses: List[np.ndarray] = []
    main_carrots: List[np.ndarray] = []
    near_carrots: List[np.ndarray] = []
    states: List[str] = []
    events: List[str] = []
    collision = False
    completed = False
    last_obstacle_active = False

    max_steps = int(np.ceil(scenario.max_time / dt))
    for step in range(max_steps):
        now = step * dt
        robot_xy = pose[:2]
        goal_distance = float(np.linalg.norm(robot_xy - scenario.path[-1]))
        if goal_distance <= options.goal_tolerance and abs(measured_v) <= options.settle_speed:
            completed = True
            events.append(f"{now:.2f}s：到达目标点")
            break

        projection = project_to_path(scenario.path, robot_xy)
        local_path = _local_suffix(scenario.path, robot_xy)
        obstacle_active = (
            scenario.obstacle_xy is not None
            and scenario.obstacle_active_time[0] <= now <= scenario.obstacle_active_time[1]
        )
        if obstacle_active != last_obstacle_active:
            events.append(f"{now:.2f}s：动态障碍物{'出现' if obstacle_active else '消失'}")
        last_obstacle_active = obstacle_active
        active_obstacle = scenario.obstacle_xy if obstacle_active else None

        profile = build_speed_profile(
            local_path,
            config.speed_planner,
            measured_v,
            active_obstacle,
            scenario.obstacle_radius + options.robot_collision_radius,
        )
        effective_time_preview = max(abs(rpp.traj_time_lookahead), config.dt)
        planner_v, planner_a = preview_speed(
            profile,
            local_path,
            robot_xy,
            measured_v,
            effective_time_preview,
            rpp.traj_speed_lookahead,
        )

        if rpp.use_velocity_scaled_lookahead_dist:
            lookahead = float(np.clip(
                abs(measured_v) * rpp.lookahead_time,
                rpp.min_lookahead_dist,
                rpp.max_lookahead_dist,
            ))
        else:
            lookahead = float(np.clip(
                rpp.min_lookahead_dist,
                rpp.min_lookahead_dist,
                rpp.max_lookahead_dist,
            ))

        main = carrot_point(scenario.path, robot_xy, lookahead)
        near = carrot_point(scenario.path, robot_xy, rpp.near_curvature_check_dist)
        main_robot = to_robot_frame(main, tuple(pose))
        near_robot = to_robot_frame(near, tuple(pose))
        main_dist_sq = float(np.dot(main_robot, main_robot))
        near_dist_sq = float(np.dot(near_robot, near_robot))
        curvature = 0.0 if main_dist_sq <= 1e-3 else 2.0 * main_robot[1] / main_dist_sq
        near_curvature = 0.0 if near_dist_sq <= 1e-3 else 2.0 * near_robot[1] / near_dist_sq
        heading_error = float(np.arctan2(main_robot[1], main_robot[0]))
        xte = projection.distance

        obstacle_distance = float("inf")
        if active_obstacle is not None:
            obstacle_distance = max(
                0.0,
                float(np.linalg.norm(robot_xy - np.asarray(active_obstacle)))
                - scenario.obstacle_radius,
            )

        exit_angle = min(rpp.rotate_to_path_exit_angle, rpp.rotate_to_path_entry_angle)
        rotate_threshold = exit_angle if rpp_state.rotating else rpp.rotate_to_path_entry_angle
        rotating = rpp.use_rotate_to_heading and abs(heading_error) > rotate_threshold
        rpp_state.rotating = rotating

        if rotating:
            controller_v = np.sign(measured_v) * max(
                0.0, abs(measured_v) - abs(rpp.max_linear_accel) * dt
            )
            desired_w = float(np.clip(
                rpp.rotate_to_path_angular_kp * heading_error,
                -rpp.rotate_to_path_angular_vel,
                rpp.rotate_to_path_angular_vel,
            ))
            controller_w = float(np.clip(
                desired_w,
                measured_w - abs(rpp.max_angular_accel) * dt,
                measured_w + abs(rpp.max_angular_accel) * dt,
            ))
            constraint_values = {
                "planner": planner_v,
                "curvature": planner_v,
                "obstacle": planner_v,
                "lateral": planner_v,
                "heading": planner_v,
                "angular_capacity": planner_v,
            }
            state_name = "ROTATE"
        else:
            constraint_values = apply_rpp_constraints(
                planner_v,
                curvature,
                near_curvature,
                xte,
                heading_error,
                obstacle_distance,
                rpp,
            )
            constrained_target = constraint_values["angular_capacity"]
            controller_v = speed_pid_step(
                constrained_target, measured_v, planner_a, dt, rpp, rpp_state
            )
            controller_w = float(np.clip(
                rpp.angular_gain * controller_v * curvature,
                -rpp.max_angular_vel,
                rpp.max_angular_vel,
            ))
            state_name = "TRACK"

        if obstacle_active and rpp.use_collision_detection and _collision_imminent(
            tuple(pose),
            controller_v,
            controller_w,
            rpp.max_allowed_time_to_collision_up_to_carrot,
            float(np.sqrt(main_dist_sq)),
            active_obstacle,
            scenario.obstacle_radius + options.robot_collision_radius,
        ):
            controller_v = 0.0
            controller_w = 0.0
            state_name = "COLLISION_STOP"
            collision = True
            events.append(f"{now:.2f}s：碰撞预测触发立即停止")

        smooth_v, smooth_w = smoother_step(
            controller_v, controller_w, dt, config.smoother, smoother_state
        )
        alpha_v = 1.0 if options.plant_linear_tau <= 0 else min(1.0, dt / options.plant_linear_tau)
        alpha_w = 1.0 if options.plant_angular_tau <= 0 else min(1.0, dt / options.plant_angular_tau)
        measured_v += alpha_v * (smooth_v - measured_v)
        measured_w += alpha_w * (smooth_w - measured_w)

        pose[0] += measured_v * np.cos(pose[2]) * dt
        pose[1] += measured_v * np.sin(pose[2]) * dt
        pose[2] = wrap_angle(pose[2] + measured_w * dt)

        poses.append(pose.copy())
        main_carrots.append(main.copy())
        near_carrots.append(near.copy())
        states.append(state_name)
        row = {
            "time": now,
            "planner": constraint_values["planner"],
            "curvature_cap": constraint_values["curvature"],
            "obstacle_cap": constraint_values["obstacle"],
            "lateral_cap": constraint_values["lateral"],
            "heading_cap": constraint_values["heading"],
            "angular_capacity_cap": constraint_values["angular_capacity"],
            "controller_v": controller_v,
            "smoother_v": smooth_v,
            "measured_v": measured_v,
            "controller_w": controller_w,
            "smoother_w": smooth_w,
            "measured_w": measured_w,
            "lookahead": lookahead,
            "curvature": curvature,
            "near_curvature": near_curvature,
            "heading_error": heading_error,
            "xte": xte,
            "obstacle_distance": obstacle_distance,
        }
        for key, value in row.items():
            history[key].append(float(value))

    channels = {key: np.asarray(value, dtype=float) for key, value in history.items()}
    pose_array = np.asarray(poses) if poses else np.empty((0, 3))
    metrics = _summary_metrics(channels, states, completed, collision)
    return SimulationResult(
        scenario=scenario,
        config=config,
        time=channels["time"],
        pose=pose_array,
        main_carrot=np.asarray(main_carrots) if main_carrots else np.empty((0, 2)),
        near_carrot=np.asarray(near_carrots) if near_carrots else np.empty((0, 2)),
        channels=channels,
        states=states,
        completed=completed,
        collision=collision,
        metrics=metrics,
        events=events,
    )
