#!/usr/bin/env python3
"""
Chassis IMU/odom calibration recorder and offline analyzer.

The default mode is read-only. Automatic motion is available only with ``--enable-motion`` and an
interactive confirmation; it uses the lowest-priority cmd_vel_mux input and strict motion bounds.
The script never changes ROS parameters. Keep an emergency stop ready in either mode.
"""

import argparse
import bisect
import csv
from dataclasses import dataclass
from datetime import datetime
import json
import math
import os
import statistics
import sys
import time
from typing import List, Optional, Sequence, Tuple

from geometry_msgs.msg import Twist
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String


HARD_MAX_SPEED = 0.25
HARD_MAX_DISTANCE = 1.0
HARD_MAX_AUTO_CYCLES = 1
HARD_MAX_AUTO_RUNTIME = 30.0
HARD_MAX_ACTUAL_SPEED = 0.30
HARD_MAX_ACTUAL_YAW_RATE = 0.20
HARD_MAX_YAW_DEVIATION = 0.20
HARD_MAX_ROTATION_YAW_RATE = 0.30
HARD_MAX_ROTATION_DRIFT = 0.10


@dataclass
class OdomSample:
    stamp: float
    phase: int
    velocity_x: float
    angular_z: float
    position_x: float
    position_y: float
    yaw: float


@dataclass
class ImuSample:
    stamp: float
    phase: int
    acceleration_x: float
    acceleration_y: float
    acceleration_z: float
    angular_z: float
    orientation_x: float
    orientation_y: float
    orientation_z: float
    orientation_w: float


@dataclass
class CorrectedImuSample:
    stamp: float
    phase: int
    raw_x: float
    reference_aligned_gravity_x: float
    gravity_removed_x: float
    bias_corrected_x: float
    base_corrected_x: float
    angular_z: float


@dataclass
class OdomAccelerationSample:
    stamp: float
    phase: int
    velocity_x: float
    angular_z: float
    acceleration_x: float
    position_x: float
    position_y: float
    yaw: float


class CalibrationRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("calibrate_chassis_imu_acceleration")
        self.args = args
        self.start_monotonic = time.monotonic()
        self.odom_samples: List[OdomSample] = []
        self.imu_samples: List[ImuSample] = []
        self.active_source = ""
        self.command_publisher = None
        self.motion_phase = 0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, args.odom_topic, self._odom_callback, qos)
        self.create_subscription(Imu, args.imu_topic, self._imu_callback, qos)
        if args.enable_motion:
            active_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                String, args.mux_active_topic, self._active_callback, active_qos
            )
            self.command_publisher = self.create_publisher(Twist, args.command_topic, 10)

    def elapsed(self) -> float:
        return time.monotonic() - self.start_monotonic

    def phase(self) -> int:
        elapsed = self.elapsed()
        if elapsed < self.args.stationary_seconds:
            return 0
        if self.args.enable_motion:
            return self.motion_phase
        if elapsed < self.args.stationary_seconds + self.args.straight_seconds:
            return 1
        return 2

    def _stamp(self, msg_stamp) -> float:
        stamp = float(msg_stamp.sec) + 1e-9 * float(msg_stamp.nanosec)
        if stamp <= 0.0:
            return self.get_clock().now().nanoseconds * 1e-9
        return stamp

    def _odom_callback(self, msg: Odometry) -> None:
        orientation = msg.pose.pose.orientation
        self.odom_samples.append(
            OdomSample(
                self._stamp(msg.header.stamp), self.phase(),
                msg.twist.twist.linear.x, msg.twist.twist.angular.z,
                msg.pose.pose.position.x, msg.pose.pose.position.y,
                quaternion_yaw(
                    orientation.x, orientation.y, orientation.z, orientation.w
                ),
            )
        )

    def _imu_callback(self, msg: Imu) -> None:
        self.imu_samples.append(
            ImuSample(
                self._stamp(msg.header.stamp), self.phase(),
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
                msg.angular_velocity.z,
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            )
        )

    def _active_callback(self, msg: String) -> None:
        self.active_source = msg.data

    def publish_command(self, velocity: float) -> None:
        if self.command_publisher is None:
            return
        command = Twist()
        command.linear.x = velocity
        self.command_publisher.publish(command)


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def shortest_angle(first: float, second: float) -> float:
    return math.atan2(math.sin(second - first), math.cos(second - first))


def mean_std(values: Sequence[float]) -> Tuple[float, float]:
    if not values:
        return math.nan, math.nan
    return statistics.fmean(values), statistics.pstdev(values) if len(values) > 1 else 0.0


def sample_rate(stamps: Sequence[float]) -> float:
    if len(stamps) < 2 or stamps[-1] <= stamps[0]:
        return math.nan
    return (len(stamps) - 1) / (stamps[-1] - stamps[0])


def linear_slope(time_values: Sequence[float], values: Sequence[float]) -> Optional[float]:
    if len(time_values) != len(values) or len(values) < 2:
        return None
    mean_time = statistics.fmean(time_values)
    mean_value = statistics.fmean(values)
    denominator = sum((value - mean_time) ** 2 for value in time_values)
    if denominator <= 1e-12:
        return None
    numerator = sum(
        (stamp - mean_time) * (value - mean_value)
        for stamp, value in zip(time_values, values)
    )
    return numerator / denominator


def gravity_body_vector(sample: ImuSample, gravity: float) -> Tuple[float, float, float]:
    # sensor_msgs/Imu orientation is body -> world. R^T * [0, 0, +g] gives the
    # stationary accelerometer gravity vector in body coordinates for this driver (z ~= +g).
    norm = math.sqrt(
        sample.orientation_x ** 2 + sample.orientation_y ** 2 +
        sample.orientation_z ** 2 + sample.orientation_w ** 2
    )
    if norm <= 1e-9:
        return (math.nan, math.nan, math.nan)
    x = sample.orientation_x / norm
    y = sample.orientation_y / norm
    z = sample.orientation_z / norm
    w = sample.orientation_w / norm
    return (
        gravity * 2.0 * (x * z - y * w),
        gravity * 2.0 * (y * z + x * w),
        gravity * (1.0 - 2.0 * (x * x + y * y)),
    )


def gravity_body_x(sample: ImuSample, gravity: float) -> float:
    return gravity_body_vector(sample, gravity)[0]


def level_reference_x_axis(
    sample: ImuSample, gravity: float,
) -> Tuple[float, float, float]:
    gravity_vector = gravity_body_vector(sample, gravity)
    if not all(math.isfinite(value) for value in gravity_vector):
        return (math.nan, math.nan, math.nan)
    level_z = tuple(value / gravity for value in gravity_vector)
    seed = (1.0, 0.0, 0.0) if abs(level_z[0]) < 0.9 else (0.0, 1.0, 0.0)
    projection = sum(lhs * rhs for lhs, rhs in zip(seed, level_z))
    level_x = tuple(
        seed[index] - projection * level_z[index] for index in range(3)
    )
    norm = math.sqrt(sum(value * value for value in level_x))
    if norm <= 1e-9:
        return (math.nan, math.nan, math.nan)
    return tuple(value / norm for value in level_x)


def reference_aligned_gravity_x(
    sample: ImuSample, gravity: float, reference_x: Tuple[float, float, float],
) -> float:
    gravity_vector = gravity_body_vector(sample, gravity)
    if not all(math.isfinite(value) for value in gravity_vector + reference_x):
        return math.nan
    return sum(lhs * rhs for lhs, rhs in zip(reference_x, gravity_vector))


def compute_odom_acceleration(
    samples: Sequence[OdomSample], window: float, min_samples: int,
) -> List[OdomAccelerationSample]:
    output: List[OdomAccelerationSample] = []
    first = 0
    for index, current in enumerate(samples):
        while first < index and current.stamp - samples[first].stamp > window:
            first += 1
        window_samples = samples[first:index + 1]
        if len(window_samples) < min_samples:
            continue
        times = [sample.stamp - window_samples[0].stamp for sample in window_samples]
        velocities = [sample.velocity_x for sample in window_samples]
        slope = linear_slope(times, velocities)
        if slope is None or not math.isfinite(slope):
            continue
        output.append(
            OdomAccelerationSample(
                current.stamp, current.phase, current.velocity_x, current.angular_z, slope,
                current.position_x, current.position_y, current.yaw,
            )
        )
    return output


def interpolate(stamps: Sequence[float], values: Sequence[float], query: float) -> Optional[float]:
    if not stamps or query < stamps[0] or query > stamps[-1]:
        return None
    upper = bisect.bisect_left(stamps, query)
    if upper == 0:
        return values[0]
    if upper >= len(stamps):
        return values[-1]
    lower = upper - 1
    duration = stamps[upper] - stamps[lower]
    if duration <= 1e-9:
        return values[upper]
    ratio = (query - stamps[lower]) / duration
    return values[lower] + ratio * (values[upper] - values[lower])


def correlation(first: Sequence[float], second: Sequence[float]) -> float:
    if len(first) != len(second) or len(first) < 3:
        return math.nan
    mean_first = statistics.fmean(first)
    mean_second = statistics.fmean(second)
    numerator = sum(
        (lhs - mean_first) * (rhs - mean_second) for lhs, rhs in zip(first, second)
    )
    first_energy = sum((value - mean_first) ** 2 for value in first)
    second_energy = sum((value - mean_second) ** 2 for value in second)
    denominator = math.sqrt(first_energy * second_energy)
    return numerator / denominator if denominator > 1e-12 else math.nan


def linear_fit(
    independent: Sequence[float], dependent: Sequence[float]
) -> Tuple[float, float, float]:
    if len(independent) != len(dependent) or len(independent) < 3:
        return math.nan, math.nan, math.nan
    slope = linear_slope(independent, dependent)
    if slope is None:
        return math.nan, math.nan, math.nan
    intercept = statistics.fmean(dependent) - slope * statistics.fmean(independent)
    rmse = math.sqrt(
        statistics.fmean(
            [(target - (slope * source + intercept)) ** 2
             for source, target in zip(independent, dependent)]
        )
    )
    return slope, intercept, rmse


def write_csv(
    output_dir: str,
    odom: Sequence[OdomAccelerationSample],
    imu: Sequence[CorrectedImuSample],
) -> None:
    os.makedirs(output_dir, exist_ok=True)
    odom_path = os.path.join(output_dir, "odom.csv")
    imu_path = os.path.join(output_dir, "imu.csv")
    with open(odom_path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "stamp", "phase", "velocity_x", "angular_z", "acceleration_x",
            "position_x", "position_y", "yaw",
        ])
        for sample in odom:
            writer.writerow([
                sample.stamp, sample.phase, sample.velocity_x,
                sample.angular_z, sample.acceleration_x,
                sample.position_x, sample.position_y, sample.yaw,
            ])
    with open(imu_path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "stamp", "phase", "raw_x", "reference_aligned_gravity_x",
            "gravity_removed_x",
            "bias_corrected_x", "base_corrected_x", "angular_z",
        ])
        for sample in imu:
            writer.writerow([
                sample.stamp, sample.phase, sample.raw_x,
                sample.reference_aligned_gravity_x, sample.gravity_removed_x,
                sample.bias_corrected_x,
                sample.base_corrected_x, sample.angular_z,
            ])


def yaml_number(value: float) -> str:
    return f"{value:.8f}" if math.isfinite(value) else ".nan"


def write_recommended_parameters(output_dir: str, result: dict) -> None:
    imu = result["recommended_parameters"]["future_imu_fusion"]
    lines = [
        "# Generated by calibrate_chassis_imu_acceleration.py; never applied automatically.",
        "# Copy this node block into the Nav2 parameter file after reviewing calibration quality.",
        "longitudinal_state_estimator:",
        "  ros__parameters:",
        f"    odom_topic: {result['topics']['odom']}",
        f"    imu_topic: {imu['topic']}",
        "    state_topic: /navigation/longitudinal_state",
        "    publish_frequency: 100.00000000",
        "    imu:",
        "      input_is_level_aligned: true",
        f"      gravity: {yaml_number(imu['gravity'])}",
        "      static_bias: 0.00000000",
        f"      scale_to_odom: {yaml_number(imu['scale_to_odom'])}",
        f"      residual_intercept: {yaml_number(imu['residual_intercept'])}",
        f"      imu_x_offset: {yaml_number(imu['imu_x_offset'])}",
        "      max_age: 0.05000000",
        "      max_abs_acceleration: 1.50000000",
        "    estimator:",
        f"      enabled: {'true' if imu['ready'] else 'false'}",
        "      fusion_delay: 0.03000000",
        "      buffer_duration: 0.50000000",
        "      max_age: 0.10000000",
        "      imu_acceleration_noise: 0.10000000",
        "      bias_process_noise: 0.00050000",
        "      odom_velocity_noise: 0.02000000",
        "      hard_velocity_reanchor: 0.08000000",
        "      acceleration_filter_tau: 0.05000000",
        "",
        "# Calibration quality (diagnostic only; not estimator parameters):",
        f"# correlation={yaml_number(imu['correlation'])}",
        f"# rmse={yaml_number(imu['rmse'])}",
        f"# observed_fit_intercept={yaml_number(imu['observed_fit_intercept'])}",
        f"# rejection_reason={imu['rejection_reason']}",
    ]
    path = os.path.join(output_dir, "recommended_params.yaml")
    with open(path, "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines) + "\n")


def analyze(recorder: CalibrationRecorder, args: argparse.Namespace) -> dict:
    stationary_imu = [sample for sample in recorder.imu_samples if sample.phase == 0]
    raw_stationary_x = [sample.acceleration_x for sample in stationary_imu]
    reference_x = (
        level_reference_x_axis(stationary_imu[0], args.gravity)
        if stationary_imu else (math.nan, math.nan, math.nan)
    )
    reference_aligned_stationary = [
        reference_aligned_gravity_x(sample, args.gravity, reference_x)
        for sample in stationary_imu
    ]
    gravity_removed_stationary = [
        sample.acceleration_x - gravity_x
        for sample, gravity_x in zip(stationary_imu, reference_aligned_stationary)
    ]
    no_gravity_stationary = [
        sample.acceleration_x - gravity_body_x(sample, args.gravity)
        for sample in stationary_imu
    ]
    # Transform original-frame orientation gravity through the same kind of static level
    # reference before comparing it with robot_server's already level-aligned acceleration.
    raw_bias_x, raw_stationary_std = mean_std(raw_stationary_x)
    bias_x, stationary_std = mean_std(gravity_removed_stationary)
    gravity_model_residual_x, gravity_model_residual_std = mean_std(no_gravity_stationary)
    if not math.isfinite(bias_x):
        bias_x = 0.0

    corrected_imu: List[CorrectedImuSample] = []
    for sample in recorder.imu_samples:
        aligned_gravity_x = reference_aligned_gravity_x(
            sample, args.gravity, reference_x
        )
        gravity_removed_x = sample.acceleration_x - aligned_gravity_x
        bias_corrected_x = gravity_removed_x - bias_x
        # IMU is r metres in front of base_link. During yaw, sensor x includes
        # centripetal -w^2*r, so add w^2*r to recover base-centre longitudinal accel.
        base_corrected_x = bias_corrected_x + sample.angular_z ** 2 * args.imu_x_offset
        corrected_imu.append(
            CorrectedImuSample(
                sample.stamp, sample.phase, sample.acceleration_x,
                aligned_gravity_x, gravity_removed_x,
                bias_corrected_x, base_corrected_x, sample.angular_z,
            )
        )

    odom_acceleration = compute_odom_acceleration(
        recorder.odom_samples, args.odom_regression_window, args.odom_min_samples,
    )
    imu_stamps = [sample.stamp for sample in corrected_imu]
    imu_values = [sample.base_corrected_x for sample in corrected_imu]

    straight_odom = [
        sample for sample in odom_acceleration
        if sample.phase == 1 and abs(sample.angular_z) <= args.max_straight_yaw_rate and
        abs(sample.acceleration_x) >= args.min_dynamic_acceleration
    ]
    best_lag = math.nan
    best_correlation = math.nan
    best_imu: List[float] = []
    best_odom: List[float] = []
    lag_steps = int(round(args.max_lag / args.lag_step))
    for step in range(-lag_steps, lag_steps + 1):
        lag = step * args.lag_step
        imu_values_for_lag: List[float] = []
        odom_values_for_lag: List[float] = []
        for sample in straight_odom:
            imu_value = interpolate(imu_stamps, imu_values, sample.stamp + lag)
            if imu_value is not None:
                imu_values_for_lag.append(imu_value)
                odom_values_for_lag.append(sample.acceleration_x)
        value = correlation(imu_values_for_lag, odom_values_for_lag)
        if math.isfinite(value) and (
            not math.isfinite(best_correlation) or value > best_correlation
        ):
            best_lag = lag
            best_correlation = value
            best_imu = imu_values_for_lag
            best_odom = odom_values_for_lag

    scale, intercept, rmse = linear_fit(best_imu, best_odom)
    rotation_samples = [sample for sample in corrected_imu if sample.phase == 2]
    rotation_raw_rms = math.sqrt(statistics.fmean(
        [sample.bias_corrected_x ** 2 for sample in rotation_samples]
    )) if rotation_samples else math.nan
    rotation_corrected_rms = math.sqrt(statistics.fmean(
        [sample.base_corrected_x ** 2 for sample in rotation_samples]
    )) if rotation_samples else math.nan

    raw_x_mean, raw_x_std = mean_std([sample.acceleration_x for sample in stationary_imu])
    raw_y_mean, raw_y_std = mean_std([sample.acceleration_y for sample in stationary_imu])
    raw_z_mean, raw_z_std = mean_std([sample.acceleration_z for sample in stationary_imu])
    odom_rate = sample_rate([sample.stamp for sample in recorder.odom_samples])
    imu_rate = sample_rate([sample.stamp for sample in recorder.imu_samples])
    if math.isfinite(odom_rate) and odom_rate > 1.0:
        recommended_window = round(min(0.15, max(0.08, 10.0 / odom_rate)), 3)
        recommended_min_span = round(min(
            recommended_window, max(0.04, 5.0 / odom_rate)
        ), 3)
        recommended_min_samples = max(
            3, int(round(odom_rate * recommended_min_span)) + 1
        )
        recommended_max_age = round(min(0.10, max(0.03, 5.0 / odom_rate)), 3)
    else:
        recommended_window = args.odom_regression_window
        recommended_min_span = 0.05
        recommended_min_samples = args.odom_min_samples
        recommended_max_age = 0.05

    rejection_reasons = []
    if len(best_imu) < 100:
        rejection_reasons.append("dynamic_straight_pairs<100")
    if not math.isfinite(best_correlation) or best_correlation < 0.6:
        rejection_reasons.append("correlation<0.6")
    if not math.isfinite(scale) or scale < 0.5 or scale > 1.5:
        rejection_reasons.append("scale_outside_[0.5,1.5]")
    if not math.isfinite(rmse) or rmse > 0.15:
        rejection_reasons.append("rmse>0.15")
    if not math.isfinite(stationary_std) or stationary_std > 0.05:
        rejection_reasons.append("stationary_std>0.05")
    if best_correlation >= 0.70:
        recommended_imu_weight = 0.50
    elif best_correlation >= 0.60:
        recommended_imu_weight = 0.30
    else:
        recommended_imu_weight = 0.0
    result = {
        "output_directory": args.output_dir,
        "topics": {"odom": args.odom_topic, "imu": args.imu_topic},
        "motion": {
            "mode": "automatic_low_priority" if args.enable_motion else "manual_read_only",
            "command_topic": args.command_topic if args.enable_motion else "",
            "max_command_speed": args.auto_max_speed if args.enable_motion else math.nan,
            "auto_cycles": args.auto_cycles if args.enable_motion else 0,
            "auto_angular_speed": (
                args.auto_angular_speed if args.enable_motion else math.nan
            ),
            "planned_round_trip_distance": (
                2.0 * profile_distance(build_auto_profile(args))
                if args.enable_motion else math.nan
            ),
            "hard_max_speed": HARD_MAX_SPEED,
            "hard_max_distance": HARD_MAX_DISTANCE,
        },
        "sample_counts": {
            "odom": len(recorder.odom_samples),
            "imu": len(recorder.imu_samples),
            "stationary_imu": len(stationary_imu),
            "dynamic_straight_pairs": len(best_imu),
            "rotation_imu": len(rotation_samples),
        },
        "measured_rates_hz": {"odom": odom_rate, "imu": imu_rate},
        "stationary": {
            "raw_x_mean": raw_x_mean, "raw_x_std": raw_x_std,
            "raw_y_mean": raw_y_mean, "raw_y_std": raw_y_std,
            "raw_z_mean": raw_z_mean, "raw_z_std": raw_z_std,
            "flat_floor_raw_bias_x": raw_bias_x,
            "flat_floor_raw_std_x": raw_stationary_std,
            "orientation_gravity_model_residual_mean_x": gravity_model_residual_x,
            "orientation_gravity_model_residual_std_x": gravity_model_residual_std,
            "reference_aligned_gravity_compensation_used_for_fusion": True,
        },
        "straight_dynamic": {
            "imu_query_offset_to_odom_s": best_lag,
            "imu_lead_over_odom_s": -best_lag if math.isfinite(best_lag) else math.nan,
            "correlation": best_correlation,
            "odom_equals_scale_times_imu_plus_intercept": {
                "scale": scale, "intercept": intercept, "rmse": rmse,
            },
        },
        "rotation": {
            "imu_x_offset_m": args.imu_x_offset,
            "bias_corrected_x_rms_before_lever_compensation": rotation_raw_rms,
            "base_x_rms_after_lever_compensation": rotation_corrected_rms,
        },
        "estimator": {
            "odom_regression_window_s": args.odom_regression_window,
            "odom_min_samples": args.odom_min_samples,
        },
        "recommended_parameters": {
            "current_odom_estimator": {
                "enabled": True,
                "window_duration": recommended_window,
                "min_samples": recommended_min_samples,
                "min_time_span": recommended_min_span,
                "max_age": recommended_max_age,
                "max_abs_acceleration": 1.50,
            },
            "future_imu_fusion": {
                "ready": not rejection_reasons,
                "topic": args.imu_topic,
                "gravity": args.gravity,
                "input_is_level_aligned": True,
                "reference_aligned_gravity_compensation_enabled": True,
                "imu_x_offset": args.imu_x_offset,
                "stationary_bias_x": bias_x,
                "scale_to_odom": scale,
                "residual_intercept": 0.0,
                "observed_fit_intercept": intercept,
                "imu_lead_over_odom": (
                    -best_lag if math.isfinite(best_lag) else math.nan
                ),
                "recommended_weight_straight": recommended_imu_weight,
                "full_weight_max_abs_wz": 0.10,
                "zero_weight_min_abs_wz": 0.15,
                "correlation": best_correlation,
                "rmse": rmse,
                "rejection_reason": ",".join(rejection_reasons),
            },
        },
    }
    write_csv(args.output_dir, odom_acceleration, corrected_imu)
    write_recommended_parameters(args.output_dir, result)
    summary_path = os.path.join(args.output_dir, "summary.json")
    with open(summary_path, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, allow_nan=True)
    return result


def build_auto_profile(args: argparse.Namespace) -> List[Tuple[float, float, float]]:
    high = min(max(args.auto_max_speed, 0.05), HARD_MAX_SPEED)
    low = 0.60 * high
    creep = 0.40 * high
    acceleration = 0.25
    deceleration = 0.30
    one_cycle = [
        (low / acceleration, 0.0, low),
        (0.55, low, low),
        ((high - low) / acceleration, low, high),
        (0.80, high, high),
        ((high - creep) / deceleration, high, creep),
        (0.30, creep, creep),
        (creep / 0.25, creep, 0.0),
        (0.50, 0.0, 0.0),
    ]
    return one_cycle


def profile_duration(profile: Sequence[Tuple[float, float, float]]) -> float:
    return sum(segment[0] for segment in profile)


def profile_distance(profile: Sequence[Tuple[float, float, float]]) -> float:
    return sum(0.5 * (start + end) * duration for duration, start, end in profile)


def profile_velocity(
    profile: Sequence[Tuple[float, float, float]], elapsed: float
) -> float:
    cursor = 0.0
    for duration, start, end in profile:
        if elapsed <= cursor + duration:
            ratio = max(0.0, min(1.0, (elapsed - cursor) / max(duration, 1e-9)))
            return start + ratio * (end - start)
        cursor += duration
    return 0.0


def message_age(recorder: CalibrationRecorder, stamp: float) -> float:
    return recorder.get_clock().now().nanoseconds * 1e-9 - stamp


def traveled_distance(samples: Sequence[OdomSample]) -> float:
    return sum(
        math.hypot(
            samples[index].position_x - samples[index - 1].position_x,
            samples[index].position_y - samples[index - 1].position_y,
        )
        for index in range(1, len(samples))
    )


def publish_zero_for(
    recorder: CalibrationRecorder, args: argparse.Namespace, duration: float
) -> None:
    if recorder.command_publisher is None:
        return
    deadline = time.monotonic() + duration
    period = 1.0 / args.command_rate
    while rclpy.ok() and time.monotonic() < deadline:
        recorder.publish_command(0.0)
        drain_callbacks(recorder, min(0.002, period), 12)
        time.sleep(max(0.0, period - 0.005))


def publish_stop(recorder: CalibrationRecorder, args: argparse.Namespace) -> None:
    publish_zero_for(recorder, args, args.stop_publish_seconds)


def drain_callbacks(
    recorder: CalibrationRecorder, first_timeout: float = 0.002, max_callbacks: int = 12
) -> None:
    """Process enough queued callbacks for two 100 Hz inputs inside one 50 Hz control tick."""
    rclpy.spin_once(recorder, timeout_sec=first_timeout)
    for _ in range(max_callbacks - 1):
        rclpy.spin_once(recorder, timeout_sec=0.0)


def verify_motion_preflight(recorder: CalibrationRecorder, args: argparse.Namespace) -> None:
    if len(recorder.odom_samples) < args.odom_min_samples or len(recorder.imu_samples) < 20:
        raise RuntimeError("odom或IMU样本不足，禁止自动运动")
    odom = recorder.odom_samples[-1]
    imu = recorder.imu_samples[-1]
    odom_age = message_age(recorder, odom.stamp)
    imu_age = message_age(recorder, imu.stamp)
    if odom_age < -0.01 or odom_age > args.max_data_age:
        raise RuntimeError("odom数据过期，禁止自动运动")
    if imu_age < -0.01 or imu_age > args.max_data_age:
        raise RuntimeError("IMU数据过期，禁止自动运动")
    if abs(odom.velocity_x) > 0.01 or abs(odom.angular_z) > 0.02:
        raise RuntimeError("机器人在标定开始前未静止，禁止自动运动")
    if recorder.active_source != "idle":
        raise RuntimeError(
            f"cmd_vel_mux当前输入为'{recorder.active_source or 'unknown'}'，"
            "请先停止导航/遥控或检查/active话题"
        )
    if (
        recorder.command_publisher is None or
        recorder.command_publisher.get_subscription_count() < 1
    ):
        raise RuntimeError("标定速度话题没有cmd_vel_mux订阅者")
    # The publisher itself is one endpoint. Any additional endpoint means another process is
    # already using this otherwise-unused lowest-priority mux input.
    if recorder.count_publishers(args.command_topic) > 1:
        raise RuntimeError(f"{args.command_topic}已有其他发布者，禁止自动运动")


def verify_motion_runtime(
    recorder: CalibrationRecorder,
    args: argparse.Namespace,
    motion_samples: Sequence[OdomSample],
    phase_samples: Sequence[OdomSample],
    motion_start: float,
    rotating: bool,
) -> None:
    if not motion_samples or not phase_samples or not recorder.imu_samples:
        raise RuntimeError("运动中odom或IMU中断")
    odom = motion_samples[-1]
    imu = recorder.imu_samples[-1]
    odom_age = message_age(recorder, odom.stamp)
    imu_age = message_age(recorder, imu.stamp)
    if odom_age < -0.01 or odom_age > args.max_data_age:
        raise RuntimeError(
            f"运动中odom超时：age={odom_age:.3f}s，limit={args.max_data_age:.3f}s"
        )
    if imu_age < -0.01 or imu_age > args.max_data_age:
        raise RuntimeError(
            f"运动中IMU超时：age={imu_age:.3f}s，limit={args.max_data_age:.3f}s"
        )
    if recorder.active_source != args.expected_mux_source:
        raise RuntimeError(
            f"速度控制被'{recorder.active_source or 'unknown'}'抢占，停止标定"
        )
    if abs(odom.velocity_x) > args.max_actual_speed:
        raise RuntimeError(f"实测速度超限：{odom.velocity_x:.3f}m/s")
    if odom.velocity_x < -0.05:
        raise RuntimeError(f"检测到非预期倒车：{odom.velocity_x:.3f}m/s")
    yaw_rate_limit = (
        args.max_rotation_actual_yaw_rate if rotating else args.max_actual_yaw_rate
    )
    if abs(odom.angular_z) > yaw_rate_limit:
        raise RuntimeError(f"实测角速度超限：{odom.angular_z:.3f}rad/s")
    distance = traveled_distance(motion_samples)
    if distance > args.max_distance:
        raise RuntimeError(f"累计位移超限：{distance:.3f}m")
    if rotating:
        rotation_drift = math.hypot(
            odom.position_x - phase_samples[0].position_x,
            odom.position_y - phase_samples[0].position_y,
        )
        if rotation_drift > args.max_rotation_drift:
            raise RuntimeError(f"原地旋转位置漂移超限：{rotation_drift:.3f}m")
        if abs(odom.velocity_x) > 0.05:
            raise RuntimeError(f"原地旋转线速度超限：{odom.velocity_x:.3f}m/s")
    else:
        heading_change = abs(shortest_angle(phase_samples[0].yaw, odom.yaw))
        if heading_change > args.max_yaw_deviation:
            raise RuntimeError(f"直线航向偏移超限：{heading_change:.3f}rad")
    if time.monotonic() - motion_start > HARD_MAX_AUTO_RUNTIME:
        raise RuntimeError("自动运动超过30s硬上限")


def run_straight_leg(
    recorder: CalibrationRecorder,
    args: argparse.Namespace,
    profile: Sequence[Tuple[float, float, float]],
    motion_start: float,
    odom_start_index: int,
) -> None:
    recorder.motion_phase = 1
    phase_start_index = max(odom_start_index, len(recorder.odom_samples) - 1)
    duration = profile_duration(profile)
    period = 1.0 / args.command_rate
    leg_start = time.monotonic()
    while rclpy.ok() and time.monotonic() - leg_start < duration:
        cycle_start = time.monotonic()
        drain_callbacks(recorder, min(0.002, period), 12)
        motion_samples = recorder.odom_samples[odom_start_index:]
        phase_samples = recorder.odom_samples[phase_start_index:]
        verify_motion_runtime(
            recorder, args, motion_samples, phase_samples, motion_start, False
        )
        command = profile_velocity(profile, time.monotonic() - leg_start)
        if command < -1e-9 or command > HARD_MAX_SPEED + 1e-9:
            raise RuntimeError(f"内部速度曲线越界：{command:.3f}m/s")
        recorder.publish_command(command)
        time.sleep(max(0.0, period - (time.monotonic() - cycle_start)))


def accumulated_yaw(samples: Sequence[OdomSample]) -> float:
    return sum(
        shortest_angle(samples[index - 1].yaw, samples[index].yaw)
        for index in range(1, len(samples))
    )


def run_half_turn(
    recorder: CalibrationRecorder,
    args: argparse.Namespace,
    motion_start: float,
    odom_start_index: int,
) -> None:
    recorder.motion_phase = 2
    publish_zero_for(recorder, args, 0.40)
    phase_start_index = max(odom_start_index, len(recorder.odom_samples) - 1)
    period = 1.0 / args.command_rate
    command = 0.0
    rotation_start = time.monotonic()
    while rclpy.ok():
        cycle_start = time.monotonic()
        drain_callbacks(recorder, min(0.002, period), 12)
        motion_samples = recorder.odom_samples[odom_start_index:]
        phase_samples = recorder.odom_samples[phase_start_index:]
        verify_motion_runtime(
            recorder, args, motion_samples, phase_samples, motion_start, True
        )
        turned = accumulated_yaw(phase_samples)
        if turned < -0.05:
            raise RuntimeError(f"原地旋转方向异常：{turned:.3f}rad")
        if turned > math.pi + 0.15:
            raise RuntimeError(f"原地旋转过冲：{turned:.3f}rad")
        remaining = max(0.0, math.pi - turned)
        if remaining <= args.rotation_tolerance:
            break
        target = min(
            args.auto_angular_speed,
            math.sqrt(2.0 * args.auto_angular_decel * remaining),
        )
        target = max(target, 0.04)
        max_change = args.auto_angular_accel * period
        command += max(-max_change, min(max_change, target - command))
        twist = Twist()
        twist.angular.z = command
        recorder.command_publisher.publish(twist)
        if time.monotonic() - rotation_start > args.rotation_timeout:
            raise RuntimeError("180度旋转超时")
        time.sleep(max(0.0, period - (time.monotonic() - cycle_start)))
    publish_zero_for(recorder, args, 0.80)
    final_phase_samples = recorder.odom_samples[phase_start_index:]
    final_turned = accumulated_yaw(final_phase_samples)
    if abs(recorder.odom_samples[-1].angular_z) > 0.03:
        raise RuntimeError("180度旋转后未停稳")
    if final_turned > math.pi + 0.15:
        raise RuntimeError(f"180度旋转停车后过冲：{final_turned:.3f}rad")


def run_auto_motion(
    recorder: CalibrationRecorder,
    args: argparse.Namespace,
    profile: Sequence[Tuple[float, float, float]],
) -> None:
    verify_motion_preflight(recorder, args)
    print(
        f"自动往返标定开始：单程{profile_distance(profile):.2f}m、"
        f"最高线速度{args.auto_max_speed:.2f}m/s、180度最大角速度"
        f"{args.auto_angular_speed:.2f}rad/s。任何其他mux输入均可抢占。"
    )

    # Publish zero first to acquire the lowest-priority mux slot and verify the selected source.
    arm_deadline = time.monotonic() + 0.40
    while rclpy.ok() and time.monotonic() < arm_deadline:
        recorder.publish_command(0.0)
        drain_callbacks(recorder, 0.002, 12)
    if recorder.active_source != args.expected_mux_source:
        raise RuntimeError(
            f"未取得cmd_vel_mux输入，active='{recorder.active_source or 'unknown'}'"
        )

    motion_start = time.monotonic()
    odom_start_index = max(0, len(recorder.odom_samples) - 1)
    print("自动阶段：低速直线驶出。")
    run_straight_leg(recorder, args, profile, motion_start, odom_start_index)
    print("自动阶段：原地左转180度。")
    run_half_turn(recorder, args, motion_start, odom_start_index)
    print("自动阶段：沿原方向低速驶回。")
    run_straight_leg(recorder, args, profile, motion_start, odom_start_index)
    publish_zero_for(recorder, args, 0.80)
    motion_samples = recorder.odom_samples[odom_start_index:]
    return_error = math.hypot(
        motion_samples[-1].position_x - motion_samples[0].position_x,
        motion_samples[-1].position_y - motion_samples[0].position_y,
    )
    if return_error > args.max_return_error:
        raise RuntimeError(f"往返结束位置误差过大：{return_error:.3f}m")


def print_phase(phase: int, args: argparse.Namespace) -> None:
    if phase == 0:
        phase_count = 2 if args.enable_motion else 3
        print(
            f"[阶段1/{phase_count}] 保持机器人完全静止 {args.stationary_seconds:.1f}s，"
            "用于估计IMU偏置和噪声。"
        )
    elif phase == 1:
        if args.enable_motion:
            print("[阶段2/2] 脚本将执行低速直线驶出、原地转180度、低速驶回。")
            return
        print(
            f"[阶段2/{3}] 在安全直线上人工执行多次平缓加速/减速 {args.straight_seconds:.1f}s；"
            "尽量保持角速度接近0。"
        )
    else:
        print(
            f"[阶段3/{3}] 原地向左右各旋转数次 {args.rotation_seconds:.1f}s；"
            "用于评估IMU前置安装造成的旋转污染。"
        )


def unique_output_directory(path: str) -> str:
    expanded = os.path.abspath(os.path.expanduser(path))
    if not os.path.exists(expanded):
        return expanded
    for index in range(1, 1000):
        candidate = f"{expanded}_{index:02d}"
        if not os.path.exists(candidate):
            return candidate
    raise RuntimeError(f"无法为输出目录生成唯一名称：{expanded}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--odom-topic", default="/feedback/odom")
    parser.add_argument("--imu-topic", default="/imu/chassis/refined")
    parser.add_argument("--stationary-seconds", type=float, default=5.0)
    parser.add_argument("--straight-seconds", type=float, default=20.0)
    parser.add_argument("--rotation-seconds", type=float, default=10.0)
    parser.add_argument("--odom-regression-window", type=float, default=0.10)
    parser.add_argument("--odom-min-samples", type=int, default=6)
    parser.add_argument("--gravity", type=float, default=9.7887)
    parser.add_argument("--imu-x-offset", type=float, default=0.273)
    parser.add_argument("--max-straight-yaw-rate", type=float, default=0.10)
    parser.add_argument("--min-dynamic-acceleration", type=float, default=0.05)
    parser.add_argument("--max-lag", type=float, default=0.30)
    parser.add_argument("--lag-step", type=float, default=0.005)
    parser.add_argument(
        "--output-dir", default="",
        help="结果文件夹；默认在家目录创建带时间戳的独立文件夹",
    )
    parser.add_argument("--output-prefix", default="", help=argparse.SUPPRESS)
    parser.add_argument(
        "--enable-motion", action="store_true",
        help="显式启用自动直线往返与180度旋转；默认只订阅分析",
    )
    parser.add_argument("--command-topic", default="/input/default")
    parser.add_argument("--mux-active-topic", default="/active")
    parser.add_argument("--expected-mux-source", default="default_input")
    parser.add_argument("--auto-max-speed", type=float, default=0.20)
    parser.add_argument("--auto-cycles", type=int, default=1)
    parser.add_argument("--auto-angular-speed", type=float, default=0.20)
    parser.add_argument("--auto-angular-accel", type=float, default=0.30)
    parser.add_argument("--auto-angular-decel", type=float, default=0.25)
    parser.add_argument("--rotation-tolerance", type=float, default=0.03)
    parser.add_argument("--rotation-timeout", type=float, default=22.0)
    parser.add_argument("--command-rate", type=float, default=50.0)
    parser.add_argument("--max-distance", type=float, default=0.90)
    parser.add_argument("--max-actual-speed", type=float, default=0.28)
    parser.add_argument("--max-actual-yaw-rate", type=float, default=0.15)
    parser.add_argument("--max-rotation-actual-yaw-rate", type=float, default=0.28)
    parser.add_argument("--max-rotation-drift", type=float, default=0.08)
    parser.add_argument("--max-return-error", type=float, default=0.15)
    parser.add_argument("--max-yaw-deviation", type=float, default=0.15)
    parser.add_argument("--max-data-age", type=float, default=0.10)
    parser.add_argument("--stop-publish-seconds", type=float, default=1.0)
    args = parser.parse_args()
    args.auto_max_speed = min(max(args.auto_max_speed, 0.05), HARD_MAX_SPEED)
    args.auto_cycles = min(max(args.auto_cycles, 1), HARD_MAX_AUTO_CYCLES)
    args.max_distance = min(max(args.max_distance, 0.10), HARD_MAX_DISTANCE)
    args.command_rate = min(max(args.command_rate, 20.0), 100.0)
    if args.enable_motion:
        if (
            args.command_topic != "/input/default" or
            args.mux_active_topic != "/active" or
            args.expected_mux_source != "default_input"
        ):
            parser.error("自动模式固定使用cmd_vel_mux最低优先级/input/default，禁止改写")
        args.stationary_seconds = max(args.stationary_seconds, 3.0)
        args.max_actual_speed = min(
            max(args.max_actual_speed, args.auto_max_speed), HARD_MAX_ACTUAL_SPEED
        )
        args.max_actual_yaw_rate = min(
            max(args.max_actual_yaw_rate, 0.02), HARD_MAX_ACTUAL_YAW_RATE
        )
        args.auto_angular_speed = min(max(args.auto_angular_speed, 0.08), 0.20)
        args.auto_angular_accel = min(max(args.auto_angular_accel, 0.05), 0.30)
        args.auto_angular_decel = min(max(args.auto_angular_decel, 0.05), 0.30)
        args.rotation_tolerance = min(max(args.rotation_tolerance, 0.02), 0.05)
        args.rotation_timeout = min(max(args.rotation_timeout, 18.0), 24.0)
        args.max_rotation_actual_yaw_rate = min(
            max(args.max_rotation_actual_yaw_rate, args.auto_angular_speed),
            HARD_MAX_ROTATION_YAW_RATE,
        )
        args.max_rotation_drift = min(
            max(args.max_rotation_drift, 0.03), HARD_MAX_ROTATION_DRIFT
        )
        args.max_return_error = min(max(args.max_return_error, 0.05), 0.20)
        args.max_yaw_deviation = min(
            max(args.max_yaw_deviation, 0.02), HARD_MAX_YAW_DEVIATION
        )
        args.max_data_age = min(max(args.max_data_age, 0.02), 0.10)
        args.stop_publish_seconds = max(args.stop_publish_seconds, 1.0)
        profile = build_auto_profile(args)
        round_distance = 2.0 * profile_distance(profile) * args.auto_cycles
        if round_distance > args.max_distance:
            parser.error(
                f"自动往返理论累计位移{round_distance:.3f}m超过"
                f"max-distance={args.max_distance:.3f}m"
            )
        if not sys.stdin.isatty():
            parser.error("自动运动必须在交互终端运行")
    if args.output_dir and args.output_prefix:
        parser.error("--output-dir与旧版--output-prefix不能同时使用")
    if args.output_dir:
        requested_output_dir = args.output_dir
    elif args.output_prefix:
        # Backward compatibility: the old prefix is now treated as a directory name.
        requested_output_dir = args.output_prefix
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        requested_output_dir = os.path.join(
            os.path.expanduser("~"), f"chassis_imu_accel_calibration_{timestamp}"
        )
    args.output_dir = unique_output_directory(requested_output_dir)
    return args


def main() -> None:
    args = parse_args()
    auto_profile = build_auto_profile(args) if args.enable_motion else []
    total_duration = args.stationary_seconds + args.straight_seconds + args.rotation_seconds
    if args.enable_motion:
        confirmation = input(
            "警告：机器人将直线驶出、原地转180度后驶回。必须清空完整走廊和转弯区、"
            "准备物理急停、停止导航。"
            "确认后输入 MOVE："
        )
        if confirmation != "MOVE":
            raise SystemExit("未确认，退出且不运动")
        print("自动模式已确认；脚本使用cmd_vel_mux最低优先级，任何其他输入均可抢占。")
    else:
        print("只读模式：不发布速度、不修改参数。人工驾驶时请准备急停并确保场地安全。")
    print_phase(0, args)

    rclpy.init()
    recorder = CalibrationRecorder(args)
    current_phase = 0
    interrupted = False
    try:
        if args.enable_motion:
            while rclpy.ok() and recorder.elapsed() < args.stationary_seconds:
                rclpy.spin_once(recorder, timeout_sec=0.05)
            print_phase(1, args)
            run_auto_motion(recorder, args, auto_profile)
        else:
            while rclpy.ok() and recorder.elapsed() < total_duration:
                rclpy.spin_once(recorder, timeout_sec=0.05)
                next_phase = recorder.phase()
                if next_phase != current_phase:
                    current_phase = next_phase
                    print_phase(current_phase, args)
    except KeyboardInterrupt:
        interrupted = True
        print("收到Ctrl+C，立即发送连续零速度并分析已采集数据。")
    except RuntimeError as error:
        interrupted = True
        print(f"安全保护触发：{error}")
    finally:
        if args.enable_motion:
            publish_stop(recorder, args)
        recorder.destroy_node()
        rclpy.shutdown()

    result = analyze(recorder, args)
    print(json.dumps(result, indent=2, allow_nan=True))
    print(f"结果目录：{args.output_dir}")
    print(f"结果摘要：{os.path.join(args.output_dir, 'summary.json')}")
    print(
        f"原始序列：{os.path.join(args.output_dir, 'odom.csv')} / "
        f"{os.path.join(args.output_dir, 'imu.csv')}"
    )
    print(f"参数建议：{os.path.join(args.output_dir, 'recommended_params.yaml')}")
    imu_recommendation = result["recommended_parameters"]["future_imu_fusion"]
    if imu_recommendation["ready"]:
        print("直线动态相关性合格，可依据summary中的偏置、延迟、比例继续设计融合参数。")
    else:
        print(
            "警告：本次结果未达到IMU融合启用条件："
            f"{imu_recommendation['rejection_reason']}。请排除保护触发后重新采集。"
        )
    if interrupted:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
