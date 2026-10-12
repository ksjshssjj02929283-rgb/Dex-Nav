#!/usr/bin/env python3
# Copyright 2026 DEXE Robot Engineer
# SPDX-License-Identifier: Apache-2.0
"""
Record and compare S-curve speed-planning behavior on the robot.

The record command is read-only: it subscribes to odom, controller commands,
the consumed trajectory target, and speed-plan diagnostics.  It never publishes
a velocity or navigation goal.
"""

import argparse
import bisect
import csv
from datetime import datetime
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time


DEFAULT_OUTPUT_ROOT = "~/navigation/scurve_comparison"
DEFAULT_TOPICS = {
    "odom": "/feedback/odom",
    "cmd": "/cmd_vel_nav",
    "smoothed": "/control/cmd_vel_nav",
    "target": "/nav_monitor/controller_target",
    "diagnostics": "/navigation/speed_plan_diagnostics",
}
MOTION_SPEED_THRESHOLD = 0.03
ACCEL_EVENT_THRESHOLD = 0.03


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def parse_value(value):
    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    try:
        number = float(value)
        return number if math.isfinite(number) else None
    except ValueError:
        return value


def json_safe(value):
    """Recursively convert ROS/Python values to strict JSON-compatible values."""
    if isinstance(value, (bytes, bytearray)):
        if len(value) == 1:
            return int(value[0])
        return value.hex()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def write_json_atomic(path, value):
    """Write complete JSON or leave the previous file untouched."""
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
    os.replace(temporary, path)


def local_linear_derivative(times, values, window=0.30):
    """Estimate dy/dt with a centered local regression; gaps remain empty."""
    count = min(len(times), len(values))
    result = [None] * count
    if count < 3:
        return result
    half_window = window / 2.0
    for index in range(count):
        left = bisect.bisect_left(times, times[index] - half_window)
        right = bisect.bisect_right(times, times[index] + half_window)
        points = [
            point for point in range(left, right)
            if finite(values[point]) and abs(times[point] - times[index]) <= half_window
        ]
        if len(points) < 3:
            continue
        # Do not bridge a missing topic interval.
        if any(times[b] - times[b - 1] > 0.5 for b in range(points[0] + 1, points[-1] + 1)):
            continue
        mean_t = sum(times[point] for point in points) / len(points)
        mean_y = sum(values[point] for point in points) / len(points)
        denominator = sum((times[point] - mean_t) ** 2 for point in points)
        if denominator <= 1e-12:
            continue
        result[index] = sum(
            (times[point] - mean_t) * (values[point] - mean_y)
            for point in points) / denominator
    return result


def series(rows, key, time_key="t"):
    points = [
        (float(row[time_key]), float(row[key])) for row in rows
        if finite(row.get(time_key)) and finite(row.get(key))
    ]
    points.sort()
    return [point[0] for point in points], [point[1] for point in points]


def diagnostic_series(data, key):
    return series(data.get("diagnostics", []), key)


def interpolate(times, values, query):
    if not times or query < times[0] or query > times[-1]:
        return None
    right = bisect.bisect_left(times, query)
    if right == 0:
        return values[0]
    if right >= len(times):
        return values[-1]
    left = right - 1
    span = times[right] - times[left]
    if span <= 1e-9 or span > 0.5:
        return None
    ratio = (query - times[left]) / span
    return values[left] + ratio * (values[right] - values[left])


def rms(values):
    clean = [value for value in values if finite(value)]
    return math.sqrt(sum(value * value for value in clean) / len(clean)) if clean else None


def first_true_time(rows, key):
    for row in rows:
        if row.get(key) is True:
            return row.get("t")
    return None


def motion_start_time(data):
    for row in data.get("odom", []):
        if abs(row.get("v", 0.0)) >= MOTION_SPEED_THRESHOLD:
            return row["t"]
    return 0.0


def cumulative_distance(rows, start_time=None, end_time=None):
    selected = [
        row for row in rows
        if (start_time is None or row["t"] >= start_time) and
        (end_time is None or row["t"] <= end_time)
    ]
    distance = 0.0
    for previous, current in zip(selected, selected[1:]):
        distance += math.hypot(current["x"] - previous["x"], current["y"] - previous["y"])
    return distance


def stable_stop_time(odom, start_time):
    if start_time is None:
        return None
    for index, row in enumerate(odom):
        if row["t"] < start_time or abs(row["v"]) >= MOTION_SPEED_THRESHOLD:
            continue
        end_time = row["t"] + 0.5
        following = [sample for sample in odom[index:] if sample["t"] <= end_time]
        if following and following[-1]["t"] >= end_time - 0.05 and all(
                abs(sample["v"]) < MOTION_SPEED_THRESHOLD for sample in following):
            return row["t"]
    return None


def count_sign_changes(values, threshold=ACCEL_EVENT_THRESHOLD):
    last_sign = 0
    count = 0
    for value in values:
        if not finite(value) or abs(value) < threshold:
            continue
        sign = 1 if value > 0.0 else -1
        if last_sign and sign != last_sign:
            count += 1
        last_sign = sign
    return count


def count_reaccelerations(values, threshold=ACCEL_EVENT_THRESHOLD):
    saw_negative = False
    count = 0
    for value in values:
        if not finite(value):
            continue
        if value < -threshold:
            saw_negative = True
        elif value > threshold and saw_negative:
            count += 1
            saw_negative = False
    return count


def aligned_pairs(data, first_key, second_key):
    rows = data.get("diagnostics", [])
    return [
        (row[first_key], row[second_key]) for row in rows
        if finite(row.get(first_key)) and finite(row.get(second_key))
    ]


def analyze_run(data):
    odom = data.get("odom", [])
    target = data.get("target", [])
    diagnostics = data.get("diagnostics", [])
    target_t, target_a = series(target, "a")
    target_jerk = local_linear_derivative(target_t, target_a)
    odom_t, odom_v = series(odom, "v")
    target_v_t, target_v = series(target, "v")
    tracking_errors = []
    for timestamp, velocity in zip(target_v_t, target_v):
        measured = interpolate(odom_t, odom_v, timestamp)
        if measured is not None:
            tracking_errors.append(velocity - measured)

    terminal_time = first_true_time(diagnostics, "terminal_brake_latched")
    stop_time = stable_stop_time(odom, terminal_time)
    preterminal_target_a = [
        row["a"] for row in target
        if terminal_time is None or row["t"] < terminal_time
    ]
    state_acceleration_key = (
        "initial_motion_state_acceleration"
        if any(finite(row.get("initial_motion_state_acceleration")) for row in diagnostics)
        else "initial_motion_inertial_limited_a")
    state_valid_key = (
        "initial_motion_state_valid"
        if any("initial_motion_state_valid" in row for row in diagnostics)
        else "initial_motion_inertial_valid")
    planned_measured = aligned_pairs(
        data, "initial_motion_planned_a", state_acceleration_key)
    boundary_measured = aligned_pairs(
        data, "boundary_acceleration", state_acceleration_key)
    boundary_planned = aligned_pairs(
        data, "boundary_acceleration", "initial_motion_planned_a")
    state_valid_rows = [row for row in diagnostics if row.get(state_valid_key)]
    source_counts = {}
    stitching_result_counts = {}
    for row in diagnostics:
        source = row.get("initial_motion_acceleration_source", "missing")
        source_counts[source] = source_counts.get(source, 0) + 1
        stitching_result = row.get("stitching_result", "missing")
        stitching_result_counts[stitching_result] = (
            stitching_result_counts.get(stitching_result, 0) + 1)
    stitched_cycles = sum(
        1 for row in diagnostics if row.get("trajectory_stitched") is True)

    return {
        "label": data.get("metadata", {}).get("label", "unknown"),
        "duration_s": data.get("metadata", {}).get("duration_s", 0.0),
        "odom_samples": len(odom),
        "diagnostic_cycles": len(diagnostics),
        "motion_start_s": motion_start_time(data),
        "distance_m": cumulative_distance(odom),
        "max_odom_speed_mps": max((abs(row["v"]) for row in odom), default=0.0),
        "max_target_speed_mps": max((abs(row["v"]) for row in target), default=0.0),
        "target_accel_sign_changes_before_terminal": count_sign_changes(preterminal_target_a),
        "target_reaccelerations_before_terminal": count_reaccelerations(preterminal_target_a),
        "target_max_abs_jerk_mps3": max(
            (abs(value) for value in target_jerk if finite(value)), default=0.0),
        "target_rms_jerk_mps3": rms(target_jerk),
        "target_odom_velocity_rmse_mps": rms(tracking_errors),
        "planned_vs_fused_accel_rmse_mps2": rms(
            [first - second for first, second in planned_measured]),
        "boundary_vs_fused_accel_rmse_mps2": rms(
            [first - second for first, second in boundary_measured]),
        "boundary_vs_planned_accel_rmse_mps2": rms(
            [first - second for first, second in boundary_planned]),
        "state_valid_fraction": (
            len(state_valid_rows) / len(diagnostics) if diagnostics else 0.0),
        "state_rejected_samples": max(
            (row.get("initial_motion_state_rejected_samples", 0) for row in diagnostics
             if finite(row.get("initial_motion_state_rejected_samples"))), default=0),
        "state_acceleration_clamps": max(
            (row.get("initial_motion_state_acceleration_clamps", 0) for row in diagnostics
             if finite(row.get("initial_motion_state_acceleration_clamps"))), default=0),
        "terminal_latch_s": terminal_time,
        "terminal_stop_delay_s": (
            stop_time - terminal_time
            if stop_time is not None and terminal_time is not None else None),
        "terminal_post_latch_distance_m": (
            cumulative_distance(odom, terminal_time, stop_time)
            if terminal_time is not None and stop_time is not None else None),
        "acceleration_source_counts": source_counts,
        "trajectory_stitched_fraction": (
            stitched_cycles / len(diagnostics) if diagnostics else 0.0),
        "stitching_result_counts": stitching_result_counts,
        "terminal_stall_recovery_triggers": sum(
            1 for row in diagnostics
            if row.get("terminal_stall_recovery_triggered") is True),
        "terminal_stall_recovery_count": max(
            (row.get("terminal_stall_recovery_count", 0) for row in diagnostics
             if finite(row.get("terminal_stall_recovery_count"))), default=0),
    }


def write_csv(path, rows):
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def load_run(path):
    candidate = Path(path).expanduser()
    if candidate.is_dir():
        candidate = candidate / "run.json"
    with candidate.open(encoding="utf-8") as stream:
        return json.load(stream), candidate.parent


def check_speed_planner_linkage():
    """Return unresolved runtime symbols before S-curve execution can crash Nav2."""
    try:
        prefix_result = subprocess.run(
            ["ros2", "pkg", "prefix", "nav2_speed_planner"],
            capture_output=True, text=True, timeout=10, check=False)
        if prefix_result.returncode != 0:
            return ["无法定位 nav2_speed_planner 安装目录"]
        library = Path(prefix_result.stdout.strip()) / "lib" / "libnav2_speed_planner.so"
        if not library.is_file():
            return [f"找不到动态库：{library}"]
        linkage = subprocess.run(
            ["ldd", "-r", str(library)], capture_output=True, text=True,
            timeout=20, check=False)
        output = linkage.stdout + linkage.stderr
        problems = [
            line.strip() for line in output.splitlines()
            if "undefined symbol" in line or "not found" in line
        ]
        return problems
    except (OSError, subprocess.SubprocessError) as error:
        return [f"动态链接检查执行失败：{error}"]


def import_plotting():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib import font_manager
        chinese_font = "DejaVu Sans"
        for candidate in (
                "Noto Sans CJK SC", "Noto Sans CJK JP",
                "WenQuanYi Micro Hei", "AR PL UMing CN"):
            try:
                font_manager.findfont(candidate, fallback_to_default=False)
                chinese_font = candidate
                break
            except ValueError:
                continue
        matplotlib.rcParams["font.sans-serif"] = [chinese_font, "DejaVu Sans"]
        matplotlib.rcParams["axes.unicode_minus"] = False
        return plt
    except ImportError as error:
        raise RuntimeError(
            "缺少 matplotlib：记录文件已经保存，可在安装 python3-matplotlib 的电脑上运行 compare") from error


def relative_series(rows, key, origin):
    timestamps, values = series(rows, key)
    return [timestamp - origin for timestamp in timestamps], values


def display_label(label):
    """Translate conventional A/B labels while preserving custom labels."""
    return {"before": "修改前", "after": "修改后"}.get(label.lower(), label)


def add_state_spans(axes, terminal_time, origin):
    if terminal_time is None:
        return
    aligned = terminal_time - origin
    for axis in axes:
        axis.axvline(
            aligned, color="#c62828", linestyle=":", linewidth=1.2,
            label="终点制动已锁存")


def add_route_markers(axes, data, origin):
    """Mark each automatically dispatched road-network destination."""
    for event in data.get("route_events", []):
        if event.get("event") != "dispatch" or not finite(event.get("t")):
            continue
        timestamp = event["t"] - origin
        for axis in axes:
            axis.axvline(
                timestamp, color="#616161", linestyle="--",
                linewidth=0.8, alpha=0.6)
        axes[0].text(
            timestamp, 0.98, f"前往 {event.get('point', '?')}", rotation=90,
            va="top", ha="right", fontsize=8,
            transform=axes[0].get_xaxis_transform())


def add_legend_if_present(axis):
    """Avoid Matplotlib's empty-legend warning on short or failed runs."""
    handles, labels = axis.get_legend_handles_labels()
    if handles and labels:
        axis.legend(loc="best", fontsize=8)


def plot_run(data, output):
    plt = import_plotting()
    origin = motion_start_time(data)
    fig, axes = plt.subplots(5, 1, figsize=(13, 17), sharex=True)
    ax_v, ax_a, ax_j, ax_error, ax_state = axes
    curves = [
        ("target", "v", "控制器消费目标", "#6a1b9a", 1.5),
        ("cmd", "v", "控制器输出", "#1565c0", 1.0),
        ("smoothed", "v", "平滑后输出", "#ef6c00", 1.2),
        ("odom", "v", "odom 实测", "#2e7d32", 1.3),
    ]
    for block, key, label, color, width in curves:
        timestamps, values = relative_series(data.get(block, []), key, origin)
        if timestamps:
            ax_v.plot(timestamps, values, label=label, color=color, linewidth=width)

    target_t, target_a = relative_series(data.get("target", []), "a", origin)
    if target_t:
        ax_a.plot(target_t, target_a, label="控制器消费目标 a",
                  color="#6a1b9a", linewidth=1.4)
        jerk = local_linear_derivative(target_t, target_a)
        ax_j.plot(target_t, jerk, label="控制器消费目标 jerk",
                  color="#6a1b9a", linewidth=1.2)

    diag_curves = [
        ("initial_motion_planned_a", "上周期规划 a", "#3949ab", "--"),
        ("initial_motion_state_acceleration_raw", "UKF原始 a", "#26a69a", ":"),
        ("initial_motion_state_acceleration", "UKF限幅 a", "#00897b", "--"),
        ("initial_motion_state_correction", "计划修正项", "#00897b", "-"),
        ("boundary_acceleration", "速度规划器输入 a0", "#e53935", "-"),
    ]
    for key, label, color, style in diag_curves:
        timestamps, values = diagnostic_series(data, key)
        timestamps = [timestamp - origin for timestamp in timestamps]
        if timestamps:
            ax_a.plot(timestamps, values, label=label, color=color, linestyle=style, linewidth=1.1)

    planned_t, planned = diagnostic_series(data, "initial_motion_planned_a")
    fused_t, fused = diagnostic_series(data, "initial_motion_state_acceleration")
    if not fused_t:
        fused_t, fused = diagnostic_series(data, "initial_motion_inertial_limited_a")
    boundary_t, boundary = diagnostic_series(data, "boundary_acceleration")
    if planned_t and fused_t:
        error_t, plan_error, boundary_error = [], [], []
        for timestamp, value in zip(fused_t, fused):
            plan_value = interpolate(planned_t, planned, timestamp)
            boundary_value = interpolate(boundary_t, boundary, timestamp)
            if plan_value is not None:
                error_t.append(timestamp - origin)
                plan_error.append(plan_value - value)
                boundary_error.append(
                    boundary_value - value if boundary_value is not None else None)
        ax_error.plot(error_t, plan_error, label="规划 a - 实测 a",
                      color="#3949ab", linestyle="--")
        ax_error.plot(error_t, boundary_error, label="输入 a0 - 实测 a", color="#e53935")

    diagnostics = data.get("diagnostics", [])
    state_t = [row["t"] - origin for row in diagnostics]
    estimator_valid = [
        1.0 if row.get(
            "initial_motion_state_valid", row.get("initial_motion_inertial_valid")) else 0.0
        for row in diagnostics]
    terminal = [1.0 if row.get("terminal_brake_latched") else 0.0 for row in diagnostics]
    ax_state.step(
        state_t, estimator_valid, where="post",
        label="纵向状态估计有效", color="#1565c0")
    ax_state.step(state_t, terminal, where="post", label="终点制动已锁存", color="#c62828")

    terminal_time = first_true_time(diagnostics, "terminal_brake_latched")
    add_state_spans(axes[:-1], terminal_time, origin)
    add_route_markers(axes, data, origin)
    titles = [
        "速度链路", "加速度：规划、实测和最终 a0",
        "控制器实际消费的 jerk", "初始加速度误差", "状态标志",
    ]
    ylabels = ["m/s", "m/s²", "m/s³", "m/s²", "0/1"]
    for axis, title, ylabel in zip(axes, titles, ylabels):
        axis.set_title(title)
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.25)
        add_legend_if_present(axis)
    axes[-1].set_xlabel("从开始运动起的时间 / s")
    run_label = display_label(data.get("metadata", {}).get("label", "未知"))
    fig.suptitle(f"S 型曲线测试：{run_label}", y=0.995)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.98))
    fig.savefig(output, dpi=160)
    plt.close(fig)


def plot_comparison(before, after, output):
    plt = import_plotting()
    fig, axes = plt.subplots(5, 1, figsize=(13, 17), sharex=True)
    colors = ["#1565c0", "#ef6c00"]
    styles = ["--", "-"]
    for data, color, style in zip((before, after), colors, styles):
        label = display_label(data.get("metadata", {}).get("label", "测试"))
        origin = motion_start_time(data)
        odom_t, odom_v = relative_series(data.get("odom", []), "v", origin)
        target_t, target_v = relative_series(data.get("target", []), "v", origin)
        accel_t, target_a = relative_series(data.get("target", []), "a", origin)
        jerk = local_linear_derivative(accel_t, target_a)
        boundary_t, boundary_a = diagnostic_series(data, "boundary_acceleration")
        fused_t, fused_a = diagnostic_series(data, "initial_motion_state_acceleration")
        if not fused_t:
            fused_t, fused_a = diagnostic_series(data, "initial_motion_inertial_limited_a")
        if not fused_t:
            fused_t, fused_a = diagnostic_series(data, "initial_motion_fused_a")
        axes[0].plot(odom_t, odom_v, color=color, linestyle=style, linewidth=1.4,
                     label=f"{label} odom 实测")
        axes[0].plot(target_t, target_v, color=color, linestyle=":", linewidth=1.0,
                     label=f"{label}目标速度")
        axes[1].plot(accel_t, target_a, color=color, linestyle=style, linewidth=1.3,
                     label=label)
        axes[2].plot(accel_t, jerk, color=color, linestyle=style, linewidth=1.1,
                     label=label)
        axes[3].plot([value - origin for value in boundary_t], boundary_a,
                     color=color, linestyle=style, linewidth=1.2, label=f"{label} a0")
        axes[3].plot([value - origin for value in fused_t], fused_a,
                     color=color, linestyle=":", linewidth=1.0,
                     label=f"{label}融合/EKF a")
        error_t, errors = [], []
        for timestamp, velocity in zip(target_t, target_v):
            measured = interpolate(odom_t, odom_v, timestamp)
            if measured is not None:
                error_t.append(timestamp)
                errors.append(velocity - measured)
        axes[4].plot(error_t, errors, color=color, linestyle=style, linewidth=1.1,
                     label=label)

    titles = [
        "目标速度与 odom 实测速度", "控制器实际消费的加速度",
        "控制器实际消费的 jerk", "速度规划器输入 a0 与融合/EKF a",
        "目标速度 - odom 速度",
    ]
    ylabels = ["m/s", "m/s²", "m/s³", "m/s²", "m/s"]
    for axis, title, ylabel in zip(axes, titles, ylabels):
        axis.set_title(title)
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.25)
        add_legend_if_present(axis)
    axes[-1].set_xlabel("各次测试从开始运动起的时间 / s")
    fig.suptitle("S 型曲线修改前后对比", y=0.995)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.98))
    fig.savefig(output, dpi=170)
    plt.close(fig)


def save_run(data, output_dir):
    output_dir.mkdir(parents=True, exist_ok=True)
    data["summary"] = analyze_run(data)
    safe_data = json_safe(data)
    write_json_atomic(output_dir / "run.json", safe_data)
    write_json_atomic(output_dir / "summary.json", safe_data["summary"])
    for block in ("odom", "cmd", "smoothed", "target", "diagnostics", "route_events"):
        write_csv(output_dir / f"{block}.csv", safe_data.get(block, []))
    try:
        plot_run(safe_data, output_dir / "overview.png")
    except RuntimeError as error:
        print(f"提示：{error}")


def record_command(args, ros_args):
    automatic_route = args.command == "route"
    if automatic_route:
        if not args.points:
            raise SystemExit("route 至少需要一个目标点。")
        if args.dwell < 0.0 or args.final_settle < 0.0:
            raise SystemExit("--dwell 和 --final-settle 不能为负数。")
        if (args.segment_timeout <= 0.0 or args.server_timeout <= 0.0 or
                args.start_tolerance <= 0.0):
            raise SystemExit("超时参数和 --start-tolerance 必须大于0。")
    try:
        import rclpy
        from action_msgs.msg import GoalStatus
        from diagnostic_msgs.msg import DiagnosticArray
        from geometry_msgs.msg import Twist, Vector3Stamped
        from lifecycle_msgs.msg import State
        from lifecycle_msgs.srv import GetState
        from nav_msgs.msg import Odometry
        from nav_interfaces.action import NavigateToPoint
        from nav_interfaces.srv import GetAssets
        from nav_interfaces.msg import NavigationState
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from rclpy.action import ActionClient
        from rclpy.duration import Duration
        from rclpy.node import Node
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformListener
    except ImportError as error:
        raise SystemExit(f"record/route 需要 ROS 2 Python 环境：{error}") from error

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    safe_label = "".join(character if character.isalnum() or character in "-_" else "_"
                         for character in args.label)
    output_dir = Path(args.output_root).expanduser() / f"{timestamp}_{safe_label}"

    class Recorder(Node):
        def __init__(self):
            super().__init__("scurve_before_after_recorder")
            self.start_ns = self.get_clock().now().nanoseconds
            self.active_goal_handle = None
            self.route_index = 0
            self.route_generation = 0
            self.route_timeout_timer = None
            self.route_timers = []
            self.route_finished = False
            self.recording_done = False
            self.data = {
                "metadata": {
                    "schema_version": 1,
                    "label": args.label,
                    "started_at": datetime.now().isoformat(),
                    "topics": {
                        "odom": args.odom_topic,
                        "cmd": args.cmd_topic,
                        "smoothed": args.smoothed_topic,
                        "target": args.target_topic,
                        "diagnostics": args.diagnostics_topic,
                    },
                    "read_only": not automatic_route,
                },
                "odom": [], "cmd": [], "smoothed": [], "target": [],
                "diagnostics": [], "route_events": [],
            }
            self.create_subscription(Odometry, args.odom_topic, self.odom_callback, 100)
            self.create_subscription(Twist, args.cmd_topic,
                                     lambda msg: self.twist_callback("cmd", msg), 50)
            self.create_subscription(Twist, args.smoothed_topic,
                                     lambda msg: self.twist_callback("smoothed", msg), 100)
            self.create_subscription(Vector3Stamped, args.target_topic,
                                     self.target_callback, 50)
            self.create_subscription(DiagnosticArray, args.diagnostics_topic,
                                     self.diagnostics_callback, 50)
            if not automatic_route and args.duration > 0.0:
                self.create_timer(args.duration, self.finish)
            if automatic_route:
                self.route_points = list(args.points)
                self.data["metadata"]["automatic_route"] = {
                    "start_point": args.start_point,
                    "destinations": self.route_points,
                    "dwell_s": args.dwell,
                    "segment_timeout_s": args.segment_timeout,
                    "final_settle_s": args.final_settle,
                    "start_tolerance_m": args.start_tolerance,
                }
                self.nav_point_client = ActionClient(
                    self, NavigateToPoint, args.navigate_action)
                self.get_point_client = self.create_client(
                    GetAssets, args.get_assets_service)
                self.nav_state = None
                self.create_subscription(
                    NavigationState, args.nav_state_topic, lambda m: setattr(self, "nav_state", m),
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
                self.lifecycle_clients = {
                    name: self.create_client(GetState, f"/{name}/get_state")
                    for name in args.required_lifecycle_nodes
                }
                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self)

        def elapsed(self):
            return (self.get_clock().now().nanoseconds - self.start_ns) * 1e-9

        def odom_callback(self, msg):
            q = msg.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.data["odom"].append({
                "t": self.elapsed(), "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y, "yaw": yaw,
                "v": msg.twist.twist.linear.x,
                "w": msg.twist.twist.angular.z,
            })

        def twist_callback(self, block, msg):
            self.data[block].append({
                "t": self.elapsed(), "v": msg.linear.x, "w": msg.angular.z,
            })

        def target_callback(self, msg):
            self.data["target"].append({
                "t": self.elapsed(), "v": msg.vector.x, "a": msg.vector.y,
                "preview_t": msg.vector.z,
            })

        def diagnostics_callback(self, msg):
            for status in msg.status:
                if status.name != "navigation_speed_plan":
                    continue
                level = json_safe(status.level)
                row = {"t": self.elapsed(), "level": level,
                       "message": status.message}
                row.update({item.key: parse_value(item.value) for item in status.values})
                self.data["diagnostics"].append(row)

        def route_event(self, event, point="", detail=""):
            row = {"t": self.elapsed(), "event": event}
            if point:
                row["point"] = point
            if detail:
                row["detail"] = detail
            self.data["route_events"].append(row)

        def reset_capture(self):
            """Discard preflight traffic so t=0 is immediately before L1 -> L2."""
            for block in ("odom", "cmd", "smoothed", "target", "diagnostics", "route_events"):
                self.data[block].clear()
            self.start_ns = self.get_clock().now().nanoseconds
            self.data["metadata"]["started_at"] = datetime.now().isoformat()

        def schedule_once(self, delay, callback):
            timer_holder = {}

            def invoke():
                timer = timer_holder["timer"]
                timer.cancel()
                callback()

            timer = self.create_timer(max(delay, 0.01), invoke)
            timer_holder["timer"] = timer
            self.route_timers.append(timer)

        def start_route(self):
            self.route_event("route_start", args.start_point)
            self.send_current_route_goal()

        def send_current_route_goal(self):
            if self.route_index >= len(self.route_points):
                self.route_event("route_complete", args.start_point)
                self.data["metadata"]["route_result"] = "succeeded"
                self.route_finished = True
                print(f"已完成全部路段，等待 {args.final_settle:.1f}s 记录停车状态。")
                self.schedule_once(args.final_settle, self.finish)
                return

            point = self.route_points[self.route_index]
            self.route_generation += 1
            generation = self.route_generation
            goal = NavigateToPoint.Goal()
            goal.point_name = point
            goal.use_pose = False
            self.route_event("dispatch", point)
            print(f"[{self.route_index + 1}/{len(self.route_points)}] 发送目标：{point}")
            future = self.nav_point_client.send_goal_async(
                goal, feedback_callback=self.route_feedback_callback)
            future.add_done_callback(
                lambda completed, token=generation, destination=point:
                self.goal_response_callback(completed, token, destination))

        def route_feedback_callback(self, feedback):
            if feedback and feedback.feedback:
                status = feedback.feedback.current_status
                if status and (not self.data["route_events"] or
                               self.data["route_events"][-1].get("detail") != status):
                    self.route_event(
                        "feedback", self.route_points[self.route_index], status)

        def goal_response_callback(self, future, generation, point):
            if generation != self.route_generation or self.route_finished:
                return
            try:
                goal_handle = future.result()
            except Exception as error:  # ROS future transports the concrete exception.
                self.fail_route(point, f"发送目标异常：{error}")
                return
            if not goal_handle.accepted:
                self.fail_route(point, "目标被拒绝")
                return
            self.active_goal_handle = goal_handle
            self.route_event("accepted", point)
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda completed, token=generation, destination=point:
                self.goal_result_callback(completed, token, destination))
            self.route_timeout_timer = self.create_timer(
                args.segment_timeout,
                lambda token=generation, destination=point:
                self.segment_timeout_callback(token, destination))

        def goal_result_callback(self, future, generation, point):
            if generation != self.route_generation or self.route_finished:
                return
            if self.route_timeout_timer is not None:
                self.route_timeout_timer.cancel()
                self.route_timeout_timer = None
            self.active_goal_handle = None
            try:
                wrapped = future.result()
                result = wrapped.result
                succeeded = (
                    wrapped.status == GoalStatus.STATUS_SUCCEEDED and
                    result is not None and result.success)
                detail = result.message if result is not None else "无结果"
            except Exception as error:  # ROS future transports the concrete exception.
                self.fail_route(point, f"读取导航结果异常：{error}")
                return
            if not succeeded:
                self.fail_route(point, detail)
                return
            self.route_event("arrived", point, detail)
            print(f"已到达 {point}")
            self.route_index += 1
            self.schedule_once(args.dwell, self.send_current_route_goal)

        def segment_timeout_callback(self, generation, point):
            if generation != self.route_generation or self.route_finished:
                return
            if self.route_timeout_timer is not None:
                self.route_timeout_timer.cancel()
                self.route_timeout_timer = None
            self.route_event("timeout", point)
            if self.active_goal_handle is not None:
                self.active_goal_handle.cancel_goal_async()
            self.fail_route(point, f"单段超过 {args.segment_timeout:.1f}s")

        def fail_route(self, point, detail):
            if self.route_finished:
                return
            self.route_finished = True
            self.route_event("route_failed", point, detail)
            self.data["metadata"]["route_result"] = "failed"
            self.data["metadata"]["route_failure"] = {"point": point, "detail": detail}
            print(f"安全停止后续任务：导航到 {point} 失败：{detail}")
            self.schedule_once(1.0, self.finish)

        def cancel_active_route(self, reason="operator_interrupt"):
            if not automatic_route or self.route_finished:
                return None
            self.route_finished = True
            point = self.route_points[self.route_index] if self.route_index < len(
                self.route_points) else args.start_point
            self.route_event("route_canceled", point, reason)
            self.data["metadata"]["route_result"] = "canceled"
            if self.route_timeout_timer is not None:
                self.route_timeout_timer.cancel()
            if self.active_goal_handle is not None:
                return self.active_goal_handle.cancel_goal_async()
            return None

        def finish(self):
            # Do not call rclpy.shutdown() from inside an executor callback. On Humble that can
            # leave the outer rclpy.spin() blocked even though the route has completed.
            self.recording_done = True

    rclpy.init(args=ros_args)
    recorder = Recorder()
    print(f"输出目录：{output_dir}")
    preflight_ok = True
    if automatic_route:
        print(
            f"自动路网测试：{args.start_point} -> " + " -> ".join(recorder.route_points))
        preflight_ok = route_preflight(
            recorder, args, rclpy, GetAssets, None, GetState, State,
            Time, Duration)
        if preflight_ok:
            recorder.reset_capture()
            recorder.start_route()
    else:
        print(f"开始只读记录：{args.label}")
        print("现在发送导航目标；任务结束且机器人稳定停止后按 Ctrl+C。")
    runtime_error = None
    try:
        if preflight_ok:
            while rclpy.ok() and not recorder.recording_done:
                rclpy.spin_once(recorder, timeout_sec=0.1)
    except KeyboardInterrupt:
        cancel_future = recorder.cancel_active_route()
        if cancel_future is not None and rclpy.ok():
            rclpy.spin_until_future_complete(recorder, cancel_future, timeout_sec=2.0)
    except Exception as error:
        runtime_error = error
        print(f"脚本运行异常，正在取消当前导航：{error}")
        cancel_future = recorder.cancel_active_route(f"script_exception: {error}")
        if cancel_future is not None and rclpy.ok():
            rclpy.spin_until_future_complete(recorder, cancel_future, timeout_sec=2.0)
        recorder.data["metadata"]["script_exception"] = str(error)
    finally:
        if automatic_route and not preflight_ok:
            recorder.data["metadata"]["route_result"] = "preflight_failed"
        recorder.data["metadata"]["ended_at"] = datetime.now().isoformat()
        recorder.data["metadata"]["duration_s"] = recorder.elapsed()
        save_run(recorder.data, output_dir)
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f"记录完成：{output_dir}")
    print(f"单次图：{output_dir / 'overview.png'}")
    counts = {
        block: len(recorder.data[block])
        for block in ("odom", "cmd", "smoothed", "target", "diagnostics")
    }
    print("样本数：" + ", ".join(f"{key}={value}" for key, value in counts.items()))
    for required in ("odom", "target", "diagnostics"):
        if counts[required] == 0:
            print(f"警告：{required} 没有收到数据，请检查对应话题名和Nav2参数。")
    if runtime_error is not None:
        raise SystemExit(2)


def route_preflight(
        node, args, rclpy, get_nav_point_type, get_status_type,
        get_state_type, lifecycle_state_type, time_type, duration_type):
    """Validate points, action availability, stationary state, and L1 proximity."""
    print("执行安全预检：检查点位、Action、当前速度和 L1 起点位置。")
    linkage_problems = check_speed_planner_linkage()
    if linkage_problems:
        print("预检失败：nav2_speed_planner 存在未解析的运行时符号：")
        for problem in linkage_problems[:10]:
            print(f"  {problem}")
        return False
    print("  nav2_speed_planner 动态链接检查：通过")
    if not node.get_point_client.wait_for_service(timeout_sec=args.server_timeout):
        print(f"预检失败：服务不可用：{args.get_assets_service}")
        return False
    if not node.nav_point_client.wait_for_server(timeout_sec=args.server_timeout):
        print(f"预检失败：Action 不可用：{args.navigate_action}")
        return False

    for name, client in node.lifecycle_clients.items():
        if not client.wait_for_service(timeout_sec=args.server_timeout):
            print(f"预检失败：Nav2 节点不存在或生命周期服务不可用：/{name}")
            return False
        future = client.call_async(get_state_type.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.server_timeout)
        if not future.done() or future.result() is None:
            print(f"预检失败：无法查询 Nav2 生命周期状态：/{name}")
            return False
        state = future.result().current_state
        if state.id != lifecycle_state_type.PRIMARY_STATE_ACTIVE:
            print(f"预检失败：/{name} 当前为 {state.label}，不是 active。")
            return False
        print(f"  Nav2 节点 /{name}: active")

    deadline = node.get_clock().now() + duration_type(seconds=3.0)
    while node.nav_state is None and node.get_clock().now() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.nav_state is None:
        print(f"预检失败：未收到导航状态：{args.nav_state_topic}")
        return False
    current_status = int(node.nav_state.state)
    if current_status in (1, 2, 3, 4):   # PENDING / RUNNING / PAUSED / BLOCKED
        print("预检失败：当前已有导航任务正在执行，不发送新的路网目标。")
        return False
    print(f"  当前导航状态：{current_status}（0 IDLE / 5 SUCCEEDED / 6 FAILED / 7 CANCELLED）")

    # 一次 get_assets 取当前地图全部点位，再按名字查
    request = get_nav_point_type.Request()
    request.kinds = get_nav_point_type.Request.KIND_POINTS
    future = node.get_point_client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=args.server_timeout)
    if not future.done() or future.result() is None or not future.result().success:
        print(f"预检失败：无法读取点位：{args.get_assets_service}")
        return False
    by_name = {p.name: p for p in future.result().points}
    point_data = {}
    for point_name in dict.fromkeys([args.start_point] + list(args.points)):
        if point_name not in by_name:
            print(f"预检失败：路网点不存在：{point_name}")
            return False
        point_data[point_name] = by_name[point_name]
        point = point_data[point_name]
        print(f"  点位 {point_name}: x={point.x:.3f}, y={point.y:.3f}, theta={point.theta:.3f}")

    deadline = time.monotonic() + args.server_timeout
    while time.monotonic() < deadline and not node.data["odom"]:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.data["odom"]:
        print(f"预检失败：未收到 odom：{args.odom_topic}")
        return False
    if abs(node.data["odom"][-1]["v"]) >= MOTION_SPEED_THRESHOLD:
        print(f"预检失败：机器人仍在运动，v={node.data['odom'][-1]['v']:.3f}m/s")
        return False

    if args.skip_start_check:
        print("已按参数跳过 L1 起点距离检查。")
        return True

    transform = None
    deadline = time.monotonic() + args.server_timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            transform = node.tf_buffer.lookup_transform(
                "map", args.base_frame, time_type(),
                timeout=duration_type(seconds=0.05))
            break
        except Exception:
            continue
    if transform is None:
        print(
            f"预检失败：无法查询 map->{args.base_frame}。确认定位正常；"
            "只有明确确认机器人位于L1时才可使用 --skip-start-check。")
        return False
    start = point_data[args.start_point]
    distance = math.hypot(
        transform.transform.translation.x - start.x,
        transform.transform.translation.y - start.y)
    if distance > args.start_tolerance:
        print(
            f"预检失败：机器人距离 {args.start_point} 为 {distance:.3f}m，"
            f"超过 {args.start_tolerance:.3f}m。")
        return False
    print(f"预检通过：机器人距离 {args.start_point} 为 {distance:.3f}m。")
    return True


def compare_command(args):
    before, before_dir = load_run(args.before)
    after, after_dir = load_run(args.after)
    output = Path(args.output).expanduser() if args.output else after_dir / "comparison.png"
    plot_comparison(before, after, output)
    before_summary = analyze_run(before)
    after_summary = analyze_run(after)
    comparison = {"before": before_summary, "after": after_summary, "delta_after_minus_before": {}}
    for key in before_summary.keys() & after_summary.keys():
        first, second = before_summary[key], after_summary[key]
        if finite(first) and finite(second):
            comparison["delta_after_minus_before"][key] = second - first
    summary_path = output.with_name(output.stem + "_summary.json")
    with summary_path.open("w", encoding="utf-8") as stream:
        json.dump(comparison, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(f"修改前：{before_dir}")
    print(f"修改后：{after_dir}")
    print(f"对比图：{output}")
    print(f"指标对比：{summary_path}")
    print("关键指标（修改前 -> 修改后）：")
    units = {
        "target_rms_jerk_mps3": "m/s³",
        "target_odom_velocity_rmse_mps": "m/s",
        "boundary_vs_fused_accel_rmse_mps2": "m/s²",
        "boundary_vs_planned_accel_rmse_mps2": "m/s²",
        "target_accel_sign_changes_before_terminal": "次",
        "target_reaccelerations_before_terminal": "次",
        "terminal_stop_delay_s": "s",
        "terminal_post_latch_distance_m": "m",
    }
    for key, unit in units.items():
        first = before_summary.get(key)
        second = after_summary.get(key)
        first_text = f"{first:.4f}" if finite(first) else "无数据"
        second_text = f"{second:.4f}" if finite(second) else "无数据"
        print(f"  {key}: {first_text} -> {second_text} {unit}")


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    subparsers = parser.add_subparsers(dest="command", required=True)

    def add_recording_topics(subparser):
        subparser.add_argument("label", help="测试标签，例如 修改前 / 修改后")
        subparser.add_argument("--output-root", default=DEFAULT_OUTPUT_ROOT)
        subparser.add_argument("--odom-topic", default=DEFAULT_TOPICS["odom"])
        subparser.add_argument("--cmd-topic", default=DEFAULT_TOPICS["cmd"])
        subparser.add_argument("--smoothed-topic", default=DEFAULT_TOPICS["smoothed"])
        subparser.add_argument("--target-topic", default=DEFAULT_TOPICS["target"])
        subparser.add_argument("--diagnostics-topic", default=DEFAULT_TOPICS["diagnostics"])

    record = subparsers.add_parser("record", help="只读记录一次导航测试")
    add_recording_topics(record)
    record.add_argument("--duration", type=float, default=0.0,
                        help="自动停止秒数；0表示按Ctrl+C结束")
    route = subparsers.add_parser(
        "route", help="自动执行 L1->L2->L3->L1 并记录")
    add_recording_topics(route)
    route.add_argument("--start-point", default="L1", help="安全起点检查名称")
    route.add_argument(
        "--points", nargs="+", default=["L2", "L3", "L1"],
        help="依次导航的点位，默认 L2 L3 L1")
    route.add_argument("--dwell", type=float, default=0.2,
                       help="每段成功后发送下一目标前等待秒数")
    route.add_argument("--final-settle", type=float, default=2.0,
                       help="回到L1后继续记录停车状态的秒数")
    route.add_argument("--segment-timeout", type=float, default=180.0,
                       help="任一段导航超时秒数，超时后取消并停止")
    route.add_argument("--server-timeout", type=float, default=10.0,
                       help="等待点位服务、Action和TF的秒数")
    route.add_argument("--start-tolerance", type=float, default=0.35,
                       help="机器人距离L1不超过该值才允许自动开始，单位m")
    route.add_argument("--base-frame", default="base_link")
    route.add_argument("--navigate-action", default="/navigation/navigate_to_point")
    route.add_argument("--get-assets-service", default="/map_asset_manager/get_assets")
    route.add_argument("--nav-state-topic", default="/navigation/state")
    route.add_argument(
        "--required-lifecycle-nodes", nargs="+",
        default=[
            "controller_server", "planner_server", "behavior_server",
            "bt_navigator", "velocity_smoother"],
        help="自动运行前必须为active的Nav2生命周期节点")
    route.add_argument(
        "--skip-start-check", action="store_true",
        help="跳过map到L1距离检查；仅在明确确认位于L1且TF临时不可用时使用")
    compare = subparsers.add_parser("compare", help="叠加比较两次记录")
    compare.add_argument("before", help="修改前记录目录或run.json")
    compare.add_argument("after", help="修改后记录目录或run.json")
    compare.add_argument("--output", default=None, help="对比PNG输出位置")
    return parser


def main():
    parser = build_parser()
    # ROS-specific arguments are accepted only by record and kept away from argparse.
    regular_args = sys.argv[1:]
    ros_args = None
    if "--ros-args" in regular_args:
        split = regular_args.index("--ros-args")
        ros_args = [sys.argv[0]] + regular_args[split:]
        regular_args = regular_args[:split]
    args = parser.parse_args(regular_args)
    if args.command in ("record", "route"):
        record_command(args, ros_args)
    else:
        compare_command(args)


if __name__ == "__main__":
    main()
