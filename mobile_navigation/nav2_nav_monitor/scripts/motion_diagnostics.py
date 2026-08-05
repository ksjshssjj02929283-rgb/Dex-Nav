#!/usr/bin/env python3
# Copyright 2026 DEXE Robot Engineer
# SPDX-License-Identifier: Apache-2.0
"""
Nav monitor 线运动导数与限值检测的公共实现.

controller_target 的 acceleration 是控制器实际消费的轨迹状态，直接使用；
jerk 由 acceleration 对时间做局部线性回归得到。cmd/smoothed/odom 只有速度，
其 acceleration/jerk 均为带时间窗的估算值，不能当作 IMU 直接测量。
"""

import bisect
import math


DEFAULT_ACCEL_WINDOW_SEC = 0.30
DEFAULT_JERK_WINDOW_SEC = 0.30
DEFAULT_MAX_GAP_SEC = 0.50


def _finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def sample_series(samples, value_index):
    """从 [t,...] 样本提取单调有限 (t,value)，相同时间保留最后一项."""
    points = []
    for sample in samples:
        if len(sample) <= value_index:
            continue
        t, value = sample[0], sample[value_index]
        if _finite(t) and _finite(value):
            points.append((float(t), float(value)))
    points.sort(key=lambda point: point[0])

    times, values = [], []
    for t, value in points:
        if times and abs(t - times[-1]) <= 1e-9:
            values[-1] = value
        else:
            times.append(t)
            values.append(value)
    return times, values


def local_linear_derivative(
        times, values, window_sec, max_gap_sec=DEFAULT_MAX_GAP_SEC):
    """局部线性回归求 dy/dt；跨长采样空洞不连线，缺数据位置返回 NaN."""
    count = min(len(times), len(values))
    if count == 0:
        return []
    if window_sec <= 0.0:
        raise ValueError("window_sec must be positive")

    result = [math.nan] * count
    segment_start = 0
    while segment_start < count:
        segment_end = segment_start + 1
        while segment_end < count:
            dt = times[segment_end] - times[segment_end - 1]
            if dt <= 0.0 or dt > max_gap_sec:
                break
            segment_end += 1

        half_window = window_sec / 2.0
        for i in range(segment_start, segment_end):
            if not _finite(values[i]):
                continue
            left = bisect.bisect_left(
                times, times[i] - half_window, segment_start, segment_end)
            right = bisect.bisect_right(
                times, times[i] + half_window, segment_start, segment_end)
            indices = [
                j for j in range(left, right) if _finite(values[j])
            ]
            # 两点差分极易把同一控制周期内的重复回显放大成假 jerk；
            # 至少三个时间点才输出局部斜率，边界/稀疏处宁可留空。
            if len(indices) < 3:
                continue

            mean_t = sum(times[j] for j in indices) / len(indices)
            mean_y = sum(values[j] for j in indices) / len(indices)
            denominator = sum((times[j] - mean_t) ** 2 for j in indices)
            if denominator <= 1e-12:
                continue
            numerator = sum(
                (times[j] - mean_t) * (values[j] - mean_y) for j in indices)
            result[i] = numerator / denominator

        segment_start = segment_end
    return result


def controller_target_dynamics(
        samples, jerk_window_sec=DEFAULT_JERK_WINDOW_SEC):
    """控制器轨迹目标：[t,v,a] → 显式 a + 回归得到的 jerk."""
    times, velocity = sample_series(samples, 1)
    accel_times, acceleration = sample_series(samples, 2)
    if times != accel_times:
        acceleration_by_time = dict(zip(accel_times, acceleration))
        common_times = [t for t in times if t in acceleration_by_time]
        velocity_by_time = dict(zip(times, velocity))
        times = common_times
        velocity = [velocity_by_time[t] for t in times]
        acceleration = [acceleration_by_time[t] for t in times]
    jerk = local_linear_derivative(times, acceleration, jerk_window_sec)
    return {
        "time": times,
        "velocity": velocity,
        "acceleration": acceleration,
        "jerk": jerk,
        "acceleration_source": "explicit_controller_target",
    }


def velocity_dynamics(
        samples, acceleration_window_sec=DEFAULT_ACCEL_WINDOW_SEC,
        jerk_window_sec=DEFAULT_JERK_WINDOW_SEC):
    """速度样本 [t,v,...] → 局部回归估算 acceleration 与 jerk."""
    times, velocity = sample_series(samples, 1)
    acceleration = local_linear_derivative(
        times, velocity, acceleration_window_sec)
    jerk = local_linear_derivative(times, acceleration, jerk_window_sec)
    return {
        "time": times,
        "velocity": velocity,
        "acceleration": acceleration,
        "jerk": jerk,
        "acceleration_source": "estimated_from_velocity",
    }


def limits_from_task(task, overrides=None):
    """读取任务内嵌限值，并用非空 CLI override 覆盖；非正值表示未配置."""
    raw = task.get("motion_limits", {})
    limits = {
        "max_acceleration": raw.get("max_acceleration"),
        "max_deceleration": raw.get("max_deceleration"),
        "max_abs_jerk": raw.get("max_abs_jerk"),
        "tolerance_ratio": raw.get("tolerance_ratio", 0.05),
    }
    if overrides:
        for key, value in overrides.items():
            if value is not None:
                limits[key] = value
    for key in ("max_acceleration", "max_deceleration", "max_abs_jerk"):
        value = limits.get(key)
        limits[key] = float(value) if _finite(value) and value > 0.0 else None
    tolerance = limits.get("tolerance_ratio")
    limits["tolerance_ratio"] = (
        max(float(tolerance), 0.0) if _finite(tolerance) else 0.05)
    return limits


def _event_count(values, predicate):
    count = 0
    active = False
    for value in values:
        exceeded = _finite(value) and predicate(value)
        if exceeded and not active:
            count += 1
        active = exceeded
    return count


def summarize_dynamics(dynamics, limits=None):
    """返回峰值与连续超限事件数；无配置限值时只统计、不判失败."""
    acceleration = [v for v in dynamics.get("acceleration", []) if _finite(v)]
    jerk = [v for v in dynamics.get("jerk", []) if _finite(v)]
    summary = {
        "sample_count": len(dynamics.get("time", [])),
        "max_acceleration": max(acceleration, default=0.0),
        "min_acceleration": min(acceleration, default=0.0),
        "max_abs_acceleration": max((abs(v) for v in acceleration), default=0.0),
        "max_abs_jerk": max((abs(v) for v in jerk), default=0.0),
        "acceleration_exceedance_events": 0,
        "deceleration_exceedance_events": 0,
        "jerk_exceedance_events": 0,
        "status": "no_data" if not acceleration else "no_limits",
    }
    if not acceleration:
        return summary

    limits = limits or {}
    tolerance = max(float(limits.get("tolerance_ratio", 0.0)), 0.0)
    accel_limit = limits.get("max_acceleration")
    decel_limit = limits.get("max_deceleration")
    jerk_limit = limits.get("max_abs_jerk")
    configured = any(v is not None for v in (accel_limit, decel_limit, jerk_limit))
    if not configured:
        return summary

    if accel_limit is not None:
        threshold = accel_limit * (1.0 + tolerance)
        summary["acceleration_exceedance_events"] = _event_count(
            dynamics["acceleration"], lambda value: value > threshold)
    if decel_limit is not None:
        threshold = decel_limit * (1.0 + tolerance)
        summary["deceleration_exceedance_events"] = _event_count(
            dynamics["acceleration"], lambda value: value < -threshold)
    if jerk_limit is not None:
        threshold = jerk_limit * (1.0 + tolerance)
        summary["jerk_exceedance_events"] = _event_count(
            dynamics["jerk"], lambda value: abs(value) > threshold)

    total_events = (
        summary["acceleration_exceedance_events"] +
        summary["deceleration_exceedance_events"] +
        summary["jerk_exceedance_events"])
    summary["status"] = "exceeded" if total_events else "ok"
    return summary
