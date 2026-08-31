#!/usr/bin/env python3
"""按目标点批量统计末端速度欠调恢复事件（TVUR）。

TVUR = Terminal Velocity Undershoot Recovery：进入 approach 后，目标仍要求
非零运动，但机器人实际速度显著下探，随后在到点前重新回升。俗称“减过头后
又加速”。脚本同时区分：

1. reference：控制器消费目标自身先下探再回升；
2. command：控制器输出相对非零目标下探再回升；
3. smoothed：速度平滑器输出下探再回升；
4. odom：机器人实测速度下探再回升（最终事件主判据）。

每个 nav_task_*.json 是一次单目标导航。多个文件按 goal_pose 自动聚类为目标点，
分别统计事件频率，最后输出全部有效目标任务的总频率。
"""

import argparse
import bisect
import csv
import json
import math
import statistics
import sys
from pathlib import Path


DEFAULT_PATTERN = "nav_task_*.json"
DETAILS_NAME = "terminal_velocity_undershoot_details.csv"
SUMMARY_NAME = "terminal_velocity_undershoot_summary.csv"
APPROACH_PHASE = "approach"


def finite(value, default=None):
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        value = float(value)
        return value if math.isfinite(value) else default
    return default


def angle_error(first, second):
    return (first - second + math.pi) % (2.0 * math.pi) - math.pi


def load_samples(task, field, value_index=1):
    """读取JSON中的 [t, value, ...]，过滤非法值并合并重复时间戳。"""
    block = task.get(field, {})
    raw = block.get("samples", []) if isinstance(block, dict) else []
    points = []
    for sample in raw:
        if not isinstance(sample, list) or len(sample) <= value_index:
            continue
        timestamp = finite(sample[0])
        value = finite(sample[value_index])
        if timestamp is not None and value is not None:
            points.append((timestamp, value))
    points.sort()
    unique = []
    for point in points:
        if unique and abs(point[0] - unique[-1][0]) <= 1e-9:
            unique[-1] = point
        else:
            unique.append(point)
    return unique


def nearest_sample(samples, timestamp, max_gap):
    if not samples:
        return None
    times = [point[0] for point in samples]
    index = bisect.bisect_left(times, timestamp)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index > 0:
        candidates.append(samples[index - 1])
    nearest = min(candidates, key=lambda point: abs(point[0] - timestamp))
    return nearest if abs(nearest[0] - timestamp) <= max_gap else None


def approach_start_time(task):
    for transition in task.get("phase_transitions", []):
        if isinstance(transition, list) and len(transition) >= 2:
            timestamp, phase = finite(transition[0]), transition[1]
        elif isinstance(transition, dict):
            timestamp, phase = finite(transition.get("t")), transition.get("phase")
        else:
            continue
        if timestamp is not None and phase == APPROACH_PHASE:
            return timestamp
    return None


def nominal_period(points, default=0.1):
    intervals = [
        second[0] - first[0]
        for first, second in zip(points, points[1:])
        if second[0] > first[0]
    ]
    return statistics.median(intervals) if intervals else default


def condition_segments(observations, key, args):
    """合并连续欠调点，并筛掉过短的单点噪声。"""
    period = nominal_period([(row["t"], 0.0) for row in observations])
    segments = []
    active = None
    for row in observations:
        reference = row["target"]
        actual = row.get(key)
        condition = (
            actual is not None and
            reference >= args.target_motion_min and
            reference - actual >= args.min_tracking_gap and
            actual <= max(args.actual_low_max, reference * args.undershoot_ratio)
        )
        if not condition:
            if active is not None:
                active["duration"] = active["end"] - active["start"] + period
                if active["duration"] + 1e-9 >= args.min_low_duration:
                    segments.append(active)
                active = None
            continue
        if active is None or row["t"] - active["end"] > args.segment_max_gap:
            if active is not None:
                active["duration"] = active["end"] - active["start"] + period
                if active["duration"] + 1e-9 >= args.min_low_duration:
                    segments.append(active)
            active = {
                "start": row["t"], "end": row["t"],
                "valley": actual, "reference_at_valley": reference,
            }
        else:
            active["end"] = row["t"]
            if actual < active["valley"]:
                active["valley"] = actual
                active["reference_at_valley"] = reference
    if active is not None:
        active["duration"] = active["end"] - active["start"] + period
        if active["duration"] + 1e-9 >= args.min_low_duration:
            segments.append(active)
    return segments


def confirmed_tracking_events(observations, key, args):
    """欠调片段前须正常运动，片段后须在限定时间内恢复。"""
    events = []
    for segment in condition_segments(observations, key, args):
        before = [
            row for row in observations
            if segment["start"] - args.pre_motion_window <= row["t"] < segment["start"]
            and row.get(key) is not None
        ]
        had_motion = any(
            row[key] >= args.recovery_abs_min and
            row[key] - segment["valley"] >= args.min_recovery_delta
            for row in before
        )
        if not had_motion:
            continue
        after = [
            row for row in observations
            if segment["end"] < row["t"] <= segment["end"] + args.recovery_window
            and row.get(key) is not None
            and row["target"] >= args.target_motion_min
        ]
        recovery = next((
            row for row in after
            if row[key] >= max(
                args.recovery_abs_min,
                row["target"] * args.recovery_ratio)
            and row[key] - segment["valley"] >= args.min_recovery_delta
        ), None)
        if recovery is None:
            continue
        event = dict(segment)
        event["recovery_time"] = recovery["t"]
        event["recovery_speed"] = recovery[key]
        events.append(event)
    return events


def reference_valley_events(targets, start_time, args):
    """检测控制器消费目标自身的低速谷值后回升。"""
    points = [(t, abs(v)) for t, v in targets if t >= start_time]
    events = []
    index = 0
    while index < len(points):
        timestamp, value = points[index]
        if value > args.reference_valley_ceiling:
            index += 1
            continue
        before = [
            item for item in points
            if timestamp - args.pre_motion_window <= item[0] < timestamp
        ]
        after = [
            item for item in points
            if timestamp < item[0] <= timestamp + args.recovery_window
        ]
        if not before or not after:
            index += 1
            continue
        prior_peak = max(value_ for _, value_ in before)
        recovery = next((
            item for item in after
            if item[1] - value >= args.reference_min_drop
        ), None)
        if (
            prior_peak - value >= args.reference_min_drop and
            recovery is not None
        ):
            events.append({
                "start": timestamp,
                "end": timestamp,
                "duration": 0.0,
                "valley": value,
                "reference_at_valley": value,
                "recovery_time": recovery[0],
                "recovery_speed": recovery[1],
            })
            while index + 1 < len(points) and points[index + 1][0] <= recovery[0]:
                index += 1
        index += 1
    return events


def build_observations(task, start_time, args, warnings):
    targets = load_samples(task, "controller_target")
    if not targets:
        targets = load_samples(task, "desired_vel")
        if targets:
            warnings.append("缺少controller_target，已回退desired_vel")
    cmd = load_samples(task, "cmd_vel")
    smoothed = load_samples(task, "smoothed_vel")
    odom = load_samples(task, "odom_vel")
    if not targets:
        warnings.append("缺少目标速度")
        return [], targets
    if not cmd:
        warnings.append("缺少cmd_vel")
    if not smoothed:
        warnings.append("缺少smoothed_vel")
    if not odom:
        warnings.append("缺少odom_vel")

    observations = []
    for timestamp, target in targets:
        if timestamp < start_time:
            continue
        row = {"t": timestamp, "target": abs(target)}
        for key, samples in (("cmd", cmd), ("smoothed", smoothed), ("odom", odom)):
            sample = nearest_sample(samples, timestamp, args.sample_match_max_gap)
            row[key] = abs(sample[1]) if sample is not None else None
        observations.append(row)
    return observations, targets


def analyze_file(path, args):
    warnings = []
    with path.open(encoding="utf-8") as stream:
        task = json.load(stream)
    target_samples = load_samples(task, "controller_target")
    approach_start = approach_start_time(task)
    if approach_start is None:
        if target_samples:
            approach_start = max(
                target_samples[0][0], target_samples[-1][0] - args.fallback_terminal_window)
            warnings.append("缺少approach记录，使用目标速度末段窗口")
        else:
            approach_start = 0.0
    observations, targets = build_observations(task, approach_start, args, warnings)
    events = {
        "reference": reference_valley_events(targets, approach_start, args),
        "command": confirmed_tracking_events(observations, "cmd", args),
        "smoothed": confirmed_tracking_events(observations, "smoothed", args),
        "odom": confirmed_tracking_events(observations, "odom", args),
    }
    # 机器人实测为主；目标自身出现明确谷值回升时也应计入，因为三路执行信号
    # 可能只是忠实跟随了错误目标，单看target-odom差值会漏检。
    detected = bool(events["odom"] or events["reference"])
    event_types = [key for key, value in events.items() if value]
    all_events = [event for value in events.values() for event in value]
    first_event = min((event["start"] for event in all_events), default=None)

    goal = task.get("goal_pose", {}) if isinstance(task.get("goal_pose"), dict) else {}
    row = {
        "file": path.name,
        "result": task.get("result", "unknown"),
        "goal_x": finite(goal.get("x")),
        "goal_y": finite(goal.get("y")),
        "goal_yaw": finite(goal.get("yaw")),
        "approach_start_s": approach_start,
        "valid": bool(observations and any(row_["odom"] is not None for row_ in observations)),
        "tvur_detected": detected,
        "event_types": "+".join(event_types),
        "reference_event_count": len(events["reference"]),
        "command_event_count": len(events["command"]),
        "smoothed_event_count": len(events["smoothed"]),
        "odom_event_count": len(events["odom"]),
        "first_event_start_s": first_event,
        "max_target_odom_gap_mps": max((
            abs(row_["target"] - row_["odom"])
            for row_ in observations if row_["odom"] is not None
        ), default=None),
        "warnings": "；".join(warnings),
        "_task": task,
    }
    return row


def explicit_goal_name(task):
    for key in ("goal_name", "target_name", "nav_point_name", "point_name"):
        value = task.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return None


def assign_goal_groups(rows, args):
    clusters = []
    for row in rows:
        explicit = explicit_goal_name(row["_task"])
        if explicit is not None:
            cluster = next((item for item in clusters if item.get("explicit") == explicit), None)
            if cluster is None:
                cluster = {"explicit": explicit, "rows": [], "x": row["goal_x"],
                           "y": row["goal_y"], "yaw": row["goal_yaw"]}
                clusters.append(cluster)
            cluster["rows"].append(row)
            continue
        if row["goal_x"] is None or row["goal_y"] is None:
            cluster = {"explicit": "UNKNOWN", "rows": [row],
                       "x": None, "y": None, "yaw": None}
            clusters.append(cluster)
            continue
        cluster = next((item for item in clusters if (
            item.get("explicit") is None and item["x"] is not None and
            math.hypot(row["goal_x"] - item["x"], row["goal_y"] - item["y"])
            <= args.goal_position_tolerance and
            (
                row["goal_yaw"] is None or item["yaw"] is None or
                abs(angle_error(row["goal_yaw"], item["yaw"]))
                <= math.radians(args.goal_yaw_tolerance_deg)
            )
        )), None)
        if cluster is None:
            cluster = {"explicit": None, "rows": [], "x": row["goal_x"],
                       "y": row["goal_y"], "yaw": row["goal_yaw"]}
            clusters.append(cluster)
        cluster["rows"].append(row)

    clusters.sort(key=lambda item: (
        item.get("explicit") or "", item["x"] if item["x"] is not None else math.inf,
        item["y"] if item["y"] is not None else math.inf))
    for index, cluster in enumerate(clusters, start=1):
        goal_id = cluster.get("explicit") or f"G{index:02d}"
        for row in cluster["rows"]:
            row["goal_id"] = goal_id
    return clusters


DETAIL_FIELDS = [
    "file", "result", "goal_id", "goal_x", "goal_y", "goal_yaw",
    "approach_start_s", "valid", "tvur_detected", "event_types",
    "reference_event_count", "command_event_count", "smoothed_event_count",
    "odom_event_count", "first_event_start_s", "max_target_odom_gap_mps", "warnings",
]


SUMMARY_FIELDS = [
    "goal_id", "goal_x", "goal_y", "goal_yaw", "task_count", "valid_task_count",
    "tvur_task_count", "tvur_frequency", "reference_task_count",
    "command_task_count", "smoothed_task_count", "odom_task_count",
]


def write_csv(path, rows, fields):
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def summarize_clusters(clusters):
    summaries = []
    for cluster in clusters:
        rows = cluster["rows"]
        valid = [row for row in rows if row["valid"]]
        events = [row for row in valid if row["tvur_detected"]]
        summaries.append({
            "goal_id": rows[0]["goal_id"],
            "goal_x": cluster["x"], "goal_y": cluster["y"], "goal_yaw": cluster["yaw"],
            "task_count": len(rows), "valid_task_count": len(valid),
            "tvur_task_count": len(events),
            "tvur_frequency": len(events) / len(valid) if valid else None,
            "reference_task_count": sum(bool(row["reference_event_count"]) for row in valid),
            "command_task_count": sum(bool(row["command_event_count"]) for row in valid),
            "smoothed_task_count": sum(bool(row["smoothed_event_count"]) for row in valid),
            "odom_task_count": sum(bool(row["odom_event_count"]) for row in valid),
        })
    return summaries


def percent(value):
    return "N/A" if value is None else f"{100.0 * value:.2f}%"


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_dir", type=Path, help="包含多个nav_task_*.json的目录")
    parser.add_argument("--pattern", default=DEFAULT_PATTERN)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--no-save", action="store_true")
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--goal-position-tolerance", type=float, default=0.10)
    parser.add_argument("--goal-yaw-tolerance-deg", type=float, default=10.0)
    parser.add_argument("--sample-match-max-gap", type=float, default=0.15)
    parser.add_argument("--fallback-terminal-window", type=float, default=8.0)
    parser.add_argument("--target-motion-min", type=float, default=0.02)
    parser.add_argument("--min-tracking-gap", type=float, default=0.02)
    parser.add_argument("--actual-low-max", type=float, default=0.012)
    parser.add_argument("--undershoot-ratio", type=float, default=0.35)
    parser.add_argument("--min-low-duration", type=float, default=0.15)
    parser.add_argument("--segment-max-gap", type=float, default=0.22)
    parser.add_argument("--pre-motion-window", type=float, default=2.0)
    parser.add_argument("--recovery-window", type=float, default=3.0)
    parser.add_argument("--recovery-ratio", type=float, default=0.75)
    parser.add_argument("--recovery-abs-min", type=float, default=0.02)
    parser.add_argument("--min-recovery-delta", type=float, default=0.015)
    parser.add_argument("--reference-valley-ceiling", type=float, default=0.08)
    parser.add_argument("--reference-min-drop", type=float, default=0.02)
    return parser


def main():
    args = build_parser().parse_args()
    if not args.input_dir.is_dir():
        raise SystemExit(f"输入目录不存在：{args.input_dir}")
    files = sorted(args.input_dir.glob(args.pattern))
    if not files:
        raise SystemExit(f"没有匹配文件：{args.input_dir / args.pattern}")

    rows = []
    for path in files:
        try:
            row = analyze_file(path, args)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            print(f"跳过 {path.name}：{error}", file=sys.stderr)
            continue
        rows.append(row)
    if not rows:
        raise SystemExit("没有可分析的JSON")

    clusters = assign_goal_groups(rows, args)
    summaries = summarize_clusters(clusters)
    if not args.quiet:
        for row in rows:
            state = "异常" if row["tvur_detected"] else "正常"
            validity = "" if row["valid"] else "（数据不足）"
            types = row["event_types"] or "-"
            print(f"[{state}] {row['file']} -> {row['goal_id']} 类型={types}{validity}")

    print("\n按目标点统计：")
    for summary in summaries:
        pose = (
            f"({summary['goal_x']:.3f}, {summary['goal_y']:.3f})"
            if summary["goal_x"] is not None else "(unknown)"
        )
        print(
            f"  {summary['goal_id']} {pose}: "
            f"{summary['tvur_task_count']}/{summary['valid_task_count']} = "
            f"{percent(summary['tvur_frequency'])}")

    valid_rows = [row for row in rows if row["valid"]]
    event_rows = [row for row in valid_rows if row["tvur_detected"]]
    total_frequency = len(event_rows) / len(valid_rows) if valid_rows else None
    print("\n总出现频率：")
    print(f"  {len(event_rows)}/{len(valid_rows)} = {percent(total_frequency)}")

    if not args.no_save:
        output_dir = args.output_dir or args.input_dir
        output_dir.mkdir(parents=True, exist_ok=True)
        write_csv(output_dir / DETAILS_NAME, rows, DETAIL_FIELDS)
        write_csv(output_dir / SUMMARY_NAME, summaries, SUMMARY_FIELDS)
        print(f"\n逐轮明细：{output_dir / DETAILS_NAME}")
        print(f"目标汇总：{output_dir / SUMMARY_NAME}")


if __name__ == "__main__":
    main()
