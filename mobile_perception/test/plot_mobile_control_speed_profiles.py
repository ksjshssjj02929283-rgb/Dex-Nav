#!/usr/bin/env python3
"""Plot the production mobile_control S profile and controller speed target.

The C++ dumper links the production SCurveSpeedProfilePlanner and calls the
production StraightController/RotationController for every profile point.
Python only loads parameters, invokes that executable and renders its CSV.
"""

from __future__ import annotations

import argparse
import csv
import io
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from typing import Dict, Iterable, List

import yaml

os.environ.setdefault(
    "MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "dexe_mobile_control_matplotlib")
)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


PROFILE_FIELDS = (
    "time",
    "progress_position",
    "profile_velocity",
    "reference_position",
    "remaining_motion",
    "planned_velocity",
    "runtime_target_velocity",
    "planned_acceleration",
    "planned_jerk",
    "slowdown_zone",
    "slowdown_applied",
    "braking_limited",
    "minimum_velocity_applied",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate linear 0.1-1.0 m and angular 0-180 deg curves by calling "
            "the production S planner and production controller slowdown logic."
        )
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path.home() / ".dexe" / "navigation" / "mobile_control_speed_profiles",
        help="Output directory (default: ~/.dexe/navigation/mobile_control_speed_profiles)",
    )
    parser.add_argument(
        "--planner-executable",
        type=Path,
        help="Path to mobile_control_speed_profile_dump; auto-detected when omitted",
    )
    parser.add_argument(
        "--params-file",
        type=Path,
        help="mobile_control_action_params.yaml; source/install path is auto-detected when omitted",
    )
    parser.add_argument("--sample-period", type=float)
    parser.add_argument("--max-duration", type=float)
    parser.add_argument("--preview-time", type=float)
    parser.add_argument("--linear-max-velocity", type=float)
    parser.add_argument("--linear-max-acceleration", type=float)
    parser.add_argument("--linear-max-deceleration", type=float)
    parser.add_argument("--linear-max-jerk", type=float)
    parser.add_argument("--linear-slowdown-distance", type=float)
    parser.add_argument("--linear-slowdown-speed", type=float)
    parser.add_argument("--angular-max-velocity", type=float)
    parser.add_argument("--angular-max-acceleration", type=float)
    parser.add_argument("--angular-max-deceleration", type=float)
    parser.add_argument("--angular-max-jerk", type=float)
    parser.add_argument("--angular-slowdown-angle", type=float)
    parser.add_argument("--angular-slowdown-speed", type=float)
    parser.add_argument("--angular-min-velocity", type=float)
    parser.add_argument("--yaw-tolerance", type=float)
    return parser.parse_args()


def find_planner_executable(explicit: Path | None) -> Path:
    candidates: List[Path] = []
    if explicit is not None:
        candidates.append(explicit.expanduser())

    env_path = os.environ.get("MOBILE_CONTROL_SPEED_PROFILE_DUMP")
    if env_path:
        candidates.append(Path(env_path).expanduser())

    script_path = Path(__file__).resolve()
    candidates.extend(
        [
            script_path.with_name("mobile_control_speed_profile_dump"),
            script_path.parents[1]
            / "build"
            / "mobile_perception"
            / "mobile_control_speed_profile_dump",
        ]
    )
    path_match = shutil.which("mobile_control_speed_profile_dump")
    if path_match:
        candidates.append(Path(path_match))

    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved.is_file() and os.access(resolved, os.X_OK):
            return resolved

    searched = "\n  ".join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(
        "mobile_control_speed_profile_dump was not found. Build mobile_perception "
        "with BUILD_TESTING enabled or pass --planner-executable. Searched:\n  "
        + searched
    )


def find_params_file(explicit: Path | None) -> Path:
    candidates: List[Path] = []
    if explicit is not None:
        candidates.append(explicit.expanduser())

    script_path = Path(__file__).resolve()
    candidates.extend(
        [
            script_path.parents[1] / "config" / "mobile_control_action_params.yaml",
            script_path.parents[2]
            / "share"
            / "mobile_perception"
            / "config"
            / "mobile_control_action_params.yaml",
        ]
    )
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            candidates.append(
                Path(prefix)
                / "share"
                / "mobile_perception"
                / "config"
                / "mobile_control_action_params.yaml"
            )

    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved.is_file():
            return resolved

    searched = "\n  ".join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(
        "mobile_control_action_params.yaml was not found. Pass --params-file. Searched:\n  "
        + searched
    )


def load_runtime_parameters(path: Path, args: argparse.Namespace) -> Dict[str, Dict[str, float]]:
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    try:
        params = document["mobile_control_action_node"]["ros__parameters"]
        s_curve = params["s_curve"]
        linear = s_curve["linear"]
        angular = s_curve["angular"]
        straight = params["straight"]
        rotation = params["rotation"]
        goal_checker = params["goal_checker"]
    except (KeyError, TypeError) as error:
        raise RuntimeError(f"invalid mobile_control parameter structure in {path}: {error}") from error

    def selected(argument: str, configured: float) -> float:
        override = getattr(args, argument)
        return float(configured if override is None else override)

    return {
        "common": {
            "sample_period": selected("sample_period", s_curve["sample_period"]),
            "max_duration": selected("max_duration", s_curve["max_duration"]),
            "preview_time": selected(
                "preview_time", params["trajectory_tracking"]["preview_time"]
            ),
        },
        "linear": {
            "max_velocity": selected("linear_max_velocity", linear["max_velocity"]),
            "max_acceleration": selected(
                "linear_max_acceleration", linear["max_acceleration"]
            ),
            "max_deceleration": selected(
                "linear_max_deceleration", linear["max_deceleration"]
            ),
            "max_jerk": selected("linear_max_jerk", linear["max_jerk"]),
            "slowdown_motion": selected(
                "linear_slowdown_distance", straight["stop_distance"]
            ),
            "slowdown_speed": selected(
                "linear_slowdown_speed", straight["stop_speed"]
            ),
            "min_velocity": 0.0,
            "goal_tolerance": float(goal_checker["longitudinal_tolerance"]),
        },
        "angular": {
            "max_velocity": selected("angular_max_velocity", angular["max_velocity"]),
            "max_acceleration": selected(
                "angular_max_acceleration", angular["max_acceleration"]
            ),
            "max_deceleration": selected(
                "angular_max_deceleration", angular["max_deceleration"]
            ),
            "max_jerk": selected("angular_max_jerk", angular["max_jerk"]),
            "slowdown_motion": selected(
                "angular_slowdown_angle", rotation["terminal_slowdown_angle"]
            ),
            "slowdown_speed": selected(
                "angular_slowdown_speed", rotation["terminal_angular_velocity"]
            ),
            "min_velocity": selected(
                "angular_min_velocity", rotation["min_angular_velocity"]
            ),
            "goal_tolerance": selected(
                "yaw_tolerance", goal_checker["yaw_tolerance"]
            ),
        },
    }


def run_runtime_reference(
    executable: Path,
    mode: str,
    distance: float,
    config: Dict[str, float],
    common: Dict[str, float],
) -> List[Dict[str, float]]:
    command = [
        str(executable),
        "--mode",
        mode,
        "--distance",
        f"{distance:.17g}",
        "--max-velocity",
        f"{config['max_velocity']:.17g}",
        "--max-acceleration",
        f"{config['max_acceleration']:.17g}",
        "--max-deceleration",
        f"{config['max_deceleration']:.17g}",
        "--max-jerk",
        f"{config['max_jerk']:.17g}",
        "--slowdown-motion",
        f"{config['slowdown_motion']:.17g}",
        "--slowdown-speed",
        f"{config['slowdown_speed']:.17g}",
        "--goal-tolerance",
        f"{config['goal_tolerance']:.17g}",
        "--min-velocity",
        f"{config['min_velocity']:.17g}",
        "--preview-time",
        f"{common['preview_time']:.17g}",
        "--sample-period",
        f"{common['sample_period']:.17g}",
        "--max-duration",
        f"{common['max_duration']:.17g}",
    ]
    result = subprocess.run(command, check=True, capture_output=True, text=True)
    reader = csv.DictReader(io.StringIO(result.stdout))
    if tuple(reader.fieldnames or ()) != PROFILE_FIELDS:
        raise RuntimeError(f"unexpected runtime-reference CSV header: {reader.fieldnames}")
    rows = [{field: float(row[field]) for field in PROFILE_FIELDS} for row in reader]
    if not rows:
        raise RuntimeError("runtime-reference dumper returned an empty profile")
    return rows


def write_csv(path: Path, rows: Iterable[Dict[str, float]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=PROFILE_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def plot_profile(
    path: Path,
    rows: List[Dict[str, float]],
    title: str,
    position_unit: str,
    velocity_unit: str,
    acceleration_unit: str,
    jerk_unit: str,
    config: Dict[str, float],
) -> None:
    time = [row["time"] for row in rows]
    fig, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True)

    axes[0].plot(
        time,
        [row["progress_position"] for row in rows],
        linewidth=2,
        label="ideal progress",
    )
    axes[0].plot(
        time,
        [row["reference_position"] for row in rows],
        linewidth=1.5,
        linestyle="-.",
        label="runtime sampled reference",
    )
    axes[0].plot(
        time,
        [row["remaining_motion"] for row in rows],
        linewidth=1.5,
        linestyle="--",
        label="remaining",
    )
    axes[0].axhline(
        config["slowdown_motion"],
        color="tab:orange",
        linestyle=":",
        label="slowdown threshold",
    )
    axes[0].set_ylabel(f"motion [{position_unit}]")
    axes[0].legend(loc="best")

    axes[1].plot(
        time,
        [row["profile_velocity"] for row in rows],
        linewidth=2,
        label="planned composite S velocity",
    )
    axes[1].plot(
        time,
        [row["planned_velocity"] for row in rows],
        linewidth=1.5,
        linestyle="-.",
        label="runtime sampled S velocity",
    )
    axes[1].plot(
        time,
        [row["runtime_target_velocity"] for row in rows],
        linewidth=2,
        linestyle="--",
        label="runtime controller target",
    )
    axes[1].axhline(
        config["max_velocity"],
        color="tab:red",
        linestyle="--",
        alpha=0.7,
        label="configured max",
    )
    axes[1].axhline(
        config["slowdown_speed"],
        color="tab:orange",
        linestyle=":",
        label="configured slowdown speed",
    )
    effective_slowdown_speed = max(config["slowdown_speed"], config["min_velocity"])
    if not math.isclose(effective_slowdown_speed, config["slowdown_speed"]):
        axes[1].axhline(
            effective_slowdown_speed,
            color="tab:purple",
            linestyle="-.",
            label="effective slowdown speed",
        )
    axes[1].set_ylabel(f"velocity [{velocity_unit}]")
    axes[1].legend(loc="best")

    axes[2].plot(
        time,
        [row["planned_acceleration"] for row in rows],
        linewidth=2,
        label="runtime sampled S acceleration",
    )
    axes[2].set_ylabel(f"acceleration [{acceleration_unit}]")
    axes[2].legend(loc="best")

    axes[3].plot(
        time,
        [row["planned_jerk"] for row in rows],
        linewidth=2,
        label="runtime sampled S jerk",
    )
    axes[3].set_ylabel(f"jerk [{jerk_unit}]")
    axes[3].set_xlabel("time [s]")
    axes[3].legend(loc="best")

    slowdown_times = [row["time"] for row in rows if row["slowdown_zone"] > 0.5]
    if slowdown_times:
        for axis in axes:
            axis.axvspan(
                min(slowdown_times),
                max(slowdown_times),
                color="tab:orange",
                alpha=0.10,
            )
    for axis in axes:
        axis.grid(True, alpha=0.3)

    fig.suptitle(
        f"{title}\n"
        f"slowdown <= {config['slowdown_motion']:.4g} {position_unit}, "
        f"configured speed={config['slowdown_speed']:.4g} {velocity_unit}, "
        f"effective={effective_slowdown_speed:.4g} {velocity_unit}"
    )
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def profile_summary(
    mode: str,
    motion: float,
    rows: List[Dict[str, float]],
    config: Dict[str, float],
) -> Dict[str, str]:
    return {
        "mode": mode,
        "motion": f"{motion:.9g}",
        "duration": f"{rows[-1]['time']:.9g}",
        "points": str(len(rows)),
        "planned_peak_velocity": f"{max(abs(row['planned_velocity']) for row in rows):.9g}",
        "profile_velocity_at_slowdown_point": (
            f"{min(rows, key=lambda row: abs(row['remaining_motion'] - config['slowdown_motion']))['profile_velocity']:.9g}"
        ),
        "runtime_peak_target_velocity": (
            f"{max(abs(row['runtime_target_velocity']) for row in rows):.9g}"
        ),
        "peak_acceleration": (
            f"{max(row['planned_acceleration'] for row in rows):.9g}"
        ),
        "peak_deceleration": (
            f"{max(-row['planned_acceleration'] for row in rows):.9g}"
        ),
        "peak_jerk": f"{max(abs(row['planned_jerk']) for row in rows):.9g}",
        "slowdown_motion": f"{config['slowdown_motion']:.9g}",
        "configured_slowdown_speed": f"{config['slowdown_speed']:.9g}",
        "effective_slowdown_speed": (
            f"{max(config['slowdown_speed'], config['min_velocity']):.9g}"
        ),
        "slowdown_zone_points": str(
            sum(row["slowdown_zone"] > 0.5 for row in rows)
        ),
        "slowdown_applied_points": str(
            sum(row["slowdown_applied"] > 0.5 for row in rows)
        ),
    }


def main() -> int:
    args = parse_args()
    executable = find_planner_executable(args.planner_executable)
    params_file = find_params_file(args.params_file)
    runtime_config = load_runtime_parameters(params_file, args)
    common = runtime_config["common"]
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    summaries: List[Dict[str, str]] = []
    linear_config = runtime_config["linear"]
    for tenth_metres in range(1, 11):
        distance = tenth_metres / 10.0
        rows = run_runtime_reference(
            executable, "linear", distance, linear_config, common
        )
        stem = f"linear_{distance:.1f}m"
        write_csv(output_dir / f"{stem}.csv", rows)
        plot_profile(
            output_dir / f"{stem}.png",
            rows,
            f"Linear runtime speed reference: {distance:.1f} m",
            "m",
            "m/s",
            "m/s²",
            "m/s³",
            linear_config,
        )
        summaries.append(
            profile_summary("linear", distance, rows, linear_config)
        )

    angular_config = runtime_config["angular"]
    for degrees in range(0, 181, 30):
        angle = math.radians(degrees)
        rows = run_runtime_reference(
            executable, "angular", angle, angular_config, common
        )
        stem = f"angular_{degrees:03d}deg"
        write_csv(output_dir / f"{stem}.csv", rows)
        plot_profile(
            output_dir / f"{stem}.png",
            rows,
            f"Angular runtime speed reference: {degrees} deg",
            "rad",
            "rad/s",
            "rad/s²",
            "rad/s³",
            angular_config,
        )
        summaries.append(
            profile_summary("angular", angle, rows, angular_config)
        )

    summary_fields = (
        "mode",
        "motion",
        "duration",
        "points",
        "planned_peak_velocity",
        "profile_velocity_at_slowdown_point",
        "runtime_peak_target_velocity",
        "peak_acceleration",
        "peak_deceleration",
        "peak_jerk",
        "slowdown_motion",
        "configured_slowdown_speed",
        "effective_slowdown_speed",
        "slowdown_zone_points",
        "slowdown_applied_points",
    )
    with (output_dir / "summary.csv").open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=summary_fields)
        writer.writeheader()
        writer.writerows(summaries)

    print(f"planner_and_controller={executable}")
    print(f"params={params_file}")
    print(f"output={output_dir}")
    print("generated=17 PNG + 17 CSV + summary.csv")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (
        FileNotFoundError,
        RuntimeError,
        subprocess.CalledProcessError,
        yaml.YAMLError,
    ) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
