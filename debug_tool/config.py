"""将生产 YAML 读取为适合仿真的轻量数据类。"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Tuple

import yaml


DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "mobile_navigation/nav2_bringup/params/w1_nav2_params_diff.yaml"
)


def _value(data: Mapping[str, Any], key: str, default: Any) -> Any:
    value = data.get(key, default)
    return default if value is None else value


@dataclass
class RPPConfig:
    min_lookahead_dist: float = 0.3
    max_lookahead_dist: float = 0.9
    lookahead_time: float = 1.5
    use_velocity_scaled_lookahead_dist: bool = True
    use_interpolation: bool = True
    angular_gain: float = 1.2
    near_curvature_check_dist: float = 0.3
    traj_speed_lookahead: float = 0.1
    traj_time_lookahead: float = 0.0
    use_regulated_linear_velocity_scaling: bool = True
    regulated_linear_scaling_min_radius: float = 1.5
    regulated_linear_scaling_min_speed: float = 0.25
    use_cost_regulated_linear_velocity_scaling: bool = True
    cost_scaling_dist: float = 0.6
    cost_scaling_gain: float = 1.0
    inflation_cost_scaling_factor: float = 3.0
    lateral_error_threshold: float = 0.3
    lateral_error_max_dist: float = 0.3
    min_lateral_speed_scale: float = 0.5
    heading_error_scaling_min_angle: float = 0.35
    heading_error_scaling_max_angle: float = 0.26
    min_heading_speed_scale: float = 0.5
    max_angular_vel: float = 1.5
    desired_linear_vel: float = 0.8
    max_linear_accel: float = 0.5
    speed_loop_kp: float = 2.0
    speed_loop_ki: float = 0.2
    speed_loop_kd: float = 0.0
    speed_loop_integrator_saturation: float = 0.3
    speed_loop_cmd_reseed_error: float = 0.3
    use_rotate_to_heading: bool = True
    max_angular_accel: float = 0.5
    rotate_to_path_entry_angle: float = 0.26
    rotate_to_path_exit_angle: float = 0.1745
    rotate_to_path_angular_vel: float = 0.5
    rotate_to_path_angular_kp: float = 1.0
    use_collision_detection: bool = True
    max_allowed_time_to_collision_up_to_carrot: float = 1.0


@dataclass
class SpeedPlannerConfig:
    max_speed: float = 0.6
    min_speed: float = 0.05
    accel: float = 0.5
    decel: float = -0.4
    end_speed: float = 0.05
    stop_distance: float = 0.075
    collision_stop_distance: float = 0.2
    curve_compute_dist: float = 0.1
    regulated_linear_scaling_min_radius: float = 0.01
    min_curvature_speed: float = 0.1
    curve_lead_in_distance: float = 0.3
    curve_lead_out_distance: float = 0.3
    max_jerk: float = 0.5
    control_dt: float = 0.05


@dataclass
class SmootherConfig:
    frequency: float = 20.0
    max_linear_velocity: float = 0.8
    min_linear_velocity: float = -0.5
    max_angular_velocity: float = 0.75
    max_linear_accel: float = 0.75
    max_linear_decel: float = -0.75
    max_angular_accel: float = 1.0
    max_angular_decel: float = -1.0


@dataclass
class PIDConfig:
    kp: float = 1.0
    ki: float = 0.01
    kd: float = 0.1
    integrator_saturation: float = 0.3
    max_accel: float = 0.5
    cmd_reseed_error: float = 0.3


@dataclass
class CostmapConfig:
    cost_scaling_factor: float = 10.0
    inflation_radius: float = 0.35


@dataclass
class ToolConfig:
    source_path: Path = DEFAULT_CONFIG
    controller_frequency: float = 10.0
    rpp: RPPConfig = field(default_factory=RPPConfig)
    speed_planner: SpeedPlannerConfig = field(default_factory=SpeedPlannerConfig)
    smoother: SmootherConfig = field(default_factory=SmootherConfig)
    pid: PIDConfig = field(default_factory=PIDConfig)
    costmap: CostmapConfig = field(default_factory=CostmapConfig)

    @property
    def dt(self) -> float:
        return 1.0 / max(self.controller_frequency, 1e-6)

    def with_rpp(self, **updates: Any) -> "ToolConfig":
        return replace(self, rpp=replace(self.rpp, **updates))


@dataclass(frozen=True)
class ValidationIssue:
    severity: str
    parameter: str
    message: str


def _dataclass_from_mapping(cls: Any, values: Mapping[str, Any]) -> Any:
    defaults = cls()
    kwargs = {
        name: _value(values, name, getattr(defaults, name))
        for name in defaults.__dataclass_fields__
    }
    return cls(**kwargs)


def load_tool_config(path: Path | str = DEFAULT_CONFIG) -> ToolConfig:
    source = Path(path).expanduser().resolve()
    with source.open("r", encoding="utf-8") as stream:
        root = yaml.safe_load(stream) or {}

    server = root.get("controller_server", {}).get("ros__parameters", {})
    rpp_values = server.get("RPPController", {})
    planner_name = server.get("current_speed_planner", "SCurveSpeedPlanner")
    planner_values = server.get(planner_name, server.get("SCurveSpeedPlanner", {}))

    smoother_values = root.get("velocity_smoother", {}).get("ros__parameters", {})
    max_velocity = smoother_values.get("max_velocity", [0.8, 0.0, 0.75])
    min_velocity = smoother_values.get("min_velocity", [-0.5, 0.0, -0.75])
    max_accel = smoother_values.get("max_accel", [0.75, 0.0, 1.0])
    max_decel = smoother_values.get("max_decel", [-0.75, 0.0, -1.0])
    smoother = SmootherConfig(
        frequency=float(_value(smoother_values, "smoothing_frequency", 20.0)),
        max_linear_velocity=float(max_velocity[0]),
        min_linear_velocity=float(min_velocity[0]),
        max_angular_velocity=float(max_velocity[2]),
        max_linear_accel=float(max_accel[0]),
        max_linear_decel=float(max_decel[0]),
        max_angular_accel=float(max_accel[2]),
        max_angular_decel=float(max_decel[2]),
    )

    local = root.get("local_costmap", {}).get("local_costmap", {}).get("ros__parameters", {})
    inflation = local.get("inflation_layer", {})
    costmap = CostmapConfig(
        cost_scaling_factor=float(_value(inflation, "cost_scaling_factor", 10.0)),
        inflation_radius=float(_value(inflation, "inflation_radius", 0.35)),
    )

    pid_values = server.get("RPPController", {})
    pid = PIDConfig(
        kp=float(_value(pid_values, "speed_loop_kp", 2.0)),
        ki=float(_value(pid_values, "speed_loop_ki", 0.2)),
        kd=float(_value(pid_values, "speed_loop_kd", 0.0)),
        integrator_saturation=float(
            _value(pid_values, "speed_loop_integrator_saturation", 0.3)
        ),
        max_accel=float(_value(pid_values, "max_linear_accel", 0.5)),
        cmd_reseed_error=float(_value(pid_values, "speed_loop_cmd_reseed_error", 0.3)),
    )

    return ToolConfig(
        source_path=source,
        controller_frequency=float(_value(server, "controller_frequency", 10.0)),
        rpp=_dataclass_from_mapping(RPPConfig, rpp_values),
        speed_planner=_dataclass_from_mapping(SpeedPlannerConfig, planner_values),
        smoother=smoother,
        pid=pid,
        costmap=costmap,
    )


def validate_config(config: ToolConfig) -> List[ValidationIssue]:
    rpp = config.rpp
    issues: List[ValidationIssue] = []

    def add(severity: str, parameter: str, message: str) -> None:
        issues.append(ValidationIssue(severity, parameter, message))

    if rpp.min_lookahead_dist <= 0 or rpp.max_lookahead_dist < rpp.min_lookahead_dist:
        add("error", "lookahead", "前瞻距离必须为正，且最大前瞻距离必须大于等于最小前瞻距离。")
    if rpp.heading_error_scaling_min_angle >= rpp.heading_error_scaling_max_angle:
        add("error", "heading_error_scaling", "朝向连续降速区间无效：起始角必须小于结束角。")
    if rpp.heading_error_scaling_max_angle > rpp.rotate_to_path_entry_angle:
        add("warning", "heading/rotate", "朝向连续降速区尚未结束就会进入路径原地转向状态。")
    if rpp.rotate_to_path_exit_angle >= rpp.rotate_to_path_entry_angle:
        add("error", "rotate_to_path", "退出角必须小于进入角，才能形成有效滞回。")
    if rpp.lateral_error_threshold >= rpp.lateral_error_max_dist:
        add("error", "lateral_error", "横向误差降速没有连续过渡区：起始距离必须小于结束距离。")
    if abs(rpp.inflation_cost_scaling_factor - config.costmap.cost_scaling_factor) > 1e-6:
        add(
            "warning",
            "inflation_cost_scaling_factor",
            "RPP 障碍距离反推系数与局部代价地图膨胀层的代价缩放系数不一致。",
        )
    if rpp.max_angular_vel > config.smoother.max_angular_velocity + 1e-6:
        add(
            "warning",
            "max_angular_vel",
            "RPP 认为的角速度能力高于速度平滑器最终上限，可能低估弯道所需降速。",
        )
    if rpp.max_linear_accel > config.smoother.max_linear_accel + 1e-6:
        add("warning", "max_linear_accel", "RPP 线加速度上限高于下游速度平滑器能力。")
    if rpp.inflation_cost_scaling_factor <= 0:
        add("error", "inflation_cost_scaling_factor", "该值必须大于0。")
    return issues
