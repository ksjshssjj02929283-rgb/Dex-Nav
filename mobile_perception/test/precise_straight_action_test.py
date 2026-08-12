#!/usr/bin/env python3
"""直线精控 Action 测试、统计和可视化脚本。"""

import argparse
import csv
import json
import math
import sys
import time
from datetime import datetime
from html import escape
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_interfaces.action import PreciseControl
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

# ======================== 常用测试参数（可直接修改） ========================

# 单次直线运动距离，单位：米；正值前进，负值后退。
DEFAULT_DISTANCE_M = 0.5

# 测试模式：visual只测1次并生成曲线，stress重复测试且只写TXT汇总。
DEFAULT_TEST_MODE = "visual"

# 压测计划次数；只在stress模式生效，必须大于0。
DEFAULT_TEST_COUNT = 100

# 每轮停车观察完成后的额外等待时间，单位：秒。
DEFAULT_TEST_INTERVAL_S = 1.0

# 压测默认前后方向交替，避免机器人持续向一个方向离开测试区。
DEFAULT_ALTERNATE_DIRECTION = True

# 发送 Action 前的静止基线采集时间，单位：秒。
DEFAULT_BASELINE_DURATION_S = 1.0

# Action 完成后的继续采集时间，单位：秒，用于观察停车后是否回弹或继续过冲。
DEFAULT_SETTLE_DURATION_S = 1.5

# 单次直线运动最大等待时间，单位：秒。
DEFAULT_ACTION_TIMEOUT_S = 30.0

# TF与曲线采样频率，单位：Hz。
DEFAULT_SAMPLE_RATE_HZ = 50.0

# 报告格式：svg、png或both；PNG使用Pillow绘制，不依赖NumPy。
DEFAULT_REPORT_FORMAT = "both"

# 判断中途停顿的实测线速度阈值和持续时间。
DEFAULT_PAUSE_SPEED_M_S = 0.01
DEFAULT_PAUSE_DURATION_S = 0.30

# 剩余距离大于该值时出现低速才记为中途停顿，避免把正常收尾算成停顿。
DEFAULT_PAUSE_REMAINING_ERROR_M = 0.15

# 忽略动作开始后的短暂起步阶段，单位：秒。
DEFAULT_PAUSE_START_IGNORE_S = 0.50

# 报告统计使用的距离到位阈值，单位：米；只影响统计，不检查控制节点参数。
DEFAULT_ANALYSIS_DISTANCE_TOLERANCE_M = 0.02

# 统计线速度正负号切换时使用的死区，单位：m/s；死区内视为零，避免噪声误计数。
DEFAULT_SIGN_SWITCH_SPEED_DEADBAND_M_S = 0.01

# Action、速度、Odom和TF接口。
DEFAULT_CONTROL_ACTION = "/control/precise_control"
DEFAULT_COMMAND_TOPIC = "/control/cmd_vel"
DEFAULT_ODOM_TOPIC = "/feedback/odom"
# 测试位姿和最终误差使用的参考坐标系；应与控制节点control_reference_frame一致。
DEFAULT_CONTROL_REFERENCE_FRAME = "odom"
DEFAULT_BASE_FRAME = "base_link"

# 每次测试按启动时间创建独立结果目录。
DEFAULT_OUTPUT_ROOT = str(
    Path(__file__).resolve().parent / "results"
)

# =============================================================================


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}

CSV_FIELDS = [
    "time_s",
    "phase",
    "action_feedback",
    "target_distance_m",
    "tf_longitudinal_m",
    "tf_lateral_m",
    "tf_planar_distance_m",
    "remaining_error_m",
    "heading_change_deg",
    "cmd_vel_observed_linear_m_s",
    "cmd_vel_observed_angular_rad_s",
    "odom_linear_m_s",
    "odom_angular_rad_s",
]


def normalize_angle(angle):
    """把角度归一化到[-pi, pi)。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(quaternion):
    """从四元数计算平面yaw。"""
    return math.atan2(
        2.0 * (
            quaternion.w * quaternion.z +
            quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y +
            quaternion.z * quaternion.z
        ),
    )


def count_velocity_sign_switches(samples, field_name, deadband):
    """统计有效速度在正负方向之间切换的次数，零速不重置上一方向。"""
    previous_sign = 0
    switch_count = 0
    for sample in samples:
        velocity = sample[field_name]
        if not math.isfinite(velocity) or abs(velocity) <= deadband:
            continue
        current_sign = 1 if velocity > 0.0 else -1
        if previous_sign != 0 and current_sign != previous_sign:
            switch_count += 1
        previous_sign = current_sign
    return switch_count


def endpoint_node_name(info):
    """拼接ROS节点全名。"""
    namespace = info.node_namespace.rstrip("/")
    return (
        f"{namespace}/{info.node_name}"
        if namespace else f"/{info.node_name}"
    )


def finite_or_none(value):
    """JSON不写NaN，缺失数值统一保存为null。"""
    return value if value is not None and math.isfinite(value) else None


def svg_polyline(times, values, x_map, y_map, color):
    """把可能包含NaN的数据拆成多段SVG折线。"""
    elements = []
    points = []
    for sample_time, value in zip(times, values):
        if math.isfinite(value):
            points.append(
                f"{x_map(sample_time):.2f},{y_map(value):.2f}"
            )
        elif points:
            elements.append(
                f'<polyline points="{" ".join(points)}" fill="none" '
                f'stroke="{color}" stroke-width="2"/>'
            )
            points = []
    if points:
        elements.append(
            f'<polyline points="{" ".join(points)}" fill="none" '
            f'stroke="{color}" stroke-width="2"/>'
        )
    return elements


def format_axis_tick(value):
    """小量级刻度直接携带科学计数法，避免遗漏公共倍率。"""
    magnitude = abs(value)
    if magnitude != 0.0 and (magnitude < 1e-3 or magnitude >= 1e4):
        mantissa, exponent = f"{value:.1e}".split("e")
        return f"{mantissa}e{int(exponent)}"
    return f"{value:.4g}"


def nice_axis_ticks(values, target_intervals=5):
    """生成1、2、2.5、5倍数的易读坐标轴范围和刻度。"""
    finite_values = [
        value for value in values if math.isfinite(value)
    ]
    if not finite_values:
        return -1.0, 1.0, [-1.0, -0.5, 0.0, 0.5, 1.0]

    value_min = min(min(finite_values), 0.0)
    value_max = max(max(finite_values), 0.0)
    if value_max <= value_min:
        padding = max(abs(value_max) * 0.1, 1.0)
        value_min -= padding
        value_max += padding

    raw_step = (value_max - value_min) / max(1, target_intervals)
    magnitude = 10.0 ** math.floor(math.log10(raw_step))
    normalized_step = raw_step / magnitude
    step_multiplier = next(
        multiplier
        for multiplier in (1.0, 2.0, 2.5, 5.0, 10.0)
        if normalized_step <= multiplier
    )
    step = step_multiplier * magnitude
    axis_min = math.floor(value_min / step) * step
    axis_max = math.ceil(value_max / step) * step
    if axis_max <= axis_min:
        axis_max = axis_min + step

    interval_count = int(round((axis_max - axis_min) / step))
    ticks = [
        axis_min + index * step
        for index in range(interval_count + 1)
    ]
    # 消除浮点计算产生的-0刻度。
    ticks = [
        0.0 if abs(value) < step * 1e-9 else value
        for value in ticks
    ]
    return axis_min, axis_max, ticks


def svg_panel(x, y, width, height, title, unit, times, lines,
              profile_start, profile_end):
    """生成一个带坐标轴、图例和运动起止线的SVG子图。"""
    plot_left = x + 70
    plot_top = y + 40
    plot_width = width - 95
    plot_height = height - 85
    finite_values = [
        value
        for _, values, _ in lines
        for value in values
        if math.isfinite(value)
    ]
    time_min, time_max, time_ticks = nice_axis_ticks(times)
    value_min, value_max, value_ticks = nice_axis_ticks(finite_values)

    def x_map(value):
        return plot_left + (
            (value - time_min) / (time_max - time_min) * plot_width
        )

    def y_map(value):
        return plot_top + plot_height - (
            (value - value_min) / (value_max - value_min) * plot_height
        )

    elements = [
        f'<rect x="{x}" y="{y}" width="{width}" height="{height}" '
        'fill="white" stroke="#d0d7de" rx="6"/>',
        f'<text x="{x + width / 2:.1f}" y="{y + 24}" '
        'text-anchor="middle" class="panel-title">'
        f'{escape(title)}</text>',
    ]
    for time_label in time_ticks:
        grid_x = x_map(time_label)
        elements.extend([
            f'<line x1="{grid_x:.2f}" y1="{plot_top}" '
            f'x2="{grid_x:.2f}" y2="{plot_top + plot_height}" '
            'stroke="#eaeef2"/>',
            f'<text x="{grid_x:.2f}" '
            f'y="{plot_top + plot_height + 19}" '
            f'text-anchor="middle" class="tick">'
            f'{format_axis_tick(time_label)}</text>',
        ])
    for value_label in value_ticks:
        grid_y = y_map(value_label)
        elements.extend([
            f'<line x1="{plot_left}" y1="{grid_y:.2f}" '
            f'x2="{plot_left + plot_width}" y2="{grid_y:.2f}" '
            'stroke="#eaeef2"/>',
            f'<text x="{plot_left - 8}" y="{grid_y + 4:.2f}" '
            f'text-anchor="end" class="tick">'
            f'{format_axis_tick(value_label)}</text>',
        ])

    for marker_time in (profile_start, profile_end):
        if time_min <= marker_time <= time_max:
            marker_x = x_map(marker_time)
            elements.append(
                f'<line x1="{marker_x:.2f}" y1="{plot_top}" '
                f'x2="{marker_x:.2f}" '
                f'y2="{plot_top + plot_height}" '
                'stroke="#6e7781" stroke-width="1.5" '
                'stroke-dasharray="5 4"/>'
            )

    elements.extend([
        f'<line x1="{plot_left}" y1="{plot_top + plot_height}" '
        f'x2="{plot_left + plot_width}" '
        f'y2="{plot_top + plot_height}" stroke="#24292f"/>',
        f'<line x1="{plot_left}" y1="{plot_top}" '
        f'x2="{plot_left}" y2="{plot_top + plot_height}" '
        'stroke="#24292f"/>',
        f'<text x="{plot_left + plot_width / 2:.1f}" '
        f'y="{y + height - 8}" text-anchor="middle" class="axis">'
        '时间（s）</text>',
        f'<text x="{x + 15}" '
        f'y="{plot_top + plot_height / 2:.1f}" '
        f'text-anchor="middle" class="axis" '
        f'transform="rotate(-90 {x + 15} '
        f'{plot_top + plot_height / 2:.1f})">{escape(unit)}</text>',
    ])

    legend_x = plot_left + 5
    for name, values, color in lines:
        elements.extend(
            svg_polyline(times, values, x_map, y_map, color)
        )
        elements.extend([
            f'<line x1="{legend_x}" y1="{plot_top + 14}" '
            f'x2="{legend_x + 20}" y2="{plot_top + 14}" '
            f'stroke="{color}" stroke-width="3"/>',
            f'<text x="{legend_x + 25}" y="{plot_top + 18}" '
            f'class="legend">{escape(name)}</text>',
        ])
        legend_x += 118
    return elements


def build_parser():
    parser = argparse.ArgumentParser(
        description="测试直线精控Action并生成中文可视化报告",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--distance-m",
        type=float,
        default=DEFAULT_DISTANCE_M,
        help="直线运动距离，正值前进、负值后退",
    )
    parser.add_argument(
        "--mode",
        choices=("visual", "stress"),
        default=DEFAULT_TEST_MODE,
        help="visual单次可视化，stress多次压测并只生成TXT",
    )
    parser.add_argument(
        "--test-count",
        type=int,
        default=DEFAULT_TEST_COUNT,
        help="stress模式的测试次数",
    )
    parser.add_argument(
        "--test-interval",
        type=float,
        default=DEFAULT_TEST_INTERVAL_S,
        help="stress模式每轮之间的额外等待时间，单位s",
    )
    parser.add_argument(
        "--same-direction",
        action="store_true",
        help="stress模式始终按请求方向运动；默认前后交替",
    )
    parser.add_argument(
        "--baseline-duration",
        type=float,
        default=DEFAULT_BASELINE_DURATION_S,
        help="动作前静止基线采集时间，单位s",
    )
    parser.add_argument(
        "--settle-duration",
        type=float,
        default=DEFAULT_SETTLE_DURATION_S,
        help="动作完成后继续采集时间，单位s",
    )
    parser.add_argument(
        "--action-timeout",
        type=float,
        default=DEFAULT_ACTION_TIMEOUT_S,
        help="Action超时时间，单位s",
    )
    parser.add_argument(
        "--sample-rate",
        type=float,
        default=DEFAULT_SAMPLE_RATE_HZ,
        help="TF和报告曲线采样频率，单位Hz",
    )
    parser.add_argument(
        "--control-reference-frame",
        default=DEFAULT_CONTROL_REFERENCE_FRAME,
        help="测试位姿和最终误差使用的TF参考坐标系",
    )
    parser.add_argument(
        "--output-root",
        default=DEFAULT_OUTPUT_ROOT,
        help="测试结果根目录",
    )
    parser.add_argument(
        "--report-format",
        choices=("svg", "png", "both"),
        default=DEFAULT_REPORT_FORMAT,
        help="可视化报告格式；PNG需要Pillow和中文字体",
    )
    return parser


def parse_args():
    ros_free_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = build_parser().parse_args(ros_free_args)
    numeric_values = {
        "distance_m": args.distance_m,
        "test_interval": args.test_interval,
        "baseline_duration": args.baseline_duration,
        "settle_duration": args.settle_duration,
        "action_timeout": args.action_timeout,
        "sample_rate": args.sample_rate,
    }
    for name, value in numeric_values.items():
        if not math.isfinite(value):
            raise ValueError(f"{name}必须是有限数值")
    if not 0.05 <= abs(args.distance_m) <= 20.0:
        raise ValueError("distance_m绝对值必须在0.05～20.0米之间")
    if args.baseline_duration < 0.0 or args.settle_duration < 0.0:
        raise ValueError("baseline_duration和settle_duration不能小于0")
    if args.test_count <= 0:
        raise ValueError("test_count必须大于0")
    if args.test_interval < 0.0:
        raise ValueError("test_interval不能小于0")
    if args.action_timeout <= 0.0 or args.sample_rate <= 0.0:
        raise ValueError("action_timeout和sample_rate必须大于0")
    args.control_reference_frame = args.control_reference_frame.strip()
    if not args.control_reference_frame:
        raise ValueError("control_reference_frame不能为空")
    args.alternate_direction = (
        DEFAULT_ALTERNATE_DIRECTION and not args.same_direction
    )
    return args


class PreciseStraightActionTest(Node):
    """执行一次闭环直线运动并采集控制命令、Odom和TF。"""

    def __init__(self, args):
        node_name = getattr(
            args, "node_name", "precise_straight_action_test"
        )
        super().__init__(node_name)
        self.args = args
        self.test_started = time.monotonic()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.output_dir = (
            Path(args.output_root).expanduser().resolve() /
            f"{timestamp}_直线运动_{args.distance_m:+.3f}米"
        )

        self.action_client = ActionClient(
            self, PreciseControl, DEFAULT_CONTROL_ACTION
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.command_sub = self.create_subscription(
            Twist,
            DEFAULT_COMMAND_TOPIC,
            self.command_callback,
            100,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            DEFAULT_ODOM_TOPIC,
            self.odom_callback,
            100,
        )
        self.sample_timer = self.create_timer(
            1.0 / args.sample_rate,
            self.sample_callback,
        )

        self.samples = []
        self.publisher_names = []
        self.latest_cmd_linear_observed = math.nan
        self.latest_cmd_angular_observed = math.nan
        self.latest_odom_angular = math.nan
        self.latest_odom_linear = math.nan
        self.latest_odom_time = None
        self.current_reference_x = None
        self.current_reference_y = None
        self.current_reference_yaw = None
        self.initial_reference_x = None
        self.initial_reference_y = None
        self.initial_reference_yaw = None
        self.latest_longitudinal = math.nan
        self.latest_lateral = math.nan
        self.latest_planar_distance = math.nan
        self.latest_heading_change = math.nan
        self.tf_failure_count = 0
        self.action_started_s = None
        self.action_finished_s = None
        self.action_feedback = "等待"
        self.action_status = "NOT_STARTED"
        self.action_success = False
        self.finish_reason = ""
        self.active_goal_handle = None
        self.active_result_future = None

    def elapsed(self):
        return time.monotonic() - self.test_started

    def current_phase(self):
        if self.action_started_s is None:
            return "基线"
        if self.action_finished_s is None:
            return "直行"
        return "停车观察"

    def command_callback(self, msg):
        # ROS 2 Humble的rclpy执行器只向订阅回调传入消息本体，不提供发布者信息。
        # 同一话题有多个发布者时，这里记录的是按接收顺序合并后的观测值。
        self.latest_cmd_linear_observed = msg.linear.x
        self.latest_cmd_angular_observed = msg.angular.z

    def odom_callback(self, msg):
        self.latest_odom_angular = msg.twist.twist.angular.z
        self.latest_odom_linear = msg.twist.twist.linear.x
        self.latest_odom_time = time.monotonic()

    def feedback_callback(self, feedback_msg):
        self.action_feedback = getattr(
            feedback_msg.feedback, "current_status", "CONTROL"
        )

    def sample_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.args.control_reference_frame,
                DEFAULT_BASE_FRAME,
                Time(),
            )
            self.current_reference_x = transform.transform.translation.x
            self.current_reference_y = transform.transform.translation.y
            self.current_reference_yaw = quaternion_to_yaw(
                transform.transform.rotation
            )
            if self.initial_reference_x is not None:
                dx = self.current_reference_x - self.initial_reference_x
                dy = self.current_reference_y - self.initial_reference_y
                cos_yaw = math.cos(self.initial_reference_yaw)
                sin_yaw = math.sin(self.initial_reference_yaw)
                # 纵向、横向均投影到Action开始时的base_link坐标系。
                self.latest_longitudinal = dx * cos_yaw + dy * sin_yaw
                self.latest_lateral = -dx * sin_yaw + dy * cos_yaw
                self.latest_planar_distance = math.hypot(dx, dy)
                self.latest_heading_change = normalize_angle(
                    self.current_reference_yaw -
                    self.initial_reference_yaw
                )
        except Exception:
            self.tf_failure_count += 1

        target_distance = self.args.distance_m
        remaining_error = (
            target_distance - self.latest_longitudinal
            if math.isfinite(self.latest_longitudinal) else math.nan
        )
        self.samples.append({
            "time_s": self.elapsed(),
            "phase": self.current_phase(),
            "action_feedback": self.action_feedback,
            "target_distance_m": target_distance,
            "tf_longitudinal_m": self.latest_longitudinal,
            "tf_lateral_m": self.latest_lateral,
            "tf_planar_distance_m": self.latest_planar_distance,
            "remaining_error_m": remaining_error,
            "heading_change_deg": math.degrees(
                self.latest_heading_change
            ),
            "cmd_vel_observed_linear_m_s":
                self.latest_cmd_linear_observed,
            "cmd_vel_observed_angular_rad_s":
                self.latest_cmd_angular_observed,
            "odom_linear_m_s": self.latest_odom_linear,
            "odom_angular_rad_s": self.latest_odom_angular,
        })

    def spin_for(self, duration_s):
        deadline = time.monotonic() + max(0.0, duration_s)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(
                self,
                timeout_sec=min(0.05, deadline - time.monotonic()),
            )
        return rclpy.ok()

    def wait_future(self, future, timeout_s):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if future.done():
                return True
            if (
                self.action_started_s is not None and
                (
                    self.latest_odom_time is None or
                    time.monotonic() - self.latest_odom_time > 1.0
                )
            ):
                self.finish_reason = "Odom超过1秒没有更新"
                return False
        if not self.finish_reason:
            self.finish_reason = "等待超时"
        return False

    def check_command_publishers(self):
        publishers = self.get_publishers_info_by_topic(
            DEFAULT_COMMAND_TOPIC
        )
        self.publisher_names = [
            endpoint_node_name(info) for info in publishers
        ]
        if not publishers:
            self.get_logger().error(
                f"{DEFAULT_COMMAND_TOPIC}没有发布者。"
            )
            return False
        return True

    def reset_tf_reference(self):
        """以当前参考系位姿建立本次直线测试的纵向和横向坐标轴。"""
        self.initial_reference_x = self.current_reference_x
        self.initial_reference_y = self.current_reference_y
        self.initial_reference_yaw = self.current_reference_yaw
        self.latest_longitudinal = 0.0
        self.latest_lateral = 0.0
        self.latest_planar_distance = 0.0
        self.latest_heading_change = 0.0

    def initialize_tf_reference(self):
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.current_reference_x is not None:
                self.reset_tf_reference()
                return True
        self.get_logger().error(
            "无法获取"
            f"{self.args.control_reference_frame} -> "
            f"{DEFAULT_BASE_FRAME} TF。"
        )
        return False

    def runtime_check(self):
        self.get_logger().info(
            "等待直控Action、Odom和TF；"
            f"误差参考系={self.args.control_reference_frame}。"
        )
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f"找不到直控Action：{DEFAULT_CONTROL_ACTION}。"
            )
            return False
        if not self.check_command_publishers():
            return False
        if not self.initialize_tf_reference():
            return False

        odom_deadline = time.monotonic() + 3.0
        while rclpy.ok() and self.latest_odom_time is None:
            if time.monotonic() >= odom_deadline:
                self.get_logger().error(
                    f"没有收到{DEFAULT_ODOM_TOPIC}。"
                )
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
        return True

    def make_goal(self):
        goal = PreciseControl.Goal()
        pose = PoseStamped()
        pose.header.frame_id = DEFAULT_BASE_FRAME
        pose.pose.position.x = self.args.distance_m
        pose.pose.orientation.w = 1.0
        goal.target_pose = pose
        return goal

    def cancel_active_goal(self):
        if self.active_goal_handle is None or not rclpy.ok():
            return
        self.get_logger().warning("正在取消直线运动并等待平滑停车。")
        cancel_future = self.active_goal_handle.cancel_goal_async()
        self.wait_future(cancel_future, 5.0)
        if self.active_result_future is not None:
            self.wait_future(self.active_result_future, 8.0)
        self.active_goal_handle = None
        self.active_result_future = None

    def execute_action(self):
        if self.current_reference_x is None:
            self.action_status = "TF_INVALID"
            self.finish_reason = "发送目标前无法获取TF位置基准"
            return False

        # 基线采集只用于确认底盘静止；实际位移从Action发送前重新计零。
        self.reset_tf_reference()
        self.action_started_s = self.elapsed()
        self.action_feedback = "发送目标"
        send_future = self.action_client.send_goal_async(
            self.make_goal(),
            feedback_callback=self.feedback_callback,
        )
        if not self.wait_future(send_future, 5.0):
            self.action_status = "GOAL_RESPONSE_TIMEOUT"
            return False

        self.active_goal_handle = send_future.result()
        if not self.active_goal_handle.accepted:
            self.action_status = "REJECTED"
            self.finish_reason = "直控目标被拒绝"
            return False

        self.get_logger().info(
            f"开始直线运动：目标={self.args.distance_m:+.3f}m。"
        )
        self.active_result_future = (
            self.active_goal_handle.get_result_async()
        )
        if not self.wait_future(
            self.active_result_future,
            self.args.action_timeout,
        ):
            self.action_status = "TIMEOUT"
            self.cancel_active_goal()
            return False

        response = self.active_result_future.result()
        self.action_status = STATUS_NAMES.get(
            response.status, f"UNKNOWN({response.status})"
        )
        result_success = bool(
            getattr(response.result, "success", False)
        )
        self.action_success = (
            response.status == GoalStatus.STATUS_SUCCEEDED and
            result_success
        )
        self.finish_reason = getattr(response.result, "message", "")
        self.active_goal_handle = None
        self.active_result_future = None
        return self.action_success

    def run(self):
        if self.args.mode == "visual":
            self.get_logger().info(
                f"本次数据目录：{self.output_dir}"
            )
        if not self.runtime_check():
            self.finish_reason = self.finish_reason or "运行检查失败"
            return False

        self.get_logger().info(
            f"采集{self.args.baseline_duration:.1f}s静止基线。"
        )
        if not self.spin_for(self.args.baseline_duration):
            self.finish_reason = "ROS关闭"
            return False
        if (
            abs(self.latest_odom_linear) > 0.02 or
            abs(self.latest_odom_angular) > 0.02
        ):
            self.finish_reason = "基线阶段底盘仍在运动"
            self.get_logger().error(
                "基线阶段底盘仍在运动，测试停止。"
            )
            return False

        success = self.execute_action()
        self.action_finished_s = self.elapsed()
        self.action_feedback = self.action_status
        action_duration = (
            self.action_finished_s - self.action_started_s
            if self.action_started_s is not None else math.nan
        )
        duration_text = (
            f"{action_duration:.2f}s"
            if math.isfinite(action_duration) else "N/A"
        )
        self.get_logger().info(
            f"直线运动结束：状态={self.action_status}，"
            f"耗时={duration_text}。"
        )
        self.spin_for(self.args.settle_duration)
        return success


def calculate_pause_metrics(node, target_distance):
    """统计距离目标尚远时Odom线速度持续接近零的中途停顿。"""
    pause_start = None
    durations = []
    ignore_before = (
        (node.action_started_s or 0.0) +
        DEFAULT_PAUSE_START_IGNORE_S
    )
    error_limit = DEFAULT_PAUSE_REMAINING_ERROR_M
    for sample in node.samples:
        time_s = sample["time_s"]
        actual = sample["tf_longitudinal_m"]
        odom = sample["odom_linear_m_s"]
        if not math.isfinite(actual) or not math.isfinite(odom):
            is_pause = False
        else:
            remaining_error = target_distance - actual
            is_pause = (
                sample["phase"] == "直行" and
                time_s >= ignore_before and
                abs(remaining_error) > error_limit and
                abs(odom) < DEFAULT_PAUSE_SPEED_M_S
            )
        if is_pause and pause_start is None:
            pause_start = time_s
        elif not is_pause and pause_start is not None:
            duration = time_s - pause_start
            if duration >= DEFAULT_PAUSE_DURATION_S:
                durations.append(duration)
            pause_start = None
    if pause_start is not None and node.samples:
        duration = node.samples[-1]["time_s"] - pause_start
        if duration >= DEFAULT_PAUSE_DURATION_S:
            durations.append(duration)
    return len(durations), max(durations, default=0.0)


def calculate_summary(node):
    target_distance = node.args.distance_m
    valid_longitudinal = [
        sample["tf_longitudinal_m"]
        for sample in node.samples
        if math.isfinite(sample["tf_longitudinal_m"])
    ]
    control_samples = [
        sample for sample in node.samples
        if sample["phase"] == "直行"
    ]
    final_longitudinal = (
        valid_longitudinal[-1] if valid_longitudinal else math.nan
    )
    final_sample = node.samples[-1] if node.samples else {}
    final_lateral = final_sample.get("tf_lateral_m", math.nan)
    final_planar = final_sample.get("tf_planar_distance_m", math.nan)
    final_heading = final_sample.get("heading_change_deg", math.nan)
    direction = 1.0 if target_distance >= 0.0 else -1.0
    directional_progress = [
        direction * value for value in valid_longitudinal
    ]
    overshoot = max(
        0.0,
        max(directional_progress, default=0.0) - abs(target_distance),
    )
    max_lateral = max(
        (
            abs(sample["tf_lateral_m"])
            for sample in control_samples
            if math.isfinite(sample["tf_lateral_m"])
        ),
        default=math.nan,
    )

    peak_cmd_vel_observed = max(
        (
            abs(sample["cmd_vel_observed_linear_m_s"])
            for sample in control_samples
            if math.isfinite(
                sample["cmd_vel_observed_linear_m_s"]
            )
        ),
        default=math.nan,
    )
    peak_odom = max(
        (
            abs(sample["odom_linear_m_s"])
            for sample in control_samples
            if math.isfinite(sample["odom_linear_m_s"])
        ),
        default=math.nan,
    )

    def directional_rates(field_name):
        """计算沿目标运动方向的速度变化率，正值加速、负值减速。"""
        previous = None
        rates = []
        for sample in control_samples:
            current = sample[field_name]
            if not math.isfinite(current):
                continue
            directional_velocity = direction * current
            if previous is not None:
                dt = sample["time_s"] - previous[0]
                if dt > 1e-6:
                    rates.append(
                        (directional_velocity - previous[1]) / dt
                    )
            previous = (sample["time_s"], directional_velocity)
        return rates

    cmd_rates = directional_rates("cmd_vel_observed_linear_m_s")
    odom_rates = directional_rates("odom_linear_m_s")
    max_cmd_accel = max(
        (rate for rate in cmd_rates if rate > 0.0), default=math.nan
    )
    max_cmd_decel = max(
        (-rate for rate in cmd_rates if rate < 0.0), default=math.nan
    )
    max_odom_decel = max(
        (-rate for rate in odom_rates if rate < 0.0), default=math.nan
    )

    cmd_vel_sign_switch_count = count_velocity_sign_switches(
        control_samples,
        "cmd_vel_observed_linear_m_s",
        DEFAULT_SIGN_SWITCH_SPEED_DEADBAND_M_S,
    )

    first_tolerance_time = None
    xy_tolerance = DEFAULT_ANALYSIS_DISTANCE_TOLERANCE_M
    for sample in control_samples:
        actual = sample["tf_longitudinal_m"]
        if not math.isfinite(actual):
            continue
        if abs(target_distance - actual) <= xy_tolerance:
            first_tolerance_time = sample["time_s"]
            break
    settle_time = (
        node.action_finished_s - first_tolerance_time
        if (
            first_tolerance_time is not None and
            node.action_finished_s is not None
        ) else math.nan
    )
    pause_count, max_pause_duration = calculate_pause_metrics(
        node, target_distance
    )

    return {
        "action_success": node.action_success,
        "action_status": node.action_status,
        "finish_reason": node.finish_reason,
        "control_reference_frame": node.args.control_reference_frame,
        "target_distance_m": target_distance,
        "final_longitudinal_m": finite_or_none(final_longitudinal),
        "final_lateral_m": finite_or_none(final_lateral),
        "final_planar_distance_m": finite_or_none(final_planar),
        "final_heading_change_deg": finite_or_none(final_heading),
        "final_error_m": finite_or_none(
            target_distance - final_longitudinal
        ),
        "max_overshoot_m": overshoot,
        "max_lateral_drift_m": finite_or_none(max_lateral),
        "peak_cmd_vel_observed_m_s": finite_or_none(
            peak_cmd_vel_observed
        ),
        "peak_odom_linear_m_s": finite_or_none(peak_odom),
        "max_cmd_vel_observed_accel_m_s2": finite_or_none(
            max_cmd_accel
        ),
        "max_cmd_vel_observed_decel_m_s2": finite_or_none(
            max_cmd_decel
        ),
        "max_odom_decel_m_s2": finite_or_none(max_odom_decel),
        "cmd_vel_sign_switch_count": cmd_vel_sign_switch_count,
        "cmd_vel_sign_switch_deadband_m_s":
            DEFAULT_SIGN_SWITCH_SPEED_DEADBAND_M_S,
        "action_duration_s": (
            node.action_finished_s - node.action_started_s
            if (
                node.action_started_s is not None and
                node.action_finished_s is not None
            ) else None
        ),
        "settle_confirmation_time_s": finite_or_none(settle_time),
        "pause_count": pause_count,
        "max_pause_duration_s": max_pause_duration,
        "analysis_distance_tolerance_m":
            DEFAULT_ANALYSIS_DISTANCE_TOLERANCE_M,
        "tf_failure_count": node.tf_failure_count,
        "command_publishers": node.publisher_names,
    }


def write_csv(node, summary, path):
    target_distance = summary["target_distance_m"]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for sample in node.samples:
            row = dict(sample)
            row["target_distance_m"] = target_distance
            actual = row["tf_longitudinal_m"]
            row["remaining_error_m"] = (
                target_distance - actual
                if math.isfinite(actual) else math.nan
            )
            writer.writerow(row)


def write_text_report(summary, path):
    def value(name, unit="", digits=4):
        number = summary.get(name)
        if number is None:
            return "N/A"
        return f"{number:.{digits}f}{unit}"

    lines = [
        "直线精控测试结果",
        "=" * 50,
        f"Action状态：{summary['action_status']}",
        f"误差参考坐标系：{summary['control_reference_frame']}",
        f"目标距离：{summary['target_distance_m']:+.3f}m",
        f"TF最终纵向位移：{value('final_longitudinal_m', 'm')}",
        f"TF最终横向偏移：{value('final_lateral_m', 'm')}",
        f"TF最终平面位移：{value('final_planar_distance_m', 'm')}",
        f"最终航向变化：{value('final_heading_change_deg', '°')}",
        f"最终距离误差：{value('final_error_m', 'm')}",
        f"最大纵向过冲：{value('max_overshoot_m', 'm')}",
        f"最大横向漂移：{value('max_lateral_drift_m', 'm')}",
        f"Action耗时：{value('action_duration_s', 's', 3)}",
        "进入距离阈值后的停稳确认时间："
        f"{value('settle_confirmation_time_s', 's', 3)}",
        "cmd_vel观测线速度峰值："
        f"{value('peak_cmd_vel_observed_m_s', 'm/s')}",
        "Odom线速度峰值："
        f"{value('peak_odom_linear_m_s', 'm/s')}",
        "cmd_vel观测最大线加速度："
        f"{value('max_cmd_vel_observed_accel_m_s2', 'm/s²')}",
        "cmd_vel观测最大线减速度："
        f"{value('max_cmd_vel_observed_decel_m_s2', 'm/s²')}",
        "Odom观测最大线减速度："
        f"{value('max_odom_decel_m_s2', 'm/s²')}",
        "cmd_vel观测线速度正负切换次数："
        f"{summary['cmd_vel_sign_switch_count']}",
        f"检测到的中途停顿次数：{summary['pause_count']}",
        "最长中途停顿："
        f"{value('max_pause_duration_s', 's', 3)}",
        f"TF采样失败次数：{summary['tf_failure_count']}",
        "",
        "指标说明",
        "-" * 50,
        "报告距离到位阈值："
        f"{DEFAULT_ANALYSIS_DISTANCE_TOLERANCE_M:.3f}m；"
        "仅用于统计，不读取或校验控制节点参数。",
        "纵向和横向：均投影到Action开始时的base_link坐标轴。",
        "最大纵向过冲：沿目标运动方向超过目标距离的最大值。",
        "加减速度：按采样速度差分得到，Odom结果可能包含测量噪声。",
        "线速度正负切换：仅统计直行阶段的cmd_vel.linear.x；"
        f"绝对值不超过{DEFAULT_SIGN_SWITCH_SPEED_DEADBAND_M_S:.3f}m/s时视为零。",
        "中途停顿：动作开始0.5秒后、剩余距离仍大于0.15m时，",
        "Odom线速度连续至少0.3秒低于0.01m/s。",
        "ROS 2 Humble不向Python订阅回调提供发布者身份；cmd_vel存在",
        "多个发布者时，命令指标和曲线是所有消息按接收顺序合并的观测结果。",
    ]
    if summary["finish_reason"]:
        lines.insert(3, f"结果信息：{summary['finish_reason']}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_svg_report(node, summary, path):
    times = [sample["time_s"] for sample in node.samples]
    target = [
        summary["target_distance_m"] for _ in node.samples
    ]
    actual = [
        sample["tf_longitudinal_m"] for sample in node.samples
    ]
    error = [
        target_value - actual_value
        if math.isfinite(actual_value) else math.nan
        for target_value, actual_value in zip(target, actual)
    ]
    cmd_vel_observed = [
        sample["cmd_vel_observed_linear_m_s"]
        for sample in node.samples
    ]
    odom = [
        sample["odom_linear_m_s"] for sample in node.samples
    ]
    lateral = [
        sample["tf_lateral_m"] for sample in node.samples
    ]
    action_start = node.action_started_s or 0.0
    action_end = (
        node.action_finished_s
        if node.action_finished_s is not None
        else (times[-1] if times else action_start)
    )

    width = 1450
    height = 1445
    panel_height = 285
    colors = {
        "target": "#0969da",
        "actual": "#1a7f37",
        "cmd": "#cf222e",
        "odom": "#bf8700",
        "error": "#cf222e",
        "lateral": "#8250df",
    }
    elements = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
        f'height="{height}" viewBox="0 0 {width} {height}">',
        "<style>"
        'text{font-family:"Noto Sans CJK SC","Source Han Sans SC",'
        '"Microsoft YaHei","WenQuanYi Micro Hei",SimHei,sans-serif;'
        "fill:#24292f}"
        ".title{font-size:24px;font-weight:600}"
        ".subtitle{font-size:15px;fill:#57606a}"
        ".metric{font-size:14px}"
        ".panel-title{font-size:17px;font-weight:600}"
        ".tick{font-size:11px;fill:#57606a}"
        ".axis{font-size:12px;fill:#57606a}"
        ".legend{font-size:11px}"
        ".footer{font-size:13px;fill:#57606a}"
        "</style>",
        '<rect width="100%" height="100%" fill="#f6f8fa"/>',
        f'<text x="{width / 2}" y="34" text-anchor="middle" '
        'class="title">直线精控测试报告</text>',
        f'<text x="{width / 2}" y="59" text-anchor="middle" '
        f'class="subtitle">目标={summary["target_distance_m"]:+.3f}m，'
        f'参考系={escape(summary["control_reference_frame"])}，'
        f'Action={escape(summary["action_status"])}</text>',
        '<text x="40" y="91" class="metric">'
        f'最终误差：{summary["final_error_m"] if summary["final_error_m"] is not None else "N/A"}m'
        "</text>",
        '<text x="380" y="91" class="metric">'
        f'最大过冲：{summary["max_overshoot_m"]:.4f}m</text>',
        '<text x="700" y="91" class="metric">'
        f'速度正负切换：{summary["cmd_vel_sign_switch_count"]}次</text>',
        '<text x="980" y="91" class="metric">'
        f'Action耗时：{summary["action_duration_s"] if summary["action_duration_s"] is not None else "N/A"}s'
        "</text>",
    ]

    panels = [
        (
            "纵向距离响应",
            "距离（m）",
            [
                ("目标距离", target, colors["target"]),
                ("TF实际纵向位移", actual, colors["actual"]),
            ],
        ),
        (
            "线速度响应",
            "线速度（m/s）",
            [
                ("cmd_vel话题观测", cmd_vel_observed, colors["cmd"]),
                ("Odom实测", odom, colors["odom"]),
            ],
        ),
        (
            "剩余距离误差",
            "误差（m）",
            [
                ("目标－TF实际", error, colors["error"]),
            ],
        ),
        (
            "横向漂移",
            "偏移（m）",
            [
                ("TF横向偏移", lateral, colors["lateral"]),
            ],
        ),
    ]
    for index, (title, unit, lines) in enumerate(panels):
        elements.extend(svg_panel(
            25,
            115 + index * 315,
            1400,
            panel_height,
            title,
            unit,
            times,
            lines,
            action_start,
            action_end,
        ))
    elements.extend([
        f'<text x="25" y="{height - 20}" class="footer">'
        "竖向虚线：Action开始／结束；详细指标见report.txt"
        "</text>",
        "</svg>",
    ])
    path.write_text("\n".join(elements), encoding="utf-8")


def write_png_report(node, summary, path):
    """使用Pillow直接绘制PNG，避开Matplotlib与NumPy版本冲突。"""
    from PIL import Image, ImageDraw, ImageFont

    font_candidates = [
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc",
        "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    ]
    font_path = next(
        (
            candidate
            for candidate in font_candidates
            if Path(candidate).is_file()
        ),
        None,
    )
    if font_path is None:
        raise RuntimeError(
            "未检测到可用中文字体；为避免PNG中文显示成方框，"
            "本次仅生成SVG报告"
        )

    fonts = {
        "title": ImageFont.truetype(font_path, 34),
        "subtitle": ImageFont.truetype(font_path, 23),
        "panel": ImageFont.truetype(font_path, 24),
        "label": ImageFont.truetype(font_path, 18),
        "tick": ImageFont.truetype(font_path, 15),
    }
    width = 1800
    height = 1875
    image = Image.new("RGB", (width, height), "#f6f8fa")
    draw = ImageDraw.Draw(image)

    def centered_text(y, text, font, fill="#24292f"):
        box = draw.textbbox((0, 0), text, font=font)
        text_width = box[2] - box[0]
        draw.text(
            ((width - text_width) / 2, y),
            text,
            font=font,
            fill=fill,
        )

    centered_text(24, "直线精控测试报告", fonts["title"])
    centered_text(
        72,
        f"目标={summary['target_distance_m']:+.3f}m，"
        f"参考系={summary['control_reference_frame']}，"
        f"Action={summary['action_status']}",
        fonts["subtitle"],
        "#57606a",
    )
    samples = node.samples
    times = [sample["time_s"] for sample in samples]
    target = [
        summary["target_distance_m"] for _ in samples
    ]
    actual = [
        sample["tf_longitudinal_m"] for sample in samples
    ]
    error = [
        target_value - actual_value
        if math.isfinite(actual_value) else math.nan
        for target_value, actual_value in zip(target, actual)
    ]
    cmd_vel_observed = [
        sample["cmd_vel_observed_linear_m_s"]
        for sample in samples
    ]
    odom = [
        sample["odom_linear_m_s"] for sample in samples
    ]
    lateral = [
        sample["tf_lateral_m"] for sample in samples
    ]
    action_start = node.action_started_s or 0.0
    action_end = (
        node.action_finished_s
        if node.action_finished_s is not None
        else (times[-1] if times else action_start)
    )

    def draw_dashed_vertical(x, top, bottom, color):
        dash_length = 9
        gap = 7
        y = top
        while y < bottom:
            draw.line(
                [(x, y), (x, min(y + dash_length, bottom))],
                fill=color,
                width=2,
            )
            y += dash_length + gap

    def draw_panel(y, title, unit, series):
        panel_x = 35
        panel_width = width - 70
        panel_height = 370
        plot_left = panel_x + 105
        plot_top = y + 65
        plot_right = panel_x + panel_width - 35
        plot_bottom = y + panel_height - 55
        draw.rounded_rectangle(
            (
                panel_x,
                y,
                panel_x + panel_width,
                y + panel_height,
            ),
            radius=10,
            fill="white",
            outline="#d0d7de",
            width=2,
        )
        centered_panel_box = draw.textbbox(
            (0, 0), title, font=fonts["panel"]
        )
        centered_panel_width = (
            centered_panel_box[2] - centered_panel_box[0]
        )
        draw.text(
            ((width - centered_panel_width) / 2, y + 20),
            title,
            font=fonts["panel"],
            fill="#24292f",
        )

        finite_values = [
            value
            for _, values, _ in series
            for value in values
            if math.isfinite(value)
        ]
        time_min, time_max, time_ticks = nice_axis_ticks(times)
        value_min, value_max, value_ticks = nice_axis_ticks(
            finite_values
        )

        def x_map(value):
            return plot_left + (
                (value - time_min) / (time_max - time_min) *
                (plot_right - plot_left)
            )

        def y_map(value):
            return plot_bottom - (
                (value - value_min) / (value_max - value_min) *
                (plot_bottom - plot_top)
            )

        for time_label in time_ticks:
            grid_x = x_map(time_label)
            draw.line(
                [(grid_x, plot_top), (grid_x, plot_bottom)],
                fill="#eaeef2",
                width=1,
            )
            label = format_axis_tick(time_label)
            label_box = draw.textbbox(
                (0, 0), label, font=fonts["tick"]
            )
            label_width = label_box[2] - label_box[0]
            draw.text(
                (grid_x - label_width / 2, plot_bottom + 9),
                label,
                font=fonts["tick"],
                fill="#57606a",
            )
        for value_label in value_ticks:
            grid_y = y_map(value_label)
            draw.line(
                [(plot_left, grid_y), (plot_right, grid_y)],
                fill="#eaeef2",
                width=1,
            )
            label = format_axis_tick(value_label)
            label_box = draw.textbbox(
                (0, 0), label, font=fonts["tick"]
            )
            label_width = label_box[2] - label_box[0]
            draw.text(
                (plot_left - label_width - 12, grid_y - 10),
                label,
                font=fonts["tick"],
                fill="#57606a",
            )

        for marker_time, color in (
            (action_start, "#6e7781"),
            (action_end, "#24292f"),
        ):
            if time_min <= marker_time <= time_max:
                draw_dashed_vertical(
                    x_map(marker_time),
                    plot_top,
                    plot_bottom,
                    color,
                )

        draw.line(
            [(plot_left, plot_bottom), (plot_right, plot_bottom)],
            fill="#24292f",
            width=2,
        )
        draw.line(
            [(plot_left, plot_top), (plot_left, plot_bottom)],
            fill="#24292f",
            width=2,
        )
        draw.text(
            (panel_x + 8, y + 42),
            unit,
            font=fonts["tick"],
            fill="#57606a",
        )
        draw.text(
            ((plot_left + plot_right) / 2 - 35, plot_bottom + 28),
            "时间（s）",
            font=fonts["tick"],
            fill="#57606a",
        )

        legend_x = plot_left + 10
        for name, values, color in series:
            points = []
            for time_value, value in zip(times, values):
                if math.isfinite(value):
                    points.append((x_map(time_value), y_map(value)))
                elif len(points) >= 2:
                    draw.line(points, fill=color, width=3)
                    points = []
                else:
                    points = []
            if len(points) >= 2:
                draw.line(points, fill=color, width=3)
            elif len(points) == 1:
                x_value, y_value = points[0]
                draw.ellipse(
                    (
                        x_value - 2,
                        y_value - 2,
                        x_value + 2,
                        y_value + 2,
                    ),
                    fill=color,
                )
            draw.line(
                [(legend_x, plot_top + 12), (legend_x + 30, plot_top + 12)],
                fill=color,
                width=4,
            )
            draw.text(
                (legend_x + 38, plot_top),
                name,
                font=fonts["tick"],
                fill="#24292f",
            )
            legend_box = draw.textbbox(
                (0, 0), name, font=fonts["tick"]
            )
            legend_x += 80 + legend_box[2] - legend_box[0]

    panels = [
        (
            "纵向距离响应",
            "距离（m）",
            [
                ("目标距离", target, "#0969da"),
                ("TF实际纵向位移", actual, "#1a7f37"),
            ],
        ),
        (
            "线速度响应",
            "线速度（m/s）",
            [
                ("cmd_vel话题观测", cmd_vel_observed, "#cf222e"),
                ("Odom实测", odom, "#bf8700"),
            ],
        ),
        (
            "剩余距离误差",
            "误差（m）",
            [
                ("目标－TF实际", error, "#cf222e"),
            ],
        ),
        (
            "横向漂移",
            "偏移（m）",
            [
                ("TF横向偏移", lateral, "#8250df"),
            ],
        ),
    ]
    for index, (title, unit, series) in enumerate(panels):
        draw_panel(125 + index * 395, title, unit, series)

    final_error = summary["final_error_m"]
    final_error_text = (
        f"{final_error:.4f}m"
        if final_error is not None else "N/A"
    )
    final_lateral = summary["final_lateral_m"]
    final_lateral_text = (
        f"{final_lateral:.4f}m"
        if final_lateral is not None else "N/A"
    )
    duration = summary["action_duration_s"]
    duration_text = (
        f"{duration:.3f}s" if duration is not None else "N/A"
    )
    draw.text(
        (35, height - 54),
        f"最终误差={final_error_text}；"
        f"最大过冲={summary['max_overshoot_m']:.4f}m；"
        f"最终横向偏移={final_lateral_text}；"
        f"速度正负切换={summary['cmd_vel_sign_switch_count']}次；"
        f"Action耗时={duration_text}",
        font=fonts["label"],
        fill="#57606a",
    )
    image.save(path, format="PNG", optimize=True)


def save_results(node):
    node.output_dir.mkdir(parents=True, exist_ok=False)
    summary = calculate_summary(node)
    data_path = node.output_dir / "data.csv"
    summary_path = node.output_dir / "summary.json"
    text_path = node.output_dir / "report.txt"
    svg_path = node.output_dir / "report.svg"
    png_path = node.output_dir / "report.png"
    write_csv(node, summary, data_path)
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    write_text_report(summary, text_path)
    report_paths = []
    if node.args.report_format in ("svg", "both"):
        write_svg_report(node, summary, svg_path)
        report_paths.append(svg_path)
    if node.args.report_format in ("png", "both"):
        try:
            write_png_report(node, summary, png_path)
            report_paths.append(png_path)
        except Exception as error:
            node.get_logger().warning(
                f"PNG报告生成失败：{error}；SVG报告仍可使用。"
            )
    if not report_paths:
        write_svg_report(node, summary, svg_path)
        report_paths.append(svg_path)
    return data_path, summary_path, text_path, report_paths


def stress_metric(value, digits, unit=""):
    """格式化压测逐轮指标，无效数据统一显示为N/A。"""
    if value is None or not math.isfinite(value):
        return "N/A"
    return f"{value:.{digits}f}{unit}"


def straight_stress_line(index, total, summary):
    """生成单轮直线运动压测的中文汇总行。"""
    success_text = "成功" if summary["action_success"] else "失败"
    duration = stress_metric(summary["action_duration_s"], 3, "s")
    error = stress_metric(summary["final_error_m"], 4, "m")
    lateral = stress_metric(summary["final_lateral_m"], 4, "m")
    overshoot = stress_metric(summary["max_overshoot_m"], 4, "m")
    finish_reason = summary["finish_reason"].replace("\n", " ") or "-"
    return (
        f"第{index:03d}/{total:03d}次："
        f"目标={summary['target_distance_m']:+.3f}m，"
        f"结果={success_text}，状态={summary['action_status']}，"
        f"耗时={duration}，最终误差={error}，"
        f"横向偏移={lateral}，最大过冲={overshoot}，"
        f"速度正负切换={summary['cmd_vel_sign_switch_count']}次，"
        f"信息={finish_reason}"
    )


def write_straight_stress_footer(report_path, summaries, planned_count):
    """追加压测整体统计，中断时也保留已完成结果。"""
    success_count = sum(
        1 for summary in summaries if summary["action_success"]
    )
    durations = [
        summary["action_duration_s"]
        for summary in summaries
        if summary["action_duration_s"] is not None
        and math.isfinite(summary["action_duration_s"])
    ]
    errors = [
        abs(summary["final_error_m"])
        for summary in summaries
        if summary["final_error_m"] is not None
        and math.isfinite(summary["final_error_m"])
    ]
    lateral_offsets = [
        abs(summary["final_lateral_m"])
        for summary in summaries
        if summary["final_lateral_m"] is not None
        and math.isfinite(summary["final_lateral_m"])
    ]
    average_duration = (
        sum(durations) / len(durations) if durations else None
    )
    average_error = sum(errors) / len(errors) if errors else None
    maximum_error = max(errors) if errors else None
    average_lateral = (
        sum(lateral_offsets) / len(lateral_offsets)
        if lateral_offsets else None
    )
    sign_switch_counts = [
        summary["cmd_vel_sign_switch_count"] for summary in summaries
    ]
    rounds_with_sign_switch = sum(
        1 for count in sign_switch_counts if count > 0
    )
    lines = [
        "",
        "整体统计",
        "-" * 72,
        f"计划次数：{planned_count}",
        f"实际完成：{len(summaries)}",
        f"成功/失败：{success_count}/{len(summaries) - success_count}",
        f"平均耗时：{stress_metric(average_duration, 3, 's')}",
        f"平均绝对误差：{stress_metric(average_error, 4, 'm')}",
        f"最大绝对误差：{stress_metric(maximum_error, 4, 'm')}",
        f"平均绝对横向偏移：{stress_metric(average_lateral, 4, 'm')}",
        f"发生速度正负切换的轮次：{rounds_with_sign_switch}",
        f"速度正负切换总次数：{sum(sign_switch_counts)}",
        "单轮最大速度正负切换次数："
        f"{max(sign_switch_counts, default=0)}",
    ]
    with report_path.open("a", encoding="utf-8") as stream:
        stream.write("\n".join(lines) + "\n")


def run_visual_mode(args):
    """执行一次直线运动并保存完整可视化结果。"""
    node = PreciseStraightActionTest(args)
    exit_code = 1
    try:
        exit_code = 0 if node.run() else 1
    except KeyboardInterrupt:
        node.finish_reason = "用户中断"
        node.action_status = "INTERRUPTED"
        node.cancel_active_goal()
        node.action_finished_s = node.elapsed()
        node.spin_for(0.5)
    except ExternalShutdownException:
        node.finish_reason = "ROS关闭"
        node.action_status = "ROS_SHUTDOWN"
    except Exception as error:
        node.finish_reason = str(error)
        node.action_status = "EXCEPTION"
        node.get_logger().error(f"测试异常：{error}")
        # 任何采集或报告异常都必须先取消Action，不能让服务端继续运动。
        node.cancel_active_goal()
        node.action_finished_s = node.elapsed()
        node.spin_for(0.5)
        exit_code = 1
    finally:
        _, _, _, report_paths = save_results(node)
        node.get_logger().info(
            f"结果已保存：{node.output_dir}"
        )
        node.get_logger().info(
            "可视化报告：" +
            "，".join(str(path) for path in report_paths)
        )
        node.destroy_node()
    return exit_code


def run_stress_mode(args):
    """重复执行直线运动，逐轮输出并写入单个TXT汇总。"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    session_dir = (
        Path(args.output_root).expanduser().resolve() /
        f"{timestamp}_直线运动压测"
    )
    session_dir.mkdir(parents=True, exist_ok=False)
    report_path = session_dir / "stress_report.txt"
    direction_text = (
        "前后目标交替" if args.alternate_direction
        else "始终按请求方向"
    )
    report_path.write_text(
        "直线运动精控压测结果\n"
        + "=" * 72 + "\n"
        + f"计划次数：{args.test_count}\n"
        + f"基础目标：{args.distance_m:+.3f}m\n"
        + f"方向策略：{direction_text}\n"
        + f"误差参考坐标系：{args.control_reference_frame}\n"
        + "速度正负切换死区："
        + f"{DEFAULT_SIGN_SWITCH_SPEED_DEADBAND_M_S:.3f}m/s\n"
        + f"轮间等待：{args.test_interval:.2f}s\n\n",
        encoding="utf-8",
    )
    print(f"压测结果：{report_path}")

    summaries = []
    base_distance = args.distance_m
    interrupted = False
    for index in range(1, args.test_count + 1):
        round_args = argparse.Namespace(**vars(args))
        # 每轮使用唯一节点名，避免rosout尚未完成注销时
        # 新节点重用同名称，产生Publisher already registered警告。
        round_args.node_name = (
            f"precise_straight_stress_{timestamp}_{index:03d}"
        )
        if args.alternate_direction and index % 2 == 0:
            round_args.distance_m = -base_distance
        node = None
        stop_after_round = False
        try:
            node = PreciseStraightActionTest(round_args)
            node.get_logger().info(
                f"压测进度：第{index}/{args.test_count}次，"
                f"目标={round_args.distance_m:+.3f}m。"
            )
            node.run()
        except KeyboardInterrupt:
            interrupted = True
            stop_after_round = True
            if node is not None:
                node.finish_reason = "用户中断"
                node.action_status = "INTERRUPTED"
                node.cancel_active_goal()
                node.action_finished_s = node.elapsed()
                node.spin_for(0.5)
        except ExternalShutdownException:
            stop_after_round = True
            if node is not None:
                node.finish_reason = "ROS关闭"
                node.action_status = "ROS_SHUTDOWN"
        except Exception as error:
            stop_after_round = True
            if node is not None:
                node.finish_reason = str(error)
                node.action_status = "EXCEPTION"
                node.get_logger().error(f"测试异常：{error}")
                node.cancel_active_goal()
                node.action_finished_s = node.elapsed()
                node.spin_for(0.5)
            else:
                print(f"第{index}次创建测试节点失败：{error}")
        finally:
            if node is not None:
                summary = calculate_summary(node)
                summaries.append(summary)
                line = straight_stress_line(
                    index, args.test_count, summary
                )
                print(line)
                with report_path.open("a", encoding="utf-8") as stream:
                    stream.write(line + "\n")
                node.destroy_node()

        if stop_after_round or not rclpy.ok():
            break
        if index < args.test_count and args.test_interval > 0.0:
            try:
                time.sleep(args.test_interval)
            except KeyboardInterrupt:
                interrupted = True
                break

    write_straight_stress_footer(
        report_path, summaries, args.test_count
    )
    if interrupted:
        print("用户中断压测，已保存完成轮次的结果。")
    print(f"压测汇总已保存：{report_path}")
    all_succeeded = (
        len(summaries) == args.test_count
        and all(summary["action_success"] for summary in summaries)
    )
    return 0 if all_succeeded else 1


def main():
    try:
        args = parse_args()
    except (ValueError, SystemExit) as error:
        if isinstance(error, SystemExit):
            raise
        print(f"参数错误：{error}", file=sys.stderr)
        return 2

    rclpy.init(
        args=sys.argv,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    try:
        if args.mode == "stress":
            return run_stress_mode(args)
        return run_visual_mode(args)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
