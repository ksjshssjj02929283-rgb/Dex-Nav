#!/usr/bin/env python3
"""可直接执行的底盘运动精度测试、统计和可视化脚本。"""

import argparse
import contextlib
import csv
import io
import json
import math
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from html import escape
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


# ======================== 常用测试参数（可直接修改） ========================
DEFAULT_WAVEFORM = 'step'       # 速度曲线：step阶跃、trapezoid梯形、sine正弦
DEFAULT_LINEAR = 0.0            # 阶跃目标或梯形/正弦峰值，单位 m/s；正值前进
DEFAULT_ANGULAR = 0.2           # 阶跃目标或梯形/正弦峰值，单位 rad/s；正值左转
DEFAULT_DURATION = 10.0         # 阶跃或正弦曲线持续时间，单位 s
DEFAULT_LINEAR_ACCEL = 0.1      # 梯形线加速度绝对值，单位 m/s²；0退化为阶跃
DEFAULT_ANGULAR_ACCEL = 0.3     # 梯形角加速度绝对值，单位 rad/s²；0退化为阶跃
DEFAULT_LINEAR_HOLD = 1.0       # 梯形线速度到达峰值后的保持时间，单位 s
DEFAULT_ANGULAR_HOLD = 1.0      # 梯形角速度到达峰值后的保持时间，单位 s
DEFAULT_SINE_FREQUENCY = 0.2    # 正弦频率，单位 Hz；0.2表示每5秒完成一个周期
DEFAULT_RATE = 20.0             # 每秒发布的命令数量；默认20 Hz，与精控Action一致
DEFAULT_STARTUP_WAIT = 1.0      # 启动连通检查等待时间，单位 s；必须大于0
DEFAULT_BASELINE = 0.0          # 正式运动前的零速基线采集时间，单位 s
DEFAULT_SETTLE = 1.0            # 速度归零后的继续采集时间，单位 s
DEFAULT_REPORT_FORMAT = 'both'  # 报告格式：svg、png或both

COMMAND_INPUT_TOPIC = '/control/cmd_vel_nav'       # 测试速度输入话题
FINAL_COMMAND_TOPIC = '/control/cmd_vel'           # 底盘最终速度命令话题
ODOM_TOPIC = '/feedback/odom'                      # 里程计反馈话题
WHEEL_TOPIC = '/feedback/wheel_velocity'           # 轮速反馈话题，可不存在
DEFAULT_OUTPUT_ROOT = str(                         # 测试结果默认保存目录
    Path(__file__).resolve().parent / 'results'
)


@dataclass
class TestConfig:
    """命令行解析后的完整测试配置。"""

    test_name: str
    description: str
    waveform: str
    linear: float
    angular: float
    duration: float
    linear_accel: float
    angular_accel: float
    linear_hold: float
    angular_hold: float
    sine_frequency: float
    rate: float
    startup_wait: float
    baseline: float
    settle: float
    output_root: str
    report_format: str
    allow_multiple_publishers: bool

    def profile_duration(self):
        if self.waveform != 'trapezoid':
            return self.duration

        linear_time = (
            2.0 * abs(self.linear) / self.linear_accel + self.linear_hold
            if self.linear_accel > 0.0 else self.linear_hold
        )
        angular_time = (
            2.0 * abs(self.angular) / self.angular_accel + self.angular_hold
            if self.angular_accel > 0.0 else self.angular_hold
        )
        return max(linear_time, angular_time)


def build_parser():
    parser = argparse.ArgumentParser(
        description='统一执行阶跃、梯形或正弦速度底盘运动精度测试',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        '--waveform',
        choices=('step', 'trapezoid', 'sine'),
        default=DEFAULT_WAVEFORM,
        help='速度曲线类型：阶跃、梯形或正弦',
    )
    parser.add_argument('--linear', type=float, default=DEFAULT_LINEAR,
                        help='阶跃目标或梯形/正弦峰值，单位 m/s')
    parser.add_argument('--angular', type=float, default=DEFAULT_ANGULAR,
                        help='阶跃目标或梯形/正弦峰值，单位 rad/s')
    parser.add_argument('--duration', type=float, default=DEFAULT_DURATION,
                        help='阶跃或正弦速度曲线持续时间，单位 s')
    parser.add_argument('--linear-accel', type=float,
                        default=DEFAULT_LINEAR_ACCEL,
                        help='梯形线速度加速度绝对值，0 表示阶跃')
    parser.add_argument('--angular-accel', type=float,
                        default=DEFAULT_ANGULAR_ACCEL,
                        help='梯形角速度加速度绝对值，0 表示阶跃')
    parser.add_argument('--linear-hold', type=float,
                        default=DEFAULT_LINEAR_HOLD,
                        help='梯形线速度匀速段时间，单位 s')
    parser.add_argument('--angular-hold', type=float,
                        default=DEFAULT_ANGULAR_HOLD,
                        help='梯形角速度匀速段时间，单位 s')
    parser.add_argument('--sine-frequency', type=float,
                        default=DEFAULT_SINE_FREQUENCY,
                        help='正弦频率，单位 Hz；0.2 表示周期为 5 s')
    parser.add_argument('--rate', type=float, default=DEFAULT_RATE,
                        help='每秒发布的速度命令数量，单位 Hz')
    parser.add_argument('--startup-wait', type=float,
                        default=DEFAULT_STARTUP_WAIT,
                        help='正式检查前持续发送零速度的等待时间，单位 s')
    parser.add_argument('--baseline', type=float, default=DEFAULT_BASELINE,
                        help='正式运动前的零速基线采集时间，单位 s')
    parser.add_argument('--settle', type=float, default=DEFAULT_SETTLE,
                        help='速度归零后的继续采集时间，单位 s')
    parser.add_argument(
        '--output-root',
        default=DEFAULT_OUTPUT_ROOT,
        help='测试结果根目录',
    )
    parser.add_argument(
        '--report-format',
        choices=('svg', 'png', 'both'),
        default=DEFAULT_REPORT_FORMAT,
        help='可视化报告格式；PNG 需要实机 Python 环境支持 matplotlib',
    )
    parser.add_argument(
        '--allow-multiple-publishers',
        action='store_true',
        help=argparse.SUPPRESS,
    )
    return parser


def parse_config():
    ros_free_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = build_parser().parse_args(ros_free_args)

    if args.waveform not in ('step', 'trapezoid', 'sine'):
        raise ValueError(
            'waveform 只能是 step、trapezoid 或 sine'
        )
    if args.report_format not in ('svg', 'png', 'both'):
        raise ValueError(
            'report_format 只能是 svg、png 或 both'
        )

    positive_values = {
        'duration': args.duration,
        'rate': args.rate,
        'startup_wait': args.startup_wait,
        'baseline': args.baseline,
        'settle': args.settle,
        'linear_hold': args.linear_hold,
        'angular_hold': args.angular_hold,
    }
    numeric_values = {
        **positive_values,
        'linear': args.linear,
        'angular': args.angular,
        'linear_accel': args.linear_accel,
        'angular_accel': args.angular_accel,
        'sine_frequency': args.sine_frequency,
    }
    for name, value in numeric_values.items():
        if not math.isfinite(value):
            raise ValueError(f'{name} 必须是有限数值')

    for name, value in positive_values.items():
        if value < 0.0 or (
            name in ('duration', 'rate', 'startup_wait') and value == 0.0
        ):
            raise ValueError(
                'duration、rate、startup_wait 必须大于 0，'
                '其他时间参数必须大于等于 0'
            )
    if args.linear_accel < 0.0 or args.angular_accel < 0.0:
        raise ValueError('加速度参数必须大于等于 0，方向由目标速度决定')
    if args.waveform == 'sine' and args.sine_frequency <= 0.0:
        raise ValueError('sine_frequency 必须大于 0')

    descriptions = {
        'step': '阶跃速度运动精度测试',
        'trapezoid': '梯形速度运动精度测试',
        'sine': '正弦速度运动精度测试',
    }
    return TestConfig(
        test_name=f'{args.waveform}_velocity',
        description=descriptions[args.waveform],
        waveform=args.waveform,
        linear=args.linear,
        angular=args.angular,
        duration=args.duration,
        linear_accel=args.linear_accel,
        angular_accel=args.angular_accel,
        linear_hold=args.linear_hold,
        angular_hold=args.angular_hold,
        sine_frequency=args.sine_frequency,
        rate=args.rate,
        startup_wait=args.startup_wait,
        baseline=args.baseline,
        settle=args.settle,
        output_root=args.output_root,
        report_format=args.report_format,
        allow_multiple_publishers=args.allow_multiple_publishers,
    )


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def endpoint_node_name(info):
    """拼接 ROS 节点全名，避免根命名空间显示成双斜杠。"""
    namespace = info.node_namespace.rstrip('/')
    return f'{namespace}/{info.node_name}' if namespace else f'/{info.node_name}'


def trapezoid_value(t, target, acceleration, hold):
    """计算带方向的梯形速度；加速度为 0 时退化为阶跃。"""
    magnitude = abs(target)
    if magnitude == 0.0:
        return 0.0
    sign = 1.0 if target >= 0.0 else -1.0
    if acceleration <= 0.0:
        return target if 0.0 <= t < hold else 0.0

    acceleration_time = magnitude / acceleration
    total_time = 2.0 * acceleration_time + hold
    if t < 0.0:
        value = 0.0
    elif t < acceleration_time:
        value = acceleration * t
    elif t < acceleration_time + hold:
        value = magnitude
    elif t < total_time:
        value = magnitude - acceleration * (
            t - acceleration_time - hold
        )
    else:
        value = 0.0
    return sign * max(0.0, value)


def sine_value(t, amplitude, frequency):
    """计算从零开始的正弦速度，amplitude 表示正负峰值。"""
    if t < 0.0:
        return 0.0
    return amplitude * math.sin(2.0 * math.pi * frequency * t)


class ChassisMotionTest(Node):
    """统一执行速度曲线，并采集最终命令、里程计和轮速反馈。"""

    def __init__(self, config):
        super().__init__(f'chassis_motion_test_{config.test_name}')
        self.config = config
        # 测试速度先进入 cmd_vel_mux，再从最终速度话题采集实际输出。
        self.cmd_pub = self.create_publisher(
            Twist, COMMAND_INPUT_TOPIC, 10
        )
        self.cmd_sub = self.create_subscription(
            Twist, FINAL_COMMAND_TOPIC, self.cmd_callback, 50
        )
        self.odom_sub = self.create_subscription(
            Odometry, ODOM_TOPIC, self.odom_callback, 50
        )
        self.wheel_sub = self.create_subscription(
            Twist, WHEEL_TOPIC, self.wheel_callback, 50
        )

        self.start_monotonic = time.monotonic()
        self.profile_duration = config.profile_duration()
        self.profile_start = config.startup_wait + config.baseline
        self.total_duration = (
            self.profile_start + self.profile_duration + config.settle
        )
        self.safety_checked = False
        self.finished = False
        self.finish_reason = ''
        self.samples = []

        self.latest_cmd_linear = math.nan
        self.latest_cmd_angular = math.nan
        self.latest_cmd_time = None
        self.baseline_nonzero_detected = False
        self.baseline_nonzero_linear = 0.0
        self.baseline_nonzero_angular = 0.0
        self.command_mismatch_window = []
        self.target_history = []
        self.latest_wheel_left = math.nan
        self.latest_wheel_right = math.nan
        self.latest_wheel_linear = math.nan
        self.latest_wheel_angular = math.nan
        self.latest_wheel_time = None

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.output_dir = (
            Path(config.output_root).expanduser().resolve()
            / f'{timestamp}_{config.test_name}'
        )
        self.timer = self.create_timer(1.0 / config.rate, self.timer_callback)

        self.get_logger().info(
            f'测试启动：{config.description}；'
            f'启动等待 {config.startup_wait:.1f}s，'
            f'基线 {config.baseline:.1f}s，运动 {self.profile_duration:.2f}s，'
            f'停车采集 {config.settle:.1f}s'
        )
        self.get_logger().info(
            f'结果将保存到：{self.output_dir}'
        )
        self.get_logger().info(
            f'速度链路：{COMMAND_INPUT_TOPIC} → cmd_vel_mux → '
            f'{FINAL_COMMAND_TOPIC}'
        )

    def elapsed(self):
        return time.monotonic() - self.start_monotonic

    def phase_and_target(self, elapsed):
        if elapsed < self.config.startup_wait:
            return 'startup', 0.0, 0.0
        if elapsed < self.profile_start:
            return 'baseline', 0.0, 0.0

        profile_time = elapsed - self.profile_start
        if profile_time < self.profile_duration:
            if self.config.waveform == 'step':
                return (
                    'motion',
                    self.config.linear,
                    self.config.angular,
                )
            if self.config.waveform == 'trapezoid':
                return (
                    'motion',
                    trapezoid_value(
                        profile_time,
                        self.config.linear,
                        self.config.linear_accel,
                        self.config.linear_hold,
                    ),
                    trapezoid_value(
                        profile_time,
                        self.config.angular,
                        self.config.angular_accel,
                        self.config.angular_hold,
                    ),
                )
            return (
                'motion',
                sine_value(
                    profile_time,
                    self.config.linear,
                    self.config.sine_frequency,
                ),
                sine_value(
                    profile_time,
                    self.config.angular,
                    self.config.sine_frequency,
                ),
            )
        return 'settle', 0.0, 0.0

    def check_runtime_safety(self):
        odom_publishers = self.get_publishers_info_by_topic(ODOM_TOPIC)
        if not odom_publishers:
            self.get_logger().error(
                f'{ODOM_TOPIC} 没有发布者，无法进行精度测试。'
            )
            return False

        if self.cmd_pub.get_subscription_count() == 0:
            self.get_logger().error(
                f'{COMMAND_INPUT_TOPIC} 没有订阅者；cmd_vel_mux 可能未启动，'
                '测试速度无法进入底盘最终命令链路。'
            )
            return False

        final_publishers = self.get_publishers_info_by_topic(
            FINAL_COMMAND_TOPIC
        )
        if not final_publishers:
            self.get_logger().error(
                f'{FINAL_COMMAND_TOPIC} 没有发布者，无法采集最终速度命令。'
            )
            return False

        if self.latest_cmd_time is None:
            self.get_logger().error(
                f'基线阶段没有收到 {FINAL_COMMAND_TOPIC} 消息；'
                '请检查 cmd_vel_mux 的订阅配置和话题 remap。'
            )
            return False

        if self.baseline_nonzero_detected:
            self.get_logger().error(
                f'基线阶段检测到非零最终速度命令：'
                f'linear.x={self.baseline_nonzero_linear:.6f} m/s，'
                f'angular.z={self.baseline_nonzero_angular:.6f} rad/s。'
                '说明导航、遥控或精控仍在控制底盘，测试已中止。'
            )
            return False

        input_publishers = self.get_publishers_info_by_topic(
            COMMAND_INPUT_TOPIC
        )
        external_input_publishers = [
            info for info in input_publishers
            if info.node_name != self.get_name()
        ]
        if external_input_publishers:
            names = ', '.join(
                endpoint_node_name(info)
                for info in external_input_publishers
            )
            self.get_logger().warning(
                f'{COMMAND_INPUT_TOPIC} 还存在其他发布者：{names}。'
                '当前基线最终命令为零，允许继续；运动阶段若最终命令'
                '偏离测试目标，脚本会立即停止。'
            )

        final_names = ', '.join(
            endpoint_node_name(info)
            for info in final_publishers
        )
        self.get_logger().info(
            f'{FINAL_COMMAND_TOPIC} 发布者：{final_names}；'
            '基线最终命令检查通过。'
        )

        if not self.get_publishers_info_by_topic(WHEEL_TOPIC):
            self.get_logger().warning(
                f'{WHEEL_TOPIC} 没有发布者；轮速曲线将留空，'
                '但最终命令与里程计测试可继续。'
            )
        return True

    def timer_callback(self):
        if self.finished:
            return

        elapsed = self.elapsed()
        if elapsed >= self.profile_start and not self.safety_checked:
            self.safety_checked = True
            if not self.check_runtime_safety():
                self.finalize('safety_check_failed')
                return
            self.get_logger().info('运行检查通过，开始发送测试速度。')

        _, target_linear, target_angular = self.phase_and_target(elapsed)
        msg = Twist()
        msg.linear.x = target_linear
        msg.angular.z = target_angular
        self.cmd_pub.publish(msg)
        self.target_history.append(
            (elapsed, target_linear, target_angular)
        )
        self.target_history = [
            target
            for target in self.target_history
            if elapsed - target[0] <= 0.5
        ]

        if elapsed >= self.total_duration:
            self.finalize('completed')

    def cmd_callback(self, msg):
        now = time.monotonic()
        elapsed = now - self.start_monotonic
        self.latest_cmd_linear = msg.linear.x
        self.latest_cmd_angular = msg.angular.z
        self.latest_cmd_time = now

        if elapsed < self.profile_start:
            if abs(msg.linear.x) > 1e-4 or abs(msg.angular.z) > 1e-4:
                self.baseline_nonzero_detected = True
                self.baseline_nonzero_linear = msg.linear.x
                self.baseline_nonzero_angular = msg.angular.z
            return

        phase, target_linear, target_angular = self.phase_and_target(elapsed)
        # 运动开始后留出 0.3 s 给 mux 和回调链路完成切换。
        if (
            phase != 'motion'
            or elapsed < self.profile_start + 0.3
            or self.finished
        ):
            self.command_mismatch_window.clear()
            return

        # 最终命令经过 mux 后会有少量延迟。动态曲线必须与最近实际发布过的
        # 目标比较，否则正弦过零和斜坡阶段容易被误判为其他节点抢占。
        recent_targets = [
            target
            for target in self.target_history
            if 0.0 <= elapsed - target[0] <= 0.3
        ]
        if not recent_targets:
            recent_targets = [(elapsed, target_linear, target_angular)]

        command_mismatch = True
        for _, recent_linear, recent_angular in recent_targets:
            linear_tolerance = max(
                0.0002, 0.25 * abs(recent_linear)
            )
            angular_tolerance = max(
                0.0005, 0.25 * abs(recent_angular)
            )
            if (
                abs(msg.linear.x - recent_linear) <= linear_tolerance
                and abs(msg.angular.z - recent_angular) <= angular_tolerance
            ):
                command_mismatch = False
                break
        self.command_mismatch_window.append(command_mismatch)
        self.command_mismatch_window = self.command_mismatch_window[-10:]

        # 最近 10 条最终命令中有至少 5 条不匹配时停止。这样即使两个
        # 发布者的消息交替出现，也不会被中间的正确消息掩盖。
        if sum(self.command_mismatch_window) >= 5:
            self.get_logger().error(
                '最终速度命令连续偏离测试目标，可能被导航、遥控或'
                '精控命令抢占：'
                f'target=({target_linear:.6f}, {target_angular:.6f})，'
                f'final=({msg.linear.x:.6f}, {msg.angular.z:.6f})。'
                '正在停止测试。'
            )
            self.finalize('final_command_conflict')

    def wheel_callback(self, msg):
        self.latest_wheel_left = msg.linear.x
        self.latest_wheel_right = msg.linear.y
        self.latest_wheel_linear = msg.linear.z
        self.latest_wheel_angular = msg.angular.z
        self.latest_wheel_time = time.monotonic()

    def odom_callback(self, msg):
        now = time.monotonic()
        elapsed = now - self.start_monotonic
        phase, target_linear, target_angular = self.phase_and_target(elapsed)
        cmd_age = (
            now - self.latest_cmd_time
            if self.latest_cmd_time is not None else math.nan
        )
        wheel_age = (
            now - self.latest_wheel_time
            if self.latest_wheel_time is not None else math.nan
        )

        self.samples.append({
            'time': elapsed,
            'phase': phase,
            'target_lin': target_linear,
            'target_ang': target_angular,
            'cmd_lin': self.latest_cmd_linear,
            'cmd_ang': self.latest_cmd_angular,
            'cmd_age': cmd_age,
            'odom_lin': msg.twist.twist.linear.x,
            'odom_ang': msg.twist.twist.angular.z,
            'wheel_left': self.latest_wheel_left,
            'wheel_right': self.latest_wheel_right,
            'wheel_lin': self.latest_wheel_linear,
            'wheel_ang': self.latest_wheel_angular,
            'wheel_age': wheel_age,
            'odom_x': msg.pose.pose.position.x,
            'odom_y': msg.pose.pose.position.y,
            'odom_yaw': quaternion_to_yaw(msg.pose.pose.orientation),
        })

    def publish_stop(self):
        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            # 让零速度在 shutdown 前有时间经过 DDS 和 cmd_vel_mux。
            time.sleep(0.02)

    def finalize(self, reason):
        if self.finished:
            return
        self.finished = True
        self.finish_reason = reason
        self.timer.cancel()
        if rclpy.ok():
            try:
                self.publish_stop()
            except Exception as exc:
                self.get_logger().error(f'发送停止命令失败：{exc}')

        try:
            data_path, summary_path, text_path, report_paths = save_results(
                self
            )
            self.get_logger().info(f'原始数据：{data_path}')
            self.get_logger().info(f'统计结果：{summary_path}')
            self.get_logger().info(f'文本报告：{text_path}')
            for report_path in report_paths:
                self.get_logger().info(f'可视化报告：{report_path}')
        except Exception as exc:
            self.get_logger().error(f'保存测试结果失败：{exc}')
        finally:
            if reason == 'completed':
                self.get_logger().info('底盘运动精度测试完成。')
            else:
                self.get_logger().warning(f'测试提前结束：{reason}')


CSV_FIELDS = [
    'time', 'phase',
    'target_lin', 'target_ang',
    'cmd_lin', 'cmd_ang', 'cmd_age',
    'odom_lin', 'odom_ang',
    'wheel_left', 'wheel_right',
    'wheel_lin', 'wheel_ang', 'wheel_age',
    'odom_x', 'odom_y', 'odom_yaw',
]


def integrate_series(times, values):
    """使用梯形积分法计算每个采样时刻的累计量。"""
    if not times:
        return []
    result = [0.0]
    for index in range(1, len(times)):
        dt = max(0.0, times[index] - times[index - 1])
        previous = values[index - 1]
        current = values[index]
        if not math.isfinite(previous) or not math.isfinite(current):
            result.append(result[-1])
        else:
            result.append(result[-1] + 0.5 * (previous + current) * dt)
    return result


def error_metrics(reference, measured, active_mask):
    errors = [
        measured[index] - reference[index]
        for index in range(len(reference))
        if active_mask[index]
        and math.isfinite(reference[index])
        and math.isfinite(measured[index])
    ]
    if not errors:
        return {
            'sample_count': 0,
            'bias': None,
            'mae': None,
            'rmse': None,
            'max_abs_error': None,
        }
    return {
        'sample_count': len(errors),
        'bias': sum(errors) / len(errors),
        'mae': sum(abs(value) for value in errors) / len(errors),
        'rmse': math.sqrt(
            sum(value * value for value in errors) / len(errors)
        ),
        'max_abs_error': max(abs(value) for value in errors),
    }


def stop_response_time(samples, field, profile_end, threshold):
    consecutive = 0
    first_time = None
    for sample in samples:
        if sample['time'] < profile_end:
            continue
        value = sample[field]
        if math.isfinite(value) and abs(value) <= threshold:
            if consecutive == 0:
                first_time = sample['time']
            consecutive += 1
            if consecutive >= 3:
                return max(0.0, first_time - profile_end)
        else:
            consecutive = 0
            first_time = None
    return None


def yaw_change(yaw_values):
    finite_values = [value for value in yaw_values if math.isfinite(value)]
    if len(finite_values) < 2:
        return None
    total = 0.0
    previous = finite_values[0]
    for current in finite_values[1:]:
        delta = math.atan2(
            math.sin(current - previous),
            math.cos(current - previous),
        )
        total += delta
        previous = current
    return total


def calculate_summary(node):
    samples = node.samples
    times = [sample['time'] for sample in samples]
    series_names = [
        'target_lin', 'target_ang', 'cmd_lin', 'cmd_ang',
        'odom_lin', 'odom_ang', 'wheel_lin', 'wheel_ang',
    ]
    series = {
        name: [sample[name] for sample in samples]
        for name in series_names
    }
    linear_active = [
        sample['phase'] == 'motion' and abs(sample['target_lin']) > 1e-9
        for sample in samples
    ]
    angular_active = [
        sample['phase'] == 'motion' and abs(sample['target_ang']) > 1e-9
        for sample in samples
    ]
    profile_end = node.profile_start + node.profile_duration

    cumulative = {
        name: integrate_series(times, values)
        for name, values in series.items()
    }

    def final_integral(name):
        if not any(math.isfinite(value) for value in series[name]):
            return None
        return cumulative[name][-1] if cumulative[name] else None

    summary = {
        'test': asdict(node.config),
        'finish_reason': node.finish_reason,
        'sample_count': len(samples),
        'profile_duration': node.profile_duration,
        'topics': {
            'command_input': COMMAND_INPUT_TOPIC,
            'final_command': FINAL_COMMAND_TOPIC,
            'odometry': ODOM_TOPIC,
            'wheel_velocity': WHEEL_TOPIC,
        },
        'linear': {
            'cmd_vs_target': error_metrics(
                series['target_lin'], series['cmd_lin'], linear_active
            ),
            'odom_vs_cmd': error_metrics(
                series['cmd_lin'], series['odom_lin'], linear_active
            ),
            'target_distance': final_integral('target_lin'),
            'cmd_distance': final_integral('cmd_lin'),
            'odom_distance': final_integral('odom_lin'),
            'wheel_distance': final_integral('wheel_lin'),
            'stop_response_time': (
                stop_response_time(
                    samples, 'odom_lin', profile_end, 0.001
                )
                if any(linear_active) else None
            ),
        },
        'angular': {
            'cmd_vs_target': error_metrics(
                series['target_ang'], series['cmd_ang'], angular_active
            ),
            'odom_vs_cmd': error_metrics(
                series['cmd_ang'], series['odom_ang'], angular_active
            ),
            'target_angle': final_integral('target_ang'),
            'cmd_angle': final_integral('cmd_ang'),
            'odom_angle': final_integral('odom_ang'),
            'wheel_angle': final_integral('wheel_ang'),
            'pose_yaw_change': yaw_change(
                [sample['odom_yaw'] for sample in samples]
            ),
            'stop_response_time': (
                stop_response_time(
                    samples, 'odom_ang', profile_end, 0.005
                )
                if any(angular_active) else None
            ),
        },
        'pose': {
            'start': None,
            'end': None,
            'xy_displacement': None,
        },
    }
    if samples:
        start = samples[0]
        end = samples[-1]
        summary['pose'] = {
            'start': {
                'x': start['odom_x'],
                'y': start['odom_y'],
                'yaw': start['odom_yaw'],
            },
            'end': {
                'x': end['odom_x'],
                'y': end['odom_y'],
                'yaw': end['odom_yaw'],
            },
            'xy_displacement': math.hypot(
                end['odom_x'] - start['odom_x'],
                end['odom_y'] - start['odom_y'],
            ),
        }
    return summary, series, cumulative


def svg_polyline(times, values, x_map, y_map, color):
    """把可能包含 NaN 的数据拆成多段 SVG 折线。"""
    elements = []
    points = []
    for sample_time, value in zip(times, values):
        if math.isfinite(value):
            points.append(f'{x_map(sample_time):.2f},{y_map(value):.2f}')
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


def report_title_lines(config):
    """生成可视化报告的中文标题和速度副标题。"""
    speed_type = (
        '目标峰值'
        if config.waveform in ('trapezoid', 'sine')
        else '目标'
    )
    main_title = f'底盘运动精度测试－{config.description}'
    speed_title = (
        f'{speed_type}线速度：{format_parameter_value(config.linear)} m/s｜'
        f'{speed_type}角速度：'
        f'{format_parameter_value(config.angular)} rad/s'
    )
    return main_title, speed_title


def format_parameter_value(value):
    """参数值最多保留 6 位小数，并去除无意义的末尾零。"""
    if value == 0.0:
        return '0'
    if abs(value) < 1e-6:
        mantissa, exponent = f'{value:.3e}'.split('e')
        return f'{mantissa.rstrip("0").rstrip(".")}e{int(exponent)}'
    return f'{value:.6f}'.rstrip('0').rstrip('.')


def format_text_value(value, unit=''):
    if value is None or not math.isfinite(value):
        return '无数据'
    suffix = f' {unit}' if unit else ''
    return f'{value:.6f}{suffix}'


def format_percent_error(actual, reference):
    if (
        actual is None
        or reference is None
        or not math.isfinite(actual)
        or not math.isfinite(reference)
        or abs(reference) < 1e-12
    ):
        return '无数据'
    return f'{(actual - reference) / abs(reference) * 100.0:.3f}%'


def format_axis_tick(value):
    """小量级刻度直接携带科学计数法，避免遗漏公共倍率。"""
    magnitude = abs(value)
    if magnitude != 0.0 and (magnitude < 1e-3 or magnitude >= 1e4):
        mantissa, exponent = f'{value:.1e}'.split('e')
        return f'{mantissa}e{int(exponent)}'
    return f'{value:.4g}'


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
    ticks = [0.0 if abs(value) < step * 1e-9 else value for value in ticks]
    return axis_min, axis_max, ticks


def metric_text(lines, title, metrics, unit):
    """把一组误差指标追加为中文文本。"""
    lines.append(title)
    if metrics['sample_count'] == 0:
        lines.append('  无有效运动样本')
        return
    lines.extend([
        f"  样本数量：{metrics['sample_count']}",
        f"  平均误差（有方向）："
        f"{format_text_value(metrics['bias'], unit)}",
        f"  平均绝对误差：{format_text_value(metrics['mae'], unit)}",
        f"  均方根误差（RMSE）："
        f"{format_text_value(metrics['rmse'], unit)}",
        f"  最大绝对误差："
        f"{format_text_value(metrics['max_abs_error'], unit)}",
    ])


def create_text_report(node, summary, report_path):
    """生成便于人工阅读的中文测试结果文本。"""
    config = node.config
    finish_names = {
        'completed': '完整完成',
        'keyboard_interrupt': '用户按 Ctrl+C 中断',
        'safety_check_failed': '运行前安全检查未通过',
        'final_command_conflict': '最终速度命令发生冲突',
        'unexpected_shutdown': '异常关闭',
    }
    finish_name = finish_names.get(
        summary['finish_reason'], summary['finish_reason']
    )
    mode_names = {
        'step': '恒定阶跃速度',
        'trapezoid': '梯形速度',
        'sine': '正弦速度',
    }
    mode_name = mode_names[config.waveform]
    linear = summary['linear']
    angular = summary['angular']
    pose = summary['pose']

    lines = [
        '底盘运动精度测试结果',
        '=' * 50,
        f'测试名称：{config.description}',
        f'完成状态：{finish_name}',
        f'运动模式：{mode_name}',
        f'目标线速度：{format_parameter_value(config.linear)} m/s',
        f'目标角速度：{format_parameter_value(config.angular)} rad/s',
        f'运动阶段时长：'
        f'{format_parameter_value(summary["profile_duration"])} s',
        f'启动等待时长：'
        f'{format_parameter_value(config.startup_wait)} s',
        f'零速基线时长：{format_parameter_value(config.baseline)} s',
        f'停车采集时长：{format_parameter_value(config.settle)} s',
        f'命令发布频率：{format_parameter_value(config.rate)} Hz',
        f'采样数量：{summary["sample_count"]}',
    ]
    if config.waveform == 'trapezoid':
        lines.extend([
            f'线加速度：'
            f'{format_parameter_value(config.linear_accel)} m/s²',
            f'角加速度：'
            f'{format_parameter_value(config.angular_accel)} rad/s²',
            f'线速度匀速时间：'
            f'{format_parameter_value(config.linear_hold)} s',
            f'角速度匀速时间：'
            f'{format_parameter_value(config.angular_hold)} s',
        ])
    elif config.waveform == 'sine':
        lines.append(
            f'正弦频率：'
            f'{format_parameter_value(config.sine_frequency)} Hz'
        )

    lines.extend([
        '',
        '话题链路',
        '-' * 50,
        f'测试命令输入：{summary["topics"]["command_input"]}',
        f'底盘最终命令：{summary["topics"]["final_command"]}',
        f'里程计反馈：{summary["topics"]["odometry"]}',
        f'轮速反馈：{summary["topics"]["wheel_velocity"]}',
        '',
        '线速度误差',
        '-' * 50,
    ])
    metric_text(
        lines,
        '最终命令相对目标值：',
        linear['cmd_vs_target'],
        'm/s',
    )
    metric_text(
        lines,
        '里程计相对最终命令：',
        linear['odom_vs_cmd'],
        'm/s',
    )

    lines.extend([
        '',
        '角速度误差',
        '-' * 50,
    ])
    metric_text(
        lines,
        '最终命令相对目标值：',
        angular['cmd_vs_target'],
        'rad/s',
    )
    metric_text(
        lines,
        '里程计相对最终命令：',
        angular['odom_vs_cmd'],
        'rad/s',
    )

    lines.extend([
        '',
        '累计距离与角度',
        '-' * 50,
        f'目标距离积分：'
        f'{format_text_value(linear["target_distance"], "m")}',
        f'最终命令距离积分：'
        f'{format_text_value(linear["cmd_distance"], "m")}',
        f'里程计距离积分：'
        f'{format_text_value(linear["odom_distance"], "m")}',
        f'轮速距离积分：'
        f'{format_text_value(linear["wheel_distance"], "m")}',
        f'里程计相对最终命令距离误差率：'
        f'{format_percent_error(linear["odom_distance"], linear["cmd_distance"])}',
        f'目标角度积分：'
        f'{format_text_value(angular["target_angle"], "rad")}',
        f'最终命令角度积分：'
        f'{format_text_value(angular["cmd_angle"], "rad")}',
        f'里程计角度积分：'
        f'{format_text_value(angular["odom_angle"], "rad")}',
        f'轮速角度积分：'
        f'{format_text_value(angular["wheel_angle"], "rad")}',
        f'里程计相对最终命令角度误差率：'
        f'{format_percent_error(angular["odom_angle"], angular["cmd_angle"])}',
        f'里程计位姿偏航角变化：'
        f'{format_text_value(angular["pose_yaw_change"], "rad")}',
        '',
        '停车响应与位姿',
        '-' * 50,
        f'线速度停车响应时间：'
        f'{format_text_value(linear["stop_response_time"], "s")}',
        f'角速度停车响应时间：'
        f'{format_text_value(angular["stop_response_time"], "s")}',
        f'起终点平面位移：'
        f'{format_text_value(pose["xy_displacement"], "m")}',
        '',
        '指标说明',
        '-' * 50,
        '平均误差（有方向）：正值表示测量值偏大，负值表示测量值偏小。',
        '平均绝对误差：忽略正负方向后的平均偏差。',
        '均方根误差（RMSE）：对较大的瞬时误差更敏感。',
        '最大绝对误差：本次有效运动样本中的最大瞬时偏差。',
    ])
    report_path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return report_path


def svg_panel(x, y, width, height, title, unit, times, lines,
              profile_start, profile_end):
    """生成一个带坐标轴、图例和运动起止线的 SVG 子图。"""
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
            f'<text x="{grid_x:.2f}" y="{plot_top + plot_height + 19}" '
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
                f'x2="{marker_x:.2f}" y2="{plot_top + plot_height}" '
                'stroke="#6e7781" stroke-width="1.5" '
                'stroke-dasharray="5 4"/>'
            )

    elements.extend([
        f'<line x1="{plot_left}" y1="{plot_top + plot_height}" '
        f'x2="{plot_left + plot_width}" y2="{plot_top + plot_height}" '
        'stroke="#24292f"/>',
        f'<line x1="{plot_left}" y1="{plot_top}" '
        f'x2="{plot_left}" y2="{plot_top + plot_height}" '
        'stroke="#24292f"/>',
        f'<text x="{plot_left + plot_width / 2:.1f}" '
        f'y="{y + height - 8}" text-anchor="middle" class="axis">'
        '时间（s）</text>',
        f'<text x="{x + 15}" y="{plot_top + plot_height / 2:.1f}" '
        f'text-anchor="middle" class="axis" '
        f'transform="rotate(-90 {x + 15} '
        f'{plot_top + plot_height / 2:.1f})">{escape(unit)}</text>',
    ])

    legend_x = plot_left + 5
    for name, values, color in lines:
        elements.extend(svg_polyline(times, values, x_map, y_map, color))
        elements.extend([
            f'<line x1="{legend_x}" y1="{plot_top + 14}" '
            f'x2="{legend_x + 20}" y2="{plot_top + 14}" '
            f'stroke="{color}" stroke-width="3"/>',
            f'<text x="{legend_x + 25}" y="{plot_top + 18}" '
            f'class="legend">{escape(name)}</text>',
        ])
        legend_x += 118
    return elements


def create_report(node, summary, series, cumulative, report_path):
    """使用纯标准库生成 SVG，避免 NumPy/Matplotlib 版本依赖。"""
    samples = node.samples
    times = [sample['time'] for sample in samples]
    profile_start = node.profile_start
    profile_end = profile_start + node.profile_duration
    colors = {
        'target': '#0969da',
        'cmd': '#cf222e',
        'odom': '#1a7f37',
        'wheel': '#8250df',
        'error_a': '#bf8700',
        'error_b': '#cf222e',
    }
    linear_cmd_error = [
        cmd - target if math.isfinite(cmd) else math.nan
        for cmd, target in zip(series['cmd_lin'], series['target_lin'])
    ]
    linear_odom_error = [
        odom - cmd
        if math.isfinite(odom) and math.isfinite(cmd) else math.nan
        for odom, cmd in zip(series['odom_lin'], series['cmd_lin'])
    ]
    angular_cmd_error = [
        cmd - target if math.isfinite(cmd) else math.nan
        for cmd, target in zip(series['cmd_ang'], series['target_ang'])
    ]
    angular_odom_error = [
        odom - cmd
        if math.isfinite(odom) and math.isfinite(cmd) else math.nan
        for odom, cmd in zip(series['odom_ang'], series['cmd_ang'])
    ]

    panels = [
        (
            '线速度对比', '线速度（m/s）',
            [
                ('目标值', series['target_lin'], colors['target']),
                ('最终命令', series['cmd_lin'], colors['cmd']),
                ('里程计', series['odom_lin'], colors['odom']),
                ('轮速', series['wheel_lin'], colors['wheel']),
            ],
        ),
        (
            '角速度对比', '角速度（rad/s）',
            [
                ('目标值', series['target_ang'], colors['target']),
                ('最终命令', series['cmd_ang'], colors['cmd']),
                ('里程计', series['odom_ang'], colors['odom']),
                ('轮速', series['wheel_ang'], colors['wheel']),
            ],
        ),
        (
            '线速度误差', '误差（m/s）',
            [
                ('最终命令－目标值', linear_cmd_error, colors['error_a']),
                ('里程计－最终命令', linear_odom_error, colors['error_b']),
            ],
        ),
        (
            '角速度误差', '误差（rad/s）',
            [
                ('最终命令－目标值', angular_cmd_error, colors['error_a']),
                ('里程计－最终命令', angular_odom_error, colors['error_b']),
            ],
        ),
        (
            '累计行驶距离', '距离（m）',
            [
                ('目标积分', cumulative['target_lin'], colors['target']),
                ('最终命令积分', cumulative['cmd_lin'], colors['cmd']),
                ('里程计积分', cumulative['odom_lin'], colors['odom']),
                ('轮速积分', cumulative['wheel_lin'], colors['wheel']),
            ],
        ),
        (
            '累计旋转角度', '角度（rad）',
            [
                ('目标积分', cumulative['target_ang'], colors['target']),
                ('最终命令积分', cumulative['cmd_ang'], colors['cmd']),
                ('里程计积分', cumulative['odom_ang'], colors['odom']),
                ('轮速积分', cumulative['wheel_ang'], colors['wheel']),
            ],
        ),
    ]

    width = 1500
    height = 1110
    panel_width = 710
    panel_height = 315
    main_title, speed_title = report_title_lines(node.config)
    elements = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<style>'
        'text{font-family:"Noto Sans CJK SC","Source Han Sans SC",'
        '"Microsoft YaHei","WenQuanYi Micro Hei",SimHei,'
        '"Droid Sans Fallback",sans-serif;fill:#24292f}'
        '.title{font-size:24px;font-weight:600}'
        '.subtitle{font-size:15px;fill:#57606a}'
        '.panel-title{font-size:17px;font-weight:600}'
        '.tick{font-size:11px;fill:#57606a}'
        '.axis{font-size:12px;fill:#57606a}'
        '.legend{font-size:11px}'
        '.footer{font-size:13px;fill:#57606a}'
        '</style>',
        '<rect width="100%" height="100%" fill="#f6f8fa"/>',
        f'<text x="{width / 2}" y="34" text-anchor="middle" class="title">'
        f'{escape(main_title)}</text>',
        f'<text x="{width / 2}" y="57" text-anchor="middle" '
        f'class="subtitle">{escape(speed_title)}</text>',
    ]

    if not samples:
        elements.append(
            f'<text x="{width / 2}" y="{height / 2}" '
            'text-anchor="middle" class="title">'
            '未收到里程计采样数据</text>'
        )
    else:
        for index, (title, unit, lines) in enumerate(panels):
            column = index % 2
            row = index // 2
            panel_x = 25 + column * 740
            panel_y = 72 + row * 330
            elements.extend(svg_panel(
                panel_x,
                panel_y,
                panel_width,
                panel_height,
                title,
                unit,
                times,
                lines,
                profile_start,
                profile_end,
            ))

    elements.append(
        f'<text x="25" y="{height - 18}" class="footer">'
        '竖向虚线：运动开始／结束；误差统计详见 report.txt'
        '</text>'
    )
    elements.append('</svg>')
    report_path.write_text('\n'.join(elements), encoding='utf-8')
    return report_path


def create_png_report(node, summary, series, cumulative, report_path):
    """实机环境支持 Matplotlib 时，额外生成 PNG 报告。"""
    import_stderr = io.StringIO()
    with contextlib.redirect_stderr(import_stderr):
        import matplotlib
        matplotlib.use('Agg')
        from matplotlib import font_manager
        from matplotlib.ticker import FuncFormatter
        import matplotlib.pyplot as plt

    chinese_font_candidates = [
        'Noto Sans CJK SC',
        'Source Han Sans SC',
        'Noto Sans CJK JP',
        'WenQuanYi Micro Hei',
        'Microsoft YaHei',
        'SimHei',
        'Droid Sans Fallback',
        'Arial Unicode MS',
    ]
    installed_fonts = {
        font.name for font in font_manager.fontManager.ttflist
    }
    selected_font = next(
        (
            font_name
            for font_name in chinese_font_candidates
            if font_name in installed_fonts
        ),
        None,
    )
    if selected_font is None:
        raise RuntimeError(
            '未检测到可用中文字体；为避免 PNG 中文显示成方框，'
            '本次仅生成 SVG 报告'
        )
    matplotlib.rcParams['font.sans-serif'] = (
        [selected_font, 'DejaVu Sans']
    )
    matplotlib.rcParams['axes.unicode_minus'] = False

    samples = node.samples
    figure, axes = plt.subplots(3, 2, figsize=(15, 12), sharex=True)
    main_title, speed_title = report_title_lines(node.config)
    figure.suptitle(
        f'{main_title}\n{speed_title}',
        fontsize=16,
    )
    if not samples:
        for axis in axes.flat:
            axis.axis('off')
        axes[1, 0].text(
            0.5, 0.5, '未收到里程计采样数据',
            ha='center', va='center', fontsize=16,
        )
        figure.tight_layout()
        figure.savefig(report_path, dpi=150)
        plt.close(figure)
        return report_path

    times = [sample['time'] for sample in samples]
    profile_start = node.profile_start
    profile_end = profile_start + node.profile_duration

    def decorate(axis, title, ylabel):
        axis.set_title(title)
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.3)
        axis.axvline(profile_start, color='gray', linestyle=':', alpha=0.8)
        axis.axvline(profile_end, color='gray', linestyle=':', alpha=0.8)

    velocity_plots = [
        (
            axes[0, 0], '线速度对比', '线速度（m/s）',
            'target_lin', 'cmd_lin', 'odom_lin', 'wheel_lin',
        ),
        (
            axes[0, 1], '角速度对比', '角速度（rad/s）',
            'target_ang', 'cmd_ang', 'odom_ang', 'wheel_ang',
        ),
    ]
    for axis, title, unit, target, cmd, odom, wheel in velocity_plots:
        axis.plot(times, series[target], label='目标值')
        axis.plot(times, series[cmd], label='最终命令')
        axis.plot(times, series[odom], label='里程计')
        if any(math.isfinite(value) for value in series[wheel]):
            axis.plot(times, series[wheel], label='轮速', alpha=0.8)
        decorate(axis, title, unit)
        axis.legend()

    error_plots = [
        (
            axes[1, 0], '线速度误差', '误差（m/s）',
            'target_lin', 'cmd_lin', 'odom_lin',
        ),
        (
            axes[1, 1], '角速度误差', '误差（rad/s）',
            'target_ang', 'cmd_ang', 'odom_ang',
        ),
    ]
    for axis, title, unit, target, cmd, odom in error_plots:
        axis.plot(
            times,
            [
                cmd_value - target_value
                if math.isfinite(cmd_value) else math.nan
                for cmd_value, target_value in zip(
                    series[cmd], series[target]
                )
            ],
            label='最终命令－目标值',
        )
        axis.plot(
            times,
            [
                odom_value - cmd_value
                if math.isfinite(odom_value) and math.isfinite(cmd_value)
                else math.nan
                for odom_value, cmd_value in zip(
                    series[odom], series[cmd]
                )
            ],
            label='里程计－最终命令',
        )
        decorate(axis, title, unit)
        axis.legend()

    integral_plots = [
        (
            axes[2, 0], '累计行驶距离', '距离（m）',
            'target_lin', 'cmd_lin', 'odom_lin', 'wheel_lin',
        ),
        (
            axes[2, 1], '累计旋转角度', '角度（rad）',
            'target_ang', 'cmd_ang', 'odom_ang', 'wheel_ang',
        ),
    ]
    for axis, title, unit, target, cmd, odom, wheel in integral_plots:
        axis.plot(times, cumulative[target], label='目标积分')
        axis.plot(times, cumulative[cmd], label='最终命令积分')
        axis.plot(times, cumulative[odom], label='里程计积分')
        if any(math.isfinite(value) for value in series[wheel]):
            axis.plot(
                times, cumulative[wheel], label='轮速积分', alpha=0.8
            )
        decorate(axis, title, unit)
        axis.set_xlabel('时间（s）')
        axis.legend()
        if title == '累计行驶距离':
            distance_values = [
                abs(value)
                for name in (target, cmd, odom, wheel)
                for value in cumulative[name]
                if math.isfinite(value)
            ]
            max_distance = max(distance_values, default=0.0)
            if 0.0 < max_distance < 1e-3:
                # 每一个刻度直接显示为 2.0e-5，不使用轴顶部的公共 1e-5。
                axis.yaxis.set_major_formatter(
                    FuncFormatter(
                        lambda value, _: format_axis_tick(value)
                    )
                )
                axis.yaxis.offsetText.set_visible(False)

    figure.text(
        0.01,
        0.005,
        '竖向虚线：运动开始／结束；误差统计详见 report.txt',
        fontsize=10,
    )
    figure.tight_layout(rect=(0.0, 0.025, 1.0, 0.94))
    figure.savefig(report_path, dpi=150)
    plt.close(figure)
    return report_path


def format_optional(value):
    return 'N/A' if value is None else f'{value:.6f}'


def save_results(node):
    node.output_dir.mkdir(parents=True, exist_ok=False)
    data_path = node.output_dir / 'data.csv'
    summary_path = node.output_dir / 'summary.json'
    text_report_path = node.output_dir / 'report.txt'
    svg_report_path = node.output_dir / 'report.svg'
    png_report_path = node.output_dir / 'report.png'

    with data_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(node.samples)

    summary, series, cumulative = calculate_summary(node)
    with summary_path.open('w', encoding='utf-8') as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)
        stream.write('\n')

    create_text_report(node, summary, text_report_path)

    report_paths = []
    if node.config.report_format in ('svg', 'both'):
        report_paths.append(create_report(
            node, summary, series, cumulative, svg_report_path
        ))
    if node.config.report_format in ('png', 'both'):
        try:
            report_paths.append(create_png_report(
                node, summary, series, cumulative, png_report_path
            ))
        except Exception as exc:
            if hasattr(node, 'get_logger'):
                node.get_logger().warning(
                    f'实机 Python 环境无法生成 PNG：{exc}；'
                    'SVG 报告仍可正常使用。'
                )
    if not report_paths:
        report_paths.append(create_report(
            node, summary, series, cumulative, svg_report_path
        ))
    return data_path, summary_path, text_report_path, report_paths


def main():
    """解析参数并运行一次完整的底盘运动精度测试。"""
    try:
        config = parse_config()
    except (ValueError, SystemExit) as exc:
        if isinstance(exc, SystemExit):
            raise
        print(f'参数错误：{exc}', file=sys.stderr)
        return 2

    # 禁用 rclpy 默认 SIGINT 处理。否则按 Ctrl+C 时 ROS Context 会先被
    # shutdown，后续无法发布零速度，也无法正常输出 rosout 日志。
    # 保留 Python 默认 SIGINT 行为，让 spin_once() 抛出 KeyboardInterrupt，
    # 再由下方按“停车 -> 保存 -> shutdown”的顺序清理。
    rclpy.init(
        args=sys.argv,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = ChassisMotionTest(config)
    try:
        # 正常完成时 finalize() 设置 finished。当前回调返回后，
        # 主循环主动结束，shutdown 统一放在 finally 中执行。
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        if not node.finished:
            node.get_logger().warning(
                '用户中断测试，正在发送零速度并保存已有数据。'
            )
            node.finalize('keyboard_interrupt')
    except Exception as exc:
        node.get_logger().error(f'测试运行异常：{exc}')
        node.finalize(f'exception: {exc}')
        raise
    finally:
        if not node.finished:
            node.finalize('unexpected_shutdown')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
