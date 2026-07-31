#!/usr/bin/env python3
"""全向底盘运动测试工具。

功能概览：
1. 支持 map 坐标系下的 X+/X-/Y+/Y-、任意角度平移和原地旋转；
2. 支持用户按“速度+距离/角度”定义有限次数的组合循环测试；
3. 通过 ROS 2 动态参数设置速度、运行时间、平移角度和旋转角度；
4. 测试期间持续发布 Twist，结束后主动发布零速并等待车辆稳定；
5. verbose 模式把状态写入可由 tail -f 查看的一般文本日志；
6. 每次测试保存 CSV、map 轨迹图、里程计速度响应图、IMU 倾角图、
   四轮电机反馈图和底盘电流/温度/故障状态图。

安全约定：
- 本脚本直接发布 /control/cmd_vel_omni。测试前应停止导航、手柄或其他同话题发布者；
- 平移角度以 map +X 为 0 度，逆时针为正；
- 平移模式令 angular.z=0；原地旋转模式令 linear.x/linear.y=0；
- 循环次数必须是有限值，脚本不提供无限循环，避免车辆无人值守持续运动；
- Ctrl-C、cancel_test=true、TF 丢失都会立即发布零速。
"""

from __future__ import annotations

import csv
import json
import logging
import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from chassis_interfaces.msg import ChassisMotorControl
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


MODE_TO_ANGLE_DEG = {
    "x_positive": 0.0,
    "x_negative": 180.0,
    "y_positive": 90.0,
    "y_negative": -90.0,
}
ROTATION_MODE_TO_SIGN = {
    "rotate_left": 1.0,
    "rotate_right": -1.0,
}
VALID_MODES = (
    tuple(MODE_TO_ANGLE_DEG)
    + ("angle",)
    + tuple(ROTATION_MODE_TO_SIGN)
)
WHEEL_NAMES = ("front_left", "front_right", "rear_left", "rear_right")


def normalize_angle(angle: float) -> float:
    """把角度归一化到 [-pi, pi)。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_rpy(
    x: float, y: float, z: float, w: float
) -> Tuple[float, float, float]:
    """从四元数提取 ROS 约定的 roll、pitch、yaw，单位均为 rad。"""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        return math.nan, math.nan, math.nan
    x /= norm
    y /= norm
    z /= norm
    w /= norm

    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return roll, pitch, yaw


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """从四元数提取平面 yaw。"""
    return quaternion_to_rpy(x, y, z, w)[2]


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float
    stamp_sec: float


@dataclass
class OdomState:
    stamp_sec: float
    receive_sec: float
    pose_x: float
    pose_y: float
    pose_yaw: float
    vx_base: float
    vy_base: float
    wz: float


@dataclass
class MotorState:
    stamp_sec: float
    position: List[float]
    speed: List[float]


@dataclass
class ImuState:
    """标准 sensor_msgs/Imu 中与底盘倾斜和动态响应有关的数据。"""

    stamp_sec: float
    receive_sec: float
    frame_id: str
    roll: float
    pitch: float
    yaw: float
    tilt: float
    angular_velocity_x: float
    angular_velocity_y: float
    angular_velocity_z: float
    linear_acceleration_x: float
    linear_acceleration_y: float
    linear_acceleration_z: float


@dataclass
class ChassisStatusState:
    """robot_server_state 中与底盘测试有关的汇总状态。

    注意：机器人当前只发布左右两侧汇总电流、左右温度和驱动器温度，
    并没有发布四个舵向电机/四个行走电机各自的温度、电流或力矩。
    """

    stamp_sec: float
    receive_sec: float
    status_word: int
    error_code_left: int
    error_code_right: int
    velocity_left: float
    velocity_right: float
    current_left: float
    current_right: float
    driver_temperature: float
    temperature_left: float
    temperature_right: float

    def has_valid_temperature(self) -> bool:
        """全零温度在当前机器人上表示未上报，不当作真实 0 摄氏度。"""
        temperatures = (
            self.driver_temperature,
            self.temperature_left,
            self.temperature_right,
        )
        return any(
            math.isfinite(value) and abs(value) > 1.0e-6
            for value in temperatures
        )


def _json_number(payload: Dict[str, object], key: str) -> float:
    """安全读取 JSON 数值；字段缺失或类型错误时返回 NaN。"""
    try:
        value = float(payload[key])
    except (KeyError, TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def _json_integer(payload: Dict[str, object], key: str) -> int:
    """安全读取 JSON 整数；字段缺失或类型错误时返回 -1。"""
    try:
        return int(payload[key])
    except (KeyError, TypeError, ValueError):
        return -1


def parse_chassis_status_json(
    message_data: str, receive_sec: float
) -> ChassisStatusState:
    """解析 /feedback/robot_server_state，并只保留底盘相关字段。"""
    decoded = json.loads(message_data)
    if not isinstance(decoded, dict):
        raise ValueError("robot_server_state 顶层必须是 JSON 对象")

    # timestamp 是机器人状态消息中的纳秒时间戳；没有该字段则保留 NaN。
    # 后续 CSV 中的接收时间仍可用于排查消息是否卡顿。
    stamp_ns = _json_number(decoded, "timestamp")
    stamp_sec = stamp_ns * 1.0e-9 if math.isfinite(stamp_ns) else math.nan
    return ChassisStatusState(
        stamp_sec=stamp_sec,
        receive_sec=receive_sec,
        status_word=_json_integer(decoded, "chassis_status_word"),
        error_code_left=_json_integer(decoded, "chassis_error_code_left"),
        error_code_right=_json_integer(decoded, "chassis_error_code_right"),
        velocity_left=_json_number(decoded, "chassis_left_velocity"),
        velocity_right=_json_number(decoded, "chassis_right_velocity"),
        current_left=_json_number(decoded, "chassis_left_current"),
        current_right=_json_number(decoded, "chassis_right_current"),
        driver_temperature=_json_number(
            decoded, "chassis_driver_temperature"),
        temperature_left=_json_number(decoded, "chassis_left_temperature"),
        temperature_right=_json_number(decoded, "chassis_right_temperature"),
    )


@dataclass
class TestConfig:
    mode: str
    speed_mps: float
    angular_speed_radps: float
    duration_sec: float
    angle_deg: float
    direction_x: float
    direction_y: float
    rotation_direction: float
    rotation_angle_deg: float
    settle_time_sec: float
    log_period_sec: float
    overshoot_threshold_m: float
    rotation_overshoot_threshold_deg: float

    @property
    def is_rotation(self) -> bool:
        return self.mode in ROTATION_MODE_TO_SIGN


@dataclass
class LoopStep:
    """用户循环计划中的一个动作步骤。"""

    mode: str
    speed: float
    amount: float
    translation_angle_deg: float = 0.0


def parse_loop_step(step_text: str) -> LoopStep:
    """解析循环步骤。

    平移格式：
      x_positive,速度m/s,距离m
      angle,速度m/s,距离m,map角度deg
    旋转格式：
      rotate_left,角速度rad/s,旋转角度deg
    """
    parts = [part.strip() for part in step_text.split(",")]
    if not parts or parts[0] not in VALID_MODES:
        raise ValueError(
            f"未知循环模式：{parts[0] if parts else step_text}")

    mode = parts[0]
    expected_parts = 4 if mode == "angle" else 3
    if len(parts) != expected_parts:
        raise ValueError(
            f"步骤“{step_text}”应包含 {expected_parts} 项，实际 {len(parts)} 项")

    try:
        speed = float(parts[1])
        amount = float(parts[2])
        translation_angle_deg = (
            float(parts[3]) if mode == "angle" else 0.0)
    except ValueError as error:
        raise ValueError(f"步骤“{step_text}”包含非数值参数") from error

    if not math.isfinite(speed) or speed <= 0.0:
        raise ValueError(f"步骤“{step_text}”的速度必须大于 0")
    if not math.isfinite(amount) or amount <= 0.0:
        unit = "旋转角度" if mode in ROTATION_MODE_TO_SIGN else "距离"
        raise ValueError(f"步骤“{step_text}”的{unit}必须大于 0")
    if (
        mode == "angle"
        and not math.isfinite(translation_angle_deg)
    ):
        raise ValueError(f"步骤“{step_text}”的平移角度不是有限数值")
    if mode in ROTATION_MODE_TO_SIGN and amount > 360.0:
        raise ValueError(
            f"步骤“{step_text}”单次旋转不能超过 360deg；"
            "需要多圈时请增加 loop_count 或拆成多个步骤")

    return LoopStep(
        mode=mode,
        speed=speed,
        amount=amount,
        translation_angle_deg=translation_angle_deg,
    )


class OmniChassisTest(Node):
    """通过参数触发单次或有限循环的全向底盘运动测试。"""

    def __init__(self) -> None:
        super().__init__("omni_chassis_test")

        self._declare_parameters()

        self.command_topic = str(self.get_parameter("command_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.motor_feedback_topic = str(
            self.get_parameter("motor_feedback_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.robot_state_topic = str(
            self.get_parameter("robot_state_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.verbose = bool(self.get_parameter("verbose").value)

        output_dir_value = str(self.get_parameter("output_dir").value)
        self.output_dir = Path(output_dir_value).expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = self.output_dir / "omni_chassis_test.log"
        self.file_logger = self._create_file_logger(self.log_path)

        self.command_pub = self.create_publisher(Twist, self.command_topic, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, 50)
        self.motor_sub = self.create_subscription(
            ChassisMotorControl,
            self.motor_feedback_topic,
            self._motor_callback,
            20,
        )
        # IMU 通常使用 best-effort 的传感器 QoS。若设备实际话题不同，
        # 启动脚本时通过 -p imu_topic:=/实际话题 修改。
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self._imu_callback,
            qos_profile_sensor_data,
        )
        # 该话题是 std_msgs/String 包装的 JSON。当前能读到左右汇总电流、
        # 温度、状态字和错误码；没有底盘力矩字段。
        self.robot_state_sub = self.create_subscription(
            String,
            self.robot_state_topic,
            self._robot_state_callback,
            10,
        )

        # map 下的位置以 TF 为准：建图/定位节点可能更新 map->odom，
        # 直接使用 odom.pose 无法满足“以 map 坐标系为准”的测试要求。
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz 必须大于 0")
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._timer_callback)
        self.add_on_set_parameters_callback(self._parameter_callback)

        self.state = "idle"
        self.start_requested = False
        self.loop_start_requested = False
        self.cancel_requested = False
        self.latest_odom: Optional[OdomState] = None
        self.latest_motor: Optional[MotorState] = None
        self.latest_imu: Optional[ImuState] = None
        self.latest_chassis_status: Optional[ChassisStatusState] = None
        self.config: Optional[TestConfig] = None
        self.start_pose: Optional[Pose2D] = None
        self.stop_pose: Optional[Pose2D] = None
        self.target_pose: Optional[Pose2D] = None
        self.test_start_sec = 0.0
        self.stop_start_sec = 0.0
        self.last_status_log_sec = -math.inf
        self.last_wait_warning_sec = -math.inf
        self.last_robot_state_parse_warning_sec = -math.inf
        self.samples: List[Dict[str, float | str]] = []
        self.run_dir: Optional[Path] = None
        self.pending_config: Optional[TestConfig] = None
        self.loop_active = False
        self.loop_queue: List[TestConfig] = []
        self.loop_total_steps = 0
        self.loop_completed_steps = 0
        self.loop_pause_until_sec = 0.0
        self.loop_session_name = ""
        self.loop_result_dirs: List[str] = []
        self.last_rotation_pose_yaw = 0.0
        self.rotation_progress_rad = 0.0

        self._log(
            "info",
            "全向底盘测试节点已启动；命令话题=%s，odom=%s，电机反馈=%s，"
            "IMU=%s，底盘状态=%s，map_frame=%s，base_frame=%s",
            self.command_topic,
            self.odom_topic,
            self.motor_feedback_topic,
            self.imu_topic,
            self.robot_state_topic,
            self.map_frame,
            self.base_frame,
        )
        self._log("info", "详细日志文件：%s", self.log_path)
        self._log(
            "info",
            "单次测试执行 run_test=true；自定义循环执行 run_loop=true")

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "run_test",
            False,
            ParameterDescriptor(
                description="设置为 true 触发一次测试；测试开始后会自动复位为 false"),
        )
        self.declare_parameter(
            "run_loop",
            False,
            ParameterDescriptor(
                description=(
                    "设置为 true 执行 loop_steps 定义的有限循环测试；"
                    "开始后自动复位为 false")),
        )
        self.declare_parameter(
            "cancel_test",
            False,
            ParameterDescriptor(
                description="设置为 true 立即取消当前测试并发布零速"),
        )
        self.declare_parameter(
            "mode",
            "x_positive",
            ParameterDescriptor(
                description=(
                    "运动模式：x_positive/x_negative/y_positive/"
                    "y_negative/angle/rotate_left/rotate_right")),
        )
        self.declare_parameter(
            "speed_mps",
            0.10,
            ParameterDescriptor(description="平移速度绝对值，单位 m/s，必须大于 0"),
        )
        self.declare_parameter(
            "duration_sec",
            3.0,
            ParameterDescriptor(description="持续发送运动速度的时间，单位 s"),
        )
        self.declare_parameter(
            "angular_speed_radps",
            0.30,
            ParameterDescriptor(
                description="原地旋转角速度绝对值，单位 rad/s，必须大于 0"),
        )
        self.declare_parameter(
            "rotation_angle_deg",
            90.0,
            ParameterDescriptor(
                description=(
                    "rotate_left/rotate_right 的旋转角度绝对值，单位 deg；"
                    "脚本据此自动计算运行时间")),
        )
        self.declare_parameter(
            "translation_angle_deg",
            0.0,
            ParameterDescriptor(
                description="mode=angle 时生效；map +X 为 0 度，逆时针为正"),
        )
        self.declare_parameter(
            "settle_time_sec",
            2.0,
            ParameterDescriptor(
                description="停止发送非零速度后继续采样的时间，用于观察过冲"),
        )
        self.declare_parameter(
            "verbose",
            True,
            ParameterDescriptor(description="是否周期性输出详细状态到日志"),
        )
        self.declare_parameter(
            "log_period_sec",
            0.20,
            ParameterDescriptor(description="verbose 状态日志周期，单位 s"),
        )
        self.declare_parameter(
            "overshoot_threshold_m",
            0.01,
            ParameterDescriptor(description="判定过冲的最小距离，单位 m"),
        )
        self.declare_parameter(
            "rotation_overshoot_threshold_deg",
            2.0,
            ParameterDescriptor(description="原地旋转过冲判定阈值，单位 deg"),
        )
        self.declare_parameter(
            "loop_steps",
            [
                "x_positive,0.05,0.20",
                "rotate_left,0.30,90.0",
                "x_negative,0.05,0.20",
                "rotate_right,0.30,90.0",
            ],
            ParameterDescriptor(
                description=(
                    "循环步骤字符串数组。平移=模式,速度m/s,距离m；"
                    "任意平移=angle,速度,距离,map角度deg；"
                    "旋转=rotate_left/right,角速度rad/s,角度deg")),
        )
        self.declare_parameter(
            "loop_count",
            1,
            ParameterDescriptor(
                description="完整执行 loop_steps 的次数，范围 1~1000"),
        )
        self.declare_parameter(
            "loop_pause_sec",
            1.0,
            ParameterDescriptor(description="循环相邻步骤之间的零速等待时间，单位 s"),
        )
        self.declare_parameter(
            "publish_rate_hz",
            20.0,
            ParameterDescriptor(description="速度命令发布频率；修改后需重启脚本"),
        )
        self.declare_parameter(
            "command_topic",
            "/control/cmd_vel_omni",
            ParameterDescriptor(description="底盘速度命令话题；修改后需重启脚本"),
        )
        self.declare_parameter(
            "odom_topic",
            "/feedback/odom",
            ParameterDescriptor(description="里程计反馈话题；修改后需重启脚本"),
        )
        self.declare_parameter(
            "motor_feedback_topic",
            "/feedback/omni_chassis_motor",
            ParameterDescriptor(description="四轮舵角和轮速反馈话题；修改后需重启脚本"),
        )
        self.declare_parameter(
            "imu_topic",
            "/imu/chassis/refined",
            ParameterDescriptor(
                description=(
                    "融合后的底盘 sensor_msgs/Imu 话题，用于记录倾角；"
                    "修改后需重启脚本")),
        )
        self.declare_parameter(
            "robot_state_topic",
            "/feedback/robot_server_state",
            ParameterDescriptor(
                description=(
                    "机器人汇总状态 JSON 话题，用于读取底盘电流、"
                    "温度和故障码；"
                    "修改后需重启脚本")),
        )
        self.declare_parameter(
            "map_frame",
            "map",
            ParameterDescriptor(description="理论轨迹和误差评估坐标系"),
        )
        self.declare_parameter(
            "base_frame",
            "base_link",
            ParameterDescriptor(description="底盘 TF 坐标系"),
        )
        self.declare_parameter(
            "output_dir",
            "./omni_test_logs",
            ParameterDescriptor(description="日志、CSV 和图片输出目录"),
        )

    @staticmethod
    def _create_file_logger(path: Path) -> logging.Logger:
        logger = logging.getLogger(f"omni_chassis_test_file_{id(path)}")
        logger.setLevel(logging.INFO)
        logger.propagate = False
        logger.handlers.clear()
        handler = logging.FileHandler(path, mode="a", encoding="utf-8")
        handler.setFormatter(
            logging.Formatter(
                fmt="%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s",
                datefmt="%Y-%m-%d %H:%M:%S",
            ))
        logger.addHandler(handler)
        return logger

    def _log(self, level: str, message: str, *args: object) -> None:
        """同时写 ROS 日志和普通文本日志。"""
        rendered = message % args if args else message
        ros_logger = self.get_logger()
        if level == "error":
            ros_logger.error(rendered)
            self.file_logger.error(rendered)
        elif level == "warning":
            ros_logger.warning(rendered)
            self.file_logger.warning(rendered)
        else:
            ros_logger.info(rendered)
            self.file_logger.info(rendered)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _parameter_callback(self, parameters: List[Parameter]) -> SetParametersResult:
        """校验动态参数；当前测试使用启动瞬间的参数快照。"""
        busy = (
            self.state != "idle"
            or self.loop_active
            or self.loop_start_requested
        )
        runtime_parameters = {
            "mode",
            "speed_mps",
            "duration_sec",
            "angular_speed_radps",
            "rotation_angle_deg",
            "translation_angle_deg",
            "settle_time_sec",
            "log_period_sec",
            "overshoot_threshold_m",
            "rotation_overshoot_threshold_deg",
            "loop_steps",
            "loop_count",
            "loop_pause_sec",
        }

        for parameter in parameters:
            name = parameter.name
            value = parameter.value

            if busy and name in runtime_parameters:
                return SetParametersResult(
                    successful=False,
                    reason="测试运行中不能修改运动参数；请等待结束或先取消测试",
                )
            if name == "mode" and str(value) not in VALID_MODES:
                return SetParametersResult(
                    successful=False,
                    reason=f"mode 必须是 {VALID_MODES} 之一",
                )
            if name == "speed_mps" and float(value) <= 0.0:
                return SetParametersResult(
                    successful=False, reason="speed_mps 必须大于 0")
            if name == "duration_sec" and float(value) <= 0.0:
                return SetParametersResult(
                    successful=False, reason="duration_sec 必须大于 0")
            if name == "angular_speed_radps" and float(value) <= 0.0:
                return SetParametersResult(
                    successful=False, reason="angular_speed_radps 必须大于 0")
            if name == "rotation_angle_deg":
                if float(value) <= 0.0 or float(value) > 360.0:
                    return SetParametersResult(
                        successful=False,
                        reason="rotation_angle_deg 必须在 (0, 360] 范围内",
                    )
            if name == "settle_time_sec" and float(value) < 0.0:
                return SetParametersResult(
                    successful=False, reason="settle_time_sec 不能小于 0")
            if name == "log_period_sec" and float(value) <= 0.0:
                return SetParametersResult(
                    successful=False, reason="log_period_sec 必须大于 0")
            if name == "overshoot_threshold_m" and float(value) < 0.0:
                return SetParametersResult(
                    successful=False,
                    reason="overshoot_threshold_m 不能小于 0",
                )
            if (
                name == "rotation_overshoot_threshold_deg"
                and float(value) < 0.0
            ):
                return SetParametersResult(
                    successful=False,
                    reason="rotation_overshoot_threshold_deg 不能小于 0",
                )
            if name == "loop_count":
                loop_count = int(value)
                if loop_count < 1 or loop_count > 1000:
                    return SetParametersResult(
                        successful=False,
                        reason="loop_count 必须在 [1, 1000] 范围内",
                    )
            if name == "loop_pause_sec" and float(value) < 0.0:
                return SetParametersResult(
                    successful=False, reason="loop_pause_sec 不能小于 0")
            if name == "loop_steps":
                try:
                    steps = [str(item) for item in value]
                    if not steps:
                        raise ValueError("loop_steps 不能为空")
                    for step in steps:
                        parse_loop_step(step)
                except (TypeError, ValueError) as error:
                    return SetParametersResult(
                        successful=False, reason=str(error))
            if name == "run_test" and bool(value):
                if busy or self.start_requested or self.loop_start_requested:
                    return SetParametersResult(
                        successful=False, reason="已有测试正在运行或等待启动")
                self.start_requested = True
            if name == "run_loop" and bool(value):
                if busy or self.start_requested:
                    return SetParametersResult(
                        successful=False, reason="已有测试正在运行或等待启动")
                self.loop_start_requested = True
            if name == "cancel_test" and bool(value):
                self.cancel_requested = True
            if name == "verbose":
                self.verbose = bool(value)

        return SetParametersResult(successful=True)

    def _odom_callback(self, message: Odometry) -> None:
        stamp = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1.0e-9
        )
        orientation = message.pose.pose.orientation
        self.latest_odom = OdomState(
            stamp_sec=stamp,
            receive_sec=self._now_sec(),
            pose_x=float(message.pose.pose.position.x),
            pose_y=float(message.pose.pose.position.y),
            pose_yaw=quaternion_to_yaw(
                orientation.x, orientation.y, orientation.z, orientation.w),
            vx_base=float(message.twist.twist.linear.x),
            vy_base=float(message.twist.twist.linear.y),
            wz=float(message.twist.twist.angular.z),
        )

    def _motor_callback(self, message: ChassisMotorControl) -> None:
        stamp = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1.0e-9
        )
        position = [float(value) for value in message.position[:4]]
        speed = [float(value) for value in message.speed[:4]]
        position.extend([math.nan] * (4 - len(position)))
        speed.extend([math.nan] * (4 - len(speed)))
        self.latest_motor = MotorState(
            stamp_sec=stamp,
            position=position,
            speed=speed,
        )

    def _imu_callback(self, message: Imu) -> None:
        stamp = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1.0e-9
        )
        orientation = message.orientation

        # sensor_msgs/Imu 规定 orientation_covariance[0] == -1 时表示设备
        # 没有提供姿态估计。此时仍保留角速度和加速度，但倾角写为 NaN，
        # 避免把默认的全零四元数错误解释为“底盘完全水平”。
        orientation_available = (
            len(message.orientation_covariance) == 0
            or float(message.orientation_covariance[0]) >= 0.0
        )
        if orientation_available:
            roll, pitch, yaw = quaternion_to_rpy(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
            if math.isfinite(roll) and math.isfinite(pitch):
                # tilt 表示 IMU Z 轴相对重力竖直方向的总倾角。
                tilt = math.acos(
                    max(
                        -1.0,
                        min(
                            1.0,
                            1.0
                            - 2.0
                            * (
                                float(orientation.x) ** 2
                                + float(orientation.y) ** 2
                            )
                            / max(
                                float(orientation.x) ** 2
                                + float(orientation.y) ** 2
                                + float(orientation.z) ** 2
                                + float(orientation.w) ** 2,
                                1.0e-24,
                            ),
                        ),
                    )
                )
            else:
                tilt = math.nan
        else:
            roll = math.nan
            pitch = math.nan
            yaw = math.nan
            tilt = math.nan

        self.latest_imu = ImuState(
            stamp_sec=stamp,
            receive_sec=self._now_sec(),
            frame_id=str(message.header.frame_id),
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            tilt=tilt,
            angular_velocity_x=float(message.angular_velocity.x),
            angular_velocity_y=float(message.angular_velocity.y),
            angular_velocity_z=float(message.angular_velocity.z),
            linear_acceleration_x=float(message.linear_acceleration.x),
            linear_acceleration_y=float(message.linear_acceleration.y),
            linear_acceleration_z=float(message.linear_acceleration.z),
        )

    def _robot_state_callback(self, message: String) -> None:
        """解析底盘汇总状态；坏 JSON 只告警，不中断行驶测试。"""
        now_sec = self._now_sec()
        try:
            self.latest_chassis_status = parse_chassis_status_json(
                message.data, now_sec)
        except (json.JSONDecodeError, ValueError, TypeError) as error:
            # 状态话题频率较高，解析异常最多每 5 秒提示一次，避免刷屏。
            if now_sec - self.last_robot_state_parse_warning_sec >= 5.0:
                self.last_robot_state_parse_warning_sec = now_sec
                self._log(
                    "warning",
                    "无法解析底盘状态话题 %s：%s",
                    self.robot_state_topic,
                    error,
                )

    def _lookup_map_pose(self) -> Optional[Pose2D]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except TransformException:
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        stamp = (
            float(transform.header.stamp.sec)
            + float(transform.header.stamp.nanosec) * 1.0e-9
        )
        return Pose2D(
            x=float(translation.x),
            y=float(translation.y),
            yaw=quaternion_to_yaw(
                rotation.x, rotation.y, rotation.z, rotation.w),
            stamp_sec=stamp,
        )

    def _read_test_config(self) -> TestConfig:
        mode = str(self.get_parameter("mode").value)
        requested_angle = float(
            self.get_parameter("translation_angle_deg").value)
        angle_deg = MODE_TO_ANGLE_DEG.get(mode, requested_angle)
        angle_rad = math.radians(angle_deg)
        angular_speed_radps = float(
            self.get_parameter("angular_speed_radps").value)
        rotation_angle_deg = float(
            self.get_parameter("rotation_angle_deg").value)
        rotation_direction = ROTATION_MODE_TO_SIGN.get(mode, 0.0)
        duration_sec = float(self.get_parameter("duration_sec").value)
        speed_mps = float(self.get_parameter("speed_mps").value)
        if mode in ROTATION_MODE_TO_SIGN:
            # 原地旋转使用“角度÷角速度”自动计算命令持续时间。
            duration_sec = (
                math.radians(rotation_angle_deg) / angular_speed_radps)
            speed_mps = 0.0
        return TestConfig(
            mode=mode,
            speed_mps=speed_mps,
            angular_speed_radps=(
                angular_speed_radps
                if mode in ROTATION_MODE_TO_SIGN
                else 0.0
            ),
            duration_sec=duration_sec,
            angle_deg=angle_deg,
            direction_x=math.cos(angle_rad),
            direction_y=math.sin(angle_rad),
            rotation_direction=rotation_direction,
            rotation_angle_deg=(
                rotation_angle_deg
                if mode in ROTATION_MODE_TO_SIGN
                else 0.0
            ),
            settle_time_sec=float(
                self.get_parameter("settle_time_sec").value),
            log_period_sec=float(self.get_parameter("log_period_sec").value),
            overshoot_threshold_m=float(
                self.get_parameter("overshoot_threshold_m").value),
            rotation_overshoot_threshold_deg=float(
                self.get_parameter(
                    "rotation_overshoot_threshold_deg").value),
        )

    def _config_from_loop_step(self, step: LoopStep) -> TestConfig:
        """把用户循环步骤转换为一次可直接执行的测试配置。"""
        settle_time_sec = float(
            self.get_parameter("settle_time_sec").value)
        log_period_sec = float(
            self.get_parameter("log_period_sec").value)
        overshoot_threshold_m = float(
            self.get_parameter("overshoot_threshold_m").value)
        rotation_overshoot_threshold_deg = float(
            self.get_parameter("rotation_overshoot_threshold_deg").value)

        if step.mode in ROTATION_MODE_TO_SIGN:
            return TestConfig(
                mode=step.mode,
                speed_mps=0.0,
                angular_speed_radps=step.speed,
                duration_sec=math.radians(step.amount) / step.speed,
                angle_deg=0.0,
                direction_x=0.0,
                direction_y=0.0,
                rotation_direction=ROTATION_MODE_TO_SIGN[step.mode],
                rotation_angle_deg=step.amount,
                settle_time_sec=settle_time_sec,
                log_period_sec=log_period_sec,
                overshoot_threshold_m=overshoot_threshold_m,
                rotation_overshoot_threshold_deg=(
                    rotation_overshoot_threshold_deg),
            )

        angle_deg = MODE_TO_ANGLE_DEG.get(
            step.mode, step.translation_angle_deg)
        angle_rad = math.radians(angle_deg)
        return TestConfig(
            mode=step.mode,
            speed_mps=step.speed,
            angular_speed_radps=0.0,
            duration_sec=step.amount / step.speed,
            angle_deg=angle_deg,
            direction_x=math.cos(angle_rad),
            direction_y=math.sin(angle_rad),
            rotation_direction=0.0,
            rotation_angle_deg=0.0,
            settle_time_sec=settle_time_sec,
            log_period_sec=log_period_sec,
            overshoot_threshold_m=overshoot_threshold_m,
            rotation_overshoot_threshold_deg=(
                rotation_overshoot_threshold_deg),
        )

    def _prepare_loop(self) -> None:
        """读取循环参数并生成有限步骤队列；这里只准备，不立即发速度。"""
        step_texts = [
            str(item) for item in self.get_parameter("loop_steps").value]
        loop_count = int(self.get_parameter("loop_count").value)
        parsed_steps = [parse_loop_step(text) for text in step_texts]

        self.loop_queue = []
        for _ in range(loop_count):
            self.loop_queue.extend(
                self._config_from_loop_step(step)
                for step in parsed_steps
            )
        self.loop_total_steps = len(self.loop_queue)
        self.loop_completed_steps = 0
        self.loop_pause_until_sec = self._now_sec()
        self.loop_session_name = (
            "loop_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
        self.loop_result_dirs = []
        self.loop_active = True
        self.loop_start_requested = False
        self.set_parameters(
            [Parameter("run_loop", Parameter.Type.BOOL, False)])
        self._log(
            "info",
            "循环测试已准备：基础步骤=%d，循环次数=%d，总步骤=%d，"
            "步骤间零速等待=%.3fs",
            len(parsed_steps),
            loop_count,
            self.loop_total_steps,
            float(self.get_parameter("loop_pause_sec").value),
        )
        for index, text in enumerate(step_texts, start=1):
            self._log("info", "循环基础步骤 %d：%s", index, text)

    def _finish_loop_session(self, canceled_reason: str = "") -> None:
        """结束循环并保存一份步骤目录索引。"""
        if not self.loop_session_name:
            # run_loop 刚触发、但循环配置尚未准备完成时也可能收到取消。
            # 此处必须清除等待标记，否则下一次定时器回调还会重新启动循环。
            self.loop_active = False
            self.loop_start_requested = False
            self.loop_queue.clear()
            self.pending_config = None
            return
        status = "canceled" if canceled_reason else "completed"
        summary_path = (
            self.output_dir / f"{self.loop_session_name}_summary.txt")
        summary_lines = [
            "全向底盘循环测试汇总",
            f"status: {status}",
            f"completed_steps: {self.loop_completed_steps}",
            f"total_steps: {self.loop_total_steps}",
        ]
        if canceled_reason:
            summary_lines.append(f"reason: {canceled_reason}")
        summary_lines.append("result_directories:")
        summary_lines.extend(
            f"  - {directory}" for directory in self.loop_result_dirs)
        summary_path.write_text(
            "\n".join(summary_lines) + "\n", encoding="utf-8")
        self._log(
            "warning" if canceled_reason else "info",
            "循环测试%s：完成 %d/%d 步，汇总=%s",
            "已取消" if canceled_reason else "已完成",
            self.loop_completed_steps,
            self.loop_total_steps,
            summary_path,
        )
        self.loop_active = False
        self.loop_start_requested = False
        self.loop_queue.clear()
        self.pending_config = None
        self.loop_session_name = ""
        self.loop_total_steps = 0
        self.loop_completed_steps = 0
        self.loop_result_dirs = []

    def _external_command_publishers(self) -> List[str]:
        """返回除本脚本外仍在发布底盘命令的节点，帮助发现话题抢占。"""
        names: List[str] = []
        try:
            for info in self.get_publishers_info_by_topic(self.command_topic):
                if info.node_name != self.get_name():
                    names.append(f"{info.node_namespace}/{info.node_name}")
        except RuntimeError:
            pass
        return sorted(set(names))

    def _begin_test(self) -> None:
        pose = self._lookup_map_pose()
        now_sec = self._now_sec()
        if pose is None or self.latest_odom is None:
            if now_sec - self.last_wait_warning_sec >= 2.0:
                self.last_wait_warning_sec = now_sec
                self._log(
                    "warning",
                    "等待启动条件：需要 TF %s -> %s 和 %s 消息",
                    self.map_frame,
                    self.base_frame,
                    self.odom_topic,
                )
            return

        config = self.pending_config or self._read_test_config()
        self.pending_config = None
        travel_distance = config.speed_mps * config.duration_sec
        if config.is_rotation:
            target = Pose2D(
                x=pose.x,
                y=pose.y,
                yaw=normalize_angle(
                    pose.yaw
                    + config.rotation_direction
                    * math.radians(config.rotation_angle_deg)
                ),
                stamp_sec=pose.stamp_sec,
            )
        else:
            target = Pose2D(
                x=pose.x + config.direction_x * travel_distance,
                y=pose.y + config.direction_y * travel_distance,
                yaw=pose.yaw,
                stamp_sec=pose.stamp_sec,
            )

        loop_prefix = ""
        if self.loop_active:
            loop_prefix = (
                f"_{self.loop_session_name}_"
                f"step{self.loop_completed_steps + 1:04d}")
        speed_label = (
            f"{config.angular_speed_radps:.3f}radps"
            if config.is_rotation
            else f"{config.speed_mps:.3f}mps"
        )
        run_name = (
            datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            + loop_prefix
            + f"_{config.mode}_{speed_label}"
        )
        self.run_dir = self.output_dir / run_name
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self._update_latest_link(self.run_dir)

        self.config = config
        self.start_pose = pose
        self.stop_pose = None
        self.target_pose = target
        self.test_start_sec = now_sec
        self.stop_start_sec = 0.0
        self.last_status_log_sec = -math.inf
        self.samples.clear()
        self.last_rotation_pose_yaw = pose.yaw
        self.rotation_progress_rad = 0.0
        self.state = "running"
        self.start_requested = False
        self.set_parameters(
            [Parameter("run_test", Parameter.Type.BOOL, False)])

        external_publishers = self._external_command_publishers()
        if external_publishers:
            self._log(
                "warning",
                "检测到其他节点也在发布 %s：%s；测试命令可能被覆盖",
                self.command_topic,
                ", ".join(external_publishers),
            )

        self._log("info", "=" * 88)
        if config.is_rotation:
            self._log(
                "info",
                "原地旋转测试开始：mode=%s，角速度=%.3f rad/s，"
                "目标角度=%.3f deg，自动计算时间=%.3f s",
                config.mode,
                config.angular_speed_radps,
                config.rotation_angle_deg,
                config.duration_sec,
            )
        else:
            self._log(
                "info",
                "平移测试开始：mode=%s，速度=%.3f m/s，时间=%.3f s，"
                "map 平移角=%.3f deg，理论距离=%.3f m",
                config.mode,
                config.speed_mps,
                config.duration_sec,
                config.angle_deg,
                travel_distance,
            )
        self._log(
            "info",
            "起点(map)：x=%.4f，y=%.4f，yaw=%.3f deg",
            pose.x,
            pose.y,
            math.degrees(pose.yaw),
        )
        self._log(
            "info",
            "理论终点(map)：x=%.4f，y=%.4f，yaw=%.3f deg",
            target.x,
            target.y,
            math.degrees(target.yaw),
        )
        if self.latest_imu is None:
            self._log(
                "warning",
                "尚未收到 IMU：%s；测试可以继续，但本次结果不会包含有效倾角",
                self.imu_topic,
            )
        else:
            self._log(
                "info",
                "IMU 起始姿态(%s)：roll=%.3fdeg，pitch=%.3fdeg，"
                "总倾角=%.3fdeg",
                self.latest_imu.frame_id or "frame_id为空",
                math.degrees(self.latest_imu.roll),
                math.degrees(self.latest_imu.pitch),
                math.degrees(self.latest_imu.tilt),
            )
        if self.latest_chassis_status is None:
            self._log(
                "warning",
                "尚未收到底盘汇总状态：%s；测试可以继续，但本次不会包含"
                "电流、温度和故障状态",
                self.robot_state_topic,
            )
        elif not self.latest_chassis_status.has_valid_temperature():
            self._log(
                "warning",
                "底盘状态已收到，但三个温度字段均为 0；"
                "按当前机器人实际表现，这通常表示温度未上报，"
                "脚本会保留原始值并标记为无效",
            )
        self._log("info", "本次数据目录：%s", self.run_dir)

    def _update_latest_link(self, run_dir: Path) -> None:
        latest_link = self.output_dir / "latest"
        try:
            if latest_link.is_symlink():
                latest_link.unlink()
            if not latest_link.exists():
                latest_link.symlink_to(run_dir.name, target_is_directory=True)
        except OSError as error:
            self._log("warning", "无法更新 latest 软链接：%s", error)

    def _publish_zero(self) -> None:
        if not rclpy.ok():
            return
        try:
            self.command_pub.publish(Twist())
        except RuntimeError as error:
            # 节点关闭过程中通信上下文可能已经失效，此时不能再发布消息。
            self._log("warning", "零速发布失败：%s", error)

    def _publish_map_velocity(
        self, pose: Pose2D, vx_map: float, vy_map: float, wz: float = 0.0
    ) -> Tuple[float, float]:
        """将 map 固定方向速度转换到底盘坐标系并附带旋转速度后发布。

        map -> base 的二维速度旋转：
          vx_base =  cos(yaw) * vx_map + sin(yaw) * vy_map
          vy_base = -sin(yaw) * vx_map + cos(yaw) * vy_map
        """
        cos_yaw = math.cos(pose.yaw)
        sin_yaw = math.sin(pose.yaw)
        vx_base = cos_yaw * vx_map + sin_yaw * vy_map
        vy_base = -sin_yaw * vx_map + cos_yaw * vy_map

        message = Twist()
        message.linear.x = vx_base
        message.linear.y = vy_base
        # 绕 Z 轴角速度在 map/base 的平面旋转下数值相同。
        message.angular.z = wz
        self.command_pub.publish(message)
        return vx_base, vy_base

    def _timer_callback(self) -> None:
        if self.cancel_requested:
            self.cancel_requested = False
            self.set_parameters(
                [Parameter("cancel_test", Parameter.Type.BOOL, False)])
            if self.state != "idle":
                self._abort_test("收到 cancel_test=true")
            elif self.loop_active or self.loop_start_requested:
                self._publish_zero()
                self._finish_loop_session("收到 cancel_test=true")
            elif self.start_requested:
                # run_test 已触发但 TF/odom 尚未满足时也允许取消，避免条件
                # 稍后恢复后车辆在用户不知情的情况下突然启动。
                self.start_requested = False
                self.pending_config = None
                self.set_parameters(
                    [Parameter("run_test", Parameter.Type.BOOL, False)])
                self._publish_zero()
                self._log("warning", "已取消尚未开始的单次测试")
            else:
                self._publish_zero()

        if self.state == "idle":
            if self.loop_start_requested:
                try:
                    self._prepare_loop()
                except (TypeError, ValueError) as error:
                    self.loop_start_requested = False
                    self.set_parameters(
                        [Parameter("run_loop", Parameter.Type.BOOL, False)])
                    self._log("error", "循环参数无效，未启动车辆：%s", error)
                    return

            if self.loop_active:
                if (
                    self.pending_config is None
                    and self.loop_queue
                    and self._now_sec() >= self.loop_pause_until_sec
                ):
                    self.pending_config = self.loop_queue.pop(0)
                if self.pending_config is not None:
                    self._begin_test()
                elif not self.loop_queue:
                    self._finish_loop_session()
                return

            if self.start_requested:
                self._begin_test()
            return

        assert self.config is not None
        assert self.start_pose is not None
        assert self.target_pose is not None

        pose = self._lookup_map_pose()
        if pose is None:
            self._abort_test(
                f"运行中丢失 TF {self.map_frame} -> {self.base_frame}")
            return

        now_sec = self._now_sec()
        elapsed_sec = now_sec - self.test_start_sec
        command_vx_map = 0.0
        command_vy_map = 0.0
        command_vx_base = 0.0
        command_vy_base = 0.0
        command_wz = 0.0

        if self.state == "running":
            if elapsed_sec < self.config.duration_sec:
                if self.config.is_rotation:
                    command_wz = (
                        self.config.rotation_direction
                        * self.config.angular_speed_radps
                    )
                else:
                    command_vx_map = (
                        self.config.speed_mps * self.config.direction_x)
                    command_vy_map = (
                        self.config.speed_mps * self.config.direction_y)
                command_vx_base, command_vy_base = self._publish_map_velocity(
                    pose, command_vx_map, command_vy_map, command_wz)
            else:
                self._publish_zero()
                self.stop_pose = pose
                self.stop_start_sec = now_sec
                self.state = "settling"
                self._log_command_end_error(pose)
        else:
            # 停止观察阶段仍周期性发布零速，避免其他测试残留命令继续生效。
            self._publish_zero()

        self._record_sample(
            now_sec=now_sec,
            elapsed_sec=elapsed_sec,
            pose=pose,
            command_vx_map=command_vx_map,
            command_vy_map=command_vy_map,
            command_vx_base=command_vx_base,
            command_vy_base=command_vy_base,
            command_wz=command_wz,
        )

        if (
            self.verbose
            and elapsed_sec - self.last_status_log_sec
            >= self.config.log_period_sec
        ):
            self.last_status_log_sec = elapsed_sec
            self._log_verbose_status(elapsed_sec, pose)

        if (
            self.state == "settling"
            and now_sec - self.stop_start_sec >= self.config.settle_time_sec
        ):
            self._finish_test(pose)

    def _theoretical_pose(self, elapsed_sec: float) -> Pose2D:
        assert self.config is not None
        assert self.start_pose is not None
        moving_time = min(max(elapsed_sec, 0.0), self.config.duration_sec)
        if self.config.is_rotation:
            return Pose2D(
                x=self.start_pose.x,
                y=self.start_pose.y,
                yaw=normalize_angle(
                    self.start_pose.yaw
                    + self.config.rotation_direction
                    * self.config.angular_speed_radps
                    * moving_time
                ),
                stamp_sec=0.0,
            )
        distance = self.config.speed_mps * moving_time
        return Pose2D(
            x=self.start_pose.x + self.config.direction_x * distance,
            y=self.start_pose.y + self.config.direction_y * distance,
            yaw=self.start_pose.yaw,
            stamp_sec=0.0,
        )

    def _record_sample(
        self,
        now_sec: float,
        elapsed_sec: float,
        pose: Pose2D,
        command_vx_map: float,
        command_vy_map: float,
        command_vx_base: float,
        command_vy_base: float,
        command_wz: float,
    ) -> None:
        theoretical = self._theoretical_pose(elapsed_sec)
        odom = self.latest_odom
        motor = self.latest_motor
        imu = self.latest_imu
        chassis_status = self.latest_chassis_status

        if self.config and self.config.is_rotation:
            yaw_increment = normalize_angle(
                pose.yaw - self.last_rotation_pose_yaw)
            self.rotation_progress_rad += (
                self.config.rotation_direction * yaw_increment)
            self.last_rotation_pose_yaw = pose.yaw

        odom_vx_map = math.nan
        odom_vy_map = math.nan
        odom_speed = math.nan
        if odom is not None:
            cos_yaw = math.cos(pose.yaw)
            sin_yaw = math.sin(pose.yaw)
            odom_vx_map = (
                cos_yaw * odom.vx_base - sin_yaw * odom.vy_base)
            odom_vy_map = (
                sin_yaw * odom.vx_base + cos_yaw * odom.vy_base)
            odom_speed = math.hypot(odom.vx_base, odom.vy_base)

        row: Dict[str, float | str] = {
            "time_sec": elapsed_sec,
            "phase": self.state,
            "map_x": pose.x,
            "map_y": pose.y,
            "map_yaw_rad": pose.yaw,
            "theory_x": theoretical.x,
            "theory_y": theoretical.y,
            "theory_yaw_rad": theoretical.yaw,
            "target_error_x": pose.x - self.target_pose.x,
            "target_error_y": pose.y - self.target_pose.y,
            "target_error_yaw_rad": normalize_angle(
                pose.yaw - self.target_pose.yaw),
            "cmd_vx_map": command_vx_map,
            "cmd_vy_map": command_vy_map,
            "cmd_vx_base": command_vx_base,
            "cmd_vy_base": command_vy_base,
            "cmd_wz": command_wz,
            # 原地旋转进度按指定方向累计，可正确跨越 ±pi 边界。
            "rotation_progress_rad": self.rotation_progress_rad,
            "odom_stamp_sec": odom.stamp_sec if odom else math.nan,
            "odom_age_sec": (
                now_sec - odom.stamp_sec
                if odom and odom.stamp_sec > 0.0
                else math.nan
            ),
            "odom_x": odom.pose_x if odom else math.nan,
            "odom_y": odom.pose_y if odom else math.nan,
            "odom_yaw_rad": odom.pose_yaw if odom else math.nan,
            "odom_vx_base": odom.vx_base if odom else math.nan,
            "odom_vy_base": odom.vy_base if odom else math.nan,
            "odom_wz": odom.wz if odom else math.nan,
            "odom_vx_map": odom_vx_map,
            "odom_vy_map": odom_vy_map,
            "odom_speed": odom_speed,
            "imu_stamp_sec": imu.stamp_sec if imu else math.nan,
            "imu_receive_age_sec": (
                now_sec - imu.receive_sec if imu else math.nan),
            "imu_frame_id": imu.frame_id if imu else "",
            "imu_roll_rad": imu.roll if imu else math.nan,
            "imu_pitch_rad": imu.pitch if imu else math.nan,
            "imu_yaw_rad": imu.yaw if imu else math.nan,
            "imu_tilt_rad": imu.tilt if imu else math.nan,
            "imu_roll_deg": (
                math.degrees(imu.roll) if imu else math.nan),
            "imu_pitch_deg": (
                math.degrees(imu.pitch) if imu else math.nan),
            "imu_yaw_deg": (
                math.degrees(imu.yaw) if imu else math.nan),
            "imu_tilt_deg": (
                math.degrees(imu.tilt) if imu else math.nan),
            "imu_angular_velocity_x": (
                imu.angular_velocity_x if imu else math.nan),
            "imu_angular_velocity_y": (
                imu.angular_velocity_y if imu else math.nan),
            "imu_angular_velocity_z": (
                imu.angular_velocity_z if imu else math.nan),
            "imu_linear_acceleration_x": (
                imu.linear_acceleration_x if imu else math.nan),
            "imu_linear_acceleration_y": (
                imu.linear_acceleration_y if imu else math.nan),
            "imu_linear_acceleration_z": (
                imu.linear_acceleration_z if imu else math.nan),
            # robot_server_state 是左右侧汇总状态，不是八个电机的逐电机数据。
            # CSV 保留原始数值和有效性标记。以后底层真正开始上报温度时，
            # 无需再修改数据格式。
            "robot_state_stamp_sec": (
                chassis_status.stamp_sec
                if chassis_status
                else math.nan
            ),
            "robot_state_receive_age_sec": (
                now_sec - chassis_status.receive_sec
                if chassis_status
                else math.nan
            ),
            "chassis_status_word": (
                chassis_status.status_word
                if chassis_status
                else -1
            ),
            "chassis_error_code_left": (
                chassis_status.error_code_left
                if chassis_status
                else -1
            ),
            "chassis_error_code_right": (
                chassis_status.error_code_right
                if chassis_status
                else -1
            ),
            "chassis_velocity_left": (
                chassis_status.velocity_left
                if chassis_status
                else math.nan
            ),
            "chassis_velocity_right": (
                chassis_status.velocity_right
                if chassis_status
                else math.nan
            ),
            "chassis_current_left": (
                chassis_status.current_left
                if chassis_status
                else math.nan
            ),
            "chassis_current_right": (
                chassis_status.current_right
                if chassis_status
                else math.nan
            ),
            "chassis_driver_temperature": (
                chassis_status.driver_temperature
                if chassis_status
                else math.nan
            ),
            "chassis_temperature_left": (
                chassis_status.temperature_left
                if chassis_status
                else math.nan
            ),
            "chassis_temperature_right": (
                chassis_status.temperature_right
                if chassis_status
                else math.nan
            ),
            "chassis_temperature_valid": (
                1.0
                if (
                    chassis_status
                    and chassis_status.has_valid_temperature()
                )
                else 0.0
            ),
        }

        for index, wheel_name in enumerate(WHEEL_NAMES):
            row[f"{wheel_name}_position"] = (
                motor.position[index] if motor else math.nan)
            row[f"{wheel_name}_speed"] = (
                motor.speed[index] if motor else math.nan)
        self.samples.append(row)

    def _log_verbose_status(self, elapsed_sec: float, pose: Pose2D) -> None:
        assert self.config is not None
        assert self.target_pose is not None
        theoretical = self._theoretical_pose(elapsed_sec)
        odom = self.latest_odom
        motor = self.latest_motor
        imu = self.latest_imu
        chassis_status = self.latest_chassis_status

        odom_text = "未收到"
        if odom is not None:
            odom_text = (
                f"base_v=({odom.vx_base:.3f},{odom.vy_base:.3f})m/s，"
                f"speed={math.hypot(odom.vx_base, odom.vy_base):.3f}m/s，"
                f"wz={odom.wz:.3f}rad/s"
            )

        motor_text = "未收到"
        if motor is not None:
            position = ",".join(f"{value:.3f}" for value in motor.position)
            speed = ",".join(f"{value:.2f}" for value in motor.speed)
            motor_text = f"position=[{position}]，speed=[{speed}]"

        imu_text = "未收到"
        if imu is not None:
            imu_text = (
                f"frame={imu.frame_id or '空'}，"
                f"roll={math.degrees(imu.roll):.2f}deg，"
                f"pitch={math.degrees(imu.pitch):.2f}deg，"
                f"tilt={math.degrees(imu.tilt):.2f}deg，"
                f"acc=({imu.linear_acceleration_x:.2f},"
                f"{imu.linear_acceleration_y:.2f},"
                f"{imu.linear_acceleration_z:.2f})m/s^2"
            )

        chassis_status_text = "未收到"
        if chassis_status is not None:
            temperature_mark = (
                "有效"
                if chassis_status.has_valid_temperature()
                else "全零/疑似未上报"
            )
            chassis_status_text = (
                f"status={chassis_status.status_word}，"
                f"error(L/R)=({chassis_status.error_code_left}/"
                f"{chassis_status.error_code_right})，"
                f"current(L/R)=({chassis_status.current_left:.2f}/"
                f"{chassis_status.current_right:.2f})A，"
                f"temperature(driver/L/R)=("
                f"{chassis_status.driver_temperature:.1f}/"
                f"{chassis_status.temperature_left:.1f}/"
                f"{chassis_status.temperature_right:.1f})degC"
                f"[{temperature_mark}]"
            )

        if self.config.is_rotation:
            motion_text = (
                f"原地旋转角速度={self.config.angular_speed_radps:.3f}rad/s，"
                f"目标旋转={self.config.rotation_angle_deg:.2f}deg，"
                f"累计旋转={math.degrees(self.rotation_progress_rad):.2f}deg"
            )
        else:
            motion_text = (
                f"平移速度={self.config.speed_mps:.3f}m/s，"
                f"map角={self.config.angle_deg:.2f}deg"
            )

        self._log(
            "info",
            "状态=%s，t=%.3f/%.3fs，%s；"
            "当前位置=(%.4f,%.4f,%.2fdeg)，"
            "理论当前位置=(%.4f,%.4f,%.2fdeg)，"
            "理论终点=(%.4f,%.4f,%.2fdeg)，终点误差=(dx=%.4f,dy=%.4f,"
            "dyaw=%.2fdeg)；odom=%s；imu=%s；motors=%s；chassis=%s",
            self.state,
            elapsed_sec,
            self.config.duration_sec,
            motion_text,
            pose.x,
            pose.y,
            math.degrees(pose.yaw),
            theoretical.x,
            theoretical.y,
            math.degrees(theoretical.yaw),
            self.target_pose.x,
            self.target_pose.y,
            math.degrees(self.target_pose.yaw),
            pose.x - self.target_pose.x,
            pose.y - self.target_pose.y,
            math.degrees(normalize_angle(
                pose.yaw - self.target_pose.yaw)),
            odom_text,
            imu_text,
            motor_text,
            chassis_status_text,
        )

    def _log_command_end_error(self, pose: Pose2D) -> None:
        assert self.target_pose is not None
        self._log(
            "info",
            "运动命令结束：当前位置(map)=(%.4f,%.4f,%.2fdeg)，"
            "相对理论终点误差=(dx=%.4f,dy=%.4f,dyaw=%.2fdeg)；"
            "继续采样以观察停止过冲",
            pose.x,
            pose.y,
            math.degrees(pose.yaw),
            pose.x - self.target_pose.x,
            pose.y - self.target_pose.y,
            math.degrees(normalize_angle(
                pose.yaw - self.target_pose.yaw)),
        )

    def _finish_test(self, final_pose: Pose2D) -> None:
        assert self.config is not None
        assert self.start_pose is not None
        assert self.target_pose is not None
        assert self.run_dir is not None

        finished_run_dir = self.run_dir
        self._publish_zero()
        dx = final_pose.x - self.target_pose.x
        dy = final_pose.y - self.target_pose.y
        dyaw = normalize_angle(final_pose.yaw - self.target_pose.yaw)
        distance_error = math.hypot(dx, dy)

        # 平移与旋转使用不同的进度量。旋转不能只比较归一化后的最终 yaw，
        # 否则跨越 ±180° 时会丢失实际转过的角度，因此使用采样时累计的进度。
        expected_distance = 0.0
        along_error = 0.0
        cross_error = 0.0
        overshoot = 0.0
        actual_rotation_deg = 0.0
        rotation_error_deg = 0.0
        rotation_overshoot_deg = 0.0
        if self.config.is_rotation:
            expected_rotation_deg = self.config.rotation_angle_deg
            actual_rotation_deg = math.degrees(self.rotation_progress_rad)
            rotation_error_deg = (
                actual_rotation_deg - expected_rotation_deg)
            max_rotation_deg = max(
                (
                    math.degrees(float(row["rotation_progress_rad"]))
                    for row in self.samples
                ),
                default=actual_rotation_deg,
            )
            rotation_overshoot_deg = max(
                0.0, max_rotation_deg - expected_rotation_deg)
            overshoot_detected = (
                rotation_overshoot_deg
                > self.config.rotation_overshoot_threshold_deg
            )
        else:
            final_progress = (
                (final_pose.x - self.start_pose.x) * self.config.direction_x
                + (final_pose.y - self.start_pose.y) * self.config.direction_y
            )
            expected_distance = (
                self.config.speed_mps * self.config.duration_sec)
            along_error = final_progress - expected_distance
            cross_error = (
                -(final_pose.x - self.start_pose.x) * self.config.direction_y
                + (final_pose.y - self.start_pose.y)
                * self.config.direction_x
            )
            max_progress = max(
                (
                    (float(row["map_x"]) - self.start_pose.x)
                    * self.config.direction_x
                    + (float(row["map_y"]) - self.start_pose.y)
                    * self.config.direction_y
                )
                for row in self.samples
            )
            overshoot = max(0.0, max_progress - expected_distance)
            overshoot_detected = (
                overshoot > self.config.overshoot_threshold_m)

        imu_roll_deg = [
            float(row["imu_roll_deg"])
            for row in self.samples
            if math.isfinite(float(row["imu_roll_deg"]))
        ]
        imu_pitch_deg = [
            float(row["imu_pitch_deg"])
            for row in self.samples
            if math.isfinite(float(row["imu_pitch_deg"]))
        ]
        imu_tilt_deg = [
            float(row["imu_tilt_deg"])
            for row in self.samples
            if math.isfinite(float(row["imu_tilt_deg"]))
        ]
        chassis_current_left = [
            float(row["chassis_current_left"])
            for row in self.samples
            if math.isfinite(float(row["chassis_current_left"]))
        ]
        chassis_current_right = [
            float(row["chassis_current_right"])
            for row in self.samples
            if math.isfinite(float(row["chassis_current_right"]))
        ]
        chassis_status_rows = [
            row
            for row in self.samples
            if int(float(row["chassis_status_word"])) >= 0
        ]
        # 当前机器人返回的温度长期全为 0。至少一个温度字段非零时，
        # 才纳入温度统计，避免报告出误导性的“最高温度 0°C”。
        valid_temperature_rows = [
            row
            for row in self.samples
            if float(row["chassis_temperature_valid"]) > 0.5
        ]
        valid_temperatures = [
            float(row[key])
            for row in valid_temperature_rows
            for key in (
                "chassis_driver_temperature",
                "chassis_temperature_left",
                "chassis_temperature_right",
            )
            if math.isfinite(float(row[key]))
        ]

        summary_lines = [
            "全向底盘测试结果",
            f"mode: {self.config.mode}",
            f"duration_sec: {self.config.duration_sec:.6f}",
            (
                "start_map: "
                f"x={self.start_pose.x:.6f}, y={self.start_pose.y:.6f}, "
                f"yaw_deg={math.degrees(self.start_pose.yaw):.6f}"
            ),
            (
                "target_map: "
                f"x={self.target_pose.x:.6f}, y={self.target_pose.y:.6f}, "
                f"yaw_deg={math.degrees(self.target_pose.yaw):.6f}"
            ),
            (
                "final_map: "
                f"x={final_pose.x:.6f}, y={final_pose.y:.6f}, "
                f"yaw_deg={math.degrees(final_pose.yaw):.6f}"
            ),
            f"final_error_x_m: {dx:.6f}",
            f"final_error_y_m: {dy:.6f}",
            f"final_error_yaw_deg: {math.degrees(dyaw):.6f}",
            f"final_position_error_m: {distance_error:.6f}",
        ]
        if self.config.is_rotation:
            summary_lines.extend(
                [
                    (
                        "angular_speed_radps: "
                        f"{self.config.angular_speed_radps:.6f}"
                    ),
                    (
                        "expected_rotation_deg: "
                        f"{self.config.rotation_angle_deg:.6f}"
                    ),
                    f"actual_rotation_deg: {actual_rotation_deg:.6f}",
                    f"rotation_error_deg: {rotation_error_deg:.6f}",
                    f"position_drift_m: {distance_error:.6f}",
                    (
                        "maximum_rotation_overshoot_deg: "
                        f"{rotation_overshoot_deg:.6f}"
                    ),
                    (
                        "rotation_overshoot_threshold_deg: "
                        f"{self.config.rotation_overshoot_threshold_deg:.6f}"
                    ),
                    f"overshoot_detected: {overshoot_detected}",
                ]
            )
        else:
            summary_lines.extend(
                [
                    f"speed_mps: {self.config.speed_mps:.6f}",
                    (
                        "translation_angle_deg: "
                        f"{self.config.angle_deg:.6f}"
                    ),
                    f"expected_distance_m: {expected_distance:.6f}",
                    f"along_track_error_m: {along_error:.6f}",
                    f"cross_track_error_m: {cross_error:.6f}",
                    f"maximum_overshoot_m: {overshoot:.6f}",
                    f"overshoot_detected: {overshoot_detected}",
                ]
            )
        summary_lines.append(
            "note: map 下误差同时包含轮式里程计误差、轮胎打滑以及 "
            "SLAM/定位对 map->odom 的修正，不能单独等同于纯里程计误差。"
        )
        if imu_roll_deg and imu_pitch_deg and imu_tilt_deg:
            start_roll_deg = imu_roll_deg[0]
            start_pitch_deg = imu_pitch_deg[0]
            relative_roll_deg = [
                math.degrees(
                    normalize_angle(math.radians(value - start_roll_deg)))
                for value in imu_roll_deg
            ]
            relative_pitch_deg = [
                math.degrees(
                    normalize_angle(math.radians(value - start_pitch_deg)))
                for value in imu_pitch_deg
            ]
            summary_lines.extend(
                [
                    f"imu_topic: {self.imu_topic}",
                    f"imu_start_roll_deg: {start_roll_deg:.6f}",
                    f"imu_start_pitch_deg: {start_pitch_deg:.6f}",
                    f"imu_final_roll_deg: {imu_roll_deg[-1]:.6f}",
                    f"imu_final_pitch_deg: {imu_pitch_deg[-1]:.6f}",
                    (
                        "imu_max_abs_roll_change_deg: "
                        f"{max(abs(value) for value in relative_roll_deg):.6f}"
                    ),
                    (
                        "imu_max_abs_pitch_change_deg: "
                        f"{max(abs(value) for value in relative_pitch_deg):.6f}"
                    ),
                    (
                        "imu_roll_peak_to_peak_deg: "
                        f"{max(imu_roll_deg) - min(imu_roll_deg):.6f}"
                    ),
                    (
                        "imu_pitch_peak_to_peak_deg: "
                        f"{max(imu_pitch_deg) - min(imu_pitch_deg):.6f}"
                    ),
                    f"imu_max_total_tilt_deg: {max(imu_tilt_deg):.6f}",
                    (
                        "imu_note: roll/pitch 是 IMU 自身坐标系相对其姿态参考系"
                        "的角度；若安装不水平，应重点看相对起始值变化。"
                    ),
                ]
            )
        else:
            summary_lines.extend(
                [
                    f"imu_topic: {self.imu_topic}",
                    "imu_status: no_valid_orientation",
                    (
                        "imu_note: 未收到有效姿态，或消息将 "
                        "orientation_covariance[0] 设为 -1。"
                    ),
                ]
            )
        summary_lines.extend(
            [
                f"chassis_status_topic: {self.robot_state_topic}",
                (
                    "chassis_torque_status: unavailable_not_published "
                    "(当前机器人状态通道没有底盘力矩字段)"
                ),
                (
                    "chassis_data_scope: left/right aggregate compatibility "
                    "fields, not per-eight-motor data"
                ),
            ]
        )
        if chassis_status_rows:
            final_chassis_status = chassis_status_rows[-1]
            summary_lines.extend(
                [
                    (
                        "chassis_final_status_word: "
                        f"{int(float(final_chassis_status['chassis_status_word']))}"
                    ),
                    (
                        "chassis_final_error_code_left: "
                        f"{int(float(final_chassis_status['chassis_error_code_left']))}"
                    ),
                    (
                        "chassis_final_error_code_right: "
                        f"{int(float(final_chassis_status['chassis_error_code_right']))}"
                    ),
                ]
            )
        else:
            summary_lines.append("chassis_status: no_valid_message")

        if chassis_current_left and chassis_current_right:
            summary_lines.extend(
                [
                    (
                        "chassis_max_abs_current_left_a: "
                        f"{max(abs(value) for value in chassis_current_left):.6f}"
                    ),
                    (
                        "chassis_max_abs_current_right_a: "
                        f"{max(abs(value) for value in chassis_current_right):.6f}"
                    ),
                ]
            )
        else:
            summary_lines.append("chassis_current_status: no_valid_sample")

        if valid_temperatures:
            summary_lines.extend(
                [
                    "chassis_temperature_status: valid_nonzero_sample",
                    (
                        "chassis_max_reported_temperature_deg_c: "
                        f"{max(valid_temperatures):.6f}"
                    ),
                ]
            )
        else:
            summary_lines.extend(
                [
                    "chassis_temperature_status: unavailable_or_all_zero",
                    (
                        "chassis_temperature_note: 三个温度字段未收到有效非零值；"
                        "CSV 仍保留原始值，但不把全零解释为真实 0°C。"
                    ),
                ]
            )
        summary_path = self.run_dir / "summary.txt"
        summary_path.write_text(
            "\n".join(summary_lines) + "\n", encoding="utf-8")

        self._write_csv(self.run_dir / "samples.csv")
        self._write_plots()

        self._log("info", "-" * 88)
        self._log(
            "info",
            "测试完成：最终位置(map)=(%.4f,%.4f,%.2fdeg)",
            final_pose.x,
            final_pose.y,
            math.degrees(final_pose.yaw),
        )
        self._log(
            "info",
            "相对理论终点：dx=%.4fm，dy=%.4fm，dyaw=%.3fdeg，"
            "位置误差=%.4fm",
            dx,
            dy,
            math.degrees(dyaw),
            distance_error,
        )
        if self.config.is_rotation:
            self._log(
                "info",
                "旋转结果：目标=%.3fdeg，累计实际=%.3fdeg，"
                "角度误差=%.3fdeg，最大旋转过冲=%.3fdeg，"
                "位置漂移=%.4fm，是否超过阈值=%s",
                self.config.rotation_angle_deg,
                actual_rotation_deg,
                rotation_error_deg,
                rotation_overshoot_deg,
                distance_error,
                "是" if overshoot_detected else "否",
            )
        else:
            self._log(
                "info",
                "路径方向误差：纵向=%.4fm，横向=%.4fm，最大过冲=%.4fm，"
                "是否超过阈值=%s",
                along_error,
                cross_error,
                overshoot,
                "是" if overshoot_detected else "否",
            )
        if imu_roll_deg and imu_pitch_deg and imu_tilt_deg:
            self._log(
                "info",
                "IMU 倾角：roll变化峰值=%.3fdeg，pitch变化峰值=%.3fdeg，"
                "最大总倾角=%.3fdeg",
                max(abs(value) for value in relative_roll_deg),
                max(abs(value) for value in relative_pitch_deg),
                max(imu_tilt_deg),
            )
        else:
            self._log(
                "warning",
                "本次没有有效 IMU 姿态数据，请检查话题 %s 和消息 orientation",
                self.imu_topic,
            )
        if chassis_status_rows:
            final_chassis_status = chassis_status_rows[-1]
            self._log(
                "info",
                "底盘状态：status=%d，error(L/R)=(%d/%d)，"
                "最大绝对电流(L/R)=(%s/%s)A",
                int(float(final_chassis_status["chassis_status_word"])),
                int(float(final_chassis_status["chassis_error_code_left"])),
                int(float(final_chassis_status["chassis_error_code_right"])),
                (
                    f"{max(abs(value) for value in chassis_current_left):.3f}"
                    if chassis_current_left
                    else "无数据"
                ),
                (
                    f"{max(abs(value) for value in chassis_current_right):.3f}"
                    if chassis_current_right
                    else "无数据"
                ),
            )
        else:
            self._log(
                "warning",
                "本次未收到有效底盘汇总状态，请检查话题 %s",
                self.robot_state_topic,
            )
        if valid_temperatures:
            self._log(
                "info",
                "底盘状态通道报告的最高温度=%.1fdegC",
                max(valid_temperatures),
            )
        else:
            self._log(
                "warning",
                "底盘温度字段全零或无数据，已标记为无效；"
                "当前状态通道也没有底盘力矩字段",
            )
        self._log("info", "结果文件已保存：%s", finished_run_dir)
        self._log("info", "=" * 88)

        self.state = "idle"
        self.config = None
        self.start_pose = None
        self.stop_pose = None
        self.target_pose = None
        self.run_dir = None
        if self.loop_active:
            # 循环中的每一步仍走完整的停止观察和结果保存流程。完成后先保持
            # 零速等待 loop_pause_sec，再由定时器自动取出下一步。
            self.loop_result_dirs.append(str(finished_run_dir))
            self.loop_completed_steps += 1
            self.loop_pause_until_sec = (
                self._now_sec()
                + float(self.get_parameter("loop_pause_sec").value)
            )
            remaining_steps = (
                len(self.loop_queue)
                + (1 if self.pending_config is not None else 0)
            )
            self._log(
                "info",
                "循环进度：已完成 %d/%d 步，剩余 %d 步；"
                "零速等待 %.3fs 后继续",
                self.loop_completed_steps,
                self.loop_total_steps,
                remaining_steps,
                float(self.get_parameter("loop_pause_sec").value),
            )

    def _abort_test(self, reason: str) -> None:
        self._publish_zero()
        self._log("error", "测试中止并已发布零速：%s", reason)
        aborted_run_dir = self.run_dir
        if self.run_dir is not None and self.samples:
            self._write_csv(self.run_dir / "samples_aborted.csv")
        self.state = "idle"
        self.start_requested = False
        self.config = None
        self.start_pose = None
        self.stop_pose = None
        self.target_pose = None
        self.run_dir = None
        if self.loop_active or self.loop_start_requested:
            if aborted_run_dir is not None:
                self.loop_result_dirs.append(
                    f"{aborted_run_dir} (aborted)")
            self._finish_loop_session(reason)

    def _write_csv(self, path: Path) -> None:
        if not self.samples:
            return
        with path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(
                csv_file, fieldnames=list(self.samples[0].keys()))
            writer.writeheader()
            writer.writerows(self.samples)

    def _write_plots(self) -> None:
        assert self.run_dir is not None
        assert self.config is not None
        assert self.start_pose is not None
        assert self.target_pose is not None
        if not self.samples:
            return

        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            self._log(
                "warning",
                "未安装 matplotlib，已保留 CSV，但跳过图片生成；"
                "可安装 python3-matplotlib 后重试",
            )
            return

        time_values = [float(row["time_sec"]) for row in self.samples]

        # 1) map 实际轨迹与理论直线。
        figure, axis = plt.subplots(figsize=(8, 7))
        axis.plot(
            [float(row["map_x"]) for row in self.samples],
            [float(row["map_y"]) for row in self.samples],
            label="actual map trajectory",
            linewidth=2.0,
        )
        axis.plot(
            [self.start_pose.x, self.target_pose.x],
            [self.start_pose.y, self.target_pose.y],
            "--",
            label="theoretical trajectory",
        )
        axis.scatter(
            [self.start_pose.x],
            [self.start_pose.y],
            marker="o",
            s=70,
            label="start",
        )
        axis.scatter(
            [self.target_pose.x],
            [self.target_pose.y],
            marker="x",
            s=90,
            label="target",
        )
        axis.scatter(
            [float(self.samples[-1]["map_x"])],
            [float(self.samples[-1]["map_y"])],
            marker="s",
            s=55,
            label="final",
        )
        axis.set_xlabel("map x [m]")
        axis.set_ylabel("map y [m]")
        axis.set_title("Omni chassis trajectory in map frame")
        axis.axis("equal")
        axis.grid(True)
        axis.legend()
        figure.tight_layout()
        figure.savefig(self.run_dir / "trajectory_map.png", dpi=160)
        plt.close(figure)

        # 2) 命令速度与 odom 速度，用于观察响应延迟、停止拖尾和过冲。
        # 原地旋转显示角速度及累计旋转角；平移保持原有的 XY 速度和路径进度。
        figure, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
        if self.config.is_rotation:
            axes[0].plot(
                time_values,
                [float(row["cmd_wz"]) for row in self.samples],
                label="cmd wz",
            )
            axes[0].plot(
                time_values,
                [float(row["odom_wz"]) for row in self.samples],
                label="odom wz",
            )
            axes[0].set_ylabel("angular velocity [rad/s]")
            axes[0].set_title(
                "Command and odometry angular velocity response")

            axes[1].plot(
                time_values,
                [
                    math.degrees(float(row["rotation_progress_rad"]))
                    for row in self.samples
                ],
                label="accumulated rotation",
            )
            axes[1].axhline(
                self.config.rotation_angle_deg,
                color="tab:red",
                linestyle="--",
                label="theoretical rotation",
            )
            axes[1].set_ylabel("rotation progress [deg]")
        else:
            axes[0].plot(
                time_values,
                [float(row["cmd_vx_map"]) for row in self.samples],
                label="cmd vx map",
            )
            axes[0].plot(
                time_values,
                [float(row["odom_vx_map"]) for row in self.samples],
                label="odom vx map",
            )
            axes[0].plot(
                time_values,
                [float(row["cmd_vy_map"]) for row in self.samples],
                label="cmd vy map",
            )
            axes[0].plot(
                time_values,
                [float(row["odom_vy_map"]) for row in self.samples],
                label="odom vy map",
            )
            axes[0].set_ylabel("velocity [m/s]")
            axes[0].set_title("Command and odometry velocity response")

            axes[1].plot(
                time_values,
                [
                    (float(row["map_x"]) - self.start_pose.x)
                    * self.config.direction_x
                    + (float(row["map_y"]) - self.start_pose.y)
                    * self.config.direction_y
                    for row in self.samples
                ],
                label="along-track progress",
            )
            axes[1].axhline(
                self.config.speed_mps * self.config.duration_sec,
                color="tab:red",
                linestyle="--",
                label="theoretical distance",
            )
            axes[1].set_ylabel("along-track distance [m]")
        axes[0].axvline(
            self.config.duration_sec,
            color="black",
            linestyle="--",
            label="command stop",
        )
        axes[0].grid(True)
        axes[0].legend(ncol=3)
        axes[1].axvline(
            self.config.duration_sec, color="black", linestyle="--")
        axes[1].set_xlabel("time [s]")
        axes[1].grid(True)
        axes[1].legend()
        figure.tight_layout()
        figure.savefig(
            self.run_dir / "odom_velocity_response.png", dpi=160)
        plt.close(figure)

        # 3) IMU 倾角及惯性数据。绝对 roll/pitch 受安装角影响，因此同时
        # 输出相对测试起点的变化量，更适合观察加减速时车体俯仰和侧倾。
        imu_roll_values = [
            float(row["imu_roll_deg"]) for row in self.samples]
        imu_pitch_values = [
            float(row["imu_pitch_deg"]) for row in self.samples]
        valid_imu_indices = [
            index
            for index, (roll, pitch) in enumerate(
                zip(imu_roll_values, imu_pitch_values))
            if math.isfinite(roll) and math.isfinite(pitch)
        ]
        if valid_imu_indices:
            reference_index = valid_imu_indices[0]
            reference_roll = imu_roll_values[reference_index]
            reference_pitch = imu_pitch_values[reference_index]
            relative_roll = [
                (
                    math.degrees(
                        normalize_angle(math.radians(value - reference_roll)))
                    if math.isfinite(value)
                    else math.nan
                )
                for value in imu_roll_values
            ]
            relative_pitch = [
                (
                    math.degrees(
                        normalize_angle(math.radians(value - reference_pitch)))
                    if math.isfinite(value)
                    else math.nan
                )
                for value in imu_pitch_values
            ]

            figure, axes = plt.subplots(
                4, 1, figsize=(12, 13), sharex=True)
            axes[0].plot(time_values, imu_roll_values, label="roll")
            axes[0].plot(time_values, imu_pitch_values, label="pitch")
            axes[0].plot(
                time_values,
                [float(row["imu_tilt_deg"]) for row in self.samples],
                label="total tilt",
            )
            axes[0].set_ylabel("angle [deg]")
            axes[0].set_title("IMU chassis tilt")
            axes[0].grid(True)
            axes[0].legend(ncol=3)

            axes[1].plot(
                time_values, relative_roll, label="roll - start")
            axes[1].plot(
                time_values, relative_pitch, label="pitch - start")
            axes[1].axhline(0.0, color="black", linewidth=0.8)
            axes[1].set_ylabel("relative angle [deg]")
            axes[1].grid(True)
            axes[1].legend(ncol=2)

            for axis_name in ("x", "y", "z"):
                axes[2].plot(
                    time_values,
                    [
                        float(row[f"imu_angular_velocity_{axis_name}"])
                        for row in self.samples
                    ],
                    label=f"gyro {axis_name}",
                )
                axes[3].plot(
                    time_values,
                    [
                        float(row[f"imu_linear_acceleration_{axis_name}"])
                        for row in self.samples
                    ],
                    label=f"acc {axis_name}",
                )
            axes[2].set_ylabel("angular velocity [rad/s]")
            axes[2].grid(True)
            axes[2].legend(ncol=3)
            axes[3].set_xlabel("time [s]")
            axes[3].set_ylabel("acceleration [m/s²]")
            axes[3].grid(True)
            axes[3].legend(ncol=3)
            for axis in axes:
                axis.axvline(
                    self.config.duration_sec,
                    color="black",
                    linestyle="--",
                    linewidth=0.9,
                )
            figure.tight_layout()
            figure.savefig(self.run_dir / "imu_tilt.png", dpi=160)
            plt.close(figure)
        else:
            self._log(
                "warning",
                "没有有效 IMU 姿态样本，跳过 imu_tilt.png",
            )

        # 4) 底盘汇总电流、温度与故障状态。这里的数据来自
        # /feedback/robot_server_state，是左右侧兼容汇总字段，不代表八个电机
        # 各自的数据；机器人当前也没有发布底盘力矩。
        chassis_current_left = [
            float(row["chassis_current_left"]) for row in self.samples]
        chassis_current_right = [
            float(row["chassis_current_right"]) for row in self.samples]
        chassis_status_word = [
            float(row["chassis_status_word"]) for row in self.samples]
        chassis_plot_available = any(
            math.isfinite(value)
            for value in chassis_current_left + chassis_current_right
        ) or any(value >= 0.0 for value in chassis_status_word)
        if chassis_plot_available:
            figure, axes = plt.subplots(
                3, 1, figsize=(12, 11), sharex=True)
            axes[0].plot(
                time_values, chassis_current_left, label="left current")
            axes[0].plot(
                time_values, chassis_current_right, label="right current")
            axes[0].set_ylabel("current [A]")
            axes[0].set_title(
                "Chassis aggregate electrical and diagnostic state")
            axes[0].grid(True)
            axes[0].legend(ncol=2)

            temperature_keys = (
                ("chassis_driver_temperature", "driver temperature"),
                ("chassis_temperature_left", "left temperature"),
                ("chassis_temperature_right", "right temperature"),
            )
            for key, label in temperature_keys:
                axes[1].plot(
                    time_values,
                    [float(row[key]) for row in self.samples],
                    label=label,
                )
            has_valid_temperature = any(
                float(row["chassis_temperature_valid"]) > 0.5
                for row in self.samples
            )
            if not has_valid_temperature:
                axes[1].text(
                    0.5,
                    0.5,
                    "All temperature fields are zero / unavailable",
                    horizontalalignment="center",
                    verticalalignment="center",
                    transform=axes[1].transAxes,
                    bbox={"facecolor": "white", "alpha": 0.8},
                )
            axes[1].set_ylabel("temperature [degC]")
            axes[1].grid(True)
            axes[1].legend(ncol=3)

            axes[2].step(
                time_values,
                chassis_status_word,
                where="post",
                label="status word",
            )
            axes[2].step(
                time_values,
                [
                    float(row["chassis_error_code_left"])
                    for row in self.samples
                ],
                where="post",
                label="left error code",
            )
            axes[2].step(
                time_values,
                [
                    float(row["chassis_error_code_right"])
                    for row in self.samples
                ],
                where="post",
                label="right error code",
            )
            axes[2].set_xlabel("time [s]")
            axes[2].set_ylabel("raw status/code")
            axes[2].grid(True)
            axes[2].legend(ncol=3)
            for axis in axes:
                axis.axvline(
                    self.config.duration_sec,
                    color="black",
                    linestyle="--",
                    linewidth=0.9,
                )
            figure.tight_layout()
            figure.savefig(
                self.run_dir / "chassis_electrical_status.png", dpi=160)
            plt.close(figure)
        else:
            self._log(
                "warning",
                "没有有效底盘汇总状态样本，跳过 chassis_electrical_status.png",
            )

        # 5) 四轮舵角和轮速反馈。position/speed 单位沿用底层消息定义。
        figure, axes = plt.subplots(2, 1, figsize=(11, 9), sharex=True)
        for wheel_name in WHEEL_NAMES:
            axes[0].plot(
                time_values,
                [float(row[f"{wheel_name}_position"])
                 for row in self.samples],
                label=wheel_name,
            )
            axes[1].plot(
                time_values,
                [float(row[f"{wheel_name}_speed"])
                 for row in self.samples],
                label=wheel_name,
            )
        axes[0].axvline(
            self.config.duration_sec, color="black", linestyle="--")
        axes[0].set_ylabel("steer position [rad]")
        axes[0].set_title("Wheel motor feedback")
        axes[0].grid(True)
        axes[0].legend(ncol=2)
        axes[1].axvline(
            self.config.duration_sec, color="black", linestyle="--")
        axes[1].set_xlabel("time [s]")
        axes[1].set_ylabel("wheel speed [driver unit]")
        axes[1].grid(True)
        axes[1].legend(ncol=2)
        figure.tight_layout()
        figure.savefig(self.run_dir / "motor_feedback.png", dpi=160)
        plt.close(figure)

    def emergency_stop(self) -> None:
        """退出前连续发布数次零速，提高停止命令被底层收到的概率。"""
        for _ in range(5):
            self._publish_zero()
        self._log("warning", "测试节点退出，已连续发布零速")


def main(args: Optional[List[str]] = None) -> None:
    # 禁用 rclpy 默认 SIGINT 处理：否则 Ctrl-C 会先关闭 context，
    # finally 中的安全零速便无法发布。这里先执行 emergency_stop，再主动 shutdown。
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = OmniChassisTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.emergency_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
