#!/usr/bin/env python3
"""执行 F710 的 X 模式验 ID、D 模式控制授权和拔插复位。"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from mobile_joy_control.controller_binding import (
    ControllerInsertionValidator,
    DEFAULT_CONTROLLER_SERIAL_CACHE_FILE,
    DEFAULT_HARDWARE_CONFIG_FILE,
    DEFAULT_MODE_SWITCH_TIMEOUT_S,
    DEFAULT_RECEIVER_ENUMERATION_TIMEOUT_S,
    validation_stage_to_string,
)

# 关键配置参数
DEFAULT_CHECK_INTERVAL_S = 0.2  # 接收器拔插检测周期，单位：秒。
MIN_CHECK_INTERVAL_S = 0.05  # 最小检测周期，单位：秒，防止配置过小占用 CPU。
BINDING_STATE_TOPIC = 'control/controller_binding_ok'  # 遥操配对许可 Topic。
DEFAULT_JOY_DEV = '/dev/input/js0'  # joy_node 使用的输入设备路径。


class ControllerBindingGuard(Node):
    """周期推进手柄授权状态机，并持续发布安全许可。"""

    def __init__(self):
        super().__init__('controller_binding_guard')
        self.declare_parameter('hardware_config_file',
                               DEFAULT_HARDWARE_CONFIG_FILE)
        self.declare_parameter(
            'controller_serial_cache_file',
            str(DEFAULT_CONTROLLER_SERIAL_CACHE_FILE))
        self.declare_parameter('joy_dev', DEFAULT_JOY_DEV)
        self.declare_parameter('check_interval_s', DEFAULT_CHECK_INTERVAL_S)
        self.declare_parameter(
            'receiver_enumeration_timeout_s',
            DEFAULT_RECEIVER_ENUMERATION_TIMEOUT_S)
        self.declare_parameter('mode_switch_timeout_s',
                               DEFAULT_MODE_SWITCH_TIMEOUT_S)

        self.hardware_config_file = self.get_parameter(
            'hardware_config_file').value
        self.controller_serial_cache_file = self.get_parameter(
            'controller_serial_cache_file').value
        self.joy_dev = self.get_parameter('joy_dev').value
        try:
            check_interval_s = float(
                self.get_parameter('check_interval_s').value)
        except (TypeError, ValueError):
            check_interval_s = DEFAULT_CHECK_INTERVAL_S
            self.get_logger().warning(
                'check_interval_s 不是数值，使用默认值 '
                f'{DEFAULT_CHECK_INTERVAL_S}s')
        if not math.isfinite(check_interval_s):
            check_interval_s = DEFAULT_CHECK_INTERVAL_S
            self.get_logger().warning(
                'check_interval_s 不是有限数值，使用默认值 '
                f'{DEFAULT_CHECK_INTERVAL_S}s')
        if check_interval_s < MIN_CHECK_INTERVAL_S:
            self.get_logger().warning(
                f'check_interval_s 小于 {MIN_CHECK_INTERVAL_S}s，'
                f'按 {MIN_CHECK_INTERVAL_S}s 使用')
            check_interval_s = MIN_CHECK_INTERVAL_S
        self.check_interval_s = check_interval_s

        try:
            enumeration_timeout_s = float(
                self.get_parameter(
                    'receiver_enumeration_timeout_s').value)
        except (TypeError, ValueError):
            enumeration_timeout_s = DEFAULT_RECEIVER_ENUMERATION_TIMEOUT_S
        if (not math.isfinite(enumeration_timeout_s) or
                enumeration_timeout_s < 0):
            enumeration_timeout_s = DEFAULT_RECEIVER_ENUMERATION_TIMEOUT_S
            self.get_logger().warning(
                'receiver_enumeration_timeout_s 非法，使用默认值 '
                f'{DEFAULT_RECEIVER_ENUMERATION_TIMEOUT_S}s')

        try:
            mode_switch_timeout_s = float(
                self.get_parameter('mode_switch_timeout_s').value)
        except (TypeError, ValueError):
            mode_switch_timeout_s = DEFAULT_MODE_SWITCH_TIMEOUT_S
        if (not math.isfinite(mode_switch_timeout_s) or
                mode_switch_timeout_s <= 0.0):
            mode_switch_timeout_s = DEFAULT_MODE_SWITCH_TIMEOUT_S
            self.get_logger().warning(
                'mode_switch_timeout_s 非法，使用默认值 '
                f'{DEFAULT_MODE_SWITCH_TIMEOUT_S}s')

        self.validator = ControllerInsertionValidator(
            self.hardware_config_file,
            joy_dev=self.joy_dev,
            enumeration_timeout_s=enumeration_timeout_s,
            serial_cache_file=self.controller_serial_cache_file,
            mode_switch_timeout_s=mode_switch_timeout_s,
        )
        self.last_detection_error = ''
        self.binding_pub = self.create_publisher(
            Bool, BINDING_STATE_TOPIC, 1)
        self.check_timer = self.create_timer(
            self.check_interval_s, self.check_receiver)
        self.check_receiver()

    def check_receiver(self):
        """检测模式和拔插变化，并持续发布当前控制许可。"""
        try:
            state_reported = self.validator.poll()
        except Exception as exc:  # 检测异常时失败关闭，禁止监视器意外退出
            # sysfs 暂时不可读时安全关闭；恢复后继续沿用最近一次插入状态。
            binding_ok = False
            detection_error = str(exc)
            if detection_error != self.last_detection_error:
                self.get_logger().error(
                    f'手柄接收器检测失败，禁止手柄控制：{detection_error}')
                self.last_detection_error = detection_error
        else:
            self.last_detection_error = ''
            binding_ok = self.validator.binding_ok
            stage_name = validation_stage_to_string(self.validator.stage)
            if state_reported and binding_ok:
                current = self.validator.current
                self.get_logger().info(
                    '手柄授权成功：'
                    f'状态={stage_name}，'
                    f'X模式校验 ID={self.validator.bound_id}，'
                    f'当前模式={current.input_mode}，'
                    f'USB路径={current.usb_path}，允许手柄控制')
            elif state_reported and self.validator.pairing_required:
                self.get_logger().warning(
                    '手柄未完成首次绑定，当前禁止手柄控制：'
                    f'状态={stage_name}，'
                    f'{self.validator.reason}')
            elif state_reported and self.validator.validation_pending:
                self.get_logger().warning(
                    '手柄尚未获得控制许可：'
                    f'状态={stage_name}，'
                    f'{self.validator.reason}')
            elif state_reported:
                self.get_logger().error(
                    '手柄接收器状态变化，禁止手柄控制：'
                    f'状态={stage_name}，'
                    f'{self.validator.reason}')

        message = Bool()
        message.data = binding_ok
        self.binding_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerBindingGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
