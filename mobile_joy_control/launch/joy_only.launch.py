#!/usr/bin/env python3

import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mobile_joy_control.controller_binding import (
    DEFAULT_CONTROLLER_SERIAL_CACHE_FILE,
    DEFAULT_HARDWARE_CONFIG_FILE,
    DEFAULT_MODE_SWITCH_TIMEOUT_S,
    DEFAULT_RECEIVER_ENUMERATION_TIMEOUT_S,
)


def launch_setup(context, *args, **kwargs):
    # 获取启动参数
    joy_dev = LaunchConfiguration('joy_dev').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)
    chassis_config_file = LaunchConfiguration('chassis_config_file').perform(
        context)
    hardware_config_file = LaunchConfiguration(
        'hardware_config_file').perform(context)
    controller_serial_cache_file = LaunchConfiguration(
        'controller_serial_cache_file').perform(context)
    receiver_enumeration_timeout_s = LaunchConfiguration(
        'receiver_enumeration_timeout_s').perform(context)
    mode_switch_timeout_s = LaunchConfiguration(
        'mode_switch_timeout_s').perform(context)
    try:
        receiver_enumeration_timeout_s = float(
            receiver_enumeration_timeout_s)
    except ValueError:
        receiver_enumeration_timeout_s = (
            DEFAULT_RECEIVER_ENUMERATION_TIMEOUT_S)
    if (not math.isfinite(receiver_enumeration_timeout_s) or
            receiver_enumeration_timeout_s < 0.0):
        receiver_enumeration_timeout_s = (
            DEFAULT_RECEIVER_ENUMERATION_TIMEOUT_S)
    try:
        mode_switch_timeout_s = float(mode_switch_timeout_s)
    except ValueError:
        mode_switch_timeout_s = DEFAULT_MODE_SWITCH_TIMEOUT_S
    if (not math.isfinite(mode_switch_timeout_s) or
            mode_switch_timeout_s <= 0.0):
        mode_switch_timeout_s = DEFAULT_MODE_SWITCH_TIMEOUT_S

    cmd_vel_mux_dir = get_package_share_directory('cmd_vel_mux')
    params_mux = os.path.join(cmd_vel_mux_dir, 'config',
                              'cmd_vel_mux_params.yaml')

    # 读取 chassis.cfg 文件（使用绝对路径）
    robot_type = "diff_drive"  # 默认值
    try:
        # chassis_config_file 是绝对路径，直接读取
        with open(chassis_config_file, 'r') as f:
            robot_config = json.load(f)  # .cfg文件内容是JSON格式
            robot_type = robot_config.get("robot_name", "diff_drive")
            print(f"[INFO] Detected robot type: {robot_type}")
    except Exception as e:
        print(
            '[WARN] Could not read chassis config file '
            f'{chassis_config_file}: {e}'
        )
        print(f"[WARN] Using default robot_type: {robot_type}")

    nodes = []

    # 每次启动或接收器拔插后，必须先在X模式读取真实ID；匹配后切到D模式
    # 且输入设备枚举完成，才发布控制许可。
    controller_binding_guard_node = Node(
        package='mobile_joy_control',
        executable='controller_binding_guard.py',
        name='controller_binding_guard',
        parameters=[{
            'hardware_config_file': hardware_config_file,
            'controller_serial_cache_file': controller_serial_cache_file,
            'joy_dev': joy_dev,
            'check_interval_s': 0.2,
            'receiver_enumeration_timeout_s':
                receiver_enumeration_timeout_s,
            # X→D正常重新枚举实测最大约0.782秒，2秒用于容纳USB负载波动。
            'mode_switch_timeout_s': mode_switch_timeout_s,
        }],
        output='screen')
    nodes.append(controller_binding_guard_node)

    # 创建joy节点
    joy_node = Node(package='joy',
                    executable='joy_node',
                    name='joy_node',
                    parameters=[{
                        # ROS 2 joy 使用 SDL 的设备名称选择手柄，不支持 dev 参数。
                        'device_name': 'Logitech Cordless RumblePad 2',
                        'deadzone': 0.3,
                        'autorepeat_rate': 20.0,
                    }],
                    output='screen')
    nodes.append(joy_node)

    # 创建mobile_joy_control节点
    mobile_joy_control_node = Node(package='mobile_joy_control',
                                   executable='mobile_joy_control',
                                   name='mobile_joy_control',
                                   parameters=[config_file],
                                   output='screen')
    nodes.append(mobile_joy_control_node)

    # 创建cmd_vel_mux节点，根据机器人类型设置不同的输出话题
    if robot_type == "omni":
        cmd_vel_remapping = [('cmd_vel', 'control/cmd_vel_omni')]
        print("[INFO] cmd_vel_mux output topic: control/cmd_vel_omni")
    else:
        cmd_vel_remapping = [('cmd_vel', 'control/cmd_vel')]
        print("[INFO] cmd_vel_mux output topic: control/cmd_vel")

    cmd_vel_mux_node = Node(package='cmd_vel_mux',
                            executable='cmd_vel_mux_node',
                            output='both',
                            parameters=[params_mux],
                            remappings=cmd_vel_remapping)
    nodes.append(cmd_vel_mux_node)

    # 根据机器人类型决定是否创建mobile_cmd_omni节点
    if robot_type == "omni":
        print("[INFO] Starting mobile_cmd_omni node...")
        cmd_omni_vel_node = Node(package='mobile_joy_control',
                                 executable='mobile_cmd_omni',
                                 name='mobile_cmd_omni',
                                 output='screen')
        nodes.append(cmd_omni_vel_node)

    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_joy_control')

    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description=(
            'Linux joydev path used to verify DInput receiver ownership'))

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'params.yaml'),
        description='Path to the joy control config file')

    chassis_config_file_arg = DeclareLaunchArgument(
        'chassis_config_file',
        default_value=(
            '/home/dexforce/w1/install/w1_server/share/w1_server/chassis.cfg'
        ),
        description=(
            'Absolute path to the chassis configuration file '
            '(.cfg with JSON content)'
        )
    )

    hardware_config_file_arg = DeclareLaunchArgument(
        'hardware_config_file',
        default_value=DEFAULT_HARDWARE_CONFIG_FILE,
        description=(
            'Hardware configuration containing the bound controller ID'))

    controller_serial_cache_file_arg = DeclareLaunchArgument(
        'controller_serial_cache_file',
        default_value=str(DEFAULT_CONTROLLER_SERIAL_CACHE_FILE),
        description=(
            'Deprecated compatibility argument; runtime authorization reads '
            'the real ID in XInput mode and does not trust the DInput cache'))

    receiver_enumeration_timeout_s_arg = DeclareLaunchArgument(
        'receiver_enumeration_timeout_s',
        default_value=str(DEFAULT_RECEIVER_ENUMERATION_TIMEOUT_S),
        description=(
            'Seconds before reporting that input enumeration is still '
            'pending; validation continues afterwards'))

    mode_switch_timeout_s_arg = DeclareLaunchArgument(
        'mode_switch_timeout_s',
        default_value=str(DEFAULT_MODE_SWITCH_TIMEOUT_S),
        description='同一USB端口从X模式重新枚举为D模式的最长等待时间，单位：秒')

    opaque_function = OpaqueFunction(function=launch_setup)

    return LaunchDescription([
        joy_dev_arg,
        config_file_arg,
        chassis_config_file_arg,
        hardware_config_file_arg,
        controller_serial_cache_file_arg,
        receiver_enumeration_timeout_s_arg,
        mode_switch_timeout_s_arg,
        opaque_function,
    ])
