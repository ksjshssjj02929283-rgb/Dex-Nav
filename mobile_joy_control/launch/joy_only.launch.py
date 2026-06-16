#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import json


def launch_setup(context, *args, **kwargs):
    # 获取启动参数
    joy_dev = LaunchConfiguration('joy_dev').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)
    chassis_config_file = LaunchConfiguration('chassis_config_file').perform(
        context)

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
            f"[WARN] Could not read chassis config file {chassis_config_file}: {e}"
        )
        print(f"[WARN] Using default robot_type: {robot_type}")

    nodes = []

    # 创建joy节点
    joy_node = Node(package='joy',
                    executable='joy_node',
                    name='joy_node',
                    parameters=[{
                        'dev': joy_dev,
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
        print(f"[INFO] cmd_vel_mux output topic: control/cmd_vel_omni")
    else:
        cmd_vel_remapping = [('cmd_vel', 'control/cmd_vel')]
        print(f"[INFO] cmd_vel_mux output topic: control/cmd_vel")

    cmd_vel_mux_node = Node(package='cmd_vel_mux',
                            executable='cmd_vel_mux_node',
                            output='both',
                            parameters=[params_mux],
                            remappings=cmd_vel_remapping)
    nodes.append(cmd_vel_mux_node)

    # 根据机器人类型决定是否创建mobile_cmd_omni节点
    if robot_type == "omni":
        print(f"[INFO] Starting mobile_cmd_omni node...")
        cmd_omni_vel_node = Node(package='mobile_joy_control',
                                 executable='mobile_cmd_omni',
                                 name='mobile_cmd_omni',
                                 output='screen')
        nodes.append(cmd_omni_vel_node)

    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_joy_control')

    # 声明启动参数
    joy_dev_arg = DeclareLaunchArgument('joy_dev',
                                        default_value='/dev/input/js0',
                                        description='Joystick device path')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'params.yaml'),
        description='Path to the joy control config file')

    # 声明 chassis.cfg 的绝对路径参数（使用您提供的具体路径）
    chassis_config_file_arg = DeclareLaunchArgument(
        'chassis_config_file',
        default_value=
        '/home/dexforce/w1/install/w1_server/share/w1_server/chassis.cfg',
        description=
        'Absolute path to the chassis configuration file (.cfg with JSON content)'
    )

    # 使用OpaqueFunction来在启动时读取配置文件并动态决定启动哪些节点
    opaque_function = OpaqueFunction(function=launch_setup)

    return LaunchDescription([
        joy_dev_arg,
        config_file_arg,
        chassis_config_file_arg,
        opaque_function,
    ])
