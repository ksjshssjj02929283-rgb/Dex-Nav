#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def launch_setup(context, *args, **kwargs):
    # 获取启动参数
    joy_dev = LaunchConfiguration('joy_dev').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)
    chassis_config_file = LaunchConfiguration('chassis_config_file').perform(
        context)

    cmd_vel_mux_dir = get_package_share_directory('cmd_vel_mux')
    params_mux = os.path.join(cmd_vel_mux_dir, 'config',
                              'cmd_vel_mux_params.yaml')

    # 读取 RobotServer 与 ChassisNode 共用的运行配置。
    robot_type = "diff_drive"  # 默认值
    try:
        with open(chassis_config_file, 'r') as f:
            robot_config = yaml.safe_load(f) or {}
            chassis_config = robot_config.get("chassis_config", {}) or {}
            robot_type = chassis_config.get("chassis_type", "diff_drive")
            print(f"[INFO] Detected robot type: {robot_type}")
    except Exception as e:
        print(
            f"[WARN] Could not read robot config file {chassis_config_file}: {e}"
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

    # 所有底盘类型统一输出；RobotServer 根据 chassis_type 选择逆解。
    cmd_vel_remapping = [('cmd_vel', 'control/cmd_vel')]
    print(f"[INFO] cmd_vel_mux output topic: control/cmd_vel")

    cmd_vel_mux_node = Node(package='cmd_vel_mux',
                            executable='cmd_vel_mux_node',
                            output='both',
                            parameters=[params_mux],
                            remappings=cmd_vel_remapping)
    nodes.append(cmd_vel_mux_node)

    # 驻车 Action 节点可用于全向底盘；它不订阅 Twist，不参与普通导航解算。
    if robot_type == "omni":
        print(f"[INFO] Starting mobile_cmd_omni Action node...")
        cmd_omni_vel_node = Node(package='mobile_joy_control',
                                 executable='mobile_cmd_omni',
                                 name='mobile_cmd_omni',
                                 output='screen')
        nodes.append(cmd_omni_vel_node)
        start_coordinator_node = Node(
            package='mobile_joy_control',
            executable='omni_start_coordinator',
            name='omni_start_coordinator',
            parameters=[config_file],
            output='screen')
        nodes.append(start_coordinator_node)

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

    # 与 w1_server/src/main.cpp 使用同一份运行配置。
    chassis_config_file_arg = DeclareLaunchArgument(
        'chassis_config_file',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'w1',
            'dexe_robot_server_data',
            'robot_config.yaml'),
        description=
        'Path to the shared w1_server runtime robot_config.yaml'
    )

    # 使用OpaqueFunction来在启动时读取配置文件并动态决定启动哪些节点
    opaque_function = OpaqueFunction(function=launch_setup)

    return LaunchDescription([
        joy_dev_arg,
        config_file_arg,
        chassis_config_file_arg,
        opaque_function,
    ])
