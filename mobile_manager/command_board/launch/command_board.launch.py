#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def command_board_parameters(chassis_backend, params_file,
                             cloudia_overrides_file):
    if chassis_backend == 'dexe':
        return [params_file, {'chassis_backend': 'dexe'}]
    if chassis_backend == 'cloudia':
        # 迦智负责导航，叠加覆盖参数以关闭本地传感器监测和导航自动启动。
        return [
            params_file, cloudia_overrides_file, {
                'chassis_backend': 'cloudia'
            }
        ]
    raise ValueError('chassis_backend must be dexe or cloudia')


def build_nodes(chassis_backend, params_file, map_asset_manager_params_file,
                cloudia_overrides_file):
    command_board_node = Node(
        package='command_board',
        executable='command_board_node',
        name='command_board',
        output='screen',
        parameters=command_board_parameters(chassis_backend, params_file,
                                            cloudia_overrides_file),
    )

    # 迦智负责地图管理，cloudia 后端只启动遥控接管与安全响应节点。
    if chassis_backend == 'cloudia':
        return [command_board_node]

    map_manager_node = Node(
        package='map_manager',
        executable='map_manager_node',
        name='map_manager',
        output='screen',
    )

    map_asset_manager_node = Node(
        package='map_asset_manager',
        executable='map_asset_manager_node',
        name='map_asset_manager',
        output='screen',
        parameters=[map_asset_manager_params_file],
    )

    return [command_board_node, map_manager_node, map_asset_manager_node]


def _build_nodes_from_context(context):
    chassis_backend = LaunchConfiguration('chassis_backend').perform(context)
    params_file = LaunchConfiguration('command_board_params_file').perform(
        context)
    map_asset_manager_params_file = LaunchConfiguration(
        'map_asset_manager_params_file').perform(context)
    cloudia_overrides_file = os.path.join(
        get_package_share_directory('command_board'), 'params',
        'command_board_cloudia_overrides.yaml')
    return build_nodes(chassis_backend, params_file,
                       map_asset_manager_params_file, cloudia_overrides_file)


def generate_launch_description():
    command_board_params_arg = DeclareLaunchArgument(
        'command_board_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('command_board'), 'params',
            'command_board_params.yaml'
        ]),
        description='Full path to the command_board parameters file',
    )

    map_asset_manager_params_arg = DeclareLaunchArgument(
        'map_asset_manager_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('map_asset_manager'), 'params',
            'map_asset_manager_params.yaml'
        ]),
        description='Full path to the map_asset_manager parameters file',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'chassis_backend',
            default_value=os.environ.get('DEXE_CHASSIS_BACKEND', 'dexe'),
            description='dexe or cloudia; must match joy and RobotServer'),
        command_board_params_arg,
        map_asset_manager_params_arg,
        OpaqueFunction(function=_build_nodes_from_context),
    ])
