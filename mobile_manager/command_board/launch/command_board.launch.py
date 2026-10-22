#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    command_board_params_arg = DeclareLaunchArgument(
        'command_board_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('command_board'), 'params',
            'command_board_params.yaml'
        ]),
        description='Full path to the command_board parameters file',
    )

    command_board_node = Node(
        package='command_board',
        executable='command_board_node',
        name='command_board',
        output='screen',
        parameters=[LaunchConfiguration('command_board_params_file')],
    )

    map_manager_node = Node(
        package='map_manager',
        executable='map_manager_node',
        name='map_manager',
        output='screen',
    )

    map_asset_manager_params_arg = DeclareLaunchArgument(
        'map_asset_manager_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('map_asset_manager'), 'params',
            'map_asset_manager_params.yaml'
        ]),
        description='Full path to the map_asset_manager parameters file',
    )

    map_asset_manager_node = Node(
        package='map_asset_manager',
        executable='map_asset_manager_node',
        name='map_asset_manager',
        output='screen',
        parameters=[LaunchConfiguration('map_asset_manager_params_file')],
    )

    return LaunchDescription([
        command_board_params_arg,
        map_asset_manager_params_arg,
        command_board_node,
        map_manager_node,
        map_asset_manager_node,
    ])
