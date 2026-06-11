#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    command_board_node = Node(
        package='command_board',
        executable='command_board_node',
        name='command_board',
        output='screen',
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

    navigation_adapter_params_arg = DeclareLaunchArgument(
        'navigation_adapter_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('navigation_adapter'), 'params',
            'navigation_adapter_params.yaml'
        ]),
        description='Full path to the navigation_adapter parameters file',
    )

    navigation_adapter_node = Node(
        package='navigation_adapter',
        executable='navigation_adapter_node',
        name='navigation_adapter_node',
        output='screen',
        parameters=[LaunchConfiguration('navigation_adapter_params_file')],
    )

    return LaunchDescription([
        map_asset_manager_params_arg,
        navigation_adapter_params_arg,
        command_board_node,
        map_manager_node,
        map_asset_manager_node,
        navigation_adapter_node,
    ])
