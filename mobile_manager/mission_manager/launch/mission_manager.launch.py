#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('mission_manager'), 'params',
            'mission_manager_params.yaml'
        ]),
        description='Path to the mission manager parameters file')

    # Mission manager node
    mission_manager_node = Node(
        package='mission_manager',
        executable='mission_manager_node',
        name='mission_manager',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        respawn=False,
    )

    return LaunchDescription([
        params_file_arg,
        mission_manager_node,
    ])
