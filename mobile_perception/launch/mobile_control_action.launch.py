#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_perception')

    default_params = os.path.join(pkg_share, 'config', 'mobile_control_action_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the YAML file with parameters for mobile_control_action_node'
    )

    params_file = LaunchConfiguration('params_file')

    mobile_control_action = Node(
        package='mobile_perception',
        executable='mobile_control_action_node',
        name='mobile_control_action_node',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        params_file_arg,
        mobile_control_action,
    ])
