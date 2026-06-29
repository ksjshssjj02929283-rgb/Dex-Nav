#!/usr/bin/env python3
"""One-shot bringup: swerve Gazebo sim + Nav2 (omni params).

Ensures map->odom TF and /clock exist before Nav2 costmaps start.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    swerve_gazebo_dir = get_package_share_directory('swerve_robot_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    slam = LaunchConfiguration('slam')

    default_map = os.path.join(
        nav2_bringup_dir, 'maps', 'my_map.yaml')
    default_params = os.path.join(
        nav2_bringup_dir, 'params', 'w1_nav2_params_omni.yaml')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock')
    declare_slam = DeclareLaunchArgument(
        'slam', default_value='False',
        description='Run SLAM instead of map_server')
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to map yaml')
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Nav2 params yaml')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(swerve_gazebo_dir, 'launch', 'sim.launch.py')),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': slam,
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'True',
        }.items(),
    )

    # Wait for Gazebo, controllers, map->odom and /clock before Nav2.
    delayed_nav2 = TimerAction(period=15.0, actions=[nav2_launch])

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam,
        declare_map,
        declare_params,
        sim_launch,
        delayed_nav2,
    ])
