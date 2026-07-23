#!/usr/bin/env python3
"""启动 W1 全向 Gazebo 建图环境与 Rviz，不启动 Nav2 或 map->odom.

启动后由 dexe Cartographer 订阅统一的双雷达、里程计与 IMU 接口，并负责
发布 map->odom。这样建图与真机使用同一个 SLAM 启动文件，无需话题重映射。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Build the omnidirectional mapping simulation launch description."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    gui = LaunchConfiguration('gui')
    render = LaunchConfiguration('render')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('swerve_robot_gazebo'), 'launch',
            'sim.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'gui': gui,
            'render': render,
        }.items())

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch',
            'rviz_launch.py')),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'False',
            'use_sim_time': use_sim_time,
            'rviz_config': os.path.join(
                get_package_share_directory('w1_nav2'), 'rviz',
                'diff_sim.rviz'),
        }.items())

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument(
            'gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'render', default_value='auto',
            choices=['auto', 'nvidia', 'software']),
        sim_launch,
        TimerAction(period=15.0, actions=[rviz_launch]),
    ])
