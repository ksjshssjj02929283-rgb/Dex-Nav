#!/usr/bin/env python3

import os

from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml  # nav2 提供的工具
import json


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    # autostart 做成参数：两套导航后端（本栈 与 path_navigation）要能同时常驻、
    # 靠 lifecycle 状态互斥选用哪套时，本栈必须能起来但停在 unconfigured，
    # 等 command_board 按 navigation_backend 调 /lifecycle_manager_navigation 的 STARTUP。
    # 默认保持 True：单独起本 launch（仿真、调试）时行为与以前完全一致。
    autostart = LaunchConfiguration('autostart')

    lattice_file_package = get_package_share_directory('nav2_smac_planner')
    lattice_file = os.path.join(lattice_file_package, 'sample_primitives',
                                '5cm_resolution', 'output.json')

    original_params_file = os.path.join(
        get_package_share_directory('nav2_bringup'), 'params',
        'w1_nav2_params_diff.yaml')

    # 用 RewrittenYaml 覆盖其中的参数
    configured_params = RewrittenYaml(
        source_file=original_params_file,
        root_key='',  # 根目录下修改
        param_rewrites={
            'planner_server.ros__parameters.GridBased.lattice_filepath':
                lattice_file
        },
        convert_types=True)

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch',
                         'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': autostart,
        }.items())

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock; keep false on the physical robot'))
    ld.add_action(DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description=(
            'True = nav2 自带的 lifecycle_manager_navigation 启动即把整套 nav2 拉到 active；'
            'False = 全部停在 unconfigured，等 command_board 按 navigation_backend 选中本后端后'
            '调 manage_nodes 的 STARTUP。两套后端同时常驻时，本栈必须用 False，'
            '否则它会自激活，两套都出速度、都落到 cmd_vel_mux 的 control/cmd_vel_nav。')))
    ld.add_action(nav2_bringup_cmd)
    # 与 Nav2 一同启动底盘位姿发布节点，复用本次启动的时钟配置。
    ld.add_action(Node(
        package='mobile_perception',
        executable='pub_current_position',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'))

    return ld
