#!/usr/bin/env python3

import os

from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml  # nav2 提供的工具
import json


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    json_file = os.path.join(
        '/home/dexforce/w1/dexe_mobile_application/mobile_simulation/w1_nav2/',
        'config', 'config.json')
    with open(json_file, 'r') as f:
        config_data = json.load(f)

    # 从 json 读取 Time
    map_folder = config_data.get("map_filename", "default")
    map_file = os.path.join(
        '/home/dexforce/w1/dexe_mobile_application/mobile_simulation/w1_nav2/'
        'maps', map_folder, 'map.yaml')

    lattice_file_package = get_package_share_directory('nav2_smac_planner')
    lattice_file = os.path.join(lattice_file_package, 'sample_primitives',
                                '5cm_resolution', 'output.json')

    original_params_file = os.path.join(
        get_package_share_directory('nav2_bringup'), 'params',
        'w1_nav2_params.yaml')

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
            'slam': 'False',
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': 'True',
        }.items())

    ld = LaunchDescription()

    ld.add_action(nav2_bringup_cmd)

    return ld
