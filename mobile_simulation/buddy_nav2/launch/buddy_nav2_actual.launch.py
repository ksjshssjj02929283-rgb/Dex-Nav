#!/usr/bin/env python3

import os

from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_aws_robomaker = get_package_share_directory('buddy_nav2')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')


    map_file = os.path.join(
        pkg_aws_robomaker,
        'maps',
        '005',
        'b1_office.yaml'
    )

   

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': 'False',
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params_0418_laser.yaml'),
            'autostart': 'True'
            }.items()
    )


    ld = LaunchDescription()

    ld.add_action(nav2_bringup_cmd)

    return ld