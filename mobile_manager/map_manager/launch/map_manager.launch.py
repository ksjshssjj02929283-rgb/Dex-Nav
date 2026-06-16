#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    node = Node(package='map_manager',
                executable='map_manager_node',
                name='map_manager',
                output='screen')

    return LaunchDescription([node])
