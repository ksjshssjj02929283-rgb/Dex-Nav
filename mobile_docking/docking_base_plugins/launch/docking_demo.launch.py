# W1 docking demo：docking_server(四 map:planner/controller/goal_checker+task) + lifecycle_manager + 静态 map->odom
# odom->base_link 由仿真/真机提供（demo 用 fake_robot 积分 cmd_vel）
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('docking_base_plugins'), 'config', 'w1_docking.yaml')

    return LaunchDescription([
        Node(
            package='docking_base', executable='docking_base', name='docking_server',
            output='screen', parameters=[params],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_docking', output='screen',
            parameters=[{'autostart': True}, {'node_names': ['docking_server']}],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),
    ])
