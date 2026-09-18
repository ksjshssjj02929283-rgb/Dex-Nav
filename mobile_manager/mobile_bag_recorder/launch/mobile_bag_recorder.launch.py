import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('mobile_bag_recorder')
    default_params = os.path.join(
        package_dir, 'params', 'mobile_bag_recorder_params.yaml')

    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='录包节点参数文件路径'),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='日志级别'),

        Node(
            package='mobile_bag_recorder',
            executable='mobile_bag_recorder_node',
            name='mobile_bag_recorder_node',
            output='screen',
            parameters=[params_file],
            arguments=['--ros-args', '--log-level', log_level],
            # 录包器是取证工具，挂了就没有证据。任何原因退出都拉起来，
            # 而不是让机器人在"以为在录"的状态下跑一整天
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
