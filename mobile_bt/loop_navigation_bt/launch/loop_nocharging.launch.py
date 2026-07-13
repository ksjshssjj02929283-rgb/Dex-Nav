import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('loop_navigation_bt')

    # 行为树配置文件路径
    bt_xml = os.path.join(pkg_dir, 'config', 'loop_nocharging_bt.xml')
    check_battery = LaunchConfiguration('check_battery')
    use_goal_checker_error = LaunchConfiguration('use_goal_checker_error')

    return LaunchDescription([
        DeclareLaunchArgument(
            'check_battery',
            default_value='false',
            description='Whether to check battery level and stop on low battery.'),
        DeclareLaunchArgument(
            'use_goal_checker_error',
            default_value='true',
            description='Whether to prefer /navigation_goal_error from goal_checker for final error stats.'),
        Node(package='loop_navigation_bt',
             executable='loop_nocharging_bt_node',
             name='loop_nocharging_bt_node',
             output='screen',
             parameters=[{
                 'bt_xml': bt_xml,
                 'check_battery': ParameterValue(check_battery, value_type=bool),
                 'goal_error_stats.use_goal_checker_error': ParameterValue(use_goal_checker_error, value_type=bool),
             }])
    ])
