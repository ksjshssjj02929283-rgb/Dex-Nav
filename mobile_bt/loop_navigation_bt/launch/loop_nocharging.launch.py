import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('loop_navigation_bt')

    # 两种循环导航入口共用同一份参数文件。
    params_file = os.path.join(pkg_dir, 'config', 'loop_navigation_bt_params.yaml')
    with open(params_file, 'r', encoding='utf-8') as stream:
        yaml_parameters = yaml.safe_load(stream)['/**']['ros__parameters']
    default_check_battery = str(
        yaml_parameters['battery.check_enabled_in_nocharging_mode']).lower()
    default_use_goal_checker_error = str(
        yaml_parameters['goal_error_stats.use_goal_checker_error']).lower()
    check_battery = LaunchConfiguration('check_battery')
    use_goal_checker_error = LaunchConfiguration('use_goal_checker_error')

    return LaunchDescription([
        DeclareLaunchArgument(
            'check_battery',
            default_value=default_check_battery,
            description='Whether to check battery level and stop on low battery.'),
        DeclareLaunchArgument(
            'use_goal_checker_error',
            default_value=default_use_goal_checker_error,
            description='Whether to prefer /navigation_goal_error from goal_checker for final error stats.'),
        Node(package='loop_navigation_bt',
             executable='loop_nocharging_bt_node',
             name='loop_nocharging_bt_node',
             output='screen',
             # 省略 ROS Logger 名称和 epoch 时间戳；需要时间的业务日志在正文中使用 UTC+8。
             additional_env={
                 'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] {message}',
             },
             parameters=[
                 params_file,
                 {
                     'battery.check_enabled_in_nocharging_mode':
                         ParameterValue(check_battery, value_type=bool),
                     'goal_error_stats.use_goal_checker_error':
                         ParameterValue(use_goal_checker_error, value_type=bool),
                 },
             ])
    ])
