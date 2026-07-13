from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_goal_checker_error = LaunchConfiguration('use_goal_checker_error')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_goal_checker_error',
            default_value='true',
            description='Whether to prefer /navigation_goal_error from goal_checker for final error stats.'),
        Node(
            package='loop_navigation_bt',
            executable='loop_navigation_bt_node',
            name='loop_navigation_bt_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'goal_error_stats.use_goal_checker_error': use_goal_checker_error,
            }],
        ),
    ])
