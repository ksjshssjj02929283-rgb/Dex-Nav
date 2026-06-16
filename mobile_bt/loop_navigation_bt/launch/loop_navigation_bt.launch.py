from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='loop_navigation_bt',
            executable='loop_navigation_bt_node',
            name='loop_navigation_bt_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
