"""Launch the path_follower node with its parameter file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("path_follower")
    params_file = os.path.join(pkg_share, "config", "path_follower.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time (/clock).",
    )

    follower_node = Node(
        package="path_follower",
        executable="path_follower_node",
        name="path_follower_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[
            ("~/path", "/path"),
            ("~/tracking_path", "/tracking_path"),
            # ("~/odom", "/odom"),
            ("~/odom", "/feedback/odom"),  # w1
            # ("~/cmd_vel", "/cmd_vel"),
            ("~/cmd_vel", "/control/cmd_vel_nav"),  # w1
            # ("~/cmd_vel", "/cmd_vel_nav"),  # w1 速度平滑器
            ("~/navigation_status", "/navigation_status"),
            ("~/stop", "/stop"),
        ],
    )

    return LaunchDescription([use_sim_time_arg, follower_node])
