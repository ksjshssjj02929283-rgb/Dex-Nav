from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("navigation_adapter"),
            "params",
            "navigation_adapter_params.yaml",
        ]),
        description="Full path to the ROS2 parameters file",
    )

    node = Node(
        package="navigation_adapter",
        executable="navigation_adapter_node",
        name="navigation_adapter_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_file_arg, node])
