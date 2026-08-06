import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()
    use_sim_time = LaunchConfiguration("use_sim_time")
    initial_x = LaunchConfiguration("x_pose")
    initial_y = LaunchConfiguration("y_pose")
    config = os.path.join(
        get_package_share_directory("swerve_controller"),
        "config",
        "robot_geometry.yaml",
    )

    node_swerve_commander = Node(
        package="swerve_controller",
        name="swerve_commander",
        executable="wheel_cmd_publisher",
        parameters=[config, {"use_sim_time": use_sim_time}],
    )

    node_swerve_odom = Node(
        package="swerve_controller",
        name="swerve_odometer",
        executable="wheel_odometry",
        parameters=[
            config,
            {
                "use_sim_time": use_sim_time,
                "initial_x": initial_x,
                "initial_y": initial_y,
            },
        ],
    )

    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="true"))
    ld.add_action(DeclareLaunchArgument("x_pose", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("y_pose", default_value="0.0"))
    ld.add_action(node_swerve_commander)
    ld.add_action(node_swerve_odom)
    return ld
