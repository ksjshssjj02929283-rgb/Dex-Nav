"""Launch the schedule_planner node with its parameter file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("nav2_schedule_planner")
    params_file = os.path.join(pkg_share, "config", "schedule_planner.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time (/clock).",
    )

    schedule_planner_node = Node(
        package="nav2_schedule_planner",
        executable="schedule_planner_node",
        name="schedule_planner_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    map_asset_manager_params_arg = DeclareLaunchArgument(
        "map_asset_manager_params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("map_asset_manager"), "params",
            "map_asset_manager_params.yaml"
        ]),
        description="Full path to the map_asset_manager parameters file",
    )

    map_asset_manager_node = Node(
        package="map_asset_manager",
        executable="map_asset_manager_node",
        name="map_asset_manager",
        output="screen",
        parameters=[LaunchConfiguration("map_asset_manager_params_file")],
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_asset_manager_params_arg,
        map_asset_manager_node,
        schedule_planner_node,
    ])
