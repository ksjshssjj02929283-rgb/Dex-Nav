# Copyright (c) DexForce
#
# Licensed under the Apache License, Version 2.0 (the "License).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml_arg = DeclareLaunchArgument(
        "map_yaml",
        description="Path to map.yaml (preprocessed/ must exist alongside)",
    )
    frame_id_arg = DeclareLaunchArgument("frame_id", default_value="map")
    show_global_arg = DeclareLaunchArgument("show_global", default_value="true")
    show_local_arg = DeclareLaunchArgument("show_local", default_value="true")
    show_raw_arg = DeclareLaunchArgument("show_raw", default_value="false")

    visualizer = Node(
        package="nav2_map_server",
        executable="nodemap_visualizer",
        name="nodemap_visualizer",
        output="screen",
        parameters=[{
            "map_yaml": LaunchConfiguration("map_yaml"),
            "frame_id": LaunchConfiguration("frame_id"),
            "show_global": LaunchConfiguration("show_global"),
            "show_local": LaunchConfiguration("show_local"),
            "show_raw": LaunchConfiguration("show_raw"),
            "publish_rate_hz": 1.0,
            "global_node_scale": 0.12,
            "local_node_scale": 0.06,
            "raw_node_scale": 0.02,
        }],
    )

    return LaunchDescription([
        map_yaml_arg,
        frame_id_arg,
        show_global_arg,
        show_local_arg,
        show_raw_arg,
        visualizer,
    ])
