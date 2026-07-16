#!/usr/bin/env python3
# Offline map planning demo: map_server + planner_server + nodemap + RViz (no Gazebo).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory('nav2_theta_star_planner')
    bringup_dir = get_package_share_directory('nav2_bringup')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    default_params = os.path.join(bringup_dir, 'params', 'w1_nav2_params_omni.yaml')
    demo_params = os.path.join(pkg_dir, 'test', 'plan_demo_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'test', 'plan_demo.rviz')

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml,
        'node_map_source_yaml': map_yaml,
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'map',
            description='Full path to map.yaml (preprocessed/ nodemap optional but recommended)'),
        DeclareLaunchArgument(
            'params_file',
            default_value=demo_params,
            description='Nav2 params (default: plan_demo_params.yaml). '
                        f'Full stack fallback: {default_params}'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planner',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['planner_server'],
            }],
        ),

        Node(
            package='nav2_map_server',
            executable='nodemap_visualizer',
            name='nodemap_visualizer',
            output='screen',
            parameters=[{
                'map_yaml': map_yaml,
                'frame_id': 'map',
                'show_global': True,
                'show_local': True,
                'show_raw': False,
                'publish_rate_hz': 1.0,
                'global_node_scale': 0.25,
                'local_node_scale': 0.12,
            }],
            remappings=[('nodemap_markers', '/nodemap_markers')],
        ),

        Node(
            package='nav2_theta_star_planner',
            executable='plan_from_rviz.py',
            name='plan_from_rviz',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
