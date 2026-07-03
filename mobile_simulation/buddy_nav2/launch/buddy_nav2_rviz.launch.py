#!/usr/bin/env python3

import os

from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml   # nav2 提供的工具
import json


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('buddi_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_aws_robomaker = get_package_share_directory('aws_robomaker_small_warehouse_world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(
        pkg_aws_robomaker,
        'worlds',
        'no_roof_small_warehouse',
        'no_roof_small_warehouse.world'
    )
    # world = os.path.join(
    #     pkg_aws_robomaker,
    #     'worlds',
    #     'workcell',
    #     'dynamic_warehouse.world'
    # )

    lattice_file_package = get_package_share_directory('nav2_smac_planner')
    lattice_file = os.path.join(
        lattice_file_package,
        'sample_primitives',
        '5cm_resolution',
        'output.json'
    )

    original_params_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'params',
        'w1_nav2_params.yaml'
    )

    configured_params = RewrittenYaml(
        source_file=original_params_file,
        root_key='',   # 根目录下修改
        param_rewrites={
            'planner_server.ros__parameters.GridBased.lattice_filepath': lattice_file
        },
        convert_types=True
    )

    map_file = os.path.join(
        pkg_aws_robomaker,
        'maps',
        '005',
        'simulation_mapping_v1_keepout.yaml'
    )
    nova_carter_dock_params_dir = os.path.join(
        get_package_share_directory('nova_carter_docking'), 'params')
    
    docking_params_file = os.path.join(
        nova_carter_dock_params_dir, 'nova_carter_docking.yaml')
    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[docking_params_file,
                    {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': ['docking_server']}],
    )

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': 'False',
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': 'True'
            }.items()
    )

    # 发布map->odom的静态tf
    # [-10.053669, -13.225729, 0.0]
    map_odom_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-1.875', '-1.825', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    ld = LaunchDescription()

    # Add the commands to the launch description
    # ld.add_action(gzserver_cmd)
    # ld.add_action(gzclient_cmd)
    # ld.add_action(robot_state_publisher_cmd)
    ld.add_action(map_odom_tf_pub)
    # ld.add_action(spawn_buddi_cmd)
    # ld.add_action(rviz_cmd)
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(docking_server)
    ld.add_action(lifecycle_manager)

    return ld