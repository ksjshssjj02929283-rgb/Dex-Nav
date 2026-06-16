#!/usr/bin/env python3

import os

from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('buddi_gazebo'),
                                   'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_aws_robomaker = get_package_share_directory(
        'aws_robomaker_small_warehouse_world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='fa;se')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(pkg_aws_robomaker, 'worlds', 'no_roof_small_warehouse',
                         'no_roof_small_warehouse.world')
    # world = os.path.join(
    #     pkg_aws_robomaker,
    #     'worlds',
    #     'workcell',
    #     'dynamic_warehouse.world'
    # )

    map_file = os.path.join(pkg_aws_robomaker, 'maps', '005',
                            'b1_showroom.yaml')

    gzserver_cmd = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
                                            launch_arguments={'world': world
                                                             }.items())

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')))
    # 发布tf
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_buddi_urdf.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())
    # 发布sdf
    spawn_buddi_cmd = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(launch_file_dir, 'spawn_buddi_sdf.launch.py')),
                                               launch_arguments={
                                                   'x_pose': x_pose,
                                                   'y_pose': y_pose
                                               }.items())

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch',
                         'rviz_launch.py')),
        launch_arguments={
            'namespace':
                '',
            'use_namespace':
                'False',
            'use_sim_time':
                use_sim_time,
            'rviz_config':
                os.path.join(get_package_share_directory('buddy_nav2'), 'rviz',
                             'b1_simulation_v2.rviz')
        }.items())

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch',
                         'bringup_launch.py')),
        launch_arguments={
            'slam':
                'False',
            'map':
                map_file,
            'use_sim_time':
                use_sim_time,
            'params_file':
                os.path.join(get_package_share_directory('nav2_bringup'),
                             'params', 'nav2_params_sim_0410.yaml'),
            'autostart':
                'True'
        }.items())

    # 发布map->odom的静态tf
    # [-10.053669, -13.225729, 0.0]
    map_odom_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.125', '-0.175', '0', '0', '0', '0', 'map', 'odom'],
        output='screen')

    ld = LaunchDescription()

    # Add the commands to the launch description
    # ld.add_action(gzserver_cmd)
    # ld.add_action(gzclient_cmd)
    # ld.add_action(robot_state_publisher_cmd)
    # ld.add_action(map_odom_tf_pub)
    # ld.add_action(spawn_buddi_cmd)
    # ld.add_action(rviz_cmd)
    ld.add_action(nav2_bringup_cmd)

    return ld
