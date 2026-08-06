#!/usr/bin/env python3
"""启动四轮全向 Gazebo 仿真、Nav2 与 Rviz.

等价于手册的三终端流程合一：
- swerve_robot_gazebo/sim.launch.py（gzserver/gzclient + 机器人 + 控制器）
- nav2_bringup/bringup_launch.py（w1_nav2_params_omni.yaml）
- nav2_bringup/rviz_launch.py（w1_nav2/diff_sim.rviz，共享配置）

延迟启动 Nav2/Rviz，等 Gazebo、控制器与 /clock 就绪。导航模式下的
map->odom 由 nav2_bringup 发布；SLAM 模式下由 SLAM 发布。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Build the omnidirectional navigation simulation launch description."""
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    warehouse_world_dir = get_package_share_directory(
        'aws_robomaker_small_warehouse_world')
    swerve_gazebo_dir = get_package_share_directory('swerve_robot_gazebo')
    w1_nav2_dir = get_package_share_directory('w1_nav2')
    rgbd_launch_dir = os.path.join(
        get_package_share_directory('dexe_sensor_process'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    slam = LaunchConfiguration('slam')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    gui = LaunchConfiguration('gui')
    render = LaunchConfiguration('render')
    map_odom_x = LaunchConfiguration('map_odom_x')
    map_odom_y = LaunchConfiguration('map_odom_y')
    map_odom_yaw = LaunchConfiguration('map_odom_yaw')

    default_map = os.path.join(warehouse_world_dir, 'maps',
                               'sim_warehouse_v1', 'slam', 'map.yaml')
    default_params = os.path.join(
        nav2_bringup_dir, 'params', 'w1_nav2_params_omni.yaml')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock')
    declare_slam = DeclareLaunchArgument(
        'slam', default_value='False',
        description='Run SLAM instead of map_server')
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to map yaml')
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Nav2 params yaml')
    declare_x_pose = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y_pose = DeclareLaunchArgument('y_pose', default_value='0.0')
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true', choices=['true', 'false'],
        description='Start the Gazebo client GUI')
    declare_render = DeclareLaunchArgument(
        'render', default_value='auto',
        choices=['auto', 'nvidia', 'software'],
        description='Gazebo client rendering backend')
    declare_map_odom_x = DeclareLaunchArgument(
        'map_odom_x', default_value='0.03143',
        description='Default map x alignment relative to simulation odom')
    declare_map_odom_y = DeclareLaunchArgument(
        'map_odom_y', default_value='-0.01856',
        description='Default map y alignment relative to simulation odom')
    declare_map_odom_yaw = DeclareLaunchArgument(
        'map_odom_yaw', default_value='0.02040',
        description='Default map yaw alignment relative to simulation odom')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(swerve_gazebo_dir, 'launch', 'sim.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'gui': gui,
            'render': render,
        }.items(),
    )

    front_rgbd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rgbd_launch_dir, 'front_p2c.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'depth_max_range': '5.0',
            'ground_filter_min_height': '0.05',
        }.items())

    rear_rgbd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rgbd_launch_dir, 'rear_p2c.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'depth_max_range': '5.0',
            'ground_filter_min_height': '0.05',
        }.items())

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': slam,
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'True',
            'map_odom_x': map_odom_x,
            'map_odom_y': map_odom_y,
            'map_odom_yaw': map_odom_yaw,
        }.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'False',
            'use_sim_time': use_sim_time,
            'rviz_config':
                os.path.join(w1_nav2_dir, 'rviz', 'diff_sim.rviz'),
        }.items(),
    )

    # Give Gazebo, controllers and /clock time to settle before Nav2/Rviz.
    # nav2_bringup creates map->odom after this delayed include starts.
    delayed_nav2 = TimerAction(period=15.0, actions=[nav2_launch])
    delayed_rviz = TimerAction(period=15.0, actions=[rviz_launch])

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam,
        declare_map,
        declare_params,
        declare_x_pose,
        declare_y_pose,
        declare_gui,
        declare_render,
        declare_map_odom_x,
        declare_map_odom_y,
        declare_map_odom_yaw,
        sim_launch,
        front_rgbd,
        rear_rgbd,
        delayed_nav2,
        delayed_rviz,
    ])
