#!/usr/bin/env python3
"""W1 差速仿真建图模式：Gazebo + W1 + Rviz，无 Nav2 或 map->odom.

与 diff_sim.launch.py 的区别：
- 不发静态 map->odom（建图时该 TF 由 dexe cartographer 的 mapping_node 发布，两者会冲突）
- 不起 Nav2（建图靠 teleop 开车）

配套流程见《全向与差速仿真建图操作手册》（另开终端并 source SLAM 工作区）：
  ros2 launch cartographer_ros dexe_slam_2d.launch.py use_sim_time:=true
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/control/cmd_vel
"""

import glob
import os

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _gazebo_resource_path(world_share):
    """构造 Gazebo 资源路径，确保 OGRE 能加载系统材质并创建渲染 Scene。"""
    resource_parts = [world_share]
    for candidate in sorted(glob.glob('/usr/share/gazebo-*'), reverse=True):
        if os.path.isdir(os.path.join(candidate, 'media', 'materials')):
            resource_parts.append(candidate)
            break
    existing_resource_path = os.environ.get('GAZEBO_RESOURCE_PATH', '')
    if existing_resource_path:
        resource_parts.append(existing_resource_path)
    return os.pathsep.join(resource_parts)


def generate_launch_description():
    """Build the differential-drive mapping simulation launch description."""
    launch_file_dir = os.path.join(
        get_package_share_directory('w1_diff_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_aws_robomaker = get_package_share_directory(
        'aws_robomaker_small_warehouse_world')

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    gui = LaunchConfiguration('gui')
    render = LaunchConfiguration('render')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    declare_x_pose = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y_pose = DeclareLaunchArgument('y_pose', default_value='0.0')
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true', choices=['true', 'false'])
    declare_render = DeclareLaunchArgument(
        'render', default_value='auto',
        choices=['auto', 'nvidia', 'software'])

    world = os.path.join(
        pkg_aws_robomaker, 'worlds', 'no_roof_small_warehouse',
        'no_roof_small_warehouse.world')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items())

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui))

    spawn_w1_diff_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_w1_diff.launch.py')),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'use_sim_time': use_sim_time,
        }.items())

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch',
                         'rviz_launch.py')),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'False',
            'use_sim_time': use_sim_time,
            'rviz_config':
                os.path.join(get_package_share_directory('w1_nav2'), 'rviz',
                             'diff_sim.rviz')
        }.items())

    # Gazebo 渲染 Scene 和 spawn 服务并非同时就绪，延迟插入带深度相机的机器人。
    delayed_gzclient_cmd = TimerAction(period=8.0, actions=[gzclient_cmd])
    delayed_spawn_w1_diff_cmd = TimerAction(
        period=8.0, actions=[spawn_w1_diff_cmd])
    delayed_rviz_cmd = TimerAction(period=15.0, actions=[rviz_cmd])

    # 环境变量同 diff_sim.launch.py
    localhost_only = SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1')
    disable_model_db = SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', '')
    use_nvidia = IfCondition(PythonExpression(["'", render, "' == 'nvidia'"]))
    use_software = IfCondition(
        PythonExpression(["'", render, "' == 'software'"]))
    nvidia_offload = SetEnvironmentVariable(
        '__NV_PRIME_RENDER_OFFLOAD', '1', condition=use_nvidia)
    nvidia_glx = SetEnvironmentVariable(
        '__GLX_VENDOR_LIBRARY_NAME', 'nvidia', condition=use_nvidia)
    software_gl = SetEnvironmentVariable(
        'LIBGL_ALWAYS_SOFTWARE', '1', condition=use_software)
    resource_path = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH', _gazebo_resource_path(pkg_aws_robomaker))
    w1_diff_share_parent = os.path.dirname(
        get_package_share_directory('w1_diff_gazebo'))
    model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.pathsep.join(filter(None, [
            w1_diff_share_parent,
            os.environ.get('GAZEBO_MODEL_PATH', ''),
        ])))

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_x_pose)
    ld.add_action(declare_y_pose)
    ld.add_action(declare_gui)
    ld.add_action(declare_render)
    ld.add_action(localhost_only)
    ld.add_action(disable_model_db)
    ld.add_action(nvidia_offload)
    ld.add_action(nvidia_glx)
    ld.add_action(software_gl)
    ld.add_action(resource_path)
    ld.add_action(model_path)
    ld.add_action(gzserver_cmd)
    ld.add_action(delayed_gzclient_cmd)
    ld.add_action(delayed_spawn_w1_diff_cmd)
    ld.add_action(delayed_rviz_cmd)

    return ld
