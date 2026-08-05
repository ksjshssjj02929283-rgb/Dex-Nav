#!/usr/bin/env python3
"""W1 双轮差速仿真一键启动：Gazebo + W1 + RGBD + Nav2 + Rviz.

机器人模型来自 diff-sim-container/w1_diff_gazebo（自包含，内置 mesh）。
Nav2 参数：直接复用实机 w1_nav2_params_diff.yaml，保持差速仿真与生产配置一致。
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
    """Build the differential-drive simulation launch description."""
    launch_file_dir = os.path.join(
        get_package_share_directory('w1_diff_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_aws_robomaker = get_package_share_directory(
        'aws_robomaker_small_warehouse_world')
    rgbd_launch_dir = os.path.join(
        get_package_share_directory('dexe_sensor_process'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rgbd = LaunchConfiguration('use_rgbd')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    slam = LaunchConfiguration('slam')
    params_file = LaunchConfiguration('params_file')
    gui = LaunchConfiguration('gui')
    render = LaunchConfiguration('render')
    map_odom_x = LaunchConfiguration('map_odom_x')
    map_odom_y = LaunchConfiguration('map_odom_y')
    map_odom_yaw = LaunchConfiguration('map_odom_yaw')

    world = os.path.join(
        pkg_aws_robomaker, 'worlds', 'no_roof_small_warehouse',
        'no_roof_small_warehouse.world')

    # 地图可换（自建图用 map:=<map.yaml 绝对路径>）；默认图与仓库世界同包维护。
    # 注意：换图必须仍与世界原点对齐（从默认出生点开始建的图天然对齐）。
    default_map = os.path.join(pkg_aws_robomaker, 'maps',
                               'sim_warehouse_v1', 'slam', 'map.yaml')
    default_params = os.path.join(
        get_package_share_directory('nav2_bringup'), 'params',
        'w1_nav2_params_diff.yaml')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock')
    declare_use_rgbd = DeclareLaunchArgument(
        'use_rgbd', default_value='true', choices=['true', 'false'],
        description='Start Gazebo RGBD sensors and point-cloud converters')
    declare_slam = DeclareLaunchArgument(
        'slam', default_value='False',
        description='Run SLAM instead of map_server')
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description=(
            'Full path to map yaml (must be aligned with world origin)'))
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
    map_file = LaunchConfiguration('map')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items())

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui))

    # W1 差速仿真机器人：rsp + spawn（diff-sim-container/w1_diff_gazebo）
    spawn_w1_diff_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_w1_diff.launch.py')),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'use_sim_time': use_sim_time,
            'use_rgbd': use_rgbd,
        }.items())

    front_rgbd_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rgbd_launch_dir, 'front_p2c.launch.py')),
        condition=IfCondition(use_rgbd),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'depth_max_range': '4.9',
            'ground_filter_min_height': '0.05',
        }.items())

    rear_rgbd_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rgbd_launch_dir, 'rear_p2c.launch.py')),
        condition=IfCondition(use_rgbd),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'depth_max_range': '4.9',
            'ground_filter_min_height': '0.05',
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

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch',
                         'bringup_launch.py')),
        launch_arguments={
            'slam': slam,
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'True',
            'map_odom_x': map_odom_x,
            'map_odom_y': map_odom_y,
            'map_odom_yaw': map_odom_yaw,
        }.items())

    # Gazebo 的 spawn 服务早于渲染 Scene 完全就绪；带深度相机的模型必须延迟插入。
    delayed_gzclient_cmd = TimerAction(period=8.0, actions=[gzclient_cmd])
    delayed_spawn_w1_diff_cmd = TimerAction(
        period=8.0, actions=[spawn_w1_diff_cmd])
    # 等机器人、里程计和动态 TF 就绪后再启动 Nav2/RViz，避免启动期反复等待 TF。
    delayed_rviz_cmd = TimerAction(period=15.0, actions=[rviz_cmd])
    delayed_nav2_bringup_cmd = TimerAction(
        period=15.0, actions=[nav2_bringup_cmd])

    # 仿真环境变量（对齐 swerve sim.launch.py 的做法）：
    # 1) 仅 loopback 通信，避免 DDS 多播发现风暴（节点多时控制器/服务会拖垮）
    localhost_only = SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1')
    # 2) 禁用 gazebo 在线模型库查询，避免阻塞在已失效的 models.gazebosim.org
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
    # 3) 加入世界资源和 Gazebo 系统材质，避免渲染 Scene 初始化为空。
    resource_path = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH', _gazebo_resource_path(pkg_aws_robomaker))
    # 4) 让 gazebo 能解析 W1 mesh 的 package://w1_diff_gazebo/... URI：
    #    gazebo classic 对 package:// 按 GAZEBO_MODEL_PATH 逐项搜索 <包名>/<相对路径>
    #    （Rviz 走 ament 索引不受影响，漏设则 gzclient 里机器人不可见）
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
    ld.add_action(declare_use_rgbd)
    ld.add_action(declare_slam)
    ld.add_action(declare_map)
    ld.add_action(declare_params)
    ld.add_action(declare_x_pose)
    ld.add_action(declare_y_pose)
    ld.add_action(declare_gui)
    ld.add_action(declare_render)
    ld.add_action(declare_map_odom_x)
    ld.add_action(declare_map_odom_y)
    ld.add_action(declare_map_odom_yaw)
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
    ld.add_action(front_rgbd_cmd)
    ld.add_action(rear_rgbd_cmd)
    ld.add_action(delayed_rviz_cmd)
    ld.add_action(delayed_nav2_bringup_cmd)

    return ld
