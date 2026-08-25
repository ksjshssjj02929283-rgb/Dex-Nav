#!/usr/bin/env python3
"""W1 双轮差速仿真一键启动：Gazebo + W1 + 传感器 + Nav2 + RViz.

机器人模型来自 diff-sim-container/w1_diff_gazebo（自包含，内置 mesh）。
Nav2 参数：直接复用实机 w1_nav2_params_diff.yaml，保持差速仿真与生产配置一致。
仿真选项：统一读取 config/diff_sim.yaml，不再通过命令行分别覆盖。
"""

import glob
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (EmitEvent, IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


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


def _require_mapping(document, key):
    """读取必需的 YAML 字典，并在配置结构错误时给出明确提示。"""
    value = document.get(key)
    if not isinstance(value, dict):
        raise ValueError(f'diff_sim.yaml: {key} 必须是字典')
    return value


def _require_bool(section, key):
    """读取必需的布尔配置，禁止把字符串 true/false 当作布尔值。"""
    value = section.get(key)
    if not isinstance(value, bool):
        raise ValueError(f'diff_sim.yaml: {key} 必须是 true 或 false')
    return value


def _require_number(section, key):
    """读取必需的数值配置。"""
    value = section.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'diff_sim.yaml: {key} 必须是数值')
    return value


def _resolve_path(value, key):
    """解析 package:// 或绝对路径，并提前阻止空路径进入 launch 链路。"""
    if not isinstance(value, str) or not value:
        raise ValueError(f'diff_sim.yaml: {key} 必须是非空路径')
    if value.startswith('package://'):
        package_path = value[len('package://'):]
        package_name, separator, relative_path = package_path.partition('/')
        if not separator or not relative_path:
            raise ValueError(
                f'diff_sim.yaml: {key} 的 package:// 路径格式无效')
        value = os.path.join(
            get_package_share_directory(package_name), relative_path)
    elif not os.path.isabs(value):
        raise ValueError(
            f'diff_sim.yaml: {key} 只支持 package:// 路径或绝对路径')
    if not os.path.isfile(value):
        raise FileNotFoundError(f'diff_sim.yaml: {key} 文件不存在: {value}')
    return value


def _load_sim_config(config_path):
    """加载并校验差速仿真统一配置。"""
    with open(config_path, 'r', encoding='utf-8') as config_file:
        document = yaml.safe_load(config_file)
    if not isinstance(document, dict):
        raise ValueError('diff_sim.yaml: 根节点必须是字典')

    sensors = _require_mapping(document, 'sensors')
    gazebo = _require_mapping(document, 'gazebo')
    robot_spawn = _require_mapping(document, 'robot_spawn')
    navigation = _require_mapping(document, 'navigation')

    config = {
        'sensors': {
            key: _require_bool(sensors, key)
            for key in (
                'rgbd_front', 'rgbd_rear', 'lidar_front',
                'lidar_rear', 'imu')
        },
        'gui': _require_bool(gazebo, 'gui'),
        'render': gazebo.get('render'),
        'x_pose': _require_number(robot_spawn, 'x_pose'),
        'y_pose': _require_number(robot_spawn, 'y_pose'),
        'slam': _require_bool(navigation, 'slam'),
        'map': _resolve_path(navigation.get('map'), 'navigation.map'),
        'params_file': _resolve_path(
            navigation.get('params_file'), 'navigation.params_file'),
        'map_odom_x': _require_number(navigation, 'map_odom_x'),
        'map_odom_y': _require_number(navigation, 'map_odom_y'),
        'map_odom_yaw': _require_number(navigation, 'map_odom_yaw'),
    }
    if config['render'] not in ('auto', 'nvidia', 'software'):
        raise ValueError(
            'diff_sim.yaml: gazebo.render 必须是 auto、nvidia 或 software')
    return config


def _bool_text(value):
    """把 YAML 布尔值转换为 ROS launch 使用的小写文本。"""
    return 'true' if value else 'false'


def _python_bool_text(value):
    """生成可直接用于下游 PythonExpression 的布尔字面量。"""
    return 'True' if value else 'False'


def _readiness_exit_handler(success_actions, failure_reason):
    """就绪门成功后继续启动；失败则终止，避免运行半套系统。"""
    def _on_exit(event, context):
        if context.is_shutdown:
            return None
        if event.returncode != 0:
            return [
                LogInfo(msg=(
                    f'错误：仿真就绪检测失败，退出码: {event.returncode}')),
                EmitEvent(event=Shutdown(reason=failure_reason)),
            ]
        return success_actions

    return _on_exit


def generate_launch_description():
    """Build the differential-drive simulation launch description."""
    w1_nav2_share = get_package_share_directory('w1_nav2')
    config_path = os.path.join(w1_nav2_share, 'config', 'diff_sim.yaml')
    config = _load_sim_config(config_path)

    launch_file_dir = os.path.join(
        get_package_share_directory('w1_diff_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_aws_robomaker = get_package_share_directory(
        'aws_robomaker_small_warehouse_world')
    rgbd_launch_dir = os.path.join(
        get_package_share_directory('dexe_sensor_process'), 'launch')

    use_sim_time = 'true'
    use_composition = 'False'
    gui = _bool_text(config['gui'])
    render = config['render']
    sensors = config['sensors']

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

    # W1 差速仿真机器人：rsp + spawn（diff-sim-container/w1_diff_gazebo）
    spawn_w1_diff_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_w1_diff.launch.py')),
        launch_arguments={
            'x_pose': str(config['x_pose']),
            'y_pose': str(config['y_pose']),
            'use_sim_time': use_sim_time,
            **{
                f'use_{sensor_name}': _bool_text(enabled)
                for sensor_name, enabled in sensors.items()
            },
        }.items())

    front_rgbd_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rgbd_launch_dir, 'front_p2c.launch.py')),
        condition=IfCondition(_bool_text(sensors['rgbd_front'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'depth_max_range': '4.9',
            'ground_filter_min_height': '0.05',
        }.items())

    rear_rgbd_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rgbd_launch_dir, 'rear_p2c.launch.py')),
        condition=IfCondition(_bool_text(sensors['rgbd_rear'])),
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
            'slam': _python_bool_text(config['slam']),
            'map': config['map'],
            'use_sim_time': use_sim_time,
            'params_file': config['params_file'],
            'autostart': 'True',
            'use_composition': use_composition,
            'map_odom_x': str(config['map_odom_x']),
            'map_odom_y': str(config['map_odom_y']),
            'map_odom_yaw': str(config['map_odom_yaw']),
        }.items())

    # 蓝色规划路径与红色实际 Odom 轨迹的对照。节点从 /plan 收到新规划时
    # 清空上一段轨迹，因此红线始终只表示当前规划后的实际行驶过程。
    path_trace_cmd = Node(
        package='w1_nav2',
        executable='path_trace_visualizer.py',
        name='path_trace_visualizer',
        output='screen',
        parameters=[{'use_sim_time': True}])

    # 第一层门控：Factory 服务可用且 /clock 连续推进后，才生成机器人。
    gazebo_ready_cmd = Node(
        package='w1_nav2',
        executable='wait_for_gazebo_ready.py',
        name='gazebo_ready_waiter',
        output='screen')

    # 第二层门控：有效 Odom 和 YAML 中启用的传感器话题全部到达后，才启动导航。
    sim_ready_cmd = Node(
        package='w1_nav2',
        executable='wait_for_sim_ready.py',
        name='sim_ready_waiter',
        output='screen',
        parameters=[{
            f'require_{sensor_name}': enabled
            for sensor_name, enabled in sensors.items()
        }])
    spawn_robot_when_gazebo_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo_ready_cmd,
            on_exit=_readiness_exit_handler(
                [
                    spawn_w1_diff_cmd,
                    front_rgbd_cmd,
                    rear_rgbd_cmd,
                    sim_ready_cmd,
                ],
                'Gazebo 未就绪')))
    start_navigation_when_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=sim_ready_cmd,
            on_exit=_readiness_exit_handler(
                [path_trace_cmd, rviz_cmd, nav2_bringup_cmd],
                '仿真接口未就绪')))

    # 仿真环境变量（对齐 swerve sim.launch.py 的做法）：
    # 1) 仅 loopback 通信，避免 DDS 多播发现风暴（节点多时控制器/服务会拖垮）
    localhost_only = SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1')
    # 2) 禁用 gazebo 在线模型库查询，避免阻塞在已失效的 models.gazebosim.org
    disable_model_db = SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', '')
    use_nvidia = IfCondition(_bool_text(render == 'nvidia'))
    use_software = IfCondition(_bool_text(render == 'software'))
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
    ld.add_action(LogInfo(msg=(
        f'差速仿真配置: RGBD前/后='
        f'{sensors["rgbd_front"]}/{sensors["rgbd_rear"]}, '
        f'雷达前/后={sensors["lidar_front"]}/{sensors["lidar_rear"]}, '
        f'IMU={sensors["imu"]}, GUI={config["gui"]}, '
        f'render={render}')))
    ld.add_action(localhost_only)
    ld.add_action(disable_model_db)
    ld.add_action(nvidia_offload)
    ld.add_action(nvidia_glx)
    ld.add_action(software_gl)
    ld.add_action(resource_path)
    ld.add_action(model_path)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    # 两个事件处理器必须先注册，再启动第一层就绪检测，避免快速事件丢失。
    ld.add_action(spawn_robot_when_gazebo_ready)
    ld.add_action(start_navigation_when_ready)
    ld.add_action(gazebo_ready_cmd)

    return ld
