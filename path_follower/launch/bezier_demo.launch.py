"""联调 launch：贝塞尔轨迹生成器 + path_follower 跟踪节点 + nav2_collision_monitor。

串联拓扑：generator ──/path──► path_follower ──/cmd_vel_raw──►
collision_monitor ──/cmd_vel_nav──► 底盘

用法：
  ros2 launch path_follower bezier_demo.launch.py
可选参数（覆盖默认贝塞尔形状）：
  ros2 launch path_follower bezier_demo.launch.py \
      control_points:="[0.0,0.0,2.0,0.0,2.0,-2.0,4.0,-2.0,0.0,3.14]"
  ros2 launch path_follower bezier_demo.launch.py use_sim_time:=false
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 贝塞尔控制点默认值：前 8 个为控制点坐标，最后 2 个为起始/目标朝向角(rad)
DEFAULT_CONTROL_POINTS = [0.0, 0.0, 2.0, 0.0, 2.0, -2.0, 4.0, -2.0, 1.57, -1.57]
# DEFAULT_CONTROL_POINTS = [3.0, -2.0, 3.5, -2.0, 3.5, -2.0, 4.0, -2.0, 0.8, 0.8]


def _parse_control_points(raw: str):
    """把命令行传入的 YAML 列表字符串解析为 float 列表。

    ROS2 参数 control_points 必须是 double 数组，不能直接传字符串，
    否则 rclpy 会抛 'Expected a non-empty sequence, with items of uniform type'。
    """
    try:
        parsed = yaml.safe_load(raw)
    except yaml.YAMLError as e:
        raise ValueError(f"control_points 不是合法的 YAML 列表: {raw!r} ({e})")
    if not isinstance(parsed, list) or len(parsed) == 0:
        raise ValueError(f"control_points 必须是非空列表，当前: {raw!r}")
    try:
        return [float(x) for x in parsed]
    except (TypeError, ValueError) as e:
        raise ValueError(f"control_points 各项必须为数值: {raw!r} ({e})")


def generate_launch_description():
    pkg_share = get_package_share_directory('path_follower')
    params_file = os.path.join(pkg_share, 'config', 'path_follower.yaml')
    collision_launch = os.path.join(pkg_share, 'launch', 'collision_monitor.launch.py')
    velocity_smoother_launch = os.path.join(pkg_share, 'launch', 'velocity_smoother.launch.py')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 贝塞尔控制点参数（可命令行覆盖，需为合法 YAML 数组字符串）
    cp_arg = DeclareLaunchArgument(
        'control_points',
        default_value=yaml.dump(DEFAULT_CONTROL_POINTS, default_flow_style=True).strip(),
        description='三次贝塞尔控制点 [x0,y0,x1,y1,x2,y2,x3,y3, start_yaw, goal_yaw]',
    )
    # 仿真时钟：与 collision_monitor.launch.py 默认一致取 true。
    sim_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时钟 /clock',
    )
    # 基准位姿：控制点将变换到该位姿坐标系
    bp_arg = DeclareLaunchArgument(
        'base_pose',
        default_value='[0.0, 0.0, 0.0]',
        # default_value='[4.0, -2.0, 0.0]',
        # default_value='[0.0, 0.0, -1.57]',
        description='基准位姿 [x, y, yaw]，控制点变换到该坐标系',
    )
    # 生成器启动延迟（用 launch.TimerAction 替代脚本内 start_delay 计时器，等 path_follower 就绪）
    delay_arg = DeclareLaunchArgument(
        'start_delay',
        default_value='5.0',
        description='生成器节点延迟启动秒数',
    )

    # 碰撞监控（含 lifecycle_manager）：/cmd_vel_raw -> /cmd_vel_nav。
    # use_sim_time 透传，保持与 generator/follower 一致。
    collision_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(collision_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 速度平滑（含 lifecycle_manager）：/cmd_vel_nav -> /control/cmd_vel_nav（yaml 配置）。
    velocity_smoother = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(velocity_smoother_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    def _make_nodes(context):
        cp_raw = LaunchConfiguration('control_points').perform(context)
        control_points = _parse_control_points(cp_raw)
        bp_raw = LaunchConfiguration('base_pose').perform(context)
        base_pose = _parse_control_points(bp_raw)  # 复用解析逻辑
        sim = LaunchConfiguration('use_sim_time').perform(context) == 'true'

        generator_node = Node(
            package='path_follower',
            executable='bezier_path_generator.py',
            name='bezier_path_generator',
            output='screen',
            parameters=[{
                'control_points': control_points,
                'base_pose': base_pose,
                'resolution': 0.005,
                'frame_id': 'map',  # odom
                'republish': True,
                'period': 40.0,
                'use_sim_time': sim,
            }],
            remappings=[('~/path', '/path')],
        )
        # 节点晚 start_delay 秒启动（launch.TimerAction 替代脚本内 start_delay 计时器）。
        start_delay = float(LaunchConfiguration('start_delay').perform(context))
        generator = TimerAction(period=start_delay, actions=[generator_node])

        follower = Node(
            package='path_follower',
            executable='path_follower_node',
            name='path_follower_node',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': sim,
                'track_deviation_limit': 0.5,
                'enable_steer_range_analysis': False,
                # 仿真里关掉末端定位切换：该特性是为了规避真机上 map/TF 在到点阶段的
                # 定位抖动，仿真的定位是干净的，切到 odom 递推反而引入一层与真值无关的
                # 估计误差，干扰对跟踪精度本身的观察。两个阈值都置 0 即关闭。
                'end_localization_switch_distance': 0.0,
                'end_localization_switch_angle': 0.0,
            }],
            # 输出重映射到 /cmd_vel_raw，交给 collision_monitor 处理后再发 /cmd_vel。
            remappings=[
                ('~/path', '/path'),
                ('~/odom', '/odom'),  # linorobot2
                # ('~/odom', '/odom/wheel'),  # w1-sim
                # ('~/odom', '/feedback/odom'),  # w1-omni
                # ('~/cmd_vel', '/cmd_vel'),  # linorobot2
                # 与 collision_monitor / velocity_smoother launch 的新默认输入话题一致
                ('~/cmd_vel', '/cmd_vel_nav_path'),  # to velocity_smoother (旁路 collision_monitor)
                # ('~/cmd_vel', '/cmd_vel_raw'),  # to collision_monitor 输入 -> /cmd_vel_nav 输出
            ],
        )
        return [generator, follower]


    # path_follower_node 现在是 LifecycleNode，不拉到 active 就只是个空转的执行器
    # （不收路线、不出速度）。demo 要能"一条命令跑起来"，所以自带一个只管它一个的管理器。
    # bond_timeout 给到 10s：装载路线时预摆舵角有 3 秒同步等待且独占单线程执行器，
    # 留 nav2 默认的 4s 会被误判成节点卡死、把它 deactivate 掉。
    path_follower_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_path_follower",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["path_follower_node"]},
            {"bond_timeout": 10.0},
        ],
    )

    return LaunchDescription([
        cp_arg,
        sim_arg,
        bp_arg,
        delay_arg,
        collision_monitor,
        # velocity_smoother,
        OpaqueFunction(function=_make_nodes),
        path_follower_lifecycle_manager,
    ])
