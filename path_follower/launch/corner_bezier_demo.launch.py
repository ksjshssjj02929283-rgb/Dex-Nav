"""联调 launch：折线拐角四次/五次贝塞尔平滑轨迹生成器 + path_follower 跟踪节点 + collision_monitor。

默认折线：[0,0] -> [2,0] -> [2,2]（直角拐弯），
在拐角处用四次/五次贝塞尔平滑（退让距离 = (bezier_degree - 2) * anchor_spacing），
输出平滑后的整段轨迹（直线 + 贝塞尔 + 直线，按弧长等距重采样）。

串联拓扑：generator ──/path──► path_follower ──/cmd_vel_nav──► velocity_smoother/底盘

用法：
  ros2 launch path_follower corner_bezier_demo.launch.py
可选参数：
  # 换一条折线（可多拐角，每个内部拐角都会被平滑）
  ros2 launch path_follower corner_bezier_demo.launch.py \
      waypoints:="[0.0,0.0, 2.0,0.0, 2.0,2.0, 4.0,2.0]"
  # 改锚点间距（退让距离 = (bezier_degree - 2) * anchor_spacing，越大拐角越缓、曲率越小）
  ros2 launch path_follower corner_bezier_demo.launch.py anchor_spacing:=0.3
  # 用四次贝塞尔（锚点 = 两边中点 + 拐角点）
  ros2 launch path_follower corner_bezier_demo.launch.py bezier_degree:=4
  # 全向底盘：朝向在起止角间线性均布，而不是跟随切线
  ros2 launch path_follower corner_bezier_demo.launch.py yaw_mode:=linear
  ros2 launch path_follower corner_bezier_demo.launch.py use_sim_time:=false
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


# 原始折线顶点 [x0,y0, x1,y1, x2,y2]：两段直线 [0,0] -> [2,0] -> [2,2]
# DEFAULT_WAYPOINTS = [0.0, 0.0, 2.0, 0.0, 2.0, 2.0]
DEFAULT_WAYPOINTS = [4.0, -2.0, 6.0, -2.0, 6.0, -4.0]
# 拐角贝塞尔阶数：5 = 两边三等分点共 6 个锚点；4 = 两边中点 + 拐角点共 5 个锚点
DEFAULT_BEZIER_DEGREE = 4
# 贝塞尔锚点沿边间距 (m)：拐角前后各退让 (DEFAULT_BEZIER_DEGREE - 2) * 该值
DEFAULT_ANCHOR_SPACING = 0.15


def _parse_float_list(raw: str, name: str):
    """把命令行传入的 YAML 列表字符串解析为 float 列表。

    ROS2 参数必须是 double 数组，不能直接传字符串，
    否则 rclpy 会抛 'Expected a non-empty sequence, with items of uniform type'。
    """
    try:
        parsed = yaml.safe_load(raw)
    except yaml.YAMLError as e:
        raise ValueError(f"{name} 不是合法的 YAML 列表: {raw!r} ({e})")
    if not isinstance(parsed, list) or len(parsed) == 0:
        raise ValueError(f"{name} 必须是非空列表，当前: {raw!r}")
    try:
        return [float(x) for x in parsed]
    except (TypeError, ValueError) as e:
        raise ValueError(f"{name} 各项必须为数值: {raw!r} ({e})")


def generate_launch_description():
    pkg_share = get_package_share_directory('path_follower')
    params_file = os.path.join(pkg_share, 'config', 'path_follower.yaml')
    collision_launch = os.path.join(pkg_share, 'launch', 'collision_monitor.launch.py')
    velocity_smoother_launch = os.path.join(pkg_share, 'launch', 'velocity_smoother.launch.py')
    use_sim_time = LaunchConfiguration('use_sim_time')

    wp_arg = DeclareLaunchArgument(
        'waypoints',
        default_value=yaml.dump(DEFAULT_WAYPOINTS, default_flow_style=True).strip(),
        description='原始折线顶点 [x0,y0, x1,y1, ...]，每个内部拐角用贝塞尔平滑',
    )
    degree_arg = DeclareLaunchArgument(
        'bezier_degree',
        default_value=str(DEFAULT_BEZIER_DEGREE),
        description='拐角贝塞尔阶数：4 或 5',
    )
    spacing_arg = DeclareLaunchArgument(
        'anchor_spacing',
        default_value=str(DEFAULT_ANCHOR_SPACING),
        description='贝塞尔锚点沿边间距 m；拐角退让距离 = (bezier_degree - 2) * 该值',
    )
    yaw_arg = DeclareLaunchArgument(
        'yaw_mode',
        default_value='tangent',
        description="每点朝向：'tangent' 取切向角；'linear' 在 start_yaw->goal_yaw 间线性均布",
    )
    # 仿真时钟：与 collision_monitor.launch.py 默认一致取 true。
    sim_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时钟 /clock',
    )
    # 基准位姿：折线顶点将变换到该位姿坐标系
    bp_arg = DeclareLaunchArgument(
        'base_pose',
        default_value='[0.0, 0.0, 0.0]',
        description='基准位姿 [x, y, yaw]，折线顶点变换到该坐标系',
    )
    # 生成器启动延迟（用 launch.TimerAction 等 path_follower 就绪）
    delay_arg = DeclareLaunchArgument(
        'start_delay',
        default_value='5.0',
        description='生成器节点延迟启动秒数',
    )

    # 碰撞监控（含 lifecycle_manager）：/cmd_vel_raw -> /cmd_vel_nav。
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
        waypoints = _parse_float_list(
            LaunchConfiguration('waypoints').perform(context), 'waypoints')
        base_pose = _parse_float_list(
            LaunchConfiguration('base_pose').perform(context), 'base_pose')
        bezier_degree = int(LaunchConfiguration('bezier_degree').perform(context))
        anchor_spacing = float(LaunchConfiguration('anchor_spacing').perform(context))
        yaw_mode = LaunchConfiguration('yaw_mode').perform(context)
        sim = LaunchConfiguration('use_sim_time').perform(context) == 'true'

        generator_node = Node(
            package='path_follower',
            executable='corner_bezier_path_generator.py',
            name='corner_bezier_path_generator',
            output='screen',
            parameters=[{
                'waypoints': waypoints,
                'bezier_degree': bezier_degree,
                'anchor_spacing': anchor_spacing,
                'base_pose': base_pose,
                'resolution': 0.005,
                'yaw_mode': yaw_mode,
                'start_yaw': 0.0,
                'goal_yaw': 1.5707963267948966,
                'frame_id': 'map',  # odom
                'republish': True,
                'period': 40.0,
                'alternate_direction': True,
                'use_sim_time': sim,
            }],
            remappings=[('~/path', '/path')],
        )
        # 节点晚 start_delay 秒启动。
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
            }],
            remappings=[
                ('~/path', '/path'),
                ('~/odom', '/odom'),  # linorobot2
                # ('~/odom', '/odom/wheel'),  # w1-sim
                # ('~/odom', '/feedback/odom'),  # w1-omni
                ('~/cmd_vel', '/cmd_vel_nav'),  # to velocity_smoother (旁路 collision_monitor)
                # ('~/cmd_vel', '/cmd_vel_raw'),  # to collision_monitor 输入 -> /cmd_vel_nav 输出
            ],
        )
        return [generator, follower]

    return LaunchDescription([
        wp_arg,
        degree_arg,
        spacing_arg,
        yaw_arg,
        sim_arg,
        bp_arg,
        delay_arg,
        collision_monitor,
        # velocity_smoother,
        OpaqueFunction(function=_make_nodes),
    ])
