"""联调 launch：TrackingPath 轨迹生成器 + path_follower 跟踪节点。

串联拓扑：generator ──/tracking_path──► path_follower ──/cmd_vel──► 底盘

用途：测试 path_follower 的 onTrackingPath 段级参数（mode/backward/
target_yaw/hold_direction/max_speed/padding）。几何默认直线段，交替正反发布。

用法：
  ros2 launch path_follower tracking_path_demo.launch.py
可选参数（覆盖默认形状与段级参数，值为合法 YAML 数组/标量字符串）：
  ros2 launch path_follower tracking_path_demo.launch.py \
      mode:=1 backward:=1 target_yaw:=nan max_speed:=0.3
  ros2 launch path_follower tracking_path_demo.launch.py \
      path_type:=bezier control_points:="[0.0,0.0,2.0,0.0,2.0,-2.0,4.0,-2.0]"

注：本 demo 不含 collision_monitor（path_follower 直发 /cmd_vel 给底盘）。
     如需碰撞监控，参考 bezier_demo.launch.py / path_follower_with_collision.launch.py。
"""

import os
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ---------------------------------------------------------------------------
# 默认值
# ---------------------------------------------------------------------------
# 默认直线段控制点 [x0,y0,x1,y1]（与默认 path_type=line 配套）。
# 切到 path_type:=bezier 时需同时传 8 个控制点 control_points:="[x0,y0,...,x3,y3]"
# DEFAULT_CONTROL_POINTS = [0.0, 0.0, 2.0, 0.0]
DEFAULT_CONTROL_POINTS = [0.0, 0.0, 2.0, 0.0, 2.0, -2.0, 4.0, -2.0]


def _parse_yaml(raw: str, default: Any):
    """把命令行传入的 YAML 字符串解析为 Python 对象。

    控制点必须为 float 列表（ROS2 参数要 double 数组）。
    """
    try:
        parsed = yaml.safe_load(raw)
    except yaml.YAMLError as e:
        raise ValueError(f"参数不是合法的 YAML: {raw!r} ({e})")
    if isinstance(parsed, list):
        try:
            return [float(x) for x in parsed]
        except (TypeError, ValueError) as e:
            raise ValueError(f"列表各项必须为数值: {raw!r} ({e})")
    return default if parsed is None else parsed


def generate_launch_description():
    pkg_share = get_package_share_directory('path_follower')
    params_file = os.path.join(pkg_share, 'config', 'path_follower.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ---------------- 声明 launch 参数 ----------------
    # 几何
    path_type_arg = DeclareLaunchArgument(
        'path_type', default_value='bezier',
        description="轨迹类型：'line' / 'bezier'",
    )
    control_points_arg = DeclareLaunchArgument(
        'control_points',
        default_value=yaml.dump(DEFAULT_CONTROL_POINTS, default_flow_style=True).strip(),
        description='轨迹控制点：line 用 4 个 [x0,y0,x1,y1]；bezier 用 8 个 [x0,y0,...,x3,y3]',
    )
    resolution_arg = DeclareLaunchArgument(
        'resolution', default_value='0.005',
        description='弧长采样间距 m',
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='map',
        description='输出 TrackingPath 坐标系（需与 path_follower 定位坐标系一致并有 TF）',
    )
    base_pose_arg = DeclareLaunchArgument(
        'base_pose', default_value='[0.0, 0.0, 0.0]',
        description='基准位姿 [x, y, yaw]，控制点变换到该坐标系',
    )
    # 段级跟踪参数
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='1',
        description='跟踪方式：1=DIFF / 2=OMNI_TRANSLATION / 3=OMNI_TRANS_ROTATION',
    )
    backward_arg = DeclareLaunchArgument(
        'backward', default_value='0',
        description='倒车：0=FORWARD / 1=REVERSE（仅 DIFF 生效）',
    )
    target_yaw_arg = DeclareLaunchArgument(
        'target_yaw', default_value='1.57',
        description='到点目标朝向 rad；nan=到点不旋转',
    )
    hold_direction_arg = DeclareLaunchArgument(
        'hold_direction', default_value='-1.57',
        description='全向平移保持朝向 rad；nan=保持当前（仅 OMNI_TRANSLATION 生效）',
    )
    max_speed_arg = DeclareLaunchArgument(
        'max_speed', default_value='nan',
        description='段限速 m/s；nan=不限',
    )
    padding_arg = DeclareLaunchArgument(
        'padding', default_value='nan',
        description='车体膨胀 m；nan=默认',
    )
    # 发布
    alternate_arg = DeclareLaunchArgument(
        'alternate', default_value='true',
        description='是否交替正反几何发布',
    )
    period_arg = DeclareLaunchArgument(
        'period', default_value='30.0',
        description='重发周期 s（按路径时长调小）',
    )
    start_delay_arg = DeclareLaunchArgument(
        'start_delay', default_value='3.0',
        description='生成器节点延迟启动秒数（launch.TimerAction 控制时机，等 path_follower 就绪）',
    )
    # 仿真时钟
    sim_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='是否使用仿真时钟 /clock',
    )

    def _make_nodes(context):
        # 解析参数（控制点 / base_pose 走 YAML 解析，其余标量走对应类型）
        control_points = _parse_yaml(
            LaunchConfiguration('control_points').perform(context), DEFAULT_CONTROL_POINTS)
        base_pose = _parse_yaml(
            LaunchConfiguration('base_pose').perform(context), [0.0, 0.0, 0.0])
        sim = LaunchConfiguration('use_sim_time').perform(context) == 'true'

        generator_node = Node(
            package='path_follower',
            executable='tracking_path_generator.py',
            name='tracking_path_generator',
            output='screen',
            parameters=[{
                'tracking_path_topic': '/tracking_path',
                'path_type': LaunchConfiguration('path_type').perform(context),
                'control_points': control_points,
                'resolution': float(LaunchConfiguration('resolution').perform(context)),
                'frame_id': LaunchConfiguration('frame_id').perform(context),
                'base_pose': base_pose,
                'mode': int(LaunchConfiguration('mode').perform(context)),
                'backward': int(LaunchConfiguration('backward').perform(context)),
                'target_yaw': float(LaunchConfiguration('target_yaw').perform(context)),
                'hold_direction': float(LaunchConfiguration('hold_direction').perform(context)),
                'max_speed': float(LaunchConfiguration('max_speed').perform(context)),
                'padding': float(LaunchConfiguration('padding').perform(context)),
                'alternate': LaunchConfiguration('alternate').perform(context) == 'true',
                'period': float(LaunchConfiguration('period').perform(context)),
                'use_sim_time': sim,
            }],
        )
        # 节点晚 start_delay 秒启动（用 launch.TimerAction 替代脚本内 start_delay 计时器）。
        start_delay = float(LaunchConfiguration('start_delay').perform(context))
        generator = TimerAction(period=start_delay, actions=[generator_node])

        follower = Node(
            package='path_follower',
            executable='path_follower_node',
            name='path_follower_node',
            output='screen',
            parameters=[params_file, {'use_sim_time': sim}],
            remappings=[
                ('~/path', '/path'),  # 保留 nav_msgs/Path 入口，便于兼用旧测试
                ('~/tracking_path', '/tracking_path'),  # 本 demo 主输入
                ('~/odom', '/odom'),  # linorobot2
                # ('~/odom', '/odom/wheel'),  # w1-sim
                # ('~/odom', '/feedback/odom'),  # w1-omni
                ('~/cmd_vel', '/cmd_vel'),  # 直发底盘
                ('~/navigation_status', '/navigation_status'),
                ('~/stop', '/stop'),
            ],
        )
        return [generator, follower]

    return LaunchDescription([
        path_type_arg,
        control_points_arg,
        resolution_arg,
        frame_id_arg,
        base_pose_arg,
        mode_arg,
        backward_arg,
        target_yaw_arg,
        hold_direction_arg,
        max_speed_arg,
        padding_arg,
        alternate_arg,
        period_arg,
        start_delay_arg,
        sim_arg,
        OpaqueFunction(function=_make_nodes),
    ])
