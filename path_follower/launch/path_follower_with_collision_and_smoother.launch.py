"""完整 bringup: path_follower + collision_monitor + velocity_smoother。

拓扑 —— 速度链是两级串联, collision_monitor 是挂在速度链上的**旁路探头**:

    path_follower ──/cmd_vel_nav──┬──► velocity_smoother ──/control/cmd_vel_nav──► 底盘
                                  │
                                  └──► collision_monitor ──/cmd_vel_nav1──► (无订阅者, 丢弃)
                                             │
                                             └──► /obstacle_status ──► path_follower

关键: collision_monitor **不在**速度链上, 它订阅 /cmd_vel_nav 纯粹是为了触发一次
障碍评估(上游 Nav2 的设计是"收到 cmd_vel 才评估"), 评估结果走 obstacle_status 回到
path_follower, 由 path_follower 自己的 VelocityTrackMode 状态机(kSlowdown/kStop +
迟滞 + 两段式速度台阶模型)执行减速与停车。它自己那路限速后的 cmd_vel 输出
(/cmd_vel_nav1) 是刻意悬空的 —— 若把它接回速度链, 等于同一个障碍被 collision_monitor
和 path_follower 各刹一次, 会和速度模型互相打架。

各级话题的来源:
  - path_follower 的 ~/cmd_vel : 本文件 remapping -> /cmd_vel_nav
  - collision_monitor 的进/出  : 本文件通过 collision_monitor.launch.py 的
                                 cmd_vel_in_topic / cmd_vel_out_topic 参数指定
                                 (与该文件的默认值一致, 这里显式写出以便一眼看全链路)
  - velocity_smoother 的进/出  : 本文件通过 velocity_smoother.launch.py 的
                                 cmd_vel_in_topic / cmd_vel_out_topic 参数指定。
                                 注意那两级都是走 **remapping** 而不是 yaml 的
                                 input_topic/output_topic —— 上游 Humble 的
                                 nav2_velocity_smoother 没有那两个参数, 详见
                                 velocity_smoother.launch.py 的注释。

生命周期:
  collision_monitor 和 velocity_smoother 都是 LifecycleNode, 不被 nav2_lifecycle_manager
  拉到 active 就不转发速度(表现为下游一条速度都收不到)。这里刻意保留两个**独立**的
  生命周期管理器(各自 include 自带), 而不是合并成一个: 合并后任一节点 bond 心跳断
  bond_timeout 秒, 管理器会把名下节点**整组** deactivate, 等于一个节点卡死拖垮另一个。
  autostart / bond_timeout 两个开关同时透传给两级, 详见 collision_monitor.launch.py。

运行:
  ros2 launch path_follower path_follower_with_collision_and_smoother.launch.py
  ros2 launch path_follower path_follower_with_collision_and_smoother.launch.py use_sim_time:=true
  # 手动接管生命周期(起来后停在 unconfigured):
  ros2 launch path_follower path_follower_with_collision_and_smoother.launch.py autostart:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# path_follower 的输出话题。velocity_smoother 与 collision_monitor 都订阅它:
# 前者是速度链的下一级, 后者只是拿它当障碍评估的触发信号。
FOLLOWER_OUT_TOPIC = "/cmd_vel_nav"
# collision_monitor 限速后的 cmd_vel 输出, 刻意无订阅者(见文件头说明)。
MONITOR_SINK_TOPIC = "/cmd_vel_nav1"
# 速度平滑器的输出, 直接进底盘。
CHASSIS_TOPIC = "/control/cmd_vel_nav"


def generate_launch_description():
    pkg_share = get_package_share_directory("path_follower")
    follower_params = os.path.join(pkg_share, "config", "path_follower.yaml")
    collision_params = os.path.join(pkg_share, "config", "collision_monitor.yaml")
    smoother_params = os.path.join(pkg_share, "config", "velocity_smoother.yaml")
    collision_launch = os.path.join(pkg_share, "launch", "collision_monitor.launch.py")
    smoother_launch = os.path.join(pkg_share, "launch", "velocity_smoother.launch.py")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    bond_timeout = LaunchConfiguration("bond_timeout")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time (/clock). 同时作用于三个节点。",
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description=(
            "true = 启动时自动把 collision_monitor 和 velocity_smoother 都拉到 active。"
        ),
    )
    bond_timeout_arg = DeclareLaunchArgument(
        "bond_timeout",
        default_value="4.0",
        description=(
            "两级托管节点的 bond 心跳看门狗超时(s)，0.0 关闭。"
            "详见 collision_monitor.launch.py。"
        ),
    )

    # 轨迹跟踪: 速度指令直接送进平滑器, 同时被碰撞监控旁路订阅用于触发障碍评估。
    path_follower_node = Node(
        package="path_follower",
        executable="path_follower_node",
        name="path_follower_node",
        output="screen",
        parameters=[follower_params, {"use_sim_time": use_sim_time}],
        remappings=[
            ("~/path", "/path"),
            ("~/tracking_path", "/tracking_path"),
            # ("~/odom", "/odom"),
            ("~/odom", "/feedback/odom"),  # w1
            ("~/cmd_vel", FOLLOWER_OUT_TOPIC),  # -> velocity_smoother (+ collision_monitor 旁路)
            ("~/navigation_status", "/navigation_status"),
            ("~/stop", "/stop"),
        ],
    )

    # 两个 include 各自套一层 scoped GroupAction —— 这不是可有可无的整洁, 是必须的:
    # IncludeLaunchDescription **不建立作用域**(include_launch_description.py:184 只是发一串
    # SetLaunchConfiguration 再把子 description 塞进父作用域), 而 DeclareLaunchArgument 是
    # "已经有值就不套默认值"(declare_launch_argument.py:205)。两个子 launch 都声明了同名的
    # params_file, 先 include 的 collision_monitor 会把它占成 collision_monitor.yaml, 后面
    # velocity_smoother 的默认值就永远套不上 —— 结果是 velocity_smoother 被喂了碰撞监控的
    # 参数文件, 自己的参数一个都没进去, 而且不报任何错(真机上已经踩过一次)。
    # scoped=True 让子 launch 的声明在退出 group 时弹掉; forwarding=True(默认)保证父作用域
    # 的值仍可见。下面同时显式传 params_file, 双保险且一眼看清各自吃的哪份 yaml。

    # 旁路探头: 碰撞监控(含生命周期管理器)。产出 obstacle_status 供 path_follower 用,
    # 其 cmd_vel 输出丢弃。
    collision_monitor = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(collision_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": collision_params,
                    "autostart": autostart,
                    "bond_timeout": bond_timeout,
                    "cmd_vel_in_topic": FOLLOWER_OUT_TOPIC,
                    "cmd_vel_out_topic": MONITOR_SINK_TOPIC,
                }.items(),
            )
        ],
        scoped=True,
    )

    # 速度链末级: 速度平滑(含生命周期管理器)。
    velocity_smoother = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(smoother_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": smoother_params,
                    "autostart": autostart,
                    "bond_timeout": bond_timeout,
                    "cmd_vel_in_topic": FOLLOWER_OUT_TOPIC,
                    "cmd_vel_out_topic": CHASSIS_TOPIC,
                }.items(),
            )
        ],
        scoped=True,
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            autostart_arg,
            bond_timeout_arg,
            path_follower_node,
            collision_monitor,
            velocity_smoother,
        ]
    )
