"""完整 bringup: path_follower + collision_monitor + velocity_smoother。

注意: path_follower_node 现在是 LifecycleNode, 本文件**不**负责把它拉到 active
(这里的 autostart 只作用于 collision_monitor_path / velocity_smoother_path 两级速度链)。
整栈请用 `nav2_schedule_planner/path_navigation_bringup.launch.py`, 它带一个
lifecycle_manager_path_navigation 按 Follower->Schedule 顺序激活两个 Server。
只想单独调跟踪时手动驱动: ros2 lifecycle set /path_follower_node configure / activate。

拓扑 —— 速度链是两级串联, collision_monitor 是挂在速度链上的**旁路探头**:

    path_follower ──/cmd_vel_nav_path──┬──► velocity_smoother_path ──/control/cmd_vel_nav──► 底盘
                                       │
                                       └──► collision_monitor_path ──/cmd_vel_nav_path_sink──► (无订阅者, 丢弃)
                                                  │
                                                  └──► /obstacle_status ──► path_follower

与 Nav2 栈的隔离: 整机上另有一套 Nav2 自主导航栈(w1_nav2 -> navigation_launch.py), 它同样
起一个叫 `velocity_smoother` 的节点、同样吃 `/cmd_vel_nav`。两套要能同时常驻(靠 lifecycle
状态互斥选用哪套), **节点名和栈内部速度链必须错开** —— 同名节点在 DDS 层就已经打架:
两个 `/velocity_smoother/change_state` 服务端, 生命周期管理器的请求会随机路由到其中一个
(真机踩过, 表现为随机激活失败且不重试)。所以本栈内部一律带 _path 后缀。
**唯独输出 /control/cmd_vel_nav 保持不变**: 那是 cmd_vel_mux 的既定入口
(navigation_stack, priority 1), 错开就得改 mux 配置, 还会多出"两个导航源按优先级仲裁"
的新语义; 同一时刻只有一套 active 由 lifecycle 保证。

关键: collision_monitor **不在**速度链上, 它订阅 path_follower 的输出只作为障碍评估的速度
上下文(100 Hz 控制环, 无速度时也做 STOP 检测), 评估结果走 obstacle_status 回到
path_follower, 由 path_follower 自己的 VelocityTrackMode 状态机(kSlowdown/kStop +
迟滞 + 两段式速度台阶模型)执行减速与停车。它自己那路限速后的 cmd_vel 输出
(/cmd_vel_nav_path_sink) 是刻意悬空的 —— 若把它接回速度链, 等于同一个障碍被 collision_monitor
和 path_follower 各刹一次, 会和速度模型互相打架。

各级话题的来源:
  - path_follower 的 ~/cmd_vel : 本文件 remapping -> /cmd_vel_nav_path
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
  use_own_lifecycle_managers:=false 时两个自带管理器都不起, 两级交给上层管理器编排 ——
  path_navigation_bringup.launch.py 就是这么用的: 把它们和 Follower/Schedule 并成一组,
  随导航后端一起 STARTUP/PAUSE, 未选中路径导航时整条速度链停在 unconfigured。

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

# path_follower 的输出话题。velocity_smoother_path 与 collision_monitor_path 都订阅它:
# 前者是速度链的下一级, 后者只是拿它当障碍评估的触发信号。
# 带 _path 后缀与 Nav2 栈的 /cmd_vel_nav 错开(见文件头"与 Nav2 栈的隔离")。
FOLLOWER_OUT_TOPIC = "/cmd_vel_nav_path"
# collision_monitor 限速后的 cmd_vel 输出, 刻意无订阅者(见文件头说明)。
MONITOR_SINK_TOPIC = "/cmd_vel_nav_path_sink"
# 速度平滑器的输出, 直接进底盘。与 Nav2 栈共用——这是 cmd_vel_mux 的入口, 刻意不错开。
CHASSIS_TOPIC = "/control/cmd_vel_nav"
# 本栈两个 nav2 托管节点的节点名(与 Nav2 栈区分)。显式写出而不是吃子 launch 的默认值,
# 便于一眼看全: 出问题时要按这两个名字去查 /<name>/change_state 和 lifecycle_manager_<name>。
SMOOTHER_NODE_NAME = "velocity_smoother_path"
MONITOR_NODE_NAME = "collision_monitor_path"


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
    use_own_lifecycle_managers = LaunchConfiguration("use_own_lifecycle_managers")

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

    use_own_lifecycle_managers_arg = DeclareLaunchArgument(
        "use_own_lifecycle_managers",
        default_value="true",
        description=(
            "true = 两级各起自带的 lifecycle_manager_<node_name>（单独运行本文件时用）；"
            "false = 都不起, 交给上层管理器（path_navigation_bringup）。"
            "false 时 autostart / bond_timeout 不起作用。"
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
                    "use_lifecycle_manager": use_own_lifecycle_managers,
                    "cmd_vel_in_topic": FOLLOWER_OUT_TOPIC,
                    "cmd_vel_out_topic": MONITOR_SINK_TOPIC,
                    "node_name": MONITOR_NODE_NAME,
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
                    "use_lifecycle_manager": use_own_lifecycle_managers,
                    "cmd_vel_in_topic": FOLLOWER_OUT_TOPIC,
                    "cmd_vel_out_topic": CHASSIS_TOPIC,
                    "node_name": SMOOTHER_NODE_NAME,
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
            use_own_lifecycle_managers_arg,
            path_follower_node,
            collision_monitor,
            velocity_smoother,
        ]
    )
