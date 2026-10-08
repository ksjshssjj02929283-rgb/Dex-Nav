"""路径导航栈的整栈启动：path_follower + schedule_planner + 专用生命周期管理器。

这是路径导航的**推荐入口**，取代过去分别起
`path_follower/path_follower_with_collision_and_smoother.launch.py` 和
`nav2_schedule_planner/schedule_planner.launch.py` 两条命令的做法 —— 那两个 launch 现在
起出来的 path_follower_node / schedule_planner_node 都是 LifecycleNode，没有管理器拉起来
就停在 unconfigured，不接任务也不出速度。

拓扑与编排::

    lifecycle_manager_path_navigation
        │  正序 configure/activate、逆序 deactivate/cleanup
        ├─ 1. path_follower_node   （先就绪：Schedule 开始接任务时它必须已经能跟踪）
        └─ 2. schedule_planner_node（后就绪；停止时先停它，Follower 最后零速退出）

    schedule_planner_node ──/tracking_path──► path_follower_node ──/cmd_vel_nav_path──►
        velocity_smoother_path ──/control/cmd_vel_nav──► cmd_vel_mux ──► 底盘

两个 Server 放同一个管理器里是有意的：它们是不可分割的执行栈，任一个 bond 断掉时整组停止，
避免出现"Schedule 还在接任务但 Follower 已经不可用"或"Follower 还在出速度但上层状态已丢失"。
collision_monitor_path / velocity_smoother_path 各自保留**独立**的管理器（在
path_follower 那条 include 里自带），刻意不并进来：合并后任一节点 bond 断，整组会被
deactivate，等于一个节点卡死拖垮速度链。

运行::

    ros2 launch nav2_schedule_planner path_navigation_bringup.launch.py
    # 起来后停在 unconfigured，交给上层（command_board）编排：
    ros2 launch nav2_schedule_planner path_navigation_bringup.launch.py autostart:=false
    # 手动驱动：
    ros2 service call /lifecycle_manager_path_navigation/manage_nodes \
        nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"   # 0=STARTUP 1=PAUSE 2=RESET 3=SHUTDOWN
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 受本管理器编排的两个 Server。**顺序就是启动顺序**：正序 configure/activate，
# 逆序 deactivate/cleanup，所以 Follower 必须排在 Schedule 前面。
MANAGED_NODES = ["path_follower_node", "schedule_planner_node"]


def generate_launch_description():
    schedule_share = get_package_share_directory("nav2_schedule_planner")
    follower_share = get_package_share_directory("path_follower")

    schedule_params = os.path.join(schedule_share, "config", "schedule_planner.yaml")
    follower_bringup = os.path.join(
        follower_share, "launch", "path_follower_with_collision_and_smoother.launch.py")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    bond_timeout = LaunchConfiguration("bond_timeout")
    support_bond_timeout = LaunchConfiguration("support_bond_timeout")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time (/clock)。同时作用于本组全部节点。",
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description=(
            "true = 本管理器启动时自动把两个 Server 依次 configure+activate（当前部署方式）；"
            "false = 两个 Server 停在 unconfigured，等上层（command_board 按 "
            "navigation_backend 选择后端）调 manage_nodes 的 STARTUP。"
            "注意这**只管两个 Server**，速度链的 collision_monitor_path / "
            "velocity_smoother_path 由 support_autostart 单独控制。"
        ),
    )
    bond_timeout_arg = DeclareLaunchArgument(
        "bond_timeout",
        default_value="10.0",
        description=(
            "两个 Server 的 bond 心跳看门狗超时(s)，0.0 关闭。"
            "**刻意比 nav2 默认的 4.0 大**：path_follower 装载路线时有一段预摆舵角的同步等待，"
            "整体预算 3 秒且期间独占单线程执行器（path_follower.cpp 里 "
            "steering_angle_control 那段），bond 心跳也在这个执行器上。留 4 秒的话，"
            "真机负载一抖就会被管理器判定为节点卡死，把整组 deactivate —— 表现为任务跑一半"
            "机器人停住。根治要么把那段等待挪出执行器线程，要么换 MultiThreadedExecutor。"
        ),
    )
    support_autostart_arg = DeclareLaunchArgument(
        "support_autostart",
        default_value="true",
        description=(
            "速度链（collision_monitor_path / velocity_smoother_path）是否自动激活。"
            "默认 true：它们是**支撑链**不是导航后端，两套导航后端都要用，"
            "跟着 navigation_backend 一起停反而会让另一套也没速度。"
        ),
    )
    support_bond_timeout_arg = DeclareLaunchArgument(
        "support_bond_timeout",
        default_value="4.0",
        description="速度链两个节点的 bond 超时(s)，保持 nav2 默认值。",
    )

    # path_follower + collision_monitor_path + velocity_smoother_path。
    # 这条 include 里的 autostart 只作用于后两个（速度链）；path_follower_node 本身
    # 是 LifecycleNode，由下面的 lifecycle_manager_path_navigation 编排。
    follower_stack = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(follower_bringup),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": LaunchConfiguration("support_autostart"),
                    "bond_timeout": support_bond_timeout,
                }.items(),
            )
        ],
        scoped=True,
    )

    schedule_planner_node = Node(
        package="nav2_schedule_planner",
        executable="schedule_planner_node",
        name="schedule_planner_node",
        output="screen",
        parameters=[schedule_params, {"use_sim_time": use_sim_time}],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_path_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": MANAGED_NODES},
            {"bond_timeout": bond_timeout},
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            autostart_arg,
            bond_timeout_arg,
            support_autostart_arg,
            support_bond_timeout_arg,
            follower_stack,
            schedule_planner_node,
            lifecycle_manager,
        ]
    )
