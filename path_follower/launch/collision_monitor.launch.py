"""启动 nav2_collision_monitor 节点及其生命周期管理器。

collision_monitor 是 LifecycleNode,必须由 nav2_lifecycle_manager 拉到 active
状态才会工作;仅启动节点本身会停留在 unconfigured。

**节点名默认带 _path 后缀**：整机上还有一套 Nav2 自主导航栈，两套要能同时常驻
（靠 lifecycle 状态互斥选用哪套），节点名和栈内速度链就必须错开——同名节点在 DDS 层
就已经打架，生命周期请求会随机路由。理由详见 velocity_smoother.launch.py 文件头。
当前 Nav2 的 navigation_launch.py 并不起 collision_monitor，这里先行改名是为了往后
它加进去时不用再动一次。

可单独运行:
  ros2 launch path_follower collision_monitor.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("path_follower")
    default_params = os.path.join(pkg_share, "config", "collision_monitor.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    bond_timeout = LaunchConfiguration("bond_timeout")
    cmd_vel_in_topic = LaunchConfiguration("cmd_vel_in_topic")
    cmd_vel_out_topic = LaunchConfiguration("cmd_vel_out_topic")
    node_name = LaunchConfiguration("node_name")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time (/clock).",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Full path to the collision_monitor parameter file.",
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description=(
            "true = 生命周期管理器启动时自动 configure+activate；"
            "false = 停在 unconfigured，需手动调 /lifecycle_manager_<node_name>/manage_nodes。"
        ),
    )
    bond_timeout_arg = DeclareLaunchArgument(
        "bond_timeout",
        default_value="4.0",
        description=(
            "bond 心跳看门狗超时(s)。托管节点 activate 后以 10Hz 向管理器发心跳，"
            "超过此值没心跳，管理器判定节点卡死，会把该管理器名下的节点整组 deactivate。"
            "设为 0.0 关闭该看门狗。"
        ),
    )
    use_lifecycle_manager_arg = DeclareLaunchArgument(
        "use_lifecycle_manager",
        default_value="true",
        description=(
            "true = 起本文件自带的 lifecycle_manager_<node_name>（单独运行时用）；"
            "false = 不起，节点交给上层管理器编排（path_navigation_bringup 把它并进 "
            "lifecycle_manager_path_navigation，随导航后端一起 STARTUP/PAUSE）。"
            "此时 autostart / bond_timeout 不起作用。"
        ),
    )
    node_name_arg = DeclareLaunchArgument(
        "node_name",
        default_value="collision_monitor_path",
        description=(
            "节点名。默认带 _path 后缀与 Nav2 栈区分（见文件头）。"
            "生命周期管理器名随之为 lifecycle_manager_<node_name>。"
            "改名不影响参数加载：yaml 顶层用 /** 通配。"
        ),
    )
    # 话题做成参数, 便于上层 bringup 显式写出接线; 默认值即当前部署的接法。
    cmd_vel_in_topic_arg = DeclareLaunchArgument(
        "cmd_vel_in_topic",
        default_value="/cmd_vel_nav_path",
        description=(
            "collision_monitor 的输入速度话题, 即 path_follower 的输出。"
            "本部署里订阅它只用于障碍评估的速度上下文与调度状态机(navigation_type=schedule), "
            "并不做速度闸门。"
        ),
    )
    cmd_vel_out_topic_arg = DeclareLaunchArgument(
        "cmd_vel_out_topic",
        default_value="/cmd_vel_nav_path_sink",
        description=(
            "collision_monitor 限速后的 cmd_vel 输出。本部署刻意悬空(无订阅者): "
            "减速/停车由 path_follower 自己的 VelocityTrackMode 状态机按 obstacle_status "
            "执行, 若把这一路接回速度链会造成同一障碍被两处各刹一次。"
        ),
    )

    # 碰撞监控节点。节点名由 node_name 参数给出; yaml 顶层用 /** 通配, 改名不会漏参数。
    # 真正被下游消费的产出是 obstacle_status, 不是它的 cmd_vel 输出(见话题参数说明)。
    collision_monitor_node = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name=node_name,
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        # 左侧是 yaml 里 cmd_vel_in_topic / cmd_vel_out_topic 的值(相对名),
        # 右侧由 launch 参数给出。改 yaml 的话题名时这里要跟着改。
        remappings=[
            ("cmd_vel_raw", cmd_vel_in_topic),
            ("cmd_vel", cmd_vel_out_topic),
        ],
    )

    # 生命周期管理器: 自动把 collision_monitor 拉到 active。use_lifecycle_manager=false 时不起(交给上层)。
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name=["lifecycle_manager_", node_name],
        output="screen",
        condition=IfCondition(use_lifecycle_manager),
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            # 嵌套一层是必须的: launch_ros 把"成员全是 Substitution 的列表"当成一个拼接后的
            # **字符串**, 直接写 [node_name] 会让 node_names 变成 string 而不是 string[]。
            {"node_names": [[node_name]]},
            {"bond_timeout": bond_timeout},
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            params_file_arg,
            autostart_arg,
            bond_timeout_arg,
            cmd_vel_in_topic_arg,
            cmd_vel_out_topic_arg,
            node_name_arg,
            use_lifecycle_manager_arg,
            collision_monitor_node,
            lifecycle_manager_node,
        ]
    )
