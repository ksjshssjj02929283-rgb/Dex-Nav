"""启动 nav2_velocity_smoother 节点及其生命周期管理器。

velocity_smoother 是 LifecycleNode，必须由 nav2_lifecycle_manager 拉到 active
状态才会工作；仅启动节点本身会停留在 unconfigured。

订阅 cmd_vel_in_topic (/cmd_vel_nav_path)，做加速度/死区平滑后发布到
cmd_vel_out_topic (/control/cmd_vel_nav)。

**为什么节点名和输入话题都带 _path 后缀**：整机上还有一套 Nav2 自主导航栈
(w1_nav2 -> nav2_bringup/navigation_launch.py)，它同样起一个叫 `velocity_smoother`
的节点、同样吃 `/cmd_vel_nav`、同样输出到 `/control/cmd_vel_nav`。两套栈要能同时常驻
（靠 lifecycle 状态互斥选用哪套），就必须先把**节点名**和**栈内部的速度链**错开——
同名节点在 DDS 层就已经打架：两个 `/velocity_smoother/change_state` 服务端，
生命周期管理器的请求会随机路由到其中一个（真机上踩过，表现为随机激活失败且不重试）。
输出话题 `/control/cmd_vel_nav` 刻意保持一致：那是 cmd_vel_mux 的既定入口
(navigation_stack, priority 1)，错开就得改 mux 配置，且会多出"两个导航源按优先级仲裁"
的新语义。谁在输出由 lifecycle 保证只有一套 active。

可单独运行：
  ros2 launch path_follower velocity_smoother.launch.py
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
    default_params = os.path.join(pkg_share, "config", "velocity_smoother.yaml")

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
        description="Full path to the velocity_smoother parameter file.",
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
            "bond 心跳看门狗超时(s)。nav2 的托管节点 activate 后会以 10Hz 向管理器发心跳，"
            "超过此值没心跳，管理器判定节点卡死/退出，会把整组托管节点 deactivate 掉。"
            "设为 0.0 关闭该看门狗（节点就只会因显式的生命周期调用才离开 active）。"
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
        default_value="velocity_smoother_path",
        description=(
            "节点名。默认带 _path 后缀与 Nav2 栈的 velocity_smoother 区分（见文件头）。"
            "生命周期管理器名随之为 lifecycle_manager_<node_name>。"
            "改名不影响参数加载：yaml 顶层用 /** 通配。"
        ),
    )
    # 话题用 remapping 而不是 yaml 的 input_topic/output_topic 指定 —— 见下方 remappings。
    cmd_vel_in_topic_arg = DeclareLaunchArgument(
        "cmd_vel_in_topic",
        default_value="/cmd_vel_nav_path",
        description="平滑器的输入速度话题（上游控制器的输出）。",
    )
    cmd_vel_out_topic_arg = DeclareLaunchArgument(
        "cmd_vel_out_topic",
        default_value="/control/cmd_vel_nav",
        description="平滑器的输出速度话题（发给底盘）。",
    )

    # 速度平滑节点。节点名由 node_name 参数给出；yaml 顶层用 /** 通配，改名不会漏参数。
    #
    # 话题用 remapping 指定，不用 yaml 的 input_topic / output_topic。那两个 key 有两种
    # 静默失效的方式，出问题时都不报错，只表现为节点默默挂到默认的 /cmd_vel 和
    # /cmd_vel_smoothed 上、整条速度链断掉：
    #   1) 本节点被喂了别的参数文件。真机上真实发生过：上层 bringup 里两个 include 都声明了
    #      同名的 params_file，而 IncludeLaunchDescription 不建作用域，velocity_smoother 因此
    #      吃到了 collision_monitor.yaml（详见 path_follower_with_collision_and_smoother
    #      .launch.py 里 GroupAction 的注释）。
    #   2) overlay 里没编出 nav2_velocity_smoother，ros2 launch 回落到 /opt/ros 的上游二进制。
    #      上游 Humble 把话题硬编码，压根没有这两个参数（`strings libvelocity_smoother_core.so`
    #      里找不到）；只有本仓 vendored 的那份（mobile_navigation/nav2_velocity_smoother/
    #      src/velocity_smoother.cpp:94）才加了它们。
    # remapping 作用在 rcl 层，上面两种情况都照样生效，所以以它为准。
    velocity_smoother_node = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name=node_name,
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[
            ("cmd_vel", cmd_vel_in_topic),
            ("cmd_vel_smoothed", cmd_vel_out_topic),
        ],
    )

    # 生命周期管理器：自动把 velocity_smoother 拉到 active。use_lifecycle_manager=false 时不起（交给上层）。
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name=["lifecycle_manager_", node_name],
        output="screen",
        condition=IfCondition(use_lifecycle_manager),
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            # 嵌套一层是必须的：launch_ros 把"成员全是 Substitution 的列表"当成一个拼接后的
            # **字符串**，直接写 [node_name] 会让 node_names 变成 string 而不是 string[]，
            # 管理器加载参数时类型不符。多包一层，外层才被当作真正的列表逐项求值。
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
            velocity_smoother_node,
            lifecycle_manager_node,
        ]
    )

