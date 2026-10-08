"""组合 bringup: path_follower + nav2_collision_monitor 一条命令起齐安全栈。

串联拓扑:
    path_follower ──/cmd_vel_raw──► collision_monitor ──/cmd_vel──► 底盘

path_follower 的原始输出 (/cmd_vel) 在此重映射到 /cmd_vel_raw, 由 collision_monitor
做碰撞检测后再以 /cmd_vel 发往底盘。原 path_follower.launch.py 保持不动, 仍可单独
运行(那时 path_follower 直接发 /cmd_vel 给底盘, 无碰撞监控)。

运行:
  ros2 launch path_follower path_follower_with_collision.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("path_follower")
    follower_params = os.path.join(pkg_share, "config", "path_follower.yaml")
    collision_launch = os.path.join(pkg_share, "launch", "collision_monitor.launch.py")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time (/clock). 同时作用于两个节点。",
    )

    # 轨迹跟踪节点: 输出重映射到 /cmd_vel_raw 交给碰撞监控。
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
            # /cmd_vel_nav_path：本栈内部速度链，与 Nav2 栈的 /cmd_vel_nav 错开，
            # 与 collision_monitor.launch.py 的 cmd_vel_in_topic 默认值一致
            ("~/cmd_vel", "/cmd_vel_nav_path"),  # to velocity_smoother (旁路 collision_monitor)
            ("~/navigation_status", "/navigation_status"),
            ("~/stop", "/stop"),
        ],
    )

    # 碰撞监控 (含生命周期管理器): /cmd_vel_raw -> /cmd_vel。
    collision_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(collision_launch),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )


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
        use_sim_time_arg,
        path_follower_node,
        path_follower_lifecycle_manager,
        collision_monitor,
    ])
