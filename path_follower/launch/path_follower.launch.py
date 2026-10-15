"""Launch the path_follower node with its parameter file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("path_follower")
    params_file = os.path.join(pkg_share, "config", "path_follower.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time (/clock).",
    )

    follower_node = Node(
        package="path_follower",
        executable="path_follower_node",
        name="path_follower_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[
            ("~/path", "/path"),
            ("~/tracking_path", "/tracking_path"),
            # ("~/odom", "/odom"),
            ("~/odom", "/feedback/odom"),  # w1
            # ("~/cmd_vel", "/cmd_vel"),
            ("~/cmd_vel", "/control/cmd_vel_nav"),  # w1
            # ("~/cmd_vel", "/cmd_vel_nav_path"),  # w1 速度平滑器(见 *_with_collision_and_smoother)
            ("~/navigation_status", "/navigation_status"),
            ("~/stop", "/stop"),
        ],
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

    return LaunchDescription([use_sim_time_arg, follower_node, path_follower_lifecycle_manager])
