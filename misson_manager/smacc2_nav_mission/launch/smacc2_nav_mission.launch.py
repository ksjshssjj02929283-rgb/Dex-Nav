import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('smacc2_nav_mission'),
        'config', 'waypoints.yaml'
    )
    return LaunchDescription([
        # 前提：nav_point_manager_node 已在外部启动（或在同一 launch 中启动）
        Node(
            package='smacc2_nav_mission',
            executable='smacc2_nav_mission_node',
            name='smacc2_nav_mission',
            output='screen',
            parameters=[config]   # 传入点位名称配置
        )
    ])
