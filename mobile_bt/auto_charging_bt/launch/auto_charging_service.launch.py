from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('auto_charging_bt'),
        'config',
        'auto_charging_bt_params.yaml')

    # AprilTag 检测节点（常驻）
    apriltag_node = Node(package='mobile_perception',
                         executable='apriltag_alignment_tf_node',
                         name='apriltag_alignment_tf_node',
                         output='screen',
                         respawn=True,
                         respawn_delay=2.0)

    # 回充服务节点（常驻，等待调用 /auto_charging）
    service_node = Node(package='auto_charging_bt',
                        executable='auto_charging_service_node',
                        name='auto_charging_service',
                        parameters=[params_file],
                        output='screen')

    return LaunchDescription([apriltag_node, service_node])
