from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # AprilTag 检测节点（常驻）
    apriltag_node = Node(package='mobile_perception',
                         executable='apriltag_detection_node',
                         name='apriltag_detection_node',
                         output='screen',
                         respawn=True,
                         respawn_delay=2.0)

    # 回充服务节点（常驻，等待调用 /auto_charing）
    service_node = Node(package='auto_charging_bt',
                        executable='auto_charging_service_node',
                        name='auto_charging_service',
                        parameters=[{
                            "backward.speed": -0.05,
                            "backward.duration": 2.0,
                            "undocking.speed": 0.10,
                            "undocking.duration": 2.0,
                        }],
                        output='screen')

    return LaunchDescription([apriltag_node, service_node])
