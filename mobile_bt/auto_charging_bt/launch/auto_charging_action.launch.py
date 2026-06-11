from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # AprilTag 检测节点（常驻）
    apriltag_node = Node(package='mobile_perception',
                         executable='apriltag_alignment_tf_node',
                         name='apriltag_alignment_tf_node',
                         output='screen',
                         respawn=True,
                         respawn_delay=2.0)

    # 回充action节点（带GDB调试）
    action_node = Node(
        package='auto_charging_bt',
        executable='auto_charging_action_server_node',
        name='auto_charging_action_server_node',
        parameters=[{
            "backward.initial_speed": -0.02,
            "backward.connected_speed": -0.005,
            "backward.max_distance": 0.2,
            "backward.stable_duration": 1.0,
            "backward.max_duration": 8.0,
            "backward.control_rate": 50.0,
            "backward.max_dt": 0.1,
            "undocking.speed": 0.10,
            "undocking.duration": 2.0,
        }],
        output='screen',
        # 关键：添加 GDB prefix
        # prefix=['gdb', '-ex', 'run', '-ex', 'bt', '-ex', 'quit', '--args'],
        # 如果需要在新终端中调试（可以看到GUI），使用下面这行：
        # prefix=['xterm', '-e', 'gdb', '-ex', 'run', '--args'],
    )

    return LaunchDescription([apriltag_node, action_node])
