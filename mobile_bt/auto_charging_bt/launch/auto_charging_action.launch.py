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
                         parameters=[{"target_forward_offset": 0.67}],
                         respawn=True,
                         respawn_delay=2.0)

    # 回充action节点（带GDB调试）
    action_node = Node(
        package='auto_charging_bt',
        executable='auto_charging_action_server_node',
        # name='auto_charging_action_server_node',
        parameters=[params_file],
        output='screen',
        # 关键：添加 GDB prefix
        # prefix=['gdb', '-ex', 'run', '-ex', 'bt', '-ex', 'quit', '--args'],
        # 如果需要在新终端中调试（可以看到GUI），使用下面这行：
        # prefix=['xterm', '-e', 'gdb', '-ex', 'run', '--args'],
    )

    return LaunchDescription([apriltag_node, action_node])
