from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('loop_navigation_bt')

    # 行为树配置文件路径
    bt_xml = os.path.join(pkg_dir, 'config', 'loop_nocharging_bt.xml')

    return LaunchDescription([
        Node(package='loop_navigation_bt',
             executable='loop_nocharging_bt_node',
             name='loop_nocharging_bt_node',
             output='screen',
             parameters=[{
                 'bt_xml': bt_xml,
             }])
    ])
