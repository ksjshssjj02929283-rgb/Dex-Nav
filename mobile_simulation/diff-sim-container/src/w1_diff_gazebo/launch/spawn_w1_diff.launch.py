"""Spawn W1 差速仿真机器人：robot_state_publisher + spawn_entity。

URDF 为 dexforce_description 可视化模型的仿真派生版（见 scripts/gen_w1_diff_sim_urdf.py），
话题契约：/control/cmd_vel 订阅、/feedback/odom + odom→base_footprint TF、
/laser_scan/front|rear、/imu/chassis、/orbbec/{front,rear}/{rgb,depth,camera_info}。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    urdf_path = os.path.join(
        get_package_share_directory('w1_diff_gazebo'), 'urdf', 'w1_diff_sim.urdf')
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_x = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y = DeclareLaunchArgument('y_pose', default_value='0.0')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'w1_diff',
                   '-file', urdf_path,
                   '-x', x_pose,
                   '-y', y_pose,
                   '-z', '0.05'],
        output='screen')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}])

    return LaunchDescription([
        declare_x,
        declare_y,
        declare_use_sim_time,
        spawn_entity,
        robot_state_publisher,
    ])
