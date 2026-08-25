"""
Spawn the W1 differential-drive simulation robot.

URDF 为 dexforce_description 可视化模型的仿真派生版（见 scripts/gen_w1_diff_sim_urdf.py），
话题契约：生产导航 /control/cmd_vel_nav 经仿真桥接后由 /control/cmd_vel 订阅、
/feedback/odom + odom→base_footprint TF、
/laser_scan/front|rear、/imu/chassis、/orbbec/{front,rear}/{rgb,depth,camera_info}。
"""
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SENSOR_NAMES = {
    'rgbd_front': 'realsense_front',
    'rgbd_rear': 'realsense_rear',
    'lidar_front': 'laser_front',
    'lidar_rear': 'laser_rear',
    'imu': 'w1_imu',
}


def _filtered_robot_description(urdf_path, enabled_sensors):
    """保留完整机器人 TF，仅移除 YAML 中关闭的 Gazebo 传感器插件。"""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    sensors_by_name = {
        sensor.get('name'): (gazebo, sensor)
        for gazebo in root.findall('gazebo')
        for sensor in gazebo.findall('sensor')
    }
    for config_name, sensor_name in SENSOR_NAMES.items():
        if sensor_name not in sensors_by_name:
            raise ValueError(f'W1 仿真 URDF 缺少传感器: {sensor_name}')
        if not enabled_sensors[config_name]:
            gazebo, sensor = sensors_by_name[sensor_name]
            gazebo.remove(sensor)
    return ET.tostring(root, encoding='unicode')


def _create_robot_nodes(context, urdf_path):
    """在 launch 配置解析后生成与传感器开关一致的机器人节点。"""
    enabled_sensors = {
        config_name: LaunchConfiguration(
            f'use_{config_name}').perform(context).lower() == 'true'
        for config_name in SENSOR_NAMES
    }
    robot_description = _filtered_robot_description(
        urdf_path, enabled_sensors)

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}])

    # robot_state_publisher 会以 TRANSIENT_LOCAL 发布过滤后的 robot_description；
    # spawn_entity 等待该话题，从而无需为不同传感器组合生成临时 URDF 文件。
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'w1_diff',
                   '-topic', 'robot_description',
                   '-x', x_pose,
                   '-y', y_pose,
                   '-z', '0.05'],
        output='screen')

    cmd_vel_bridge = Node(
        package='w1_diff_gazebo',
        executable='sim_cmd_vel_bridge',
        name='sim_cmd_vel_bridge',
        output='screen',
        parameters=[{
            'input_topic': '/control/cmd_vel_nav',
            'output_topic': '/control/cmd_vel',
            'use_sim_time': use_sim_time,
        }])
    return [robot_state_publisher, spawn_entity, cmd_vel_bridge]


def generate_launch_description() -> LaunchDescription:
    urdf_path = os.path.join(
        get_package_share_directory('w1_diff_gazebo'),
        'urdf', 'w1_diff_sim.urdf')

    declare_x = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y = DeclareLaunchArgument('y_pose', default_value='0.0')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    sensor_arguments = [
        DeclareLaunchArgument(
            f'use_{config_name}', default_value='true',
            choices=['true', 'false'])
        for config_name in SENSOR_NAMES
    ]

    return LaunchDescription([
        declare_x,
        declare_y,
        declare_use_sim_time,
        *sensor_arguments,
        OpaqueFunction(
            function=_create_robot_nodes,
            kwargs={'urdf_path': urdf_path}),
    ])
