"""Run front/rear processing pipelines in one ROS node."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _camera_node(context):
    cameras = [name for name in ('front', 'rear')
               if LaunchConfiguration('enable_' + name).perform(context).lower()
               in ('true', '1')]
    if not cameras:
        return []
    overrides = {'cameras': cameras}
    # Empty arguments retain the values in params_file.
    for name, convert in (
        ('use_sim_time', lambda value: value.lower() in ('true', '1')),
        ('depth_max_range', float),
        ('ground_filter_min_height', float),
    ):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = convert(value)
    return [Node(
        package='dexe_sensor_process',
        executable='rgbd_to_pointcloud',
        name='rgbd_to_pointcloud',
        parameters=[LaunchConfiguration('params_file'), overrides],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('dexe_sensor_process'),
                'config', 'rgbd_to_pointcloud.yaml'),
            description='公共 ROS 参数配置文件'),
        DeclareLaunchArgument('enable_front', default_value='true',
                              choices=['true', 'false', 'True', 'False', '1', '0']),
        DeclareLaunchArgument('enable_rear', default_value='true',
                              choices=['true', 'false', 'True', 'False', '1', '0']),
        DeclareLaunchArgument('use_sim_time', default_value=''),
        DeclareLaunchArgument('depth_max_range', default_value=''),
        DeclareLaunchArgument('ground_filter_min_height', default_value=''),
        OpaqueFunction(function=_camera_node),
    ])
