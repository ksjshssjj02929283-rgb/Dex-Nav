# 公共算法参数集中在 config/rgbd_to_pointcloud.yaml。
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('dexe_sensor_process'), 'config', 'rgbd_to_pointcloud.yaml'),
        description='公共 ROS 参数配置文件',
    )

    front_rgbd_node = Node(
        package='dexe_sensor_process',
        executable='rgbd_to_pointcloud',
        name='front_rgbd_to_pointcloud',
        parameters=[LaunchConfiguration('params_file'), {
            'camera_name': 'front',
            'input_topic': '/orbbec/front',
            'output_topic_raw': '/orbbec/front/pointcloud2_raw',
            'output_topic_filtered': '/orbbec/front/pointcloud2_filtered',
            'output_topic_laserscan': '/orbbec/front/laser_scan',
        }],
        output='screen',
    )

    rear_rgbd_node = Node(
        package='dexe_sensor_process',
        executable='rgbd_to_pointcloud',
        name='rear_rgbd_to_pointcloud',
        parameters=[LaunchConfiguration('params_file'), {
            'camera_name': 'rear',
            'input_topic': '/orbbec/rear',
            'output_topic_raw': '/orbbec/rear/pointcloud2_raw',
            'output_topic_filtered': '/orbbec/rear/pointcloud2_filtered',
            'output_topic_laserscan': '/orbbec/rear/laser_scan',
        }],
        output='screen',
    )

    return LaunchDescription([
        params_file_arg,
        front_rgbd_node,
        rear_rgbd_node,
    ])
