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

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='rear',
        description='相机名称，用于构建默认话题名称',
    )

    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='',
        description='输入的 RGBD 话题名称（为空时使用默认规则：/orbbec/{camera_name}）',
    )

    output_topic_raw_arg = DeclareLaunchArgument(
        'output_topic_raw',
        default_value='',
        description='输出的原始点云话题（为空时使用默认规则：{input_topic}/pointcloud2_raw）',
    )

    output_topic_filtered_arg = DeclareLaunchArgument(
        'output_topic_filtered',
        default_value='',
        description='输出的预处理点云话题（为空时使用默认规则：{input_topic}/pointcloud2_filtered）',
    )

    output_topic_laserscan_arg = DeclareLaunchArgument(
        'output_topic_laserscan',
        default_value='',
        description='输出的 LaserScan 话题（为空时使用默认规则：{input_topic}/laser_scan）',
    )

    rgbd_to_pointcloud_node = Node(
        package='dexe_sensor_process',
        executable='rgbd_to_pointcloud',
        name='rgbd_to_pointcloud',
        parameters=[LaunchConfiguration('params_file'), {
            'camera_name': LaunchConfiguration('camera_name'),
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic_raw': LaunchConfiguration('output_topic_raw'),
            'output_topic_filtered': LaunchConfiguration('output_topic_filtered'),
            'output_topic_laserscan': LaunchConfiguration('output_topic_laserscan'),
        }],
        output='screen',
    )

    return LaunchDescription([
        params_file_arg,
        camera_name_arg,
        input_topic_arg,
        output_topic_raw_arg,
        output_topic_filtered_arg,
        output_topic_laserscan_arg,
        rgbd_to_pointcloud_node,
    ])
