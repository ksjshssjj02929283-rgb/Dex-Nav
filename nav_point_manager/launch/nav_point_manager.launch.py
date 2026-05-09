from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('nav_point_manager'), 'params',
            'nav_point_manager_params.yaml'
        ]),
        description='Full path to the ROS2 parameters file')

    # 创建节点 - 直接使用params文件中配置的参数
    nav_point_manager_node = Node(package='nav_point_manager',
                                  executable='nav_point_manager_node',
                                  name='nav_point_manager_node',
                                  output='screen',
                                  parameters=[
                                      LaunchConfiguration('params_file'),
                                  ],
                                  remappings=[])

    return LaunchDescription([params_file_arg, nav_point_manager_node])
