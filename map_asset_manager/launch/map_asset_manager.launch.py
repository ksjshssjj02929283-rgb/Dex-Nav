from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('map_asset_manager'), 'params',
            'map_asset_manager_params.yaml'
        ]),
        description='Full path to the ROS2 parameters file')

    map_asset_manager_node = Node(
        package='map_asset_manager',
        executable='map_asset_manager_node',
        name='map_asset_manager',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[])

    return LaunchDescription([params_file_arg, map_asset_manager_node])
