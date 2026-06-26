from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_simulation_arg = DeclareLaunchArgument(
        'use_simulation',
        default_value='false',
        description='是否使用仿真模式'
    )
    use_simulation = LaunchConfiguration('use_simulation')
    urdf_path = os.path.join(
        get_package_share_directory('dexforce_description'),
        'urdf',
        'DexforceW1V020.urdf'
    )

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    def launch_setup(context, *args, **kwargs):
        # 根据仿真模式选择话题
        use_sim = LaunchConfiguration('use_simulation').perform(context).lower() == 'true'

        return [
            # 机器人状态发布节点
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_description': robot_desc
                }],
            )
        ]

    return LaunchDescription([
        # 参数声明
        use_simulation_arg,
        # 使用OpaqueFunction来处理条件逻辑
        OpaqueFunction(function=launch_setup)
    ])