#!/usr/bin/env python3
"""
启动所有必要的服务：
- dexforce_description (机器人描述)
- joint_converter (关节转换)
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_simulation_arg = DeclareLaunchArgument(
        'use_simulation',
        default_value='true',
        description='是否启动仿真模式'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10',
        description='关节状态发布频率 (Hz)'
    )

    # 获取参数值
    use_simulation = LaunchConfiguration('use_simulation')
    publish_rate = LaunchConfiguration('publish_rate')

    # 1. 启动机器人描述和RViz显示
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('dexforce_description'),
                'launch',
                'description.launch.py'
            ])
        ]),
        launch_arguments={
            'use_simulation': use_simulation  # 传递仿真模式参数
        }.items()
    )
    
    # 2. 关节转换器
    joint_converter_node = Node(
        package='joint_converter',
        executable='joint_converter',
        name='joint_converter',
        parameters=[{
            'use_simulation': use_simulation,
            'publish_rate': publish_rate
        }],
        output='screen',
        emulate_tty=True
    )

    
    return LaunchDescription([
        # 参数声明
        use_simulation_arg,
        publish_rate_arg,
        # 启动节点
        robot_description_launch,
        joint_converter_node,
    ])