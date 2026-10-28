"""
Launch file for W1 pure ROS 2 kinematics simulation with Nav2 ControllerServer.

Components:
  1. w1_diff_kinematic_emulator : 100Hz kinematics emulation based on w1_diff_sim.urdf
  2. robot_state_publisher      : Broadcasts TF tree from w1_diff_sim.urdf
  3. controller_server          : Nav2 controller server with LocalPlanner FSM and RPP
  4. lifecycle_manager          : Manages controller_server lifecycle
  5. rosbridge_websocket        : WebSocket gateway for Web frontend simulation (ws://localhost:9090)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    w1_diff_dir = get_package_share_directory('w1_diff_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    urdf_path = os.path.join(w1_diff_dir, 'urdf', 'w1_diff_sim.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    default_params_file = os.path.join(
        nav2_bringup_dir, 'params', 'w1_nav2_params_diff.yaml')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file for Nav2 nodes')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_enable_rosbridge = DeclareLaunchArgument(
        'enable_rosbridge',
        default_value='true',
        description='Launch rosbridge_websocket on port 9090')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    enable_rosbridge = LaunchConfiguration('enable_rosbridge')

    # 1. 严格基于 w1_diff_sim.urdf 的 100Hz 差速运动学递推节点
    kinematic_emulator_node = Node(
        package='w1_diff_gazebo',
        executable='w1_diff_kinematic_emulator',
        name='w1_diff_kinematic_emulator',
        output='screen',
        parameters=[{
            'wheel_separation': 0.5078,
            'wheel_diameter': 0.126,
            'max_wheel_acceleration': 1.0,
            'max_wheel_torque': 30.0,
            'chassis_length': 0.70,
            'chassis_width': 0.50,
            'update_rate_hz': 100.0,
            'joint_state_rate_hz': 30.0,
            'use_sim_time': use_sim_time,
            'publish_map_odom_tf': True
        }]
    )

    # 2. 机器人 TF 树广播
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # 3. 官方 Nav2 ControllerServer (加载 PathPruningLocalPlanner, FrenetLocalPlanner, LexLocalPlanner, SpeedPlanner, RPP)
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel', '/control/cmd_vel_nav')
        ]
    )

    # 4. Lifecycle Manager 自动激活 controller_server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_pure_sim',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['controller_server']
        }]
    )

    # 5. rosbridge WebSocket 服务 (供前端 HTML 仿真交互)
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090}],
        condition=IfCondition(enable_rosbridge)
    )

    # 6. Web 路径与 FollowPath Action 桥接节点 (自动将 /route_planner/global_path 接入 controller_server)
    action_bridge_node = ExecuteProcess(
        cmd=['/usr/bin/python3', '/home/dex/Desktop/work/kuawei/dexe_mobile_nav/tools/route_planner_action_bridge.py'],
        output='screen'
    )

    return LaunchDescription([
        declare_params_file,
        declare_use_sim_time,
        declare_autostart,
        declare_enable_rosbridge,
        kinematic_emulator_node,
        robot_state_publisher_node,
        controller_server_node,
        lifecycle_manager_node,
        rosbridge_node,
        action_bridge_node
    ])
