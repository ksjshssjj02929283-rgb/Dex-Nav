from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os
import pathlib

def generate_launch_description():

    # Get path to URDF
    urdf_dir = get_package_share_directory('buddi_gazebo')
    # urdf_path = os.path.join(urdf_dir, 'urdf', 'v1.1', 'buddi_v1_1_full.urdf')
    urdf_path = os.path.join(urdf_dir, 'urdf', 'v1.0', 'buddi_v1_0.urdf')
    with open(urdf_path, 'r') as infp:
        urdf_file = infp.read()
        
    robot_description = {'robot_description' : urdf_file}

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen')
        
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'buddi', '-file', urdf_path],
        output='screen')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description,  {'use_sim_time': True}])

    node_joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='both',
        parameters=[robot_description, {'use_sim_time': True}])

    return LaunchDescription([
        # gazebo,
        # spawn_entity,
        node_robot_state_publisher,
        node_joint_state_publisher
    ])

