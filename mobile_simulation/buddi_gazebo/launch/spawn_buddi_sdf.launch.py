from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
import pathlib

def generate_launch_description():

    # Get path to URDF
    urdf_dir = get_package_share_directory('buddi_gazebo')
    # v1.0 : models/v1.0/buddi_devel_v2.sdf
    # urdf_path = os.path.join(urdf_dir, 'models', 'v1.1', 'buddi_v1_1_full.sdf')
    urdf_path = os.path.join(urdf_dir, 'models', 'v1.0', 'buddi_devel_v2.sdf')
    with open(urdf_path, 'r') as infp:
        urdf_file = infp.read()
        
    robot_description = {'robot_description' : urdf_file}

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')
    
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen')
        
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'buddi', 
                   '-file', urdf_path,
                #    '-robot_namespace','/buddi',
                   '-x', x_pose,
                   '-y', y_pose,
                   '-z', '0.01',
                   '-Y', '0.0'],
        output='screen')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description])

    node_joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='both',
        parameters=[robot_description])

    return LaunchDescription([
        declare_x_position_cmd,
        declare_y_position_cmd,
        # gazebo,
        spawn_entity,
        # node_robot_state_publisher,
        # node_joint_state_publisher
    ])