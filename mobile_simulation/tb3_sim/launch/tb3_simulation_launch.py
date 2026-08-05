# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This launch script launches a light-weight fake simulator or Gazebo, along with the Nav2 stack for turtlebot3_waffle."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the share directories
    tb3_sim_dir = get_package_share_directory('tb3_sim')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Paths to simulation assets inside nav2_bringup and tb3_sim
    urdf_file = os.path.join(tb3_sim_dir, 'urdf', 'w1_diff_sim.urdf')
    world_file = os.path.join(tb3_sim_dir, 'config', 'world_only.model')
    robot_sdf_file = os.path.join(tb3_sim_dir, 'config', 'waffle.model')
    
    # Paths to simulation configs inside our new tb3_sim package
    map_yaml_file = os.path.join(tb3_sim_dir, 'maps', 'turtlebot3_world.yaml')
    params_file = os.path.join(tb3_sim_dir, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(tb3_sim_dir, 'rviz', 'nav2_default_view.rviz')

    # Set Gazebo model path and resource path so Gazebo can resolve package://tb3_sim/... meshes and local models
    package_share_parent = os.path.dirname(tb3_sim_dir)
    local_models_path = os.path.join(tb3_sim_dir, 'models')

    model_path_elements = [local_models_path, package_share_parent]
    if 'GAZEBO_MODEL_PATH' in os.environ and os.environ['GAZEBO_MODEL_PATH']:
        model_path_elements.append(os.environ['GAZEBO_MODEL_PATH'])
    model_path = ':'.join(model_path_elements)

    resource_path_elements = [package_share_parent]
    if 'GAZEBO_RESOURCE_PATH' in os.environ and os.environ['GAZEBO_RESOURCE_PATH']:
        resource_path_elements.append(os.environ['GAZEBO_RESOURCE_PATH'])
    resource_path = ':'.join(resource_path_elements)

    # Set GAZEBO_MODEL_PATH and GAZEBO_RESOURCE_PATH environment variables
    set_gazebo_model_path_cmd = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', model_path
    )
    set_gazebo_resource_path_cmd = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH', resource_path
    )

    # Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    # Launch configuration variables specific to simulation
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')

    pose = {'x': LaunchConfiguration('x_pose', default='-2.00'),
            'y': LaunchConfiguration('y_pose', default='-0.50'),
            'z': LaunchConfiguration('z_pose', default='0.20'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}
    robot_name = LaunchConfiguration('robot_name')

    # Map static TF names
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_yaml_file,
        description='Full path to map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=rviz_config_file,
        description='Full path to the RVIZ config file to use')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start Gazebo simulation. If false, starts a lightweight dummy_robot simulator node.')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient (set to False to see gazebo GUI)')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='w1_diff',
        description='name of the robot')

    # ========================== Gazebo Simulation Nodes (Conditional) ==========================
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression(
            ['"', use_simulator, '".lower() == "true" and not "', headless, '".lower() == "true"'])),
        cmd=['gzclient'],
        output='screen')

    # 构造 spawn_entity.py 参数，防止 namespace 为空时错把 -x 抢占为命名空间
    spawn_args = [
        '-entity', robot_name,
        '-file', urdf_file,
        '-spawn_service_timeout', '60.0',
        '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
        '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']
    ]

    start_gazebo_spawner_cmd = Node(
        condition=IfCondition(use_simulator),
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=spawn_args)

    # ========================== Lightweight Dummy Robot Simulator (Conditional) ==========================
    # When Gazebo is disabled, launch dummy_robot script and map->odom static TF publisher
    start_dummy_robot_cmd = Node(
        condition=UnlessCondition(use_simulator),
        package='tb3_sim',
        executable='dummy_robot.py',
        name='dummy_robot',
        output='screen',
        parameters=[{
            'x': -2.0,
            'y': -0.5,
            'yaw': 0.0
        }])

    start_map_odom_tf_cmd = Node(
        condition=UnlessCondition(use_simulator),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'])

    start_fake_amcl_cmd = Node(
        condition=IfCondition(use_simulator),
        package='tb3_sim',
        executable='fake_amcl.py',
        name='fake_amcl',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    # ========================== Common Nodes (Robot state, RViz, Nav2) ==========================
    # Robot State Publisher Node
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}],
        remappings=remappings)

    # RViz Launch Action
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'use_sim_time': use_sim_time,
                          'rviz_config': rviz_config_file}.items())

    # Nav2 Navigation Stack Bringup Launch Action (delayed 3s to let Gazebo initialize first)
    bringup_cmd = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
                launch_arguments={'namespace': namespace,
                                  'use_namespace': use_namespace,
                                  'slam': slam,
                                  'map': map_yaml_file,
                                  'use_sim_time': use_sim_time,
                                  'params_file': params_file,
                                  'autostart': autostart,
                                  'use_composition': use_composition,
                                  'use_respawn': use_respawn}.items())
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add environment variable setup first
    ld.add_action(set_gazebo_model_path_cmd)
    ld.add_action(set_gazebo_resource_path_cmd)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_robot_name_cmd)

    # Add any Gazebo conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_gazebo_spawner_cmd)

    # Add Dummy robot simulator actions (when use_simulator is false)
    ld.add_action(start_dummy_robot_cmd)
    ld.add_action(start_map_odom_tf_cmd)
    ld.add_action(start_fake_amcl_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)

    return ld
