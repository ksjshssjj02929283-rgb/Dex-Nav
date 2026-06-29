import os
from typing import Final, List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Packages
SWERVE_ROBOT_GAZEBO_PKG: Final[str] = "swerve_robot_gazebo"
SWERVE_ROBOT_DESCRIPTION_PKG: Final[str] = "swerve_robot_description"
SWERVE_CONTROLLER_PKG: Final[str] = "swerve_controller"

# File Names
DESCRIPTON_LAUNCH_FILE: Final[str] = "description.launch.py"
CONTROL_LAUNCH_FILE: Final[str] = "swerve_control_odom.launch.py"

# Directory Names
LAUNCH_DIR: Final[str] = "launch"

# File Paths
description_launch_path = os.path.join(
    get_package_share_directory(SWERVE_ROBOT_DESCRIPTION_PKG),
    LAUNCH_DIR,
    DESCRIPTON_LAUNCH_FILE,
)
swerve_controller_launch_path = os.path.join(
    get_package_share_directory(SWERVE_CONTROLLER_PKG), LAUNCH_DIR, CONTROL_LAUNCH_FILE
)

# Launch Arguments
ARGUMENTS: Final[List[DeclareLaunchArgument]] = []


def generate_launch_description() -> LaunchDescription:
    """
    1. Launch description launch file
    2. Load all joint state controller and 8 Gazebo joint controllers
    3. Launch control and odom nodes

    Returns:
        LaunchDescription
    """

    # Include description launch file
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([description_launch_path]),
        launch_arguments={"sim": "true"}.items(),
    )

    # Spawn robot
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "swerve_robot",
            "-z",
            "0.35",
            "-timeout",
            "120",
        ],
        output="screen",
    )

    # Load + activate every controller with a SINGLE controller_manager
    # spawner.
    #
    # Why not the previous chain of `ros2 control load_controller` processes:
    # each of those has a hard-coded 10 s service timeout with 3 retries, and on
    # timeout the process *dies* and the next one starts. With this heavy model
    # the gazebo-embedded controller_manager needs well over 10 s to answer, so
    # every controller would time out, die and respawn (x3 retries x9
    # controllers). Each of those short-lived CLI nodes is a fresh DDS
    # participant, so the constant join/leave churn floods gzserver's discovery
    # and progressively starves the (single-threaded) gazebo update loop -- the
    # controller loads then get slower and slower until /controller_manager
    # becomes unreachable and the sensor bridge plugins stop publishing.
    #
    # The spawner is ONE long-lived node (one DDS participant, no churn) that
    # waits patiently for the controller_manager (--controller-manager-timeout)
    # and loads/activates all controllers sequentially. joint_state_broadcaster
    # is listed first so joint feedback is up before the motion controllers.
    load_controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "steering_front_right",
            "steering_front_left",
            "steering_rear_left",
            "steering_rear_right",
            "wheel_front_right",
            "wheel_front_left",
            "wheel_rear_left",
            "wheel_rear_right",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )

    eh_load_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_controllers],
        )
    )

    swerve_control_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([swerve_controller_launch_path]),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(description_launch)
    ld.add_action(spawn_entity)
    ld.add_action(eh_load_controllers)
    ld.add_action(swerve_control_odom_launch)

    return ld
