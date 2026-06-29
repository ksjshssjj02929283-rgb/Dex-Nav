import os
import re
from typing import Final, List

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Packages
DESCRIPTION_PKG: Final[str] = "swerve_robot_description"

# Directories
URDF_DIR: Final[str] = "urdf"

# Files
ROBOT_XACRO_FILE: Final[str] = "swerve_robot.xacro"

# File Paths
description_path = os.path.join(
    get_package_share_directory(DESCRIPTION_PKG), URDF_DIR, ROBOT_XACRO_FILE
)

# Launch Arguments
ARGUMENTS: Final[List[DeclareLaunchArgument]] = [
    DeclareLaunchArgument(
        name="sim",
        default_value="false",
        choices=["true", "false"],
        description="Enable simulation control and sensor plugins",
    ),
    DeclareLaunchArgument(
        name="steering_limit_deg",
        default_value="90.0",
        description="Max steering angle of each wheel (left/right), in degrees",
    ),
    DeclareLaunchArgument(
        name="upper_body",
        default_value="true",
        choices=["true", "false"],
        description="Attach the DexForce upper body (visual + collision)",
    ),
]


def _build_robot_description(context) -> str:
    """Process the xacro into a URDF string and strip XML comments.

    The XML comments must be removed because gazebo_ros2_control (0.4.10) forwards
    the whole robot_description to the controller_manager as a CLI argument
    `--param robot_description:=<urdf>`, which rcl parses as YAML. Sequences inside
    XML comments break that YAML parse, so rcl_parse_arguments fails and the plugin
    returns *before* creating the controller_manager -> no controllers, no
    /joint_states, no odometry. Comments are non-functional, so dropping them keeps
    the model identical while letting the controller_manager start.
    """
    mappings = {
        "sim": LaunchConfiguration("sim").perform(context),
        "steering_limit_deg": LaunchConfiguration("steering_limit_deg").perform(context),
        "upper_body": LaunchConfiguration("upper_body").perform(context),
    }
    doc = xacro.process_file(description_path, mappings=mappings)
    urdf = doc.toxml()
    urdf = re.sub(r"<!--.*?-->", "", urdf, flags=re.DOTALL)
    return urdf


def launch_setup(context, *args, **kwargs) -> List[Node]:
    robot_description = {"robot_description": _build_robot_description(context)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    return [robot_state_publisher_node]


def generate_launch_description() -> LaunchDescription:
    """
    1. Parse robot xacro file (comments stripped for controller_manager safety)
    2. Launch robot state publisher for /robot_description and /tf

    Launch Args:
    sim: false(default)
        - Loads gazebo control and sensor plugins
    """
    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=launch_setup)])
