import os
from typing import Final, List, Optional

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Packages
TELEOP_JOY_PKG: Final[str] = "teleop_twist_joy"
SWERVE_ROBOT_GAZEBO_PKG: Final[str] = "swerve_robot_gazebo"

# Directories
LAUNCH_DIR: Final[str] = "launch"
CONFIG_DIR: Final[str] = "config"

# Files
TELEOP_JOY_LAUNCH: Final[str] = "teleop-launch.py"
TELEOP_JOY_CONFIG: Final[str] = "xbox_holonomic.config.yaml"

# File Paths
teleop_joy_launch_path = os.path.join(
    get_package_share_directory(TELEOP_JOY_PKG), LAUNCH_DIR, TELEOP_JOY_LAUNCH
)
teleop_joy_config_path = os.path.join(
    get_package_share_directory(SWERVE_ROBOT_GAZEBO_PKG), CONFIG_DIR, TELEOP_JOY_CONFIG
)

# Launch Arguments
ARGUMENTS: Optional[List[DeclareLaunchArgument]] = []


def generate_launch_description() -> LaunchDescription:
    """
    Gamepad teleop for the swerve base.

    Starts joy + teleop_twist_joy with the holonomic Xbox mapping and publishes
    geometry_msgs/Twist on /cmd_vel, which swerve_commander consumes. Mapping
    (see config/xbox_holonomic.config.yaml): hold LB (deadman), left stick =
    translate (up/down vx, left/right vy strafe), right stick horizontal = yaw,
    hold RB for turbo.
    """
    # Must match the simulation, which pins DDS to loopback (see sim.launch.py).
    # Without this the joy/teleop nodes run with the default ROS_LOCALHOST_ONLY=0
    # and either fail to discover /cmd_vel's subscriber or re-introduce the
    # multicast discovery storm that starves gzserver.
    localhost_only = SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1")

    teleop_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([teleop_joy_launch_path]),
        launch_arguments={
            "config_filepath": teleop_joy_config_path,
        }.items(),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(localhost_only)
    ld.add_action(teleop_joy)

    return ld
