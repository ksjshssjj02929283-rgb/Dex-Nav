import glob
import os
from typing import Final, List, Optional

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

# ROS Pkg Share
WORLD_PKG: Final[str] = "aws_robomaker_small_warehouse_world"
SWERVE_ROBOT_GAZEBO_PKG: Final[str] = "swerve_robot_gazebo"
GAZEBO_ROS_PKG: Final[str] = "gazebo_ros"

# File Names
SPAWN_LAUNCH_FILE: Final[str] = "spawn_swerve_robot.launch.py"
GZSERVER_LAUNCH_FILE: Final[str] = "gzserver.launch.py"
GZCLIENT_LAUNCH_FILE: Final[str] = "gzclient.launch.py"

# Directory Names
LAUNCH_DIR: Final[str] = "launch"

# Relative path (within the world package share) of the world file to load.
WORLD_FILE_REL: Final[str] = os.path.join(
    "worlds", "no_roof_small_warehouse", "no_roof_small_warehouse.world"
)

# File Paths
world_pkg_share = get_package_share_directory(WORLD_PKG)
world_file_path = os.path.join(world_pkg_share, WORLD_FILE_REL)
gazebo_ros_launch_dir = os.path.join(
    get_package_share_directory(GAZEBO_ROS_PKG), LAUNCH_DIR
)
gzserver_launch_path = os.path.join(gazebo_ros_launch_dir, GZSERVER_LAUNCH_FILE)
gzclient_launch_path = os.path.join(gazebo_ros_launch_dir, GZCLIENT_LAUNCH_FILE)
spawn_launch_path = os.path.join(
    get_package_share_directory(SWERVE_ROBOT_GAZEBO_PKG), LAUNCH_DIR, SPAWN_LAUNCH_FILE
)

# Launch Arguments
ARGUMENTS: Optional[List[DeclareLaunchArgument]] = [
    DeclareLaunchArgument(
        name="render",
        default_value="auto",
        choices=["auto", "nvidia", "software"],
        description=(
            "gzclient GUI rendering backend. 'auto' uses the system default "
            "GPU; 'nvidia' renders on the NVIDIA dGPU via PRIME offload (use "
            "this on hybrid-GPU laptops if gzclient crashes with a 'Camera "
            "px != 0' assertion); 'software' forces CPU rendering (slow but "
            "most compatible)."
        ),
    ),
    DeclareLaunchArgument(
        name="gui",
        default_value="true",
        choices=["true", "false"],
        description="Start the gzclient GUI. Set false to run Gazebo headless.",
    ),
]

# Seconds to wait before starting the gzclient GUI. Starting it together with
# gzserver causes an intermittent OGRE crash ("Camera px != 0") because the GUI
# tries to read the scene/user-camera before gzserver has built it. Delaying the
# client until the server scene exists avoids the race.
GZCLIENT_DELAY_SEC: Final[float] = 8.0

# Seconds to wait after starting Gazebo before spawning the robot, so that the
# /spawn_entity service (and the gazebo_ros2_control controller_manager) is ready.
SPAWN_DELAY_SEC: Final[float] = 8.0


def _gazebo_system_resource_dir() -> Optional[str]:
    """
    Locate the Gazebo classic system media directory (e.g. /usr/share/gazebo-11).

    OGRE needs this on GAZEBO_RESOURCE_PATH to find built-in materials such as
    'Gazebo/shadow_caster'; if it is missing, gzclient aborts with a null-camera
    ("px != 0") assertion. Normally /usr/share/gazebo/setup.sh adds it, but we
    locate it here so the sim works even when that file was not sourced.
    """
    for candidate in sorted(glob.glob("/usr/share/gazebo-*"), reverse=True):
        if os.path.isdir(os.path.join(candidate, "media", "materials")):
            return candidate
    return None


def generate_launch_description() -> LaunchDescription:
    """
    Generate Launch Description for Harmony Sim.

    Returns
    -------
        LaunchDescription: Description of the launch-able ROS system

    """
    # Pin all ROS 2 / DDS traffic to the loopback interface.
    #
    # With the default (ROS_LOCALHOST_ONLY=0) FastDDS runs multicast discovery
    # over EVERY network interface. This sim brings up ~24 nodes (the gazebo_ros
    # bridge, the 9 ros2_control controllers, the controller_manager, the state
    # publishers, odom/commander, ...) and that discovery chatter saturates the
    # DDS layer and collapses gzserver's single-threaded ROS interface:
    #   * gazebo_ros2_control init goes from ~0.02 s to 7-12 s,
    #   * each controller takes ~6 s to load (and progressively worse), so the
    #     /controller_manager services eventually become unreachable, and
    #   * the sensor-bridge plugins drop from full rate to ~0 Hz even though the
    #     native Gazebo sensors keep producing data at 100/10 Hz.
    # Restricting to loopback removes the storm: controllers then load in <1 s
    # and IMU/lidar/camera all publish at full rate. Run any auxiliary tools
    # (rviz2, teleop, `ros2 topic echo`) with `export ROS_LOCALHOST_ONLY=1` too.
    localhost_only = SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1")

    # Disable Gazebo's online model database lookup. The warehouse world
    # references models that are not bundled locally (e.g. Apriltag36_11_00000);
    # without this, Gazebo blocks for a long time trying to reach the defunct
    # models.gazebosim.org server, which delays /spawn_entity past its timeout.
    disable_model_db = SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", "")

    # Build GAZEBO_RESOURCE_PATH. Two things must be on it:
    #   1. The warehouse package share dir, because its model meshes use relative
    #      URIs like "file://models/.../mesh.DAE" resolved against this path.
    #   2. The Gazebo system media dir (e.g. /usr/share/gazebo-11), because OGRE
    #      loads its built-in materials from there; without it gzclient crashes
    #      with a "Camera px != 0" assertion.
    # We always include both so the sim does not depend on the user having
    # sourced /usr/share/gazebo/setup.sh beforehand.
    resource_parts = [world_pkg_share]
    gazebo_system_dir = _gazebo_system_resource_dir()
    if gazebo_system_dir:
        resource_parts.append(gazebo_system_dir)
    existing_resource_path = os.environ.get("GAZEBO_RESOURCE_PATH", "")
    if existing_resource_path:
        resource_parts.append(existing_resource_path)
    set_resource_path = SetEnvironmentVariable(
        "GAZEBO_RESOURCE_PATH", os.pathsep.join(resource_parts)
    )

    # Optional rendering backend selection for gzclient. Gazebo classic's OGRE
    # renderer can abort with a "Camera px != 0" assertion on some Mesa/iGPU
    # drivers. These env vars must be set before gzclient is launched.
    render = LaunchConfiguration("render")
    use_nvidia = IfCondition(PythonExpression(["'", render, "' == 'nvidia'"]))
    use_software = IfCondition(PythonExpression(["'", render, "' == 'software'"]))

    nvidia_offload = SetEnvironmentVariable(
        "__NV_PRIME_RENDER_OFFLOAD", "1", condition=use_nvidia
    )
    nvidia_glx = SetEnvironmentVariable(
        "__GLX_VENDOR_LIBRARY_NAME", "nvidia", condition=use_nvidia
    )
    software_gl = SetEnvironmentVariable(
        "LIBGL_ALWAYS_SOFTWARE", "1", condition=use_software
    )

    # Gazebo server (physics, no GUI), loading the warehouse world directly.
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gzserver_launch_path]),
        launch_arguments={"world": world_file_path}.items(),
    )

    # Gazebo client (GUI), started after a delay to avoid the startup race.
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gzclient_launch_path]),
        condition=IfCondition(LaunchConfiguration("gui")),
    )
    delayed_gzclient = TimerAction(
        period=GZCLIENT_DELAY_SEC,
        actions=[gzclient],
    )

    swerve_robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([spawn_launch_path]),
    )

    # Give Gazebo time to come up before spawning the robot.
    delayed_swerve_robot_spawn = TimerAction(
        period=SPAWN_DELAY_SEC,
        actions=[swerve_robot_spawn],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(localhost_only)
    ld.add_action(disable_model_db)
    ld.add_action(set_resource_path)
    ld.add_action(nvidia_offload)
    ld.add_action(nvidia_glx)
    ld.add_action(software_gl)
    ld.add_action(gzserver)
    ld.add_action(delayed_gzclient)
    ld.add_action(delayed_swerve_robot_spawn)

    return ld
