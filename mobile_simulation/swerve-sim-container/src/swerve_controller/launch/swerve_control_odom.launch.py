import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("swerve_controller"),
        "config",
        "robot_geometry.yaml",
    )

    node_swerve_commander = Node(
        package="swerve_controller",
        name="swerve_commander",
        executable="wheel_cmd_publisher",
        parameters=[config],
    )

    node_swerve_odom = Node(
        package="swerve_controller",
        name="swerve_odometer",
        executable="wheel_odometry",
        parameters=[config],
    )

    # Root the whole TF tree at the global "map" frame. The odometer publishes
    # odom -> base_link, and base_link -> (all links) comes from
    # robot_state_publisher, so a static map -> odom makes every frame resolve
    # under map. (In a real setup a localizer like AMCL would own map -> odom;
    # here we use a static identity so the sim has a single global frame.)
    node_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_static_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    ld.add_action(node_swerve_commander)
    ld.add_action(node_swerve_odom)
    ld.add_action(node_map_to_odom)
    return ld
