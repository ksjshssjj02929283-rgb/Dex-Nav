"""Static regression tests for the shared W1 simulation interface contract."""

import ast
from pathlib import Path
import re


REPO = Path(__file__).resolve().parents[3]

COMMAND_TOPIC = "/control/cmd_vel"
ODOM_TOPIC = "/feedback/odom"
FRONT_SCAN_TOPIC = "/laser_scan/front"
REAR_SCAN_TOPIC = "/laser_scan/rear"
IMU_TOPIC = "/imu/chassis"
LEGACY_ODOM_TOPIC = "/odom/wheel"
LEGACY_SCAN_TOPICS = ("/scan_front", "/scan_rear")


def read(relative_path: str) -> str:
    """Read a UTF-8 repository file."""
    return (REPO / relative_path).read_text(encoding="utf-8")


def declared_launch_arguments(relative_path: str) -> set[str]:
    """Return statically declared arguments from a Python launch file."""
    tree = ast.parse(read(relative_path))
    arguments = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name):
            continue
        if node.func.id != "DeclareLaunchArgument":
            continue
        if node.args and isinstance(node.args[0], ast.Constant):
            arguments.add(node.args[0].value)
            continue
        for keyword in node.keywords:
            if (keyword.arg == "name" and
                    isinstance(keyword.value, ast.Constant)):
                arguments.add(keyword.value.value)
    return arguments


def launch_argument_defaults(relative_path: str) -> dict[str, str]:
    """Return constant defaults declared by a Python launch file."""
    tree = ast.parse(read(relative_path))
    defaults = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name):
            continue
        if node.func.id != "DeclareLaunchArgument" or not node.args:
            continue
        if not isinstance(node.args[0], ast.Constant):
            continue
        for keyword in node.keywords:
            if (keyword.arg == "default_value" and
                    isinstance(keyword.value, ast.Constant)):
                defaults[node.args[0].value] = keyword.value.value
    return defaults


def test_omni_and_diff_publish_the_production_topics():
    """Both chassis variants expose the production base and sensor topics."""
    commander = read(
        "mobile_simulation/swerve-sim-container/src/swerve_controller/"
        "swerve_controller/swerve_commander.py")
    odometer = read(
        "mobile_simulation/swerve-sim-container/src/swerve_controller/"
        "swerve_controller/swerve_odometer.py")
    omni_sensors = read(
        "mobile_simulation/swerve-sim-container/src/swerve_robot_description/"
        "urdf/include/dexforce_sensors.xacro")
    diff_urdf = read(
        "mobile_simulation/diff-sim-container/src/w1_diff_gazebo/"
        "urdf/w1_diff_sim.urdf")

    assert f'Twist, "{COMMAND_TOPIC}"' in commander
    assert f'WHEEL_ODOM_TOPIC: Final[str] = "{ODOM_TOPIC}"' in odometer
    assert LEGACY_ODOM_TOPIC not in odometer
    for topic in (COMMAND_TOPIC, ODOM_TOPIC):
        assert topic in diff_urdf
    for topic in (FRONT_SCAN_TOPIC, REAR_SCAN_TOPIC, IMU_TOPIC):
        assert topic in omni_sensors
        assert topic in diff_urdf
    for model in (omni_sensors, diff_urdf):
        for legacy_scan_topic in LEGACY_SCAN_TOPICS:
            assert legacy_scan_topic not in model


def test_rgbd_topics_and_frames_are_aligned():
    """Both models expose the same RGBD topics and sensor frame names."""
    omni_sensors = read(
        "mobile_simulation/swerve-sim-container/src/"
        "swerve_robot_description/urdf/include/dexforce_sensors.xacro")
    diff_urdf = read(
        "mobile_simulation/diff-sim-container/src/w1_diff_gazebo/"
        "urdf/w1_diff_sim.urdf")

    for model in (omni_sensors, diff_urdf):
        for side in ("front", "rear"):
            assert f"/orbbec/{side}" in model
        for remapping in (
            "realsense/image_raw:=rgb",
            "realsense/depth/image_raw:=depth",
            "realsense/camera_info:=camera_info",
            "realsense/points:=points",
        ):
            assert remapping in model
        for frame in (
            "laser_front_link", "laser_rear_link",
            "chassis_depth_front_link", "chassis_depth_rear_link",
            "imu_chassis_link",
        ):
            assert frame in model

    for side in ("front", "rear"):
        processor = read(f"dexe_sensor_process/launch/{side}_p2c.launch.py")
        assert f"/orbbec/{side}/pointcloud2_raw" in processor
        assert f"/orbbec/{side}/pointcloud2_filtered" in processor


def test_navigation_parameters_consume_the_same_topics():
    """Production and simulation Nav2 configs consume canonical topics."""
    for params in (
        "mobile_navigation/nav2_bringup/params/w1_nav2_params.yaml",
        "mobile_navigation/nav2_bringup/params/w1_nav2_rgbd_params.yaml",
        "mobile_navigation/nav2_bringup/params/w1_nav2_params_omni.yaml",
        "mobile_navigation/nav2_bringup/params/w1_nav2_params_diff_sim.yaml",
    ):
        text = read(params)
        for topic in (ODOM_TOPIC, FRONT_SCAN_TOPIC, REAR_SCAN_TOPIC):
            assert topic in text
        assert LEGACY_ODOM_TOPIC not in text
        for legacy_scan_topic in LEGACY_SCAN_TOPICS:
            assert legacy_scan_topic not in text

    for params in (
        "mobile_navigation/nav2_bringup/params/w1_nav2_params_omni.yaml",
        "mobile_navigation/nav2_bringup/params/w1_nav2_params_diff_sim.yaml",
    ):
        text = read(params)
        assert re.search(r'input_topic:\s*["\']?cmd_vel_nav', text)
        assert re.search(r'output_topic:\s*["\']?control/cmd_vel', text)
        for side in ("front", "rear"):
            assert f"/orbbec/{side}/pointcloud2_filtered" in text


def test_tf_chain_uses_base_footprint_between_odom_and_base_link():
    """Both variants implement odom to base_footprint to base_link."""
    odometer = read(
        "mobile_simulation/swerve-sim-container/src/swerve_controller/"
        "swerve_controller/swerve_odometer.py")
    omni_base = read(
        "mobile_simulation/swerve-sim-container/src/swerve_robot_description/"
        "urdf/include/swerve_robot_base.xacro")
    diff_urdf = read(
        "mobile_simulation/diff-sim-container/src/w1_diff_gazebo/"
        "urdf/w1_diff_sim.urdf")

    assert 'BASE_FRAME: Final[str] = "base_footprint"' in odometer
    for model in (omni_base, diff_urdf):
        assert '<parent link="base_footprint"' in model
        assert '<child link="base_link"' in model


def test_navigation_launch_arguments_are_aligned():
    """Both navigation entry points expose the same launch API."""
    expected = {
        "use_sim_time", "map", "params_file", "slam",
        "x_pose", "y_pose", "gui", "render",
        "map_odom_x", "map_odom_y", "map_odom_yaw",
    }
    omni = declared_launch_arguments(
        "mobile_simulation/w1_nav2/launch/omni_sim.launch.py")
    diff = declared_launch_arguments(
        "mobile_simulation/w1_nav2/launch/diff_sim.launch.py")
    assert omni == expected
    assert diff == expected


def test_navigation_launches_share_the_default_map_alignment():
    """Both chassis align the default warehouse map to odom identically."""
    expected = {
        "map_odom_x": "0.03143",
        "map_odom_y": "-0.01856",
        "map_odom_yaw": "0.02040",
    }
    for launch_file in (
        "mobile_simulation/w1_nav2/launch/omni_sim.launch.py",
        "mobile_simulation/w1_nav2/launch/diff_sim.launch.py",
    ):
        defaults = launch_argument_defaults(launch_file)
        assert {name: defaults[name] for name in expected} == expected
        launch = read(launch_file)
        for name in expected:
            assert f"'{name}': {name}" in launch


def test_all_sim_launches_share_the_same_rviz_configuration():
    """Navigation and mapping use one RViz configuration for both chassis."""
    for launch_file in (
        "mobile_simulation/w1_nav2/launch/omni_sim.launch.py",
        "mobile_simulation/w1_nav2/launch/diff_sim.launch.py",
        "mobile_simulation/w1_nav2/launch/omni_sim_mapping.launch.py",
        "mobile_simulation/w1_nav2/launch/diff_sim_mapping.launch.py",
    ):
        launch = read(launch_file)
        assert "diff_sim.rviz" in launch
        assert "omni_display.rviz" not in launch


def test_mapping_launches_leave_map_to_odom_to_slam():
    """Mapping entries expose one API and leave map-to-odom to SLAM."""
    expected = {"use_sim_time", "x_pose", "y_pose", "gui", "render"}
    for launch_file in (
        "mobile_simulation/w1_nav2/launch/omni_sim_mapping.launch.py",
        "mobile_simulation/w1_nav2/launch/diff_sim_mapping.launch.py",
    ):
        assert declared_launch_arguments(launch_file) == expected
        assert "static_transform_publisher" not in read(launch_file)

    control_launch = read(
        "mobile_simulation/swerve-sim-container/src/swerve_controller/"
        "launch/swerve_control_odom.launch.py")
    assert "static_transform_publisher" not in control_launch
    assert "map_to_odom_static_tf" not in control_launch


def test_navigation_owns_only_the_non_slam_map_to_odom_transform():
    """Nav2 owns simulation localization TF; mapping leaves it to SLAM."""
    bringup = read(
        "mobile_navigation/nav2_bringup/launch/bringup_launch.py")
    assert "map_to_odom_static_tf" in bringup
    assert ".lower() != 'true'" in bringup
    assert {"map_odom_x", "map_odom_y", "map_odom_yaw"}.issubset(
        declared_launch_arguments(
            "mobile_navigation/nav2_bringup/launch/bringup_launch.py"))
    assert "'--x', map_odom_x" in bringup
    assert "'--y', map_odom_y" in bringup
    assert "'--yaw', map_odom_yaw" in bringup
    assert "'--frame-id', 'map'" in bringup
    assert "'--child-frame-id', 'odom'" in bringup
