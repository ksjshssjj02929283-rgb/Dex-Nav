"""Static regression tests for the shared W1 simulation interface contract."""

import ast
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[3]

COMMAND_TOPIC = "/control/cmd_vel"
NAV_COMMAND_TOPIC = "/control/cmd_vel_nav"
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
        "mobile_navigation/nav2_bringup/params/w1_nav2_params_diff.yaml",
        "mobile_navigation/nav2_bringup/params/w1_nav2_rgbd_params.yaml",
        "mobile_navigation/nav2_bringup/params/w1_nav2_params_omni.yaml",
    ):
        text = read(params)
        for topic in (ODOM_TOPIC, FRONT_SCAN_TOPIC, REAR_SCAN_TOPIC):
            assert topic in text
        assert LEGACY_ODOM_TOPIC not in text
        for legacy_scan_topic in LEGACY_SCAN_TOPICS:
            assert legacy_scan_topic not in text

    for params in (
        "mobile_navigation/nav2_bringup/params/w1_nav2_params_omni.yaml",
    ):
        text = read(params)
        assert re.search(r'input_topic:\s*["\']?cmd_vel_nav', text)
        assert re.search(r'output_topic:\s*["\']?control/cmd_vel', text)
        for side in ("front", "rear"):
            assert f"/orbbec/{side}/pointcloud2_filtered" in text


def test_arm_build_keeps_w1_nav2_for_real_robot_launch_transition():
    """0.4.6 ARM builds retain w1_nav2 until the 0.5.0 package split."""
    manifest = read("mobile_simulation/w1_nav2/package.xml")
    for dependency in (
        "aws_robomaker_small_warehouse_world",
        "swerve_robot_gazebo",
        "w1_diff_gazebo",
    ):
        assert f"<exec_depend>{dependency}</exec_depend>" not in manifest

    assert (REPO / "mobile_simulation/w1_nav2/launch/"
            "w1_nav2_actual.launch.py").is_file()

    build_script = read("build.sh")
    assert 'BASH_SOURCE[0]' in build_script
    assert 'SOURCE_ROOT="${SCRIPT_DIR}"' in build_script
    assert 'SIMULATION_DIR="${SOURCE_ROOT}/mobile_simulation"' in build_script
    assert '[ "$package_name" != "w1_nav2" ]' in build_script
    assert ('run_colcon_build --packages-select w1_nav2 '
            '"${COLCON_BUILD_ARGS[@]}"') in build_script
    assert build_script.count(
        'run_colcon_build --packages-select w1_nav2') == 1
    assert 'FINAL_SKIP_PACKAGES+=("w1_nav2")' in build_script
    assert ('run_colcon_build '
            '"${FINAL_COLCON_BUILD_ARGS[@]}"') in build_script
    direct_colcon_builds = [
        line.strip()
        for line in build_script.splitlines()
        if line.strip().startswith("colcon build")
    ]
    assert direct_colcon_builds == ['colcon build "$@"']
    assert "printf ' %q' colcon build \"$@\"" in build_script
    for future_contract in ("0.5.0", "nav2_bringup", "w1_sim"):
        assert future_contract in build_script


def test_diff_sim_bridges_production_navigation_commands_to_gazebo():
    """Diff simulation adapts the production Nav2 output to its chassis API."""
    production_params = read(
        "mobile_navigation/nav2_bringup/params/w1_nav2_params_diff.yaml")
    bridge = read(
        "mobile_simulation/diff-sim-container/src/w1_diff_gazebo/"
        "src/sim_cmd_vel_bridge.cpp")
    spawn = read(
        "mobile_simulation/diff-sim-container/src/w1_diff_gazebo/"
        "launch/spawn_w1_diff.launch.py")

    assert re.search(
        r'output_topic:\s*["\']?control/cmd_vel_nav', production_params)
    for topic in (NAV_COMMAND_TOPIC, COMMAND_TOPIC):
        assert topic in bridge
        assert topic in spawn
    assert "sim_cmd_vel_bridge" in spawn


def test_diff_chassis_supports_are_coplanar_and_pitch_stable():
    """Drive wheels and two centerline skids form one ground plane."""
    root = ET.fromstring(read(
        "mobile_simulation/diff-sim-container/src/w1_diff_gazebo/"
        "urdf/w1_diff_sim.urdf"))
    chassis = root.find("./link[@name='chassis_base_link']")
    assert chassis is not None

    skids = []
    for collision in chassis.findall("collision"):
        sphere = collision.find("./geometry/sphere")
        if sphere is None:
            continue
        x, y, z = (
            float(value)
            for value in collision.find("origin").get("xyz").split()
        )
        skids.append((x, y, z, float(sphere.get("radius"))))

    assert len(skids) == 2
    assert {(x, y) for x, y, _, _ in skids} == {(0.28, 0.0), (-0.28, 0.0)}

    wheel_joint = root.find("./joint[@name='WHEEL_LEFT']/origin")
    wheel_collision = root.find(
        "./link[@name='wheel_left']/collision/geometry/cylinder")
    assert wheel_joint is not None
    assert wheel_collision is not None
    wheel_center_z = float(wheel_joint.get("xyz").split()[2])
    wheel_ground_z = wheel_center_z - float(wheel_collision.get("radius"))

    for _, _, center_z, radius in skids:
        assert math.isclose(center_z - radius, wheel_ground_z, abs_tol=1e-9)


def test_missing_route_graph_uses_smac_fallback():
    """An unavailable route-graph service requests the configured fallback."""
    production_params = read(
        "mobile_navigation/nav2_bringup/params/w1_nav2_params_diff.yaml")
    route_planner = read(
        "mobile_navigation/nav2_route_planner/src/"
        "nav_route_graph_planner.cpp")
    planner_server = read(
        "mobile_navigation/nav2_planner/src/planner_server.cpp")
    loader = read(
        "mobile_navigation/nav2_route_planner/src/route_graph_loader.cpp")

    assert 'route_fallback_planner: "SmacGridBased"' in production_params
    assert "last_fallback_reason_ = FallbackReason::NO_ROUTE_GRAPH" in (
        route_planner)
    assert "planners_[route_fallback_planner_]->createPlan" in planner_server
    assert "without blocking planning requests" in loader


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
    """Navigation entries share common arguments; diff adds an RGBD switch."""
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
    assert diff == expected | {"use_rgbd"}


def test_diff_sim_can_disable_rgbd_at_source():
    """use_rgbd=false removes Gazebo sensors and point-cloud converters."""
    launch = read("mobile_simulation/w1_nav2/launch/diff_sim.launch.py")
    spawn_launch = read(
        "mobile_simulation/diff-sim-container/src/w1_diff_gazebo/"
        "launch/spawn_w1_diff.launch.py")
    no_rgbd_urdf = read(
        "mobile_simulation/diff-sim-container/src/w1_diff_gazebo/"
        "urdf/w1_diff_sim_no_rgbd.urdf")

    assert "condition=IfCondition(use_rgbd)" in launch
    assert "'use_rgbd': use_rgbd" in launch
    assert "UnlessCondition(use_rgbd)" in spawn_launch
    assert "w1_diff_sim_no_rgbd.urdf" in spawn_launch
    assert 'sensor name="realsense_front"' not in no_rgbd_urdf
    assert 'sensor name="realsense_rear"' not in no_rgbd_urdf
    assert 'sensor name="laser_front"' in no_rgbd_urdf
    assert 'sensor name="laser_rear"' in no_rgbd_urdf
    assert 'sensor name="w1_imu"' in no_rgbd_urdf


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


def test_sim_navigation_goal_interfaces_match_production_rviz():
    """Simulation exposes both goal tools; diff simulation uses production params."""
    rviz = read("mobile_simulation/w1_nav2/rviz/diff_sim.rviz")
    assert rviz.count("nav2_rviz_plugins/GoalTool") == 1
    assert rviz.count("rviz_default_plugins/SetGoal") == 1
    assert re.search(
        r"rviz_default_plugins/SetGoal\s+Topic:.*?Value:\s+/?goal_pose",
        rviz,
        re.DOTALL,
    )

    omni_params = read(
        "mobile_navigation/nav2_bringup/params/w1_nav2_params_omni.yaml")
    assert 'current_goal_checker: "goal_checker"' in omni_params
    assert 'current_controller: "FollowPath"' in omni_params
    assert re.search(r"staged_control:\s+enabled:\s+false", omni_params)
    assert 'plugin: "nav2_simple_goal_checker::SimpleGoalChecker"' in omni_params
    assert 'plugin: "nav2_controller::SimpleGoalChecker"' not in omni_params

    production_params = read(
        "mobile_navigation/nav2_bringup/params/w1_nav2_params_diff.yaml")
    assert 'current_goal_checker: "high_precision_goal_checker"' in production_params
    assert 'current_controller: "RPPController"' in production_params
    assert 'speed_planner_plugins: ["TrapezoidalSpeedPlanner", "SCurveSpeedPlanner"]' in (
        production_params)
    assert 'plugin: "nav2_simple_goal_checker::SimpleGoalChecker"' in production_params

    diff_launch = read("mobile_simulation/w1_nav2/launch/diff_sim.launch.py")
    assert "'w1_nav2_params_diff.yaml'" in diff_launch
    assert "w1_nav2_params_diff_sim.yaml" not in diff_launch
    assert not (
        REPO / "mobile_navigation/nav2_bringup/params/"
        "w1_nav2_params_diff_sim.yaml").exists()


def test_diff_sim_delays_depth_camera_spawn_until_gazebo_scene_is_ready():
    """带深度相机的机器人延迟生成，Nav2等待里程计和TF就绪后再启动。"""
    diff_launch = read("mobile_simulation/w1_nav2/launch/diff_sim.launch.py")

    assert "SetEnvironmentVariable, TimerAction" in diff_launch
    assert "'GAZEBO_RESOURCE_PATH'" in diff_launch
    assert re.search(
        r"delayed_spawn_w1_diff_cmd\s*=\s*TimerAction\(\s*"
        r"period=8\.0,\s*actions=\[spawn_w1_diff_cmd\]\)",
        diff_launch,
    )
    assert re.search(
        r"delayed_nav2_bringup_cmd\s*=\s*TimerAction\(\s*"
        r"period=15\.0,\s*actions=\[nav2_bringup_cmd\]\)",
        diff_launch,
    )
    assert "ld.add_action(gzserver_cmd)" in diff_launch
    assert "ld.add_action(delayed_spawn_w1_diff_cmd)" in diff_launch
    assert "ld.add_action(delayed_nav2_bringup_cmd)" in diff_launch
    assert "ld.add_action(spawn_w1_diff_cmd)" not in diff_launch
    assert "ld.add_action(nav2_bringup_cmd)" not in diff_launch


def test_speed_planner_plugin_is_not_linked_into_controller_server():
    """The host exports state support only; pluginlib exclusively loads factories."""
    speed_cmake = read(
        "mobile_navigation/nav2_speed_planner/CMakeLists.txt")
    assert "set(support_library_name nav2_speed_plan_state)" in speed_cmake
    assert re.search(
        r"add_library\(\$\{support_library_name\}\s+SHARED\s+"
        r"src/speed_plan_state\.cpp\)",
        speed_cmake,
    )
    assert re.search(
        r"add_library\(\$\{library_name\}\s+SHARED\s+"
        r"src/speed_planner_base\.cpp\s+"
        r"src/trapezoidal_speed_planner\.cpp\s+"
        r"src/scurve_speed_planner\.cpp\)",
        speed_cmake,
    )
    assert "ament_export_libraries(${support_library_name})" in speed_cmake
    assert "ament_export_libraries(${library_name})" not in speed_cmake


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
