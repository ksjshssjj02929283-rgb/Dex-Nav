"""Static regression tests for the shared W1 simulation interface contract."""

import ast
import importlib.util
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


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


def read_yaml(relative_path: str) -> dict:
    """Read a repository YAML file."""
    return yaml.safe_load(read(relative_path))


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


def test_diff_sim_visualizes_planned_and_actual_paths_separately():
    """蓝色 /plan 与红色实际 Odom 轨迹必须使用不同 Topic。"""
    launch = read("mobile_simulation/w1_nav2/launch/diff_sim.launch.py")
    rviz = read("mobile_simulation/w1_nav2/rviz/diff_sim.rviz")
    visualizer = read(
        "mobile_simulation/w1_nav2/script/path_trace_visualizer.py")

    assert "path_trace_visualizer.py" in launch
    assert "Value: /plan" in rviz
    assert "Color: 0; 80; 255" in rviz
    assert "Value: /navigation/actual_path" in rviz
    assert "Color: 255; 0; 0" in rviz
    assert "PLAN_TOPIC = '/plan'" in visualizer
    assert "ODOM_TOPIC = '/feedback/odom'" in visualizer
    assert "self.actual_path = Path()" in visualizer


def test_diff_sim_uses_process_isolation_for_clean_shutdown():
    """Humble 仿真固定禁用组合容器，避免 ROSAdapter 重复关闭和容器超时。"""
    launch = read("mobile_simulation/w1_nav2/launch/diff_sim.launch.py")

    assert "use_composition = 'False'" in launch
    assert "'use_composition': use_composition" in launch


def test_diff_sim_formats_slam_for_downstream_python_expression():
    """slam 必须传 Python 布尔字面量，避免 `not false` 触发 NameError。"""
    launch_path = REPO / "mobile_simulation/w1_nav2/launch/diff_sim.launch.py"
    spec = importlib.util.spec_from_file_location(
        "diff_sim_boolean_format_test", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    assert module._python_bool_text(False) == "False"
    assert module._python_bool_text(True) == "True"
    launch = launch_path.read_text(encoding="utf-8")
    assert "'slam': _python_bool_text(config['slam'])" in launch


def test_sim_ready_waiter_rejects_invalid_odom():
    """就绪检测拒绝无效 Odom，并按传感器开关生成话题集合。"""
    script = REPO / "mobile_simulation/w1_nav2/script/wait_for_sim_ready.py"
    spec = importlib.util.spec_from_file_location(
        "wait_for_sim_ready_test", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    odom = module.Odometry()
    odom.pose.pose.orientation.w = 1.0
    odom.pose.pose.position.x = math.nan
    assert not module.SimReadyWaiter.odom_is_valid(odom)

    odom.pose.pose.position.x = 0.0
    assert module.SimReadyWaiter.odom_is_valid(odom)

    disabled = {name: False for name in module.SENSOR_TOPICS}
    assert set(module.required_topics(disabled)) == {module.ODOM_TOPIC}
    disabled["rgbd_front"] = True
    assert set(module.required_topics(disabled)) == {
        module.ODOM_TOPIC,
        "/orbbec/front/rgb",
        "/orbbec/front/depth",
        "/orbbec/front/camera_info",
    }
    assert module.required_output_topics(disabled) == {
        "/orbbec/front/laser_scan",
    }
    all_enabled = {name: True for name in module.SENSOR_TOPICS}
    assert len(module.required_topics(all_enabled)) == 10
    assert len(module.required_output_topics(all_enabled)) == 2


def test_sim_ready_waiter_does_not_override_rclpy_node_properties():
    """订阅句柄不能写入 Humble Node 的 subscriptions 只读属性。"""
    script = read("mobile_simulation/w1_nav2/script/wait_for_sim_ready.py")

    assert "self.subscriptions =" not in script
    assert "self._topic_subscriptions = []" in script


def test_gazebo_ready_waiter_requires_service_and_continuous_clock():
    """Gazebo 门控必须同时满足服务、连续时钟和0.75秒稳定期。"""
    script = REPO / "mobile_simulation/w1_nav2/script/wait_for_gazebo_ready.py"
    spec = importlib.util.spec_from_file_location(
        "wait_for_gazebo_ready_test", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    state = module.GazeboReadinessState()
    state.update_clock(0, 0.0)
    assert not state.is_ready(service_ready=True, wall_time=0.0)
    state.update_clock(1, 0.1)
    assert not state.is_ready(service_ready=False, wall_time=0.1)
    assert not state.is_ready(service_ready=True, wall_time=0.1)
    state.update_clock(2, 0.3)
    assert not state.is_ready(service_ready=True, wall_time=0.3)
    state.update_clock(3, 0.5)
    assert not state.is_ready(service_ready=True, wall_time=0.5)
    state.update_clock(4, 0.7)
    assert not state.is_ready(service_ready=True, wall_time=0.7)
    state.update_clock(5, 0.9)
    assert state.is_ready(service_ready=True, wall_time=0.9)
    assert not state.is_ready(service_ready=False, wall_time=0.9)

    # 持续收到相同时间戳不等于时钟持续推进。
    stalled = module.GazeboReadinessState()
    stalled.update_clock(0, 0.0)
    stalled.update_clock(1, 0.1)
    assert not stalled.is_ready(service_ready=True, wall_time=0.1)
    stalled.update_clock(1, 0.5)
    assert not stalled.is_ready(service_ready=True, wall_time=0.5)
    stalled.update_clock(1, 0.9)
    assert not stalled.is_ready(service_ready=True, wall_time=0.9)

    # 时间回退后必须重新积累连续推进和稳定时间。
    state.update_clock(0, 1.0)
    assert not state.is_ready(service_ready=True, wall_time=1.0)


def test_diff_sim_readiness_failure_stops_launch(monkeypatch):
    """任一门控失败都必须关闭 launch，不能继续返回成功动作。"""
    monkeypatch.setenv("ROS_LOG_DIR", "/tmp/w1_nav2_launch_test")
    for launch_name in ("diff_sim.launch.py", "diff_sim_mapping.launch.py"):
        launch_path = (
            REPO / "mobile_simulation/w1_nav2/launch" / launch_name)
        spec = importlib.util.spec_from_file_location(
            f"{launch_name}_readiness_handler_test", launch_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        class Event:
            returncode = 0

        class Context:
            is_shutdown = False

        success_actions = [object()]
        handler = module._readiness_exit_handler(
            success_actions, "仿真未就绪")
        assert handler(Event(), Context()) is success_actions

        Event.returncode = 1
        failure_actions = handler(Event(), Context())
        assert len(failure_actions) == 2
        assert isinstance(failure_actions[0], module.LogInfo)
        assert isinstance(failure_actions[1], module.EmitEvent)

        Context.is_shutdown = True
        assert handler(Event(), Context()) is None


def test_path_visualizer_treats_closed_ros_context_as_clean_exit(monkeypatch):
    """ROS 上下文关闭异常不应让轨迹节点以错误码结束。"""
    script = REPO / "mobile_simulation/w1_nav2/script/path_trace_visualizer.py"
    spec = importlib.util.spec_from_file_location(
        "path_trace_visualizer_shutdown_test", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class FakeNode:
        destroyed = False

        def destroy_node(self):
            self.destroyed = True

    monkeypatch.setattr(module.rclpy, "init", lambda: None)
    monkeypatch.setattr(module.rclpy, "ok", lambda: False)
    for shutdown_error in (
            RuntimeError("context closed"),
            module.ExternalShutdownException()):
        fake_node = FakeNode()
        shutdown_called = False

        def fail_if_shutdown_called():
            nonlocal shutdown_called
            shutdown_called = True

        monkeypatch.setattr(
            module, "PathTraceVisualizer", lambda: fake_node)
        monkeypatch.setattr(
            module.rclpy, "spin",
            lambda _node, error=shutdown_error:
            (_ for _ in ()).throw(error))
        monkeypatch.setattr(
            module.rclpy, "shutdown", fail_if_shutdown_called)

        module.main()

        assert fake_node.destroyed
        assert not shutdown_called


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
    """差速仿真统一使用 YAML，全向入口继续保留原有命令行参数。"""
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
    assert diff == set()


def test_diff_sim_yaml_controls_each_gazebo_sensor_at_source():
    """统一 YAML 的每个传感器开关都能实际移除对应 Gazebo 插件。"""
    launch = read("mobile_simulation/w1_nav2/launch/diff_sim.launch.py")
    config_path = "mobile_simulation/w1_nav2/config/diff_sim.yaml"
    config = read_yaml(config_path)
    spawn_path = (
        "mobile_simulation/diff-sim-container/src/w1_diff_gazebo/"
        "launch/spawn_w1_diff.launch.py")
    spawn_launch = read(spawn_path)
    urdf_path = REPO / (
        "mobile_simulation/diff-sim-container/src/w1_diff_gazebo/"
        "urdf/w1_diff_sim.urdf")

    expected_sensors = {
        "rgbd_front": "realsense_front",
        "rgbd_rear": "realsense_rear",
        "lidar_front": "laser_front",
        "lidar_rear": "laser_rear",
        "imu": "w1_imu",
    }
    assert set(config["sensors"]) == set(expected_sensors)
    assert "config', 'diff_sim.yaml'" in launch
    assert "DeclareLaunchArgument" not in launch
    assert "use_rgbd" not in launch
    assert "w1_diff_sim_no_rgbd.urdf" not in spawn_launch
    assert "'-topic', 'robot_description'" in spawn_launch

    spec = importlib.util.spec_from_file_location(
        "spawn_w1_diff_sensor_test", REPO / spawn_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    for disabled_config_name, disabled_sensor_name in expected_sensors.items():
        enabled = {name: True for name in expected_sensors}
        enabled[disabled_config_name] = False
        filtered = module._filtered_robot_description(urdf_path, enabled)
        remaining = {
            sensor.get("name")
            for sensor in ET.fromstring(filtered).iter("sensor")
        }
        assert disabled_sensor_name not in remaining
        assert (
            set(expected_sensors.values()) - {disabled_sensor_name}
        ).issubset(remaining)

    assert "condition=IfCondition(_bool_text(sensors['rgbd_front']))" in launch
    assert "condition=IfCondition(_bool_text(sensors['rgbd_rear']))" in launch


def test_diff_sim_rviz_is_required_and_not_configurable():
    """RViz 必须无条件启动，统一 YAML 中不提供关闭开关。"""
    launch = read("mobile_simulation/w1_nav2/launch/diff_sim.launch.py")
    config = read_yaml("mobile_simulation/w1_nav2/config/diff_sim.yaml")

    assert "rviz" not in config
    assert "target_action=sim_ready_cmd" in launch
    assert "[path_trace_cmd, rviz_cmd, nav2_bringup_cmd]" in launch
    assert "condition=" not in launch[launch.index("rviz_cmd ="):launch.index(
        "nav2_bringup_cmd =")]


def test_navigation_launches_share_the_default_map_alignment():
    """Both chassis align the default warehouse map to odom identically."""
    expected = {
        "map_odom_x": "0.03143",
        "map_odom_y": "-0.01856",
        "map_odom_yaw": "0.02040",
    }
    omni_launch = "mobile_simulation/w1_nav2/launch/omni_sim.launch.py"
    omni_defaults = launch_argument_defaults(omni_launch)
    assert {name: omni_defaults[name] for name in expected} == expected

    diff_config = read_yaml(
        "mobile_simulation/w1_nav2/config/diff_sim.yaml")["navigation"]
    for name, value in expected.items():
        assert math.isclose(
            float(diff_config[name]), float(value), abs_tol=1e-9)


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
    """Simulation goal tools and diff production params stay aligned."""
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
    assert (
        'plugin: "nav2_simple_goal_checker::SimpleGoalChecker"'
        in omni_params)
    assert 'plugin: "nav2_controller::SimpleGoalChecker"' not in omni_params

    production_params = read(
        "mobile_navigation/nav2_bringup/params/w1_nav2_params_diff.yaml")
    assert (
        'current_goal_checker: "high_precision_goal_checker"'
        in production_params)
    assert 'current_controller: "RPPController"' in production_params
    production_config = read_yaml(
        "mobile_navigation/nav2_bringup/params/w1_nav2_params_diff.yaml")
    assert production_config["controller_server"]["ros__parameters"][
        "speed_planner_plugins"] == [
            "TrapezoidalSpeedPlanner",
            "SCurveSpeedPlanner",
            "ReachabilitySpeedPlanner",
        ]
    assert (
        'plugin: "nav2_simple_goal_checker::SimpleGoalChecker"'
        in production_params)

    diff_launch = read("mobile_simulation/w1_nav2/launch/diff_sim.launch.py")
    diff_config = read_yaml(
        "mobile_simulation/w1_nav2/config/diff_sim.yaml")["navigation"]
    assert diff_config["params_file"] == (
        "package://nav2_bringup/params/w1_nav2_params_diff.yaml")
    assert "navigation.get('params_file')" in diff_launch
    assert "w1_nav2_params_diff_sim.yaml" not in diff_launch
    assert not (
        REPO / "mobile_navigation/nav2_bringup/params/"
        "w1_nav2_params_diff_sim.yaml").exists()


def test_diff_sim_uses_two_adaptive_readiness_gates():
    """Gazebo就绪后生成模型，全部启用接口就绪后启动Nav2。"""
    diff_launch = read("mobile_simulation/w1_nav2/launch/diff_sim.launch.py")
    config = read_yaml("mobile_simulation/w1_nav2/config/diff_sim.yaml")
    gazebo_waiter = read(
        "mobile_simulation/w1_nav2/script/wait_for_gazebo_ready.py")
    sim_waiter = read(
        "mobile_simulation/w1_nav2/script/wait_for_sim_ready.py")

    assert "'GAZEBO_RESOURCE_PATH'" in diff_launch
    assert "model_spawn_delay_s" not in config["gazebo"]
    assert "model_spawn_delay_s" not in diff_launch
    assert "TimerAction" not in diff_launch
    assert "ld.add_action(gzclient_cmd)" in diff_launch
    assert "wait_for_gazebo_ready.py" in diff_launch
    assert "wait_for_sim_ready.py" in diff_launch
    assert "SPAWN_SERVICE = '/spawn_entity'" in gazebo_waiter
    assert "CLOCK_TOPIC = '/clock'" in gazebo_waiter
    assert "STABILIZATION_S = 0.75" in gazebo_waiter
    for topic in (
        "/feedback/odom", "/orbbec/front/rgb", "/orbbec/front/depth",
        "/orbbec/front/camera_info", "/orbbec/rear/rgb",
        "/orbbec/rear/depth", "/orbbec/rear/camera_info",
        "/orbbec/front/laser_scan", "/orbbec/rear/laser_scan",
        "/laser_scan/front", "/laser_scan/rear", "/imu/chassis",
    ):
        assert topic in sim_waiter
    assert "target_action=gazebo_ready_cmd" in diff_launch
    assert "target_action=sim_ready_cmd" in diff_launch
    assert "period=15.0" not in diff_launch
    assert "ld.add_action(gzserver_cmd)" in diff_launch
    assert "ld.add_action(spawn_robot_when_gazebo_ready)" in diff_launch
    assert "ld.add_action(start_navigation_when_ready)" in diff_launch
    assert diff_launch.index(
        "ld.add_action(spawn_robot_when_gazebo_ready)") < diff_launch.index(
            "ld.add_action(gazebo_ready_cmd)")
    assert diff_launch.index(
        "ld.add_action(start_navigation_when_ready)") < diff_launch.index(
            "ld.add_action(gazebo_ready_cmd)")
    assert "ld.add_action(spawn_w1_diff_cmd)" not in diff_launch
    assert "ld.add_action(sim_ready_cmd)" not in diff_launch
    assert "ld.add_action(nav2_bringup_cmd)" not in diff_launch


def test_speed_planner_plugin_is_not_linked_into_controller_server():
    """Only the support library may be linked into the host."""
    speed_cmake = read(
        "mobile_navigation/nav2_speed_planner/CMakeLists.txt")
    assert (
        "set(support_library_name nav2_initial_motion_arbiter)"
        in speed_cmake)
    assert re.search(
        r"add_library\(\$\{support_library_name\}\s+SHARED\s+"
        r"src/core/initial_motion\.cpp\)",
        speed_cmake,
    )
    plugin_block = speed_cmake[
        speed_cmake.index("add_library(${library_name}"):
        speed_cmake.index("ament_target_dependencies(${library_name}")]
    for plugin_source in (
        "src/speed_planner_base.cpp",
        "src/backends/trapezoidal_speed_planner.cpp",
        "src/backends/scurve_speed_planner.cpp",
        "src/backends/reachability_speed_planner.cpp",
    ):
        assert plugin_source in plugin_block
    assert "ament_export_libraries(${support_library_name})" in speed_cmake
    assert "ament_export_libraries(${library_name})" not in speed_cmake


def test_diff_mapping_uses_adaptive_gazebo_readiness_gate():
    """差速建图必须按 Gazebo 实际状态启动，禁止恢复固定延迟。"""
    launch = read(
        "mobile_simulation/w1_nav2/launch/diff_sim_mapping.launch.py")

    assert "TimerAction" not in launch
    assert "wait_for_gazebo_ready.py" in launch
    assert "target_action=gazebo_ready_cmd" in launch
    assert "ld.add_action(gzclient_cmd)" in launch
    assert "ld.add_action(spawn_w1_diff_cmd)" not in launch
    assert "ld.add_action(rviz_cmd)" not in launch
    assert launch.index(
        "ld.add_action(spawn_robot_when_gazebo_ready)") < launch.index(
            "ld.add_action(gazebo_ready_cmd)")


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
