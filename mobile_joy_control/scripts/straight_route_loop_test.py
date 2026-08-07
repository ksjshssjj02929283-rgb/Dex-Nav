#!/usr/bin/env python3
"""读取任意数量的连续 LINE 路网边，逐段停车并做往返循环测试。

脚本可在配置区选择本仓 Theta* -> MPPI 的 START 或 TANGENT 模式协议：
- START：每个路段在真实起点前插入两个零位移 marker，该段保持接收任务时的 yaw；
- TANGENT：每个路段在真实路径首尾各插入一个零位移 marker，先原地对齐该段
  切线 yaw，再沿切线行驶。反向起步切线相差 pi，因此可用于观察原地
  旋转时的定位漂移。

每条 LINE 边单独发送一次 FollowPath。机器人到达中间导航点后，上一段 Action
必须成功结束并停车，等待 segment_pause_sec 后重新对舵，再发送下一段。
MPPI 识别并移除 marker 后再跟踪真实路径点，不会把重复点交给优化器。

每一次单程测试都通过 TF ``map -> base_link`` 采样实际轨迹，并输出相对
整条折线路径的最大横向偏差。结果同时写入 CSV，并生成各单程实际平移速度曲线图。

使用前修改下方 ``SCRIPT_CONFIG``，然后直接运行：

  python3 straight_route_loop_test.py

约定：
- loop_count 表示单程次数；10 次即 A->B、B->A 各 5 次；
- 一个单程会逐段走完整条边链，中间导航点会停车；
- 只接受能拼成一条无分叉连续链的 LINE 边；往返测试要求所有边均可双向通行；
- 第一次启动时机器人必须靠近整条边链的一个端点；
- 脚本直接调用 /follow_path，不经过全局规划器和导航行为树；
- 测试前应确认 Nav2、局部代价地图和 controller_server 已激活。
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from chassis_interfaces.msg import ChassisMotorControl
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_interfaces.action import SteeringAngleControl
from nav_interfaces.srv import GetRouteGraph
from nav_msgs.msg import Path as NavPath
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


WHEEL_NAMES = ("front_left", "front_right", "rear_left", "rear_right")
WHEEL_COUNT = len(WHEEL_NAMES)
ROS_LOG_WARN_LEVEL = 30

# ============================ 用户配置区 ============================
# 正常使用时只需要修改这里，不需要在命令行附带 --ros-args。
SCRIPT_CONFIG = {
    # 路网与导航接口
    "route_graph_service": "/map_asset_manager/get_route_graph",
    # 按需填写要走的 LINE 边 ID，例如 ["edge_1", "edge_2"]。填写顺序不限，
    # 脚本会根据共享导航点自动排序。保留 [""] 表示自动使用路网中的全部 LINE
    # 边；此时这些边也必须恰好组成一条无分叉、无环的连续链。
    "edge_ids": [""],
    "follow_path_action": "/follow_path",
    "controller_id": "FollowPath",
    "goal_checker_id": "goal_checker",
    "map_frame": "map",
    "base_frame": "base_link",

    # 循环与轨迹
    # 可选 "start" 或 "tangent"。直接在这里切换，不需要命令行 param set。
    # tangent 会在每个路段起点把车体 yaw 对齐该段方向；往返测试的起步方向
    # 相差 pi，可用于检查原地旋转时定位是否发生漂移。
    # controller_server 需已启用 FollowPath.tangent_phase_control_enabled；本仓
    # w1_nav2_params.yaml 当前已经设为 true，脚本本身无需再修改该 YAML。
    "path_control_mode": "tangent",
    "loop_count": 10,                 # 单程次数；10 表示共走 10 个单程
    "path_resolution_m": 0.05,       # 各路段插值点间距
    "sample_period_sec": 0.05,       # 实际轨迹采样周期
    "segment_pause_sec": 1.0,        # 到达中间导航点后的停车时间
    "loop_pause_sec": 1.0,           # 相邻单程之间的等待时间
    "start_endpoint_tolerance_m": 0.30,

    # 每次起步前的静止对舵
    "presteer_enabled": True,
    "steering_action_name": "/steering_angle_control",
    "motor_feedback_topic": "/feedback/omni_chassis_motor",
    "action_timeout_sec": 3.0,
    "settle_tolerance_rad": 0.08,
    "steering_feedback_timeout_sec": 0.20,
    "steering_limit_rad": 0.8 * math.pi,

    # 相对于运行命令所在的当前目录；文件夹不存在时会自动创建。
    "output_dir": "./straight_route_loop_results",
    # 留空时自动搜索系统中文字体；也可填写 .ttf/.otf/.ttc 字体文件绝对路径。
    "chinese_font_path": "",
}
# ==================================================================


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class RouteEdge:
    edge_id: str
    from_key: str
    to_key: str
    from_id: str
    to_id: str
    from_point: Point2D
    to_point: Point2D
    bidirectional: bool


@dataclass(frozen=True)
class RouteChain:
    edge_ids: List[str]
    point_ids: List[str]
    points: List[Point2D]
    forward_allowed: bool
    reverse_allowed: bool


@dataclass(frozen=True)
class MotorFeedback:
    receive_sec: float
    positions: List[float]


@dataclass(frozen=True)
class SpeedSample:
    elapsed_sec: float
    speed_mps: float


@dataclass(frozen=True)
class RunSpeedSeries:
    run_index: int
    from_id: str
    to_id: str
    samples: List[SpeedSample]


@dataclass(frozen=True)
class YawSample:
    elapsed_sec: float
    yaw_error_rad: float


@dataclass(frozen=True)
class IntermediatePointError:
    point_id: str
    x_error_m: float
    y_error_m: float
    xy_error_m: float


def normalize_angle(angle: float) -> float:
    """把角度归一化到 [-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """从四元数提取平面 yaw。"""
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def distance(first: Point2D, second: Point2D) -> float:
    return math.hypot(first.x - second.x, first.y - second.y)


def point_to_segment_distance(
    point: Point2D, segment_start: Point2D, segment_end: Point2D
) -> float:
    """返回点到有限线段的最短距离，单位 m。"""
    dx = segment_end.x - segment_start.x
    dy = segment_end.y - segment_start.y
    squared_length = dx * dx + dy * dy
    if squared_length <= 1.0e-18:
        return distance(point, segment_start)
    projection = (
        (point.x - segment_start.x) * dx
        + (point.y - segment_start.y) * dy
    ) / squared_length
    ratio = min(1.0, max(0.0, projection))
    projected = Point2D(
        segment_start.x + ratio * dx,
        segment_start.y + ratio * dy,
    )
    return distance(point, projected)


def polyline_length(points: List[Point2D]) -> float:
    return sum(
        distance(points[index], points[index + 1])
        for index in range(len(points) - 1)
    )


def closest_polyline_segment(
    point: Point2D, route_points: List[Point2D]
) -> Tuple[float, float]:
    """返回到折线的最短距离及对应线段切线 yaw。"""
    if len(route_points) < 2:
        raise ValueError("连续路网至少需要两个不同端点")
    best_distance = math.inf
    best_yaw = 0.0
    for index in range(len(route_points) - 1):
        start = route_points[index]
        end = route_points[index + 1]
        segment_length = distance(start, end)
        if segment_length <= 1.0e-9:
            continue
        candidate_distance = point_to_segment_distance(point, start, end)
        if candidate_distance < best_distance:
            best_distance = candidate_distance
            best_yaw = math.atan2(end.y - start.y, end.x - start.x)
    if not math.isfinite(best_distance):
        raise ValueError("连续路网的所有相邻端点都重合")
    return best_distance, best_yaw


class StraightRouteLoopTest(Node):
    """读取连续 LINE 路网边链并执行 START/TANGENT 模式往返测试。"""

    def __init__(self) -> None:
        super().__init__("straight_route_loop_test")

        for parameter_name, default_value in SCRIPT_CONFIG.items():
            self.declare_parameter(parameter_name, default_value)

        self.route_graph_service = str(
            self.get_parameter("route_graph_service").value)
        self.requested_edge_ids = [
            str(edge_id).strip()
            for edge_id in self.get_parameter("edge_ids").value
            if str(edge_id).strip()
        ]
        if len(set(self.requested_edge_ids)) != len(self.requested_edge_ids):
            raise ValueError("edge_ids 中不能包含重复的边 ID")
        self.follow_path_action = str(
            self.get_parameter("follow_path_action").value)
        self.controller_id = str(
            self.get_parameter("controller_id").value)
        self.goal_checker_id = str(
            self.get_parameter("goal_checker_id").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.path_control_mode = str(
            self.get_parameter("path_control_mode").value
        ).strip().lower()
        self.loop_count = int(self.get_parameter("loop_count").value)
        self.path_resolution_m = float(
            self.get_parameter("path_resolution_m").value)
        self.sample_period_sec = float(
            self.get_parameter("sample_period_sec").value)
        self.segment_pause_sec = float(
            self.get_parameter("segment_pause_sec").value)
        self.loop_pause_sec = float(
            self.get_parameter("loop_pause_sec").value)
        self.start_endpoint_tolerance_m = float(
            self.get_parameter("start_endpoint_tolerance_m").value)
        self.presteer_enabled = bool(
            self.get_parameter("presteer_enabled").value)
        self.steering_action_name = str(
            self.get_parameter("steering_action_name").value)
        self.motor_feedback_topic = str(
            self.get_parameter("motor_feedback_topic").value)
        self.action_timeout_sec = float(
            self.get_parameter("action_timeout_sec").value)
        self.settle_tolerance_rad = float(
            self.get_parameter("settle_tolerance_rad").value)
        self.steering_feedback_timeout_sec = float(
            self.get_parameter("steering_feedback_timeout_sec").value)
        self.steering_limit_rad = float(
            self.get_parameter("steering_limit_rad").value)
        self.chinese_font_path = str(
            self.get_parameter("chinese_font_path").value).strip()

        if self.path_control_mode not in ("start", "tangent"):
            raise ValueError(
                "path_control_mode 只支持 'start' 或 'tangent'，"
                f"当前值为 '{self.path_control_mode}'")
        if self.loop_count <= 0:
            raise ValueError("loop_count 必须大于 0")
        if self.path_resolution_m <= 0.0:
            raise ValueError("path_resolution_m 必须大于 0")
        if self.sample_period_sec <= 0.0:
            raise ValueError("sample_period_sec 必须大于 0")
        if self.segment_pause_sec < 0.0:
            raise ValueError("segment_pause_sec 不能小于 0")
        if self.loop_pause_sec < 0.0:
            raise ValueError("loop_pause_sec 不能小于 0")
        if self.start_endpoint_tolerance_m <= 0.0:
            raise ValueError("start_endpoint_tolerance_m 必须大于 0")
        if self.action_timeout_sec <= 0.0:
            raise ValueError("action_timeout_sec 必须大于 0")
        if self.settle_tolerance_rad <= 0.0:
            raise ValueError("settle_tolerance_rad 必须大于 0")
        if self.steering_feedback_timeout_sec <= 0.0:
            raise ValueError("steering_feedback_timeout_sec 必须大于 0")
        if not 0.0 < self.steering_limit_rad <= math.pi:
            raise ValueError("steering_limit_rad 必须在 (0, pi] 内")

        output_root = Path(
            str(self.get_parameter("output_dir").value)
        ).expanduser().resolve()
        output_root.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.output_dir = output_root / (
            f"直线路网测试_{self.path_control_mode.upper()}_{timestamp}")
        self.output_dir.mkdir(parents=True, exist_ok=False)
        self.output_csv_path = self.output_dir / "测试结果.csv"
        self.csv_stream = self.output_csv_path.open(
            "w", encoding="utf-8-sig", newline="")
        self.csv_writer = csv.DictWriter(
            self.csv_stream,
            fieldnames=[
                "单程序号",
                "起点ID",
                "终点ID",
                "状态",
                "行驶耗时_秒",
                "预对舵耗时_秒",
                "预对舵最终最大误差_弧度",
                "采样点数",
                "实际路程长度_米",
                "最大轨迹横向偏差_米",
                "中间点X误差_米",
                "中间点Y误差_米",
                "中间点XY误差_米",
                "终点位置误差_米",
            ],
        )
        self.csv_writer.writeheader()
        self.csv_stream.flush()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.graph_client = self.create_client(
            GetRouteGraph, self.route_graph_service)
        self.follow_path_client = ActionClient(
            self, FollowPath, self.follow_path_action)
        self.steering_action_client = ActionClient(
            self, SteeringAngleControl, self.steering_action_name)
        self.motor_feedback_sub = self.create_subscription(
            ChassisMotorControl,
            self.motor_feedback_topic,
            self._motor_feedback_callback,
            20,
        )
        self.rosout_sub = self.create_subscription(
            Log, "/rosout", self._rosout_callback, 100)

        self.route_chain: Optional[RouteChain] = None
        self.phase = "waiting_graph"
        self.graph_request_in_flight = False
        self.finished = False
        self.success = False

        self.next_route_forward: Optional[bool] = None
        self.active_route_forward = True
        self.run_route_points: List[Point2D] = []
        self.run_route_point_ids: List[str] = []
        self.run_from_id = ""
        self.run_to_id = ""
        self.active_segment_index = 0
        self.active_segment_count = 0
        self.active_route_points: List[Point2D] = []
        self.active_route_point_ids: List[str] = []
        self.active_start = Point2D(0.0, 0.0)
        self.active_end = Point2D(0.0, 0.0)
        self.active_from_id = ""
        self.active_to_id = ""
        self.active_path_yaw = 0.0
        self.active_goal_handle = None
        self.presteer_goal_handle = None
        self.presteer_request_id = 0
        self.presteer_started_sec = 0.0
        self.presteer_deadline_sec = 0.0
        self.presteer_elapsed_sec = math.nan
        self.presteer_final_max_error_rad = math.nan
        self.run_presteer_elapsed_sec = 0.0
        self.run_presteer_max_error_rad = math.nan
        self.presteer_targets_rad: List[float] = []
        self.latest_motor_feedback: Optional[MotorFeedback] = None
        self.completed_runs = 0
        self.run_started_sec = 0.0
        self.run_motion_started = False
        self.segment_started_sec = 0.0
        self.segment_pause_until_sec = 0.0
        self.pause_until_sec = 0.0
        self.max_cross_track_error_m = 0.0
        self.travel_distance_m = 0.0
        self.trajectory: List[Pose2D] = []
        self.speed_samples: List[SpeedSample] = []
        self.yaw_samples: List[YawSample] = []
        self.intermediate_point_errors: List[IntermediatePointError] = []
        self.active_diagnostic_logs: List[str] = []
        self.last_speed_pose: Optional[Pose2D] = None
        self.last_speed_sample_sec: Optional[float] = None
        self.last_wait_log_sec = -math.inf
        self.last_tf_warning_sec = -math.inf

        self.timer = self.create_timer(
            self.sample_period_sec, self._timer_callback)
        self.get_logger().info(
            "直线路网循环测试启动："
            f"service={self.route_graph_service}，"
            "edge_ids="
            f"{self.requested_edge_ids or '自动选择全部连续 LINE 边'}，"
            f"路径控制模式={self.path_control_mode.upper()}，"
            f"单程次数={self.loop_count}，"
            f"中间点停车={self.segment_pause_sec:.3f}s，"
            f"采样间隔={self.path_resolution_m:.3f}m，"
            f"静止预对舵={'启用' if self.presteer_enabled else '禁用'}，"
            f"结果目录={self.output_dir}"
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _throttled_wait_log(self, text: str) -> None:
        now_sec = self._now_sec()
        if now_sec - self.last_wait_log_sec >= 2.0:
            self.last_wait_log_sec = now_sec
            self.get_logger().info(text)

    def _motor_feedback_callback(self, message: ChassisMotorControl) -> None:
        positions = [float(value) for value in message.position[:WHEEL_COUNT]]
        if len(positions) != WHEEL_COUNT or not all(
            math.isfinite(value) for value in positions
        ):
            return
        self.latest_motor_feedback = MotorFeedback(
            receive_sec=self._now_sec(), positions=positions)

    def _rosout_callback(self, message: Log) -> None:
        """暂存控制器在本次单程中的警告/错误，供中止诊断使用。"""
        if self.phase not in ("sending_goal", "running"):
            return
        if int(message.level) < ROS_LOG_WARN_LEVEL:
            return
        logger_name = str(message.name)
        normalized_name = logger_name.lower()
        relevant_names = (
            "controller", "mppi", "progress", "costmap", "velocity_smoother")
        if not any(name in normalized_name for name in relevant_names):
            return
        receive_time = datetime.now().isoformat(timespec="milliseconds")
        self.active_diagnostic_logs.append(
            f"[{receive_time}] [{logger_name}] {message.msg}")

    def _start_dependencies_ready(self) -> bool:
        """检查启动一次单程测试所需的服务与新鲜舵角反馈。"""
        if not self.follow_path_client.server_is_ready():
            self._throttled_wait_log(
                f"等待 FollowPath Action {self.follow_path_action}")
            return False
        if not self.presteer_enabled:
            return True
        if not self.steering_action_client.server_is_ready():
            self._throttled_wait_log(
                f"等待静止预对舵 Action {self.steering_action_name}")
            return False
        feedback = self.latest_motor_feedback
        feedback_fresh = (
            feedback is not None
            and self._now_sec() - feedback.receive_sec
            <= self.steering_feedback_timeout_sec
        )
        if not feedback_fresh:
            self._throttled_wait_log(
                "等待新鲜四轮舵角反馈 "
                f"{self.motor_feedback_topic}（阈值 "
                f"{self.steering_feedback_timeout_sec:.3f}s）")
            return False
        return True

    def _timer_callback(self) -> None:
        if self.finished:
            return

        if self.phase == "waiting_graph":
            self._request_route_graph_if_ready()
            return

        if self.phase == "waiting_start":
            pose = self._lookup_robot_pose()
            if pose is None:
                self._throttled_wait_log(
                    f"等待 TF {self.map_frame} -> {self.base_frame}")
                return
            if not self._start_dependencies_ready():
                return
            self._select_initial_direction_and_start(pose)
            return

        if self.phase == "presteering":
            if self.run_motion_started:
                self._sample_robot_pose()
            if self._now_sec() >= self.presteer_deadline_sec:
                self._fail(
                    "等待静止预对舵结果超时（服务端超时后额外等待了 1s）")
            return

        if self.phase == "waiting_follow_path":
            if self.run_motion_started:
                self._sample_robot_pose()
            pose = self._lookup_robot_pose()
            if pose is None:
                self._throttled_wait_log(
                    f"预对舵已完成，等待 TF "
                    f"{self.map_frame} -> {self.base_frame}")
                return
            if not self.follow_path_client.server_is_ready():
                self._throttled_wait_log(
                    f"预对舵已完成，等待 FollowPath Action "
                    f"{self.follow_path_action}")
                return
            self._send_follow_path(pose)
            return

        if self.phase == "running":
            self._sample_robot_pose()
            return

        if self.phase == "segment_pause":
            # 停车期间继续采样，使整次单程的速度曲线能显示中间停车过程。
            self._sample_robot_pose()
            if self._now_sec() < self.segment_pause_until_sec:
                return
            pose = self._lookup_robot_pose()
            if pose is None:
                self._throttled_wait_log(
                    f"中间点停车结束，等待 TF "
                    f"{self.map_frame} -> {self.base_frame}")
                return
            if not self._start_dependencies_ready():
                return
            self._start_active_segment(pose)
            return

        if self.phase == "pause" and self._now_sec() >= self.pause_until_sec:
            pose = self._lookup_robot_pose()
            if pose is None:
                self._throttled_wait_log(
                    f"等待 TF {self.map_frame} -> {self.base_frame}")
                return
            if not self._start_dependencies_ready():
                return
            self._start_next_run(pose)

    def _request_route_graph_if_ready(self) -> None:
        if self.graph_request_in_flight:
            return
        if not self.graph_client.service_is_ready():
            self._throttled_wait_log(
                f"等待路网服务 {self.route_graph_service}")
            return

        self.graph_request_in_flight = True
        request = GetRouteGraph.Request()
        future = self.graph_client.call_async(request)
        future.add_done_callback(self._route_graph_response_callback)
        self.get_logger().info(
            f"正在读取路网：{self.route_graph_service}")

    def _route_graph_response_callback(self, future) -> None:
        self.graph_request_in_flight = False
        try:
            response = future.result()
        except Exception as error:
            self._fail(f"读取路网服务异常：{error}")
            return

        if response is None or not bool(response.success):
            message = getattr(response, "message", "服务返回失败")
            self._fail(f"读取路网失败：{message}")
            return

        try:
            self.route_chain = self._select_route_chain(response)
        except ValueError as error:
            self._fail(str(error))
            return

        if self.loop_count > 1 and not (
            self.route_chain.forward_allowed
            and self.route_chain.reverse_allowed
        ):
            self._fail(
                "所选连续路网包含单向边，不能进行整体往返循环测试")
            return

        route_length = polyline_length(self.route_chain.points)
        route_description = " -> ".join(self.route_chain.point_ids)
        self.phase = "waiting_start"
        self.get_logger().info(
            f"已拼接 {len(self.route_chain.edge_ids)} 条连续 LINE 边："
            f"{route_description}；边 ID={self.route_chain.edge_ids}，"
            f"总长度={route_length:.3f}m，"
            "正向/反向可通行="
            f"{self.route_chain.forward_allowed}/"
            f"{self.route_chain.reverse_allowed}"
        )

    def _select_route_chain(self, response) -> RouteChain:
        """选取 LINE 边，并按共享端点自动拼成一条无分叉边链。"""
        line_edges = [
            edge for edge in response.edges
            if str(edge.edge_type).upper() == "LINE"
        ]
        edges_by_id = {str(edge.id): edge for edge in line_edges}
        if self.requested_edge_ids:
            missing_ids = [
                edge_id for edge_id in self.requested_edge_ids
                if edge_id not in edges_by_id
            ]
            if missing_ids:
                available = ", ".join(sorted(edges_by_id))
                raise ValueError(
                    f"找不到 LINE 边 {missing_ids}；"
                    f"当前 LINE 边：{available or '无'}")
            selected_messages = [
                edges_by_id[edge_id] for edge_id in self.requested_edge_ids]
        else:
            selected_messages = line_edges
        if not selected_messages:
            raise ValueError("路网中没有可用于测试的 LINE 边")

        nav_points = {
            str(point.name): Point2D(float(point.x), float(point.y))
            for point in response.nav_points
        }
        junctions = {
            str(point.id): Point2D(float(point.x), float(point.y))
            for point in response.junctions
        }

        def resolve_endpoint(endpoint) -> Tuple[str, str, Point2D]:
            endpoint_id = str(endpoint.id)
            endpoint_type = str(endpoint.type).lower()
            if endpoint_type == "nav" and endpoint_id in nav_points:
                return (
                    f"nav:{endpoint_id}", endpoint_id,
                    nav_points[endpoint_id])
            if endpoint_type == "junction" and endpoint_id in junctions:
                return (
                    f"junction:{endpoint_id}", endpoint_id,
                    junctions[endpoint_id])
            # 兼容旧数据中 type 缺失或大小写不一致，但 id 必须唯一。
            candidates: List[Tuple[str, str, Point2D]] = []
            if endpoint_id in nav_points:
                candidates.append((
                    f"nav:{endpoint_id}", endpoint_id,
                    nav_points[endpoint_id]))
            if endpoint_id in junctions:
                candidates.append((
                    f"junction:{endpoint_id}", endpoint_id,
                    junctions[endpoint_id]))
            if len(candidates) == 1:
                return candidates[0]
            raise ValueError(
                f"无法解析路网端点 {endpoint_type}:{endpoint_id}")

        selected_edges: List[RouteEdge] = []
        for edge_message in selected_messages:
            from_key, from_id, from_point = resolve_endpoint(
                edge_message.from_ep)
            to_key, to_id, to_point = resolve_endpoint(edge_message.to_ep)
            edge_id = str(edge_message.id)
            if from_key == to_key or distance(from_point, to_point) <= 1.0e-6:
                raise ValueError(f"路网边 {edge_id} 两端重合，无法测试")
            selected_edges.append(RouteEdge(
                edge_id=edge_id,
                from_key=from_key,
                to_key=to_key,
                from_id=from_id,
                to_id=to_id,
                from_point=from_point,
                to_point=to_point,
                bidirectional=bool(edge_message.bidirectional),
            ))

        adjacency: Dict[str, List[int]] = {}
        point_by_key: Dict[str, Point2D] = {}
        point_id_by_key: Dict[str, str] = {}
        for edge_index, edge in enumerate(selected_edges):
            adjacency.setdefault(edge.from_key, []).append(edge_index)
            adjacency.setdefault(edge.to_key, []).append(edge_index)
            point_by_key[edge.from_key] = edge.from_point
            point_by_key[edge.to_key] = edge.to_point
            point_id_by_key[edge.from_key] = edge.from_id
            point_id_by_key[edge.to_key] = edge.to_id

        branch_keys = [
            key for key, edge_indexes in adjacency.items()
            if len(edge_indexes) > 2
        ]
        if branch_keys:
            branch_ids = [point_id_by_key[key] for key in branch_keys]
            raise ValueError(
                f"所选 LINE 边存在分叉点 {branch_ids}，无法确定唯一行驶顺序")
        terminal_keys = [
            key for key, edge_indexes in adjacency.items()
            if len(edge_indexes) == 1
        ]
        if len(terminal_keys) != 2:
            raise ValueError(
                "所选 LINE 边必须组成一条无环连续链；"
                f"当前检测到 {len(terminal_keys)} 个链端点")

        # 从稳定排序后的一个链端点开始遍历。用户配置中的 edge_ids 顺序不影响
        # 拼接结果；机器人首次运行时仍会自动选择离自己最近的链端作为起点。
        current_key = sorted(terminal_keys)[0]
        ordered_point_keys = [current_key]
        ordered_edge_ids: List[str] = []
        visited_edges = set()
        forward_allowed = True
        reverse_allowed = True
        while len(visited_edges) < len(selected_edges):
            next_indexes = [
                edge_index for edge_index in adjacency[current_key]
                if edge_index not in visited_edges
            ]
            if len(next_indexes) != 1:
                raise ValueError("所选 LINE 边彼此不连续，无法拼成一条完整路径")
            edge_index = next_indexes[0]
            edge = selected_edges[edge_index]
            traversing_from_to = edge.from_key == current_key
            if not edge.bidirectional:
                forward_allowed = forward_allowed and traversing_from_to
                reverse_allowed = reverse_allowed and not traversing_from_to
            next_key = edge.to_key if traversing_from_to else edge.from_key
            visited_edges.add(edge_index)
            ordered_edge_ids.append(edge.edge_id)
            ordered_point_keys.append(next_key)
            current_key = next_key

        if current_key != sorted(terminal_keys)[1]:
            raise ValueError("所选 LINE 边未形成一条完整连续链")

        return RouteChain(
            edge_ids=ordered_edge_ids,
            point_ids=[point_id_by_key[key] for key in ordered_point_keys],
            points=[point_by_key[key] for key in ordered_point_keys],
            forward_allowed=forward_allowed,
            reverse_allowed=reverse_allowed,
        )

    def _lookup_robot_pose(self) -> Optional[Pose2D]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except TransformException as error:
            now_sec = self._now_sec()
            if now_sec - self.last_tf_warning_sec >= 2.0:
                self.last_tf_warning_sec = now_sec
                self.get_logger().warning(
                    f"查询 TF {self.map_frame} -> {self.base_frame} "
                    f"失败：{error}"
                )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return Pose2D(
            x=float(translation.x),
            y=float(translation.y),
            yaw=quaternion_to_yaw(
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            ),
        )

    def _select_initial_direction_and_start(self, pose: Pose2D) -> None:
        assert self.route_chain is not None
        robot_point = Point2D(pose.x, pose.y)
        distance_from = distance(robot_point, self.route_chain.points[0])
        distance_to = distance(robot_point, self.route_chain.points[-1])
        nearest_distance = min(distance_from, distance_to)
        if nearest_distance > self.start_endpoint_tolerance_m:
            self._fail(
                "机器人不在连续路网的链端点附近："
                f"到 {self.route_chain.point_ids[0]}={distance_from:.3f}m，"
                f"到 {self.route_chain.point_ids[-1]}={distance_to:.3f}m，"
                f"允许阈值={self.start_endpoint_tolerance_m:.3f}m")
            return

        self.next_route_forward = distance_from <= distance_to
        self._start_next_run(pose)

    def _start_next_run(self, pose: Pose2D) -> None:
        assert self.route_chain is not None
        assert self.next_route_forward is not None

        if self.completed_runs >= self.loop_count:
            self._complete()
            return

        if self.next_route_forward:
            route_points = list(self.route_chain.points)
            route_point_ids = list(self.route_chain.point_ids)
            direction_allowed = self.route_chain.forward_allowed
        else:
            route_points = list(reversed(self.route_chain.points))
            route_point_ids = list(reversed(self.route_chain.point_ids))
            direction_allowed = self.route_chain.reverse_allowed

        start = route_points[0]
        from_id = route_point_ids[0]
        to_id = route_point_ids[-1]
        if not direction_allowed:
            self._fail(
                f"连续路网不允许从 {from_id} 向 {to_id} 行驶，"
                "请检查单向边方向或把机器人放到另一链端")
            return

        start_error = distance(Point2D(pose.x, pose.y), start)
        if start_error > self.start_endpoint_tolerance_m:
            self._fail(
                f"第 {self.completed_runs + 1} 次测试前机器人未到达起点 "
                f"{from_id}：误差={start_error:.3f}m，"
                f"允许阈值={self.start_endpoint_tolerance_m:.3f}m")
            return

        self.active_route_forward = self.next_route_forward
        self.run_route_points = route_points
        self.run_route_point_ids = route_point_ids
        self.run_from_id = from_id
        self.run_to_id = to_id
        self.active_segment_index = 0
        self.active_segment_count = len(route_points) - 1
        self.max_cross_track_error_m = 0.0
        self.travel_distance_m = 0.0
        self.trajectory = []
        self.speed_samples = []
        self.yaw_samples = []
        self.intermediate_point_errors = []
        self.active_diagnostic_logs = []
        self.last_speed_pose = None
        self.last_speed_sample_sec = None
        self.presteer_elapsed_sec = math.nan
        self.presteer_final_max_error_rad = math.nan
        self.run_presteer_elapsed_sec = 0.0
        self.run_presteer_max_error_rad = math.nan
        self.presteer_targets_rad = []
        self.run_started_sec = 0.0
        self.run_motion_started = False

        self._start_active_segment(pose)

    def _start_active_segment(self, pose: Pose2D) -> None:
        """准备当前路段；每条 LINE 边都单独预对舵并发送 FollowPath。"""
        if not 0 <= self.active_segment_index < self.active_segment_count:
            self._fail("当前路段序号无效，无法继续测试")
            return

        index = self.active_segment_index
        self.active_route_points = self.run_route_points[index:index + 2]
        self.active_route_point_ids = self.run_route_point_ids[index:index + 2]
        self.active_start = self.active_route_points[0]
        self.active_end = self.active_route_points[1]
        self.active_from_id = self.active_route_point_ids[0]
        self.active_to_id = self.active_route_point_ids[1]

        start_error = distance(
            Point2D(pose.x, pose.y), self.active_start)
        if start_error > self.start_endpoint_tolerance_m:
            self._fail(
                f"第 {self.completed_runs + 1} 次单程的第 "
                f"{index + 1}/{self.active_segment_count} 段起步前，机器人未到达 "
                f"{self.active_from_id}：误差={start_error:.3f}m，"
                f"允许阈值={self.start_endpoint_tolerance_m:.3f}m")
            return

        self.presteer_elapsed_sec = math.nan
        self.presteer_final_max_error_rad = math.nan
        self.presteer_targets_rad = []
        self.get_logger().info(
            f"第 {self.completed_runs + 1}/{self.loop_count} 次单程，"
            f"准备第 {index + 1}/{self.active_segment_count} 段："
            f"{self.active_from_id} -> {self.active_to_id}"
        )
        if self.presteer_enabled:
            self._start_presteer(pose)
        else:
            self._send_follow_path(pose)

    def _compute_presteer_targets(self, pose: Pose2D) -> List[float]:
        """求当前路段在 base 坐标系内的最近等价舵角。"""
        feedback = self.latest_motor_feedback
        if feedback is None:
            raise ValueError("尚未收到四轮舵角反馈")
        if (
            self._now_sec() - feedback.receive_sec
            > self.steering_feedback_timeout_sec
        ):
            raise ValueError("四轮舵角反馈已超时")

        _, line_yaw_map = closest_polyline_segment(
            self.active_start, self.active_route_points)
        direction_base = normalize_angle(line_yaw_map - pose.yaw)
        candidate_a = direction_base
        candidate_b = normalize_angle(direction_base + math.pi)
        valid_a = abs(candidate_a) <= self.steering_limit_rad
        valid_b = abs(candidate_b) <= self.steering_limit_rad
        if not valid_a and not valid_b:
            raise ValueError(
                "路径起步方向在舵角限位内没有可用等价解："
                f"方向={direction_base:.3f}rad，"
                f"限位=±{self.steering_limit_rad:.3f}rad")

        targets: List[float] = []
        for wheel, current_angle in enumerate(feedback.positions):
            cost_a = (
                abs(normalize_angle(candidate_a - current_angle))
                if valid_a else math.inf
            )
            cost_b = (
                abs(normalize_angle(candidate_b - current_angle))
                if valid_b else math.inf
            )
            target = candidate_a if cost_a <= cost_b else candidate_b
            if not math.isfinite(target):
                raise ValueError(f"{WHEEL_NAMES[wheel]} 目标舵角无效")
            targets.append(target)
        return targets

    def _start_presteer(self, pose: Pose2D) -> None:
        try:
            targets = self._compute_presteer_targets(pose)
        except ValueError as error:
            self._fail(f"无法计算静止预对舵目标：{error}")
            return

        goal = SteeringAngleControl.Goal()
        goal.target_steer_angles_rad = targets
        goal.timeout_sec = self.action_timeout_sec
        goal.settle_tolerance_rad = self.settle_tolerance_rad

        self.presteer_request_id += 1
        request_id = self.presteer_request_id
        self.presteer_started_sec = self._now_sec()
        self.presteer_deadline_sec = (
            self.presteer_started_sec + self.action_timeout_sec + 1.0)
        self.presteer_targets_rad = list(targets)
        self.phase = "presteering"

        try:
            future = self.steering_action_client.send_goal_async(
                goal,
                feedback_callback=lambda feedback_message: (
                    self._presteer_feedback_callback(
                        feedback_message, request_id)),
            )
        except Exception as error:
            self._fail(f"发送静止预对舵目标失败：{error}")
            return
        future.add_done_callback(
            lambda done_future: self._presteer_goal_response_callback(
                done_future, request_id))
        self.get_logger().info(
            f"第 {self.completed_runs + 1}/{self.loop_count} 次单程，"
            f"第 {self.active_segment_index + 1}/"
            f"{self.active_segment_count} 段预对舵："
            "FL/FR/RL/RR="
            f"[{targets[0]:.3f}, {targets[1]:.3f}, "
            f"{targets[2]:.3f}, {targets[3]:.3f}]rad，"
            f"容差={self.settle_tolerance_rad:.3f}rad，"
            f"超时={self.action_timeout_sec:.2f}s；四轮速度保持为零"
        )

    def _presteer_feedback_callback(
        self, feedback_message, request_id: int
    ) -> None:
        if request_id != self.presteer_request_id:
            return
        if self.phase != "presteering":
            return
        self.presteer_final_max_error_rad = float(
            feedback_message.feedback.max_steer_error)

    def _presteer_goal_response_callback(
        self, future, request_id: int
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            if request_id == self.presteer_request_id:
                self._fail(f"发送静止预对舵目标异常：{error}")
            return

        if (
            request_id != self.presteer_request_id
            or self.phase != "presteering"
        ):
            if goal_handle is not None and goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if goal_handle is None or not goal_handle.accepted:
            self._fail("SteeringAngleControl 拒绝了静止预对舵目标")
            return

        self.presteer_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done_future: self._presteer_result_callback(
                done_future, request_id))

    def _presteer_result_callback(self, future, request_id: int) -> None:
        if (
            request_id != self.presteer_request_id
            or self.phase != "presteering"
        ):
            return
        self.presteer_goal_handle = None
        self.presteer_elapsed_sec = max(
            0.0, self._now_sec() - self.presteer_started_sec)
        try:
            wrapped_result = future.result()
        except Exception as error:
            self._fail(f"读取静止预对舵结果异常：{error}")
            return

        result = wrapped_result.result
        succeeded = (
            int(wrapped_result.status) == GoalStatus.STATUS_SUCCEEDED
            and result is not None
            and bool(result.success)
        )
        if not succeeded:
            message = str(result.message) if result is not None else "无结果"
            self._fail(
                "静止预对舵未成功："
                f"status={self._goal_status_text(int(wrapped_result.status))}，"
                f"message={message}")
            return

        self.run_presteer_elapsed_sec += self.presteer_elapsed_sec
        if math.isfinite(self.presteer_final_max_error_rad) and (
            not math.isfinite(self.run_presteer_max_error_rad)
            or self.presteer_final_max_error_rad
            > self.run_presteer_max_error_rad
        ):
            self.run_presteer_max_error_rad = (
                self.presteer_final_max_error_rad)

        self.get_logger().info(
            f"第 {self.active_segment_index + 1}/"
            f"{self.active_segment_count} 段静止预对舵完成："
            f"耗时={self.presteer_elapsed_sec:.3f}s，"
            "最终四轮最大舵角误差="
            f"{self.presteer_final_max_error_rad:.4f}rad；准备发送 FollowPath"
        )
        self.phase = "waiting_follow_path"

    def _send_follow_path(self, pose: Pose2D) -> None:
        start_error = distance(Point2D(pose.x, pose.y), self.active_start)
        if start_error > self.start_endpoint_tolerance_m:
            stage = "预对舵后" if self.presteer_enabled else "发送路径前"
            self._fail(
                f"{stage}机器人偏离起点 {self.active_from_id}："
                f"误差={start_error:.3f}m，"
                f"允许阈值={self.start_endpoint_tolerance_m:.3f}m")
            return

        if self.path_control_mode == "start":
            # START 保持接收任务时的当前 yaw；路径点也全部写当前 yaw，避免
            # PathAlignCritic 看到一个无法通过 wz 修正的虚假朝向误差。
            self.active_path_yaw = pose.yaw
            path = self._build_start_mode_path(
                self.active_route_points, self.active_path_yaw)
            yaw_description = f"固定 yaw={self.active_path_yaw:.3f}rad"
        else:
            # TANGENT 的真实路径点使用所在路段的切线 yaw。起点 marker 保留
            # 接收目标时的机器人 yaw，终点 marker 保存最后一段的切线 yaw；
            # MPPI 会先停车对齐第一段切线，再开始 TRACK。
            self.active_path_yaw = math.atan2(
                self.active_route_points[1].y - self.active_route_points[0].y,
                self.active_route_points[1].x - self.active_route_points[0].x,
            )
            final_tangent_yaw = math.atan2(
                self.active_route_points[-1].y
                - self.active_route_points[-2].y,
                self.active_route_points[-1].x
                - self.active_route_points[-2].x,
            )
            path = self._build_tangent_mode_path(
                self.active_route_points,
                start_yaw=pose.yaw,
                goal_yaw=final_tangent_yaw,
            )
            initial_rotation = abs(normalize_angle(
                self.active_path_yaw - pose.yaw))
            yaw_description = (
                f"切线 yaw={self.active_path_yaw:.3f}rad，"
                f"预计起步旋转={initial_rotation:.3f}rad")
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id

        self.phase = "sending_goal"
        try:
            future = self.follow_path_client.send_goal_async(goal)
        except Exception as error:
            self._fail(f"发送 FollowPath 目标失败：{error}")
            return
        future.add_done_callback(self._goal_response_callback)
        self.get_logger().info(
            f"第 {self.completed_runs + 1}/{self.loop_count} 次单程，"
            f"第 {self.active_segment_index + 1}/"
            f"{self.active_segment_count} 段："
            f"发送 {self.active_from_id} -> {self.active_to_id}，"
            f"模式={self.path_control_mode.upper()}，"
            f"路径点={len(path.poses)}，"
            f"{yaw_description}"
        )

    def _make_pose(
        self, x: float, y: float, yaw: float, stamp
    ) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = stamp
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def _interpolate_route(self, route_points: List[Point2D]) -> List[Point2D]:
        """按 path_resolution_m 对连续路网逐段插值，不重复中间导航点。"""
        if len(route_points) < 2:
            raise ValueError("连续路网至少需要两个端点")
        interpolated = [route_points[0]]
        for segment_index in range(len(route_points) - 1):
            start = route_points[segment_index]
            end = route_points[segment_index + 1]
            line_length = distance(start, end)
            if line_length <= 1.0e-9:
                raise ValueError(
                    f"连续路网第 {segment_index + 1} 段两端重合")
            segment_count = max(
                1, int(math.ceil(line_length / self.path_resolution_m)))
            for index in range(1, segment_count + 1):
                ratio = index / segment_count
                interpolated.append(Point2D(
                    start.x + ratio * (end.x - start.x),
                    start.y + ratio * (end.y - start.y),
                ))
        return interpolated

    def _build_start_mode_path(
        self, route_points: List[Point2D], fixed_yaw: float
    ) -> NavPath:
        stamp = self.get_clock().now().to_msg()
        path = NavPath()
        path.header.frame_id = self.map_frame
        path.header.stamp = stamp

        interpolated = self._interpolate_route(route_points)
        start = interpolated[0]

        # 两个额外 marker + 后续真实 P0 = 三个连续同位置点。
        path.poses.append(
            self._make_pose(start.x, start.y, fixed_yaw, stamp))
        path.poses.append(
            self._make_pose(start.x, start.y, fixed_yaw, stamp))
        for point in interpolated:
            path.poses.append(
                self._make_pose(point.x, point.y, fixed_yaw, stamp))
        return path

    def _build_tangent_mode_path(
        self,
        route_points: List[Point2D],
        start_yaw: float,
        goal_yaw: float,
    ) -> NavPath:
        """按 MPPI 的 TANGENT 首尾 marker 协议构建连续折线路径。

        返回顺序为：起点 marker、真实 P0...Pn、终点 marker。起点 marker 与
        P0 同位置但 yaw 可不同，触发起点切线对齐；Pn 与终点 marker 同位置，
        后者的 yaw 只作为最终 goal yaw。每个真实路径点使用下一有效段的切线
        yaw，最后一点沿用最后一段切线。
        """
        stamp = self.get_clock().now().to_msg()
        path = NavPath()
        path.header.frame_id = self.map_frame
        path.header.stamp = stamp

        interpolated = self._interpolate_route(route_points)
        tangent_yaws: List[float] = []
        for index in range(len(interpolated) - 1):
            start = interpolated[index]
            end = interpolated[index + 1]
            tangent_yaws.append(math.atan2(
                end.y - start.y, end.x - start.x))
        tangent_yaws.append(tangent_yaws[-1])

        # 一个起点 marker；后面的 index=0 是同位置的真实 P0。
        start = interpolated[0]
        path.poses.append(
            self._make_pose(start.x, start.y, start_yaw, stamp))
        for point, tangent_yaw in zip(interpolated, tangent_yaws):
            path.poses.append(
                self._make_pose(point.x, point.y, tangent_yaw, stamp))

        # 一个终点 marker；与真实 Pn 同位置，构成 TANGENT 的终点协议。
        end = interpolated[-1]
        path.poses.append(self._make_pose(end.x, end.y, goal_yaw, stamp))
        return path

    def _goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            self._fail(f"发送 FollowPath 目标异常：{error}")
            return
        if goal_handle is None or not goal_handle.accepted:
            self._fail("FollowPath 拒绝了连续路网路径目标")
            return

        self.active_goal_handle = goal_handle
        self.phase = "running"
        self.segment_started_sec = self._now_sec()
        if not self.run_motion_started:
            self.run_started_sec = self.segment_started_sec
            self.run_motion_started = True
        self._sample_robot_pose()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _path_reference_yaw(self, pose: Pose2D) -> float:
        """返回当前模式在机器人最近路段上的参考 yaw。"""
        if self.path_control_mode == "start":
            return self.active_path_yaw
        _, tangent_yaw = closest_polyline_segment(
            Point2D(pose.x, pose.y), self.active_route_points)
        return tangent_yaw

    def _sample_robot_pose(self) -> None:
        pose = self._lookup_robot_pose()
        if pose is None:
            return
        sample_sec = self._now_sec()
        self.trajectory.append(pose)
        route_error, _ = closest_polyline_segment(
            Point2D(pose.x, pose.y), self.active_route_points)
        self.max_cross_track_error_m = max(
            self.max_cross_track_error_m,
            route_error,
        )
        reference_yaw = self._path_reference_yaw(pose)
        self.yaw_samples.append(YawSample(
            elapsed_sec=max(0.0, sample_sec - self.run_started_sec),
            yaw_error_rad=normalize_angle(pose.yaw - reference_yaw),
        ))
        if self.last_speed_pose is None or self.last_speed_sample_sec is None:
            self.speed_samples.append(SpeedSample(0.0, 0.0))
        else:
            delta_sec = sample_sec - self.last_speed_sample_sec
            if delta_sec > 1.0e-6:
                displacement_m = math.hypot(
                    pose.x - self.last_speed_pose.x,
                    pose.y - self.last_speed_pose.y,
                )
                self.travel_distance_m += displacement_m
                self.speed_samples.append(SpeedSample(
                    elapsed_sec=max(0.0, sample_sec - self.run_started_sec),
                    speed_mps=displacement_m / delta_sec,
                ))
        self.last_speed_pose = pose
        self.last_speed_sample_sec = sample_sec

    def _save_run_plots(
        self, run: RunSpeedSeries, yaw_samples: List[YawSample]
    ) -> None:
        """为一个单程分别生成速度曲线和路径 yaw 偏移曲线。"""
        if not run.samples and not yaw_samples:
            return
        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            from matplotlib import font_manager, ft2font
        except Exception as error:
            self.get_logger().error(f"无法加载 matplotlib，未生成速度图：{error}")
            return

        # 不能只写字体 family 名称：目标机器上的 Matplotlib 字体缓存可能
        # 把它回退到不含中文字形的 DejaVu Sans。这里直接找到字体文件并
        # 逐字检查中文字形，再通过 FontProperties(fname=...) 强制加载。
        required_plot_chars = (
            "直线路网循环测试速度曲线时间第次偏航角移"
            "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
            "（）()m/s.-_ "
        )
        font_candidates: List[str] = []
        if self.chinese_font_path:
            configured_path = str(
                Path(self.chinese_font_path).expanduser().resolve())
            font_candidates.append(configured_path)
        system_fonts = font_manager.findSystemFonts()
        system_fonts.sort(key=lambda path: (
            0 if "notosanscjk" in path.lower() else
            1 if "wenquanyi" in path.lower() else
            2 if "simhei" in path.lower() else
            3 if "ukai" in path.lower() else 4,
            path,
        ))
        font_candidates.extend(system_fonts)

        chinese_font_path: Optional[str] = None
        checked_paths = set()
        for candidate_path in font_candidates:
            if candidate_path in checked_paths:
                continue
            checked_paths.add(candidate_path)
            try:
                font = ft2font.FT2Font(candidate_path)
                if all(
                    font.get_char_index(ord(character)) != 0
                    for character in required_plot_chars
                ):
                    chinese_font_path = candidate_path
                    break
            except (OSError, RuntimeError):
                continue

        chinese_font = None
        if chinese_font_path is not None:
            font_manager.fontManager.addfont(chinese_font_path)
            chinese_font = font_manager.FontProperties(
                fname=chinese_font_path)
        else:
            self.get_logger().error(
                "系统中未找到包含中文字形的字体，速度图中文可能显示为方框；"
                "请在 SCRIPT_CONFIG 的 chinese_font_path 中填写中文字体文件路径")

        matplotlib.rcParams["axes.unicode_minus"] = False

        def save_curve(
            elapsed_values: List[float],
            data_values: List[float],
            title: str,
            y_label: str,
            output_path: Path,
            draw_zero_line: bool = False,
        ) -> None:
            figure, axes = plt.subplots(figsize=(10, 6))
            axes.plot(elapsed_values, data_values, linewidth=1.5)
            if draw_zero_line:
                axes.axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
            axes.set_title(title, fontproperties=chinese_font)
            axes.set_xlabel("时间（s）", fontproperties=chinese_font)
            axes.set_ylabel(y_label, fontproperties=chinese_font)
            axes.grid(True, linestyle="--", alpha=0.4)
            figure.tight_layout()
            try:
                figure.savefig(output_path, dpi=160)
                self.get_logger().info(f"已保存曲线图：{output_path}")
            except Exception as error:
                self.get_logger().error(f"保存曲线图失败：{error}")
            finally:
                plt.close(figure)

        if run.samples:
            save_curve(
                [sample.elapsed_sec for sample in run.samples],
                [sample.speed_mps for sample in run.samples],
                f"第{run.run_index}次速度曲线",
                "速度（m/s）",
                self.output_dir / f"第{run.run_index:03d}次_速度曲线.png",
            )
        if yaw_samples:
            save_curve(
                [sample.elapsed_sec for sample in yaw_samples],
                [sample.yaw_error_rad for sample in yaw_samples],
                f"第{run.run_index}次偏航角偏移曲线",
                "偏航角偏移（rad）",
                self.output_dir / f"第{run.run_index:03d}次_Yaw偏移曲线.png",
                draw_zero_line=True,
            )

    def _write_abort_diagnostics(
        self, status: int, action_result, final_pose: Optional[Pose2D]
    ) -> None:
        """把 Action 状态、姿态偏移和相关 rosout 信息写入中文诊断文件。"""
        diagnostic_path = self.output_dir / "中止原因.txt"
        error_code = getattr(action_result, "error_code", None)
        error_message = str(
            getattr(action_result, "error_msg", "") or "").strip()
        max_abs_yaw_error = max(
            (abs(sample.yaw_error_rad) for sample in self.yaw_samples),
            default=math.nan,
        )
        final_yaw = final_pose.yaw if final_pose is not None else math.nan
        final_reference_yaw = (
            self._path_reference_yaw(final_pose)
            if final_pose is not None else math.nan
        )
        final_yaw_error = (
            normalize_angle(final_yaw - final_reference_yaw)
            if final_pose is not None else math.nan
        )

        lines = [
            f"第 {self.completed_runs + 1} 次单程："
            f"{self.run_from_id} -> {self.run_to_id}",
            f"中止路段：第 {self.active_segment_index + 1}/"
            f"{self.active_segment_count} 段，"
            f"{self.active_from_id} -> {self.active_to_id}",
            f"路径控制模式：{self.path_control_mode.upper()}",
            f"FollowPath 状态：{self._goal_status_text(status)}（{status}）",
            f"路径起步参考 Yaw：{self.active_path_yaw:.6f} rad",
            f"最终所在路段参考 Yaw：{final_reference_yaw:.6f} rad",
            f"最终实际 Yaw：{final_yaw:.6f} rad",
            f"最终 Yaw 偏移：{final_yaw_error:.6f} rad",
            f"最大绝对 Yaw 偏移：{max_abs_yaw_error:.6f} rad",
        ]
        if error_code is not None:
            lines.append(f"FollowPath 错误码：{error_code}")
        if error_message:
            lines.append(f"FollowPath 错误信息：{error_message}")
        else:
            lines.append(
                "FollowPath 错误信息：当前 ROS 2 Humble 的 FollowPath "
                "Result 是空消息，本身不返回详细中止原因。")

        if self.active_diagnostic_logs:
            lines.append("控制器相关警告/错误：")
            lines.extend(
                f"  {message}" for message in self.active_diagnostic_logs)
        else:
            lines.append(
                "控制器相关警告/错误：脚本未从 /rosout 捕获到相关消息；"
                "请同时查看 controller_server 所在终端。")
        lines.append("")

        try:
            with diagnostic_path.open("a", encoding="utf-8") as stream:
                stream.write("\n".join(lines))
        except OSError as error:
            self.get_logger().error(f"保存中止诊断失败：{error}")
            return
        self.get_logger().error(f"中止诊断已保存：{diagnostic_path}")

    def _goal_result_callback(self, future) -> None:
        self._sample_robot_pose()
        try:
            wrapped_result = future.result()
            status = int(wrapped_result.status)
            action_result = wrapped_result.result
        except Exception as error:
            self._fail(f"读取 FollowPath 结果异常：{error}")
            return

        self.active_goal_handle = None
        now_sec = self._now_sec()
        segment_duration_sec = max(
            0.0, now_sec - self.segment_started_sec)
        final_pose = self._lookup_robot_pose()
        end_error = (
            distance(Point2D(final_pose.x, final_pose.y), self.active_end)
            if final_pose is not None
            else math.nan
        )
        status_text = self._goal_status_text(status)
        run_index = self.completed_runs + 1
        segment_summary = (
            f"第 {run_index}/{self.loop_count} 次单程的第 "
            f"{self.active_segment_index + 1}/{self.active_segment_count} 段结束："
            f"{self.active_from_id} -> {self.active_to_id}，"
            f"状态={status_text}，终点位置误差={end_error:.4f}m，"
            f"耗时={segment_duration_sec:.3f}s"
        )
        # rclpy Humble 要求同一个日志调用点的 severity 固定。
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(segment_summary)
        else:
            self.get_logger().error(segment_summary)

        if status != GoalStatus.STATUS_SUCCEEDED:
            self._save_run_result(status, final_pose)
            self._write_abort_diagnostics(status, action_result, final_pose)
            self._fail(
                f"第 {run_index} 次单程的第 "
                f"{self.active_segment_index + 1}/"
                f"{self.active_segment_count} 段 FollowPath 未成功，"
                f"停止后续循环；状态={status_text}")
            return

        if self.active_segment_index + 1 < self.active_segment_count:
            reached_id = self.active_to_id
            if final_pose is None:
                x_error_m = math.nan
                y_error_m = math.nan
                xy_error_m = math.nan
            else:
                # 使用“实际位置 - 路网目标位置”的有符号误差。
                x_error_m = final_pose.x - self.active_end.x
                y_error_m = final_pose.y - self.active_end.y
                xy_error_m = math.hypot(x_error_m, y_error_m)
            self.intermediate_point_errors.append(IntermediatePointError(
                point_id=reached_id,
                x_error_m=x_error_m,
                y_error_m=y_error_m,
                xy_error_m=xy_error_m,
            ))
            self.active_segment_index += 1
            self.phase = "segment_pause"
            self.segment_pause_until_sec = now_sec + self.segment_pause_sec
            self.get_logger().info(
                f"已到达中间导航点 {reached_id} 并结束当前 FollowPath；"
                f"X/Y/XY 误差={x_error_m:.4f}/"
                f"{y_error_m:.4f}/{xy_error_m:.4f}m；"
                f"停车 {self.segment_pause_sec:.3f}s 后再执行第 "
                f"{self.active_segment_index + 1}/"
                f"{self.active_segment_count} 段"
            )
            return

        self._save_run_result(status, final_pose)
        self.completed_runs += 1
        self.next_route_forward = not self.active_route_forward
        if self.completed_runs >= self.loop_count:
            self._complete()
            return

        self.phase = "pause"
        self.pause_until_sec = now_sec + self.loop_pause_sec

    def _save_run_result(
        self, status: int, final_pose: Optional[Pose2D]
    ) -> None:
        """保存一条完整边链单程；失败时保存已执行部分。"""
        duration_sec = (
            max(0.0, self._now_sec() - self.run_started_sec)
            if self.run_motion_started else 0.0
        )
        end_error = (
            distance(Point2D(final_pose.x, final_pose.y), self.active_end)
            if final_pose is not None else math.nan
        )
        status_text = self._goal_status_text(status)
        run_index = self.completed_runs + 1

        def format_intermediate_errors(attribute: str) -> str:
            return ";".join(
                f"{error.point_id}={getattr(error, attribute):.6f}"
                for error in self.intermediate_point_errors
            )

        speed_run = RunSpeedSeries(
            run_index=run_index,
            from_id=self.run_from_id,
            to_id=self.run_to_id,
            samples=list(self.speed_samples),
        )
        self._save_run_plots(speed_run, list(self.yaw_samples))

        self.csv_writer.writerow({
            "单程序号": run_index,
            "起点ID": self.run_from_id,
            "终点ID": self.run_to_id,
            "状态": status_text,
            "行驶耗时_秒": f"{duration_sec:.6f}",
            "预对舵耗时_秒": f"{self.run_presteer_elapsed_sec:.6f}",
            "预对舵最终最大误差_弧度": (
                f"{self.run_presteer_max_error_rad:.6f}"),
            "采样点数": len(self.trajectory),
            "实际路程长度_米": f"{self.travel_distance_m:.6f}",
            "最大轨迹横向偏差_米": (
                f"{self.max_cross_track_error_m:.6f}"),
            "中间点X误差_米": format_intermediate_errors("x_error_m"),
            "中间点Y误差_米": format_intermediate_errors("y_error_m"),
            "中间点XY误差_米": format_intermediate_errors("xy_error_m"),
            "终点位置误差_米": f"{end_error:.6f}",
        })
        self.csv_stream.flush()

        run_summary = (
            f"第 {run_index}/{self.loop_count} 次结果："
            f"{self.run_from_id} -> {self.run_to_id}，"
            f"已执行路段={self.active_segment_index + 1}/"
            f"{self.active_segment_count}，"
            f"状态={status_text}，"
            f"实际路程={self.travel_distance_m:.4f}m，"
            f"最大轨迹横向偏差={self.max_cross_track_error_m:.4f}m，"
            f"终点位置误差={end_error:.4f}m，"
            f"预对舵总耗时={self.run_presteer_elapsed_sec:.3f}s，"
            f"耗时={duration_sec:.3f}s，样本={len(self.trajectory)}"
        )
        # ROS 2 Humble 的 rclpy 会缓存每个日志调用点的 severity；同一行先调用
        # info、后调用 error 会抛出“Logger severity cannot be changed”。成功和
        # 失败必须使用两个固定且彼此独立的日志调用点。
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(run_summary)
        else:
            self.get_logger().error(run_summary)

    @staticmethod
    def _goal_status_text(status: int) -> str:
        names = {
            GoalStatus.STATUS_UNKNOWN: "未知",
            GoalStatus.STATUS_ACCEPTED: "已接受",
            GoalStatus.STATUS_EXECUTING: "执行中",
            GoalStatus.STATUS_CANCELING: "正在取消",
            GoalStatus.STATUS_SUCCEEDED: "成功",
            GoalStatus.STATUS_CANCELED: "已取消",
            GoalStatus.STATUS_ABORTED: "已中止",
        }
        return names.get(status, f"状态码_{status}")

    def _complete(self) -> None:
        self.phase = "done"
        self.success = True
        self.finished = True
        self.get_logger().info(
            f"循环测试完成：成功单程={self.completed_runs}，"
            f"结果目录={self.output_dir}"
        )

    def _cancel_presteer_goal(self, reason: str) -> None:
        """取消本脚本持有的预对舵目标，并使迟到的异步回调失效。"""
        self.presteer_request_id += 1
        goal_handle = self.presteer_goal_handle
        self.presteer_goal_handle = None
        if goal_handle is None:
            return
        self.get_logger().warning(f"正在取消静止预对舵：{reason}")
        try:
            goal_handle.cancel_goal_async()
        except Exception as error:
            self.get_logger().error(f"取消静止预对舵失败：{error}")

    def _fail(self, reason: str) -> None:
        if self.finished:
            return
        self._cancel_presteer_goal(reason)
        if self.active_goal_handle is not None:
            try:
                self.active_goal_handle.cancel_goal_async()
            except Exception as error:
                self.get_logger().error(f"取消 FollowPath 失败：{error}")
            self.active_goal_handle = None
        self.phase = "failed"
        self.success = False
        self.finished = True
        self.get_logger().error(
            f"循环测试停止：{reason}；"
            f"已完成单程={self.completed_runs}；"
            f"结果 CSV={self.output_csv_path}"
        )

    def request_stop(self) -> None:
        """Ctrl-C 时取消预对舵和 FollowPath，确保不再继续测试。"""
        self._cancel_presteer_goal("用户中止测试")
        if self.active_goal_handle is not None:
            self.get_logger().warning("正在取消当前 FollowPath 测试目标")
            try:
                self.active_goal_handle.cancel_goal_async()
            except Exception as error:
                self.get_logger().error(f"取消 FollowPath 失败：{error}")
        self.finished = True
        self.success = False

    def close(self) -> None:
        if not self.csv_stream.closed:
            self.csv_stream.flush()
            self.csv_stream.close()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node: Optional[StraightRouteLoopTest] = None
    try:
        node = StraightRouteLoopTest()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        if node is not None:
            node.request_stop()
            # 给取消请求一个短暂的通信窗口。
            for _ in range(10):
                if not rclpy.ok():
                    break
                rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
