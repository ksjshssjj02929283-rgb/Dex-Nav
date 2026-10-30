#!/usr/bin/env python3
# ==============================================================================
# DEXE Mobile Navigation - 原生 ROS 2 仿真桥接节点 (类似 Gazebo 仿真引擎与桥)
# 职责:
#   1. 提供 /map_asset_manager/get_assets (nav_interfaces/srv/GetAssets) 服务，
#      将手绘/前端路网无缝提供给原生 nav2_route_planner (RouteGraphLoader)
#   2. 接入原生 planner_server 的 /compute_path_to_pose Action，驱动原生全局规划
#   3. 模拟全向虚拟雷达 (/laser_scan/front) 与基础参考地图 (/map)，
#      使原生 global_costmap 与 local_costmap 自动叠加障碍物并进行代价膨胀
#   4. 桥接 FollowPath Action 到 controller_server (RPPController + FrenetLocalPlanner)
# ==============================================================================

import json
import math
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav2_msgs.action import ComputePathToPose, FollowPath
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from nav_interfaces.msg import NavPoint, RouteEdge, RouteEndpoint, RouteJunction
from nav_interfaces.srv import GetAssets


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class RoutePlannerActionBridge(Node):
    def __init__(self):
        super().__init__('route_planner_action_bridge')
        self.cb_group = ReentrantCallbackGroup()

        # W1 底盘参数 (对齐 w1_diff_sim.urdf)
        self.wheel_separation = 0.5078  # 轮距 [m]

        # 内部状态
        self.active_follow_goal = None
        self.active_plan_goal = None
        self.is_navigating = False
        self.goal_reached = False
        self.current_path = []  # List[Tuple[x, y, s, yaw]]
        self.total_path_s = 0.0

        self.robot_x = 3.50
        self.robot_y = 15.625
        self.robot_yaw = -0.35
        self.current_v = 0.0
        self.current_omega = 0.0
        self.avoidance_mode = "PATH_FOLLOW_RPP"

        # 障碍物配置
        self.obstacle_active = False
        self.obstacle_x = 0.0
        self.obstacle_y = 0.0
        self.obstacle_radius = 0.3
        self.obstacles_list = []

        # 路网数据 (内存缓存，默认加载 custom_route_network.json 若存在)
        self.road_nodes = {}
        self.road_edges = []
        self._load_default_network()

        # 1. 提供 /map_asset_manager/get_assets 服务 (供原生 nav2_route_planner 自动加载路网)
        self.asset_service = self.create_service(
            GetAssets,
            '/map_asset_manager/get_assets',
            self._handle_get_assets,
            callback_group=self.cb_group
        )
        self.get_logger().info('✅ 原生路网服务就绪: /map_asset_manager/get_assets')

        # 2. 原生 Action 客户端: PlannerServer (/compute_path_to_pose) 与 ControllerServer (/follow_path)
        self._planner_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose', callback_group=self.cb_group)
        self._follow_client = ActionClient(self, FollowPath, 'follow_path', callback_group=self.cb_group)

        # 3. 订阅前端或外部下发的话题 (使用普通 QoS 10，完全兼容网页端与外部节点)
        qos_transient = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(Path, '/route_planner/global_path', self._path_callback, 10)
        self.create_subscription(Path, '/plan', self._path_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_pose_callback, 10)
        self.create_subscription(String, '/simulation/road_network_json', self._road_network_json_callback, 10)
        self.create_subscription(PointStamped, '/simulation/obstacle', self._obstacle_callback, 10)
        self.create_subscription(String, '/simulation/obstacles_json', self._obstacles_json_callback, 10)
        self.create_subscription(Odometry, '/feedback/odom', self._odom_callback, 10)
        self.create_subscription(Twist, '/control/cmd_vel_nav', self._cmd_vel_callback, 10)
        self.create_subscription(String, '/controller_server/avoidance_mode', self._avoidance_mode_callback, 10)

        # 4. 发布器：遥测状态、全向虚拟雷达、以及满足 static_layer 的虚拟 /map
        self.fsm_pub = self.create_publisher(String, '/controller_server/fsm_state', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/laser_scan/front', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos_transient)

        # 5. 定时器
        self.create_timer(0.05, self._telemetry_timer_callback)  # 20Hz 遥测
        self.create_timer(0.10, self._scan_timer_callback)       # 10Hz 虚拟雷达
        self.create_timer(2.0, self._publish_static_map)         # 0.5Hz /map 维持
        self._publish_static_map()

        self.get_logger().info('✅ 原生仿真环境与桥接节点 (RoutePlannerActionBridge) 已就绪！')

    def _load_default_network(self):
        json_path = '/home/dex/Desktop/work/kuawei/dexe_mobile_nav/docs/custom_route_network.json'
        if os.path.exists(json_path):
            try:
                with open(json_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    self._parse_network_data(data)
                self.get_logger().info(f'从 {json_path} 加载了 {len(self.road_nodes)} 个节点, {len(self.road_edges)} 条车道。')
            except Exception as e:
                self.get_logger().warn(f'读取默认路网失败: {e}')

    def _parse_network_data(self, data: dict):
        nodes_dict = {}
        canvas_h = data.get("canvas_size", {}).get("height", 680.0)

        # 解析节点
        raw_nodes = data.get("nodes", {})
        for k, n in raw_nodes.items():
            if "metric" in n:
                x = float(n["metric"]["x"])
                y = float(n["metric"]["y"])
            elif "pixel" in n:
                x = float(n["pixel"]["x"]) / 40.0
                y = (canvas_h - float(n["pixel"]["y"])) / 40.0
            else:
                x = float(n.get("x", 0.0)) / 40.0
                y = (canvas_h - float(n.get("y", 0.0))) / 40.0
            yaw = float(n.get("yaw", 0.0))
            nodes_dict[k] = {"id": k, "x": x, "y": y, "yaw": yaw}

        # 解析边
        edges_list = []
        raw_edges = data.get("edges", [])
        for e in raw_edges:
            eid = e.get("id", f"{e.get('from')}_{e.get('to')}")
            etype = e.get("type", "line").upper()
            bidir = bool(e.get("bidirectional", False))
            ctrl_pts = []
            if etype == "BEZIER":
                if "ctrl_metric" in e:
                    cx = float(e["ctrl_metric"]["x"])
                    cy = float(e["ctrl_metric"]["y"])
                    ctrl_pts.append((cx, cy))
                elif "ctrl" in e:
                    cx = float(e["ctrl"]["x"]) / 40.0 if float(e["ctrl"]["x"]) > 100 else float(e["ctrl"]["x"])
                    cy = (canvas_h - float(e["ctrl"]["y"])) / 40.0 if float(e["ctrl"]["y"]) > 100 else float(e["ctrl"]["y"])
                    ctrl_pts.append((cx, cy))
            edges_list.append({
                "id": eid,
                "from": e.get("from"),
                "to": e.get("to"),
                "type": etype,
                "bidirectional": bidir,
                "control_points": ctrl_pts
            })

        self.road_nodes = nodes_dict
        self.road_edges = edges_list

    def _road_network_json_callback(self, msg: String):
        """前端同步路网更新"""
        try:
            data = json.loads(msg.data)
            self._parse_network_data(data)
            self.get_logger().info(f'🔄 收到新路网: {len(self.road_nodes)} 节点, {len(self.road_edges)} 车道')
        except Exception as e:
            self.get_logger().warn(f'解析 /simulation/road_network_json 失败: {e}')

    def _handle_get_assets(self, request: GetAssets.Request, response: GetAssets.Response):
        """为原生 nav2_route_planner (RouteGraphLoader) 提供路网数据，无需任何 C++ 接口修改"""
        response.success = True
        response.error_code = 0
        response.message = "OK from simulation bridge"
        response.map_name = request.map_name if request.map_name else "sim_map"
        response.is_current = True

        # 构建 points (点位)
        for nid, ndata in self.road_nodes.items():
            np = NavPoint()
            np.name = nid
            np.type = "STATION"
            np.x = float(ndata["x"])
            np.y = float(ndata["y"])
            np.theta = float(ndata["yaw"])
            np.tolerance = 0.1
            response.points.append(np)

        # 构建 edges (车道)
        for edata in self.road_edges:
            re = RouteEdge()
            re.id = edata["id"]
            re.from_ep.id = edata["from"]
            re.from_ep.type = "nav"
            re.to_ep.id = edata["to"]
            re.to_ep.type = "nav"
            re.edge_type = edata["type"]
            re.bidirectional = edata["bidirectional"]
            re.lane_width = 1.2
            re.max_speed = 0.8

            for cpx, cpy in edata.get("control_points", []):
                pt = Point()
                pt.x = float(cpx)
                pt.y = float(cpy)
                pt.z = 0.0
                re.control_points.append(pt)

            response.edges.append(re)

        self.get_logger().info(f'Serving get_assets: {len(response.points)} points, {len(response.edges)} edges to RouteGraphLoader')
        return response

    def _goal_pose_callback(self, msg: PoseStamped):
        """接收到目标点时，自动调用原生 PlannerServer 的 /compute_path_to_pose Action"""
        self.get_logger().info(f'收到目标点: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})，请求原生 PlannerServer 规划...')
        if not self._planner_client.server_is_ready():
            self._planner_client.wait_for_server(timeout_sec=2.0)

        plan_goal = ComputePathToPose.Goal()
        plan_goal.goal = msg
        plan_goal.planner_id = 'RouteGridPlanner'
        plan_goal.use_start = False  # 使用当前小车位姿

        future = self._planner_client.send_goal_async(plan_goal)
        future.add_done_callback(self._plan_goal_response_callback)

    def _plan_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ PlannerServer 拒绝了规划请求！')
            return
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self._plan_result_callback)

    def _plan_result_callback(self, future):
        result = future.result().result
        path = result.path
        self.get_logger().info(f'🎯 原生 PlannerServer 规划完成！输出路径点数: {len(path.poses)}')
        # 自动触发控制器跟踪
        if len(path.poses) >= 2:
            self._path_callback(path)

    def _publish_static_map(self):
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = 0.05
        grid.info.width = 1000
        grid.info.height = 1000
        grid.info.origin.position.x = -10.0
        grid.info.origin.position.y = -10.0
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (grid.info.width * grid.info.height)
        self.map_pub.publish(grid)

    def _path_callback(self, msg: Path):
        if len(msg.poses) < 2:
            if self.is_navigating or self.active_follow_goal is not None:
                self._cancel_active_goal()
                self.current_path = []
                self.total_path_s = 0.0
                self.is_navigating = False
                self.goal_reached = False
            return

        self.current_path = []
        accum_s = 0.0
        for i, pose_stamped in enumerate(msg.poses):
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            yaw = quat_to_yaw(pose_stamped.pose.orientation)
            if i > 0:
                prev_x, prev_y, _, _ = self.current_path[-1]
                accum_s += math.hypot(px - prev_x, py - prev_y)
            self.current_path.append((px, py, accum_s, yaw))
        self.total_path_s = accum_s

        self.get_logger().info(
            f'📥 下发路径到 controller_server: 共 {len(msg.poses)} 点，总长 {self.total_path_s:.2f} m。'
        )

        if not self._follow_client.server_is_ready():
            if not self._follow_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error('FollowPath Action Server 连接超时！')
                return

        goal_msg = FollowPath.Goal()
        goal_msg.path = msg
        goal_msg.controller_id = 'RPPController'
        goal_msg.goal_checker_id = 'high_precision_goal_checker'

        self.is_navigating = True
        self.goal_reached = False
        send_future = self._follow_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._follow_goal_response_callback)

    def _cancel_active_goal(self):
        if self.active_follow_goal is not None:
            try:
                self.active_follow_goal.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f'取消 Goal 异常: {e}')
            self.active_follow_goal = None

    def _follow_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ controller_server 拒绝了 FollowPath！')
            self.is_navigating = False
            return
        self.active_follow_goal = goal_handle
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self._follow_result_callback)

    def _follow_result_callback(self, future):
        status = future.result().status
        self.is_navigating = False
        self.active_follow_goal = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.goal_reached = True
            self.get_logger().info('🏁 FollowPath 执行成功，小车精准停靠！')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('⏹️ FollowPath 任务已取消。')
        else:
            self.get_logger().warn(f'⚠️ FollowPath 结束，状态码: {status}')

    def _odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = quat_to_yaw(msg.pose.pose.orientation)

    def _cmd_vel_callback(self, msg: Twist):
        self.current_v = msg.linear.x
        self.current_omega = msg.angular.z

    def _avoidance_mode_callback(self, msg: String):
        self.avoidance_mode = msg.data

    def _obstacle_callback(self, msg: PointStamped):
        self.obstacle_x = msg.point.x
        self.obstacle_y = msg.point.y
        radius = msg.point.z
        if radius > 0.0:
            self.obstacle_active = True
            self.obstacle_radius = min(0.60, radius)
        else:
            self.obstacle_active = False

    def _obstacles_json_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            obs_items = data.get("obstacles", []) if isinstance(data, dict) else data
            parsed = []
            for item in obs_items:
                if not item.get("active", True):
                    continue
                parsed.append({
                    "id": item.get("id", "obs"),
                    "shape": item.get("shape", "rectangle"),
                    "x": float(item.get("x", 0.0)),
                    "y": float(item.get("y", 0.0)),
                    "yaw": float(item.get("yaw", 0.0)),
                    "r": min(0.60, max(0.15, float(item.get("radius_eq", item.get("radius", 0.4))))),
                    "w": float(item.get("w", 1.15)),
                    "h": float(item.get("h", 0.90)),
                })
            self.obstacles_list = parsed
            if parsed:
                self.obstacle_active = True
                self.obstacle_x = parsed[0]["x"]
                self.obstacle_y = parsed[0]["y"]
                self.obstacle_radius = parsed[0]["r"]
            else:
                self.obstacle_active = False
        except Exception as e:
            self.get_logger().warn(f"解析 obstacles_json 失败: {e}")

    def _calculate_frenet_sd(self):
        if not self.current_path or len(self.current_path) < 2:
            return 0.0, 0.0, float('inf')
        min_dist_sq = float('inf')
        closest_idx = 0
        rx, ry = self.robot_x, self.robot_y
        for i, (px, py, s, yaw) in enumerate(self.current_path):
            d_sq = (px - rx) ** 2 + (py - ry) ** 2
            if d_sq < min_dist_sq:
                min_dist_sq = d_sq
                closest_idx = i
        cx, cy, cs, cyaw = self.current_path[closest_idx]
        nx = -math.sin(cyaw)
        ny = math.cos(cyaw)
        d = (rx - cx) * nx + (ry - cy) * ny
        gx, gy, _, _ = self.current_path[-1]
        dist_to_goal = math.hypot(rx - gx, ry - gy)
        return cs, d, dist_to_goal

    def _telemetry_timer_callback(self):
        s, d, dist_to_goal = self._calculate_frenet_sd()
        v = self.current_v
        w = self.current_omega
        vl = v - 0.5 * w * self.wheel_separation
        vr = v + 0.5 * w * self.wheel_separation
        is_spinning = (abs(w) > 0.15 and abs(v) < 0.05)

        if self.goal_reached or (dist_to_goal < 0.25 and abs(v) < 0.04 and self.total_path_s > 0.5):
            state = "GOAL_REACHED"
            state_desc = "已精准停靠终点工位 (Nav2 到站)"
        elif not self.is_navigating and self.total_path_s == 0.0:
            state = "IDLE"
            state_desc = "待命中 (IDLE)"
        elif is_spinning:
            state = "ROTATE_TO_PATH"
            state_desc = "差速原地旋转对齐参考路径"
        elif "AVOIDING" in self.avoidance_mode:
            state = self.avoidance_mode
            state_desc = f"局部避障动态绕行中 ({self.avoidance_mode})"
        elif self.avoidance_mode == "OBSTACLE_STOP" or (self.obstacle_active and abs(v) < 0.04 and self.is_navigating):
            state = "OBSTACLE_STOP"
            state_desc = "前方障碍物阻挡，安全停车等候中"
        else:
            state = "PATH_FOLLOW_RPP"
            state_desc = "纯追踪巡航 (RPP Tracking)"

        telemetry_dict = {
            "state": state,
            "state_desc": state_desc,
            "v": round(v, 3),
            "omega": round(w, 3),
            "v_left": round(vl, 3),
            "v_right": round(vr, 3),
            "s": round(s, 2),
            "d": round(d, 2),
            "total_s": round(self.total_path_s, 1),
            "is_spinning": is_spinning,
            "lookahead_dist": 0.60,
            "curvature": 0.0
        }
        msg = String()
        msg.data = json.dumps(telemetry_dict)
        self.fsm_pub.publish(msg)

    def _scan_timer_callback(self):
        """10Hz 生成多障碍物虚拟激光雷达扫描 (/laser_scan/front)，使 Costmap 自动叠加障碍物"""
        scan = LaserScan()
        scan.header.frame_id = 'base_link'
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.radians(1.0)
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.05
        scan.range_max = 12.0

        num_readings = int(round((scan.angle_max - scan.angle_min) / scan.angle_increment))
        ranges = [13.0] * num_readings

        active_targets = []
        if self.obstacles_list:
            for item in self.obstacles_list:
                active_targets.append((item["x"], item["y"], item["r"]))
        elif self.obstacle_active:
            active_targets.append((self.obstacle_x, self.obstacle_y, self.obstacle_radius))

        for ox, oy, orad in active_targets:
            dx = ox - self.robot_x
            dy = oy - self.robot_y
            local_x = dx * math.cos(-self.robot_yaw) - dy * math.sin(-self.robot_yaw)
            local_y = dx * math.sin(-self.robot_yaw) + dy * math.cos(-self.robot_yaw)
            dist_to_obs_center = math.hypot(local_x, local_y)

            dist_to_edge = max(0.06, dist_to_obs_center - orad)
            if dist_to_edge <= scan.range_max:
                obs_angle = math.atan2(local_y, local_x)
                half_fov = math.asin(min(0.98, orad / max(0.05, dist_to_obs_center)))
                start_angle = obs_angle - half_fov
                end_angle = obs_angle + half_fov

                start_idx = int(round((normalize_angle(start_angle) - scan.angle_min) / scan.angle_increment)) % num_readings
                end_idx = int(round((normalize_angle(end_angle) - scan.angle_min) / scan.angle_increment)) % num_readings

                curr_idx = start_idx
                step_count = 0
                while step_count < num_readings:
                    if dist_to_edge < ranges[curr_idx]:
                        ranges[curr_idx] = dist_to_edge
                    if curr_idx == end_idx:
                        break
                    curr_idx = (curr_idx + 1) % num_readings
                    step_count += 1

        scan.ranges = ranges
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = RoutePlannerActionBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
