#!/usr/bin/env python3
"""测试用假机器人：提供 nbt pathnav / speed profile 依赖的接口，供 test_pathnav_fake.py 起子进程用。

  /navigate_through_graph_points   沿直线开到终点，到点前 5 mm 就上报成功，再低速蠕行 0.3 s 后停下；停点比目标纵向多 +2 mm
  /map_asset_manager/get_assets    点位 A(0,0,0°)、B(1,0,0°)、C(1,1,90°)，路网单向 A→B→C→A
  /map_asset_manager/resolve_nav_points  相邻点连直线段
  /schedule_planner/force_cancel   立即停车
  /path_follower_node、/velocity_smoother_path 两个参数节点；/feedback/odom 50 Hz；TF map→odom→base_link
  导航模式：/command_board/navigation_backend（锁存，FAKE_BACKEND，默认 path_navigation）；path_navigation 一套节点
  的 get_state 返回 active；/schedule_planner_node 的 mode 参数取 FAKE_SP_MODE（默认 independence）
"""
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import Point, TransformStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_interfaces.action import NavigateThroughGraphPoints
from nav_interfaces.msg import NavPoint, PathSegment, RouteEdge, RouteEndpoint
from nav_interfaces.srv import GetAssets, ResolveNavPoints
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

POINTS = {"A": (0.0, 0.0, 0.0), "B": (1.0, 0.0, 0.0), "C": (1.0, 1.0, math.pi / 2)}
EDGES = [("A", "B"), ("B", "C"), ("C", "A")]                      # 路网：单向三角形
SPEED, OVERSHOOT = 1.0, 0.002


class Fake(Node):
    def __init__(self):
        super().__init__("fake_robot")
        self.lock = threading.Lock()
        self.x = self.y = self.yaw = 0.0
        self.vx = self.wz = 0.0
        self.cancel = False
        cb = ReentrantCallbackGroup()
        self.tfb = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "/feedback/odom", 50)
        self.create_timer(0.02, self.tick, callback_group=cb)
        self.create_service(GetAssets, "/map_asset_manager/get_assets", self.assets, callback_group=cb)
        self.create_service(ResolveNavPoints, "/map_asset_manager/resolve_nav_points", self.resolve, callback_group=cb)
        self.create_service(Trigger, "/schedule_planner/force_cancel", self.force_cancel, callback_group=cb)
        ActionServer(self, NavigateThroughGraphPoints, "/navigate_through_graph_points", self.execute,
                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=cb)
        self.backend_pub = self.create_publisher(String, "/command_board/navigation_backend",
                                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.backend_pub.publish(String(data=os.environ.get("FAKE_BACKEND", "path_navigation")))
        for n in ("/schedule_planner_node", "/path_follower_node", "/velocity_smoother_path"):
            self.create_service(GetState, f"{n}/get_state", self.active_state, callback_group=cb)

    def active_state(self, req, res):
        res.current_state = State(id=State.PRIMARY_STATE_ACTIVE, label="active")
        return res

    def tick(self):
        now = self.get_clock().now().to_msg()
        with self.lock:
            x, y, yaw, vx, wz = self.x, self.y, self.yaw, self.vx, self.wz
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = now, "odom", "base_link"
        o.twist.twist.linear.x, o.twist.twist.angular.z = vx, wz
        self.odom_pub.publish(o)
        trs = []
        for parent, child, px, py, pw in (("map", "odom", 0.0, 0.0, 0.0), ("odom", "base_link", x, y, yaw)):
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = now, parent, child
            t.transform.translation.x, t.transform.translation.y = px, py
            t.transform.rotation.z, t.transform.rotation.w = math.sin(pw / 2), math.cos(pw / 2)
            trs.append(t)
        self.tfb.sendTransform(trs)

    def assets(self, req, res):
        res.success, res.map_name = True, "fake"
        for n, (x, y, th) in POINTS.items():
            res.points.append(NavPoint(name=n, type="STATION", x=x, y=y, theta=th))
        for i, (a, b) in enumerate(EDGES):
            res.edges.append(RouteEdge(id=f"e{i}", from_ep=RouteEndpoint(id=a, type="nav"),
                                       to_ep=RouteEndpoint(id=b, type="nav"), edge_type="LINE"))
        return res

    def resolve(self, req, res):
        names = list(req.nav_point_names)
        if any(n not in POINTS for n in names):
            res.success, res.message = False, "unknown point"
            return res
        for a, b in zip(names, names[1:]):
            s = PathSegment(type=0, target_yaw=POINTS[b][2])
            s.control_points = [Point(x=POINTS[a][0], y=POINTS[a][1]), Point(x=POINTS[b][0], y=POINTS[b][1])]
            res.segments.append(s)
        res.success = True
        return res

    def force_cancel(self, req, res):
        self.cancel = True
        res.success, res.message = True, "stopped"
        return res

    def set_motion(self, vx, wz):
        with self.lock:
            self.vx, self.wz = vx, wz

    def step_to(self, gx, gy, speed, dt=0.02):
        with self.lock:
            d = math.hypot(gx - self.x, gy - self.y)
            if d < 1e-9:
                return 0.0
            k = min(1.0, speed * dt / d)
            self.yaw = math.atan2(gy - self.y, gx - self.x) if d > 0.01 else self.yaw
            self.x += (gx - self.x) * k
            self.y += (gy - self.y) * k
            self.vx = speed if k < 1.0 else 0.0
            return math.hypot(gx - self.x, gy - self.y)

    def execute(self, gh):
        names = list(gh.request.nav_point_names)
        self.cancel = False
        fb = NavigateThroughGraphPoints.Feedback(total_points=len(names))
        gx, gy, gth = POINTS[names[-1]]
        with self.lock:
            hx, hy = gx - self.x, gy - self.y
        n = math.hypot(hx, hy) or 1.0
        ex, ey = gx + OVERSHOOT * hx / n, gy + OVERSHOOT * hy / n        # 沿行进方向多走 2 mm
        while True:
            if self.cancel or gh.is_cancel_requested:
                self.set_motion(0.0, 0.0)
                gh.canceled()
                return NavigateThroughGraphPoints.Result(success=False, message="canceled")
            remain = self.step_to(ex, ey, SPEED)
            fb.current_status = "tracking"
            gh.publish_feedback(fb)
            if remain < 0.005 + OVERSHOOT:
                break
            time.sleep(0.02)
        with self.lock:
            self.yaw = gth
        res = NavigateThroughGraphPoints.Result(success=True, message="ok")
        threading.Thread(target=self.creep, args=(ex, ey), daemon=True).start()
        gh.succeed()
        return res

    def creep(self, ex, ey):
        end = time.time() + 0.3
        while time.time() < end:
            self.step_to(ex, ey, 0.02)
            time.sleep(0.02)
        with self.lock:
            self.x, self.y = ex, ey
        self.set_motion(0.0, 0.0)


def main():
    rclpy.init()
    fake = Fake()
    follower = Node("path_follower_node")
    for n, v in (("max_linear_velocity", 0.7), ("max_linear_acceleration", 0.5), ("max_linear_deceleration", 0.4),
                 ("max_linear_jerk", 0.2)):
        follower.declare_parameter(n, v)
    smoother = Node("velocity_smoother_path")
    for n, v in (("max_velocity", [0.8, 0.8, 1.1]), ("min_velocity", [-0.8, -0.8, -1.1]),
                 ("max_accel", [0.6, 0.6, 0.8]), ("max_decel", [-0.9, -0.9, -1.1])):
        smoother.declare_parameter(n, v)
    planner = Node("schedule_planner_node")
    planner.declare_parameter("mode", os.environ.get("FAKE_SP_MODE", "independence"))
    ex = MultiThreadedExecutor(num_threads=6)
    for nd in (fake, follower, smoother, planner):
        ex.add_node(nd)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
