#!/usr/bin/env python3
# Copyright 2026 DEXE Robot Engineer
# SPDX-License-Identifier: Apache-2.0

import math
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PointStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from nav2_msgs.action import FollowPath
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String

class DirectControllerTester(Node):
    def __init__(self):
        super().__init__('direct_controller_tester')
        self.odom = None
        self.fsm_state = "UNKNOWN"
        self.rollouts_received = 0
        self.latest_cmd_vel = Twist()
        
        self.create_subscription(Odometry, '/feedback/odom', self._odom_cb, 10)
        self.create_subscription(String, '/controller_server/fsm_state', self._fsm_cb, 10)
        self.create_subscription(Twist, '/control/cmd_vel_nav', self._cmd_cb, 10)
        self.create_subscription(MarkerArray, '/controller_server/frenet_rollouts', self._rollouts_cb, 10)
        
        self.obs_pub = self.create_publisher(PointStamped, '/simulation/obstacle', 10)
        self.obs_json_pub = self.create_publisher(String, '/simulation/obstacles_json', 10)
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.action_client = ActionClient(self, FollowPath, 'follow_path')

    def _odom_cb(self, msg):
        self.odom = msg

    def _fsm_cb(self, msg):
        self.fsm_state = msg.data

    def _cmd_cb(self, msg):
        self.latest_cmd_vel = msg

    def _rollouts_cb(self, msg):
        if msg.markers:
            self.rollouts_received += 1

def run():
    rclpy.init()
    tester = DirectControllerTester()
    print("Waiting for /feedback/odom and follow_path Action Server...")
    
    t0 = time.time()
    while tester.odom is None and time.time() - t0 < 5.0:
        rclpy.spin_once(tester, timeout_sec=0.1)

    if not tester.action_client.wait_for_server(timeout_sec=5.0):
        print("ERROR: follow_path server not available!")
        return

    # 重置小车到基准起点 (3.50, 15.625)
    init_msg = PoseWithCovarianceStamped()
    init_msg.header.frame_id = 'map'
    init_msg.header.stamp = tester.get_clock().now().to_msg()
    init_msg.pose.pose.position.x = 3.50
    init_msg.pose.pose.position.y = 15.625
    init_msg.pose.pose.orientation.w = 1.0
    for _ in range(10):
        tester.init_pose_pub.publish(init_msg)
        rclpy.spin_once(tester, timeout_sec=0.1)
    
    t_wait_reset = time.time()
    while time.time() - t_wait_reset < 3.0:
        rclpy.spin_once(tester, timeout_sec=0.1)
        if abs(tester.odom.pose.pose.position.x - 3.50) < 0.2:
            break
        tester.init_pose_pub.publish(init_msg)

    cur_x = tester.odom.pose.pose.position.x
    cur_y = tester.odom.pose.pose.position.y
    print(f"Robot successfully reset to: ({cur_x:.3f}, {cur_y:.3f})")
    print(f"Current robot pose: ({cur_x:.3f}, {cur_y:.3f})")

    # 1. 放置障碍物在车道中央 (8.00, 15.625) r=0.35m
    obs_x = 8.00
    obs_y = 15.625
    obs_r = 0.35
    print(f"Placing obstacle continuously at ({obs_x:.2f}, {obs_y:.2f}) r={obs_r}m")

    def publish_obs_now():
        p = PointStamped()
        p.header.frame_id = 'map'
        p.header.stamp = tester.get_clock().now().to_msg()
        p.point.x = obs_x
        p.point.y = obs_y
        p.point.z = obs_r
        tester.obs_pub.publish(p)
        tester.obs_json_pub.publish(String(data=f'{{"avoidance_enabled":true,"obstacles":[{{"id":"test_obs","shape":"circle","x":{obs_x},"y":{obs_y},"radius":{obs_r},"radius_eq":{obs_r},"active":true}}]}}'))

    # 持续预热发布 1 秒使雷达与 costmap 稳定吸收
    t_wait = time.time()
    while time.time() - t_wait < 1.0:
        publish_obs_now()
        rclpy.spin_once(tester, timeout_sec=0.05)

    # 2. 生成一条长 11 米的标准车道直线路径 (从 3.50 到 14.50)
    target_x = 14.50
    target_y = 15.625
    path_msg = Path()
    path_msg.header.frame_id = 'map'
    path_msg.header.stamp = tester.get_clock().now().to_msg()
    num_pts = 110
    for i in range(num_pts):
        alpha = i / (num_pts - 1)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = path_msg.header.stamp
        pose.pose.position.x = cur_x + alpha * (target_x - cur_x)
        pose.pose.position.y = cur_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        path_msg.poses.append(pose)

    goal_msg = FollowPath.Goal()
    goal_msg.path = path_msg
    goal_msg.controller_id = 'RPPController'
    goal_msg.goal_checker_id = 'high_precision_goal_checker'

    print("Sending FollowPath goal to controller_server...")
    send_goal_future = tester.action_client.send_goal_async(goal_msg)
    
    # 等待 Goal 被接受
    while not send_goal_future.done():
        rclpy.spin_once(tester, timeout_sec=0.05)
    
    goal_handle = send_goal_future.result()
    if not goal_handle.accepted:
        print("ERROR: Goal was rejected by controller_server!")
        return
    print("Goal accepted! Monitoring navigation...")

    get_result_future = goal_handle.get_result_async()
    
    start_t = time.time()
    last_print = 0
    max_lateral_offset = 0.0
    collision = False

    while not get_result_future.done() and (time.time() - start_t < 35.0):
        publish_obs_now()
        rclpy.spin_once(tester, timeout_sec=0.05)
        now = time.time()
        
        rx = tester.odom.pose.pose.position.x
        ry = tester.odom.pose.pose.position.y
        lat_offset = abs(ry - cur_y)
        if lat_offset > max_lateral_offset:
            max_lateral_offset = lat_offset
            
        dist_to_obs = math.hypot(rx - obs_x, ry - obs_y)
        if dist_to_obs < (obs_r + 0.30):
            collision = True

        if now - last_print > 1.0:
            last_print = now
            v = tester.latest_cmd_vel.linear.x
            w = tester.latest_cmd_vel.angular.z
            print(f"t={now-start_t:.1f}s | Pose=({rx:.2f}, {ry:.2f}) | cmd=(v={v:.2f}, w={w:.2f}) | FSM={tester.fsm_state} | rollouts_rx={tester.rollouts_received} | max_lat={max_lateral_offset:.2f}m")

    final_rx = tester.odom.pose.pose.position.x
    final_ry = tester.odom.pose.pose.position.y
    print("\n========== RESULT ==========")
    print(f"Start: ({cur_x:.2f}, {cur_y:.2f}) -> End: ({final_rx:.2f}, {final_ry:.2f})")
    print(f"Max lateral deviation: {max_lateral_offset:.3f} m")
    print(f"Collision with obstacle: {collision}")
    print(f"Rollout messages count: {tester.rollouts_received}")
    print(f"Final state: {tester.fsm_state}")
    print("============================\n")
    rclpy.shutdown()

if __name__ == '__main__':
    run()
