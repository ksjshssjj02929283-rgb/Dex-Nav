#!/usr/bin/env python3
"""Exercise an isolated controller_server without connecting its command outputs."""

import argparse
import math
import time

import rclpy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=25.0)
    parser.add_argument("--distance", type=float, default=2.0)
    args = parser.parse_args()
    rclpy.init()
    node = Node("control_timing_probe_client")
    odom = None

    def on_odom(msg):
        nonlocal odom
        odom = msg

    subscription = node.create_subscription(
        Odometry, "/feedback/odom", on_odom, qos_profile_sensor_data)
    client = ActionClient(node, FollowPath, "/control_probe/follow_path")
    deadline = time.monotonic() + 15.0
    while odom is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if odom is None:
        raise RuntimeError("No /feedback/odom sample")
    if not client.wait_for_server(timeout_sec=10.0):
        raise RuntimeError("Isolated FollowPath server unavailable")

    q = odom.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    path = Path()
    path.header.frame_id = odom.header.frame_id
    path.header.stamp = node.get_clock().now().to_msg()
    for index in range(81):
        arc = args.distance * index / 80.0
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = odom.pose.pose.position.x + arc * math.cos(yaw)
        pose.pose.position.y = odom.pose.pose.position.y + arc * math.sin(yaw)
        pose.pose.position.z = odom.pose.pose.position.z
        pose.pose.orientation = q
        path.poses.append(pose)

    goal = FollowPath.Goal()
    goal.path = path
    goal.controller_id = "MPPIController"
    goal.goal_checker_id = "high_precision_goal_checker"
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)
    handle = send_future.result()
    if handle is None or not handle.accepted:
        raise RuntimeError("Isolated FollowPath goal rejected")
    print(f"Isolated probe accepted: {len(path.poses)} points, {args.distance:.2f} m")

    deadline = time.monotonic() + args.seconds
    result = handle.get_result_async()
    while not result.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if result.done():
        print(f"Probe ended early with status {result.result().status}")
    else:
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel, timeout_sec=5.0)
        print("Probe canceled after requested duration")
    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
