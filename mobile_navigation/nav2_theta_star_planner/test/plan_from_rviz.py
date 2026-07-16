#!/usr/bin/env python3
# Copyright (c) DexForce
#
# Bridge RViz 2D Pose Estimate + 2D Nav Goal to planner_server/compute_path_to_pose.

import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav2_msgs.action import ComputePathToPose
import tf2_ros


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PlanFromRviz(Node):
    def __init__(self):
        super().__init__('plan_from_rviz')
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('default_start_x', 0.0)
        self.declare_parameter('default_start_y', 0.0)
        self.declare_parameter('default_start_yaw', 0.0)

        self._planner_id = self.get_parameter('planner_id').get_parameter_value().string_value
        self._client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self._start = PoseStamped()
        self._start.header.frame_id = 'map'
        self._start.pose.orientation.w = 1.0
        self.set_start(
            self.get_parameter('default_start_x').value,
            self.get_parameter('default_start_y').value,
            self.get_parameter('default_start_yaw').value,
            log=False)

        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._on_initialpose, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)

        self.get_logger().info(
            'Ready: set start with "2D Pose Estimate", then "2D Nav Goal" to plan.')

    def set_start(self, x, y, yaw, log=True):
        self._start.pose.position.x = float(x)
        self._start.pose.position.y = float(y)
        self._start.pose.position.z = 0.0
        self._start.pose.orientation.z = math.sin(yaw * 0.5)
        self._start.pose.orientation.w = math.cos(yaw * 0.5)
        self._publish_base_tf()
        if log:
            self.get_logger().info(f'Start: ({x:.2f}, {y:.2f}), yaw={math.degrees(yaw):.1f}°')

    def _publish_base_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self._start.pose.position.x
        t.transform.translation.y = self._start.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self._start.pose.orientation
        self._tf_broadcaster.sendTransform(t)

    def _on_initialpose(self, msg: PoseWithCovarianceStamped):
        self._start.header = msg.header
        if not self._start.header.frame_id:
            self._start.header.frame_id = 'map'
        self._start.pose = msg.pose.pose
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.get_logger().info(
            f'Start: ({self._start.pose.position.x:.2f}, '
            f'{self._start.pose.position.y:.2f}), yaw={math.degrees(yaw):.1f}°')
        self._publish_base_tf()

    def _on_goal(self, msg: PoseStamped):
        if not self._client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(
                'planner_server action not available — is lifecycle active?')
            return

        goal = ComputePathToPose.Goal()
        goal.goal = msg
        if not goal.goal.header.frame_id:
            goal.goal.header.frame_id = 'map'
        goal.start = self._start
        goal.start.header.stamp = msg.header.stamp
        if not goal.start.header.frame_id:
            goal.start.header.frame_id = 'map'
        goal.use_start = True
        goal.planner_id = self._planner_id

        self.get_logger().info(
            f'Planning: ({goal.start.pose.position.x:.2f}, '
            f'{goal.start.pose.position.y:.2f}) -> '
            f'({goal.goal.pose.position.x:.2f}, {goal.goal.pose.position.y:.2f})')

        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Plan request rejected by planner_server')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        n = len(result.path.poses)
        sec = result.planning_time.sec + result.planning_time.nanosec * 1e-9
        if n == 0:
            self.get_logger().error(f'Planning failed (t={sec:.3f}s, empty path)')
        else:
            self.get_logger().info(
                f'Plan OK: {n} poses, action planning_time={sec:.3f}s → /plan '
                f'(planner_server also logs baseline vs curve post-process ms if smooth_log_timing=true)')


def main(args=None):
    rclpy.init(args=args)
    node = PlanFromRviz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
