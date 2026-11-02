#!/usr/bin/env python3
"""将融合 Odom 中的底盘位姿发布到 /feedback/chassis_pose。"""

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


FILTERED_ODOM_TOPIC = '/localization/odometry/filtered'
POSE_TOPIC = '/feedback/chassis_pose'
TARGET_FRAME = 'map'
ALLOWED_CHILD_FRAMES = ('base_link', 'base_footprint')
LOG_THROTTLE_SEC = 5.0
ODOM_TIMEOUT_SEC = 2.0
MONITOR_PERIOD_SEC = 1.0


class BaseLinkPosition(Node):
    def __init__(self):
        super().__init__('baselink_position')

        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(PoseStamped, POSE_TOPIC, output_qos)
        self.odom_subscription = self.create_subscription(
            Odometry,
            FILTERED_ODOM_TOPIC,
            self._odom_callback,
            input_qos,
        )
        self.monitor_timer = self.create_timer(
            MONITOR_PERIOD_SEC,
            self._monitor_odom,
        )

        self._last_valid_odom_monotonic = time.monotonic()
        self._last_problem_log_monotonic = float('-inf')
        self._has_valid_odom = False
        self._odom_timeout_active = False
        self._rejected_odom_count = 0

        self.get_logger().info(
            f'Chassis pose publisher started; input: {FILTERED_ODOM_TOPIC}; '
            f'output: {POSE_TOPIC}.')

    def _log_problem(self, message):
        now = time.monotonic()
        if now - self._last_problem_log_monotonic < LOG_THROTTLE_SEC:
            return
        self.get_logger().warning(message)
        self._last_problem_log_monotonic = now

    def _monitor_odom(self):
        elapsed = time.monotonic() - self._last_valid_odom_monotonic
        if elapsed > ODOM_TIMEOUT_SEC:
            self._log_problem(
                f'No valid odometry received for {elapsed:.1f} seconds.')
            self._odom_timeout_active = True

    def _odom_callback(self, msg):
        frame_id = msg.header.frame_id.lstrip('/')
        child_frame_id = msg.child_frame_id.lstrip('/')
        if frame_id != TARGET_FRAME or child_frame_id not in ALLOWED_CHILD_FRAMES:
            self._rejected_odom_count += 1
            self._log_problem(
                'Rejected odometry; expected map frame with base_link or '
                f'base_footprint, got {frame_id}/{child_frame_id}. '
                f'Rejected messages: {self._rejected_odom_count}.')
            return

        self._last_valid_odom_monotonic = time.monotonic()
        if not self._has_valid_odom:
            self.get_logger().info('Received valid odometry.')
        elif self._odom_timeout_active:
            self.get_logger().info('Odometry recovered.')
        self._has_valid_odom = True
        self._odom_timeout_active = False

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = TARGET_FRAME
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = msg.pose.pose.position.x
        pose_msg.pose.position.y = msg.pose.pose.position.y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = msg.pose.pose.orientation
        self.publisher.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BaseLinkPosition()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
