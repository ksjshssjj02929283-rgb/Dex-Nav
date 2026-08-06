#!/usr/bin/env python3
"""发布当前规划周期内的实际 Odom 行驶轨迹，供 RViz 与 /plan 对照。"""

import copy
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


# 关键配置：规划路径到来时清空一次历史；实际轨迹采样和发布均由下面参数限制。
PLAN_TOPIC = '/plan'  # Nav2 PlannerServer 发布的当前全局规划路径。
ODOM_TOPIC = '/feedback/odom'  # Gazebo 差速底盘的实际里程计。
ACTUAL_PATH_TOPIC = '/navigation/actual_path'  # RViz 中显示为红色曲线。
MIN_POINT_DISTANCE_M = 0.01  # 相邻记录点至少相距 1 cm，避免 Path 消息无界增长。
PUBLISH_INTERVAL_S = 0.1  # 最多 10 Hz 发布，降低 RViz 和 DDS 负载。
MAX_PATH_POINTS = 5000  # 单轮规划最多保留的轨迹点，超出时丢弃最旧点。


class PathTraceVisualizer(Node):
    """将当前规划后的 Odom 轨迹转成 nav_msgs/Path。"""

    def __init__(self):
        super().__init__('path_trace_visualizer')
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_publisher = self.create_publisher(
            Path, ACTUAL_PATH_TOPIC, path_qos)
        self.create_subscription(Path, PLAN_TOPIC, self._on_plan, 1)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 20)

        self.actual_path = Path()
        self.have_plan = False
        self.last_publish_time = None

    def _on_plan(self, plan: Path):
        """每次规划都清空上一轮红色实际轨迹。"""
        self.actual_path = Path()
        self.actual_path.header = copy.deepcopy(plan.header)
        self.have_plan = True
        self.last_publish_time = None
        # 发布空 Path 立即清除 RViz 中上一轮留下的红线。
        self.path_publisher.publish(self.actual_path)

    def _on_odom(self, odom: Odometry):
        """按空间阈值记录实际位姿，并以受限频率发布。"""
        if not self.have_plan:
            return

        pose = PoseStamped()
        pose.header = copy.deepcopy(odom.header)
        pose.pose = copy.deepcopy(odom.pose.pose)
        if self.actual_path.poses:
            last = self.actual_path.poses[-1].pose.position
            current = pose.pose.position
            if math.hypot(current.x - last.x, current.y - last.y) < MIN_POINT_DISTANCE_M:
                return

        self.actual_path.header = copy.deepcopy(odom.header)
        self.actual_path.poses.append(pose)
        if len(self.actual_path.poses) > MAX_PATH_POINTS:
            del self.actual_path.poses[:-MAX_PATH_POINTS]

        now = self.get_clock().now()
        if (self.last_publish_time is None or
                (now - self.last_publish_time).nanoseconds >=
                int(PUBLISH_INTERVAL_S * 1e9)):
            self.path_publisher.publish(self.actual_path)
            self.last_publish_time = now


def main():
    rclpy.init()
    node = PathTraceVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
