#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import sys
import time


class SingleCtrlPosePublisher(Node):

    def __init__(self, x, y, yaw):
        super().__init__('ctrl_pose_single_publisher')

        self.pub = self.create_publisher(PoseStamped, 'control/ctrl_pose', 10)

        # 立即发布一次
        msg = PoseStamped()
        # msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        # yaw 转四元数
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        msg.pose.orientation.z = sy
        msg.pose.orientation.w = cy

        self.pub.publish(msg)
        self.get_logger().info(f"[Single Publish] x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

        # 给一点时间确保消息发出
        time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)

    # 默认值
    x, y, yaw = 0.5, 0.0, -1.0

    # 命令行参数控制
    if len(sys.argv) == 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        yaw = float(sys.argv[3])

    node = SingleCtrlPosePublisher(x, y, yaw)

    # 发布完马上退出
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
