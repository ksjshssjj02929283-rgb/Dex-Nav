#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math
import time


class TwoPointPatrol(Node):

    def __init__(self):
        super().__init__('two_point_patrol')
        self.navigator = BasicNavigator()

        # 定义两个导航目标点 (请根据你的地图修改)
        self.point_a = self._create_pose(3.00, 1.95, 1.57)
        # self.point_b = self._create_pose(3.00, 10.85, 1.57)
        self.point_c = self._create_pose(6.10, 5.85, 0.0)
        self.points = [self.point_a, self.point_c]
        self.current_index = 0

        self.last_log_time = 0  # 用于节流日志输出

        self.get_logger().info("Two point patrol started.")
        self.patrol_loop()

    def _create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def patrol_loop(self):
        while rclpy.ok():
            target = self.points[self.current_index]
            self.get_logger().info(
                f"Navigating to point {self.current_index + 1}...")

            self.navigator.goToPose(target)

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    distance = feedback.distance_remaining
                    # 每隔 2 秒打印一次距离
                    if (self.get_clock().now().nanoseconds -
                            self.last_log_time) / 1e9 > 0.5:
                        self.get_logger().info(
                            f"Distance remaining: {distance:.2f} m")
                        self.last_log_time = self.get_clock().now().nanoseconds

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(
                    f"Reached point {self.current_index + 1} successfully!")
            elif result == TaskResult.CANCELED:
                self.get_logger().warn("Navigation task canceled.")
                break
            elif result == TaskResult.FAILED:
                self.get_logger().error(
                    f"Navigation to point {self.current_index + 1} failed!")
                # 等待人工确认
                user_input = input(
                    "导航失败，输入 y 后继续到下一个点，或按 Ctrl+C 退出: ").strip().lower()
                if user_input == "y":
                    self.get_logger().info("继续导航下一个点...")
                    # 继续循环（自动切换下一个点）
                    pass
                else:
                    self.get_logger().warn("未确认继续，退出巡航任务。")
                    break

            self.get_logger().info("Waiting 3 seconds before next goal...")
            time.sleep(3)

            self.current_index = (self.current_index + 1) % len(self.points)


def main(args=None):
    rclpy.init(args=args)
    node = TwoPointPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Patrol stopped by user.")
    finally:
        node.navigator.cancelTask()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
