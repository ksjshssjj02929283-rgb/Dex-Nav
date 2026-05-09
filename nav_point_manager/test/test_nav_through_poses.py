#!/usr/bin/env python3
"""
测试脚本：通过 NavThroughPoses 实现连续通过多个路径点的导航
NavThroughPoses 不会在每个点停顿，适合路网导航场景
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
import math
import time


class NavThroughPosesTester(Node):

    def __init__(self):
        super().__init__('nav_through_poses_tester')

        self.action_client = ActionClient(self, NavigateThroughPoses,
                                          '/navigate_through_poses')

        self.nav_complete = False
        self.nav_success = False

        self.get_logger().info('NavThroughPoses Tester initialized')
        self.get_logger().info('Waiting for action server...')

        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'NavigateThroughPoses action server not available!')
            return

        self.get_logger().info('Action server is ready')

    def create_pose(self, x, y, theta):
        """创建 PoseStamped 消息"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        q = self.euler_to_quaternion(0, 0, theta)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return pose

    def euler_to_quaternion(self, roll, pitch, yaw):
        """欧拉角转四元数"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return [x, y, z, w]

    def send_waypoints(self, waypoints):
        """发送路径点列表"""
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = waypoints
        goal_msg.behavior_tree = ""

        self.get_logger().info(
            f'Sending {len(waypoints)} waypoints to navigate through...')

        self._goal_handle = None
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """处理目标接受/拒绝"""
        self._goal_handle = future.result()
        if self._goal_handle.accepted:
            self.get_logger().info('Goal accepted by Nav2 server')
            self._result_future = self._goal_handle.get_result_async()
            self._result_future.add_done_callback(self.result_callback)
        else:
            self.get_logger().error('Goal rejected by Nav2 server')
            self.nav_complete = True
            self.nav_success = False

    def feedback_callback(self, feedback_msg):
        """处理进度反馈"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Progress: {feedback.number_of_poses_remaining} poses remaining, '
            f'distance: {feedback.distance_remaining:.2f}m')

    def result_callback(self, future):
        """处理最终结果"""
        result = future.result()
        status = result.status

        # GoalStatus: SUCCEEDED=4, ABORTED=6, CANCELED=5
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation through all poses SUCCEEDED')
            self.nav_success = True
        elif status == 6:  # ABORTED
            self.get_logger().error('Navigation ABORTED')
            self.nav_success = False
        elif status == 5:  # CANCELED
            self.get_logger().error('Navigation CANCELED')
            self.nav_success = False
        else:
            self.get_logger().error(f'Navigation failed with status: {status}')
            self.nav_success = False

        self.nav_complete = True

    def wait_for_complete(self, timeout_sec=120.0):
        """等待导航完成"""
        self.get_logger().info('Waiting for navigation to complete...')

        start_time = self.get_clock().now()

        while rclpy.ok() and not self.nav_complete:
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().error(
                    f'Timeout waiting for navigation ({timeout_sec}s)')
                return False
            rclpy.spin_once(self, timeout_sec=0.5)

        return self.nav_success


def main():
    rclpy.init()
    tester = NavThroughPosesTester()

    try:
        waypoints = [
            tester.create_pose(-1.0, -4.0, -1.57),
            tester.create_pose(-0.5, -7.0, 3.14),
            tester.create_pose(-3.5, -6.0, 3.14),
            tester.create_pose(-6.0, -4.0, 1.57),
        ]

        tester.get_logger().info('\n=== Test: NavThroughPoses ===')
        tester.get_logger().info(f'Waypoints: {len(waypoints)} points')
        for i, wp in enumerate(waypoints):
            tester.get_logger().info(
                f'  {i+1}. ({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f})'
            )

        tester.send_waypoints(waypoints)

        if tester.wait_for_complete(timeout_sec=120.0):
            tester.get_logger().info('NavThroughPoses test PASSED')
        else:
            tester.get_logger().error('NavThroughPoses test FAILED')

    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted by user')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
