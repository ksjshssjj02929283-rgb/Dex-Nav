#!/usr/bin/env python3
"""
测试脚本：取消导航
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class NavigationCanceller(Node):

    def __init__(self):
        super().__init__('navigation_canceller')

        # 创建取消导航服务客户端
        self.cancel_client = self.create_client(
            Trigger, '/nav_point_manager_node/cancel_navigation')

        self.get_logger().info('Navigation Canceller initialized')
        self.get_logger().info(
            'Waiting for nav_point_manager cancel service...')

        if not self.cancel_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                'nav_point_manager cancel service not available!')
            return

        self.get_logger().info('Cancel service is ready')

    def cancel_navigation(self):
        """发送取消导航请求"""
        request = Trigger.Request()

        self.get_logger().info('Sending cancel navigation request...')

        future = self.cancel_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info('Navigation cancelled successfully!')
            else:
                self.get_logger().warn(
                    'Cancel request failed (maybe no active navigation)')
            return response.success
        else:
            self.get_logger().error('Service call failed!')
            return False


def main():
    rclpy.init()
    canceller = NavigationCanceller()

    try:
        # 发送取消请求
        canceller.cancel_navigation()

    except KeyboardInterrupt:
        canceller.get_logger().info('Cancelled by user')
    finally:
        canceller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
