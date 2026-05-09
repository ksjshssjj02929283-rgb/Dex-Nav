#!/usr/bin/env python3
"""
测试脚本：验证 nav_point_manager 的基本功能
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time


class NavPointManagerTester(Node):

    def __init__(self):
        super().__init__('nav_point_manager_tester')

        # 创建服务客户端
        self.reload_client = self.create_client(
            Trigger, '/nav_point_manager_node/reload_points')

        self.get_logger().info('Waiting for services...')
        self.reload_client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info('Services ready!')

    def test_reload_points(self):
        """测试重新加载点位"""
        self.get_logger().info('Testing reload_points service...')

        request = Trigger.Request()
        future = self.reload_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(
                f'Reload result: {response.success}, {response.message}')
            return response.success
        else:
            self.get_logger().error('Service call failed')
            return False


def main():
    rclpy.init()

    tester = NavPointManagerTester()

    # 运行测试
    time.sleep(1.0)

    success = tester.test_reload_points()

    if success:
        tester.get_logger().info('✓ All tests passed!')
    else:
        tester.get_logger().error('✗ Some tests failed')

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
