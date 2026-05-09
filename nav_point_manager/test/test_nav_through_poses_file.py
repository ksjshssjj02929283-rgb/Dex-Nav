#!/usr/bin/env python3
"""
测试脚本：调用 nav_through_poses_file 服务进行路网导航
"""

import rclpy
from rclpy.node import Node
from nav_interfaces.srv import NavThroughPosesFile


class NavThroughPosesFileTester(Node):

    def __init__(self):
        super().__init__('nav_through_poses_file_tester')

        # 创建服务客户端
        self.client = self.create_client(
            NavThroughPosesFile,
            '/nav_point_manager_node/nav_through_poses_file')

        self.get_logger().info('NavThroughPosesFile Tester initialized')
        self.get_logger().info('Waiting for service...')

        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service not available!')
            return

        self.get_logger().info('Service is ready')

    def send_request(self, filename):
        """发送路网导航请求"""
        request = NavThroughPosesFile.Request()
        request.filename = filename

        self.get_logger().info(f'Sending request with filename: {filename}')

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'SUCCESS: {response.message}')
                else:
                    self.get_logger().error(f'FAILED: {response.message}')
                return response.success
            except Exception as e:
                self.get_logger().error(f'Service call failed: {str(e)}')
                return False
        else:
            self.get_logger().error('Service call timed out')
            return False


def main():
    rclpy.init()
    tester = NavThroughPosesFileTester()

    try:
        # 测试 1：使用示例路径文件
        tester.get_logger().info('\n=== Test 1: example_route.json ===')
        success = tester.send_request('example_route.json')

        if success:
            tester.get_logger().info(
                'Test PASSED - Navigation started successfully')
        else:
            tester.get_logger().error('Test FAILED')

        # 可以添加更多测试
        # tester.get_logger().info('\n=== Test 2: another_route.json ===')
        # tester.send_request('another_route.json')

    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted by user')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
