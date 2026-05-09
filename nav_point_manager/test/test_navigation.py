#!/usr/bin/env python3
"""
测试脚本：发送导航目标点并接收导航结果
"""

import rclpy
from rclpy.node import Node
from nav_interfaces.srv import NavToPoint, GetStatus
from nav_interfaces.msg import NavPoint
from std_srvs.srv import SetBool


class NavigationTester(Node):

    def __init__(self):
        super().__init__('navigation_tester')

        self.nav_client = self.create_client(
            NavToPoint, '/nav_point_manager_node/start_navigation')
        self.status_client = self.create_client(
            GetStatus, '/nav_point_manager_node/get_navigation_status')

        self.result_service = self.create_service(
            SetBool, '/mission_manager_node/set_navigation_success',
            self.navigation_result_callback)

        self.navigation_complete = False
        self.last_status = "idle"

        self.get_logger().info('Navigation Tester initialized')
        self.get_logger().info('Waiting for nav_point_manager service...')

        if not self.nav_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('nav_point_manager service not available!')
            return

        if not self.status_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                'nav_point_manager status service not available!')
            return

        self.get_logger().info('nav_point_manager service is ready')

    def navigation_result_callback(self, request, response):
        """接收导航结果的回调"""
        result_str = 'SUCCESS' if request.data else 'FAILED'
        self.get_logger().info(f'========================================')
        self.get_logger().info(f'Navigation Result Received: {result_str}')
        self.get_logger().info(f'========================================')
        response.success = True
        response.message = 'Result received'
        return response

    def wait_for_navigation_complete(self, timeout_sec=60.0):
        """等待导航完成（状态变为 succeeded 或 idle）"""
        self.get_logger().info('Waiting for navigation to complete...')

        start_time = self.get_clock().now()

        while rclpy.ok():
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().error(
                    f'Timeout waiting for navigation to complete ({timeout_sec}s)'
                )
                return False

            request = GetStatus.Request()
            future = self.status_client.call_async(request)

            try:
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            except Exception as e:
                self.get_logger().warn(f'Spin exception: {e}')
                continue

            if future.result() is not None:
                response = future.result()
                status = response.status
                point_name = response.name if response.name else 'N/A'

                self.get_logger().info(
                    f'Current navigation status: {status} (point: {point_name})'
                )

                if status == 'succeeded' or status == 'idle':
                    self.get_logger().info('Navigation completed!')
                    return True

                if status == 'aborted' or status == 'cancelled':
                    self.get_logger().error(
                        f'Navigation failed with status: {status}')
                    return False

        return False

    def send_navigation_goal(self,
                             x,
                             y,
                             theta,
                             tolerance=0.0,
                             name='test_point'):
        """发送导航目标点"""
        request = NavToPoint.Request()
        request.nav_point.name = name
        request.nav_point.x = x
        request.nav_point.y = y
        request.nav_point.theta = theta
        request.nav_point.tolerance = tolerance
        request.nav_point.header.frame_id = 'map'

        self.get_logger().info(f'Sending navigation goal:')
        self.get_logger().info(f'  Name: {name}')
        self.get_logger().info(f'  Position: ({x:.2f}, {y:.2f}, {theta:.2f})')
        self.get_logger().info(f'  Tolerance: {tolerance:.3f}')

        future = self.nav_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info('Navigation goal accepted!')
            else:
                self.get_logger().error('Navigation goal rejected!')
            return response.success
        else:
            self.get_logger().error('Service call failed!')
            return False


def main():
    rclpy.init()
    tester = NavigationTester()

    try:
        tester.get_logger().info('\n=== Test 1: Sending point_1 ===')
        tester.send_navigation_goal(x=-2.0,
                                    y=-2.0,
                                    theta=0.0,
                                    tolerance=0.1,
                                    name='point_1')

        if not tester.wait_for_navigation_complete(timeout_sec=60.0):
            tester.get_logger().error('First navigation failed, aborting test')
            return

        tester.get_logger().info('\n=== Test 2: Sending point_2 ===')
        tester.send_navigation_goal(x=-1.0,
                                    y=-5.0,
                                    theta=1.57,
                                    tolerance=0.05,
                                    name='point_2')

        tester.get_logger().info('Waiting for second navigation to complete...')
        tester.wait_for_navigation_complete(timeout_sec=60.0)

        tester.get_logger().info('All navigation tests completed!')
        rclpy.spin(tester)

    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted by user')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
