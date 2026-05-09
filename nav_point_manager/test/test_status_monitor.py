#!/usr/bin/env python3
"""
测试脚本：定时查询导航状态（每1秒查询一次）
"""

import rclpy
from rclpy.node import Node
from nav_interfaces.srv import GetStatus


class StatusMonitor(Node):

    def __init__(self):
        super().__init__('status_monitor')

        # 创建状态查询服务客户端
        self.status_client = self.create_client(
            GetStatus, '/nav_point_manager_node/get_navigation_status')

        # 创建定时器，每1秒查询一次
        self.timer = self.create_timer(1.0, self.query_status)

        self.get_logger().info('Status Monitor initialized')
        self.get_logger().info(
            'Waiting for nav_point_manager status service...')

        if not self.status_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                'nav_point_manager status service not available!')
            return

        self.get_logger().info(
            'Status service is ready, monitoring every 1 second...')

    def query_status(self):
        """定时查询导航状态"""
        request = GetStatus.Request()

        future = self.status_client.call_async(request)
        future.add_done_callback(self.status_callback)

    def status_callback(self, future):
        """处理状态查询结果"""
        try:
            response = future.result()
            if response.success:
                status_str = response.status
                point_name = response.name if response.name else 'N/A'

                # 根据状态使用不同的日志级别
                if status_str == 'succeeded':
                    self.get_logger().info(
                        f'[STATUS] {status_str.upper()} - Point: {point_name}')
                elif status_str == 'navigating':
                    self.get_logger().info(
                        f'[STATUS] {status_str} - Point: {point_name}')
                elif status_str == 'idle':
                    self.get_logger().info(f'[STATUS] {status_str}')
                else:
                    self.get_logger().warn(
                        f'[STATUS] {status_str} - Point: {point_name}')
            else:
                self.get_logger().error('Failed to get status')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')


def main():
    rclpy.init()
    monitor = StatusMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('Status monitor stopped by user')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
