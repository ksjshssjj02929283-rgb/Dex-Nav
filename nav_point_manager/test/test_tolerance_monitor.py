#!/usr/bin/env python3
"""
测试脚本：监控 controller_server 的 lon/lat_goal_tolerance 参数（每1秒查询一次）
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters


class ToleranceMonitor(Node):

    def __init__(self):
        super().__init__('tolerance_monitor')

        # 声明参数
        self.declare_parameter('controller_server_name', '/controller_server')
        controller_name = self.get_parameter('controller_server_name').value

        # 创建参数查询服务客户端
        self.param_client = self.create_client(
            GetParameters, f'{controller_name}/get_parameters')

        # 创建定时器，每1秒查询一次
        self.timer = self.create_timer(1.0, self.query_tolerance)

        self.last_lon_tolerance = None
        self.last_lat_tolerance = None

        self.get_logger().info('Tolerance Monitor initialized')
        self.get_logger().info(
            f'Monitoring: {controller_name}/goal_checker.lon_goal_tolerance')
        self.get_logger().info(
            f'Monitoring: {controller_name}/goal_checker.lat_goal_tolerance')
        self.get_logger().info(
            'Waiting for controller_server parameter service...')

        if not self.param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                'controller_server parameter service not available!')
            return

        self.get_logger().info(
            'Parameter service is ready, monitoring every 1 second...')

    def query_tolerance(self):
        """定时查询 lon/lat_goal_tolerance 参数"""
        request = GetParameters.Request()
        request.names = [
            'goal_checker.lon_goal_tolerance', 'goal_checker.lat_goal_tolerance'
        ]

        future = self.param_client.call_async(request)
        future.add_done_callback(self.tolerance_callback)

    def tolerance_callback(self, future):
        """处理参数查询结果"""
        try:
            response = future.result()
            if len(response.values) >= 2:
                lon_param = response.values[0]
                lat_param = response.values[1]

                if lon_param.type == 3 and lat_param.type == 3:  # PARAMETER_DOUBLE
                    lon_tolerance = lon_param.double_value
                    lat_tolerance = lat_param.double_value

                    # 只在值变化时打印
                    lon_changed = (self.last_lon_tolerance is None or
                                   abs(lon_tolerance - self.last_lon_tolerance)
                                   > 0.0001)
                    lat_changed = (self.last_lat_tolerance is None or
                                   abs(lat_tolerance - self.last_lat_tolerance)
                                   > 0.0001)

                    if lon_changed or lat_changed:
                        self.get_logger().info(
                            f'[TOLERANCE] lon_goal_tolerance = {lon_tolerance:.4f}, '
                            f'lat_goal_tolerance = {lat_tolerance:.4f}')
                        if self.last_lon_tolerance is not None:
                            self.get_logger().info(
                                f'  Changed from (lon={self.last_lon_tolerance:.4f}, '
                                f'lat={self.last_lat_tolerance:.4f}) to '
                                f'(lon={lon_tolerance:.4f}, lat={lat_tolerance:.4f})'
                            )
                        self.last_lon_tolerance = lon_tolerance
                        self.last_lat_tolerance = lat_tolerance
                else:
                    self.get_logger().warn(
                        f'Unexpected parameter types: {lon_param.type}, {lat_param.type}'
                    )
            else:
                self.get_logger().warn(
                    f'Expected 2 parameter values, got {len(response.values)}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')


def main():
    rclpy.init()
    monitor = ToleranceMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('Tolerance monitor stopped by user')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
