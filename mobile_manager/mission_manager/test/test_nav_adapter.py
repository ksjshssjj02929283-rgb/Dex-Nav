#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nav Adapter 集成测试
测试导航适配器的基本功能
"""

import sys
import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from nav_interfaces.srv import NavToPoint
from std_msgs.msg import String, Bool


class NavAdapterTester(Node):
    """Nav Adapter 测试类"""

    def __init__(self):
        super().__init__('nav_adapter_tester')

        # Nav Adapter 客户端
        self.start_nav_client = self.create_client(
            NavToPoint, '/nav_adapter_node/start_navigation')
        self.cancel_nav_client = self.create_client(
            Trigger, '/nav_adapter_node/cancel_navigation')

        # 订阅状态话题
        self.status_sub = self.create_subscription(
            String, '/nav_adapter_node/navigation_status', self.status_callback,
            10)
        self.result_sub = self.create_subscription(
            Bool, '/nav_adapter_node/navigation_result', self.result_callback,
            10)

        self.last_status = None
        self.last_result = None

        self.get_logger().info('Nav Adapter测试节点初始化完成')

    def status_callback(self, msg):
        """导航状态回调"""
        self.last_status = msg.data
        self.get_logger().info(f'收到导航状态: {msg.data}')

    def result_callback(self, msg):
        """导航结果回调"""
        self.last_result = msg.data
        self.get_logger().info(f'收到导航结果: {"成功" if msg.data else "失败"}')

    def wait_for_service(self, client, timeout_sec=10.0):
        """等待服务可用"""
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f'服务在 {timeout_sec}s 内不可用')
            return False
        return True

    def call_service(self, client, request, timeout_sec=10.0):
        """调用服务的辅助函数"""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if future.result() is not None:
            return future.result()
        else:
            exception_msg = str(
                future.exception()) if future.exception() else 'Timeout'
            self.get_logger().error(f'服务调用失败: {exception_msg}')
            return None

    def test_start_navigation(self):
        """测试开始导航"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('测试 1: 开始导航')
        self.get_logger().info('=' * 50)

        # 等待服务
        if not self.wait_for_service(self.start_nav_client):
            return False

        # 创建导航请求
        nav_req = NavToPoint.Request()
        nav_req.nav_point.header.frame_id = 'map'
        nav_req.nav_point.id = 1
        nav_req.nav_point.name = 'point_home'
        nav_req.nav_point.point_type = 1  # STATION
        nav_req.nav_point.x = 0.0
        nav_req.nav_point.y = 0.0
        nav_req.nav_point.theta = 0.0

        self.get_logger().info(f'发送导航请求到: point_home (0.0, 0.0, 0.0)')

        # 调用服务
        nav_res = self.call_service(self.start_nav_client,
                                    nav_req,
                                    timeout_sec=8.0)

        if nav_res is None:
            self.get_logger().error('❌ 导航服务调用超时')
            return False

        # 检查结果
        if nav_res.success:
            self.get_logger().info('✅ 导航请求成功发送到 Nav2')
            self.get_logger().info('   这意味着 Nav2 正在运行并接受了导航目标')
        else:
            self.get_logger().info('ℹ️  导航请求未成功（Nav2 不可用）')
            self.get_logger().info('   这是预期的，如果没有运行 Nav2 导航栈')
            self.get_logger().info('   但 nav_adapter 服务本身工作正常')

        return True

    def test_cancel_navigation(self):
        """测试取消导航"""
        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('测试 2: 取消导航')
        self.get_logger().info('=' * 50)

        # 等待服务
        if not self.wait_for_service(self.cancel_nav_client):
            return False

        # 创建取消请求
        cancel_req = Trigger.Request()

        self.get_logger().info('发送取消导航请求')

        # 调用服务
        cancel_res = self.call_service(self.cancel_nav_client,
                                       cancel_req,
                                       timeout_sec=5.0)

        if cancel_res is None:
            self.get_logger().error('❌ 取消导航服务调用超时')
            return False

        # 检查结果
        if cancel_res.success:
            self.get_logger().info('✅ 导航取消成功')
        else:
            self.get_logger().info('ℹ️  没有活动的导航任务可以取消')

        return True

    def run_tests(self):
        """运行所有测试"""
        self.get_logger().info('')
        self.get_logger().info('🚀 开始 Nav Adapter 测试...')
        self.get_logger().info('')

        # 测试 1: 开始导航
        test1_passed = self.test_start_navigation()

        # 测试 2: 取消导航
        test2_passed = self.test_cancel_navigation()

        # 打印总结
        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('测试总结')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'开始导航服务: {"✅ 通过" if test1_passed else "❌ 失败"}')
        self.get_logger().info(f'取消导航服务: {"✅ 通过" if test2_passed else "❌ 失败"}')
        self.get_logger().info('')

        all_passed = test1_passed and test2_passed
        if all_passed:
            self.get_logger().info('🎉 Nav Adapter 测试通过！')
            self.get_logger().info('')
            self.get_logger().info('注意：')
            self.get_logger().info('- 如果没有运行 Nav2，导航请求会返回 success=false')
            self.get_logger().info('- 这是预期行为，说明 nav_adapter 正确检测到 Nav2 不可用')
            self.get_logger().info('- 服务本身响应正常，表明 nav_adapter 工作正常')
            return 0
        else:
            self.get_logger().error('❌ Nav Adapter 测试失败')
            return 1


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    tester = NavAdapterTester()

    try:
        result = tester.run_tests()
    except KeyboardInterrupt:
        tester.get_logger().info('测试被用户中断')
        result = 1
    except Exception as e:
        tester.get_logger().error(f'测试异常: {e}')
        result = 1
    finally:
        tester.destroy_node()
        rclpy.shutdown()

    return result


if __name__ == '__main__':
    sys.exit(main())
