#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mission Manager 集成测试
测试与map_manager和nav2_point_manager的集成
"""

import sys
import time
import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from nav_interfaces.srv import (GetMaps, MapManagerCommand, SetNavPoint,
                                GetAllNavPoints, SampleNavPoint, GetStatus,
                                NavToPoint, AddTask)
from nav_interfaces.msg import NavPoint as NavPointMsg, Task as TaskMsg


class IntegrationTester(Node):
    """集成测试类"""

    def __init__(self):
        super().__init__('integration_tester')

        # Map Manager客户端
        # 2026-09-09 接口收敛：load_map/get_map_name/get_current_map_info → map_command/get_maps
        self.map_command_client = self.create_client(
            MapManagerCommand, '/map_manager/map_command')
        self.get_maps_client = self.create_client(
            GetMaps, '/map_manager/get_maps')

        # Nav Point Manager客户端
        self.nav_set_map_client = self.create_client(
            Trigger, '/nav_point_manager_node/set_map_changed')
        self.add_nav_point_client = self.create_client(
            SetNavPoint, '/map_asset_manager/set_point')
        self.get_all_nav_points_client = self.create_client(
            GetAllNavPoints, '/nav_point_manager_node/get_all_nav_points')
        self.sample_nav_point_client = self.create_client(
            SampleNavPoint, '/nav_point_manager_node/sample_point')

        # Mission Manager客户端
        # 注意: 单独启动时节点名是 mission_manager_node，通过 launch 启动时是 mission_manager
        self.mission_set_map_client = self.create_client(
            Trigger, '/mission_manager/set_map_changed')
        self.mission_node_set_map_client = self.create_client(
            Trigger, '/mission_manager_node/set_map_changed')
        self.mission_get_status_client = self.create_client(
            GetStatus, '/mission_manager/get_mission_status')
        self.mission_node_get_status_client = self.create_client(
            GetStatus, '/mission_manager_node/get_mission_status')
        self.add_task_client = self.create_client(AddTask,
                                                  '/mission_manager/add_task')
        self.add_task_node_client = self.create_client(
            AddTask, '/mission_manager_node/add_task')

        # Nav Adapter客户端
        self.nav_adapter_start_nav_client = self.create_client(
            NavToPoint, '/nav_adapter_node/start_navigation')
        self.nav_adapter_cancel_nav_client = self.create_client(
            Trigger, '/nav_adapter_node/cancel_navigation')

        self.get_logger().info('集成测试节点初始化完成')

    def wait_for_services(self, timeout_sec=10.0):
        """等待所有服务可用"""
        # 首先检测 mission_manager 的服务名称（单独启动 vs launch启动）
        mission_set_map_client = self.mission_node_set_map_client
        mission_service_name = '/mission_manager_node/set_map_changed'

        self.get_logger().info(f'检测 mission_manager 服务...')
        if not mission_set_map_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(
                '未找到 mission_manager_node，尝试 mission_manager')
            mission_set_map_client = self.mission_set_map_client
            mission_service_name = '/mission_manager/set_map_changed'

            if not mission_set_map_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('未找到 mission_manager 服务')
                return False

        # 检测 add_task 服务
        if self.add_task_node_client.wait_for_service(timeout_sec=1.0):
            add_task_client = self.add_task_node_client
            add_task_service_name = '/mission_manager_node/add_task'
        else:
            add_task_client = self.add_task_client
            add_task_service_name = '/mission_manager/add_task'

        # 检测 get_status 服务
        if self.mission_node_get_status_client.wait_for_service(
                timeout_sec=1.0):
            get_status_client = self.mission_node_get_status_client
            get_status_service_name = '/mission_manager_node/get_mission_status'
        else:
            get_status_client = self.mission_get_status_client
            get_status_service_name = '/mission_manager/get_mission_status'

        services = [
            (self.map_command_client, '/map_manager/map_command'),
            (self.nav_set_map_client,
             '/nav_point_manager_node/set_map_changed'),
            (mission_set_map_client, mission_service_name),
            (add_task_client, add_task_service_name),
            (get_status_client, get_status_service_name),
        ]

        for client, name in services:
            self.get_logger().info(f'等待服务: {name}')
            if not client.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().error(f'服务 {name} 在 {timeout_sec}s 内不可用')
                return False

        self.get_logger().info('所有必需服务已就绪')
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

    def test_map_manager(self, map_name='test_map'):
        """测试地图管理器"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('测试1: 地图管理器')
        self.get_logger().info('=' * 50)

        # 1. 加载地图
        self.get_logger().info(f'1.1 加载地图: {map_name}')
        load_req = MapManagerCommand.Request()
        load_req.op = MapManagerCommand.Request.OP_LOAD
        load_req.map_name = map_name
        load_res = self.call_service(self.map_command_client, load_req)

        if load_res:
            self.get_logger().info(f'✅ 地图加载结果: success={load_res.success}')
            if not load_res.success:
                self.get_logger().warn(f'⚠️  地图 {map_name} 不存在，这是预期的')
        else:
            self.get_logger().error('❌ 加载地图服务调用失败')

        # 2. 获取当前地图名称
        self.get_logger().info('1.2 获取当前地图名称')
        get_name_req = GetMaps.Request()
        get_name_res = self.call_service(self.get_maps_client, get_name_req)

        if get_name_res:
            self.get_logger().info(f'✅ 当前地图: {get_name_res.current_map_name}')
        else:
            self.get_logger().warn('⚠️  获取地图名称失败')

        return load_res and load_res.success if load_res else False

    def test_nav_point_manager(self):
        """测试导航点管理器"""
        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('测试2: 导航点管理器')
        self.get_logger().info('=' * 50)

        # 1. 设置地图变更
        self.get_logger().info('2.1 通知nav_point_manager地图已变更')
        set_map_req = Trigger.Request()
        set_map_res = self.call_service(self.nav_set_map_client, set_map_req)

        if set_map_res:
            self.get_logger().info(f'✅ 设置地图变更: success={set_map_res.success}')
            self.get_logger().info(f'   消息: {set_map_res.message}')
        else:
            self.get_logger().error('❌ 设置地图变更失败')
            return False

        # 2. 获取所有导航点
        self.get_logger().info('2.2 获取所有导航点')
        get_all_req = GetAllNavPoints.Request()
        get_all_res = self.call_service(self.get_all_nav_points_client,
                                        get_all_req)

        if get_all_res:
            self.get_logger().info(f'✅ 获取导航点: success={get_all_res.success}')
            self.get_logger().info(f'   导航点数量: {len(get_all_res.nav_points)}')
            for i, point in enumerate(get_all_res.nav_points[:5]):  # 只显示前5个
                self.get_logger().info(
                    f'   点位{i+1}: {point.name} ({point.x:.2f}, {point.y:.2f}, {point.theta:.2f})'
                )
        else:
            self.get_logger().error('❌ 获取导航点失败')
            return False

        # 3. 添加测试导航点（可选）
        self.get_logger().info('2.3 添加测试导航点')
        add_req = SetNavPoint.Request()
        add_req.mode = SetNavPoint.Request.MODE_UPSERT
        add_req.nav_point.name = 'test_point_integration'
        add_req.nav_point.x = 1.0
        add_req.nav_point.y = 2.0
        add_req.nav_point.theta = 0.0
        add_req.nav_point.point_type = 0

        add_res = self.call_service(self.add_nav_point_client, add_req)
        if add_res:
            self.get_logger().info(f'✅ 添加导航点: success={add_res.success}')
        else:
            self.get_logger().warn('⚠️  添加导航点失败（可能已存在）')

        return True

    def test_mission_manager(self):
        """测试任务管理器"""
        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('测试3: 任务管理器')
        self.get_logger().info('=' * 50)

        # 检测使用哪个服务（单独启动 vs launch启动）
        if self.mission_node_set_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('检测到 mission_manager_node (单独启动)')
            set_map_client = self.mission_node_set_map_client
            get_status_client = self.mission_node_get_status_client
        else:
            self.get_logger().info('使用 mission_manager (通过 launch 启动)')
            set_map_client = self.mission_set_map_client
            get_status_client = self.mission_get_status_client

        # 1. 通知mission_manager地图已变更
        self.get_logger().info('3.1 通知mission_manager地图已变更')
        set_map_req = Trigger.Request()
        set_map_res = self.call_service(set_map_client,
                                        set_map_req,
                                        timeout_sec=15.0)

        if set_map_res:
            self.get_logger().info(f'✅ 设置地图变更: success={set_map_res.success}')
            self.get_logger().info(f'   消息: {set_map_res.message}')
        else:
            self.get_logger().error('❌ 设置地图变更失败（超时）')
            self.get_logger().info('   这通常是因为nav_point_manager未响应')
            return False

        # 2. 获取mission_manager状态
        self.get_logger().info('3.2 获取mission_manager状态')
        get_status_req = GetStatus.Request()
        get_status_res = self.call_service(get_status_client,
                                           get_status_req,
                                           timeout_sec=15.0)

        if get_status_res:
            self.get_logger().info(f'✅ 获取状态: success={get_status_res.success}')
            self.get_logger().info(f'   状态: {get_status_res.status}')
            self.get_logger().info(
                f'   当前任务: {get_status_res.name if get_status_res.name else "无"}'
            )
        else:
            self.get_logger().error('❌ 获取状态失败')
            return False

        return True

    def test_add_navigation_task(self, target_point='point_home'):
        """测试添加导航任务"""
        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('测试4: 添加导航任务')
        self.get_logger().info('=' * 50)

        # 检测使用哪个服务（单独启动 vs launch启动）
        if self.add_task_node_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('使用 mission_manager_node/add_task')
            add_task_client = self.add_task_node_client
            get_status_client = self.mission_node_get_status_client
        else:
            self.get_logger().info('使用 mission_manager/add_task')
            add_task_client = self.add_task_client
            get_status_client = self.mission_get_status_client

        # 1. 添加导航任务
        self.get_logger().info(f'4.1 添加导航任务到: {target_point}')
        add_task_req = AddTask.Request()
        add_task_req.task.name = f'nav_to_{target_point}'
        add_task_req.task.type = TaskMsg.NAVIGATION
        add_task_req.task.station_names = [target_point]
        add_task_req.task.priority = 0
        add_task_req.task.waiting_time = 0.0

        add_task_res = self.call_service(add_task_client,
                                         add_task_req,
                                         timeout_sec=15.0)

        if add_task_res:
            self.get_logger().info(f'✅ 添加任务: success={add_task_res.success}')
        else:
            self.get_logger().error('❌ 添加任务失败（超时）')
            return False

        # 2. 等待并检查任务状态
        self.get_logger().info('4.2 等待任务开始执行...')
        time.sleep(2)

        get_status_req = GetStatus.Request()
        get_status_res = self.call_service(get_status_client, get_status_req)

        if get_status_res:
            self.get_logger().info(f'✅ 任务状态: {get_status_res.status}')
            self.get_logger().info(f'   当前任务: {get_status_res.name}')
        else:
            self.get_logger().warn('⚠️  无法获取任务状态')

        return True

    def run_tests(self):
        """运行所有测试"""
        self.get_logger().info('')
        self.get_logger().info('🚀 开始集成测试...')
        self.get_logger().info('')

        # 等待服务
        if not self.wait_for_services():
            self.get_logger().error('❌ 服务不可用，测试中止')
            return False

        time.sleep(1)

        # 测试1: Map Manager
        map_loaded = self.test_map_manager('test_map')

        # 测试2: Nav Point Manager
        nav_ok = self.test_nav_point_manager()

        # 测试3: Mission Manager
        mission_ok = self.test_mission_manager()

        # 测试4: 添加导航任务（如果前面测试都通过）
        task_ok = False
        if nav_ok and mission_ok:
            task_ok = self.test_add_navigation_task('point_home')
        else:
            self.get_logger().warn('⚠️  跳过导航任务测试（前置测试未通过）')

        # 总结
        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('测试总结')
        self.get_logger().info('=' * 50)
        self.get_logger().info(
            f'Map Manager: {"✅ 通过" if map_loaded else "⚠️  部分通过（地图不存在）"}')
        self.get_logger().info(
            f'Nav Point Manager: {"✅ 通过" if nav_ok else "❌ 失败"}')
        self.get_logger().info(
            f'Mission Manager: {"✅ 通过" if mission_ok else "❌ 失败"}')
        self.get_logger().info(
            f'Navigation Task: {"✅ 通过" if task_ok else "⚠️  跳过或失败"}')

        overall = nav_ok and mission_ok
        self.get_logger().info('')
        if overall:
            self.get_logger().info('🎉 集成测试通过！')
        else:
            self.get_logger().error('❌ 集成测试失败')

        return overall


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    tester = IntegrationTester()

    try:
        result = tester.run_tests()
        return 0 if result else 1
    except KeyboardInterrupt:
        tester.get_logger().info('测试被用户中断')
        return 1
    except Exception as e:
        tester.get_logger().error(f'测试出现异常: {e}')
        import traceback
        traceback.print_exc()
        return 1
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
