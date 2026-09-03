#!/usr/bin/env python3
"""等待 Gazebo Factory 服务和连续仿真时钟，成功后正常退出。"""

import sys
import time

import rclpy
from gazebo_msgs.srv import SpawnEntity
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


# 关键配置：Gazebo 基本就绪后保留短暂稳定期，不再依赖固定模型生成延迟。
CLOCK_TOPIC = '/clock'  # Gazebo 仿真时钟。
SPAWN_SERVICE = '/spawn_entity'  # Gazebo Factory 模型生成服务。
READY_TIMEOUT_S = 30.0  # 最长等待时间，单位：s。
STABILIZATION_S = 0.75  # 服务和时钟同时正常后的稳定时间，单位：s。
CLOCK_STALE_S = 0.3  # 超过该时间未观察到仿真时间增长，视为时钟停止推进。
SPIN_INTERVAL_S = 0.05  # 墙钟轮询周期，单位：s。


class GazeboReadinessState:
    """维护可独立测试的服务、时钟和稳定期状态。"""

    def __init__(self):
        self.last_clock_ns = None
        self.last_clock_advance_wall_time = None
        self.has_clock_advanced = False
        self.stable_since = None

    def update_clock(self, clock_ns, wall_time):
        """记录时钟推进；仿真时间必须至少增长一次。"""
        if self.last_clock_ns is not None:
            if clock_ns > self.last_clock_ns:
                if (self.last_clock_advance_wall_time is not None and
                        wall_time - self.last_clock_advance_wall_time >
                        CLOCK_STALE_S):
                    self.stable_since = None
                self.has_clock_advanced = True
                self.last_clock_advance_wall_time = wall_time
            elif clock_ns < self.last_clock_ns:
                # 仿真时间回退后重新观察连续推进，禁止沿用回退前的就绪状态。
                self.has_clock_advanced = False
                self.last_clock_advance_wall_time = None
                self.stable_since = None
        self.last_clock_ns = clock_ns

    def is_ready(self, service_ready, wall_time):
        """服务存在且时钟连续推进满稳定期后返回 true。"""
        clock_is_advancing = (
            self.has_clock_advanced and
            self.last_clock_advance_wall_time is not None and
            wall_time - self.last_clock_advance_wall_time <= CLOCK_STALE_S)
        if not service_ready or not clock_is_advancing:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = wall_time
            return False
        return wall_time - self.stable_since >= STABILIZATION_S


class GazeboReadyWaiter(Node):
    """检测 Gazebo 模型生成服务和连续仿真时钟。"""

    def __init__(self):
        super().__init__('gazebo_ready_waiter')
        self.state = GazeboReadinessState()
        self.spawn_client = self.create_client(SpawnEntity, SPAWN_SERVICE)
        self.clock_subscription = self.create_subscription(
            Clock, CLOCK_TOPIC, self._on_clock, qos_profile_sensor_data)

    def _on_clock(self, message):
        """使用墙钟记录每次仿真时钟更新。"""
        clock_ns = message.clock.sec * 1_000_000_000 + message.clock.nanosec
        self.state.update_clock(clock_ns, time.monotonic())

    def is_ready(self):
        """返回当前 Gazebo 就绪状态。"""
        return self.state.is_ready(
            self.spawn_client.service_is_ready(), time.monotonic())

    def missing_requirements(self):
        """生成超时时的简明诊断信息。"""
        missing = []
        if not self.spawn_client.service_is_ready():
            missing.append(SPAWN_SERVICE)
        if not self.state.has_clock_advanced:
            missing.append(f'{CLOCK_TOPIC}未连续推进')
        elif (self.state.last_clock_advance_wall_time is None or
              time.monotonic() - self.state.last_clock_advance_wall_time >
              CLOCK_STALE_S):
            missing.append(f'{CLOCK_TOPIC}已停止推进')
        return missing


def main():
    """使用墙钟等待 Gazebo；Ctrl+C 属于正常退出。"""
    rclpy.init()
    node = GazeboReadyWaiter()
    deadline = time.monotonic() + READY_TIMEOUT_S
    exit_code = 0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=SPIN_INTERVAL_S)
            if node.is_ready():
                node.get_logger().info(
                    'Gazebo 已就绪，开始生成机器人模型')
                break
        else:
            if rclpy.ok():
                missing = ', '.join(node.missing_requirements()) or '稳定期'
                node.get_logger().error(
                    f'等待 Gazebo 超时（{READY_TIMEOUT_S:.0f}s）：{missing}')
                exit_code = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
