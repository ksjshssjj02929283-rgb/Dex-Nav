#!/usr/bin/env python3
"""等待差速仿真 Odom 和已启用传感器话题，成功后正常退出。"""

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan


# 关键配置：确认底盘、动态 TF 和已启用的感知链路都已开始工作。
ODOM_TOPIC = '/feedback/odom'  # 差速底盘里程计话题。
READY_TIMEOUT_S = 45.0  # 最长等待时间；超时后终止整套仿真启动。
SPIN_INTERVAL_S = 0.1  # 墙钟轮询周期，不依赖尚未稳定的仿真时钟。

SENSOR_TOPICS = {
    'rgbd_front': (
        ('/orbbec/front/rgb', Image),
        ('/orbbec/front/depth', Image),
        ('/orbbec/front/camera_info', CameraInfo),
    ),
    'rgbd_rear': (
        ('/orbbec/rear/rgb', Image),
        ('/orbbec/rear/depth', Image),
        ('/orbbec/rear/camera_info', CameraInfo),
    ),
    'lidar_front': (('/laser_scan/front', LaserScan),),
    'lidar_rear': (('/laser_scan/rear', LaserScan),),
    'imu': (('/imu/chassis', Imu),),
}

# RGBD 转换节点启动后会立即创建这些发布者；只检查发布链路存在，
# 不要求场景中必须提取到障碍物才能启动 Nav2。
SENSOR_OUTPUT_TOPICS = {
    'rgbd_front': ('/orbbec/front/laser_scan',),
    'rgbd_rear': ('/orbbec/rear/laser_scan',),
}


def required_topics(enabled_sensors):
    """根据 YAML 传感器开关生成必须收到的正式话题。"""
    topics = {ODOM_TOPIC: Odometry}
    for sensor_name, topic_specs in SENSOR_TOPICS.items():
        if enabled_sensors[sensor_name]:
            topics.update(topic_specs)
    return topics


def required_output_topics(enabled_sensors):
    """返回已启用 RGBD 对应的 Nav2 派生输出话题。"""
    return {
        topic
        for sensor_name, topics in SENSOR_OUTPUT_TOPICS.items()
        if enabled_sensors[sensor_name]
        for topic in topics
    }


class SimReadyWaiter(Node):
    """等待有效 Odom 和所有已启用的传感器话题。"""

    def __init__(self):
        super().__init__('sim_ready_waiter')
        enabled_sensors = {}
        for sensor_name in SENSOR_TOPICS:
            self.declare_parameter(f'require_{sensor_name}', True)
            enabled_sensors[sensor_name] = self.get_parameter(
                f'require_{sensor_name}').value

        self.received_topics = set()
        self.required_topics = required_topics(enabled_sensors)
        self._required_output_topics = required_output_topics(enabled_sensors)
        # Node.subscriptions 是 rclpy 的只读属性，使用私有成员持有订阅对象。
        self._topic_subscriptions = []
        for topic, message_type in self.required_topics.items():
            callback = (
                self._on_odom if topic == ODOM_TOPIC else
                lambda _message, received_topic=topic:
                self.received_topics.add(received_topic))
            self._topic_subscriptions.append(self.create_subscription(
                message_type, topic, callback, qos_profile_sensor_data))

    @property
    def is_ready(self):
        """原始数据已到达且派生输出发布链路存在后返回 true。"""
        return (
            set(self.required_topics).issubset(self.received_topics) and
            not self.missing_output_publishers())

    @staticmethod
    def odom_is_valid(odom):
        """检查 Odom 位姿是否均为有限数值。"""
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        values = (
            position.x, position.y, position.z,
            orientation.x, orientation.y, orientation.z, orientation.w)
        return all(math.isfinite(value) for value in values)

    def _on_odom(self, odom):
        """过滤 NaN/Inf，防止物理模型异常时误启动 Nav2。"""
        if not self.odom_is_valid(odom):
            return
        self.received_topics.add(ODOM_TOPIC)

    def missing_topics(self):
        """返回尚未收到的必需话题。"""
        return sorted(set(self.required_topics) - self.received_topics)

    def missing_output_publishers(self):
        """返回尚未发现发布者的 RGBD 派生输出话题。"""
        return sorted(
            topic for topic in self._required_output_topics
            if not self.get_publishers_info_by_topic(topic))


def main():
    """使用墙钟等待 Odom；Ctrl+C 属于正常退出。"""
    rclpy.init()
    node = SimReadyWaiter()
    deadline = time.monotonic() + READY_TIMEOUT_S
    exit_code = 0
    try:
        while (rclpy.ok() and not node.is_ready and
               time.monotonic() < deadline):
            rclpy.spin_once(node, timeout_sec=SPIN_INTERVAL_S)

        if node.is_ready:
            node.get_logger().info('仿真接口已就绪，开始启动 Nav2 和 RViz')
        elif rclpy.ok():
            missing = [
                *node.missing_topics(),
                *(f'{topic}（无发布者）'
                  for topic in node.missing_output_publishers()),
            ]
            node.get_logger().error(
                f'等待仿真话题超时（{READY_TIMEOUT_S:.0f}s）：'
                f'{", ".join(missing)}')
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
