#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import os
from datetime import datetime


class VelocityOdomRecorder(Node):

    def __init__(self):
        super().__init__('velocity_odom_recorder')

        # 创建速度发布者
        self.cmd_vel_publisher = self.create_publisher(Twist,
                                                       '/control/cmd_vel', 10)

        # 创建里程计订阅者
        self.odom_subscriber = self.create_subscription(Odometry,
                                                        '/feedback/odom',
                                                        self.odom_callback, 10)

        # 初始化数据存储列表
        self.odom_data = []

        # 记录开始时间
        self.start_time = time.time()

        # 设置保存文件路径（脚本所在目录）
        script_dir = os.path.dirname(os.path.abspath(__file__))
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_file = os.path.join(script_dir,
                                        f"odom_record_{timestamp}.txt")

        # 设置时间参数
        self.publish_duration = 100.0  # 发布持续时间：100秒
        self.wait_duration = 20.0  # 等待持续时间：20秒

        # 创建定时器用于发布速度（10 Hz）
        self.publish_timer = self.create_timer(0.01,
                                               self.publish_velocity_callback)

        # 创建控制定时器用于管理整个流程（1 Hz检查）
        self.control_timer = self.create_timer(1.0, self.control_callback)

        # 状态标志
        self.publishing = True
        self.waiting = False

        self.get_logger().info("Velocity odom recorder started.")
        self.get_logger().info(
            f"Will publish velocity for {self.publish_duration} seconds, then wait {self.wait_duration} seconds."
        )

    def publish_velocity_callback(self):
        """定时器回调函数，发布速度命令"""
        if not self.publishing:
            return

        # 创建并发布速度消息
        twist_msg = Twist()
        twist_msg.linear.x = 0.002  # 线速度：0.002 m/s
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0  # 角速度：0.0 rad/s

        self.cmd_vel_publisher.publish(twist_msg)

    def control_callback(self):
        """控制定时器回调，管理整个执行流程"""
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        if self.publishing:
            # 检查是否超过发布持续时间
            if elapsed_time >= self.publish_duration:
                self.publishing = False
                self.publish_timer.cancel()

                # 发布零速度以确保停止
                stop_msg = Twist()
                self.cmd_vel_publisher.publish(stop_msg)

                self.get_logger().info(
                    f"Velocity publishing completed after {self.publish_duration} seconds."
                )
                self.get_logger().info(
                    f"Total odom records so far: {len(self.odom_data)}")
                self.get_logger().info(
                    f"Waiting {self.wait_duration} seconds while continuing to record odom..."
                )

                # 记录等待开始时间
                self.wait_start_time = current_time
                self.waiting = True

        elif self.waiting:
            # 检查是否超过等待持续时间
            wait_elapsed = current_time - self.wait_start_time
            if wait_elapsed >= self.wait_duration:
                self.waiting = False
                self.control_timer.cancel()

                self.get_logger().info(
                    f"Wait completed. Final total odom records: {len(self.odom_data)}"
                )

                # 保存数据到文件
                self.save_odom_data_to_file()

                self.get_logger().info(
                    "Script execution completed. Shutting down...")

                # 触发关闭
                rclpy.shutdown()

    def odom_callback(self, msg):
        """里程计回调函数，记录速度和位置信息"""
        # 提取位置信息
        position = msg.pose.pose.position
        # 提取速度信息
        linear_vel = msg.twist.twist.linear
        angular_vel = msg.twist.twist.angular

        # 记录数据
        odom_record = {
            'timestamp': time.time(),
            'position': {
                'x': position.x,
                'y': position.y,
                'z': position.z
            },
            'velocity': {
                'linear': {
                    'x': linear_vel.x,
                    'y': linear_vel.y,
                    'z': linear_vel.z
                },
                'angular': {
                    'x': angular_vel.x,
                    'y': angular_vel.y,
                    'z': angular_vel.z
                }
            }
        }

        self.odom_data.append(odom_record)

        # 每隔一段时间打印一次（避免日志过多）
        if len(self.odom_data) % 10 == 0:
            self.get_logger().info(
                f"Recorded odom data: position=({position.x:.3f}, {position.y:.3f}), "
                f"linear_vel=({linear_vel.x:.6f}, {linear_vel.y:.6f}), "
                f"angular_vel.z={angular_vel.z:.6f}")

    def save_odom_data_to_file(self):
        """将里程计数据保存到txt文件"""
        try:
            with open(self.output_file, 'w', encoding='utf-8') as f:
                # 写入文件头
                f.write("# Odometry Data Record\n")
                f.write(f"# Total records: {len(self.odom_data)}\n")
                f.write(
                    f"# Start time: {datetime.fromtimestamp(self.start_time).strftime('%Y-%m-%d %H:%M:%S')}\n"
                )
                f.write(
                    "# Format: timestamp, pos_x, pos_y, pos_z, lin_vel_x, lin_vel_y, lin_vel_z, ang_vel_x, ang_vel_y, ang_vel_z\n"
                )
                f.write("# " + "=" * 80 + "\n\n")

                # 写入数据
                for record in self.odom_data:
                    timestamp = record['timestamp']
                    pos = record['position']
                    lin_vel = record['velocity']['linear']
                    ang_vel = record['velocity']['angular']

                    # 格式化输出：时间戳、位置、线速度、角速度
                    f.write(
                        f"{timestamp:.6f}\t"
                        f"{pos['x']:.6f}\t{pos['y']:.6f}\t{pos['z']:.6f}\t"
                        f"{lin_vel['x']:.6f}\t{lin_vel['y']:.6f}\t{lin_vel['z']:.6f}\t"
                        f"{ang_vel['x']:.6f}\t{ang_vel['y']:.6f}\t{ang_vel['z']:.6f}\n"
                    )

            self.get_logger().info(f"Odom data saved to: {self.output_file}")
            self.get_logger().info(
                f"Total records saved: {len(self.odom_data)}")
            return True
        except Exception as e:
            self.get_logger().error(
                f"Failed to save odom data to file: {str(e)}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = VelocityOdomRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Script stopped by user.")
        # 如果用户中断，也保存已记录的数据
        if len(node.odom_data) > 0:
            node.save_odom_data_to_file()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
