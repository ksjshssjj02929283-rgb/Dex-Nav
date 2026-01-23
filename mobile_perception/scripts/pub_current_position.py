#!/usr/bin/env python3
import rclpy
import rclpy.duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import rclpy.time
from tf2_ros import Buffer, TransformListener
import time


class BaseLinkPosition(Node):

    def __init__(self):
        super().__init__("baselink_position")
        # 设置TF buffer cache时间为10秒，减少内存占用和提高查询效率
        self.tf2_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.listener = TransformListener(self.tf2_buffer, self)

        self.publisher = self.create_publisher(PoseStamped, '/feedback/chassis_pose', 10)
        # 不使用定时器，改用手动控制循环频率

        # 错误计数器，避免频繁日志输出
        self.error_count = 0
        self.error_log_interval = 50  # 每50次错误才记录一次日志

        # 发布频率控制（Hz）
        self.publish_rate = 2.0  # 2Hz，可根据需要调整

        self.get_logger().info("Publish Chassis Position! ")

    def get_position(self):
        try:
            # 直接查询TF，使用最短timeout，避免can_transform的额外开销
            transform = self.tf2_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)  # 进一步缩短timeout
            )

            translation = transform.transform.translation
            orientation = transform.transform.rotation

            pose_msg = PoseStamped()
            pose_msg.header.frame_id = 'map'
            pose_msg.header.stamp = self.get_clock().now().to_msg()

            pose_msg.pose.position.x = translation.x
            pose_msg.pose.position.y = translation.y
            pose_msg.pose.position.z = 0.0

            pose_msg.pose.orientation = orientation
            self.publisher.publish(pose_msg)

            # 成功时重置错误计数
            if self.error_count > 0:
                self.error_count = 0

        except Exception as e:
            self.error_count += 1
            # 减少错误日志频率，避免频繁输出
            if self.error_count % self.error_log_interval == 0:
                self.get_logger().error(f'TF error after {self.error_count} attempts: {str(e)}')
            # 静默失败，不阻塞主循环


def main(args=None):
    rclpy.init(args=args)
    node = BaseLinkPosition()

    # 计算休眠时间（秒）
    sleep_duration = 1.0 / node.publish_rate

    try:
        # 使用spin_once配合休眠，比spin更高效，避免持续轮询
        while rclpy.ok():
            # 非阻塞处理ROS2事件（如TF订阅），超时时间很短
            rclpy.spin_once(node, timeout_sec=0.005)  # 5ms超时，足够处理一次事件

            # 执行位置查询和发布
            node.get_position()

            # 精确控制频率，使用系统sleep而非ROS2定时器，降低CPU占用
            time.sleep(sleep_duration)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
