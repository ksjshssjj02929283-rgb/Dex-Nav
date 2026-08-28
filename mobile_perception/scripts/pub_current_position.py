#!/usr/bin/env python3
"""以固定频率发布底盘在 map 坐标系中的位姿。"""

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

# ==================== 关键配置参数 ====================
POSE_TOPIC = '/feedback/chassis_pose'  # 底盘位姿输出Topic，下游按约10 Hz接收
TARGET_FRAME = 'map'  # 输出位姿所在坐标系
SOURCE_FRAME = 'base_link'  # 机器人本体坐标系
PUBLISH_PERIOD_S = 0.1  # 正常发布周期，s；0.1 s对应10 Hz
TF_CACHE_DURATION_S = 10.0  # TF Buffer缓存时间，s
POSE_CACHE_MAX_AGE_S = 0.2  # TF短暂失败时允许复用位姿的最长时间，s
BACKOFF_START_FAILURES = 50  # 连续失败达到该次数后开始降低TF查询频率
BACKOFF_MAX_PERIOD_S = 1.0  # 故障退避后的最大查询周期，s；即最低1 Hz
PUBLISH_QOS_DEPTH = 10  # 可靠发布队列深度


class BaseLinkPosition(Node):

    def __init__(self):
        super().__init__('baselink_position')

        self.tf2_buffer = Buffer(cache_time=Duration(
            seconds=TF_CACHE_DURATION_S))
        self.listener = TransformListener(self.tf2_buffer, self)

        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=PUBLISH_QOS_DEPTH,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(PoseStamped, POSE_TOPIC,
                                               pose_qos)

        self.failure_count = 0
        self.retry_period_s = PUBLISH_PERIOD_S
        self.next_lookup_time = 0.0
        self.last_pose_msg = None
        self.last_pose_time = 0.0

        # 使用ROS Timer统一调度，使Executor能持续处理TF回调，避免spin_once造成回调饥饿。
        self.position_timer = self.create_timer(PUBLISH_PERIOD_S,
                                                self._position_timer_callback)
        self.get_logger().info(f'底盘位姿发布节点已启动：topic={POSE_TOPIC}，'
                               f'frequency={1.0 / PUBLISH_PERIOD_S:.1f}Hz，'
                               f'frame={TARGET_FRAME} <- {SOURCE_FRAME}')

    def _build_pose_msg(self, translation, orientation):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = TARGET_FRAME
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = translation.x
        pose_msg.pose.position.y = translation.y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = orientation
        return pose_msg

    def _publish_cached_pose(self, now_monotonic):
        """TF瞬时失败时短暂沿用缓存，超过有效期后停止发布旧位姿。"""
        if self.last_pose_msg is None:
            return False
        if now_monotonic - self.last_pose_time > POSE_CACHE_MAX_AGE_S:
            return False

        cached_pose = PoseStamped()
        cached_pose.header.frame_id = self.last_pose_msg.header.frame_id
        cached_pose.header.stamp = self.get_clock().now().to_msg()
        cached_pose.pose = self.last_pose_msg.pose
        self.publisher.publish(cached_pose)
        return True

    def _record_tf_failure(self, now_monotonic, reason):
        self.failure_count += 1
        previous_retry_period = self.retry_period_s

        if self.failure_count >= BACKOFF_START_FAILURES:
            self.retry_period_s = min(
                self.retry_period_s * 2.0,
                BACKOFF_MAX_PERIOD_S,
            )
        else:
            self.retry_period_s = PUBLISH_PERIOD_S

        self.next_lookup_time = now_monotonic + self.retry_period_s
        self._publish_cached_pose(now_monotonic)

        # 仅在退避级别变化时记录一次，避免定位故障期间刷屏并额外消耗CPU。
        if self.retry_period_s != previous_retry_period:
            self.get_logger().warning(
                f'TF连续失败：count={self.failure_count}，原因={reason}，'
                f'查询周期调整为{self.retry_period_s:.1f}s')

    def _record_tf_success(self):
        if self.failure_count >= BACKOFF_START_FAILURES:
            self.get_logger().info(f'TF已恢复：连续失败{self.failure_count}次，查询频率恢复为'
                                   f'{1.0 / PUBLISH_PERIOD_S:.1f}Hz')
        self.failure_count = 0
        self.retry_period_s = PUBLISH_PERIOD_S
        self.next_lookup_time = 0.0

    def _position_timer_callback(self):
        now_monotonic = time.monotonic()
        if now_monotonic < self.next_lookup_time:
            return

        # timeout=0保证查询不阻塞Executor，TF订阅回调可以持续被处理。
        if not self.tf2_buffer.can_transform(
                TARGET_FRAME, SOURCE_FRAME, Time(),
                timeout=Duration(seconds=0.0)):
            self._record_tf_failure(now_monotonic, 'TF尚未就绪')
            return

        try:
            transform = self.tf2_buffer.lookup_transform(
                TARGET_FRAME,
                SOURCE_FRAME,
                Time(),
                timeout=Duration(seconds=0.0),
            )
        except TransformException as ex:
            # can_transform与lookup_transform之间TF仍可能变化，因此保留异常保护。
            self._record_tf_failure(now_monotonic, str(ex))
            return

        pose_msg = self._build_pose_msg(
            transform.transform.translation,
            transform.transform.rotation,
        )
        self.publisher.publish(pose_msg)
        self.last_pose_msg = pose_msg
        self.last_pose_time = now_monotonic
        self._record_tf_success()


def main(args=None):
    rclpy.init(args=args)
    node = BaseLinkPosition()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            # systemd停止时上下文可能已由其他路径关闭。
            pass


if __name__ == '__main__':
    main()
