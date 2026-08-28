#!/usr/bin/env python3
"""以固定频率发布底盘在 map 坐标系中的位姿。"""

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_interfaces.msg import LocalizationState
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

# ==================== 关键配置参数 ====================
POSE_TOPIC = '/feedback/chassis_pose'  # 底盘位姿输出Topic，下游按约10 Hz接收
LOCALIZATION_STATE_TOPIC = '/localization/state'  # 定位状态Topic，只有定位成功才允许发布
FILTERED_ODOM_TOPIC = '/localization/odometry/filtered'  # map下的融合位姿输入Topic
TARGET_FRAME = 'map'  # 输出和filtered Odom必须使用的坐标系
ALLOWED_ODOM_CHILD_FRAMES = ('base_link', 'base_footprint')  # 允许作为二维底盘位姿的子坐标系
PUBLISH_PERIOD_S = 0.1  # 正常发布周期，s；0.1 s对应10 Hz
LOCALIZATION_STATE_MAX_AGE_S = 0.5  # 定位状态最大允许间隔，s；超时后停止发布
ODOM_MAX_AGE_S = 0.2  # filtered Odom最大允许间隔，s；超时后停止发布
PUBLISH_QOS_DEPTH = 10  # 保持原接口的可靠发布队列深度
SUBSCRIPTION_QOS_DEPTH = 1  # 输入只使用最新值，避免积压历史消息

# 默认走低CPU的filtered Odom链路；启动时附加--ros-args -p pose_source:=tf可恢复原TF模式。
DEFAULT_POSE_SOURCE = 'filtered_odom'
FILTERED_ODOM_POSE_SOURCE = 'filtered_odom'
TF_POSE_SOURCE = 'tf'

# 以下配置仅在pose_source=tf回退模式下生效。
SOURCE_FRAME = 'base_link'  # TF回退模式下的机器人本体坐标系
TF_CACHE_DURATION_S = 10.0  # TF Buffer缓存时间，s
POSE_CACHE_MAX_AGE_S = 0.2  # TF短暂失败时允许复用位姿的最长时间，s
BACKOFF_START_FAILURES = 50  # 连续失败达到该次数后开始降低TF查询频率
BACKOFF_MAX_PERIOD_S = 1.0  # 故障退避后的最大查询周期，s；即最低1 Hz


def _normalize_frame_id(frame_id):
    """统一去除坐标系前导斜杠，避免等价frame被误判。"""
    return frame_id.lstrip('/')


class BaseLinkPosition(Node):

    def __init__(self):
        super().__init__('baselink_position')

        self.pose_source = str(
            self.declare_parameter('pose_source',
                                   DEFAULT_POSE_SOURCE).value).strip().lower()
        if self.pose_source not in (FILTERED_ODOM_POSE_SOURCE, TF_POSE_SOURCE):
            self.get_logger().warning(f'位姿源参数无效：pose_source={self.pose_source}，'
                                      f'回退为{DEFAULT_POSE_SOURCE}')
            self.pose_source = DEFAULT_POSE_SOURCE

        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=PUBLISH_QOS_DEPTH,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(PoseStamped, POSE_TOPIC,
                                               pose_qos)

        if self.pose_source == FILTERED_ODOM_POSE_SOURCE:
            self._initialize_filtered_odom_source()
        else:
            self._initialize_tf_source()

        self.position_timer = self.create_timer(PUBLISH_PERIOD_S,
                                                self._position_timer_callback)
        self.get_logger().info(f'底盘位姿发布节点已启动：topic={POSE_TOPIC}，'
                               f'frequency={1.0 / PUBLISH_PERIOD_S:.1f}Hz，'
                               f'source={self.pose_source}')

    def _initialize_filtered_odom_source(self):
        """创建低CPU输入链路，不再常驻订阅整条TF数据流。"""
        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=SUBSCRIPTION_QOS_DEPTH,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.is_localized = False
        self.localization_status = None
        self.localization_state_time = 0.0
        self.is_localization_state_stale = False
        self.localization_generation = 0

        self.latest_odom = None
        self.latest_odom_time = 0.0
        self.odom_generation = -1
        self.is_odom_stale = False
        self.invalid_frame_signature = None

        self.localization_state_sub = self.create_subscription(
            LocalizationState,
            LOCALIZATION_STATE_TOPIC,
            self._localization_state_callback,
            input_qos,
        )
        self.filtered_odom_sub = self.create_subscription(
            Odometry,
            FILTERED_ODOM_TOPIC,
            self._filtered_odom_callback,
            input_qos,
        )

    def _initialize_tf_source(self):
        """保留原TF实现，便于异常时通过参数快速回滚。"""
        self.tf2_buffer = Buffer(cache_time=Duration(
            seconds=TF_CACHE_DURATION_S))
        self.listener = TransformListener(self.tf2_buffer, self)

        self.failure_count = 0
        self.retry_period_s = PUBLISH_PERIOD_S
        self.next_lookup_time = 0.0
        self.last_pose_msg = None
        self.last_pose_time = 0.0

    def _clear_odom_cache(self):
        self.latest_odom = None
        self.latest_odom_time = 0.0
        self.odom_generation = -1
        self.is_odom_stale = False

    def _localization_state_callback(self, msg):
        now_monotonic = time.monotonic()
        previous_status = self.localization_status
        was_localized = self.is_localized
        recovered_from_stale = self.is_localization_state_stale

        self.localization_status = msg.status
        self.localization_state_time = now_monotonic
        self.is_localization_state_stale = False
        self.is_localized = msg.status == LocalizationState.LOC_SUCCESS

        state_changed = self.is_localized != was_localized
        if state_changed or recovered_from_stale:
            # 定位重新有效后必须等待新Odom，防止发布定位中断前的旧位姿。
            self.localization_generation += 1
            self._clear_odom_cache()

        if self.is_localized and (state_changed or recovered_from_stale):
            self.get_logger().info('定位状态有效，等待本次定位后的新filtered Odom')
        elif not self.is_localized and (was_localized or
                                        msg.status != previous_status):
            self.get_logger().warning(f'定位状态无效：status={msg.status}，停止发布底盘位姿')

    def _filtered_odom_callback(self, msg):
        if not self.is_localized or self.is_localization_state_stale:
            return

        frame_id = _normalize_frame_id(msg.header.frame_id)
        child_frame_id = _normalize_frame_id(msg.child_frame_id)
        frame_is_valid = frame_id == TARGET_FRAME
        child_frame_is_valid = child_frame_id in ALLOWED_ODOM_CHILD_FRAMES

        if not frame_is_valid or not child_frame_is_valid:
            self._clear_odom_cache()
            signature = (frame_id, child_frame_id)
            if signature != self.invalid_frame_signature:
                self.get_logger().warning(
                    'filtered Odom坐标系不符合要求：'
                    f'frame_id={frame_id or "<empty>"}，'
                    f'child_frame_id={child_frame_id or "<empty>"}，停止发布')
                self.invalid_frame_signature = signature
            return

        was_ready = (self.latest_odom is not None and
                     self.odom_generation == self.localization_generation)
        self.latest_odom = msg
        self.latest_odom_time = time.monotonic()
        self.odom_generation = self.localization_generation
        self.is_odom_stale = False
        self.invalid_frame_signature = None

        if not was_ready:
            self.get_logger().info(
                f'filtered Odom已就绪：frame={frame_id}，child={child_frame_id}')

    def _build_pose_msg(self, position, orientation):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = TARGET_FRAME
        # 保持原接口行为：发布时刻使用当前ROS时间，二维底盘位姿固定z=0。
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = position.x
        pose_msg.pose.position.y = position.y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = orientation
        return pose_msg

    def _publish_filtered_odom_pose(self):
        now_monotonic = time.monotonic()
        if not self.is_localized:
            return

        state_age_s = now_monotonic - self.localization_state_time
        if state_age_s > LOCALIZATION_STATE_MAX_AGE_S:
            if not self.is_localization_state_stale:
                self.is_localization_state_stale = True
                self.localization_generation += 1
                self._clear_odom_cache()
                self.get_logger().warning(
                    f'定位状态超时：age={state_age_s:.3f}s > '
                    f'{LOCALIZATION_STATE_MAX_AGE_S:.3f}s，停止发布底盘位姿')
            return

        if (self.latest_odom is None or
                self.odom_generation != self.localization_generation):
            return

        odom_age_s = now_monotonic - self.latest_odom_time
        if odom_age_s > ODOM_MAX_AGE_S:
            if not self.is_odom_stale:
                self.is_odom_stale = True
                self.get_logger().warning(
                    f'filtered Odom超时：age={odom_age_s:.3f}s > '
                    f'{ODOM_MAX_AGE_S:.3f}s，停止发布底盘位姿')
            return

        pose_msg = self._build_pose_msg(
            self.latest_odom.pose.pose.position,
            self.latest_odom.pose.pose.orientation,
        )
        self.publisher.publish(pose_msg)

    def _publish_cached_tf_pose(self, now_monotonic):
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
        self._publish_cached_tf_pose(now_monotonic)

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

    def _publish_tf_pose(self):
        now_monotonic = time.monotonic()
        if now_monotonic < self.next_lookup_time:
            return

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

    def _position_timer_callback(self):
        if self.pose_source == FILTERED_ODOM_POSE_SOURCE:
            self._publish_filtered_odom_pose()
        else:
            self._publish_tf_pose()


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
