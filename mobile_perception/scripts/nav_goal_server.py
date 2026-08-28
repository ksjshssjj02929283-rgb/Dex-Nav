#!/usr/bin/env python3
"""接收业务目标并串行转发给 Nav2 NavigateToPose Action。"""

import math
import time
from enum import Enum, auto

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# ==================== 关键配置参数 ====================
GOAL_TOPIC = '/control/nav2_pose'  # 业务侧导航目标Topic
NAV_ACTION_NAME = 'navigate_to_pose'  # Nav2导航Action名称
NAV_FRAME_ID = 'map'  # 转发给Nav2的目标坐标系
ACTION_SERVER_INITIAL_WAIT_S = 10.0  # 启动时等待Action服务器的最长时间，s
ACTION_SERVER_RETRY_PERIOD_S = 2.0  # Action服务器未就绪时的重试周期，s
GOAL_RESPONSE_TIMEOUT_S = 10.0  # 目标请求发出后等待GoalHandle的最长时间，s
CANCEL_COMPLETION_TIMEOUT_S = 15.0  # 取消请求发出后等待目标结束的最长时间，s
NAVIGATION_TIMEOUT_S = 120.0  # 无导航反馈后的取消阈值，s
TIMEOUT_CHECK_PERIOD_S = 1.0  # 导航超时检查周期，s
POSITION_CHANGE_THRESHOLD_M = 0.01  # 新目标位置变化阈值，m
ORIENTATION_CHANGE_THRESHOLD = 0.01  # 四元数分量变化阈值


class NavigationState(Enum):
    """导航目标转发状态，禁止多个布尔量形成非法组合。"""

    IDLE = auto()
    WAITING_GOAL_RESPONSE = auto()
    NAVIGATING = auto()
    CANCELING = auto()


class CancelReason(Enum):
    """取消原因用于稳定日志字段，避免把超时误报成业务更新目标。"""

    NEW_GOAL = 'NEW_GOAL'
    FEEDBACK_TIMEOUT = 'FEEDBACK_TIMEOUT'


class NavGoalServer(Node):

    def __init__(self):
        super().__init__('nav_goal_server')

        # 状态变量
        self.target_pose = None
        self.navigation_state = NavigationState.IDLE
        self.pending_goal = None  # 存储待发送的目标
        self.frame_id = NAV_FRAME_ID

        # Action异步对象先初始化，确保重连和异常路径都能安全访问。
        self._goal_handle = None
        self._send_goal_future = None
        self._get_result_future = None
        self._cancel_future = None
        self._action_server_ready = False
        self._goal_generation = 0
        self._cancel_attempt = 0
        self._cancel_reason = None
        self._state_started_time = None
        self._last_feedback_time = None

        # 创建订阅器
        self.subscription = self.create_subscription(PoseStamped, GOAL_TOPIC,
                                                     self.goal_callback, 10)

        # 创建Action客户端
        self._action_client = ActionClient(self, NavigateToPose,
                                           NAV_ACTION_NAME)

        # 首次等待有明确上限；失败后必须进入spin，由定时器继续发现Nav2。
        self._server_retry_timer = self.create_timer(
            ACTION_SERVER_RETRY_PERIOD_S,
            self._retry_action_server,
        )
        self.get_logger().info(f'等待导航Action服务器：name={NAV_ACTION_NAME}，'
                               f'timeout={ACTION_SERVER_INITIAL_WAIT_S:.1f}s')
        if self._action_client.wait_for_server(
                timeout_sec=ACTION_SERVER_INITIAL_WAIT_S):
            self._set_action_server_ready()
        else:
            self.get_logger().warning(
                f'导航Action服务器在{ACTION_SERVER_INITIAL_WAIT_S:.1f}s内未就绪；'
                f'节点将继续运行并每{ACTION_SERVER_RETRY_PERIOD_S:.1f}s重试')

        self.navigation_timeout = NAVIGATION_TIMEOUT_S
        self.navigation_timer = self.create_timer(TIMEOUT_CHECK_PERIOD_S,
                                                  self.check_navigation_timeout)

    def _set_navigation_state(self, state):
        """统一维护状态进入时间，供各异步阶段执行超时保护。"""
        self.navigation_state = state
        self._state_started_time = (None if state == NavigationState.IDLE else
                                    time.monotonic())

    def _clear_current_goal(self, clear_target):
        """幂等清理当前目标的异步状态；正常结束时保留目标用于重复判定。"""
        self._goal_handle = None
        self._send_goal_future = None
        self._get_result_future = None
        self._cancel_future = None
        self._cancel_reason = None
        if clear_target:
            self.target_pose = None
        self._last_feedback_time = None
        self._set_navigation_state(NavigationState.IDLE)

    def _recover_after_action_server_loss(self, reason):
        """Action服务器消失时丢弃不确定的旧目标，等待重连后只发送新目标。"""
        self._goal_generation += 1
        self._cancel_attempt += 1

        # cancel()只停止本地Future等待；服务器端点已不可见时，旧目标状态无法继续确认。
        for future in (self._send_goal_future, self._get_result_future,
                       self._cancel_future):
            if future is not None and not future.done():
                future.cancel()

        self._clear_current_goal(clear_target=True)
        self._set_action_server_unavailable(reason)
        self.get_logger().error(f'导航Action状态已复位：原因={reason}；不自动重发状态不确定的旧目标')

    def _set_action_server_ready(self):
        """记录服务器已就绪，并发送等待期间收到的最新目标。"""
        was_ready = self._action_server_ready
        self._action_server_ready = True
        self._server_retry_timer.cancel()

        if not was_ready:
            self.get_logger().info(f'导航Action服务器已连接：name={NAV_ACTION_NAME}')

        if (self.pending_goal is not None and
                self.navigation_state == NavigationState.IDLE):
            self.get_logger().info('Action服务器恢复，发送暂存的最新导航目标')
            self.send_new_goal()

    def _set_action_server_unavailable(self, reason):
        """进入周期重试状态；已有待发送目标继续保留。"""
        state_changed = self._action_server_ready
        self._action_server_ready = False
        if self._server_retry_timer.is_canceled():
            self._server_retry_timer.reset()
        if state_changed:
            self.get_logger().warning(f'导航Action服务器连接丢失：原因={reason}；'
                                      f'每{ACTION_SERVER_RETRY_PERIOD_S:.1f}s重试')

    def _retry_action_server(self):
        """非阻塞检查Action服务器，避免构造函数永久等待。"""
        if self._action_client.wait_for_server(timeout_sec=0.0):
            self._set_action_server_ready()

    def goal_callback(self, msg):
        """回调函数，接收目标点坐标"""
        # 检查是否为新目标
        if not self.is_new_goal(msg.pose):
            self.get_logger().info('收到重复导航目标，本次忽略')
            return

        self.get_logger().info(
            f'收到新导航目标：x={msg.pose.position.x:.2f}m，'
            f'y={msg.pose.position.y:.2f}m，'
            f'yaw={self.quaternion_to_yaw(msg.pose.orientation):.2f}rad')

        # 存储新目标
        self.pending_goal = msg.pose

        # 如果正在导航，先取消当前导航
        if self.navigation_state != NavigationState.IDLE:
            if self.navigation_state != NavigationState.CANCELING:
                self.get_logger().info('暂存新目标，先取消当前导航目标')
                self.cancel_current_navigation(CancelReason.NEW_GOAL)
        else:
            # 没有正在进行的导航，直接发送新目标
            self.send_new_goal()

    def is_new_goal(self, new_pose):
        """检查是否为新目标点"""
        # Action服务器未就绪或正在取消时，以待发送目标为比较基准，避免重复覆盖。
        reference_pose = (self.pending_goal if self.pending_goal is not None
                          else self.target_pose)
        if reference_pose is None:
            return True

        # 比较位置和方向（允许小误差）
        pos_diff = (abs(reference_pose.position.x - new_pose.position.x) +
                    abs(reference_pose.position.y - new_pose.position.y))
        ori_diff = (abs(reference_pose.orientation.z - new_pose.orientation.z) +
                    abs(reference_pose.orientation.w - new_pose.orientation.w))

        return (pos_diff > POSITION_CHANGE_THRESHOLD_M or
                ori_diff > ORIENTATION_CHANGE_THRESHOLD)

    def quaternion_to_yaw(self, q):
        """四元数转换为yaw角"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def cancel_current_navigation(self, reason):
        """取消当前正在进行的导航"""
        if self._goal_handle is None:
            if (self._send_goal_future is not None and
                    not self._send_goal_future.done()):
                # 目标请求已经发出但尚未返回句柄，等待响应后再取消，禁止并发发送第二个目标。
                self.get_logger().info('导航目标正在等待Action响应，新目标已暂存')
                return
            self.get_logger().warning('当前没有可取消的导航目标句柄')
            self._set_navigation_state(NavigationState.IDLE)
            self.send_new_goal()
            return

        self._cancel_reason = reason
        self._set_navigation_state(NavigationState.CANCELING)
        self.get_logger().warning(f'正在取消当前导航目标：reason={reason.value}')

        self._request_cancel_current_goal()

    def _request_cancel_current_goal(self):
        """发送或重试当前目标的取消请求；最终结果仍由result回调统一收口。"""
        if self._goal_handle is None:
            return

        try:
            self._cancel_attempt += 1
            cancel_attempt = self._cancel_attempt
            previous_future = self._cancel_future
            self._cancel_future = None
            if previous_future is not None and not previous_future.done():
                # 重试前移除本地旧请求，避免通信长期异常时积累无效Future。
                previous_future.cancel()
            self._cancel_future = self._goal_handle.cancel_goal_async()
            generation = self._goal_generation
            self._cancel_future.add_done_callback(
                lambda future: self.cancel_done_callback(
                    future, generation, cancel_attempt))
        except Exception as ex:
            self._cancel_future = None
            self.get_logger().error(f'创建导航取消请求失败：{str(ex)}')

    def cancel_done_callback(self, future, generation, cancel_attempt):
        """取消操作完成回调"""
        if (generation != self._goal_generation or
                cancel_attempt != self._cancel_attempt):
            return
        try:
            cancel_response = future.result()
            if len(cancel_response.goals_canceling) > 0:
                self.get_logger().info('导航取消请求已受理，等待最终结果')
                # 从取消受理时重新计算最终结果等待时间。
                self._set_navigation_state(NavigationState.CANCELING)
            else:
                self.get_logger().warning('导航取消响应未包含目标，等待当前目标最终结果')
        except Exception as e:
            self.get_logger().error(f'导航取消请求失败：{str(e)}')
        finally:
            if cancel_attempt == self._cancel_attempt:
                self._cancel_future = None

    def send_new_goal(self):
        """发送新的导航目标"""
        if self.pending_goal is None:
            self.get_logger().warning('当前没有待发送的导航目标')
            return

        if not self._action_client.server_is_ready():
            self._set_action_server_unavailable('发送目标前服务器未就绪')
            self.get_logger().warning(
                '导航Action服务器尚未就绪，保留最新目标等待重连',
                throttle_duration_sec=10.0,
            )
            return

        if not self._action_server_ready:
            # 服务器可能在两次重试之间恢复；此处确认后直接进入可用状态。
            self._action_server_ready = True
            self._server_retry_timer.cancel()
            self.get_logger().info(f'导航Action服务器已连接：name={NAV_ACTION_NAME}')

        # 更新当前目标
        self.target_pose = self.pending_goal
        self.pending_goal = None

        # 构造导航目标消息
        goal_msg = NavigateToPose.Goal()
        pose_stamped = PoseStamped()

        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self.frame_id
        pose_stamped.pose = self.target_pose

        goal_msg.pose = pose_stamped

        self.get_logger().info(
            f'发送导航目标：x={self.target_pose.position.x:.2f}m，'
            f'y={self.target_pose.position.y:.2f}m，'
            f'yaw={self.quaternion_to_yaw(self.target_pose.orientation):.2f}rad'
        )

        # 设置导航状态
        self._set_navigation_state(NavigationState.WAITING_GOAL_RESPONSE)
        self._last_feedback_time = None
        self._goal_generation += 1
        generation = self._goal_generation

        try:
            self._send_goal_future = self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=lambda feedback_msg: self.feedback_callback(
                    feedback_msg, generation),
            )
            self._send_goal_future.add_done_callback(
                lambda future: self.goal_response_callback(future, generation))
        except Exception as ex:
            # server_is_ready与真正发送之间仍可能掉线，失败时必须把目标放回待发队列。
            self.pending_goal = self.target_pose
            self.target_pose = None
            self._set_navigation_state(NavigationState.IDLE)
            self._last_feedback_time = None
            self._send_goal_future = None
            self._set_action_server_unavailable('创建导航目标请求失败')
            self.get_logger().error(f'创建导航目标请求失败：{str(ex)}')

    def goal_response_callback(self, future, generation):
        """处理目标响应"""
        if generation != self._goal_generation:
            return
        self._send_goal_future = None
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('导航目标被Nav2拒绝')
                self._set_navigation_state(NavigationState.IDLE)
                self.target_pose = None
                if self.pending_goal is not None:
                    self.send_new_goal()
                return

            self.get_logger().info('导航目标已被Nav2接受')
            self._goal_handle = goal_handle
            self._set_navigation_state(NavigationState.NAVIGATING)
            self._last_feedback_time = time.monotonic()

            # 获取结果
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(
                lambda result_future: self.get_result_callback(
                    result_future, generation))

            # 等待Action响应期间如果收到更新目标，拿到句柄后再按顺序取消旧目标。
            if self.pending_goal is not None:
                self.cancel_current_navigation(CancelReason.NEW_GOAL)

        except Exception as e:
            self.get_logger().error(f'处理导航目标响应失败：{str(e)}')
            self._set_navigation_state(NavigationState.IDLE)
            self._goal_handle = None
            self.target_pose = None
            self._last_feedback_time = None
            if not self._action_client.server_is_ready():
                self._set_action_server_unavailable('发送目标响应异常')
            elif self.pending_goal is not None:
                self.send_new_goal()

    def get_result_callback(self, future, generation):
        """处理最终结果"""
        if generation != self._goal_generation:
            return
        try:
            result = future.result()
            status = result.status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('导航完成：状态=SUCCEEDED')
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('导航完成：状态=CANCELED')
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().warning('导航完成：状态=ABORTED')
            else:
                self.get_logger().warning(f'导航完成：status={status}')

        except Exception as e:
            self.get_logger().error(f'处理导航结果失败：{str(e)}')
            if not self._action_client.server_is_ready():
                self._set_action_server_unavailable('导航结果响应异常')
        finally:
            self._clear_current_goal(clear_target=False)

            # 如果有待处理的目标，发送它
            if self.pending_goal is not None:
                self.get_logger().info('当前目标结束，发送暂存的最新目标')
                self.send_new_goal()
            else:
                self.get_logger().info('当前导航流程结束，等待新目标')

    def feedback_callback(self, feedback_msg, generation):
        """处理导航反馈"""
        if generation != self._goal_generation:
            return
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining

        if hasattr(feedback, 'estimated_time_remaining'):
            eta = feedback.estimated_time_remaining.sec
            self.get_logger().info(f'导航进度：remaining={distance:.2f}m，eta={eta}s',
                                   throttle_duration_sec=2.0)
        else:
            self.get_logger().info(f'导航进度：remaining={distance:.2f}m',
                                   throttle_duration_sec=2.0)

        # 更新超时计时器（有反馈说明还在运行）
        self._last_feedback_time = time.monotonic()

    def check_navigation_timeout(self):
        """检查目标响应、导航反馈和取消完成三个异步阶段是否超时。"""
        now = time.monotonic()
        if self._state_started_time is None:
            return

        state_elapsed = now - self._state_started_time
        if self.navigation_state == NavigationState.WAITING_GOAL_RESPONSE:
            if state_elapsed <= GOAL_RESPONSE_TIMEOUT_S:
                return
            send_future_done = (self._send_goal_future is not None and
                                self._send_goal_future.done())
            if send_future_done:
                # Future已完成但done回调尚未获得执行机会，让回调按正常路径收口。
                return
            if self._action_client.server_is_ready():
                # 服务器端点仍存在时无法判断目标是否已被接受，禁止盲目重发。
                self.get_logger().error(
                    f'导航目标响应超时：elapsed={state_elapsed:.1f}s > '
                    f'limit={GOAL_RESPONSE_TIMEOUT_S:.1f}s；Action端点仍存在，'
                    '保留当前请求并继续等待，禁止重复发送')
                self._state_started_time = now
                return
            self._recover_after_action_server_loss('等待导航目标响应时服务器不可用')
            return

        if self.navigation_state == NavigationState.CANCELING:
            if state_elapsed <= CANCEL_COMPLETION_TIMEOUT_S:
                return
            result_future_done = (self._get_result_future is not None and
                                  self._get_result_future.done())
            if result_future_done:
                return
            if not self._action_client.server_is_ready():
                self._recover_after_action_server_loss('等待导航取消完成时服务器不可用')
                return

            self.get_logger().error(
                f'导航取消等待超时：elapsed={state_elapsed:.1f}s > '
                f'limit={CANCEL_COMPLETION_TIMEOUT_S:.1f}s，'
                f'reason={self._cancel_reason.value if self._cancel_reason else "UNKNOWN"}；'
                'Action端点仍存在，重新发送取消请求')
            self._set_navigation_state(NavigationState.CANCELING)
            self._request_cancel_current_goal()
            return

        if (self.navigation_state != NavigationState.NAVIGATING or
                self._last_feedback_time is None):
            return

        feedback_elapsed = now - self._last_feedback_time
        if feedback_elapsed > self.navigation_timeout:
            self.get_logger().error(
                f'导航反馈超时：elapsed={feedback_elapsed:.1f}s > '
                f'limit={self.navigation_timeout:.1f}s，执行取消')
            self.cancel_current_navigation(CancelReason.FEEDBACK_TIMEOUT)


def main(args=None):
    rclpy.init(args=args)

    nav_goal_server = None
    try:
        nav_goal_server = NavGoalServer()
        nav_goal_server.get_logger().info(f'导航目标转发节点已启动：topic={GOAL_TOPIC}')
        rclpy.spin(nav_goal_server)
    except (KeyboardInterrupt, ExternalShutdownException):
        if nav_goal_server:
            nav_goal_server.get_logger().info('导航目标转发节点正在退出')
    except Exception as e:
        if nav_goal_server:
            nav_goal_server.get_logger().error(f'节点主循环异常：{str(e)}')
        else:
            print(f'节点创建前异常：{str(e)}')
    finally:
        if nav_goal_server:
            nav_goal_server.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            # systemd停止时上下文可能已由其他路径关闭。
            pass


if __name__ == '__main__':
    main()
