#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time
import sys
import math

class NavGoalServer(Node):
    def __init__(self):
        super().__init__('nav_goal_server')
        
        # 状态变量
        self.target_pose = None
        self.is_navigating = False
        self.is_canceling = False
        self.pending_goal = None  # 存储待发送的目标
        self.frame_id = 'map'  # 固定使用map坐标系
        
        # 创建订阅器
        self.subscription = self.create_subscription(
            PoseStamped, 
            '/control/nav2_pose', 
            self.goal_callback, 
            10
        )
        
        # 创建Action客户端
        self._action_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose'
        )
        
        # 等待Action服务器
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server connected!")
        
        # 导航超时设置
        self.navigation_timeout = 120.0  # 2分钟超时
        self.navigation_timer = self.create_timer(1.0, self.check_navigation_timeout)
        self.start_time = None
        
        # 目标句柄
        self._goal_handle = None
        self._send_goal_future = None
        self._get_result_future = None
        self._cancel_future = None

    def goal_callback(self, msg):
        """回调函数，接收目标点坐标"""
        # 检查是否为新目标
        if not self.is_new_goal(msg.pose):
            self.get_logger().info("Received same goal, ignoring...")
            return
        
        self.get_logger().info(f"Received new target pose: x={msg.pose.position.x:.2f}, "
                              f"y={msg.pose.position.y:.2f}, "
                              f"theta={self.quaternion_to_yaw(msg.pose.orientation):.2f}")
        
        # 存储新目标
        self.pending_goal = msg.pose
        
        # 如果正在导航，先取消当前导航
        if self.is_navigating:
            if not self.is_canceling:
                self.get_logger().info("Canceling current navigation to accept new goal...")
                self.cancel_current_navigation()
        else:
            # 没有正在进行的导航，直接发送新目标
            self.send_new_goal()
    
    def is_new_goal(self, new_pose):
        """检查是否为新目标点"""
        if self.target_pose is None:
            return True
        
        # 比较位置和方向（允许小误差）
        pos_threshold = 0.01  # 1cm
        ori_threshold = 0.01  # 约0.57度
        
        pos_diff = abs(self.target_pose.position.x - new_pose.position.x) + \
                   abs(self.target_pose.position.y - new_pose.position.y)
        ori_diff = abs(self.target_pose.orientation.z - new_pose.orientation.z) + \
                   abs(self.target_pose.orientation.w - new_pose.orientation.w)
        
        return pos_diff > pos_threshold or ori_diff > ori_threshold
    
    def quaternion_to_yaw(self, q):
        """四元数转换为yaw角"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def cancel_current_navigation(self):
        """取消当前正在进行的导航"""
        if self._goal_handle is None:
            self.get_logger().warn("No goal handle to cancel")
            self.is_navigating = False
            self.send_new_goal()
            return
        
        self.is_canceling = True
        self.get_logger().warn(
            "nav_goal_server is canceling its /navigate_to_pose goal because a new /control/nav2_pose goal arrived"
        )
        
        self._cancel_future = self._goal_handle.cancel_goal_async()
        self._cancel_future.add_done_callback(self.cancel_done_callback)
    
    def cancel_done_callback(self, future):
        """取消操作完成回调"""
        try:
            cancel_response = future.result()
            if len(cancel_response.goals_canceling) > 0:
                self.get_logger().info("Navigation canceled successfully")
            else:
                self.get_logger().warn("Cancel response empty, assuming canceled")
        except Exception as e:
            self.get_logger().error(f"Cancel failed: {str(e)}")
        finally:
            self.is_navigating = False
            self.is_canceling = False
            self._goal_handle = None
            
            # 发送待处理的新目标
            if self.pending_goal is not None:
                self.send_new_goal()

    def send_new_goal(self):
        """发送新的导航目标"""
        if self.pending_goal is None:
            self.get_logger().warn("No pending goal to send")
            return
        
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
        
        self.get_logger().info(f"Sending new goal: x={self.target_pose.position.x:.2f}, "
                              f"y={self.target_pose.position.y:.2f}, "
                              f"theta={self.quaternion_to_yaw(self.target_pose.orientation):.2f}")
        
        # 设置导航状态
        self.is_navigating = True
        self.start_time = time.time()
        
        # 发送目标
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """处理目标响应"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected by navigation server!')
                self.is_navigating = False
                return
            
            self.get_logger().info('Goal accepted by navigation server')
            self._goal_handle = goal_handle
            
            # 获取结果
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)
            
        except Exception as e:
            self.get_logger().error(f"Error in goal response: {str(e)}")
            self.is_navigating = False

    def get_result_callback(self, future):
        """处理最终结果"""
        try:
            result = future.result()
            status = result.status
            
            # Status codes: UNKNOWN=0, ACCEPTED=1, EXECUTING=2, CANCELING=3, 
            #               SUCCEEDED=4, CANCELED=5, ABORTED=6
            if status == 4:  # SUCCEEDED
                self.get_logger().info("Navigation succeeded!")
            elif status == 5:  # CANCELED
                self.get_logger().info("Navigation was canceled")
            elif status == 6:  # ABORTED
                self.get_logger().warn("Navigation aborted!")
            else:
                self.get_logger().warn(f"Navigation ended with status: {status}")
            
        except Exception as e:
            self.get_logger().error(f"Error in result callback: {str(e)}")
        finally:
            self.is_navigating = False
            self._goal_handle = None
            
            # 如果有待处理的目标，发送它
            if self.pending_goal is not None:
                self.get_logger().info("Sending pending goal after completion")
                self.send_new_goal()
            else:
                self.get_logger().info("Navigation completed. Waiting for next goal...")

    def feedback_callback(self, feedback_msg):
        """处理导航反馈"""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        
        if hasattr(feedback, 'estimated_time_remaining'):
            eta = feedback.estimated_time_remaining.sec
            self.get_logger().info(
                f'Progress - Distance: {distance:.2f}m, ETA: {eta}s',
                throttle_duration_sec=2.0
            )
        else:
            self.get_logger().info(
                f'Distance remaining: {distance:.2f}m',
                throttle_duration_sec=2.0
            )
        
        # 更新超时计时器（有反馈说明还在运行）
        self.start_time = time.time()

    def check_navigation_timeout(self):
        """检查导航是否超时"""
        if not self.is_navigating or self.start_time is None:
            return
        
        elapsed = time.time() - self.start_time
        if elapsed > self.navigation_timeout:
            self.get_logger().error(
                f"Navigation timeout after {elapsed:.1f}s (limit: {self.navigation_timeout}s)"
            )
            self.cancel_current_navigation()

def main(args=None):
    rclpy.init(args=args)
    
    nav_goal_server = None
    try:
        nav_goal_server = NavGoalServer()
        nav_goal_server.get_logger().info("Nav goal server started, waiting for goals on /control/nav2_pose")
        rclpy.spin(nav_goal_server)
    except KeyboardInterrupt:
        if nav_goal_server:
            nav_goal_server.get_logger().info("Shutting down by keyboard interrupt")
    except Exception as e:
        if nav_goal_server:
            nav_goal_server.get_logger().error(f"Error in main: {str(e)}")
        else:
            print(f"Error before node creation: {str(e)}")
    finally:
        if nav_goal_server:
            nav_goal_server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
