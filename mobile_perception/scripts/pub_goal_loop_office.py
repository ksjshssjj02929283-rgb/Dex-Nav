#!/usr/bin/env python3
# import rclpy
# from rclpy.action import ActionClient
# from rclpy.node import Node
# from nav2_msgs.action import NavigateToPose
# from geometry_msgs.msg import PoseStamped, Point, Quaternion
# import math
# import time
# import threading
# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# from rclpy.executors import MultiThreadedExecutor

# class Nav2GoalSender(Node):
#     def __init__(self):
#         super().__init__('nav2_goal_sender')
        
#         # 使用独立的回调组避免阻塞
#         self.action_callback_group = MutuallyExclusiveCallbackGroup()
        
#         # 创建NavigateToPose动作客户端
#         self.action_client = ActionClient(
#             self, 
#             NavigateToPose, 
#             'navigate_to_pose',
#             callback_group=self.action_callback_group
#         )
        
#         self.get_logger().info('等待NavigateToPose动作服务器...')
#         self.action_client.wait_for_server()
#         self.get_logger().info('动作服务器已连接')
        
#         # 创建两个目标位姿
#         self.goal1 = self.create_pose(-1.3, 21.0, 0.0, 0.0)  # 位置(0,0,0) 方向180度
#         self.goal2 = self.create_pose(22.0, 21.0, 0.0, -90.0)    # 位置(0.9,0,0) 方向0度
#         self.goal3 = self.create_pose(20.0, 0.0, 0.0, 180.0)
#         self.goal4 = self.create_pose(-1.0, 0.0, 0.0, 90.0)
                
#         # 当前目标计数器
#         self.current_goal = self.goal1
#         self.goal_count = 0
#         self.goal_handle = None
#         self.is_navigating = False

#         # 启动第一个目标
#         self.send_goal(self.current_goal)

#     def create_pose(self, x, y, z, yaw_deg):
#         """创建带有指定位置和偏航角的位姿"""
#         pose = PoseStamped()
#         pose.header.frame_id = 'map'  # 坐标系
        
#         # 设置位置
#         pose.pose.position = Point(x=x, y=y, z=z)
        
#         # 将偏航角转换为四元数
#         yaw_rad = math.radians(yaw_deg)
#         qz = math.sin(yaw_rad / 2)
#         qw = math.cos(yaw_rad / 2)
#         pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        
#         return pose

#     def send_goal(self, goal_pose):
#         """发送导航目标到动作服务器"""
#         if self.is_navigating:
#             self.get_logger().warn('已有导航任务进行中，跳过新目标发送')
#             return
            
#         self.is_navigating = True
#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose = goal_pose
        
#         # 记录发送信息
#         pos = goal_pose.pose.position
#         orient = goal_pose.pose.orientation
#         self.get_logger().info('发送目标 #{}: 位置({:.1f}, {:.1f}, {:.1f}) 方向(z: {:.2f}, w: {:.2f})'.format(
#             self.goal_count + 1,
#             pos.x, pos.y, pos.z,
#             orient.z, orient.w
#         ))
        
#         # 异步发送目标
#         send_goal_future = self.action_client.send_goal_async(
#             goal_msg, 
#             feedback_callback=self.feedback_callback
#         )
        
#         # 添加完成回调
#         send_goal_future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         """处理目标响应"""
#         self.goal_handle = future.result()
#         if not self.goal_handle.accepted:
#             self.get_logger().warn('目标被拒绝!')
#             self.is_navigating = False
#             return
        
#         self.get_logger().info('目标已接受，正在导航...')
        
#         # 获取结果回调
#         result_future = self.goal_handle.get_result_async()
#         result_future.add_done_callback(self.get_result_callback)

#     def get_result_callback(self, future):
#         """处理导航结果"""
#         try:
#             result = future.result().result
#             self.get_logger().info(f'导航完成! 结果: {result}')
            
#             # 导航完成后等待一段时间再发送下一个目标
#             self.get_logger().info('导航完成，等待10秒后发送下一个目标...')
            
#             # 使用线程等待而不阻塞主线程
#             wait_thread = threading.Thread(target=self.wait_and_send_next)
#             wait_thread.daemon = True
#             wait_thread.start()
            
#         except Exception as e:
#             self.get_logger().error(f'获取结果时出错: {str(e)}')
#         finally:
#             self.is_navigating = False

#     def wait_and_send_next(self):
#         """等待一段时间后发送下一个目标"""
#         # 等待10秒
#         time.sleep(10.0)
        
#         # 切换到下一个目标
#         self.goal_count += 1
#         self.id = self.goal_count % 4
#         if self.id == 0:
#             self.current_goal = self.goal1
#         elif self.id == 1:
#             self.current_goal = self.goal2
#         elif self.id == 2:
#             self.current_goal = self.goal3
#         else:
#             self.current_goal = self.goal4
            
#         # 在节点上下文中发送新目标
#         self.create_timer(0.0, lambda: self.send_goal(self.current_goal))

#     def feedback_callback(self, feedback_msg):
#         """处理导航反馈"""
#         feedback = feedback_msg.feedback
#         # 可以在这里处理反馈信息，如显示当前位置
#         self.get_logger().info(f'导航进度: {feedback.distance_remaining:.2f}米')

# def main(args=None):
#     rclpy.init(args=args)
#     nav2_goal_sender = Nav2GoalSender()
    
#     # 使用多线程执行器处理动作客户端
#     executor = MultiThreadedExecutor(num_threads=2)
#     executor.add_node(nav2_goal_sender)
    
#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         # 销毁节点并关闭ROS
#         nav2_goal_sender.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import math
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__('nav2_goal_sender')
        
        # 使用独立的回调组避免阻塞
        self.action_callback_group = MutuallyExclusiveCallbackGroup()
        
        # 创建NavigateToPose动作客户端
        self.action_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose',
            callback_group=self.action_callback_group
        )
        
        self.get_logger().info('等待NavigateToPose动作服务器...')
        self.action_client.wait_for_server()
        self.get_logger().info('动作服务器已连接')
        
        # 创建两个目标位姿
        self.goal1 = self.create_pose(-1.3, 21.0, 0.0, 0.0)  # 位置(0,0,0) 方向180度
        self.goal2 = self.create_pose(22.0, 21.0, 0.0, -90.0)    # 位置(0.9,0,0) 方向0度
        self.goal3 = self.create_pose(20.0, 0.0, 0.0, 180.0)
        self.goal4 = self.create_pose(-1.0, 0.0, 0.0, 90.0)
                
        # 当前目标计数器
        self.current_goal = self.goal1
        self.goal_count = 0
        self.goal_handle = None
        self.is_navigating = False
        self.wait_timer = None  # 添加等待定时器

        # 启动第一个目标
        self.send_goal(self.current_goal)

    def create_pose(self, x, y, z, yaw_deg):
        """创建带有指定位置和偏航角的位姿"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'  # 坐标系
        
        # 设置位置
        pose.pose.position = Point(x=x, y=y, z=z)
        
        # 将偏航角转换为四元数
        yaw_rad = math.radians(yaw_deg)
        qz = math.sin(yaw_rad / 2)
        qw = math.cos(yaw_rad / 2)
        pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        
        return pose

    def send_goal(self, goal_pose):
        """发送导航目标到动作服务器"""
        if self.is_navigating:
            self.get_logger().warn('已有导航任务进行中，跳过新目标发送')
            return
            
        self.is_navigating = True
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        # 记录发送信息
        pos = goal_pose.pose.position
        orient = goal_pose.pose.orientation
        self.get_logger().info('发送目标 #{}: 位置({:.1f}, {:.1f}, {:.1f}) 方向(z: {:.2f}, w: {:.2f})'.format(
            self.goal_count + 1,
            pos.x, pos.y, pos.z,
            orient.z, orient.w
        ))
        
        # 异步发送目标
        send_goal_future = self.action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        
        # 添加完成回调
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """处理目标响应"""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn('目标被拒绝!')
            self.is_navigating = False
            return
        
        self.get_logger().info('目标已接受，正在导航...')
        
        # 获取结果回调
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """处理导航结果"""
        try:
            result = future.result().result
            self.get_logger().info(f'导航完成! 结果: {result}')
            
            # 导航完成后设置状态
            self.is_navigating = False
            
            # 创建等待定时器
            self.get_logger().info('导航完成，等待10秒后发送下一个目标...')
            self.wait_timer = self.create_timer(10.0, self.wait_timer_callback)
            
        except Exception as e:
            self.get_logger().error(f'获取结果时出错: {str(e)}')
            self.is_navigating = False

    def wait_timer_callback(self):
        """定时器回调：发送下一个目标"""
        # 销毁定时器
        if self.wait_timer:
            self.wait_timer.cancel()
            self.wait_timer = None
            
        # 切换到下一个目标
        self.goal_count += 1
        self.id = self.goal_count % 4
        if self.id == 0:
            self.current_goal = self.goal1
        elif self.id == 1:
            self.current_goal = self.goal2
        elif self.id == 2:
            self.current_goal = self.goal3
        else:
            self.current_goal = self.goal4
            
        # 发送新目标
        self.send_goal(self.current_goal)

    def feedback_callback(self, feedback_msg):
        """处理导航反馈"""
        feedback = feedback_msg.feedback
        # 可以在这里处理反馈信息，如显示当前位置
        self.get_logger().info(f'导航进度: {feedback.distance_remaining:.2f}米')

def main(args=None):
    rclpy.init(args=args)
    nav2_goal_sender = Nav2GoalSender()
    
    # 使用多线程执行器处理动作客户端
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(nav2_goal_sender)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 销毁节点并关闭ROS
        nav2_goal_sender.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()