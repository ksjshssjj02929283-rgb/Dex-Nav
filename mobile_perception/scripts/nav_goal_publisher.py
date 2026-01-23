#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import yaml
import time
import sys


class NavGoalPublisher(Node):

    def __init__(self):
        super().__init__('nav_goal_publisher')

        # 声明参数文件路径
        self.declare_parameter('params_file', 'config/params.yaml')
        params_file = self.get_parameter('params_file').get_parameter_value().string_value

        # 加载参数文件
        self.goal_params = self.load_params(params_file)
        self.get_logger().info(f"Loaded goal: {self.goal_params}")

        # 创建Action客户端
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 等待Action服务器
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server connected!")

        # 发送目标点
        self.send_goal()

        # 添加导航超时计时器
        self.navigation_timeout = 120  # 2分钟超时
        self.navigation_timer = self.create_timer(
            1.0,  # 每秒检查一次
            self.check_navigation_timeout)
        self.start_time = time.time()

    def load_params(self, file_path):
        """从YAML文件加载目标点参数"""
        try:
            with open(file_path, 'r') as file:
                params = yaml.safe_load(file)
                return {
                    'position': params['goal_position'],
                    'orientation': params['goal_orientation'],
                    'frame_id': params.get('frame_id', 'map')
                }
        except Exception as e:
            self.get_logger().error(f"Failed to load parameters: {str(e)}")
            self.safe_shutdown()
            return {'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'orientation': {'z': 0.0, 'w': 1.0}, 'frame_id': 'map'}

    def send_goal(self):
        """构造并发送导航目标"""
        goal_msg = NavigateToPose.Goal()
        pose_stamped = PoseStamped()

        # 设置header
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self.goal_params['frame_id']

        # 设置位置
        pose_stamped.pose.position.x = self.goal_params['position']['x']
        pose_stamped.pose.position.y = self.goal_params['position']['y']
        pose_stamped.pose.position.z = self.goal_params['position']['z']

        # 设置方向
        pose_stamped.pose.orientation.z = self.goal_params['orientation']['z']
        pose_stamped.pose.orientation.w = self.goal_params['orientation']['w']

        goal_msg.pose = pose_stamped
        self.get_logger().info(f"Sending goal: {pose_stamped}")

        # 异步发送目标
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """处理目标响应"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            self.safe_shutdown()
            return

        self.get_logger().info('Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """处理最终结果"""
        try:
            result = future.result().result
            self.get_logger().info(f'Navigation result: {result}')

            # 导航完成后安全关闭节点
            self.get_logger().info("Navigation completed. Shutting down node.")
            self.safe_shutdown()
        except Exception as e:
            self.get_logger().error(f"Error in result callback: {str(e)}")
            self.safe_shutdown()

    def feedback_callback(self, feedback_msg):
        """处理反馈"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} meters')
        # 重置超时计时器
        self.start_time = time.time()

    def check_navigation_timeout(self):
        """检查导航是否超时"""
        elapsed = time.time() - self.start_time
        if elapsed > self.navigation_timeout:
            self.get_logger().error(f"Navigation timed out after {elapsed:.1f} seconds")
            self.safe_shutdown()

    def safe_shutdown(self):
        """安全关闭节点"""
        self.destroy_node()
        rclpy.try_shutdown()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)

    try:
        nav_goal_publisher = NavGoalPublisher()
        rclpy.spin(nav_goal_publisher)
    except KeyboardInterrupt:
        nav_goal_publisher.get_logger().info("Node shutdown by keyboard interrupt")
    except Exception as e:
        nav_goal_publisher.get_logger().error(f"Error in main: {str(e)}")
    finally:
        # 确保在退出前正确关闭
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
