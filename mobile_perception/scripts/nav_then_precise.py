#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
import subprocess
import signal
import sys
import time
import threading
from rclpy.executors import MultiThreadedExecutor
import ament_index_python.packages
import os

# 导入 AprilTag 检测节点
from detect_apriltags import UnifiedDetector, get_package_path

class NavThenPreciseControl(Node):
    def __init__(self):
        super().__init__('nav_then_precise_control')
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ---- 后退参数 ----
        self.backward_speed   = -0.08         # m/s（负号=后退）
        self.backward_duration = 2.5           # s
        self._cmd_pub = self.create_publisher(Twist, 'control/cmd_vel', 10)
        self._backward_start = None
        self._backwarding = False

        # AprilTag 检测节点相关
        self.apriltag_detector_node = None
        
        # 执行完成标志
        self.execution_completed = False

    # ---------------- 导航部分（未改动） ----------------
    def nav_to_pose(self, goal_pose: PoseStamped) -> bool:
        self.get_logger().info("等待 nav2 服务器...")
        self._nav_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        self.get_logger().info("发送导航目标...")
        send_future = self._nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("导航目标被拒绝！")
            return False
        self.get_logger().info("导航中...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        if status == 4:     # SUCCEEDED
            self.get_logger().info("导航完成！")
            return True
        else:
            self.get_logger().error(f"导航失败，状态码：{status}")
            return False

    # ---------------- 子进程封装 ----------------
    def run_subprocess(self, cmd: list) -> int:
        self.get_logger().info(f"执行：{' '.join(cmd)}")
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        print(proc.stdout)
        return proc.returncode

    # ---------------- AprilTag 检测节点管理 ----------------
    def start_apriltag_detection(self):
        """启动 AprilTag 检测节点"""
        try:
            self.get_logger().info("启动 AprilTag 检测节点...")
            
            # 获取配置文件路径
            package_path = get_package_path('mobile_perception')
            camera_config_path = os.path.join(package_path, 'config/camera_config.json')
            apriltags_field_path = os.path.join(package_path, 'config/tags_config.json')
            
            # 创建 AprilTag 检测节点
            self.apriltag_detector_node = UnifiedDetector(camera_config_path, apriltags_field_path)
            self.apriltag_detector_node.detect_by_camera_topics('/camera/image_raw')
            
            self.get_logger().info("AprilTag 检测节点已启动")
            return True
            
        except Exception as e:
            self.get_logger().error(f"启动 AprilTag 检测节点失败: {e}")
            return False

    def stop_apriltag_detection(self):
        """停止 AprilTag 检测节点"""
        try:
            if self.apriltag_detector_node is not None:
                self.get_logger().info("正在关闭 AprilTag 检测节点...")
                self.apriltag_detector_node.destroy_node()
                self.apriltag_detector_node = None
                self.get_logger().info("AprilTag 检测节点已关闭")
        except Exception as e:
            self.get_logger().error(f"关闭 AprilTag 检测节点时出错: {e}")

    # ---------------- 后退阶段 ----------------
    def start_backward(self):
        self.get_logger().info(f"开始后退 {self.backward_duration} s ...")
        self._backwarding = True
        self._backward_start = self.get_clock().now()

    def update_backward(self):
        if not self._backwarding:
            return False     # 未在后退
        elapsed = (self.get_clock().now() - self._backward_start).nanoseconds * 1e-9
        if elapsed < self.backward_duration:
            twist = Twist()
            twist.linear.x = self.backward_speed
            twist.angular.z = 0.0
            self._cmd_pub.publish(twist)
            return True      # 继续后退
        else:
            self.get_logger().info("后退完成，发布零速度")
            twist = Twist()
            self._cmd_pub.publish(twist)
            self._backwarding = False
            return False     # 后退结束

def main():
    rclpy.init()
    
    # 创建多线程执行器
    executor = MultiThreadedExecutor(num_threads=4)
    node = NavThenPreciseControl()
    executor.add_node(node)

    # 1. 读取充电点
    import os, json
    charging_pose_path = os.path.join(os.path.dirname(__file__), 'charging_pose.json')
    goal_a = PoseStamped()
    try:
        with open(charging_pose_path, 'r') as f:
            data = json.load(f)
        goal_a.header.frame_id = data.get('frame_id', 'map')
        goal_a.pose.position.x = data['position']['x']
        goal_a.pose.position.y = data['position']['y']
        goal_a.pose.position.z = data['position']['z']
        goal_a.pose.orientation.x = data['orientation']['x']
        goal_a.pose.orientation.y = data['orientation']['y']
        goal_a.pose.orientation.z = data['orientation']['z']
        goal_a.pose.orientation.w = data['orientation']['w']
    except Exception as e:
        print(f"充电点读取失败: {e}")
        rclpy.shutdown()
        return 1

    # 创建执行线程
    def execute_navigation_and_detection():
        try:
            # 2. 导航到 A（如需可取消注释）
            if not node.nav_to_pose(goal_a):
                node.get_logger().error("导航到 A 点失败，退出。")
                node.execution_completed = True
                return False

            # 3. 启动 AprilTag 检测节点
            if not node.start_apriltag_detection():
                node.get_logger().error("启动 AprilTag 检测失败，退出。")
                node.execution_completed = True
                return False
            
            # 将 AprilTag 检测节点添加到执行器
            if node.apriltag_detector_node:
                executor.add_node(node.apriltag_detector_node)

            # 4. 精定位 charging
            ret = node.run_subprocess([
                "ros2", "run", "mobile_perception",
                "control_robot.py", "--goal_name", "charging_pose"
            ])
            if ret != 0:
                node.get_logger().error("control_robot.py 失败")
                node.execution_completed = True
                return False

            # 5. 后退 2 s
            node.start_backward()
            # 用 spin 周期发速度
            while node.update_backward():
                time.sleep(0.02)  # 50 Hz 足够

            node.get_logger().info("全部流程完成！程序将自动退出...")
            node.execution_completed = True
            return True
            
        except KeyboardInterrupt:
            node.get_logger().info("用户中断")
            node.execution_completed = True
            return False
        except Exception as e:
            node.get_logger().error(f"执行过程中出错: {e}")
            node.execution_completed = True
            return False

    # 启动执行线程
    execution_thread = threading.Thread(target=execute_navigation_and_detection)
    execution_thread.daemon = True  # 设为守护线程
    execution_thread.start()

    try:
        # 执行器运行，直到执行完成
        while not node.execution_completed:
            executor.spin_once(timeout_sec=0.1)
            
    except KeyboardInterrupt:
        node.get_logger().info("用户中断，正在关闭...")
        node.execution_completed = True
        
    finally:
        # 清理工作
        try:
            # 停止 AprilTag 检测节点
            node.stop_apriltag_detection()
            
            # 确保停车
            node._cmd_pub.publish(Twist())
            
            # 关闭执行器和节点
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()
            
        except Exception as e:
            print(f"清理过程中出错: {e}")

    # 等待执行线程结束
    execution_thread.join(timeout=1.0)
    
    node.get_logger().info("程序已退出")
    return 0

if __name__ == '__main__':
    sys.exit(main())

