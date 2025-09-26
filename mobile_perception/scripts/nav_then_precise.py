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

# ---------------- main ----------------
def main():
    rclpy.init()
    node = NavThenPreciseControl()

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
        return 1

    try:
        # 2. 导航到 A（如需可取消注释）
        if not node.nav_to_pose(goal_a):
            node.get_logger().error("导航到 A 点失败，退出。")
            return 1

        # 3. 启动 AprilTag 检测（后台运行）
        node.get_logger().info("启动 AprilTag 检测...")
        import subprocess
        apriltag_process = subprocess.Popen([
            "bash", "-c", 
            "cd ~/w1 && source install/setup.bash && ros2 run mobile_perception detect_apriltags.py"
        ])

        # 4. 精定位 charging
        ret = node.run_subprocess([
            "ros2", "run", "mobile_perception",
            "control_robot.py", "--goal_name", "charging_pose"
        ])
        if ret != 0:
            node.get_logger().error("control_robot.py 失败")
            return 1

        # 5. 后退 2 s
        node.start_backward()
        # 用 spin 周期发速度
        while node.update_backward():
            rclpy.spin_once(node, timeout_sec=0.02)   # 50 Hz 足够

        node.get_logger().info("全部流程完成！")
        
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        # 关闭 AprilTag 检测进程
        try:
            if 'apriltag_process' in locals():
                node.get_logger().info("正在关闭 AprilTag 检测进程...")
                apriltag_process.terminate()  # 发送 SIGTERM，温和终止
                apriltag_process.wait(timeout=5)  # 等待最多 5 秒
                node.get_logger().info("AprilTag 检测进程已关闭")
        except subprocess.TimeoutExpired:
            node.get_logger().warn("AprilTag 进程未响应 SIGTERM，强制终止...")
            apriltag_process.kill()  # 强制终止
            apriltag_process.wait()
        except Exception as e:
            node.get_logger().error(f"关闭 AprilTag 进程时出错: {e}")
        
        # 确保停车
        node._cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
    return 0

if __name__ == '__main__':
    sys.exit(main())