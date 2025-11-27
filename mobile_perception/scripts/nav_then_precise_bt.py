#!/usr/bin/env python3
"""
一键回充行为树版本（带失败重试和undocking）
基于原始nav_then_precise.py重构，使用py_trees行为树框架
"""
import json
import os
import subprocess
import time
import signal
import sys
import py_trees
from py_trees import common
import rclpy
from sensor_msgs.msg import BatteryState
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
import threading

# ---------------- 工具函数 ---------------- #
def read_charging_pose(json_path: str) -> PoseStamped:
    """读取充电点位置"""
    pose = PoseStamped()
    try:
        with open(json_path, 'r') as f:
            data = json.load(f)
        pose.header.frame_id = data.get('frame_id', 'map')
        pose.pose.position.x = data['position']['x']
        pose.pose.position.y = data['position']['y']
        pose.pose.position.z = data['position']['z']
        pose.pose.orientation.x = data['orientation']['x']
        pose.pose.orientation.y = data['orientation']['y']
        pose.pose.orientation.z = data['orientation']['z']
        pose.pose.orientation.w = data['orientation']['w']
        return pose
    except Exception as e:
        print(f"充电点读取失败: {e}")
        raise

# ---------------- ROS2节点封装 ---------------- #
class NavThenPreciseNode(Node):
    def __init__(self):
        super().__init__('nav_then_precise_bt_node')
        # self.executor = MultiThreadedExecutor(num_threads=4)
        # self.executor.add_node(self) 
        # 导航客户端
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 速度发布器
        self._cmd_pub = self.create_publisher(Twist, 'control/cmd_vel', 10)
        
        # 充电状态订阅器
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, 
            depth=1
        )
        self._sub_charging = self.create_subscription(
            BatteryState, '/feedback/battery', self._cb_charging, qos
        )
        self.is_charging = False


        
        # 后退参数（保持原始参数）
        self.backward_speed = -0.08         # m/s（负号=后退）
        self.backward_duration = 2.5        # s

        # 模拟控制开关：是否强制让充电检测失败以触发重试
        # 可通过环境变量 BT_SIM_CHECK_FAIL=1 打开
        self.sim_check_force_fail = os.getenv('BT_SIM_CHECK_FAIL', '0') == '1'

        # 检测节点与执行器
        self.apriltag_detector_node = None
        # self.executor = None
        self._apriltag_needs_add = False

    # ---------------- AprilTag 检测节点管理 ----------------
    def start_apriltag_detection(self) -> bool:
        """在当前进程内启动 AprilTag 检测节点，并将其加入执行器"""
        try:
            if self.apriltag_detector_node is not None:
                self.get_logger().warn("AprilTag 检测节点已在运行")
                return True
            # 获取配置文件路径
            package_path = get_package_path('mobile_perception')
            camera_config_path = os.path.join(package_path, 'config/camera_config.json')
            apriltags_field_path = os.path.join(package_path, 'config/tags_config.json')

            # 创建检测节点并订阅话题
            self.apriltag_detector_node = UnifiedDetector(camera_config_path, apriltags_field_path)
            self.apriltag_detector_node.detect_by_camera_topics('/camera/image_raw')

            # 延迟加入执行器：仅打标记，主循环里统一加入，避免此处 executor 为空
            self._apriltag_needs_add = True
            self.get_logger().info("AprilTag 检测节点已启动（进程内，等待加入执行器）")
            return True
        except Exception as e:
            self.get_logger().error(f"启动 AprilTag 检测节点失败: {e}")
            self.apriltag_detector_node = None
            return False

    def stop_apriltag_detection(self):
        """停止并销毁 AprilTag 检测节点，从执行器移除"""
        try:
            if self.apriltag_detector_node is not None:
                self.get_logger().info("正在关闭 AprilTag 检测节点...")
                try:
                    if self.executor is not None:
                        try:
                            self.executor.remove_node(self.apriltag_detector_node)
                        except Exception:
                            pass
                finally:
                    self.apriltag_detector_node.destroy_node()
                    self.apriltag_detector_node = None
                self.get_logger().info("AprilTag 检测节点已关闭")
        except Exception as e:
            self.get_logger().error(f"关闭 AprilTag 检测节点时出错: {e}")

    def _cb_charging(self, msg: BatteryState):
        """充电状态回调"""
        self.get_logger().info(f"电源状态码: {msg.power_supply_status}")
    
        if msg.power_supply_status == 1:
            self.is_charging = True
        else:
            self.is_charging = False

    def nav_to_pose(self, goal_pose: PoseStamped) -> bool:
        """导航到指定位置（模拟）
        注意：保留真实实现于下方注释，便于随时恢复。
        """

        # --- 真实实现（请在需要时取消注释恢复） ---
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
        if status == 4:  # SUCCEEDED
            self.get_logger().info("导航完成！")
            return True
        else:
            self.get_logger().error(f"导航失败，状态码：{status}")
            return False

    def run_subprocess(self, cmd: list) -> int:
        """执行子进程命令（模拟）
        注意：保留真实实现于下方注释，便于随时恢复。
        """


        # --- 真实实现（请在需要时取消注释恢复） ---
        self.get_logger().info(f"执行：{' '.join(cmd)}")
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        print(proc.stdout)
        return proc.returncode

    def backward(self, speed=None, duration=None):
        """后退运动（模拟）
        注意：保留真实实现于下方注释，便于随时恢复。
        """
        if speed is None:
            speed = self.backward_speed
        if duration is None:
            duration = self.backward_duration

        self.get_logger().info(f"开始后退 {duration} s ...")
        t_start = self.get_clock().now()
        while rclpy.ok():
            elapsed = (self.get_clock().now() - t_start).nanoseconds * 1e-9
            if elapsed >= duration:
                break
            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = 0.0
            self._cmd_pub.publish(twist)
            time.sleep(0.02)
        self.get_logger().info("后退完成，发布零速度")
        self._cmd_pub.publish(Twist())

    def undocking(self, speed=0.15, duration=3.0):
        """undocking：向前运动（模拟）
        注意：保留真实实现于下方注释，便于随时恢复。
        """
        self.get_logger().info(f"undocking 向前 {duration} s ...")
        t_start = self.get_clock().now()
        while rclpy.ok():
            elapsed = (self.get_clock().now() - t_start).nanoseconds * 1e-9
            if elapsed >= duration:
                break
            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = 0.0
            self._cmd_pub.publish(twist)
            time.sleep(0.02)
        self._cmd_pub.publish(Twist())

# ---------------- 行为树叶子节点 ---------------- #
class NavigateToCharging(py_trees.behaviour.Behaviour):
    """导航到充电点"""
    def __init__(self, node: NavThenPreciseNode, pose: PoseStamped):
        super().__init__('NavigateToCharging')
        self.node = node
        self.pose = pose

    def update(self):
        ok = self.node.nav_to_pose(self.pose)
        return common.Status.SUCCESS if ok else common.Status.FAILURE

class AprilTagAndDock(py_trees.behaviour.Behaviour):
    """AprilTag检测和精定位（子进程 ros2 run 方式）"""
    def __init__(self, node: NavThenPreciseNode):
        super().__init__('AprilTagAndDock')
        self.node = node
        self.apriltag_proc = None

    def initialise(self):
        """初始化：用 ros2 run 启动检测进程；直到行为树结束才清理"""
        if self.apriltag_proc is None:
            self.node.get_logger().info("启动 AprilTag 检测进程 (ros2 run)...")
            # 新会话启动，便于按进程组发送信号
            self.apriltag_proc = subprocess.Popen([
                "bash", "-lc",
                "cd ~/w1 && source install/setup.bash && exec ros2 run mobile_perception detect_apriltags.py"
            ], start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.node.get_logger().info(f"AprilTag 进程已启动，pid={self.apriltag_proc.pid}")

    def update(self):
        """执行精定位和后退"""
        ret = self.node.run_subprocess([
            "ros2", "run", "mobile_perception",
            "control_robot.py", "--goal_name", "charging_pose"
        ])
        if ret != 0:
            self.node.get_logger().error("control_robot.py 失败")
            return common.Status.FAILURE
        self.node.backward()
        return common.Status.SUCCESS

    def terminate(self, new_status):
        """终止：关闭AprilTag检测进程（模拟）
        注意：保留真实实现于下方注释，便于随时恢复。
        """
        # 此方法不再清理进程，由 RetryWithUndocking 在树结束时统一清理
        pass
    
    def cleanup_apriltag(self):
        """清理AprilTag检测进程：SIGTERM -> SIGKILL（不改对方脚本）"""
        if not hasattr(self, 'apriltag_proc') or self.apriltag_proc is None:
            return
        # 已退出
        if self.apriltag_proc.poll() is not None:
            self.apriltag_proc = None
            return
        pid = self.apriltag_proc.pid
        self.node.get_logger().info(f"正在关闭 AprilTag 检测进程，pid={pid} ...")
        try:
            # 获取进程组
            try:
                pgid = os.getpgid(pid)
            except Exception:
                pgid = None

            # 1) 直接发送 SIGTERM（避免触发对方 SIGINT handler 异常栈）
            try:
                if pgid is not None:
                    os.killpg(pgid, signal.SIGTERM)
                else:
                    os.kill(pid, signal.SIGTERM)
            except Exception:
                pass
            try:
                self.apriltag_proc.wait(timeout=3.0)
                self.node.get_logger().info("AprilTag 检测进程已通过 SIGTERM 退出")
                self.apriltag_proc = None
                return
            except subprocess.TimeoutExpired:
                self.node.get_logger().warn("SIGTERM 超时，发送 SIGKILL ...")

            # 2) 最终：SIGKILL
            try:
                if pgid is not None:
                    os.killpg(pgid, signal.SIGKILL)
                else:
                    os.kill(pid, signal.SIGKILL)
            except Exception:
                pass
            try:
                self.apriltag_proc.wait(timeout=5.0)
            except Exception:
                pass
            self.node.get_logger().info("AprilTag 检测进程已关闭")
        finally:
            self.apriltag_proc = None

class CheckCharging(py_trees.behaviour.Behaviour):
    """检查充电状态"""
    def __init__(self, node: NavThenPreciseNode, timeout=5.0):
        super().__init__('CheckCharging')
        self.node = node
        self.timeout = timeout
        self.start = None

    def initialise(self):
        """初始化：记录开始时间"""
        self.start = self.node.get_clock().now()
        self.node.get_logger().info("开始检查充电状态...")

    def update(self):
        """检查充电状态"""
        elapsed = (self.node.get_clock().now() - self.start).nanoseconds * 1e-9

        self.node.get_logger().info(f"电源状态码：{self.node.is_charging}, 已等待{elapsed:.1f}s")
        
        if self.node.is_charging:
            self.node.get_logger().info("✅ 检测到充电状态！")
            return common.Status.SUCCESS
            
        if elapsed >= self.timeout:
            self.node.get_logger().warn(f"❌ 充电检测超时({self.timeout}s)，未检测到充电")
            return common.Status.FAILURE
            
        # 继续检测
        return common.Status.RUNNING

class Undocking(py_trees.behaviour.Behaviour):
    """undocking行为：向前运动"""
    def __init__(self, node: NavThenPreciseNode):
        super().__init__('Undocking')
        self.node = node

    def update(self):
        """执行undocking"""
        self.node.undocking()
        return common.Status.SUCCESS

# ---------------- 自定义重试节点 ---------------- #
class RetryWithUndocking(py_trees.behaviour.Behaviour):
    """带undocking的重试节点：最多尝试5次"""
    def __init__(self, name: str, node: NavThenPreciseNode, dock_subtree: py_trees.behaviour.Behaviour):
        super().__init__(name)
        self.node = node
        self.counter = 0
        self.max_tries = 5
        
        # 状态机成员
        self._dock = dock_subtree
        self._check = CheckCharging(node, timeout=5.0)
        self._phase = 'DOCK'  # 'DOCK', 'CHECK', 'UNDOCK'
        self._apriltag_started = False  # 标记是否已启动AprilTag

    def initialise(self):
        self.node.get_logger().info(f"重试节点初始化，最多尝试{self.max_tries}次")
        self.counter = 0
        self._phase = 'DOCK'
        self._apriltag_started = False
        
        # 启动AprilTag检测进程（仅启动一次，持续到树结束）
        if not self._apriltag_started:
            self._dock.initialise()
            self._apriltag_started = True
            self.node.get_logger().info("AprilTag检测进程已启动，将持续运行到行为树结束")

    def update(self):
        """状态机：DOCK -> CHECK -> (SUCCESS | UNDOCK -> DOCK)"""
        
        if self._phase == 'DOCK':
            # 执行精定位
            dock_status = self._dock.update()
            
            if dock_status == common.Status.SUCCESS:
                self.node.get_logger().info("精定位完成，进入充电检测阶段")
                self._check.initialise()  # ✅ 只在精定位成功后初始化一次
                self._phase = 'CHECK'
                return common.Status.RUNNING
                
            elif dock_status == common.Status.FAILURE:
                self.node.get_logger().warn("精定位失败，准备undocking重试")
                self._phase = 'UNDOCK'
                return common.Status.RUNNING
                
            else:  # RUNNING
                return common.Status.RUNNING

        elif self._phase == 'CHECK':
            # 检查充电状态（非阻塞）
            check_status = self._check.update()
            
            if check_status == common.Status.SUCCESS:
                return common.Status.SUCCESS  # 整个流程成功
                
            elif check_status == common.Status.FAILURE:
                self.node.get_logger().warn("充电检测失败，准备undocking重试")
                self._phase = 'UNDOCK'
                return common.Status.RUNNING
                
            else:  # RUNNING
                return common.Status.RUNNING

        elif self._phase == 'UNDOCK':
            # 执行undocking
            self.counter += 1
            if self.counter > self.max_tries:
                self.node.get_logger().error(f"已达到最大重试次数{self.max_tries}")
                return common.Status.FAILURE
                
            self.node.get_logger().info(f"第{self.counter}/{self.max_tries}次undocking...")
            self.node.undocking()
            
            # 重置状态，准备下一次尝试（不再调用initialise，保持AprilTag进程运行）
            self._phase = 'DOCK'
            return common.Status.RUNNING
    
    def terminate(self, new_status):
        """树结束时清理AprilTag进程"""
        if self._apriltag_started:
            self.node.get_logger().info(f"行为树结束（状态：{new_status}），清理AprilTag检测进程...")
            self._dock.cleanup_apriltag()
            self._apriltag_started = False


# ---------------- 行为树构建 ---------------- #
def create_root(node: NavThenPreciseNode, pose: PoseStamped):
    """创建行为树根节点"""
    # 导航节点
    nav = NavigateToCharging(node, pose)
    
    # 精定位节点
    dock = AprilTagAndDock(node)
    
    # 重试节点（包含undocking）
    retry_root = RetryWithUndocking('RetryWithUndocking', node, dock)
    
    # 根序列：导航 -> 重试精定位
    root = py_trees.composites.Sequence('Root', memory=True)
    root.add_children([nav, retry_root])
    
    return root


# ---------------- 主函数 ---------------- #
def main():
    """主函数"""
    rclpy.init()
    node = NavThenPreciseNode()

    try:
        # 读取充电点位置
        charging_pose_path = os.path.join(os.path.dirname(__file__), 'charging_pose.json')
        goal_pose = read_charging_pose(charging_pose_path)

        # 创建行为树
        root = create_root(node, goal_pose)
        tree = py_trees.trees.BehaviourTree(root)
        tree.setup(timeout=10.0)

        node.get_logger().info("开始执行一键回充行为树...")

        # 执行行为树
        while rclpy.ok() and root.status not in (common.Status.SUCCESS, common.Status.FAILURE):
            tree.tick()
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)

        # 树结束时清理资源（包括AprilTag进程）
        final_status = root.status
        root.stop(final_status)

        # 输出最终结果
        if final_status == common.Status.SUCCESS:
            node.get_logger().info("全部流程完成！")
            return 0
        else:
            node.get_logger().error("流程失败！")
            return 1

    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
        # 中断时也要清理资源
        if root.status != common.Status.INVALID:
            root.stop(common.Status.INVALID)
        return 1
    except Exception as e:
        node.get_logger().error(f"执行出错: {e}")
        # 异常时也要清理资源
        if root.status != common.Status.INVALID:
            root.stop(common.Status.INVALID)
        return 1
    finally:
        # 确保停车
        node._cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    sys.exit(main())