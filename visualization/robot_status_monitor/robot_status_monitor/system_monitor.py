#!/usr/bin/env python3
"""
Comprehensive System Monitor Node

This node monitors both ROS2 nodes and audio service process, 
publishing their status to separate topics.
"""

import rclpy
from rclpy.node import Node
import subprocess
import psutil
from robot_status_msgs.msg import RobotStatus
# from robot_status_msgs.msg import AudioServerStatus

# 配置需要监控的ROS2节点名称
CHECK_NODES = [
    '/arm_joint1',
    '/arm_joint2', 
]

# 配置语音服务进程名称（支持进程名和脚本名检测）
# AUDIO_SERVICE_PROCESS_NAMES = [
#     'w1_voice_server.py',     # 脚本文件名
#     'w1_voice_server',        # 可能的进程名
#     'python w1_voice_server.py'  # 完整命令行片段
# ]

# 检查间隔（秒）
CHECK_INTERVAL = 2.0

class SystemMonitor(Node):
    """
    系统综合监控器
    
    同时监控ROS2节点和语音服务进程的运行状态。
    """
    
    def __init__(self):
        super().__init__('system_monitor')
        
        # 声明参数
        self.declare_parameter('check_nodes', CHECK_NODES)
        # self.declare_parameter('audio_process_names', AUDIO_SERVICE_PROCESS_NAMES)
        self.declare_parameter('check_interval', CHECK_INTERVAL)
        self.declare_parameter('enable_status_logging', True)  # 新增参数控制日志输出
        
        # 获取参数
        self.check_nodes = self.get_parameter('check_nodes').get_parameter_value().string_array_value
        # self.audio_process_names = self.get_parameter('audio_process_names').get_parameter_value().string_array_value
        check_interval = self.get_parameter('check_interval').get_parameter_value().double_value
        self.enable_status_logging = self.get_parameter('enable_status_logging').get_parameter_value().bool_value
        
        # 创建发布者
        self.robot_status_publisher_ = self.create_publisher(RobotStatus, 'robot_status', 10)
        # self.audio_status_publisher_ = self.create_publisher(AudioServerStatus, 'audio_server_status', 10)
        
        # 状态缓存，用于检测状态变化
        self.last_robot_status = None
        # self.last_audio_status = None
        
        # 创建定时器
        self.timer = self.create_timer(check_interval, self.monitor_callback)
        
        self.get_logger().info(f'System monitor started.')
        self.get_logger().info(f'Monitoring ROS2 nodes: {self.check_nodes}')
        # self.get_logger().info(f'Monitoring audio processes: {self.audio_process_names}')
        self.get_logger().info(f'Check interval: {check_interval} seconds')

    def get_running_nodes(self):
        """
        获取当前运行的ROS2节点列表
        
        Returns:
            list: 运行中的节点名称列表
        """
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'], 
                capture_output=True, 
                text=True, 
                timeout=5
            )
            if result.returncode == 0:
                return result.stdout.strip().splitlines()
            else:
                self.get_logger().error(f'Failed to get node list: {result.stderr}')
                return []
        except subprocess.TimeoutExpired:
            self.get_logger().error('Timeout when getting node list')
            return []
        except Exception as e:
            self.get_logger().error(f'Exception when getting node list: {e}')
            return []

    def is_process_running(self, process_name):
        """
        检查指定进程是否正在运行
        
        Args:
            process_name (str): 进程名称
            
        Returns:
            bool: 进程是否正在运行
        """
        try:
            for process in psutil.process_iter(['name', 'cmdline']):
                try:
                    # 检查进程名称
                    if process.info['name'] and process_name in process.info['name']:
                        return True
                    # 检查命令行参数
                    if process.info['cmdline']:
                        cmdline = ' '.join(process.info['cmdline'])
                        if process_name in cmdline:
                            return True
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            return False
        except Exception as e:
            self.get_logger().error(f'Error checking process {process_name}: {e}')
            return False

    def check_robot_nodes(self):
        """
        检查ROS2节点状态
        
        Returns:
            tuple: (is_ok, error_nodes)
        """
        running_nodes = self.get_running_nodes()
        
        # 检查哪些节点不在运行
        error_nodes = []
        for node in self.check_nodes:
            if node not in running_nodes:
                error_nodes.append(node)
        
        return len(error_nodes) == 0, error_nodes

    # def check_audio_service(self):
    #     """
    #     检查语音服务状态
        
    #     Returns:
    #         tuple: (is_online, status_description)
    #     """
    #     try:
    #         running_processes = []
            
    #         for process_name in self.audio_process_names:
    #             if self.is_process_running(process_name):
    #                 running_processes.append(process_name)
            
    #         if running_processes:
    #             is_online = True
    #             status = f"运行中: {', '.join(running_processes)}"
    #         else:
    #             is_online = False
    #             status = "未运行"
                
    #         return is_online, status
            
    #     except Exception as e:
    #         self.get_logger().error(f'Exception when checking audio service: {e}')
    #         return False, f"检查异常: {str(e)}"

    def monitor_callback(self):
        """
        定时监控回调函数
        """
        # 检查ROS2节点状态
        robot_is_ok, error_nodes = self.check_robot_nodes()
        
        # 创建并发布机器人状态消息
        robot_msg = RobotStatus()
        robot_msg.is_ok = robot_is_ok
        robot_msg.error_nodes = error_nodes
        self.robot_status_publisher_.publish(robot_msg)
        
        # 只在状态发生变化时记录日志（如果启用了日志记录）
        current_robot_status = (robot_is_ok, tuple(error_nodes))  # 使用元组便于比较
        if self.enable_status_logging and self.last_robot_status != current_robot_status:
            if robot_is_ok:
                self.get_logger().info('机器人系统状态: 正常')
            else:
                self.get_logger().warn(f'机器人系统状态: 异常 - 离线节点: {error_nodes}')
        self.last_robot_status = current_robot_status
        
        # 检查语音服务状态
        # audio_is_online, audio_status = self.check_audio_service()
        
        # 创建并发布语音服务状态消息
        # audio_msg = AudioServerStatus()
        # audio_msg.is_online = audio_is_online
        # audio_msg.status = audio_status
        # self.audio_status_publisher_.publish(audio_msg)
        
        # # 只在状态发生变化时记录日志（如果启用了日志记录）
        # current_audio_status = (audio_is_online, audio_status)
        # if self.enable_status_logging and self.last_audio_status != current_audio_status:
        #     if audio_is_online:
        #         self.get_logger().info(f'语音服务状态: 在线 - {audio_status}')
        #     else:
        #         self.get_logger().warn(f'语音服务状态: 离线 - {audio_status}')
        # self.last_audio_status = current_audio_status

def main(args=None):
    """
    主函数
    """
    rclpy.init(args=args)
    
    try:
        node = SystemMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()