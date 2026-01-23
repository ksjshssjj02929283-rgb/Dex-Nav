#!/usr/bin/env python3
"""
获取移动控制节点状态的工具脚本
提供get_control_status()函数用于查询当前控制状态
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys

# 全局变量用于管理ROS2节点
_node_instance = None
_service_client = None


def _init_ros2():
    """初始化ROS2（如果尚未初始化）"""
    global _node_instance, _service_client

    if _node_instance is None:
        if not rclpy.ok():
            rclpy.init()
        _node_instance = Node('get_control_status_client')
        _service_client = _node_instance.create_client(Trigger, 'control/get_status')

        # 等待服务可用
        if not _service_client.wait_for_service(timeout_sec=2.0):
            _node_instance.get_logger().warn('服务 control/get_status 不可用')
            return False
    return True


def get_control_status():
    """
    调用服务查询当前控制状态
    
    Returns:
        str: 状态字符串，可能的值：
            - "空闲中": 已到达目标并停止
            - "控制中": 正在执行控制任务
            - "等待中": 其他情况
        None: 如果服务调用失败
    
    Example:
        >>> status = get_control_status()
        >>> print(f"当前状态: {status}")
    """
    global _node_instance, _service_client

    # 初始化ROS2
    if not _init_ros2():
        print("错误: 无法连接到服务 control/get_status")
        return None

    # 创建请求
    request = Trigger.Request()

    # 调用服务
    try:
        future = _service_client.call_async(request)
        rclpy.spin_until_future_complete(_node_instance, future, timeout_sec=1.0)

        if future.done():
            response = future.result()
            if response.success:
                return response.message
            else:
                print(f"服务调用失败: {response.message}")
                return None
        else:
            print("错误: 服务调用超时")
            return None
    except Exception as e:
        print(f"错误: 调用服务时发生异常: {e}")
        return None


def cleanup():
    """清理ROS2资源"""
    global _node_instance
    if _node_instance is not None:
        _node_instance.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    """主函数，用于命令行测试"""
    try:
        status = get_control_status()
        if status is not None:
            print(f"当前控制状态: {status}")
            return 0
        else:
            print("无法获取状态")
            return 1
    except KeyboardInterrupt:
        print("\n中断")
        return 1
    finally:
        cleanup()


if __name__ == '__main__':
    sys.exit(main())
