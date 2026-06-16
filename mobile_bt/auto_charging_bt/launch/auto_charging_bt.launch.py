from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    # 1. AprilTag 检测节点（独立，不含对接逻辑）
    apriltag_node = Node(
        package='mobile_perception',
        executable='detect_apriltags.py',
        name='detect_apriltags',
        output='screen',
        respawn=True,  # 崩溃自动重启（流程结束时会整体Shutdown）
        respawn_delay=2.0)

    # 2. 行为树节点（只负责对接、检测、重试）
    bt_node = Node(package='auto_charging_bt',
                   executable='auto_charging_bt_node',
                   name='charging_service',
                   output='screen')

    # 当 bt_node 退出（成功或失败）时，关闭整个Launch，从而结束所有子进程
    shutdown_on_bt_exit = RegisterEventHandler(
        OnProcessExit(target_action=bt_node,
                      on_exit=[
                          EmitEvent(event=Shutdown(
                              reason='charging flow completed or failed'))
                      ]))

    return LaunchDescription([apriltag_node, bt_node, shutdown_on_bt_exit])
