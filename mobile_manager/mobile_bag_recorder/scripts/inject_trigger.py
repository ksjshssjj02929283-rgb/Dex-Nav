#!/usr/bin/env python3
"""Inject a synthetic trigger source message for testing mobile_bag_recorder.

`ros2 topic pub` cannot publish DiagnosticArray: its `level` field is a ROS `byte`,
and the CLI rejects plain integers with
"The 'level' field must be of type 'bytes' or 'ByteString' with length 1".
This script publishes the message properly instead.

Usage:
    ros2 run mobile_bag_recorder inject_trigger.py diagnostics [level]
    ros2 run mobile_bag_recorder inject_trigger.py goal_error [distance_error]
    ros2 run mobile_bag_recorder inject_trigger.py nav_aborted
"""

import sys

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


def wait_for_subscriber(node, publisher, timeout_s=3.0):
    waited = 0.0
    while waited < timeout_s and publisher.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
        waited += 0.1
    return publisher.get_subscription_count()


def publish(node, publisher, msg, times=3):
    for _ in range(times):
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.2)


def main():
    kind = sys.argv[1] if len(sys.argv) > 1 else 'diagnostics'
    rclpy.init()
    node = rclpy.create_node('mobile_bag_recorder_trigger_injector')

    if kind == 'diagnostics':
        level = int(sys.argv[2]) if len(sys.argv) > 2 else 2
        pub = node.create_publisher(DiagnosticArray, '/navigation/speed_plan_diagnostics', 10)
        subs = wait_for_subscriber(node, pub)
        status = DiagnosticStatus()
        status.level = bytes([level])
        status.name = 'speed_plan'
        status.message = 'injected by inject_trigger.py'
        msg = DiagnosticArray()
        msg.status = [status]
        publish(node, pub, msg)
        print(f'published diagnostics level={level}, subscribers={subs}')

    elif kind == 'goal_error':
        distance = sys.argv[2] if len(sys.argv) > 2 else '0.5'
        pub = node.create_publisher(DiagnosticArray, '/navigation_goal_error', 10)
        subs = wait_for_subscriber(node, pub)
        status = DiagnosticStatus()
        status.level = bytes([0])
        status.name = 'navigation_goal_error'
        status.values = [KeyValue(key='distance_error', value=str(distance))]
        msg = DiagnosticArray()
        msg.status = [status]
        publish(node, pub, msg)
        print(f'published goal_error distance_error={distance}, subscribers={subs}')

    elif kind == 'nav_aborted':
        pub = node.create_publisher(GoalStatusArray, '/navigate_to_pose/_action/status', 10)
        subs = wait_for_subscriber(node, pub)
        status = GoalStatus()
        status.goal_info.goal_id.uuid = list(range(16))
        status.status = GoalStatus.STATUS_ABORTED
        msg = GoalStatusArray()
        msg.status_list = [status]
        publish(node, pub, msg)
        print(f'published nav_aborted, subscribers={subs}')

    else:
        print(f'unknown trigger kind: {kind}', file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
