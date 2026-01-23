#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import signal
import argparse
import os
import json


def parse_arguments():
    parser = argparse.ArgumentParser(description='Control Node for Spinning')
    parser.add_argument('--speed',
                        type=float,
                        default=0.02,
                        help='Spin speed of the robot')
    parser.add_argument('--timeout',
                        type=float,
                        default=5.0,
                        help='Timeout for the spinning loop in seconds')
    parser.add_argument(
        '--direction',
        type=int,
        default=1,
        help='Spin direction, 1 for clockwise, -1 for counter-clockwise')
    return parser.parse_args()


class ControlNode(Node):

    def __init__(self, spin_speed, time_out, direction):
        super().__init__('control_node')

        self.spin_speed = spin_speed
        self.time_out = time_out
        self.direction = direction

        # Initialize start time
        self.start_time = self.get_clock().now()

        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def reset_start_time(self):
        self.start_time = self.get_clock().now()

    def calculate_elapsed_time(self):
        elapsed_time = self.get_clock().now() - self.start_time
        return elapsed_time.to_msg().sec + elapsed_time.to_msg().nanosec * 1e-9

    def publish_twist(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

    def control_loop(self):
        # Check for timeout
        if self.calculate_elapsed_time() > self.time_out:
            self.get_logger().info('Timeout reached')
            self.publish_twist(0.0, 0.0)
            self.shutdown()
            return

        # Simulate spinning
        self.publish_twist(0.0, self.spin_speed * self.direction)

    def shutdown(self):
        # Stop publishing twist messages
        self.publish_twist(0.0, 0.0)
        # Shutdown rclpy
        rclpy.shutdown()
        sys.exit(0)  # Exit the program


def main(args=None):
    rclpy.init(args=args)
    args = parse_arguments()

    # Create the control node
    control_node = ControlNode(spin_speed=args.speed,
                               time_out=args.timeout,
                               direction=args.direction)

    def sigint_handler(signal, frame):
        control_node.get_logger().info('SIGINT received, shutting down...')
        control_node.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sigint_handler)

    # rclpy.spin(control_node)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(control_node)
    executor.spin()


if __name__ == '__main__':
    main()
