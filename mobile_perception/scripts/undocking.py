#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import signal
import argparse

def parse_arguments():
    parser = argparse.ArgumentParser(description='Control Node for Docking')
    parser.add_argument('--timeout', type=float, default=5.0, help='Timeout for the control loop in seconds')
    parser.add_argument('--speed', type=float, default=0.015, help='Speed of the robot')
    parser.add_argument('--direction', type=int, default=1, help='Direction of the robot movement, 1 for forward, -1 for backward')
    return parser.parse_args()

class ControlNode(Node):
    def __init__(self, time_out=5.0, speed=0.015, direction=1):
        super().__init__('control_node')
        
        self.time_out = time_out
        self.speed = speed
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

        # Simulate moving towards the goal
        self.publish_twist(self.speed * self.direction, 0.0)

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
    control_node = ControlNode(time_out=args.timeout, speed=args.speed, direction=args.direction)

    def sigint_handler(signal, frame):
        control_node.get_logger().info('SIGINT received, shutting down...') 
        control_node.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sigint_handler)
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(control_node)
     
    executor.spin() 

if __name__ == '__main__':
    main()
