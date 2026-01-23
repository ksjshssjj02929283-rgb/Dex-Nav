#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import numpy as np
import transforms3d as t3d
import time
import sys
import signal
import math
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Int32
import json

from goal_controller_simulation import GoalController

import ament_index_python.packages
import os
import argparse

from custom_interfaces.msg import Status


def parse_arguments():
    parser = argparse.ArgumentParser(description='Control Node for Docking')
    parser.add_argument('--goal_name', type=str, required=True, help='Name of the goal from the JSON config')
    return parser.parse_args()


def get_package_path(package_name):
    try:
        return ament_index_python.packages.get_package_share_directory(package_name)
    except ament_index_python.packages.PackageNotFoundError:
        print(f"Package '{package_name}' not found.")
        return None


class ControlNode(Node):

    def __init__(self, target_pose, time_out=10.0):
        super().__init__('control_node')

        # Target pose in the robot base link frame
        self.target_pose = target_pose
        self.time_out = time_out

        # Initialize start time
        self.start_time = self.get_clock().now()

        # Publisher for Result, 0, 1, 2 for Success, Failure, Running
        self.control_result = 2

        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Publisher for control result
        self.result_pub = self.create_publisher(Int32, 'docking_ctrl_result', 10)

        # Subscriber for AprilTag poses
        self.pose_sub = self.create_subscription(PoseArray, 'apriltag_poses', self.pose_callback, 10)

        self.status_sub = self.create_subscription(Status, 'status', self.status_callback, 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Variable to store the latest detected AprilTag pose
        self.fdb_pose = None

        # # Variable to store the latest status
        self.latest_status = None

        # Controller object, calculate velocity from current pose to goal pose
        self.controller = GoalController()

    def pose_callback(self, msg):
        # Check if the pose list is empty or None
        if msg.poses:
            self.fdb_pose = msg.poses[0]  # Assuming only one tag is detected
        else:
            self.fdb_pose = None

    def status_callback(self, msg):
        self.latest_status = msg

    def get_result(self):
        return self.control_result

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

    def publish_result(self):
        self.result_pub.publish(Int32(data=self.control_result))

    def control_loop(self):
        # Check for timeout
        if self.calculate_elapsed_time() > self.time_out:
            self.control_result = 1  # Failure
            self.get_logger().info('Timeout reached')
            self.publish_twist(0.0, 0.0)
            self.publish_result()
            self.timer.cancel()  # Stop the timer
            self.shutdown()
            return

        # Check the latest status message
        # log self.latest_status.subindex024
        # if self.latest_status:
        # self.get_logger().info(f"subindex024: {self.latest_status.subindex024}")

        if self.latest_status and self.latest_status.subindex024 > 0.5:
            self.get_logger().info('subindex024 is greater than 0.5, acting as goal reached')
            self.reset_start_time()
            self.control_result = 0  # Success
            self.publish_twist(0.0, 0.0)
            self.publish_result()
            self.timer.cancel()  # Stop the timer
            self.shutdown()
            return

        if self.fdb_pose is None:
            self.get_logger().info('No tag detected yet')
            self.reset_start_time()
            self.publish_twist(0.0, 0.0)
            self.control_result = 2  # Running
            self.publish_result()
            return

        # Calculate velocities
        cur = self.fdb_pose
        goal = self.target_pose
        vel, omega = self.controller.get_velocity(cur, goal, 0.1, direction=-1)

        # Check if the robot has reached the goal
        if self.controller.at_goal(goal, cur):
            self.get_logger().info('Goal reached')
            self.reset_start_time()
            vel = 0.0
            omega = 0.0
            self.control_result = 0  # Success
            self.publish_twist(0.0, 0.0)
            self.publish_result()
            self.timer.cancel()  # Stop the timer
            self.shutdown()
            return

        else:
            self.control_result = 2  # Running

        self.publish_twist(vel, omega)
        self.publish_result()

    def shutdown(self):
        # Stop publishing twist messages
        self.publish_twist(0.0, 0.0)
        # Shutdown rclpy
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    # Get the path to the package
    package_path = get_package_path('b1_perception')
    # if package_path:
    #     print(f"The path to the package is: {package_path}")

    docking_path = os.path.join(package_path + '/config/docking_config.json')

    with open(docking_path, 'r') as f:
        docking_config = json.load(f)

    # Define the target pose using data from the JSON file
    args = parse_arguments()
    goal_name = args.goal_name

    target_pose = Pose()
    target_pose.position.x = docking_config[goal_name]["position"]["x"]
    target_pose.position.y = docking_config[goal_name]["position"]["y"]
    target_pose.position.z = docking_config[goal_name]["position"]["z"]
    target_pose.orientation.x = docking_config[goal_name]["quaternion"]["x"]
    target_pose.orientation.y = docking_config[goal_name]["quaternion"]["y"]
    target_pose.orientation.z = docking_config[goal_name]["quaternion"]["z"]
    target_pose.orientation.w = docking_config[goal_name]["quaternion"]["w"]

    # Print pose
    # print("target_pose: ", target_pose)

    # Create the control node
    control_node = ControlNode(target_pose, time_out=60.0)

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
