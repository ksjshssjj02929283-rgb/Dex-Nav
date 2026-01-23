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
from goal_controller import GoalController


class PIDController:

    def __init__(self,
                 kp,
                 ki=0.0,
                 kd=0.0,
                 output_limits=(None, None),
                 integral_limits=(None, None),
                 zone_limit=None,
                 timeout=None,
                 tolerance=None,
                 tolerance_duration=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0.0
        self.integral = 0.0
        self.output_min, self.output_max = output_limits
        self.integral_min, self.integral_max = integral_limits
        self.zone_limit = zone_limit
        self.timeout = timeout
        self.tolerance = tolerance
        self.tolerance_duration = tolerance_duration
        self.start_time = time.time()
        self.tolerance_start_time = None
        self.previous_time = self.start_time

    def compute_command(self, error):
        current_time = time.time()
        delta_time = current_time - self.previous_time
        self.previous_time = current_time

        # Check timeout
        if self.timeout is not None and (current_time -
                                         self.start_time) > self.timeout:
            return 0.0  # Stop the PID controller

        # Check tolerance
        if self.tolerance is not None and abs(error) < self.tolerance:
            if self.tolerance_start_time is None:
                self.tolerance_start_time = current_time
            elif (current_time -
                  self.tolerance_start_time) >= self.tolerance_duration:
                return 0.0  # Stop the PID controller
        else:
            self.tolerance_start_time = None

        # Calculate integral and derivative components
        if self.zone_limit is None or abs(error) < self.zone_limit:
            self.integral += error * delta_time

        # Apply integral limits (anti-windup) to the term ki * integral
        integral_term = self.ki * self.integral
        if self.integral_min is not None:
            integral_term = max(self.integral_min, integral_term)
        if self.integral_max is not None:
            integral_term = min(self.integral_max, integral_term)

        derivative = (
            error - self.previous_error) / delta_time if delta_time > 0 else 0.0

        # Calculate PID output
        output = self.kp * error + integral_term + self.kd * derivative

        # Apply output limits (saturation)
        if self.output_min is not None:
            output = max(self.output_min, output)
        if self.output_max is not None:
            output = min(self.output_max, output)

        # Save error for next derivative calculation
        self.previous_error = error

        return output

    def reset(self):
        """Reset the PID controller."""
        self.previous_error = 0.0
        self.integral = 0.0
        self.start_time = time.time()
        self.tolerance_start_time = None


class ControlNode(Node):

    def __init__(self, target_pose):
        super().__init__('control_node')

        # Target pose in the robot base link frame
        self.target_pose = target_pose
        self.tolerance = 0.05

        # PID controllers for x, y, and yaw (theta) axes
        self.pid_x = PIDController(kp=0.5,
                                   ki=0.001,
                                   kd=0.01,
                                   output_limits=(-0.5, 0.5),
                                   integral_limits=(-0.10, 0.10),
                                   zone_limit=None,
                                   timeout=None,
                                   tolerance=0.01,
                                   tolerance_duration=0.5)

        self.pid_yaw = PIDController(kp=0.50,
                                     ki=0.0001,
                                     kd=0.0015,
                                     output_limits=(-0.5, 0.5),
                                     integral_limits=(-0.15, 0.15),
                                     zone_limit=None,
                                     timeout=None,
                                     tolerance=0.01,
                                     tolerance_duration=0.5)
        # Publisher for Result, 0, 1, 2 for Success, Failure, Running
        # Todo: Add a publisher for the result
        self.control_result = 2

        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber for AprilTag poses
        self.pose_sub = self.create_subscription(PoseArray, 'apriltag_poses',
                                                 self.pose_callback, 10)

        # TF buffer and listener for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Variable to store the latest detected AprilTag pose
        self.fdb_pose = None
        self.x_reached = False
        self.yaw_reached = False
        self.account = 0

        self.previous_immediate_yaw = None
        self.cumulative_yaw = 0.0
        self.y_control_account = 0

        # For managing the sequence of motions
        self.move_commands = []
        self.current_command_index = 0

        self.controller = GoalController()

    def pose_callback(self, msg):
        # Check if the pose list is empty or None
        # print("msg.poses: ", msg.poses)
        # msg.poses:  []
        if msg.poses:
            self.fdb_pose = msg.poses[0]  # Assuming only one tag is detected
        else:
            self.fdb_pose = None
            self.get_logger().info('No tag detected')

    def cleanup_controller(self):
        self.pid_x.reset()
        self.pid_yaw.reset()
        # Reset command list and index
        self.move_commands = []
        self.current_command_index = 0
        # Stop the robot
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Robot stopped')

    def control_x(self, error_x, twist):
        # move forward
        control_signal_x = self.pid_x.compute_command(error_x)
        twist.linear.x = control_signal_x
        twist.angular.z = 0.00
        self.cmd_vel_pub.publish(twist)
        # log control_signal_x
        # self.get_logger().info(f'control_signal_x: {control_signal_x}')
        if abs(error_x) < self.pid_x.tolerance:
            self.x_reached = True
            self.get_logger().info('X reached')
            self.cleanup_controller()
            self.pid_x.reset()
            return

    def control_yaw(self, error_yaw, twist):
        control_signal_yaw = self.pid_x.compute_command(error_yaw)
        twist.angular.z = control_signal_yaw
        twist.linear.x = 0.00
        self.cmd_vel_pub.publish(twist)
        # log control_signal_x
        # self.get_logger().info(f'control_signal_yaw: {control_signal_yaw}')
        if abs(error_yaw) < self.pid_yaw.tolerance:
            self.yaw_reached = True
            self.get_logger().info('Yaw reached')
            # self.cleanup_controller()
            self.pid_yaw.reset()
            return

    def get_result(self):
        return self.control_result

    def control_loop(self):
        twist = Twist()
        if self.fdb_pose is None:
            self.get_logger().info('No tag detected yet')
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return  # Exit if no target detected
        else:
            cur = self.fdb_pose
            goal = self.target_pose
            vel, omega = self.controller.get_velocity(cur, goal, 0.1)

            # Check if the robot has reached the goal
            if self.controller.at_goal(goal, cur):
                print("Goal reached.....")
                vel = 0.0
                omega = 0.0
                self.control_result = 0  #'Success'
            else:
                self.control_result = 2  #'Running'

            twist.linear.x = vel
            twist.angular.z = omega  #0.0 susan change
            self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    # Define the target pose using ROS Pose structure
    target_pose = Pose()
    target_pose.position.x = 0.7
    target_pose.position.y = 0.0
    target_pose.position.z = 0.22417
    target_pose.orientation.x = 0.7074599806708379
    target_pose.orientation.y = 0.0003533762606984598
    target_pose.orientation.z = -0.7067532281494411
    target_pose.orientation.w = 0.00035408301321985673

    # small error
    # target_pose.orientation.x = -0.7050926596521246
    # target_pose.orientation.y=-0.053332366389399165
    # target_pose.orientation.z=0.7050926596521246
    # target_pose.orientation.w=-0.053332366389399165

    # # 11 deg error
    # target_pose.orientation.x=-0.7035391593941575
    # target_pose.orientation.y=-0.07094118126280587
    # target_pose.orientation.z=0.7035391593941575
    # target_pose.orientation.w=-0.07094118126280587

    #
    # x=0.7074599806708379, y=0.0003533762606984598, z=-0.7067532281494411, w=0.00035408301321985673
    # x=2.24587, y=0.00083, z=0.22417

    # Create the control node
    control_node = ControlNode(target_pose)

    def sigint_handler(signal, frame):
        control_node.get_logger().info('SIGINT received, shutting down...')
        control_node.cleanup_controller()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sigint_handler)

    try:
        rclpy.spin(control_node)
    finally:
        control_node.cleanup_controller()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
