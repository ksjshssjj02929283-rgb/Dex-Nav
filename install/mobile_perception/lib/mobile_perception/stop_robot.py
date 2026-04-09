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
from std_msgs.msg import Int32
import json

import ament_index_python.packages  
import os

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
        self.pose_sub = self.create_subscription(
            PoseArray,
            'apriltag_poses',
            self.pose_callback,
            10
        )
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Variable to store the latest detected AprilTag pose
        self.fdb_pose = None  
        
        # Controller object, calculate velocity from current pose to goal pose
        self.controller = GoalController()
        
    def pose_callback(self, msg):
        # Check if the pose list is empty or None  
        if  msg.poses:
            self.fdb_pose = msg.poses[0]  # Assuming only one tag is detected 
        else:
            self.fdb_pose = None
            # self.get_logger().info('No tag detected')

    # Result, 0, 1, 2 for Success, Failure, Running
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
        # self.publish_twist(vel, omega)
        self.publish_twist(0.0, 0.0)  
    
def main(args=None):
    rclpy.init(args=args)
    
    # Get the path to the package
    package_path = get_package_path('b1_perception')
    # if package_path:
    #     print(f"The path to the package is: {package_path}") 
    
    docking_path = os.path.join(package_path+ '/config/docking_config.json') 
    
    with open(docking_path, 'r') as f:
        docking_config = json.load(f)

    # Define the target pose using data from the JSON file
    target_pose = Pose()
    target_pose.position.x = docking_config["pre_charging_pose_03"]["position"]["x"]
    target_pose.position.y = docking_config["pre_charging_pose_03"]["position"]["y"]
    target_pose.position.z = docking_config["pre_charging_pose_03"]["position"]["z"]
    target_pose.orientation.x = docking_config["pre_charging_pose_03"]["quaternion"]["x"]
    target_pose.orientation.y = docking_config["pre_charging_pose_03"]["quaternion"]["y"]
    target_pose.orientation.z = docking_config["pre_charging_pose_03"]["quaternion"]["z"]
    target_pose.orientation.w = docking_config["pre_charging_pose_03"]["quaternion"]["w"]

    # print pose
    # print("target_pose: ", target_pose)
    
    # # Create the control node
    control_node = ControlNode(target_pose, time_out=60.0)

    def sigint_handler(signal, frame):
        control_node.get_logger().info('SIGINT received, shutting down...') 
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sigint_handler)
    
    try:
        rclpy.spin(control_node)
    finally: 
        rclpy.shutdown()

if __name__ == '__main__':
    main()
