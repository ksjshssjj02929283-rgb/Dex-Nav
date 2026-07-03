#!/usr/bin/env python3
import time
import json
import argparse
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from rclpy.parameter import ParameterValue
from std_msgs.msg import Header

class Pose3D:
    def __init__(self, x=0, y=0, z=0, qx=0, qy=0, qz=0, qw=1):
        self.pose_stamped = PoseStamped()
        self.pose_stamped.pose.position = Point()
        self.pose_stamped.pose.position.x = x
        self.pose_stamped.pose.position.y = y
        self.pose_stamped.pose.position.z = z
        self.pose_stamped.pose.orientation = Quaternion()
        self.pose_stamped.pose.orientation.x = qx
        self.pose_stamped.pose.orientation.y = qy
        self.pose_stamped.pose.orientation.z = qz
        self.pose_stamped.pose.orientation.w = qw
        self.pose_stamped.header = Header()
        self.pose_stamped.header.frame_id = 'map'


class RobotNavigation:

    def __init__(self, pose_name=None):
        self.pose_name = pose_name

    def buddy_move(self):
        with open("/home/susan/BuddyDemoAGI/src/buddy_AGI/json/poses.json", "r") as file:
            data = json.load(file)

        # Convert the JSON data to a dictionary of Pose3D objects
        poses = {}
        for item in data["poses"]:
            position = item["position"]
            orientation = item["orientation"]
            pose = Pose3D(position["x"], position["y"], position["z"],
                        orientation["x"], orientation["y"], orientation["z"], orientation["w"])
            pose_name = item["pose_name"]
            poses[pose_name] = pose.pose_stamped

        # Existing code...
class RobotNavigation:

    def __init__(self):
        self.poses = self.load_poses()

    def load_poses(self):
        with open("/home/susan/BuddyDemoAGI/src/buddy_AGI/json/poses.json", "r") as file:
            data = json.load(file)

        # Convert the JSON data to a dictionary of Pose3D objects
        poses = {}
        for item in data["poses"]:
            position = item["position"]
            orientation = item["orientation"]
            pose = Pose3D(position["x"], position["y"], position["z"],
                          orientation["x"], orientation["y"], orientation["z"], orientation["w"])
            pose_name = item["pose_name"]
            poses[pose_name] = pose.pose_stamped
        return poses

    def continuous_navigation(self):
        nav = BasicNavigator()
        nav.waitUntilNav2Active()
        while True:
            try:
                pose_name = input("Enter the pose name to navigate to, or type 'quit' to exit: ")
                if pose_name.lower() == 'quit':
                    print('Exiting...')
                    break

                if pose_name in self.poses:
                    target_pose = self.poses[pose_name]
                    nav.goThroughPoses([target_pose])
                    position = target_pose.pose.position
                    orientation = target_pose.pose.orientation
                    print(f"Target pose position: x={position.x}, y={position.y}, z={position.z}")
                    print(f"Target pose orientation: x={orientation.x}, y={orientation.y}, w={orientation.w}")

                    # Updated feedback section
                    while nav.feedback is None:
                        rclpy.spin_once(nav)
                        nav.spin()                         
                    feedback = nav.getFeedback()  
                    print("Position feedback: ", feedback )  
                else:
                    print(f"Pose name '{pose_name}' not found in the JSON file.")

                result = nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    print('Goal succeeded!')
                elif result == TaskResult.CANCELED:
                    print('Goal was canceled!')
                elif result == TaskResult.FAILED:
                    print('Goal failed!')

            except KeyboardInterrupt:
                print('Interrupted by Ctrl+c')
                break

if __name__ == '__main__':
    rclpy.init()  # Add this line to initialize rclpy before using it
    robot_navigation = RobotNavigation()
    robot_navigation.continuous_navigation()
    rclpy.shutdown()  # Add this line to properly shut down rclpy after using it

