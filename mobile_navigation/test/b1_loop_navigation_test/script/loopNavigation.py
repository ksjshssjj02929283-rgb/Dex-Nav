import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time
import json
import os

import sys
import signal
import atexit

script_dir = os.path.dirname(__file__) #<-- absolute dir the script is in
relative_dir = "../json/"
file_store_dir = os.path.join(script_dir, relative_dir)

class RobotNavigation:

    def __init__(self, list_pose_json_file='/home/grabotics/B1-Project-new/b1_devel/b1_loop_navigation_test/json/grabotics_nav_waypoints.json'):
        # rclpy.init()
        self.nav_waypoints = self.load_waypoints_from_json(list_pose_json_file)
        print('The amount of waypoints are:%d'%len(self.nav_waypoints))
        self.waypoint_index = 0
        self.nav = BasicNavigator() 
        print('Init succeed...')
        
        signal.signal(signal.SIGINT, self.signal_handler)
        atexit.register(self.stop_robot)
        

    def load_waypoints_from_json(self, list_pose_json_file):
        try:
            with open(list_pose_json_file, 'r') as json_file:
                waypoints = json.load(json_file)
            return waypoints
        except FileNotFoundError:
            self.get_logger().error('JSON file not found. Make sure to create nav_goals.json')
            return []
    
    def signal_handler(self, sig, frame):
        print('Pressed Ctrl+C!')
        self.stop_robot()
        sys.exit(0)
        
    def stop_robot(self):
        self.nav.stop()
        print('Robot stopped!')
        
    # 单点导航
    def single_waypoint_nav(self, nav_waypoint, wait_time):
        # nav = BasicNavigator()
        # nav.waitUntilNav2Active()  
        # if nav_waypoint in self.nav_waypoints:
            print('Navigate to waypoint %d ...'%(self.waypoint_index))
            self.nav.goToPose(nav_waypoint)
            
            i = 0
            while not self.nav.isTaskComplete(): 
                i += 1
                feedback = self.nav.getFeedback()
                if feedback and i % 5 == 0:
                    print('Estimated distance remaining to goal position: ' +
                        '{0:.3f}'.format(feedback.distance_remaining))
                   
            print('Goal succeeded!')                     
            time.sleep(wait_time)  # wait for the specified amount of time before moving to the next pose
       
    # 循环连续导航                  
    def continuous_navigation(self, wait_time=0):
        # self.nav.waitUntilNav2Active('bt_navigator','cartographer_node')
        self.nav._waitForNodeToActivate('bt_navigator')
        self.nav.info('Nav2 is ready for use!')
        print('Start loop navigation...')
        while rclpy.ok() and len(self.nav_waypoints) > 0:
            # print('Start loop navigation...')
            current_goal = self.nav_waypoints[self.waypoint_index]
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = current_goal['position']['x']
            pose.pose.position.y = current_goal['position']['y']
            pose.pose.position.z = current_goal['position']['z']
            pose.pose.orientation.x = current_goal['orientation']['x']
            pose.pose.orientation.y = current_goal['orientation']['y']
            pose.pose.orientation.z = current_goal['orientation']['z']
            pose.pose.orientation.w = current_goal['orientation']['w']
            
            self.single_waypoint_nav(pose, wait_time)
            self.waypoint_index = (self.waypoint_index + 1) % len(self.nav_waypoints)

def main(args=None):
    rclpy.init(args=args)
    nav_waypoints_loop = RobotNavigation(file_store_dir)
    nav_waypoints_loop.continuous_navigation()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
