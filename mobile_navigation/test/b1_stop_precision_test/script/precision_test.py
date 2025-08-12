import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion
import time
import json
import os
import sys
import signal
import atexit

script_dir = os.path.dirname(__file__)  # <-- absolute dir the script is in
relative_dir = "../json/test_poses.json"
file_store_dir = os.path.join(script_dir, relative_dir)

class RobotNavigation(Node):

    def __init__(self, list_pose_json_file='/home/Projects/b1_main_v1/b1_devel/b1_navigation/test/b1_stop_precision_test/json/test_poses.json'):
        super().__init__('robot_navigation')
        self.nav_waypoints = self.load_waypoints_from_json(list_pose_json_file)
        print('The amount of waypoints are: %d' % len(self.nav_waypoints))
        self.waypoint_index = 0
        self.nav = BasicNavigator() 
        print('Init succeed...')

        signal.signal(signal.SIGINT, self.signal_handler)
        # atexit.register(self.stop_task)

        # 创建一个订阅器来监听定位话题
        self.tracked_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/tracked_pose',
            self.tracked_pose_callback,
            10
        )

        # 存储最后一次接收的定位位姿
        self.last_tracked_pose = None

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
        self.stop_task()
        sys.exit(0)

    def stop_task(self):
        self.nav.cancelTask()
        print('Cancelled nav task!')

    def tracked_pose_callback(self, msg):
        # 存储最后一次接收到的定位位姿
        self.last_tracked_pose = msg.pose

    def calculate_error(self, target_pose, actual_pose):
        # 计算x, y, yaw误差
        dx = actual_pose.position.x - target_pose.position.x
        dy = actual_pose.position.y - target_pose.position.y
        q_target = [target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w]
        q_actual = [actual_pose.orientation.x, actual_pose.orientation.y, actual_pose.orientation.z, actual_pose.orientation.w]
        _, _, yaw_target = euler_from_quaternion(q_target)
        _, _, yaw_actual = euler_from_quaternion(q_actual)
        dyaw = yaw_actual - yaw_target
        return dx, dy, dyaw

    def single_waypoint_nav(self, nav_pose, wait_time):
        self.nav.goToPose(nav_pose)

        i = 0
        while not self.nav.isTaskComplete():
            i += 1
            feedback = self.nav.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated distance remaining to goal position: %.3f' % feedback.distance_remaining)
            # 这里调用 spin_once 来处理事件循环
            rclpy.spin_once(self, timeout_sec=0.05)

        if self.nav.isTaskComplete():
            print('Goal succeeded!')
            # 获取实际定位位姿
            actual_pose = self.last_tracked_pose
            if actual_pose is not None:
                dx, dy, dyaw = self.calculate_error(nav_pose.pose, actual_pose)
                print('X error: %.3f, Y error: %.3f, Yaw error: %.3f' % (dx, dy, dyaw))
            else:
                print('No tracked pose received yet.')
        else:
            print('Goal failed!')

        time.sleep(wait_time)  # Wait for the specified amount of time before moving to the next pose

    def continuous_navigation(self, repeat_times=1, wait_time=1):
        self.nav._waitForNodeToActivate('bt_navigator')
        self.nav.info('Nav2 is ready for use!')
        print('Start navigation test, repeat %d times navigation' % repeat_times)
        repeat_count = 0
        while rclpy.ok() and len(self.nav_waypoints) > 0:
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

            if self.waypoint_index == 0:
                repeat_count += 1
                time.sleep(wait_time)
                if repeat_count >= repeat_times:
                    break

def main(args=None):
    rclpy.init(args=args)
    nav_precesion_test = RobotNavigation(file_store_dir)
    nav_precesion_test.continuous_navigation(5)
    nav_precesion_test.stop_task()
    # 关闭事件循环
    rclpy.shutdown()

if __name__ == '__main__':
    main()