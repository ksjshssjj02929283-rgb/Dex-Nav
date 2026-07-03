import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import json

class NavGoalRecorder(Node):

    def __init__(self):
        super().__init__('nav_goal_recorder')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.clicked_point_callback,
            10)
        self.nav_goals = []

    def clicked_point_callback(self, msg):
        # Assuming the '/clicked_point' topic publishes Path messages with a single pose
        print("Record waypoint...")
        pose = msg.pose
        goal = {
            'position': {
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z
            },
            'orientation': {
                'x': pose.orientation.x,
                'y': pose.orientation.y,
                'z': pose.orientation.z,
                'w': pose.orientation.w
            }
        }
        self.nav_goals.append(goal)
        self.save_goals_to_json()

    def save_goals_to_json(self):
        with open('/home/hz/hz/Work/Grabotics/b1-simulation/b1-simulation_v1/src/b1_simulation_v1/buddy_nav2/json/grabotics_nav_waypoints.json', 'w') as json_file:
            json.dump(self.nav_goals, json_file, indent=4)

def main(args=None):
    rclpy.init(args=args)
    node = NavGoalRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
