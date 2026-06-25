#!/usr/bin/env python3
# 假机器人 + DockRobot 闭环测试：积分 cmd_vel -> 发 odom->base_link TF + battery，
# 发 DockRobot goal(home_dock, 跳过 staging 导航)，等结果。
import math, time, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformBroadcaster
from docking_base_msgs.action import DockRobot


class FakeRobotDock(Node):
    def __init__(self):
        super().__init__('fake_robot_dock')
        self.x = self.y = self.th = 0.0
        self.cmd = Twist()
        self.is_charging = False
        self.dock = (2.0, 0.0)
        self.tf = TransformBroadcaster(self)
        self.batt = self.create_publisher(BatteryState, 'battery_state', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_timer(0.05, self.tick)
        self.ac = ActionClient(self, DockRobot, 'dock_robot')
        self.result = None

    def on_cmd(self, m):
        self.cmd = m

    def tick(self):
        dt = 0.05
        self.x += (math.cos(self.th) * self.cmd.linear.x - math.sin(self.th) * self.cmd.linear.y) * dt
        self.y += (math.sin(self.th) * self.cmd.linear.x + math.cos(self.th) * self.cmd.linear.y) * dt
        self.th += self.cmd.angular.z * dt
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)
        self.tf.sendTransform(t)
        d = math.hypot(self.dock[0] - self.x, self.dock[1] - self.y)
        if d < 0.2:
            self.is_charging = True
        b = BatteryState()
        b.current = 1.0 if self.is_charging else -1.0
        self.batt.publish(b)

    def send(self):
        self.ac.wait_for_server()
        g = DockRobot.Goal()
        g.use_dock_id = True
        g.dock_id = 'home_dock'
        g.navigate_to_staging_pose = False
        self.ac.send_goal_async(g).add_done_callback(self.on_goal)

    def on_goal(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.result = 'REJECTED'
            return
        gh.get_result_async().add_done_callback(self.on_result)

    def on_result(self, fut):
        r = fut.result().result
        self.result = 'success=%s error_code=%s num_retries=%s' % (r.success, r.error_code, r.num_retries)


def main():
    rclpy.init()
    n = FakeRobotDock()
    end = time.time() + 3                       # 预热 TF
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.get_logger().info('sending DockRobot goal...')
    n.send()
    end = time.time() + 45
    while time.time() < end and n.result is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    print('DOCK_RESULT:', n.result, '| robot=(%.3f, %.3f)' % (n.x, n.y))
    n.destroy_node()
    rclpy.shutdown()


main()
