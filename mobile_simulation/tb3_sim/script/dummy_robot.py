#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import math

class DummyRobot(Node):
    def __init__(self):
        super().__init__('dummy_robot')
        self.declare_parameter('x', -2.0)
        self.declare_parameter('y', -0.5)
        self.declare_parameter('yaw', 0.0)
        
        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.yaw = self.get_parameter('yaw').value
        
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        
        # 强制该虚拟仿真节点使用系统真实时间驱动，防止在没有 /clock 话题时挂起崩溃
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, False)])
        self.last_time = self.get_clock().now()
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
            
        self.odom_pub = self.create_publisher(
            Odometry, 'odom', 10)
            
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.timer = self.create_timer(0.02, self.update_pose) # 50Hz
        self.get_logger().info("Dummy Robot simulator initialized at x=%.2f, y=%.2f, yaw=%.2f" % (self.x, self.y, self.yaw))
        
    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z
        
    def update_pose(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0 or dt > 0.5:
            # 过滤异常大延迟
            return
            
        # 差速机器人运动学积分
        delta_x = (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        delta_y = (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
        delta_yaw = self.wz * dt
        
        self.x += delta_x
        self.y += delta_y
        self.yaw += delta_yaw
        
        # 1. 发布 TF: odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        
        self.tf_broadcaster.sendTransform(t)
        
        # 2. 发布 Odom 话题
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = t.transform.rotation
        
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz
        
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = DummyRobot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
