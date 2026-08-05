#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

class FakeAMCL(Node):
    def __init__(self):
        super().__init__('fake_amcl')
        self.latest_odom = None
        
        # 订阅 Gazebo 里程计
        self.odom_sub = self.create_subscription(
            Odometry, '/feedback/odom', self.odom_callback, 10)
        
        # 发布话题
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.cloud_pub = self.create_publisher(
            PoseArray, '/particlecloud', 10)
            
        # TF 广播
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 限制在 10Hz 触发发布，释放 DDS 通信带宽，防止网络堵塞导致状态机超时
        self.timer = self.create_timer(0.1, self.publish_localization)
        self.get_logger().info("Fake AMCL localization node initialized (10Hz throttle)!")

    def odom_callback(self, msg):
        self.latest_odom = msg

    def publish_localization(self):
        if self.latest_odom is None:
            return
            
        msg = self.latest_odom
        
        # 1. 广播 map -> odom TF
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # 2. 发布 amcl_pose
        amcl_pose = PoseWithCovarianceStamped()
        amcl_pose.header.stamp = msg.header.stamp
        amcl_pose.header.frame_id = 'map'
        amcl_pose.pose.pose = msg.pose.pose
        amcl_pose.pose.covariance = [0.0] * 36
        amcl_pose.pose.covariance[0] = 0.01
        amcl_pose.pose.covariance[7] = 0.01
        amcl_pose.pose.covariance[35] = 0.01
        self.pose_pub.publish(amcl_pose)

        # 3. 发布 particlecloud
        cloud = PoseArray()
        cloud.header.stamp = msg.header.stamp
        cloud.header.frame_id = 'map'
        cloud.poses.append(msg.pose.pose)
        self.cloud_pub.publish(cloud)

def main(args=None):
    rclpy.init(args=args)
    node = FakeAMCL()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
