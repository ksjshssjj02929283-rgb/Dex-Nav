#!/usr/bin/env python3
import rclpy
import rclpy.duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import rclpy.time
from tf2_ros import Buffer, TransformListener
# from tf2_ros import LookupTransform
# import tf_transformations

class BaseLinkPosition(Node):
    def __init__(self):
        super().__init__("baselink_position")
        self.tf2_buffer = Buffer()
        self.listener = TransformListener(self.tf2_buffer,self)

        self.publisher = self.create_publisher(PoseStamped, '/feedback/chassis_pos', 10)
        self.timer = self.create_timer(0.1, self.get_position)

        self.get_logger().info("Publish Chassis Position! ")

    def get_position(self):
        try:
            transform = self.tf2_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            translation = transform.transform.translation
            orientation = transform.transform.rotation

            pose_msg = PoseStamped()

            pose_msg.header.frame_id = 'map'

            pose_msg.pose.position.x = translation.x
            pose_msg.pose.position.y = translation.y
            pose_msg.pose.position.z = 0.0

            pose_msg.pose.orientation = orientation
            self.publisher.publish(pose_msg)
        except(LookupError) as e:
            self.get_logger().error(f'TF error:{str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = BaseLinkPosition()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()