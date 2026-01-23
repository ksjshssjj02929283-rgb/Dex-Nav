#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import json
import os

SAVE_PATH = os.path.join(os.path.dirname(__file__), 'charging_pose.json')


class ChargingPoseSaver(Node):

    def __init__(self):
        super().__init__('charging_pose_saver')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_pose(self):
        # 检测5次，取均值，每次最多重试20次（约4秒）
        import time
        positions = []
        orientations = []
        header = None
        for i in range(5):
            trans = None
            for retry in range(20):
                try:
                    trans = self.tf_buffer.lookup_transform(
                        'map',
                        'base_link',
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.2))
                    break
                except Exception as e:
                    self.get_logger().warn(f"第{i+1}次TF查找重试{retry+1}: {e}")
                    rclpy.spin_once(self, timeout_sec=0.2)
                    time.sleep(0.1)
            if trans is None:
                self.get_logger().error(f"第{i+1}次TF查找最终失败")
                return None
            if header is None:
                header = trans.header
            positions.append([
                trans.transform.translation.x, trans.transform.translation.y,
                trans.transform.translation.z
            ])
            orientations.append([
                trans.transform.rotation.x, trans.transform.rotation.y,
                trans.transform.rotation.z, trans.transform.rotation.w
            ])
        # 计算均值
        import numpy as np
        pos_mean = np.mean(positions, axis=0)
        ori_mean = np.mean(orientations, axis=0)
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = float(pos_mean[0])
        pose.pose.position.y = float(pos_mean[1])
        pose.pose.position.z = float(pos_mean[2])
        pose.pose.orientation.x = float(ori_mean[0])
        pose.pose.orientation.y = float(ori_mean[1])
        pose.pose.orientation.z = float(ori_mean[2])
        pose.pose.orientation.w = float(ori_mean[3])
        return pose

    def save_pose(self, pose):
        data = {
            'frame_id': pose.header.frame_id,
            'position': {
                'x': pose.pose.position.x,
                'y': pose.pose.position.y,
                'z': pose.pose.position.z
            },
            'orientation': {
                'x': pose.pose.orientation.x,
                'y': pose.pose.orientation.y,
                'z': pose.pose.orientation.z,
                'w': pose.pose.orientation.w
            }
        }
        with open(SAVE_PATH, 'w') as f:
            json.dump(data, f, indent=2)
        self.get_logger().info(f"充电点已保存到 {SAVE_PATH}")


def main():
    rclpy.init()
    node = ChargingPoseSaver()
    rclpy.spin_once(node, timeout_sec=1.0)  # 等待TF缓存
    pose = node.get_pose()
    if pose:
        node.save_pose(pose)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
