#!/usr/bin/env python3
import os
import json
import time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory


class ChargingPoseSaver(Node):

    def __init__(self):
        super().__init__('charging_pose_saver')

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 保存路径：mobile_perception/config
        pkg_path = get_package_share_directory('mobile_perception')
        self.save_path = os.path.join(pkg_path, 'config', 'charging_pose.json')

        # Service
        self.srv = self.create_service(Trigger, 'save_charging_pose', self.handle_save_request)

        self.get_logger().info(f'Charging pose saver service ready. Save path: {self.save_path}')

    # ===============================
    # Service callback
    # ===============================
    def handle_save_request(self, request, response):
        pose = self.get_pose()

        if pose is None:
            response.success = False
            response.message = 'Failed to get TF: map -> base_link'
            return response

        self.save_pose(pose)

        # 组织返回信息
        msg = {
            'frame_id': pose.header.frame_id,
            'position': {
                'x': pose.pose.position.x,
                'y': pose.pose.position.y,
                'z': pose.pose.position.z
            },
            'orientation':
                {
                    'x': pose.pose.orientation.x,
                    'y': pose.pose.orientation.y,
                    'z': pose.pose.orientation.z,
                    'w': pose.pose.orientation.w
                }
        }

        response.success = True
        response.message = json.dumps(msg, ensure_ascii=False)

        return response

    # ===============================
    # TF 取均值
    # ===============================
    def get_pose(self):
        """
        检测 5 次 TF，每次最多重试 20 次，取均值
        """
        positions = []
        orientations = []
        header = None

        for i in range(5):
            trans = None
            for retry in range(20):
                try:
                    trans = self.tf_buffer.lookup_transform(
                        'map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2))
                    break
                except Exception as e:
                    self.get_logger().warn(f'第{i+1}次 TF 查找重试 {retry+1}: {e}')
                    rclpy.spin_once(self, timeout_sec=0.2)
                    time.sleep(0.1)

            if trans is None:
                self.get_logger().error(f'第{i+1}次 TF 查找最终失败')
                return None

            if header is None:
                header = trans.header

            positions.append(
                [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])

            orientations.append(
                [
                    trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z,
                    trans.transform.rotation.w
                ])

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

    # ===============================
    # 保存 JSON
    # ===============================
    def save_pose(self, pose: PoseStamped):
        data = {
            'frame_id': pose.header.frame_id,
            'position': {
                'x': pose.pose.position.x,
                'y': pose.pose.position.y,
                'z': pose.pose.position.z
            },
            'orientation':
                {
                    'x': pose.pose.orientation.x,
                    'y': pose.pose.orientation.y,
                    'z': pose.pose.orientation.z,
                    'w': pose.pose.orientation.w
                }
        }

        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)

        with open(self.save_path, 'w') as f:
            json.dump(data, f, indent=2)

        self.get_logger().info(f'充电点已保存到 {self.save_path}')


def main():
    rclpy.init()
    node = ChargingPoseSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
