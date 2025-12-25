#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray

from tf2_ros import Buffer, TransformListener

from cv_bridge import CvBridge
from pupil_apriltags import Detector

import numpy as np
import cv2
import json
import os
import ament_index_python.packages

from scipy.spatial.transform import Rotation as R
from chassis_interfaces.srv import StartDetection, StopDetection


def get_package_path(pkg):
    return ament_index_python.packages.get_package_share_directory(pkg)


class AprilTagDetectionNode(Node):

    def __init__(self):
        super().__init__('apriltag_detection_node')

        # ---------------- load config ----------------
        pkg = get_package_path('mobile_perception')
        cam_cfg = os.path.join(pkg, 'config/camera_config.json')
        tag_cfg = os.path.join(pkg, 'config/tags_config.json')

        self.camera_config = self._load_json(cam_cfg)
        self.tag_config = self._load_json(tag_cfg)

        cam = self.camera_config['cameras'][0]
        self.camera_link = cam['camera_link']
        self.image_topic = cam['topic_name']
        self.tag_size = self.tag_config['tag_size']

        self.last_detect_time = 0.0
        self.detect_period = 0.2 # 5 Hz


        # ---------------- apriltag ----------------
        self.detector = Detector(
            families=self.tag_config['tag_family']
        )
        self.bridge = CvBridge()

        # ---------------- TF listener ONLY ----------------
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- ROS ----------------
        self.image_sub = None
        self.is_detecting = False

        self.pose_pub = self.create_publisher(
            PoseArray, 'apriltag_poses', 10
        )

        self.create_service(
            StartDetection,
            'start_apriltag_detection',
            self.start_cb
        )
        self.create_service(
            StopDetection,
            'stop_apriltag_detection',
            self.stop_cb
        )

        self.get_logger().info('AprilTag Detection Node (NO TF broadcast) ready.')

    # ==================================================
    # Services
    # ==================================================

    def start_cb(self, req, res):
        if self.is_detecting:
            res.success = False
            res.message = 'Detection already running'
            return res
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_cb, 10
        )
        self.detector = Detector(
            families=self.tag_config['tag_family'],
            nthreads=1
        )
        self.is_detecting = True
        res.success = True
        res.message = f'Detection started: {self.image_topic}'
        self.get_logger().info(res.message)
        return res

    def stop_cb(self, req, res):
        if self.image_sub:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
        
        # if self.tf_listener:
        #     self.tf_listener = None
        #     self.tf_buffer = None

        self.is_detecting = False

        # if self.detector:
        #     self.detector = None

        res.success = True
        res.message = 'Detection stopped'
        self.get_logger().info(res.message)
        return res

    # ==================================================
    # Image callback
    # ==================================================

    def image_cb(self, msg):
        if not self.is_detecting:
            return
        
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_detect_time < self.detect_period:
            return
        self.last_detect_time = now

        # ---------- TF lookup (base <- camera) ----------
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link',
                self.camera_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
        except Exception as e:
            self.get_logger().warn(
                f'TF lookup failed: {e}',
                throttle_duration_sec=2.0
            )
            return

        # ---------- image ----------
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        except Exception:
            return

        tags = self.detector.detect(gray)
        if not tags:
            return

        # ---------- camera intrinsics ----------
        cam = self.camera_config['cameras'][0]['settings']
        intr = cam['intrinsic_params']

        K = np.array([
            [intr['fx'], 0, intr['cx']],
            [0, intr['fy'], intr['cy']],
            [0, 0, 1]
        ])
        dist = np.array(cam['dist_coeffs'])

        half = self.tag_size / 2.0
        obj_pts = np.array([
            [-half, -half, 0],
            [ half, -half, 0],
            [ half,  half, 0],
            [-half,  half, 0]
        ])

        # ---------- TF matrix ----------
        t = tf.transform.translation
        q = tf.transform.rotation

        # camera -> base rotation
        R_bc = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        # camera -> base translation
        t_bc = np.array([t.x, t.y, t.z])    


        pose_array = PoseArray()
        pose_array.header.stamp = msg.header.stamp
        pose_array.header.frame_id = 'base_link'

        for tag in tags:
            img_pts = np.asarray(tag.corners, dtype=np.float32)
            ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist)
            if not ok:
                continue

            # ---------- camera <- tag ----------
            R_ct, _ = cv2.Rodrigues(rvec)
            t_ct = tvec.reshape(3)

            # ---------- base <- tag ----------
            R_bt = R_bc @ R_ct
            t_bt = R_bc @ t_ct + t_bc

            pose = Pose()
            pose.position.x = float(t_bt[0])
            pose.position.y = float(t_bt[1])
            pose.position.z = float(t_bt[2])

            # orientation in base_link
            q_bt = R.from_matrix(R_bt).as_quat()
            pose.orientation.x = q_bt[0]
            pose.orientation.y = q_bt[1]
            pose.orientation.z = q_bt[2]
            pose.orientation.w = q_bt[3]

            pose_array.poses.append(pose)

        if pose_array.poses:
            self.pose_pub.publish(pose_array)

    # ==================================================
    @staticmethod
    def _load_json(path):
        with open(path, 'r') as f:
            return json.load(f)


def main():
    rclpy.init()
    node = AprilTagDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
