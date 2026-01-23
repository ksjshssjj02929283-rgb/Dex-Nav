#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from cv_bridge import CvBridge
from pupil_apriltags import Detector
import numpy as np
import cv2
import json
import os
import signal
import sys
from scipy.spatial.transform import Rotation as R
import ament_index_python.packages
from custom_interfaces.msg import Status


# =========================
# Utils
# =========================
def get_package_path(pkg):
    return ament_index_python.packages.get_package_share_directory(pkg)


# =========================
# Unified Detector
# =========================
class UnifiedDetector(Node):

    def __init__(self, camera_cfg, tag_cfg):
        super().__init__('unified_detector')

        # ---------- config ----------
        self.camera_config = self._load_json(camera_cfg)
        self.tag_config = self._load_json(tag_cfg)
        self.camera_link = self.camera_config['cameras'][0]['camera_link']
        self.image_topic = self.camera_config['cameras'][0]['topic_name']
        self.tag_size = self.tag_config['tag_size']

        # ---------- detector ----------
        self.detector = Detector(families=self.tag_config['tag_family'])
        self.bridge = CvBridge()

        # ---------- TF ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------- pub ----------
        self.pose_pub = self.create_publisher(PoseArray, 'apriltag_poses', 30)

        # ---------- sub ----------
        self.create_subscription(Image, self.image_topic, self.image_callback,
                                 30)

        self.create_subscription(Status, 'status', self.status_callback, 10)

        # ---------- timers ----------
        self.tf_query_timer = self.create_timer(
            0.05,  # 20Hz
            self.tf_query_callback)

        # ---------- runtime state ----------
        self.detected_tag_ids = set()
        self.latest_status = None

        self.get_logger().info('UnifiedDetector started.')

    # =========================
    # Callbacks
    # =========================
    def status_callback(self, msg):
        self.latest_status = msg
        if msg.subindex024 > 0.5:
            self.get_logger().info('Goal reached, shutting down detector.')
            self.shutdown()

    # ---------- image callback (ONLY publish TF) ----------
    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        except Exception as e:
            self.get_logger().error(f'Image convert failed: {e}')
            return

        tags = self.detector.detect(gray)

        cam = self.camera_config['cameras'][0]['settings']
        intr = cam['intrinsic_params']

        K = np.array([[intr['fx'], 0, intr['cx']], [0, intr['fy'], intr['cy']],
                      [0, 0, 1]])
        dist = np.array(cam['dist_coeffs'])

        half = self.tag_size / 2.0
        obj_pts = np.array([[-half, -half, 0], [half, -half, 0],
                            [half, half, 0], [-half, half, 0]])

        self.detected_tag_ids.clear()

        for tag in tags:
            img_pts = np.array(tag.corners, dtype=np.float32)

            ok, rvec, tvec = cv2.solvePnP(obj_pts,
                                          img_pts,
                                          K,
                                          dist,
                                          flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                continue

            self.detected_tag_ids.add(tag.tag_id)

            self.publish_tag_tf_camera(tag.tag_id, rvec, tvec, msg.header.stamp)

    # ---------- TF query callback (ONLY lookup + publish pose) ----------
    def tf_query_callback(self):
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'base_link'

        for tag_id in self.detected_tag_ids:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'base_link',
                    f'apriltag_{tag_id}',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05))

                pose = Pose()
                pose.position.x = tf.transform.translation.x
                pose.position.y = tf.transform.translation.y
                pose.position.z = tf.transform.translation.z
                pose.orientation = tf.transform.rotation

                pose_array.poses.append(pose)

            except Exception:
                continue

        if pose_array.poses:
            self.pose_pub.publish(pose_array)

    # =========================
    # TF publish
    # =========================
    def publish_tag_tf_camera(self, tag_id, rvec, tvec, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.camera_link
        t.child_frame_id = f'apriltag_{tag_id}'

        Rm, _ = cv2.Rodrigues(rvec)
        q = R.from_matrix(Rm).as_quat()

        tvec = tvec.reshape(3)

        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    # =========================
    # Utils
    # =========================
    def shutdown(self):
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    @staticmethod
    def _load_json(path):
        with open(path, 'r') as f:
            return json.load(f)


# =========================
# main
# =========================
def main():
    rclpy.init()

    pkg = get_package_path('mobile_perception')
    cam_cfg = os.path.join(pkg, 'config/camera_config.json')
    tag_cfg = os.path.join(pkg, 'config/tags_config.json')

    node = UnifiedDetector(cam_cfg, tag_cfg)

    def sigint_handler(sig, frame):
        node.get_logger().info('SIGINT, shutdown.')
        node.shutdown()

    signal.signal(signal.SIGINT, sigint_handler)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    while rclpy.ok():
        executor.spin_once(timeout_sec=0.01)


if __name__ == '__main__':
    main()
