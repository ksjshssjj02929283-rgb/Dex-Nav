#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from pupil_apriltags import Detector
import numpy as np
import cv2
import json
import os
import tf2_ros
from scipy.spatial.transform import Rotation as R
import sys
import ament_index_python.packages  
import transforms3d as t3d 
from cv2 import SOLVEPNP_ITERATIVE, projectPoints
import ament_index_python.packages  
import os
import signal
from custom_interfaces.msg import Status


def get_package_path(package_name):
    try:
        return ament_index_python.packages.get_package_share_directory(package_name)
    except ament_index_python.packages.PackageNotFoundError:
        print(f"Package '{package_name}' not found.")
        return None

class UnifiedDetector(Node):
    def __init__(self, camera_config_path, apriltags_feild_path):
        # Initialize the Node class
        super().__init__('unified_detector')
        
        # Load configurations
        self.camera_config = self.load_config(camera_config_path)
        self.apriltags_feild_config = self.load_config(apriltags_feild_path)
        self.apriltags_feild_path = apriltags_feild_path
        
        # Initialize the detector
        self.apriltag_detector = Detector(families=self.apriltags_feild_config["tag_family"])
         
        # CV Bridge for ROS2 Image messages
        self.bridge = CvBridge()
        
        # Publisher for the pose array
        self.pose_pub = self.create_publisher(PoseArray, 'apriltag_poses', 30)  
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # camera_info , topic_name, camera_link
        self.topic_name = self.camera_config['cameras'][0]["topic_name"]
        print("topic_name: ", self.topic_name)

        self.camera_link = self.camera_config['cameras'][0]["camera_link"]
        print("camera_link: ", self.camera_link)

        # Variable to store the latest status 
        self.latest_status = None

        self.status_sub = self.create_subscription(
            Status,
            'status',
            self.status_callback,
            30
        )
        self.timer = self.create_timer(0.1, self.check_status_and_shutdown)

    def check_status_and_shutdown(self):
        return 
        # if self.latest_status and self.latest_status.subindex024 > 0.5:
        #     self.get_logger().info('In detect: subindex024 is greater than 0.5, acting as goal reached.')
        #     self.shutdown_node()
    
    def shutdown_node(self):
        self.get_logger().info('Shutting down node...')
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)  # Ensure that the script exits completely
        
    def on_shutdown(self):
        self.get_logger().info('Ctrl+C pressed. Shutting down...')
    
    def status_callback(self, msg):
        self.latest_status = msg

    @staticmethod
    def load_config(path):
        with open(path) as f:
            return json.load(f)
    
    def get_camera_config(self):
        return self.camera_config
    
    def detect_by_photography(self, image_path):
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError(f"Image not loaded correctly. Check the path: {image_path}")
        tag_size = self.apriltags_feild_config["tag_size"]
        
        tags = self.apriltag_detector.detect(image, estimate_tag_pose=False, camera_params=None, tag_size=tag_size)
        
        # Print the detected tags
        for tag in tags:
            print('Detected tag by photography:', tag)
            
        return tags

    def transform_pose_to_base_frame(self, pose_matrix):
        pose_matrix = np.round(pose_matrix, 4)
        # log pose matrix
        # print("detected_matrix: ", pose_matrix) 
        
        # Lookup the transformation from camera_rgb_optical_frame to base_link, camera_color_optical_frame(buddi)
        # change to the same transname in the Buddi
        # link_name = 'camera_color_optical_frame'
        transform = self.tf_buffer.lookup_transform('base_link', self.camera_link, rclpy.time.Time())
        # print()
        # Extract the translation and rotation from the transformation
        translation = transform.transform.translation
        rotation = transform.transform.rotation
         
        # print translation, rotation
        # print("translation: ", translation)
        # print("rotation: ", rotation)

        translation_vec = np.array([translation.x, translation.y, translation.z])
        rotation_quat = np.array([rotation.x, rotation.y, rotation.z, rotation.w])

        # Print rotation quaternion
        # quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
        # print(f"Rotation (quaternion): {quaternion}")
        # # Convert quaternion to Euler angles
        # euler_zyx = R.from_quat(quaternion).as_euler('ZYX')
        # # euler_xyz = R.from_quat(quaternion).as_euler('XYZ')
        # print(f"Rotation (Euler ZYX): {euler_zyx}")
        # # print(f"Rotation (Euler XYZ): {euler_xyz}")
        
        # # Convert Euler angles to quaternion
        # rotation_quaternion11 = R.from_euler('ZYX', euler_zyx).as_quat()
        # print("rotation_quaternion11: ", rotation_quaternion11)

        # Construct the transformation matrix from the translation and rotation
        transformation_matrix = t3d.affines.compose(
            translation_vec, 
            t3d.quaternions.quat2mat(rotation_quat), 
            [1, 1, 1]
        )
        transformation_matrix = np.round(transformation_matrix, 4)
        # print("111transformation_matrix: ", transformation_matrix) 
        # print("pose_matrix: ", pose_matrix)
        
        # Apply the transformation to the input pose matrix
        transformed_matrix = np.dot(transformation_matrix, pose_matrix)
        # print("transformed_matrix:", transformed_matrix) 
        
        # Extract the transformed pose as a new Pose message
        transformed_pose = Pose()
        transformed_pose.position.x = transformed_matrix[0, 3]
        transformed_pose.position.y = transformed_matrix[1, 3]
        transformed_pose.position.z = transformed_matrix[2, 3]

        # Extract rotation as quaternion from the transformed 3x3 rotation matrix
        transformed_rot = R.from_matrix(transformed_matrix[:3, :3]).as_quat()
        transformed_pose.orientation.x = transformed_rot[0]
        transformed_pose.orientation.y = transformed_rot[1]
        transformed_pose.orientation.z = transformed_rot[2]
        transformed_pose.orientation.w = transformed_rot[3]
        
        # print pose from TF
        # print("TF xyz", transformed_pose.position.x, transformed_pose.position.y, transformed_pose.position.z)       
        # print("TF orientation", transformed_rot)        

        return transformed_pose
    
    def detect_by_camera_topics(self, topic_name):
        self.create_subscription(
            Image,
            # topic_name,
            self.topic_name,
            self.image_callback,
            30
        )
    
    def create_transformation_matrix(self, rvec, tvec):
        # Convert the rotation vector to a rotation matrix
        R, _ = cv2.Rodrigues(rvec) 
        # Create a 4x4 transformation matrix
        transformation_matrix = np.zeros((4, 4), dtype=np.float64)
        # Insert the rotation matrix into the top-left 3x3 part of the transformation matrix
        transformation_matrix[:3, :3] = R
        
        # Insert the translation vector into the first three elements of the fourth column
        transformation_matrix[:3, 3] = tvec.squeeze()
        
        # Set the bottom row to [0, 0, 0, 1] for homogeneous coordinates
        transformation_matrix[3, :] = [0, 0, 0, 1]
        
        return transformation_matrix

    def round_pose(self, pose, decimals=5):
        # Round the position values
        pose.position.x = round(pose.position.x, decimals)
        pose.position.y = round(pose.position.y, decimals)
        pose.position.z = round(pose.position.z, decimals)

        # Optionally round the orientation values
        pose.orientation.x = round(pose.orientation.x, decimals)
        pose.orientation.y = round(pose.orientation.y, decimals)
        pose.orientation.z = round(pose.orientation.z, decimals)
        pose.orientation.w = round(pose.orientation.w, decimals)

        return pose

    def image_callback(self, msg): 
        # change to buddi
        # print("here msg encoding: ", msg.encoding)
        try:
            # Convert ROS Image message to OpenCV BGR image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert the BGR image to grayscale
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # Debug: Print shape to ensure conversion
            # print("Original Image shape (BGR):", cv_image.shape)
            # print("Converted Image shape (Gray):", gray_image.shape)
            
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")
            return
        except AttributeError as e:
            if '_ARRAY_API' in str(e):
                self.get_logger().error("AttributeError: _ARRAY_API not found. This may be due to NumPy version incompatibility.")
                return
            else:
                raise e

        tag_size = self.apriltags_feild_config["tag_size"]

        # Detect AprilTags
        tags = self.apriltag_detector.detect(gray_image, estimate_tag_pose=False, camera_params=None, tag_size=tag_size)

        # Access the camera intrinsic parameters and distortion coefficients
        camera_settings = self.camera_config['cameras'][0]['settings']
        intrinsic_params = camera_settings['intrinsic_params']

        # Camera intrinsic parameters
        camera_matrix = np.array([
            [intrinsic_params['fx'], 0, intrinsic_params['cx']],
            [0, intrinsic_params['fy'], intrinsic_params['cy']],
            [0, 0, 1]
        ])
        # print("camera_matrix", camera_matrix)
        # Distortion coefficients
        dist_coeffs = np.array(camera_settings['dist_coeffs'])

        # 3D coordinates of the AprilTag corners in the tag coordinate system
        tag_half_size = tag_size / 2
        object_points = np.array([
            [-tag_half_size, -tag_half_size, 0],
            [tag_half_size, -tag_half_size, 0],
            [tag_half_size, tag_half_size, 0],
            [-tag_half_size, tag_half_size, 0]
        ])

        pose_array = PoseArray() 
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'base_link'  # Change frame ID to base_link

        try: 
            # Estimate Pose3D by solvePnP (center of the AprilTags to the camera)
            for tag in tags:
                # 2D coordinates of the detected tag corners in the image plane
                image_points = np.array([
                    [tag.corners[0][0], tag.corners[0][1]],
                    [tag.corners[1][0], tag.corners[1][1]],
                    [tag.corners[2][0], tag.corners[2][1]],
                    [tag.corners[3][0], tag.corners[3][1]]
                ])
                
                # SolvePnP to find the rotation and translation vectors 
                # Use a more stable PnP method
                success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs, flags=SOLVEPNP_ITERATIVE)

                if success: 
                    det_matrix = self.create_transformation_matrix(rvec, tvec) 
                    # print("tag in camera detect tvec : ", tvec)
                    # print("tag in camera detect rvec : ", rvec)
                    transformed_pose = self.transform_pose_to_base_frame(det_matrix) 
                    x=transformed_pose.position.x
                    y=transformed_pose.position.y
                    z=transformed_pose.position.z
                    # print('tag in Base_link x,y,z: ' , x, y, z)
                    # print(' quatanion: ' , transformed_pose.orientation)
                    
                    pose_array.poses.append(transformed_pose)
                    
                else:
                    self.get_logger().warn(f'Could not estimate pose for tag ID {tag.tag_id}')

            # Publish the pose array 
            self.pose_pub.publish(pose_array)  
            # Log published poses
            for pose in pose_array.poses:
                x = round(pose.position.x, 4)
                y = round(pose.position.y, 4)
                z = round(pose.position.z, 4)
                # self.get_logger().info(f'Published x: {x}, y: {y}, z: {z}')

            # # Check the latest status message
            # if self.latest_status and self.latest_status.subindex024 > 0.5:
            #     self.get_logger().info('in detect: subindex024 is greater than 0.5, acting as goal reached')
            #     rclpy.shutdown()
            
        except Exception as e:
            self.get_logger().error(f'Could not transform pose: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    package_path = get_package_path('b1_perception')
    camera_config_path = os.path.join(package_path + '/config/camera_config.json')
    apriltags_feild_path = os.path.join(package_path + '/config/tags_config.json')

    unified_detector = UnifiedDetector(camera_config_path, apriltags_feild_path)
    unified_detector.detect_by_camera_topics('/camera/image_raw')
    def sigint_handler(signal, frame):
        unified_detector.get_logger().info('SIGINT received, shutting down...')
        unified_detector.shutdown()

    signal.signal(signal.SIGINT, sigint_handler)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(unified_detector)

    executor.spin()

if __name__ == "__main__":
    main()

# if __name__ == "__main__":
#     rclpy.init(args=sys.argv)
    
#     # Get the path to the package
#     package_path =  get_package_path('b1_perception')
#     # if package_path:
#     #     print(f"The path to the package is: {package_path}")
#         # /home/Projects/b1_humble/install/b1_perception/share/b1_perception
    
#     camera_config_path = os.path.join(package_path+ '/config/camera_config.json')
#     apriltags_feild_path = os.path.join(package_path+ '/config/tags_config.json')
#     # print(camera_config_path)
#     # print(apriltags_feild_path)
    
#     unified_detector = UnifiedDetector(camera_config_path, apriltags_feild_path)
#     # jpg_path = os.path.join(package_path+ '/images/sample_tag.jpg')
#     # unified_detector.detect_by_photography(jpg_path)
    
#     # Subscribe to the image topic
#     unified_detector.detect_by_camera_topics('/camera/image_raw')
    
#     # Register the signal handler
#     # unified_detector.signal.signal(signal.SIGINT, lambda signum, frame: rclpy.shutdown())
    
#     rclpy.spin(unified_detector)
#     rclpy.shutdown()
