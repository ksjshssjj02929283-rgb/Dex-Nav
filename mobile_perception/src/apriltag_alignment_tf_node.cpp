// ----------------------------------------------------------------------------
// Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------

#include <rclcpp/rclcpp.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <nlohmann/json.hpp>

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

using json = nlohmann::json;

class AprilTagAlignmentTfNode : public rclcpp::Node
{
public:
  AprilTagAlignmentTfNode()
  : Node("apriltag_alignment_tf_node")
  {
    auto pkg_path =
      ament_index_cpp::get_package_share_directory("mobile_perception");

    load_json(pkg_path + "/config/camera_config.json", camera_cfg_);
    load_json(pkg_path + "/config/tags_config.json", tag_cfg_);

    const auto & cam = camera_cfg_["cameras"][0];
    image_topic_ = declare_parameter(
      "image_topic", cam["topic_name"].get<std::string>());
    camera_link_ = declare_parameter(
      "camera_link", cam["camera_link"].get<std::string>());
    tag_size_ = declare_parameter(
      "tag_size", tag_cfg_["tag_size"].get<double>());
    pose_topic_ = declare_parameter(
      "pose_topic", std::string("apriltag_poses_debug"));
    publish_tag_tf_ = declare_parameter("publish_tag_tf", true);
    publish_aligned_tf_ = declare_parameter("publish_aligned_tf", true);
    raw_frame_prefix_ = declare_parameter(
      "raw_frame_prefix", std::string("apriltag_raw_"));
    aligned_frame_prefix_ = declare_parameter(
      "aligned_frame_prefix", std::string("apriltag_aligned_"));
    publish_target_pose_ = declare_parameter("publish_target_pose", true);
    target_pose_topic_ = declare_parameter(
      "target_pose_topic", std::string("apriltag_target_pose_odom"));
    target_frame_ = declare_parameter("target_frame", std::string("odom"));
    target_forward_offset_ = declare_parameter("target_forward_offset", 0.6);

    pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseArray>(pose_topic_, 10);
    target_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(target_pose_topic_, 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 10,
      std::bind(&AprilTagAlignmentTfNode::image_cb, this,
      std::placeholders::_1));

    tag_family_ = tag36h11_create();
    detector_ = apriltag_detector_create();
    apriltag_detector_add_family(detector_, tag_family_);

    detector_->nthreads = 1;
    detector_->quad_decimate = 1.0;
    detector_->quad_sigma = 0.0;
    detector_->refine_edges = 1;
    detector_->decode_sharpening = 0.25;

    reference_tag_quaternion_ = create_reference_tag_quaternion();
    alignment_quaternion_ = reference_tag_quaternion_.inverse();
    alignment_quaternion_.normalize();

    RCLCPP_INFO(
      get_logger(),
      "AprilTag alignment TF node started. image_topic=%s pose_topic=%s raw_prefix=%s aligned_prefix=%s target_topic=%s target_frame=%s target_forward_offset=%.3f",
      image_topic_.c_str(), pose_topic_.c_str(), raw_frame_prefix_.c_str(),
      aligned_frame_prefix_.c_str(), target_pose_topic_.c_str(),
      target_frame_.c_str(), target_forward_offset_);
  }

  ~AprilTagAlignmentTfNode()
  {
    destroy_detector();
  }

private:
  void image_cb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!detector_ || !tf_buffer_) {
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        "base_link",
        camera_link_,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF lookup failed: %s", ex.what());
      return;
    }

    cv::Mat gray;
    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Image conversion failed: %s", ex.what());
      return;
    }

    image_u8_t img_header{
      gray.cols,
      gray.rows,
      gray.cols,
      gray.data
    };

    zarray_t * detections =
      apriltag_detector_detect(detector_, &img_header);

    if (zarray_size(detections) == 0) {
      apriltag_detections_destroy(detections);
      return;
    }

    const auto & intr =
      camera_cfg_["cameras"][0]["settings"]["intrinsic_params"];

    cv::Mat K = (cv::Mat_<double>(3, 3) <<
      intr["fx"], 0, intr["cx"],
      0, intr["fy"], intr["cy"],
      0, 0, 1);

    std::vector<double> dist =
      camera_cfg_["cameras"][0]["settings"]["dist_coeffs"];
    cv::Mat dist_coeffs(dist);

    double h = tag_size_ / 2.0;
    std::vector<cv::Point3f> obj_pts = {
      {-h, -h, 0},
      { h, -h, 0},
      { h,  h, 0},
      {-h,  h, 0}
    };

    tf2::Quaternion q(
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z,
      tf.transform.rotation.w);
    tf2::Matrix3x3 R_bc(q);
    tf2::Vector3 t_bc(
      tf.transform.translation.x,
      tf.transform.translation.y,
      tf.transform.translation.z);

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header = msg->header;
    pose_array.header.frame_id = "base_link";

    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t * det;
      zarray_get(detections, i, &det);

      std::vector<cv::Point2f> img_pts;
      for (int k = 0; k < 4; k++) {
        img_pts.emplace_back(det->p[k][0], det->p[k][1]);
      }

      cv::Mat rvec, tvec;
      if (!cv::solvePnP(obj_pts, img_pts, K, dist_coeffs, rvec, tvec)) {
        continue;
      }

      cv::Mat R_ct;
      cv::Rodrigues(rvec, R_ct);

      tf2::Matrix3x3 R_ct_tf(
        R_ct.at<double>(0, 0), R_ct.at<double>(0, 1), R_ct.at<double>(0, 2),
        R_ct.at<double>(1, 0), R_ct.at<double>(1, 1), R_ct.at<double>(1, 2),
        R_ct.at<double>(2, 0), R_ct.at<double>(2, 1), R_ct.at<double>(2, 2));

      tf2::Vector3 t_ct(
        tvec.at<double>(0),
        tvec.at<double>(1),
        tvec.at<double>(2));

      tf2::Matrix3x3 R_bt = R_bc * R_ct_tf;
      tf2::Vector3 t_bt = R_bc * t_ct + t_bc;

      tf2::Quaternion q_bt;
      R_bt.getRotation(q_bt);
      q_bt.normalize();

      // AprilTag raw frame is the detected tag frame in base_link. The aligned
      // frame removes the fixed tag-frame bias so that a centered docking tag
      // behind the robot matches base_link orientation.
      tf2::Quaternion q_bt_aligned = q_bt * alignment_quaternion_;
      q_bt_aligned.normalize();

      geometry_msgs::msg::Pose pose;
      pose.position.x = t_bt.x();
      pose.position.y = t_bt.y();
      pose.position.z = t_bt.z();
      pose.orientation.x = q_bt.x();
      pose.orientation.y = q_bt.y();
      pose.orientation.z = q_bt.z();
      pose.orientation.w = q_bt.w();
      pose_array.poses.push_back(pose);

      if (publish_tag_tf_) {
        tf_broadcaster_->sendTransform(
          create_tag_transform(
            msg->header.stamp,
            raw_frame_prefix_ + std::to_string(det->id),
            t_bt, q_bt));
      }

      if (publish_aligned_tf_) {
        tf_broadcaster_->sendTransform(
          create_tag_transform(
            msg->header.stamp,
            aligned_frame_prefix_ + std::to_string(det->id),
            t_bt, q_bt_aligned));
      }

      if (publish_target_pose_) {
        publish_target_pose(msg->header.stamp, t_bt, q_bt_aligned, det->id);
      }

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "tag=%d raw_q=[%.4f %.4f %.4f %.4f] aligned_q=[%.4f %.4f %.4f %.4f] raw_yaw=%.4f aligned_yaw=%.4f",
        det->id,
        q_bt.x(), q_bt.y(), q_bt.z(), q_bt.w(),
        q_bt_aligned.x(), q_bt_aligned.y(), q_bt_aligned.z(), q_bt_aligned.w(),
        get_yaw(q_bt), get_yaw(q_bt_aligned));
    }

    apriltag_detections_destroy(detections);

    if (!pose_array.poses.empty()) {
      pose_pub_->publish(pose_array);
    }
  }

  geometry_msgs::msg::TransformStamped create_tag_transform(
    const builtin_interfaces::msg::Time & stamp,
    const std::string & child_frame_id,
    const tf2::Vector3 & translation,
    const tf2::Quaternion & rotation) const
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = "base_link";
    transform.child_frame_id = child_frame_id;
    transform.transform.translation.x = translation.x();
    transform.transform.translation.y = translation.y();
    transform.transform.translation.z = translation.z();
    transform.transform.rotation.x = rotation.x();
    transform.transform.rotation.y = rotation.y();
    transform.transform.rotation.z = rotation.z();
    transform.transform.rotation.w = rotation.w();
    return transform;
  }

  void publish_target_pose(
    const builtin_interfaces::msg::Time & stamp,
    const tf2::Vector3 & aligned_translation,
    const tf2::Quaternion & aligned_rotation,
    int tag_id)
  {
    tf2::Matrix3x3 aligned_matrix(aligned_rotation);
    tf2::Vector3 target_translation =
      aligned_translation +
      aligned_matrix * tf2::Vector3(target_forward_offset_, 0.0, 0.0);

    geometry_msgs::msg::PoseStamped target_base;
    target_base.header.stamp = stamp;
    target_base.header.frame_id = "base_link";
    target_base.pose.position.x = target_translation.x();
    target_base.pose.position.y = target_translation.y();
    target_base.pose.position.z = target_translation.z();
    target_base.pose.orientation.x = aligned_rotation.x();
    target_base.pose.orientation.y = aligned_rotation.y();
    target_base.pose.orientation.z = aligned_rotation.z();
    target_base.pose.orientation.w = aligned_rotation.w();

    try {
      geometry_msgs::msg::TransformStamped base_to_target =
        tf_buffer_->lookupTransform(
          target_frame_,
          "base_link",
          tf2::TimePointZero);

      geometry_msgs::msg::PoseStamped target_pose;
      tf2::doTransform(target_base, target_pose, base_to_target);
      target_pose.header.stamp = stamp;
      target_pose.header.frame_id = target_frame_;
      target_pose_pub_->publish(target_pose);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "tag=%d target_pose[%s]: x=%.3f y=%.3f z=%.3f offset=%.3f",
        tag_id, target_frame_.c_str(),
        target_pose.pose.position.x,
        target_pose.pose.position.y,
        target_pose.pose.position.z,
        target_forward_offset_);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Target pose transform to %s failed: %s",
        target_frame_.c_str(), ex.what());
    }
  }

  tf2::Quaternion create_reference_tag_quaternion() const
  {
    tf2::Quaternion reference;
    reference.setRPY(kHalfPi, 0.0, kHalfPi);
    reference.normalize();
    return reference;
  }

  double get_yaw(const tf2::Quaternion & quaternion) const
  {
    double roll;
    double pitch;
    double yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    return yaw;
  }

  void destroy_detector()
  {
    if (detector_) {
      apriltag_detector_destroy(detector_);
      detector_ = nullptr;
    }
    if (tag_family_) {
      tag36h11_destroy(tag_family_);
      tag_family_ = nullptr;
    }
  }

  static void load_json(const std::string & path, json & j)
  {
    std::ifstream f(path);
    f >> j;
  }

  json camera_cfg_;
  json tag_cfg_;

  static constexpr double kHalfPi = 1.5707963267948966;

  std::string image_topic_;
  std::string camera_link_;
  std::string pose_topic_;
  std::string raw_frame_prefix_;
  std::string aligned_frame_prefix_;
  std::string target_pose_topic_;
  std::string target_frame_;
  double tag_size_{0.0};
  double target_forward_offset_{0.6};
  bool publish_tag_tf_{true};
  bool publish_aligned_tf_{true};
  bool publish_target_pose_{true};

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  tf2::Quaternion reference_tag_quaternion_;
  tf2::Quaternion alignment_quaternion_;

  apriltag_detector_t * detector_{nullptr};
  apriltag_family_t * tag_family_{nullptr};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AprilTagAlignmentTfNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
