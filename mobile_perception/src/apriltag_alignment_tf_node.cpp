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

#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

using json = nlohmann::json;

/**
 * @brief 检测 AprilTag，并将标签位姿转换为底盘对接目标。
 *
 * 节点通过服务按需启停检测。检测启动后订阅相机图像，将标签从相机坐标系
 * 转换到 base_link，并发布经过方向校正和前向偏移后的对接目标。
 */
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
    publish_target_tf_ = declare_parameter("publish_target_tf", true);
    target_pose_topic_ = declare_parameter(
      "target_pose_topic", std::string("apriltag_target_pose"));
    target_frame_ = declare_parameter("target_frame", std::string("base_link"));
    target_frame_prefix_ = declare_parameter(
      "target_frame_prefix", std::string("apriltag_target_"));
    target_forward_offset_ = declare_parameter("target_forward_offset", 0.67);

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "start_apriltag_detection",
      std::bind(&AprilTagAlignmentTfNode::start_cb, this,
      std::placeholders::_1, std::placeholders::_2));

    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "stop_apriltag_detection",
      std::bind(&AprilTagAlignmentTfNode::stop_cb, this,
      std::placeholders::_1, std::placeholders::_2));

    reference_tag_quaternion_ = create_reference_tag_quaternion();
    alignment_quaternion_ = reference_tag_quaternion_.inverse();
    alignment_quaternion_.normalize();

    RCLCPP_INFO(
      get_logger(),
      "[Tag] ready image=%s target=%s frame=%s offset=%.3f",
      image_topic_.c_str(), target_pose_topic_.c_str(), target_frame_.c_str(),
      target_forward_offset_);
  }

  ~AprilTagAlignmentTfNode()
  {
    destroy_detector();
  }

private:
  /// 创建检测器、图像订阅和 TF 资源，开始处理相机图像。
  void start_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (detector_) {
      res->success = false;
      res->message = "AprilTag alignment detection already running";
      return;
    }

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

    reset_detection_statistics();
    statistics_timer_ = create_wall_timer(
      std::chrono::seconds(5),
      [this]() { log_detection_statistics(false); });

    res->success = true;
    res->message = "AprilTag alignment detection started";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  /// 停止检测并释放运行期资源，避免空闲时持续占用计算资源。
  void stop_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (detector_) {
      log_detection_statistics(true);
    }
    statistics_timer_.reset();
    image_sub_.reset();
    tf_broadcaster_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    pose_pub_.reset();
    target_pose_pub_.reset();
    destroy_detector();

    res->success = true;
    res->message = "AprilTag alignment detection stopped";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  /**
   * @brief 处理单帧图像并发布标签位姿。
   *
   * 处理顺序为：查询相机外参、转换灰度图、检测标签、求解位姿、
   * 转换到 base_link，最后发布调试位姿、TF 和对接目标。
   */
  void image_cb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!detector_ || !tf_buffer_ || !pose_pub_) {
      return;
    }

    ++window_image_frames_;
    ++total_image_frames_;
    const uint64_t targets_before = total_target_messages_;

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        "base_link",
        camera_link_,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "TF lookup failed: %s", ex.what());
      ++window_tf_failures_;
      ++total_tf_failures_;
      ++consecutive_frames_without_target_;
      return;
    }

    cv::Mat gray;
    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "Image conversion failed: %s", ex.what());
      ++window_conversion_failures_;
      ++total_conversion_failures_;
      ++consecutive_frames_without_target_;
      return;
    }

    cv::Scalar gray_mean;
    cv::Scalar gray_stddev;
    cv::meanStdDev(gray, gray_mean, gray_stddev);
    latest_gray_mean_ = gray_mean[0];
    latest_gray_stddev_ = gray_stddev[0];
    has_gray_statistics_ = true;

    image_u8_t img_header{
      gray.cols,
      gray.rows,
      gray.cols,
      gray.data
    };

    zarray_t * detections =
      apriltag_detector_detect(detector_, &img_header);

    if (zarray_size(detections) == 0) {
      ++consecutive_frames_without_target_;
      apriltag_detections_destroy(detections);
      return;
    }

    ++window_raw_detection_frames_;
    ++total_raw_detection_frames_;

    const auto & intr =
      camera_cfg_["cameras"][0]["settings"]["intrinsic_params"];

    cv::Mat K = (cv::Mat_<double>(3, 3) <<
      intr["fx"], 0, intr["cx"],
      0, intr["fy"], intr["cy"],
      0, 0, 1);

    std::vector<double> dist =
      camera_cfg_["cameras"][0]["settings"]["dist_coeffs"];
    cv::Mat dist_coeffs(dist);

    const float h = static_cast<float>(tag_size_ / 2.0);
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

      // raw 坐标系保留标签原始方向；aligned 坐标系消除标签安装方向带来的固定偏差，
      // 使机器人正对后方充电桩时，标签方向与 base_link 的方向定义保持一致。
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

      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 10000,
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

    if (total_target_messages_ > targets_before) {
      ++window_target_frames_;
      ++total_target_frames_;
      consecutive_frames_without_target_ = 0;
    } else {
      ++consecutive_frames_without_target_;
    }
  }

  /// 生成以 base_link 为父坐标系的标签 TF。
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

  /**
   * @brief 计算并发布最终对接目标。
   *
   * 目标以校正后的标签位姿为基准，沿标签 X 轴增加前向偏移，再按配置转换到
   * target_frame。该目标是回充控制器实际使用的输入，不是原始标签位姿。
   */
  void publish_target_pose(
    const builtin_interfaces::msg::Time & stamp,
    const tf2::Vector3 & aligned_translation,
    const tf2::Quaternion & aligned_rotation,
    int tag_id)
  {
    if (!target_pose_pub_ || !tf_buffer_) {
      return;
    }

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

    geometry_msgs::msg::PoseStamped target_pose;
    if (target_frame_ == "base_link") {
      target_pose = target_base;
    } else {
      geometry_msgs::msg::TransformStamped base_to_target;
      try {
        base_to_target = tf_buffer_->lookupTransform(
          target_frame_,
          "base_link",
          tf2::timeFromSec(
            static_cast<double>(stamp.sec) +
            static_cast<double>(stamp.nanosec) * 1e-9));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "Target pose transform at image stamp to %s failed, fallback to latest: %s",
          target_frame_.c_str(), ex.what());

        try {
          base_to_target = tf_buffer_->lookupTransform(
            target_frame_,
            "base_link",
            tf2::TimePointZero);
        } catch (const tf2::TransformException & latest_ex) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 10000,
            "Target pose transform to %s failed: %s",
            target_frame_.c_str(), latest_ex.what());
          return;
        }
      }

      tf2::doTransform(target_base, target_pose, base_to_target);
      target_pose.header.stamp = stamp;
      target_pose.header.frame_id = target_frame_;
    }

    target_pose_pub_->publish(target_pose);
    ++window_target_messages_;
    ++total_target_messages_;
    last_target_publish_time_ = std::chrono::steady_clock::now();
    has_target_publish_time_ = true;
    if (publish_target_tf_) {
      if (!tf_broadcaster_) {
        return;
      }
      tf_broadcaster_->sendTransform(
        create_target_transform(target_pose, tag_id));
    }

    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "tag=%d target_pose[%s]: x=%.3f y=%.3f z=%.3f offset=%.3f",
      tag_id, target_frame_.c_str(),
      target_pose.pose.position.x,
      target_pose.pose.position.y,
      target_pose.pose.position.z,
      target_forward_offset_);
  }

  /// 生成与目标位姿一致的可视化 TF。
  geometry_msgs::msg::TransformStamped create_target_transform(
    const geometry_msgs::msg::PoseStamped & target_pose,
    int tag_id) const
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = target_pose.header;
    transform.child_frame_id = target_frame_prefix_ + std::to_string(tag_id);
    transform.transform.translation.x = target_pose.pose.position.x;
    transform.transform.translation.y = target_pose.pose.position.y;
    transform.transform.translation.z = target_pose.pose.position.z;
    transform.transform.rotation = target_pose.pose.orientation;
    return transform;
  }

  /// 构造标签标准安装方向，用于消除检测坐标系的固定方向偏差。
  tf2::Quaternion create_reference_tag_quaternion() const
  {
    tf2::Quaternion reference;
    reference.setRPY(kHalfPi, 0.0, kHalfPi);
    reference.normalize();
    return reference;
  }

  /// 从四元数中提取偏航角，仅用于调试日志。
  double get_yaw(const tf2::Quaternion & quaternion) const
  {
    double roll;
    double pitch;
    double yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    return yaw;
  }

  /// 新一轮检测开始时清空窗口统计和累计统计。
  void reset_detection_statistics()
  {
    statistics_started_time_ = std::chrono::steady_clock::now();
    last_statistics_time_ = statistics_started_time_;
    last_target_publish_time_ = statistics_started_time_;
    window_image_frames_ = 0;
    window_raw_detection_frames_ = 0;
    window_target_frames_ = 0;
    window_target_messages_ = 0;
    window_tf_failures_ = 0;
    window_conversion_failures_ = 0;
    total_image_frames_ = 0;
    total_raw_detection_frames_ = 0;
    total_target_frames_ = 0;
    total_target_messages_ = 0;
    total_tf_failures_ = 0;
    total_conversion_failures_ = 0;
    consecutive_frames_without_target_ = 0;
    latest_gray_mean_ = 0.0;
    latest_gray_stddev_ = 0.0;
    has_gray_statistics_ = false;
    has_target_publish_time_ = false;
  }

  /**
   * @brief 输出检测统计。
   * @param final_summary 为 true 时输出本轮累计值，否则输出最近 5 秒窗口值。
   */
  void log_detection_statistics(bool final_summary)
  {
    const auto now = std::chrono::steady_clock::now();
    const double window_seconds = std::max(
      1e-6, std::chrono::duration<double>(now - last_statistics_time_).count());
    const double total_seconds = std::max(
      1e-6, std::chrono::duration<double>(now - statistics_started_time_).count());
    const double statistics_seconds = final_summary ? total_seconds : window_seconds;
    const uint64_t image_frames = final_summary ? total_image_frames_ : window_image_frames_;
    const uint64_t raw_detection_frames = final_summary ?
      total_raw_detection_frames_ : window_raw_detection_frames_;
    const uint64_t target_frames = final_summary ? total_target_frames_ : window_target_frames_;
    const uint64_t target_messages = final_summary ?
      total_target_messages_ : window_target_messages_;
    const uint64_t tf_failures = final_summary ? total_tf_failures_ : window_tf_failures_;
    const uint64_t conversion_failures = final_summary ?
      total_conversion_failures_ : window_conversion_failures_;
    const double image_rate = static_cast<double>(image_frames) / statistics_seconds;
    const double raw_detection_rate = image_frames > 0 ?
      100.0 * static_cast<double>(raw_detection_frames) / static_cast<double>(image_frames) : 0.0;
    const double target_frame_rate = image_frames > 0 ?
      100.0 * static_cast<double>(target_frames) / static_cast<double>(image_frames) : 0.0;
    const double last_target_age = has_target_publish_time_ ?
      std::chrono::duration<double>(now - last_target_publish_time_).count() : -1.0;
    char last_target_description[32];
    if (has_target_publish_time_) {
      std::snprintf(
        last_target_description, sizeof(last_target_description), "%.2fs", last_target_age);
    } else {
      std::snprintf(
        last_target_description, sizeof(last_target_description), "%s", "尚未发布");
    }
    const double gray_mean = has_gray_statistics_ ? latest_gray_mean_ : -1.0;
    const double gray_stddev = has_gray_statistics_ ? latest_gray_stddev_ : -1.0;
    const char * label = final_summary ? "TagSummary" : "TagStats";

    if (image_frames == 0) {
      RCLCPP_WARN(
        get_logger(),
        "[%s] %.1fs 无图像 topic=%s last=%s",
        label, statistics_seconds, image_topic_.c_str(), last_target_description);
    } else if (target_messages == 0) {
      RCLCPP_WARN(
        get_logger(),
        "[%s] %.1fs image=%llu(%.1fHz) detect=%llu(%.0f%%) target=0 "
        "lost=%llu last=%s gray=%.0f/%.0f tf_err=%llu cv_err=%llu",
        label, statistics_seconds, static_cast<unsigned long long>(image_frames),
        image_rate, static_cast<unsigned long long>(raw_detection_frames), raw_detection_rate,
        static_cast<unsigned long long>(consecutive_frames_without_target_), last_target_description,
        gray_mean, gray_stddev, static_cast<unsigned long long>(tf_failures),
        static_cast<unsigned long long>(conversion_failures));
    } else {
      RCLCPP_INFO(
        get_logger(),
        "[%s] %.1fs image=%llu(%.1fHz) detect=%llu(%.0f%%) target=%llu(%.0f%%) "
        "lost=%llu last=%s gray=%.0f/%.0f tf_err=%llu cv_err=%llu",
        label, statistics_seconds, static_cast<unsigned long long>(image_frames),
        image_rate, static_cast<unsigned long long>(raw_detection_frames), raw_detection_rate,
        static_cast<unsigned long long>(target_messages),
        target_frame_rate, static_cast<unsigned long long>(consecutive_frames_without_target_),
        last_target_description, gray_mean, gray_stddev,
        static_cast<unsigned long long>(tf_failures),
        static_cast<unsigned long long>(conversion_failures));
    }

    window_image_frames_ = 0;
    window_raw_detection_frames_ = 0;
    window_target_frames_ = 0;
    window_target_messages_ = 0;
    window_tf_failures_ = 0;
    window_conversion_failures_ = 0;
    last_statistics_time_ = now;
  }

  /// 释放 AprilTag C 库创建的检测器和标签族对象。
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

  /// 读取相机或标签 JSON 配置。
  static void load_json(const std::string & path, json & j)
  {
    std::ifstream f(path);
    f >> j;
  }

  json camera_cfg_;  ///< 相机话题、坐标系、内参和畸变参数。
  json tag_cfg_;  ///< 标签尺寸和检测相关配置。

  static constexpr double kHalfPi = 1.5707963267948966;

  // 节点配置。话题、坐标系和偏移量在构造阶段确定，检测启停不会改变这些值。
  std::string image_topic_;
  std::string camera_link_;
  std::string pose_topic_;
  std::string raw_frame_prefix_;
  std::string aligned_frame_prefix_;
  std::string target_pose_topic_;
  std::string target_frame_;
  std::string target_frame_prefix_;
  double tag_size_{0.0};
  double target_forward_offset_{0.6};
  bool publish_tag_tf_{true};
  bool publish_aligned_tf_{true};
  bool publish_target_pose_{true};
  bool publish_target_tf_{true};

  // 检测启动后创建的 ROS 接口，停止检测时主动释放。
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr statistics_timer_;

  // 服务始终存在，用于控制检测资源的创建和销毁。
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;

  // TF 资源仅在检测运行期间有效。
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 方向校正量：reference 表示标准安装方向，alignment 是它的逆旋转。
  tf2::Quaternion reference_tag_quaternion_;
  tf2::Quaternion alignment_quaternion_;

  // AprilTag C 库对象。空指针表示当前未启动检测。
  apriltag_detector_t * detector_{nullptr};
  apriltag_family_t * tag_family_{nullptr};

  // 统计时间点使用 steady_clock，避免系统时间校准导致统计周期跳变。
  std::chrono::steady_clock::time_point statistics_started_time_;
  std::chrono::steady_clock::time_point last_statistics_time_;
  std::chrono::steady_clock::time_point last_target_publish_time_;
  // window_* 只统计最近 5 秒，输出一次后清零。
  uint64_t window_image_frames_{0};
  uint64_t window_raw_detection_frames_{0};
  // target_frames 统计成功发布目标的图像帧，target_messages 统计实际发布消息数。
  uint64_t window_target_frames_{0};
  uint64_t window_target_messages_{0};
  uint64_t window_tf_failures_{0};
  uint64_t window_conversion_failures_{0};
  // total_* 统计一次 start 到 stop 的完整检测周期。
  uint64_t total_image_frames_{0};
  uint64_t total_raw_detection_frames_{0};
  // 同一图像可能检测到多个标签，因此目标消息数可能大于目标图像帧数。
  uint64_t total_target_frames_{0};
  uint64_t total_target_messages_{0};
  uint64_t total_tf_failures_{0};
  uint64_t total_conversion_failures_{0};
  // 连续丢失和最近帧信息用于快速判断识别是否间歇中断或画面过暗。
  uint64_t consecutive_frames_without_target_{0};
  double latest_gray_mean_{0.0};
  double latest_gray_stddev_{0.0};
  bool has_gray_statistics_{false};
  bool has_target_publish_time_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AprilTagAlignmentTfNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
