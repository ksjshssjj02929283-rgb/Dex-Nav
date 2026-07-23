#include <rclcpp/rclcpp.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <nlohmann/json.hpp>

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

#include <std_srvs/srv/trigger.hpp>


#include <fstream>
#include <memory>
#include <string>

using json = nlohmann::json;

class AprilTagDetectionNode : public rclcpp::Node
{
public:
  AprilTagDetectionNode()
  : Node("apriltag_detection_node")
  {
    // ---------- load config ----------
    auto pkg_path =
      ament_index_cpp::get_package_share_directory("mobile_perception");

    load_json(pkg_path + "/config/camera_config.json", camera_cfg_);
    load_json(pkg_path + "/config/tags_config.json", tag_cfg_);

    const auto & cam = camera_cfg_["cameras"][0];
    image_topic_ = cam["topic_name"].get<std::string>();
    camera_link_ = cam["camera_link"].get<std::string>();
    tag_size_ = tag_cfg_["tag_size"].get<double>();

    // ---------- publisher ----------
    pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseArray>("apriltag_poses", 10);

    // ---------- services ----------
    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "start_apriltag_detection",
      std::bind(&AprilTagDetectionNode::start_cb, this,
                std::placeholders::_1, std::placeholders::_2));

    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "stop_apriltag_detection",
      std::bind(&AprilTagDetectionNode::stop_cb, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "AprilTag Detection Node ready (C API, low CPU)");
  }

  ~AprilTagDetectionNode()
  {
    destroy_detector();
  }

private:
  // ==================================================
  // Services
  // ==================================================

  void start_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (detector_) {
      res->success = false;
      res->message = "Detection already running";
      return;
    }

    // TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Image subscription
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 10,
      std::bind(&AprilTagDetectionNode::image_cb, this,
                std::placeholders::_1));

    // -------- create apriltag detector --------
    tag_family_ = tag36h11_create();
    detector_ = apriltag_detector_create();
    apriltag_detector_add_family(detector_, tag_family_);

    // CPU-friendly params
    detector_->nthreads = 1;
    detector_->quad_decimate = 1.0;
    detector_->quad_sigma = 0.0;
    detector_->refine_edges = 1;
    detector_->decode_sharpening = 0.25;

    res->success = true;
    res->message = "AprilTag detection started";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  void stop_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (image_sub_) {
      image_sub_.reset();
    }

    tf_broadcaster_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();

    destroy_detector();

    res->success = true;
    res->message = "AprilTag detection stopped";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  // ==================================================
  // Image callback
  // ==================================================

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
    } catch (...) {
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
        R_ct.at<double>(0,0), R_ct.at<double>(0,1), R_ct.at<double>(0,2),
        R_ct.at<double>(1,0), R_ct.at<double>(1,1), R_ct.at<double>(1,2),
        R_ct.at<double>(2,0), R_ct.at<double>(2,1), R_ct.at<double>(2,2));

      tf2::Vector3 t_ct(
        tvec.at<double>(0),
        tvec.at<double>(1),
        tvec.at<double>(2));

      tf2::Matrix3x3 R_bt = R_bc * R_ct_tf;
      tf2::Vector3 t_bt = R_bc * t_ct + t_bc;

      geometry_msgs::msg::Pose pose;
      pose.position.x = t_bt.x();
      pose.position.y = t_bt.y();
      pose.position.z = t_bt.z();

      tf2::Quaternion q_bt;
      R_bt.getRotation(q_bt);

      pose.orientation.x = q_bt.x();
      pose.orientation.y = q_bt.y();
      pose.orientation.z = q_bt.z();
      pose.orientation.w = q_bt.w();

      pose_array.poses.push_back(pose);

      publish_debug_tf(msg->header.stamp, det->id, pose);
    }

    apriltag_detections_destroy(detections);

    if (!pose_array.poses.empty()) {
      pose_pub_->publish(pose_array);
    }
  }

  // ==================================================

  void publish_debug_tf(
    const builtin_interfaces::msg::Time & stamp,
    int tag_id,
    const geometry_msgs::msg::Pose & pose)
  {
    if (!tf_broadcaster_) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = "base_link";
    transform.child_frame_id =
      "apriltag_detection_debug_" + std::to_string(tag_id);
    transform.transform.translation.x = pose.position.x;
    transform.transform.translation.y = pose.position.y;
    transform.transform.translation.z = pose.position.z;
    transform.transform.rotation = pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  // ==================================================

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

  // ---------- members ----------
  json camera_cfg_;
  json tag_cfg_;

  std::string image_topic_;
  std::string camera_link_;
  double tag_size_{0.0};

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  apriltag_detector_t * detector_{nullptr};
  apriltag_family_t * tag_family_{nullptr};
};

// ==================================================

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AprilTagDetectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
