#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  

#include <nlohmann/json.hpp>
#include <fstream>
#include <vector>
#include <numeric>
#include <sstream>

using Trigger = std_srvs::srv::Trigger;
using json = nlohmann::json;

class ChargingPoseService : public rclcpp::Node
{
public:
  ChargingPoseService()
  : Node("charging_pose_service")
  {
    save_path_ = "/home/dexforce/w1/install/mobile_perception/lib/mobile_perception/charging_pose.json";

    service_ = this->create_service<Trigger>(
      "save_charging_pose",
      std::bind(&ChargingPoseService::onService, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "save_charging_pose service ready");
  }

private:
  void onService(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    geometry_msgs::msg::PoseStamped pose;
    if (!getMeanPoseOnce(pose)) {
      response->success = false;
      response->message = "Failed to get TF(map -> base_link)";
      return;
    }

    if (!savePose(pose)) {
      response->success = false;
      response->message = "Failed to save charging pose file";
      return;
    }

    response->success = true;
    response->message = buildPoseString(pose);
  }

  bool getMeanPoseOnce(geometry_msgs::msg::PoseStamped & pose)
  {
    // 1️⃣ 临时 TF buffer + listener
    tf2_ros::Buffer tf_buffer(this->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);
    geometry_msgs::msg::TransformStamped tf;

    // 2️⃣ 给 TF 一点时间进入 buffer
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    std::vector<double> xs, ys, qzs, qws;

    for (int i = 0; i < 5; ++i) {
      bool ok = false;
      for (int retry = 0; retry < 20; ++retry) {
        try {
          tf = tf_buffer.lookupTransform(
            "map", "base_link", tf2::TimePointZero);
          ok = true;
          break;
        } catch (const tf2::TransformException & e) {
          RCLCPP_WARN(this->get_logger(),
            "TF retry %d-%d: %s", i + 1, retry + 1, e.what());
          rclcpp::sleep_for(std::chrono::milliseconds(200));
        }
      }
      if (!ok) {
        RCLCPP_ERROR(this->get_logger(), "TF lookup failed");
        return false;
      }

      xs.push_back(tf.transform.translation.x);
      ys.push_back(tf.transform.translation.y);
      qzs.push_back(tf.transform.rotation.z);
      qws.push_back(tf.transform.rotation.w);

      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    auto mean = [](const std::vector<double> & v) {
      return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    };

    pose.header = tf.header;
    pose.pose.position.x = mean(xs);
    pose.pose.position.y = mean(ys);

    pose.pose.orientation.z = mean(qzs);
    pose.pose.orientation.w = mean(qws);

    return true;
  }

  bool savePose(const geometry_msgs::msg::PoseStamped & pose)
  {
    json j;
    j["frame_id"] = pose.header.frame_id;
    j["position"] = {
      {"x", pose.pose.position.x},
      {"y", pose.pose.position.y},
      {"z", pose.pose.position.z}
    };
    j["orientation"] = {
      {"x", pose.pose.orientation.x},
      {"y", pose.pose.orientation.y},
      {"z", pose.pose.orientation.z},
      {"w", pose.pose.orientation.w}
    };

    try {
      std::ofstream ofs(save_path_);
      ofs << j.dump(2);
      ofs.close();
    } catch (...) {
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Charging pose saved to %s",
                save_path_.c_str());
    return true;
  }

    std::string buildPoseString(
    const geometry_msgs::msg::PoseStamped & pose)
  {
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double theta = tf2::getYaw(pose.pose.orientation);

    std::ostringstream ss;
    ss << x << "," << y << "," << theta;

    return ss.str();
  }


private:
  std::string save_path_;
  rclcpp::Service<Trigger>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChargingPoseService>());
  rclcpp::shutdown();
  return 0;
}
