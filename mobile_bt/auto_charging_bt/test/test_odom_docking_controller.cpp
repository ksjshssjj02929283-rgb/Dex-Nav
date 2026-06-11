#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <functional>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "docking_controller.h"

class OdomDockingControllerTester : public rclcpp::Node {
public:
    OdomDockingControllerTester() : Node("odom_docking_controller_tester") {
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        target_base_frame_ = declare_parameter<std::string>("target_base_frame", "base_link_docking_target");
        target_topic_ = declare_parameter<std::string>("target_topic", "apriltag_target_pose");
        cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "control/cmd_vel_nav");
        direction_ = declare_parameter<int>("direction", -1);
        double control_rate = declare_parameter<double>("control_rate", 20.0);
        if (control_rate <= 0.0) {
            RCLCPP_WARN(get_logger(), "Invalid control_rate %.3f, use 20.0 Hz instead", control_rate);
            control_rate = 20.0;
        }

        controller_.setConstants(declare_parameter<double>("kP", 0.16), declare_parameter<double>("kA", 0.28),
                                 declare_parameter<double>("kB", -0.02));
        controller_.setLinearTolerance(declare_parameter<double>("linear_tolerance", 0.02));
        controller_.setAngularTolerance(declare_parameter<double>("angular_tolerance", 0.04));
        controller_.setMaxLinearSpeed(declare_parameter<double>("max_linear_speed", 0.25));
        controller_.setMaxAngularSpeed(declare_parameter<double>("max_angular_speed", 0.60));
        controller_.setMinLinearSpeed(declare_parameter<double>("min_linear_speed", 0.0));
        controller_.setMinAngularSpeed(declare_parameter<double>("min_angular_speed", 0.0075));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
        target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            target_topic_, 10, std::bind(&OdomDockingControllerTester::targetCallback, this, std::placeholders::_1));

        const auto period = std::chrono::duration<double>(1.0 / control_rate);
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                   std::bind(&OdomDockingControllerTester::controlLoop, this));

        RCLCPP_INFO(get_logger(), "Base-link docking tester started. target_topic=%s, cmd_vel_topic=%s, base_frame=%s",
                    target_topic_.c_str(), cmd_vel_topic_.c_str(), base_frame_.c_str());
    }

private:
    void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!msg->header.frame_id.empty() && msg->header.frame_id != base_frame_) {
            RCLCPP_WARN(get_logger(), "Ignore target in frame '%s'; expected '%s'", msg->header.frame_id.c_str(),
                        base_frame_.c_str());
            return;
        }

        target_pose_ = msg->pose;
        publishBaseTargetTf(target_pose_.value());
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Updated base_link target: x=%.3f, y=%.3f",
                             target_pose_->position.x, target_pose_->position.y);
    }

    void controlLoop() {
        if (!target_pose_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for target on %s", target_topic_.c_str());
            return;
        }
        publishBaseTargetTf(target_pose_.value());

        geometry_msgs::msg::Pose current_pose_base;
        current_pose_base.orientation.w = 1.0;

        if (controller_.atGoal(current_pose_base, target_pose_.value())) {
            publishZero();
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Reached base_link target");
            return;
        }

        const auto [v, w] = controller_.getVelocity(current_pose_base, target_pose_.value(), direction_);
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = v;
        cmd.angular.z = w;
        cmd_pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "base control: target_base=(%.3f, %.3f), v=%.4f, w=%.4f",
                             target_pose_->position.x, target_pose_->position.y, v, w);
    }

    void publishZero() {
        geometry_msgs::msg::Twist zero;
        cmd_pub_->publish(zero);
    }

    void publishBaseTargetTf(const geometry_msgs::msg::Pose& target_pose_base) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now();
        tf.header.frame_id = base_frame_;
        tf.child_frame_id = target_base_frame_;
        tf.transform.translation.x = target_pose_base.position.x;
        tf.transform.translation.y = target_pose_base.position.y;
        tf.transform.translation.z = target_pose_base.position.z;
        tf.transform.rotation = target_pose_base.orientation;
        tf_broadcaster_->sendTransform(tf);
    }

    DockingController controller_;
    std::string base_frame_;
    std::string target_base_frame_;
    std::string target_topic_;
    std::string cmd_vel_topic_;
    int direction_;
    std::optional<geometry_msgs::msg::Pose> target_pose_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomDockingControllerTester>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
