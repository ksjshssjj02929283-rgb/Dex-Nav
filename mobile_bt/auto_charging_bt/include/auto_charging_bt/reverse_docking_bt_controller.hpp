#ifndef AUTO_CHARGING_BT_REVERSE_DOCKING_BT_CONTROLLER_HPP_
#define AUTO_CHARGING_BT_REVERSE_DOCKING_BT_CONTROLLER_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

#include "reverse_docking_controller.h"

class AutoChargingNode;

class ReverseDockingBtController {
public:
    explicit ReverseDockingBtController(std::shared_ptr<AutoChargingNode> charging_node);
    ~ReverseDockingBtController();

    void start();
    void stop();
    bool done() const { return controller_done_; }

private:
    enum class Stage {
        INITIAL_SAMPLING,
        FIRST_APPROACH,
        REFINEMENT_SAMPLING,
        SECOND_APPROACH,
        FINISHED,
    };

    void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void controlLoop();
    void handleSamplingStage();
    void handleReachedTarget(const geometry_msgs::msg::Pose& target_pose_base);
    bool isSamplingStage() const;
    double currentSamplingDuration() const;
    void startSampling(double duration);
    std::pair<double, double> limitStageVelocity(double linear_x, double angular_z) const;
    double clamp(double value, double min_value, double max_value) const;
    const char* stageName() const;
    std::optional<geometry_msgs::msg::PoseStamped> lockTargetInFixedFrame(
        const geometry_msgs::msg::Pose& target_pose_base);
    std::optional<geometry_msgs::msg::Pose> getLockedTargetInBaseFrame();
    geometry_msgs::msg::Pose averageTargetSamples() const;
    void publishZero();
    void publishStaticTargetTf();
    void publishBaseTargetTf(const geometry_msgs::msg::Pose& target_pose_base);
    void logTargetError(const geometry_msgs::msg::Pose& target_pose_base, const std::string& prefix) const;
    void logTargetErrorThrottled(const geometry_msgs::msg::Pose& target_pose_base) const;
    double normalizePi(double value) const;
    void resetState();
    void configureFromParameters();

    std::shared_ptr<AutoChargingNode> charging_node_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    ReverseDockingController controller_;
    std::string base_frame_;
    std::string target_base_frame_;
    std::string static_target_frame_;
    std::string target_topic_;
    std::string fixed_frame_;
    double target_lock_duration_ = 2.0;
    double refinement_lock_duration_ = 1.0;
    double second_stage_linear_scale_ = 0.5;
    double second_stage_angular_scale_ = 0.5;
    double second_stage_max_linear_speed_ = 0.08;
    double second_stage_max_angular_speed_ = 0.20;
    double final_yaw_offset_ = 0.0;
    Stage stage_ = Stage::INITIAL_SAMPLING;
    bool sampling_started_ = false;
    bool controller_done_ = false;
    rclcpp::Time sample_start_time_;
    std::vector<geometry_msgs::msg::Pose> target_samples_;
    std::optional<geometry_msgs::msg::PoseStamped> locked_target_fixed_;
};

#endif  // AUTO_CHARGING_BT_REVERSE_DOCKING_BT_CONTROLLER_HPP_
