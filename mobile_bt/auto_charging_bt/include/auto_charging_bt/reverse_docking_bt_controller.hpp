#ifndef AUTO_CHARGING_BT_REVERSE_DOCKING_BT_CONTROLLER_HPP_
#define AUTO_CHARGING_BT_REVERSE_DOCKING_BT_CONTROLLER_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
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
        SECOND_SAMPLING,
        SECOND_APPROACH,
        FINAL_SAMPLING,
        FINAL_APPROACH,
        FINISHED,
    };

    struct ApproachConfig {
        double linear_tolerance = 0.020;
        double lateral_tolerance = 0.020;
        double final_yaw_tolerance = 0.020;
        double linear_scale = 1.0;
        double angular_scale = 1.0;
        double max_linear_speed = 0.0;
        double max_angular_speed = 0.0;
        bool use_stage_limit = false;
    };

    struct TargetError {
        double linear = 0.0;
        double lateral = 0.0;
        double final_yaw = 0.0;
    };

    void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void controlLoop();
    void handleSamplingStage();
    void handleReachedTarget(const geometry_msgs::msg::Pose& target_pose_base);
    bool isSamplingStage() const;
    double currentSamplingDuration() const;
    double currentSamplingMaxDistance() const;
    int currentMinValidSampleCount() const;
    void startSampling(double duration);
    bool isSampleFresh(const geometry_msgs::msg::PoseStamped& msg) const;
    bool isSampleDistanceValid(const geometry_msgs::msg::Pose& pose) const;
    bool isLockedTargetReasonable(const geometry_msgs::msg::Pose& target_pose_base) const;
    void configureControllerForStage();
    std::pair<double, double> limitStageVelocity(double linear_x, double angular_z) const;
    const ApproachConfig& currentApproachConfig() const;
    double clamp(double value, double min_value, double max_value) const;
    const char* stageName() const;
    const char* stageName(Stage stage) const;
    const char* stageDescription() const;
    const char* stageDescription(Stage stage) const;
    const char* controllerStageDescription() const;
    const char* targetErrorDescription(const std::string& prefix) const;
    void logStageBanner(Stage stage) const;
    std::optional<geometry_msgs::msg::PoseStamped> lockTargetInFixedFrame(
        const geometry_msgs::msg::Pose& target_pose_base);
    std::optional<geometry_msgs::msg::Pose> getLockedTargetInBaseFrame();
    geometry_msgs::msg::Pose averageTargetSamples() const;
    double trimmedMean(std::vector<double> values) const;
    double averageSampleYaw() const;
    TargetError calculateTargetError(const geometry_msgs::msg::Pose& target_pose_base) const;
    bool isFinalSampleConsistentWithSecondReach(const geometry_msgs::msg::Pose& target_pose_base) const;
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
    double initial_sampling_duration_ = 1.0;
    double second_sampling_duration_ = 2.0;
    double final_sampling_duration_ = 1.0;
    double final_resampling_duration_ = 2.0;
    double max_sample_age_ = 1.5;
    double initial_sampling_max_distance_ = 0.0;
    double second_sampling_max_distance_ = 0.20;
    double final_sampling_max_distance_ = 0.08;
    int min_valid_sample_count_ = 5;
    int initial_min_valid_sample_count_ = 5;
    int second_min_valid_sample_count_ = 10;
    int final_min_valid_sample_count_ = 5;
    int final_resample_min_valid_sample_count_ = 10;
    double sample_trim_ratio_ = 0.20;
    int final_resample_max_attempts_ = 3;
    double final_resample_linear_margin_ = 0.020;
    double final_resample_lateral_margin_ = 0.015;
    double final_resample_yaw_margin_ = 0.035;
    ApproachConfig first_stage_config_;
    ApproachConfig second_stage_config_;
    ApproachConfig final_stage_config_;
    double final_yaw_offset_ = 0.0;
    Stage stage_ = Stage::INITIAL_SAMPLING;
    bool sampling_started_ = false;
    bool controller_done_ = false;
    int final_sampling_retry_count_ = 0;
    bool second_immediate_recheck_done_ = false;
    bool second_stage_commanded_ = false;
    rclcpp::Time sample_start_time_;
    std::vector<geometry_msgs::msg::Pose> target_samples_;
    std::optional<geometry_msgs::msg::PoseStamped> locked_target_fixed_;
    std::optional<TargetError> second_reached_error_;
};

#endif  // AUTO_CHARGING_BT_REVERSE_DOCKING_BT_CONTROLLER_HPP_
