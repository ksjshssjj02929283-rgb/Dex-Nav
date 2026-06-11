#include "auto_charging_bt/reverse_docking_bt_controller.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>

#include "auto_charging_bt/auto_charging_bt.hpp"

ReverseDockingBtController::ReverseDockingBtController(std::shared_ptr<AutoChargingNode> charging_node)
    : charging_node_(std::move(charging_node)) {}

ReverseDockingBtController::~ReverseDockingBtController() { stop(); }

void ReverseDockingBtController::start() {
    resetState();
    configureFromParameters();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(charging_node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*charging_node_);

    target_sub_ = charging_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        target_topic_, 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { targetCallback(msg); });

    double control_rate = charging_node_->get_parameter("dock.reverse.control_rate").as_double();
    if (control_rate <= 0.0) {
        RCLCPP_WARN(charging_node_->get_logger(), "Invalid dock reverse control_rate %.3f, use 20.0 Hz instead",
                    control_rate);
        control_rate = 20.0;
    }
    const auto period = std::chrono::duration<double>(1.0 / control_rate);
    timer_ = charging_node_->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                               [this]() { controlLoop(); });

    RCLCPP_INFO(charging_node_->get_logger(),
                "Dock reverse control started. target_topic=%s, base_frame=%s, fixed_frame=%s", target_topic_.c_str(),
                base_frame_.c_str(), fixed_frame_.c_str());
}

void ReverseDockingBtController::stop() {
    if (timer_) {
        timer_->cancel();
    }
    target_sub_.reset();
    tf_broadcaster_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    publishZero();
}

void ReverseDockingBtController::configureFromParameters() {
    base_frame_ = charging_node_->get_parameter("dock.reverse.base_frame").as_string();
    target_base_frame_ = charging_node_->get_parameter("dock.reverse.target_base_frame").as_string();
    static_target_frame_ = charging_node_->get_parameter("dock.reverse.static_target_frame").as_string();
    target_topic_ = charging_node_->get_parameter("dock.reverse.target_topic").as_string();
    fixed_frame_ = charging_node_->get_parameter("dock.reverse.fixed_frame").as_string();
    target_lock_duration_ = charging_node_->get_parameter("dock.reverse.target_lock_duration").as_double();
    if (target_lock_duration_ <= 0.0) {
        RCLCPP_WARN(charging_node_->get_logger(), "Invalid target_lock_duration %.3f, use 2.0 s instead",
                    target_lock_duration_);
        target_lock_duration_ = 2.0;
    }
    refinement_lock_duration_ = charging_node_->get_parameter("dock.reverse.refinement_lock_duration").as_double();
    if (refinement_lock_duration_ <= 0.0) {
        RCLCPP_WARN(charging_node_->get_logger(), "Invalid refinement_lock_duration %.3f, use 1.0 s instead",
                    refinement_lock_duration_);
        refinement_lock_duration_ = 1.0;
    }
    second_stage_linear_scale_ = charging_node_->get_parameter("dock.reverse.second_stage_linear_scale").as_double();
    second_stage_angular_scale_ = charging_node_->get_parameter("dock.reverse.second_stage_angular_scale").as_double();
    second_stage_max_linear_speed_ =
        std::abs(charging_node_->get_parameter("dock.reverse.second_stage_max_linear_speed").as_double());
    second_stage_max_angular_speed_ =
        std::abs(charging_node_->get_parameter("dock.reverse.second_stage_max_angular_speed").as_double());

    controller_.setLinearGain(charging_node_->get_parameter("dock.reverse.linear_gain").as_double());
    controller_.setHeadingGain(charging_node_->get_parameter("dock.reverse.heading_gain").as_double());
    controller_.setFinalYawGain(charging_node_->get_parameter("dock.reverse.final_yaw_gain").as_double());
    controller_.setFinalYawIntegralGain(charging_node_->get_parameter("dock.reverse.final_yaw_i_gain").as_double());
    controller_.setFinalYawDerivativeGain(charging_node_->get_parameter("dock.reverse.final_yaw_d_gain").as_double());
    controller_.setFinalYawIntegralLimit(charging_node_->get_parameter("dock.reverse.final_yaw_i_limit").as_double());
    final_yaw_offset_ = charging_node_->get_parameter("dock.reverse.final_yaw_offset").as_double();
    controller_.setFinalYawOffset(final_yaw_offset_);
    controller_.setLinearTolerance(charging_node_->get_parameter("dock.reverse.linear_tolerance").as_double());
    controller_.setLateralTolerance(charging_node_->get_parameter("dock.reverse.lateral_tolerance").as_double());
    controller_.setHeadingTolerance(charging_node_->get_parameter("dock.reverse.heading_tolerance").as_double());
    controller_.setFinalYawTolerance(charging_node_->get_parameter("dock.reverse.final_yaw_tolerance").as_double());
    controller_.setApproachDistance(charging_node_->get_parameter("dock.reverse.approach_distance").as_double());
    controller_.setLargeHeadingThreshold(
        charging_node_->get_parameter("dock.reverse.large_heading_threshold").as_double());
    controller_.setMaxLinearSpeed(charging_node_->get_parameter("dock.reverse.max_linear_speed").as_double());
    controller_.setMinLinearSpeed(charging_node_->get_parameter("dock.reverse.min_linear_speed").as_double());
    controller_.setMaxAngularSpeed(charging_node_->get_parameter("dock.reverse.max_angular_speed").as_double());
    controller_.setMinAngularSpeed(charging_node_->get_parameter("dock.reverse.min_angular_speed").as_double());
    controller_.setFinalMaxAngularSpeed(
        charging_node_->get_parameter("dock.reverse.final_max_angular_speed").as_double());
    controller_.setFinalMinAngularSpeed(
        charging_node_->get_parameter("dock.reverse.final_min_angular_speed").as_double());
    controller_.setMaxLinearAcceleration(charging_node_->get_parameter("dock.reverse.max_linear_accel").as_double());
    controller_.setMaxAngularAcceleration(charging_node_->get_parameter("dock.reverse.max_angular_accel").as_double());
    controller_.reset();
}

void ReverseDockingBtController::targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != base_frame_) {
        RCLCPP_WARN(charging_node_->get_logger(), "Ignore dock target in frame '%s'; expected '%s'",
                    msg->header.frame_id.c_str(), base_frame_.c_str());
        return;
    }

    if (!isSamplingStage()) {
        RCLCPP_INFO_THROTTLE(charging_node_->get_logger(), *charging_node_->get_clock(), 2000,
                             "Ignore live dock target update in %s", stageName());
        return;
    }

    if (!sampling_started_) {
        startSampling(currentSamplingDuration());
    }

    target_samples_.push_back(msg->pose);
    RCLCPP_INFO_THROTTLE(charging_node_->get_logger(), *charging_node_->get_clock(), 500,
                         "Sampling dock reverse target: stage=%s count=%zu latest=(%.3f, %.3f)", stageName(),
                         target_samples_.size(), msg->pose.position.x, msg->pose.position.y);
}

void ReverseDockingBtController::controlLoop() {
    if (stage_ == Stage::FINISHED) {
        publishZero();
        publishStaticTargetTf();
        return;
    }

    if (isSamplingStage()) {
        publishZero();
        handleSamplingStage();
        return;
    }

    if (!locked_target_fixed_) {
        RCLCPP_WARN_THROTTLE(charging_node_->get_logger(), *charging_node_->get_clock(), 1000,
                             "No locked dock target in %s; restart initial sampling", stageName());
        stage_ = Stage::INITIAL_SAMPLING;
        target_samples_.clear();
        sampling_started_ = false;
        publishZero();
        return;
    }

    publishStaticTargetTf();

    auto target_pose_base = getLockedTargetInBaseFrame();
    if (!target_pose_base) {
        publishZero();
        return;
    }
    publishBaseTargetTf(target_pose_base.value());

    geometry_msgs::msg::Pose current_pose_base;
    current_pose_base.orientation.w = 1.0;

    if (controller_.atGoal(current_pose_base, target_pose_base.value())) {
        publishZero();
        handleReachedTarget(target_pose_base.value());
        return;
    }

    const auto [raw_v, raw_w] = controller_.getVelocity(current_pose_base, target_pose_base.value());
    const auto [v, w] = limitStageVelocity(raw_v, raw_w);
    charging_node_->publishTwist(v, w);

    RCLCPP_INFO_THROTTLE(charging_node_->get_logger(), *charging_node_->get_clock(), 500,
                         "Dock reverse control: stage=%s controller_stage=%s target_base=(%.3f, %.3f), raw_v=%.4f "
                         "raw_w=%.4f v=%.4f w=%.4f",
                         stageName(), controller_.stageName().c_str(), target_pose_base->position.x,
                         target_pose_base->position.y, raw_v, raw_w, v, w);
    logTargetErrorThrottled(target_pose_base.value());
}

void ReverseDockingBtController::handleSamplingStage() {
    if (!sampling_started_) {
        RCLCPP_INFO_THROTTLE(charging_node_->get_logger(), *charging_node_->get_clock(), 2000,
                             "Waiting for dock target on %s in %s", target_topic_.c_str(), stageName());
        return;
    }

    const double duration = currentSamplingDuration();
    const double elapsed = (charging_node_->now() - sample_start_time_).seconds();
    if (elapsed < duration) {
        RCLCPP_INFO_THROTTLE(charging_node_->get_logger(), *charging_node_->get_clock(), 500,
                             "Sampling dock target in %s: %.2f / %.2f s, count=%zu", stageName(), elapsed, duration,
                             target_samples_.size());
        return;
    }

    if (target_samples_.empty()) {
        RCLCPP_WARN_THROTTLE(charging_node_->get_logger(), *charging_node_->get_clock(), 1000,
                             "No dock target samples collected in %s; restart sampling", stageName());
        sampling_started_ = false;
        return;
    }

    const geometry_msgs::msg::Pose averaged_target_base = averageTargetSamples();
    auto locked_target = lockTargetInFixedFrame(averaged_target_base);
    if (!locked_target) {
        publishZero();
        return;
    }

    const auto sample_count = target_samples_.size();
    locked_target_fixed_ = locked_target.value();
    target_samples_.clear();
    sampling_started_ = false;
    controller_.reset();
    const Stage next_stage = stage_ == Stage::INITIAL_SAMPLING ? Stage::FIRST_APPROACH : Stage::SECOND_APPROACH;
    RCLCPP_INFO(charging_node_->get_logger(),
                "Locked %s dock target from %zu samples into %s: base=(%.3f, %.3f, %.3f), fixed=(%.3f, %.3f, %.3f)",
                stageName(), sample_count, fixed_frame_.c_str(), averaged_target_base.position.x,
                averaged_target_base.position.y, averaged_target_base.position.z, locked_target_fixed_->pose.position.x,
                locked_target_fixed_->pose.position.y, locked_target_fixed_->pose.position.z);
    logTargetError(averaged_target_base,
                   stage_ == Stage::INITIAL_SAMPLING ? "first_locked_base" : "second_locked_base");
    stage_ = next_stage;
}

void ReverseDockingBtController::handleReachedTarget(const geometry_msgs::msg::Pose& target_pose_base) {
    logTargetError(target_pose_base, stage_ == Stage::FIRST_APPROACH ? "first_reached_base" : "second_reached_base");
    if (stage_ == Stage::FIRST_APPROACH) {
        RCLCPP_INFO(charging_node_->get_logger(),
                    "First reverse dock target reached; stop and start refinement sampling");
        locked_target_fixed_.reset();
        target_samples_.clear();
        sampling_started_ = false;
        controller_.reset();
        stage_ = Stage::REFINEMENT_SAMPLING;
        return;
    }

    if (stage_ == Stage::SECOND_APPROACH) {
        RCLCPP_INFO(charging_node_->get_logger(), "Second reverse dock target reached; finish Dock controller");
        stage_ = Stage::FINISHED;
        controller_.reset();
        controller_done_ = true;
        return;
    }

    RCLCPP_WARN(charging_node_->get_logger(), "Reached target in unexpected reverse dock stage %s", stageName());
    stage_ = Stage::FINISHED;
    controller_done_ = true;
}

bool ReverseDockingBtController::isSamplingStage() const {
    return stage_ == Stage::INITIAL_SAMPLING || stage_ == Stage::REFINEMENT_SAMPLING;
}

double ReverseDockingBtController::currentSamplingDuration() const {
    return stage_ == Stage::REFINEMENT_SAMPLING ? refinement_lock_duration_ : target_lock_duration_;
}

void ReverseDockingBtController::startSampling(double duration) {
    sampling_started_ = true;
    sample_start_time_ = charging_node_->now();
    target_samples_.clear();
    RCLCPP_INFO(charging_node_->get_logger(), "Start sampling dock reverse target in %s for %.2f s", stageName(),
                duration);
}

std::pair<double, double> ReverseDockingBtController::limitStageVelocity(double linear_x, double angular_z) const {
    if (stage_ != Stage::SECOND_APPROACH) {
        return {linear_x, angular_z};
    }

    linear_x *= second_stage_linear_scale_;
    angular_z *= second_stage_angular_scale_;
    linear_x = clamp(linear_x, -second_stage_max_linear_speed_, second_stage_max_linear_speed_);
    angular_z = clamp(angular_z, -second_stage_max_angular_speed_, second_stage_max_angular_speed_);
    return {linear_x, angular_z};
}

double ReverseDockingBtController::clamp(double value, double min_value, double max_value) const {
    return std::max(min_value, std::min(max_value, value));
}

const char* ReverseDockingBtController::stageName() const {
    switch (stage_) {
        case Stage::INITIAL_SAMPLING:
            return "INITIAL_SAMPLING";
        case Stage::FIRST_APPROACH:
            return "FIRST_APPROACH";
        case Stage::REFINEMENT_SAMPLING:
            return "REFINEMENT_SAMPLING";
        case Stage::SECOND_APPROACH:
            return "SECOND_APPROACH";
        case Stage::FINISHED:
            return "FINISHED";
    }
    return "UNKNOWN";
}

std::optional<geometry_msgs::msg::PoseStamped> ReverseDockingBtController::lockTargetInFixedFrame(
    const geometry_msgs::msg::Pose& target_pose_base) {
    geometry_msgs::msg::PoseStamped target_base;
    target_base.header.stamp = charging_node_->now();
    target_base.header.frame_id = base_frame_;
    target_base.pose = target_pose_base;

    try {
        const auto base_to_fixed = tf_buffer_->lookupTransform(fixed_frame_, base_frame_, tf2::TimePointZero);
        geometry_msgs::msg::PoseStamped target_fixed;
        tf2::doTransform(target_base, target_fixed, base_to_fixed);
        target_fixed.header.stamp = charging_node_->now();
        target_fixed.header.frame_id = fixed_frame_;
        return target_fixed;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(charging_node_->get_logger(), *charging_node_->get_clock(), 1000,
                             "Failed to lock dock target into fixed frame '%s': %s", fixed_frame_.c_str(), ex.what());
        return std::nullopt;
    }
}

std::optional<geometry_msgs::msg::Pose> ReverseDockingBtController::getLockedTargetInBaseFrame() {
    if (!locked_target_fixed_) {
        return std::nullopt;
    }

    try {
        const auto fixed_to_base = tf_buffer_->lookupTransform(base_frame_, fixed_frame_, tf2::TimePointZero);
        geometry_msgs::msg::PoseStamped target_base;
        tf2::doTransform(locked_target_fixed_.value(), target_base, fixed_to_base);
        return target_base.pose;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(charging_node_->get_logger(), *charging_node_->get_clock(), 1000,
                             "Failed to transform locked dock target from '%s' to '%s': %s", fixed_frame_.c_str(),
                             base_frame_.c_str(), ex.what());
        return std::nullopt;
    }
}

geometry_msgs::msg::Pose ReverseDockingBtController::averageTargetSamples() const {
    geometry_msgs::msg::Pose average;
    tf2::Quaternion q_sum(0.0, 0.0, 0.0, 0.0);
    tf2::Quaternion q_ref;
    bool has_ref = false;

    for (const auto& sample : target_samples_) {
        average.position.x += sample.position.x;
        average.position.y += sample.position.y;
        average.position.z += sample.position.z;

        tf2::Quaternion q;
        tf2::fromMsg(sample.orientation, q);
        if (!has_ref) {
            q_ref = q;
            has_ref = true;
        } else if (q_ref.dot(q) < 0.0) {
            q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
        }
        q_sum = tf2::Quaternion(q_sum.x() + q.x(), q_sum.y() + q.y(), q_sum.z() + q.z(), q_sum.w() + q.w());
    }

    const double count = static_cast<double>(target_samples_.size());
    average.position.x /= count;
    average.position.y /= count;
    average.position.z /= count;

    if (q_sum.length2() > 1e-12) {
        q_sum.normalize();
        average.orientation = tf2::toMsg(q_sum);
    } else {
        average.orientation.w = 1.0;
    }

    return average;
}

void ReverseDockingBtController::publishZero() {
    if (charging_node_) {
        charging_node_->publishTwist(0.0, 0.0);
    }
}

void ReverseDockingBtController::publishStaticTargetTf() {
    if (!locked_target_fixed_ || !tf_broadcaster_) {
        return;
    }

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = charging_node_->now();
    tf.header.frame_id = fixed_frame_;
    tf.child_frame_id = static_target_frame_;
    tf.transform.translation.x = locked_target_fixed_->pose.position.x;
    tf.transform.translation.y = locked_target_fixed_->pose.position.y;
    tf.transform.translation.z = locked_target_fixed_->pose.position.z;
    tf.transform.rotation = locked_target_fixed_->pose.orientation;
    tf_broadcaster_->sendTransform(tf);
}

void ReverseDockingBtController::publishBaseTargetTf(const geometry_msgs::msg::Pose& target_pose_base) {
    if (!tf_broadcaster_) {
        return;
    }

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = charging_node_->now();
    tf.header.frame_id = base_frame_;
    tf.child_frame_id = target_base_frame_;
    tf.transform.translation.x = target_pose_base.position.x;
    tf.transform.translation.y = target_pose_base.position.y;
    tf.transform.translation.z = target_pose_base.position.z;
    tf.transform.rotation = target_pose_base.orientation;
    tf_broadcaster_->sendTransform(tf);
}

void ReverseDockingBtController::logTargetError(const geometry_msgs::msg::Pose& target_pose_base,
                                                const std::string& prefix) const {
    const double local_x = target_pose_base.position.x;
    const double local_y = target_pose_base.position.y;
    const double distance = std::hypot(local_x, local_y);
    const double reverse_heading_error = normalizePi(std::atan2(local_y, local_x) + M_PI);
    const double final_yaw_error = normalizePi(tf2::getYaw(target_pose_base.orientation) - final_yaw_offset_);
    RCLCPP_INFO(charging_node_->get_logger(),
                "%s target error: local_x=%.4f local_y=%.4f dist=%.4f reverse_heading=%.4f rad final_yaw=%.4f rad",
                prefix.c_str(), local_x, local_y, distance, reverse_heading_error, final_yaw_error);
}

void ReverseDockingBtController::logTargetErrorThrottled(const geometry_msgs::msg::Pose& target_pose_base) const {
    const double local_x = target_pose_base.position.x;
    const double local_y = target_pose_base.position.y;
    const double distance = std::hypot(local_x, local_y);
    const double reverse_heading_error = normalizePi(std::atan2(local_y, local_x) + M_PI);
    const double final_yaw_error = normalizePi(tf2::getYaw(target_pose_base.orientation) - final_yaw_offset_);
    RCLCPP_INFO_THROTTLE(
        charging_node_->get_logger(), *charging_node_->get_clock(), 500,
        "target error: local_x=%.4f local_y=%.4f dist=%.4f reverse_heading=%.4f rad final_yaw=%.4f rad", local_x,
        local_y, distance, reverse_heading_error, final_yaw_error);
}

double ReverseDockingBtController::normalizePi(double value) const {
    while (value > M_PI) value -= 2.0 * M_PI;
    while (value < -M_PI) value += 2.0 * M_PI;
    return value;
}

void ReverseDockingBtController::resetState() {
    target_samples_.clear();
    locked_target_fixed_.reset();
    sampling_started_ = false;
    controller_done_ = false;
    stage_ = Stage::INITIAL_SAMPLING;
    controller_.reset();
}
