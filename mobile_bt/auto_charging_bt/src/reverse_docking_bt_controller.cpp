#include "auto_charging_bt/reverse_docking_bt_controller.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
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
        target_topic_, 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { targetCallback(msg); });

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
                "Dock reverse control started. target_topic=%s, base_frame=%s, fixed_frame=%s",
                target_topic_.c_str(), base_frame_.c_str(), fixed_frame_.c_str());
    logStageBanner(stage_);
    RCLCPP_INFO(charging_node_->get_logger(),
                "倒车对接控制已启动：订阅目标话题=%s，机器人坐标系=%s，"
                "锁定目标固定坐标系=%s，当前阶段=%s（%s）",
                target_topic_.c_str(), base_frame_.c_str(), fixed_frame_.c_str(),
                stageName(), stageDescription());
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

    initial_sampling_duration_ =
        charging_node_->get_parameter("dock.reverse.initial_sampling_duration").as_double();
    if (initial_sampling_duration_ <= 0.0) {
        RCLCPP_WARN(charging_node_->get_logger(),
                    "Invalid initial_sampling_duration %.3f, use 1.0 s instead",
                    initial_sampling_duration_);
        initial_sampling_duration_ = 1.0;
    }

    second_sampling_duration_ =
        charging_node_->get_parameter("dock.reverse.second_sampling_duration").as_double();
    if (second_sampling_duration_ <= 0.0) {
        RCLCPP_WARN(charging_node_->get_logger(),
                    "Invalid second_sampling_duration %.3f, use 2.0 s instead",
                    second_sampling_duration_);
        second_sampling_duration_ = 2.0;
    }

    final_sampling_duration_ =
        charging_node_->get_parameter("dock.reverse.final_sampling_duration").as_double();
    if (final_sampling_duration_ <= 0.0) {
        RCLCPP_WARN(charging_node_->get_logger(),
                    "Invalid final_sampling_duration %.3f, use 1.0 s instead",
                    final_sampling_duration_);
        final_sampling_duration_ = 1.0;
    }
    final_resampling_duration_ =
        charging_node_->get_parameter("dock.reverse.final_resampling_duration").as_double();
    if (final_resampling_duration_ <= 0.0) {
        RCLCPP_WARN(charging_node_->get_logger(),
                    "Invalid final_resampling_duration %.3f, use 2.0 s instead",
                    final_resampling_duration_);
        final_resampling_duration_ = 2.0;
    }

    max_sample_age_ = charging_node_->get_parameter("dock.reverse.max_sample_age").as_double();
    initial_sampling_max_distance_ =
        std::abs(charging_node_->get_parameter("dock.reverse.initial_sampling_max_distance").as_double());
    second_sampling_max_distance_ =
        std::abs(charging_node_->get_parameter("dock.reverse.second_sampling_max_distance").as_double());
    final_sampling_max_distance_ =
        std::abs(charging_node_->get_parameter("dock.reverse.final_sampling_max_distance").as_double());
    min_valid_sample_count_ = charging_node_->get_parameter("dock.reverse.min_valid_sample_count").as_int();
    if (min_valid_sample_count_ < 1) {
        RCLCPP_WARN(charging_node_->get_logger(),
                    "Invalid min_valid_sample_count %d, use 1 instead", min_valid_sample_count_);
        min_valid_sample_count_ = 1;
    }
    initial_min_valid_sample_count_ =
        charging_node_->get_parameter("dock.reverse.initial_min_valid_sample_count").as_int();
    if (initial_min_valid_sample_count_ < 1) {
        RCLCPP_WARN(charging_node_->get_logger(),
                    "Invalid initial_min_valid_sample_count %d, use %d instead",
                    initial_min_valid_sample_count_, min_valid_sample_count_);
        initial_min_valid_sample_count_ = min_valid_sample_count_;
    }
    second_min_valid_sample_count_ =
        charging_node_->get_parameter("dock.reverse.second_min_valid_sample_count").as_int();
    if (second_min_valid_sample_count_ < 1) {
        RCLCPP_WARN(charging_node_->get_logger(),
                    "Invalid second_min_valid_sample_count %d, use %d instead",
                    second_min_valid_sample_count_, min_valid_sample_count_);
        second_min_valid_sample_count_ = min_valid_sample_count_;
    }
    final_min_valid_sample_count_ =
        charging_node_->get_parameter("dock.reverse.final_min_valid_sample_count").as_int();
    if (final_min_valid_sample_count_ < 1) {
        RCLCPP_WARN(charging_node_->get_logger(),
                    "Invalid final_min_valid_sample_count %d, use %d instead",
                    final_min_valid_sample_count_, min_valid_sample_count_);
        final_min_valid_sample_count_ = min_valid_sample_count_;
    }
    final_resample_min_valid_sample_count_ =
        charging_node_->get_parameter("dock.reverse.final_resample_min_valid_sample_count").as_int();
    if (final_resample_min_valid_sample_count_ < 1) {
        RCLCPP_WARN(charging_node_->get_logger(),
                    "Invalid final_resample_min_valid_sample_count %d, use %d instead",
                    final_resample_min_valid_sample_count_, final_min_valid_sample_count_);
        final_resample_min_valid_sample_count_ = final_min_valid_sample_count_;
    }
    sample_trim_ratio_ = charging_node_->get_parameter("dock.reverse.sample_trim_ratio").as_double();
    if (sample_trim_ratio_ < 0.0 || sample_trim_ratio_ >= 0.5) {
        RCLCPP_WARN(charging_node_->get_logger(),
                    "Invalid sample_trim_ratio %.3f, use 0.20 instead", sample_trim_ratio_);
        sample_trim_ratio_ = 0.20;
    }
    final_resample_max_attempts_ =
        charging_node_->get_parameter("dock.reverse.final_resample_max_attempts").as_int();
    if (final_resample_max_attempts_ < 0) {
        RCLCPP_WARN(charging_node_->get_logger(),
                    "Invalid final_resample_max_attempts %d, use 3 instead", final_resample_max_attempts_);
        final_resample_max_attempts_ = 3;
    }
    final_resample_linear_margin_ =
        std::abs(charging_node_->get_parameter("dock.reverse.final_resample_linear_margin").as_double());
    final_resample_lateral_margin_ =
        std::abs(charging_node_->get_parameter("dock.reverse.final_resample_lateral_margin").as_double());
    final_resample_yaw_margin_ =
        std::abs(charging_node_->get_parameter("dock.reverse.final_resample_yaw_margin").as_double());

    first_stage_config_.linear_tolerance =
        charging_node_->get_parameter("dock.reverse.first_stage_linear_tolerance").as_double();
    first_stage_config_.lateral_tolerance =
        charging_node_->get_parameter("dock.reverse.first_stage_lateral_tolerance").as_double();
    first_stage_config_.final_yaw_tolerance =
        charging_node_->get_parameter("dock.reverse.first_stage_final_yaw_tolerance").as_double();
    first_stage_config_.use_stage_limit = false;

    second_stage_config_.linear_tolerance =
        charging_node_->get_parameter("dock.reverse.second_stage_linear_tolerance").as_double();
    second_stage_config_.lateral_tolerance =
        charging_node_->get_parameter("dock.reverse.second_stage_lateral_tolerance").as_double();
    second_stage_config_.final_yaw_tolerance =
        charging_node_->get_parameter("dock.reverse.second_stage_final_yaw_tolerance").as_double();
    second_stage_config_.linear_scale =
        charging_node_->get_parameter("dock.reverse.second_stage_linear_scale").as_double();
    second_stage_config_.angular_scale =
        charging_node_->get_parameter("dock.reverse.second_stage_angular_scale").as_double();
    second_stage_config_.max_linear_speed =
        std::abs(charging_node_->get_parameter("dock.reverse.second_stage_max_linear_speed").as_double());
    second_stage_config_.max_angular_speed =
        std::abs(charging_node_->get_parameter("dock.reverse.second_stage_max_angular_speed").as_double());
    second_stage_config_.use_stage_limit = true;

    final_stage_config_.linear_tolerance =
        charging_node_->get_parameter("dock.reverse.final_stage_linear_tolerance").as_double();
    final_stage_config_.lateral_tolerance =
        charging_node_->get_parameter("dock.reverse.final_stage_lateral_tolerance").as_double();
    final_stage_config_.final_yaw_tolerance =
        charging_node_->get_parameter("dock.reverse.final_stage_final_yaw_tolerance").as_double();
    final_stage_config_.linear_scale =
        charging_node_->get_parameter("dock.reverse.final_stage_linear_scale").as_double();
    final_stage_config_.angular_scale =
        charging_node_->get_parameter("dock.reverse.final_stage_angular_scale").as_double();
    final_stage_config_.max_linear_speed =
        std::abs(charging_node_->get_parameter("dock.reverse.final_stage_max_linear_speed").as_double());
    final_stage_config_.max_angular_speed =
        std::abs(charging_node_->get_parameter("dock.reverse.final_stage_max_angular_speed").as_double());
    final_stage_config_.use_stage_limit = true;

    controller_.setLinearGain(charging_node_->get_parameter("dock.reverse.linear_gain").as_double());
    controller_.setHeadingGain(charging_node_->get_parameter("dock.reverse.heading_gain").as_double());
    controller_.setFinalYawGain(charging_node_->get_parameter("dock.reverse.final_yaw_gain").as_double());
    controller_.setFinalYawIntegralGain(charging_node_->get_parameter("dock.reverse.final_yaw_i_gain").as_double());
    controller_.setFinalYawDerivativeGain(charging_node_->get_parameter("dock.reverse.final_yaw_d_gain").as_double());
    controller_.setFinalYawIntegralLimit(charging_node_->get_parameter("dock.reverse.final_yaw_i_limit").as_double());
    final_yaw_offset_ = charging_node_->get_parameter("dock.reverse.final_yaw_offset").as_double();
    controller_.setFinalYawOffset(final_yaw_offset_);
    configureControllerForStage();
    controller_.setHeadingTolerance(charging_node_->get_parameter("dock.reverse.heading_tolerance").as_double());
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

    if (!isSampleFresh(*msg)) {
        return;
    }

    if (!isSampleDistanceValid(msg->pose)) {
        return;
    }

    target_samples_.push_back(msg->pose);
    RCLCPP_INFO_THROTTLE(charging_node_->get_logger(), *charging_node_->get_clock(), 500,
                         "Sampling dock reverse target: stage=%s count=%zu latest=(%.3f, %.3f)",
                         stageName(), target_samples_.size(), msg->pose.position.x, msg->pose.position.y);
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
        logStageBanner(Stage::INITIAL_SAMPLING);
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

    configureControllerForStage();
    if (controller_.atGoal(current_pose_base, target_pose_base.value())) {
        publishZero();
        if (stage_ == Stage::SECOND_APPROACH && !second_stage_commanded_ && !second_immediate_recheck_done_) {
            second_immediate_recheck_done_ = true;
            RCLCPP_INFO(charging_node_->get_logger(),
                        "Second reverse dock target is already within tolerance before moving; resample once for %.2f s",
                        second_sampling_duration_);
            RCLCPP_INFO(charging_node_->get_logger(),
                        "第二阶段刚开始就已满足到位阈值：先不直接结束，重新采样 %.2f 秒确认目标稳定",
                        second_sampling_duration_);
            locked_target_fixed_.reset();
            target_samples_.clear();
            sampling_started_ = false;
            controller_.reset();
            logStageBanner(Stage::SECOND_SAMPLING);
            stage_ = Stage::SECOND_SAMPLING;
            return;
        }
        handleReachedTarget(target_pose_base.value());
        return;
    }

    const auto [raw_v, raw_w] = controller_.getVelocity(current_pose_base, target_pose_base.value());
    const auto [v, w] = limitStageVelocity(raw_v, raw_w);
    if (stage_ == Stage::SECOND_APPROACH) {
        second_stage_commanded_ = true;
    }
    charging_node_->publishTwist(v, w);

    RCLCPP_INFO_THROTTLE(charging_node_->get_logger(), *charging_node_->get_clock(), 500,
                         "Dock reverse control: stage=%s controller_stage=%s target_base=(%.3f, %.3f), raw_v=%.4f raw_w=%.4f v=%.4f w=%.4f",
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
                             "Sampling dock target in %s: %.2f / %.2f s, count=%zu",
                             stageName(), elapsed, duration, target_samples_.size());
        return;
    }

    const int min_valid_sample_count = currentMinValidSampleCount();
    if (target_samples_.size() < static_cast<size_t>(min_valid_sample_count)) {
        RCLCPP_INFO_THROTTLE(charging_node_->get_logger(), *charging_node_->get_clock(), 1000,
                             "Only %zu valid dock target samples collected in %s, need at least %d; keep sampling",
                             target_samples_.size(), stageName(), min_valid_sample_count);
        return;
    }

    const geometry_msgs::msg::Pose averaged_target_base = averageTargetSamples();
    if (!isLockedTargetReasonable(averaged_target_base)) {
        RCLCPP_WARN(charging_node_->get_logger(),
                    "Reject averaged dock target in %s: base=(%.3f, %.3f), dist=%.3f exceeds max %.3f; restart sampling",
                    stageName(), averaged_target_base.position.x, averaged_target_base.position.y,
                    std::hypot(averaged_target_base.position.x, averaged_target_base.position.y),
                    currentSamplingMaxDistance());
        target_samples_.clear();
        sampling_started_ = false;
        publishZero();
        return;
    }

    if (stage_ == Stage::FINAL_SAMPLING && !isFinalSampleConsistentWithSecondReach(averaged_target_base)) {
        ++final_sampling_retry_count_;
        target_samples_.clear();
        sampling_started_ = false;
        publishZero();
        return;
    }

    auto locked_target = lockTargetInFixedFrame(averaged_target_base);
    if (!locked_target) {
        publishZero();
        return;
    }

    const Stage sampling_stage = stage_;
    const auto sample_count = target_samples_.size();
    locked_target_fixed_ = locked_target.value();
    target_samples_.clear();
    sampling_started_ = false;
    controller_.reset();
    Stage next_stage = Stage::FIRST_APPROACH;
    std::string error_prefix = "first_locked_base";
    if (sampling_stage == Stage::SECOND_SAMPLING) {
        next_stage = Stage::SECOND_APPROACH;
        error_prefix = "second_locked_base";
        second_stage_commanded_ = false;
    } else if (sampling_stage == Stage::FINAL_SAMPLING) {
        next_stage = Stage::FINAL_APPROACH;
        error_prefix = "final_locked_base";
        final_sampling_retry_count_ = 0;
    }
    RCLCPP_INFO(charging_node_->get_logger(),
                "Locked %s dock target from %zu samples into %s: base=(%.3f, %.3f, %.3f), fixed=(%.3f, %.3f, %.3f)",
                stageName(), sample_count, fixed_frame_.c_str(), averaged_target_base.position.x,
                averaged_target_base.position.y, averaged_target_base.position.z,
                locked_target_fixed_->pose.position.x, locked_target_fixed_->pose.position.y,
                locked_target_fixed_->pose.position.z);
    RCLCPP_INFO(charging_node_->get_logger(),
                "锁定对接目标：采样阶段=%s（%s），样本数=%zu，"
                "平均目标base_link=(%.3f, %.3f, %.3f)，"
                "已转换并锁定到%s=(%.3f, %.3f, %.3f)，下一阶段=%s（%s）",
                stageName(), stageDescription(), sample_count,
                averaged_target_base.position.x, averaged_target_base.position.y,
                averaged_target_base.position.z, fixed_frame_.c_str(),
                locked_target_fixed_->pose.position.x, locked_target_fixed_->pose.position.y,
                locked_target_fixed_->pose.position.z,
                next_stage == Stage::FIRST_APPROACH ? "FIRST_APPROACH" :
                next_stage == Stage::SECOND_APPROACH ? "SECOND_APPROACH" : "FINAL_APPROACH",
                next_stage == Stage::FIRST_APPROACH ? "第一阶段粗靠近" :
                next_stage == Stage::SECOND_APPROACH ? "第二阶段近距离精靠近" : "最终阶段低速精调");
    logTargetError(averaged_target_base, error_prefix);
    logStageBanner(next_stage);
    stage_ = next_stage;
}

void ReverseDockingBtController::handleReachedTarget(const geometry_msgs::msg::Pose& target_pose_base) {
    if (stage_ == Stage::FIRST_APPROACH) {
        logTargetError(target_pose_base, "first_reached_base");
    } else if (stage_ == Stage::SECOND_APPROACH) {
        logTargetError(target_pose_base, "second_reached_base");
    } else if (stage_ == Stage::FINAL_APPROACH) {
        logTargetError(target_pose_base, "final_reached_base");
    }

    if (stage_ == Stage::FIRST_APPROACH) {
        RCLCPP_INFO(charging_node_->get_logger(),
                    "First reverse dock target reached; stop and start second sampling");
        RCLCPP_INFO(charging_node_->get_logger(),
                    "第一阶段目标已到达：停车，清空旧目标，进入 SECOND_SAMPLING（二次近距离采样）");
        locked_target_fixed_.reset();
        target_samples_.clear();
        sampling_started_ = false;
        second_immediate_recheck_done_ = false;
        second_stage_commanded_ = false;
        controller_.reset();
        logStageBanner(Stage::SECOND_SAMPLING);
        stage_ = Stage::SECOND_SAMPLING;
        return;
    }

    if (stage_ == Stage::SECOND_APPROACH) {
        second_reached_error_ = calculateTargetError(target_pose_base);
        final_sampling_retry_count_ = 0;
        RCLCPP_INFO(charging_node_->get_logger(),
                    "Second reverse dock target reached; stop and start final sampling");
        RCLCPP_INFO(charging_node_->get_logger(),
                    "第二阶段目标已到达：停车，记录当前误差，进入 FINAL_SAMPLING（最终目标确认采样）");
        locked_target_fixed_.reset();
        target_samples_.clear();
        sampling_started_ = false;
        controller_.reset();
        logStageBanner(Stage::FINAL_SAMPLING);
        stage_ = Stage::FINAL_SAMPLING;
        return;
    }

    if (stage_ == Stage::FINAL_APPROACH) {
        RCLCPP_INFO(charging_node_->get_logger(), "Final reverse dock target reached; finish Dock controller");
        RCLCPP_INFO(charging_node_->get_logger(),
                    "最终对接目标已到达：Dock 控制器结束，后续由行为树执行末端后退压入和充电确认");
        logStageBanner(Stage::FINISHED);
        stage_ = Stage::FINISHED;
        controller_.reset();
        controller_done_ = true;
        return;
    }

    RCLCPP_WARN(charging_node_->get_logger(), "Reached target in unexpected reverse dock stage %s", stageName());
    logStageBanner(Stage::FINISHED);
    stage_ = Stage::FINISHED;
    controller_done_ = true;
}

bool ReverseDockingBtController::isSamplingStage() const {
    return stage_ == Stage::INITIAL_SAMPLING || stage_ == Stage::SECOND_SAMPLING ||
           stage_ == Stage::FINAL_SAMPLING;
}

double ReverseDockingBtController::currentSamplingDuration() const {
    switch (stage_) {
        case Stage::SECOND_SAMPLING:
            return second_sampling_duration_;
        case Stage::FINAL_SAMPLING:
            if (final_sampling_retry_count_ > 0) {
                return final_resampling_duration_;
            }
            return final_sampling_duration_;
        case Stage::INITIAL_SAMPLING:
        default:
            return initial_sampling_duration_;
    }
}

double ReverseDockingBtController::currentSamplingMaxDistance() const {
    switch (stage_) {
        case Stage::SECOND_SAMPLING:
            return second_sampling_max_distance_;
        case Stage::FINAL_SAMPLING:
            return final_sampling_max_distance_;
        case Stage::INITIAL_SAMPLING:
        default:
            return initial_sampling_max_distance_;
    }
}

int ReverseDockingBtController::currentMinValidSampleCount() const {
    switch (stage_) {
        case Stage::SECOND_SAMPLING:
            return second_min_valid_sample_count_;
        case Stage::FINAL_SAMPLING:
            if (final_sampling_retry_count_ > 0) {
                return final_resample_min_valid_sample_count_;
            }
            return final_min_valid_sample_count_;
        case Stage::INITIAL_SAMPLING:
        default:
            return initial_min_valid_sample_count_;
    }
}

void ReverseDockingBtController::startSampling(double duration) {
    sampling_started_ = true;
    sample_start_time_ = charging_node_->now();
    target_samples_.clear();
    RCLCPP_INFO(charging_node_->get_logger(),
                "Start sampling dock reverse target in %s for %.2f s, need at least %d samples",
                stageName(), duration, currentMinValidSampleCount());
    RCLCPP_INFO(charging_node_->get_logger(),
                "开始采样倒车对接目标：阶段=%s（%s），采样时长=%.2f 秒，至少需要 %d 个有效样本",
                stageName(), stageDescription(), duration, currentMinValidSampleCount());
}

bool ReverseDockingBtController::isSampleFresh(const geometry_msgs::msg::PoseStamped& msg) const {
    if (max_sample_age_ <= 0.0) {
        return true;
    }

    if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0) {
        return true;
    }

    const rclcpp::Time stamp(msg.header.stamp);
    const double age = (charging_node_->now() - stamp).seconds();
    if (age > max_sample_age_) {
        RCLCPP_WARN_THROTTLE(charging_node_->get_logger(), *charging_node_->get_clock(), 1000,
                             "Reject stale dock target sample in %s: age=%.3fs exceeds max %.3fs",
                             stageName(), age, max_sample_age_);
        return false;
    }

    if (age < -0.1) {
        RCLCPP_WARN_THROTTLE(charging_node_->get_logger(), *charging_node_->get_clock(), 1000,
                             "Reject future dock target sample in %s: age=%.3fs",
                             stageName(), age);
        return false;
    }

    return true;
}

bool ReverseDockingBtController::isSampleDistanceValid(const geometry_msgs::msg::Pose& pose) const {
    const double max_distance = currentSamplingMaxDistance();
    if (max_distance <= 0.0) {
        return true;
    }

    const double distance = std::hypot(pose.position.x, pose.position.y);
    if (distance > max_distance) {
        RCLCPP_WARN_THROTTLE(charging_node_->get_logger(), *charging_node_->get_clock(), 1000,
                             "Reject dock target sample in %s: base=(%.3f, %.3f), dist=%.3f exceeds max %.3f",
                             stageName(), pose.position.x, pose.position.y, distance, max_distance);
        return false;
    }

    return true;
}

bool ReverseDockingBtController::isLockedTargetReasonable(const geometry_msgs::msg::Pose& target_pose_base) const {
    return isSampleDistanceValid(target_pose_base);
}

void ReverseDockingBtController::configureControllerForStage() {
    const auto& config = currentApproachConfig();
    controller_.setLinearTolerance(config.linear_tolerance);
    controller_.setLateralTolerance(config.lateral_tolerance);
    controller_.setFinalYawTolerance(config.final_yaw_tolerance);
}

std::pair<double, double> ReverseDockingBtController::limitStageVelocity(double linear_x, double angular_z) const {
    const auto& config = currentApproachConfig();
    if (!config.use_stage_limit) {
        return {linear_x, angular_z};
    }

    linear_x *= config.linear_scale;
    angular_z *= config.angular_scale;
    linear_x = clamp(linear_x, -config.max_linear_speed, config.max_linear_speed);
    angular_z = clamp(angular_z, -config.max_angular_speed, config.max_angular_speed);
    return {linear_x, angular_z};
}

const ReverseDockingBtController::ApproachConfig& ReverseDockingBtController::currentApproachConfig() const {
    if (stage_ == Stage::SECOND_APPROACH) {
        return second_stage_config_;
    }
    if (stage_ == Stage::FINAL_APPROACH) {
        return final_stage_config_;
    }
    return first_stage_config_;
}

double ReverseDockingBtController::clamp(double value, double min_value, double max_value) const {
    return std::max(min_value, std::min(max_value, value));
}

const char* ReverseDockingBtController::stageName() const {
    return stageName(stage_);
}

const char* ReverseDockingBtController::stageName(Stage stage) const {
    switch (stage) {
        case Stage::INITIAL_SAMPLING:
            return "INITIAL_SAMPLING";
        case Stage::FIRST_APPROACH:
            return "FIRST_APPROACH";
        case Stage::SECOND_SAMPLING:
            return "SECOND_SAMPLING";
        case Stage::SECOND_APPROACH:
            return "SECOND_APPROACH";
        case Stage::FINAL_SAMPLING:
            return "FINAL_SAMPLING";
        case Stage::FINAL_APPROACH:
            return "FINAL_APPROACH";
        case Stage::FINISHED:
            return "FINISHED";
    }
    return "UNKNOWN";
}

const char* ReverseDockingBtController::stageDescription() const {
    return stageDescription(stage_);
}

const char* ReverseDockingBtController::stageDescription(Stage stage) const {
    switch (stage) {
        case Stage::INITIAL_SAMPLING:
            return "初始采样：停车收集 AprilTag 目标，用于第一阶段粗靠近";
        case Stage::FIRST_APPROACH:
            return "第一阶段粗靠近：使用初始锁定目标倒车接近";
        case Stage::SECOND_SAMPLING:
            return "二次采样：第一阶段到位后停车，近距离重新采样";
        case Stage::SECOND_APPROACH:
            return "第二阶段精靠近：使用二次锁定目标低速靠近";
        case Stage::FINAL_SAMPLING:
            return "最终采样：第二阶段到位后确认最终目标";
        case Stage::FINAL_APPROACH:
            return "最终精调：使用最终锁定目标低速修正位置和姿态";
        case Stage::FINISHED:
            return "对接控制完成：持续发布零速度";
    }
    return "未知阶段";
}

const char* ReverseDockingBtController::controllerStageDescription() const {
    const std::string controller_stage = controller_.stageName();
    if (controller_stage == "APPROACHING") {
        return "倒车靠近目标";
    }
    if (controller_stage == "ADJUST_HEADING") {
        return "优先修正倒车方向";
    }
    if (controller_stage == "ADJUST_DISTANCE") {
        return "继续修正距离和横向误差";
    }
    if (controller_stage == "ROTATING_TO_GOAL") {
        return "距离很近，修正最终姿态";
    }
    if (controller_stage == "FINISH") {
        return "控制器内部已到位";
    }
    return "未知控制器阶段";
}

const char* ReverseDockingBtController::targetErrorDescription(const std::string& prefix) const {
    if (prefix == "first_locked_base") {
        return "第一阶段锁定目标后的误差";
    }
    if (prefix == "first_reached_base") {
        return "第一阶段到位时的误差";
    }
    if (prefix == "second_locked_base") {
        return "第二阶段锁定目标后的误差";
    }
    if (prefix == "second_reached_base") {
        return "第二阶段到位时的误差";
    }
    if (prefix == "final_locked_base") {
        return "最终阶段锁定目标后的误差";
    }
    if (prefix == "final_reached_base") {
        return "最终阶段到位时的误差";
    }
    return "目标误差";
}

void ReverseDockingBtController::logStageBanner(Stage stage) const {
    RCLCPP_INFO(charging_node_->get_logger(), "-------------------------");
    RCLCPP_INFO(charging_node_->get_logger(), "进入阶段：%s（%s）", stageName(stage), stageDescription(stage));
    RCLCPP_INFO(charging_node_->get_logger(), "-------------------------");
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
                             "Failed to lock dock target into fixed frame '%s': %s", fixed_frame_.c_str(),
                             ex.what());
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
                             "Failed to transform locked dock target from '%s' to '%s': %s",
                             fixed_frame_.c_str(), base_frame_.c_str(), ex.what());
        return std::nullopt;
    }
}

geometry_msgs::msg::Pose ReverseDockingBtController::averageTargetSamples() const {
    geometry_msgs::msg::Pose average;
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> zs;
    xs.reserve(target_samples_.size());
    ys.reserve(target_samples_.size());
    zs.reserve(target_samples_.size());

    for (const auto& sample : target_samples_) {
        xs.push_back(sample.position.x);
        ys.push_back(sample.position.y);
        zs.push_back(sample.position.z);
    }

    average.position.x = trimmedMean(xs);
    average.position.y = trimmedMean(ys);
    average.position.z = trimmedMean(zs);

    tf2::Quaternion yaw_q;
    yaw_q.setRPY(0.0, 0.0, averageSampleYaw());
    yaw_q.normalize();
    average.orientation = tf2::toMsg(yaw_q);

    return average;
}

double ReverseDockingBtController::trimmedMean(std::vector<double> values) const {
    if (values.empty()) {
        return 0.0;
    }

    std::sort(values.begin(), values.end());
    size_t trim_count = static_cast<size_t>(std::floor(static_cast<double>(values.size()) * sample_trim_ratio_));
    if (values.size() - 2 * trim_count < 3) {
        trim_count = 0;
    }

    const size_t begin = trim_count;
    const size_t end = values.size() - trim_count;
    double sum = 0.0;
    for (size_t i = begin; i < end; ++i) {
        sum += values[i];
    }
    return sum / static_cast<double>(end - begin);
}

double ReverseDockingBtController::averageSampleYaw() const {
    if (target_samples_.empty()) {
        return 0.0;
    }

    std::vector<double> yaws;
    yaws.reserve(target_samples_.size());
    const double reference_yaw = tf2::getYaw(target_samples_.front().orientation);
    for (const auto& sample : target_samples_) {
        const double yaw = tf2::getYaw(sample.orientation);
        yaws.push_back(reference_yaw + normalizePi(yaw - reference_yaw));
    }

    return normalizePi(trimmedMean(yaws));
}

ReverseDockingBtController::TargetError ReverseDockingBtController::calculateTargetError(
    const geometry_msgs::msg::Pose& target_pose_base) const {
    TargetError error;
    error.linear = std::abs(target_pose_base.position.x);
    error.lateral = std::abs(target_pose_base.position.y);
    error.final_yaw = std::abs(normalizePi(tf2::getYaw(target_pose_base.orientation) - final_yaw_offset_));
    return error;
}

bool ReverseDockingBtController::isFinalSampleConsistentWithSecondReach(
    const geometry_msgs::msg::Pose& target_pose_base) const {
    if (!second_reached_error_) {
        return true;
    }

    const TargetError final_error = calculateTargetError(target_pose_base);
    const bool linear_worse =
        final_error.linear > second_reached_error_->linear + final_resample_linear_margin_;
    const bool lateral_worse =
        final_error.lateral > second_reached_error_->lateral + final_resample_lateral_margin_;
    const bool yaw_worse =
        final_error.final_yaw > second_reached_error_->final_yaw + final_resample_yaw_margin_;

    if (!linear_worse && !lateral_worse && !yaw_worse) {
        return true;
    }

    if (final_sampling_retry_count_ >= final_resample_max_attempts_) {
        RCLCPP_WARN(charging_node_->get_logger(),
                    "Accept final dock target after %d resampling attempts although it is worse than second reached: "
                    "second=(linear=%.4f lateral=%.4f yaw=%.4f), final=(linear=%.4f lateral=%.4f yaw=%.4f), "
                    "margin=(linear=%.4f lateral=%.4f yaw=%.4f)",
                    final_sampling_retry_count_, second_reached_error_->linear, second_reached_error_->lateral,
                    second_reached_error_->final_yaw, final_error.linear, final_error.lateral, final_error.final_yaw,
                    final_resample_linear_margin_, final_resample_lateral_margin_, final_resample_yaw_margin_);
        return true;
    }

    RCLCPP_WARN(charging_node_->get_logger(),
                "Reject final dock target and resample (%d/%d): error is worse than second reached. "
                "second=(linear=%.4f lateral=%.4f yaw=%.4f), final=(linear=%.4f lateral=%.4f yaw=%.4f), "
                "margin=(linear=%.4f lateral=%.4f yaw=%.4f)",
                final_sampling_retry_count_ + 1, final_resample_max_attempts_,
                second_reached_error_->linear, second_reached_error_->lateral, second_reached_error_->final_yaw,
                final_error.linear, final_error.lateral, final_error.final_yaw,
                final_resample_linear_margin_, final_resample_lateral_margin_, final_resample_yaw_margin_);
    return false;
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
    RCLCPP_INFO(charging_node_->get_logger(),
                "%s：前后误差local_x=%.4f m，横向误差local_y=%.4f m，"
                "距离=%.4f m，倒车方向误差=%.4f rad，最终姿态误差=%.4f rad",
                targetErrorDescription(prefix), local_x, local_y, distance,
                reverse_heading_error, final_yaw_error);
}

void ReverseDockingBtController::logTargetErrorThrottled(const geometry_msgs::msg::Pose& target_pose_base) const {
    const double local_x = target_pose_base.position.x;
    const double local_y = target_pose_base.position.y;
    const double distance = std::hypot(local_x, local_y);
    const double reverse_heading_error = normalizePi(std::atan2(local_y, local_x) + M_PI);
    const double final_yaw_error = normalizePi(tf2::getYaw(target_pose_base.orientation) - final_yaw_offset_);
    RCLCPP_INFO_THROTTLE(charging_node_->get_logger(), *charging_node_->get_clock(), 500,
                         "target error: local_x=%.4f local_y=%.4f dist=%.4f reverse_heading=%.4f rad final_yaw=%.4f rad",
                         local_x, local_y, distance, reverse_heading_error, final_yaw_error);
}

double ReverseDockingBtController::normalizePi(double value) const {
    while (value > M_PI) value -= 2.0 * M_PI;
    while (value < -M_PI) value += 2.0 * M_PI;
    return value;
}

void ReverseDockingBtController::resetState() {
    target_samples_.clear();
    locked_target_fixed_.reset();
    second_reached_error_.reset();
    sampling_started_ = false;
    controller_done_ = false;
    final_sampling_retry_count_ = 0;
    second_immediate_recheck_done_ = false;
    second_stage_commanded_ = false;
    stage_ = Stage::INITIAL_SAMPLING;
    controller_.reset();
}
