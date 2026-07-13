#include "reverse_docking_controller.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>

ReverseDockingController::ReverseDockingController()
    : linear_gain_(0.35),
      heading_gain_(1.20),
      final_yaw_gain_(0.90),
      final_yaw_integral_gain_(0.0),
      final_yaw_derivative_gain_(0.05),
      final_yaw_integral_limit_(0.30),
      final_yaw_offset_(0.0),
      max_linear_speed_(0.18),
      min_linear_speed_(0.015),
      max_angular_speed_(0.45),
      min_angular_speed_(0.015),
      final_max_angular_speed_(0.45),
      final_min_angular_speed_(0.015),
      max_linear_accel_(0.20),
      max_angular_accel_(0.60),
      linear_tolerance_(0.020),
      lateral_tolerance_(0.020),
      heading_tolerance_(0.06),
      final_yaw_tolerance_(0.05),
      approach_distance_(0.25),
      large_heading_threshold_(0.35),
      stage_(Stage::APPROACHING),
      final_yaw_pid_active_(false),
      final_yaw_integral_(0.0),
      prev_final_yaw_error_(0.0),
      final_yaw_pid_time_(std::chrono::steady_clock::now()),
      accel_first_call_(true),
      prev_linear_x_(0.0),
      prev_angular_z_(0.0),
      last_time_(std::chrono::steady_clock::now()) {}

void ReverseDockingController::reset() {
    stage_ = Stage::APPROACHING;
    accel_first_call_ = true;
    prev_linear_x_ = 0.0;
    prev_angular_z_ = 0.0;
    resetFinalYawPid();
    last_time_ = std::chrono::steady_clock::now();
}

void ReverseDockingController::setLinearGain(double gain) { linear_gain_ = gain; }

void ReverseDockingController::setHeadingGain(double gain) { heading_gain_ = gain; }

void ReverseDockingController::setFinalYawGain(double gain) { final_yaw_gain_ = gain; }

void ReverseDockingController::setFinalYawIntegralGain(double gain) { final_yaw_integral_gain_ = gain; }

void ReverseDockingController::setFinalYawDerivativeGain(double gain) { final_yaw_derivative_gain_ = gain; }

void ReverseDockingController::setFinalYawIntegralLimit(double limit) { final_yaw_integral_limit_ = std::abs(limit); }

void ReverseDockingController::setFinalYawOffset(double offset) { final_yaw_offset_ = offset; }

void ReverseDockingController::setMaxLinearSpeed(double speed) { max_linear_speed_ = std::abs(speed); }

void ReverseDockingController::setMinLinearSpeed(double speed) { min_linear_speed_ = std::abs(speed); }

void ReverseDockingController::setMaxAngularSpeed(double speed) { max_angular_speed_ = std::abs(speed); }

void ReverseDockingController::setMinAngularSpeed(double speed) { min_angular_speed_ = std::abs(speed); }

void ReverseDockingController::setFinalMaxAngularSpeed(double speed) { final_max_angular_speed_ = std::abs(speed); }

void ReverseDockingController::setFinalMinAngularSpeed(double speed) { final_min_angular_speed_ = std::abs(speed); }

void ReverseDockingController::setMaxLinearAcceleration(double accel) { max_linear_accel_ = std::abs(accel); }

void ReverseDockingController::setMaxAngularAcceleration(double accel) { max_angular_accel_ = std::abs(accel); }

void ReverseDockingController::setLinearTolerance(double tolerance) { linear_tolerance_ = std::abs(tolerance); }

void ReverseDockingController::setLateralTolerance(double tolerance) { lateral_tolerance_ = std::abs(tolerance); }

void ReverseDockingController::setHeadingTolerance(double tolerance) { heading_tolerance_ = std::abs(tolerance); }

void ReverseDockingController::setFinalYawTolerance(double tolerance) { final_yaw_tolerance_ = std::abs(tolerance); }

void ReverseDockingController::setAlignHeadingThreshold(double tolerance) {
    setLargeHeadingThreshold(tolerance);
}

void ReverseDockingController::setApproachDistance(double distance) { approach_distance_ = std::abs(distance); }

void ReverseDockingController::setLargeHeadingThreshold(double tolerance) {
    large_heading_threshold_ = std::abs(tolerance);
}

std::pair<double, double> ReverseDockingController::getVelocity(const geometry_msgs::msg::Pose& current_pose,
                                                               const geometry_msgs::msg::Pose& goal_pose) {
    const PoseError error = computeError(current_pose, goal_pose);
    const Stage previous_stage = stage_;

    if (atGoal(current_pose, goal_pose)) {
        stage_ = Stage::FINISH;
        prev_linear_x_ = 0.0;
        prev_angular_z_ = 0.0;
        accel_first_call_ = true;
        resetFinalYawPid();
        return {0.0, 0.0};
    }

    if (error.distance > approach_distance_ || stage_ == Stage::FINISH) {
        stage_ = Stage::APPROACHING;
    }

    if (stage_ == Stage::APPROACHING && error.distance <= approach_distance_) {
        stage_ = Stage::ADJUST_HEADING;
    }

    if (stage_ == Stage::ADJUST_HEADING &&
        std::abs(error.reverse_heading_error) <= heading_tolerance_) {
        stage_ = Stage::ADJUST_DISTANCE;
    }

    if (stage_ == Stage::ADJUST_DISTANCE &&
        std::abs(error.local_x) <= linear_tolerance_ &&
        std::abs(error.local_y) > lateral_tolerance_) {
        stage_ = Stage::ADJUST_HEADING;
    }

    if (std::abs(error.local_x) <= linear_tolerance_ * 0.5 &&
        std::abs(error.local_y) <= lateral_tolerance_ * 0.75) {
        stage_ = Stage::ROTATING_TO_GOAL;
    }

    if (stage_ == Stage::ROTATING_TO_GOAL && previous_stage != Stage::ROTATING_TO_GOAL) {
        resetFinalYawPid();
    } else if (stage_ != Stage::ROTATING_TO_GOAL) {
        resetFinalYawPid();
    }

    double linear_x = 0.0;
    double angular_z = 0.0;
    double linear_scale = 0.0;

    switch (stage_) {
        case Stage::FINISH:
            break;
        case Stage::APPROACHING:
            linear_scale = 1.0;
            angular_z = heading_gain_ * error.reverse_heading_error;
            break;
        case Stage::ADJUST_HEADING:
            linear_scale = 0.75;
            angular_z = heading_gain_ * error.reverse_heading_error;
            break;
        case Stage::ADJUST_DISTANCE:
            linear_scale = 0.75;
            angular_z = heading_gain_ * error.reverse_heading_error;
            break;
        case Stage::ROTATING_TO_GOAL:
            linear_scale = 0.25;
            angular_z = computeFinalYawVelocity(error.final_yaw_error);
            break;
    }

    linear_x = linear_gain_ * error.local_x * linear_scale;
    if (stage_ != Stage::ROTATING_TO_GOAL && std::abs(error.reverse_heading_error) > large_heading_threshold_) {
        linear_x = 0.0;
        angular_z = heading_gain_ * error.reverse_heading_error;
    }

    return limitVelocity(linear_x, angular_z, stage_ == Stage::ROTATING_TO_GOAL);
}

bool ReverseDockingController::atGoal(const geometry_msgs::msg::Pose& current_pose,
                                      const geometry_msgs::msg::Pose& goal_pose) const {
    const PoseError error = computeError(current_pose, goal_pose);
    return std::abs(error.local_x) <= linear_tolerance_ &&
           std::abs(error.local_y) <= lateral_tolerance_ &&
           std::abs(error.final_yaw_error) <= final_yaw_tolerance_;
}

std::string ReverseDockingController::stageName() const {
    switch (stage_) {
        case Stage::FINISH:
            return "FINISH";
        case Stage::APPROACHING:
            return "APPROACHING";
        case Stage::ADJUST_HEADING:
            return "ADJUST_HEADING";
        case Stage::ADJUST_DISTANCE:
            return "ADJUST_DISTANCE";
        case Stage::ROTATING_TO_GOAL:
            return "ROTATING_TO_GOAL";
    }
    return "UNKNOWN";
}

ReverseDockingController::PoseError ReverseDockingController::computeError(
    const geometry_msgs::msg::Pose& current_pose,
    const geometry_msgs::msg::Pose& goal_pose) const {
    const double current_yaw = getPoseYaw(current_pose);
    const double goal_yaw = getPoseYaw(goal_pose);
    const double dx = goal_pose.position.x - current_pose.position.x;
    const double dy = goal_pose.position.y - current_pose.position.y;
    const double cos_yaw = std::cos(current_yaw);
    const double sin_yaw = std::sin(current_yaw);

    PoseError error;
    error.local_x = dx * cos_yaw + dy * sin_yaw;
    error.local_y = -dx * sin_yaw + dy * cos_yaw;
    error.distance = std::hypot(error.local_x, error.local_y);
    error.reverse_heading_error = normalizePi(std::atan2(error.local_y, error.local_x) + M_PI);
    error.final_yaw_error = normalizePi(goal_yaw - current_yaw - final_yaw_offset_);
    return error;
}

double ReverseDockingController::computeFinalYawVelocity(double final_yaw_error) {
    const auto now = std::chrono::steady_clock::now();
    double dt = 0.0;
    if (final_yaw_pid_active_) {
        dt = std::chrono::duration<double>(now - final_yaw_pid_time_).count();
    }

    if (!final_yaw_pid_active_ || dt <= 0.0 || dt > 1.0) {
        final_yaw_pid_active_ = true;
        final_yaw_pid_time_ = now;
        prev_final_yaw_error_ = final_yaw_error;
        return final_yaw_gain_ * final_yaw_error;
    }

    final_yaw_integral_ += final_yaw_error * dt;
    final_yaw_integral_ = std::max(-final_yaw_integral_limit_,
                                   std::min(final_yaw_integral_limit_, final_yaw_integral_));
    const double derivative = normalizePi(final_yaw_error - prev_final_yaw_error_) / dt;

    prev_final_yaw_error_ = final_yaw_error;
    final_yaw_pid_time_ = now;
    return final_yaw_gain_ * final_yaw_error +
           final_yaw_integral_gain_ * final_yaw_integral_ +
           final_yaw_derivative_gain_ * derivative;
}

std::pair<double, double> ReverseDockingController::limitVelocity(double linear_x, double angular_z,
                                                                  bool final_rotation) {
    linear_x = std::max(-max_linear_speed_, std::min(max_linear_speed_, linear_x));
    const double angular_max = final_rotation ? final_max_angular_speed_ : max_angular_speed_;
    const double angular_min = final_rotation ? final_min_angular_speed_ : min_angular_speed_;
    angular_z = std::max(-angular_max, std::min(angular_max, angular_z));

    linear_x = applyMinSpeed(linear_x, min_linear_speed_);
    angular_z = applyMinSpeed(angular_z, angular_min);

    const auto now = std::chrono::steady_clock::now();
    if (accel_first_call_) {
        accel_first_call_ = false;
        prev_linear_x_ = linear_x;
        prev_angular_z_ = angular_z;
        last_time_ = now;
        return {linear_x, angular_z};
    }

    const double dt = std::chrono::duration<double>(now - last_time_).count();
    if (dt > 0.0 && dt <= 1.0) {
        const double max_linear_delta = max_linear_accel_ * dt;
        const double max_angular_delta = max_angular_accel_ * dt;
        linear_x = std::max(prev_linear_x_ - max_linear_delta,
                            std::min(prev_linear_x_ + max_linear_delta, linear_x));
        angular_z = std::max(prev_angular_z_ - max_angular_delta,
                             std::min(prev_angular_z_ + max_angular_delta, angular_z));
    }

    linear_x = std::max(-max_linear_speed_, std::min(max_linear_speed_, linear_x));
    angular_z = std::max(-angular_max, std::min(angular_max, angular_z));

    prev_linear_x_ = linear_x;
    prev_angular_z_ = angular_z;
    last_time_ = now;
    return {linear_x, angular_z};
}

double ReverseDockingController::applyMinSpeed(double value, double min_value) const {
    if (std::abs(value) <= 1e-6 || std::abs(value) >= min_value) {
        return value;
    }
    return value > 0.0 ? min_value : -min_value;
}

void ReverseDockingController::resetFinalYawPid() {
    final_yaw_pid_active_ = false;
    final_yaw_integral_ = 0.0;
    prev_final_yaw_error_ = 0.0;
    final_yaw_pid_time_ = std::chrono::steady_clock::now();
}

double ReverseDockingController::getPoseYaw(const geometry_msgs::msg::Pose& pose) const {
    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

double ReverseDockingController::normalizePi(double value) const {
    while (value > M_PI) value -= 2.0 * M_PI;
    while (value < -M_PI) value += 2.0 * M_PI;
    return value;
}
