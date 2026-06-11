#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>
#include <vector>

#include "reverse_docking_controller.h"

class ReverseDockingControllerTester : public rclcpp::Node {
public:
    ReverseDockingControllerTester() : Node("reverse_docking_controller_tester") {
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        target_base_frame_ = declare_parameter<std::string>("target_base_frame", "reverse_docking_target");
        static_target_frame_ = declare_parameter<std::string>("static_target_frame", "reverse_docking_static_target");
        target_topic_ = declare_parameter<std::string>("target_topic", "apriltag_target_pose");
        fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
        cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "control/cmd_vel_nav");
        target_lock_duration_ = declare_parameter<double>("target_lock_duration", 2.0);
        if (target_lock_duration_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "Invalid target_lock_duration %.3f, use 2.0 s instead", target_lock_duration_);
            target_lock_duration_ = 2.0;
        }
        refinement_lock_duration_ = declare_parameter<double>("refinement_lock_duration", 1.0);
        if (refinement_lock_duration_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "Invalid refinement_lock_duration %.3f, use 1.0 s instead",
                        refinement_lock_duration_);
            refinement_lock_duration_ = 1.0;
        }
        second_stage_linear_scale_ = declare_parameter<double>("second_stage_linear_scale", 0.5);
        second_stage_angular_scale_ = declare_parameter<double>("second_stage_angular_scale", 0.5);
        second_stage_max_linear_speed_ = std::abs(declare_parameter<double>("second_stage_max_linear_speed", 0.08));
        second_stage_max_angular_speed_ = std::abs(declare_parameter<double>("second_stage_max_angular_speed", 0.20));
        double control_rate = declare_parameter<double>("control_rate", 20.0);
        if (control_rate <= 0.0) {
            RCLCPP_WARN(get_logger(), "Invalid control_rate %.3f, use 20.0 Hz instead", control_rate);
            control_rate = 20.0;
        }

        controller_.setLinearGain(declare_parameter<double>("linear_gain", 0.35));
        controller_.setHeadingGain(declare_parameter<double>("heading_gain", 1.20));
        controller_.setFinalYawGain(declare_parameter<double>("final_yaw_gain", 0.90));
        controller_.setFinalYawIntegralGain(declare_parameter<double>("final_yaw_i_gain", 0.0));
        controller_.setFinalYawDerivativeGain(declare_parameter<double>("final_yaw_d_gain", 0.05));
        controller_.setFinalYawIntegralLimit(declare_parameter<double>("final_yaw_i_limit", 0.30));
        final_yaw_offset_ = declare_parameter<double>("final_yaw_offset", 0.0);
        controller_.setFinalYawOffset(final_yaw_offset_);
        controller_.setLinearTolerance(declare_parameter<double>("linear_tolerance", 0.020));
        controller_.setLateralTolerance(declare_parameter<double>("lateral_tolerance", 0.020));
        controller_.setHeadingTolerance(declare_parameter<double>("heading_tolerance", 0.06));
        controller_.setFinalYawTolerance(declare_parameter<double>("final_yaw_tolerance", 0.02));
        controller_.setApproachDistance(declare_parameter<double>("approach_distance", 0.25));
        controller_.setLargeHeadingThreshold(declare_parameter<double>("large_heading_threshold", 0.35));
        controller_.setMaxLinearSpeed(declare_parameter<double>("max_linear_speed", 0.18));
        controller_.setMinLinearSpeed(declare_parameter<double>("min_linear_speed", 0.015));
        controller_.setMaxAngularSpeed(declare_parameter<double>("max_angular_speed", 0.45));
        controller_.setMinAngularSpeed(declare_parameter<double>("min_angular_speed", 0.015));
        controller_.setFinalMaxAngularSpeed(declare_parameter<double>("final_max_angular_speed", 0.15));
        controller_.setFinalMinAngularSpeed(declare_parameter<double>("final_min_angular_speed", 0.0));
        controller_.setMaxLinearAcceleration(declare_parameter<double>("max_linear_accel", 0.20));
        controller_.setMaxAngularAcceleration(declare_parameter<double>("max_angular_accel", 0.60));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
        target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            target_topic_, 10, std::bind(&ReverseDockingControllerTester::targetCallback, this, std::placeholders::_1));

        const auto period = std::chrono::duration<double>(1.0 / control_rate);
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                   std::bind(&ReverseDockingControllerTester::controlLoop, this));

        RCLCPP_INFO(get_logger(),
                    "Reverse docking controller tester started. target_topic=%s, cmd_vel_topic=%s, base_frame=%s",
                    target_topic_.c_str(), cmd_vel_topic_.c_str(), base_frame_.c_str());
    }

private:
    enum class TestStage {
        INITIAL_SAMPLING,
        FIRST_APPROACH,
        REFINEMENT_SAMPLING,
        SECOND_APPROACH,
        FINISHED,
    };

    void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!msg->header.frame_id.empty() && msg->header.frame_id != base_frame_) {
            RCLCPP_WARN(get_logger(), "Ignore target in frame '%s'; expected '%s'", msg->header.frame_id.c_str(),
                        base_frame_.c_str());
            return;
        }

        if (!isSamplingStage()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Ignore live target update in %s", testStageName());
            return;
        }

        if (!sampling_started_) {
            startSampling(currentSamplingDuration());
        }

        target_samples_.push_back(msg->pose);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "Sampling reverse target: test_stage=%s count=%zu latest=(%.3f, %.3f)", testStageName(),
                             target_samples_.size(), msg->pose.position.x, msg->pose.position.y);
    }

    void controlLoop() {
        if (test_stage_ == TestStage::FINISHED) {
            publishZero();
            publishStaticTargetTf();
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Reverse docking tester finished");
            return;
        }

        if (isSamplingStage()) {
            publishZero();
            handleSamplingStage();
            return;
        }

        if (!locked_target_fixed_) {
            if (!sampling_started_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "No locked target in %s; restart initial sampling", testStageName());
            }
            test_stage_ = TestStage::INITIAL_SAMPLING;
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
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = v;
        cmd.angular.z = w;
        cmd_pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "reverse control: test_stage=%s controller_stage=%s target_base=(%.3f, %.3f), raw_v=%.4f "
                             "raw_w=%.4f v=%.4f w=%.4f",
                             testStageName(), controller_.stageName().c_str(), target_pose_base->position.x,
                             target_pose_base->position.y, raw_v, raw_w, v, w);
        logTargetErrorThrottled(target_pose_base.value());
    }

    void handleSamplingStage() {
        if (!sampling_started_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for target on %s in %s",
                                 target_topic_.c_str(), testStageName());
            return;
        }

        const double duration = currentSamplingDuration();
        const double elapsed = (now() - sample_start_time_).seconds();
        if (elapsed < duration) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Sampling target in %s: %.2f / %.2f s, count=%zu",
                                 testStageName(), elapsed, duration, target_samples_.size());
            return;
        }

        if (target_samples_.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "No target samples collected in %s; restart sampling", testStageName());
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
        const TestStage next_stage =
            test_stage_ == TestStage::INITIAL_SAMPLING ? TestStage::FIRST_APPROACH : TestStage::SECOND_APPROACH;
        RCLCPP_INFO(
            get_logger(),
            "Locked %s reverse target from %zu samples into %s: base=(%.3f, %.3f, %.3f), fixed=(%.3f, %.3f, %.3f)",
            testStageName(), sample_count, fixed_frame_.c_str(), averaged_target_base.position.x,
            averaged_target_base.position.y, averaged_target_base.position.z, locked_target_fixed_->pose.position.x,
            locked_target_fixed_->pose.position.y, locked_target_fixed_->pose.position.z);
        logTargetError(averaged_target_base,
                       test_stage_ == TestStage::INITIAL_SAMPLING ? "first_locked_base" : "second_locked_base");
        test_stage_ = next_stage;
    }

    void handleReachedTarget(const geometry_msgs::msg::Pose& target_pose_base) {
        logTargetError(target_pose_base,
                       test_stage_ == TestStage::FIRST_APPROACH ? "first_reached_base" : "second_reached_base");
        if (test_stage_ == TestStage::FIRST_APPROACH) {
            RCLCPP_INFO(get_logger(), "First reverse docking target reached; stop and start refinement sampling");
            locked_target_fixed_.reset();
            target_samples_.clear();
            sampling_started_ = false;
            controller_.reset();
            test_stage_ = TestStage::REFINEMENT_SAMPLING;
            return;
        }

        if (test_stage_ == TestStage::SECOND_APPROACH) {
            RCLCPP_INFO(get_logger(), "Second reverse docking target reached; finish tester");
            test_stage_ = TestStage::FINISHED;
            controller_.reset();
            return;
        }

        RCLCPP_WARN(get_logger(), "Reached target in unexpected test stage %s", testStageName());
        test_stage_ = TestStage::FINISHED;
    }

    bool isSamplingStage() const {
        return test_stage_ == TestStage::INITIAL_SAMPLING || test_stage_ == TestStage::REFINEMENT_SAMPLING;
    }

    double currentSamplingDuration() const {
        return test_stage_ == TestStage::REFINEMENT_SAMPLING ? refinement_lock_duration_ : target_lock_duration_;
    }

    void startSampling(double duration) {
        sampling_started_ = true;
        sample_start_time_ = now();
        target_samples_.clear();
        RCLCPP_INFO(get_logger(), "Start sampling reverse target in %s for %.2f s", testStageName(), duration);
    }

    std::pair<double, double> limitStageVelocity(double linear_x, double angular_z) const {
        if (test_stage_ != TestStage::SECOND_APPROACH) {
            return {linear_x, angular_z};
        }

        linear_x *= second_stage_linear_scale_;
        angular_z *= second_stage_angular_scale_;
        linear_x = clamp(linear_x, -second_stage_max_linear_speed_, second_stage_max_linear_speed_);
        angular_z = clamp(angular_z, -second_stage_max_angular_speed_, second_stage_max_angular_speed_);
        return {linear_x, angular_z};
    }

    double clamp(double value, double min_value, double max_value) const {
        return std::max(min_value, std::min(max_value, value));
    }

    const char* testStageName() const {
        switch (test_stage_) {
            case TestStage::INITIAL_SAMPLING:
                return "INITIAL_SAMPLING";
            case TestStage::FIRST_APPROACH:
                return "FIRST_APPROACH";
            case TestStage::REFINEMENT_SAMPLING:
                return "REFINEMENT_SAMPLING";
            case TestStage::SECOND_APPROACH:
                return "SECOND_APPROACH";
            case TestStage::FINISHED:
                return "FINISHED";
        }
        return "UNKNOWN";
    }

    std::optional<geometry_msgs::msg::PoseStamped> lockTargetInFixedFrame(
        const geometry_msgs::msg::Pose& target_pose_base) {
        geometry_msgs::msg::PoseStamped target_base;
        target_base.header.stamp = now();
        target_base.header.frame_id = base_frame_;
        target_base.pose = target_pose_base;

        try {
            const auto base_to_fixed = tf_buffer_->lookupTransform(fixed_frame_, base_frame_, tf2::TimePointZero);
            geometry_msgs::msg::PoseStamped target_fixed;
            tf2::doTransform(target_base, target_fixed, base_to_fixed);
            target_fixed.header.stamp = now();
            target_fixed.header.frame_id = fixed_frame_;
            return target_fixed;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Failed to lock target into fixed frame '%s': %s",
                                 fixed_frame_.c_str(), ex.what());
            return std::nullopt;
        }
    }

    std::optional<geometry_msgs::msg::Pose> getLockedTargetInBaseFrame() {
        if (!locked_target_fixed_) {
            return std::nullopt;
        }

        try {
            const auto fixed_to_base = tf_buffer_->lookupTransform(base_frame_, fixed_frame_, tf2::TimePointZero);
            geometry_msgs::msg::PoseStamped target_base;
            tf2::doTransform(locked_target_fixed_.value(), target_base, fixed_to_base);
            return target_base.pose;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Failed to transform locked target from '%s' to '%s': %s", fixed_frame_.c_str(),
                                 base_frame_.c_str(), ex.what());
            return std::nullopt;
        }
    }

    void logTargetError(const geometry_msgs::msg::Pose& target_pose_base, const std::string& prefix) const {
        const double local_x = target_pose_base.position.x;
        const double local_y = target_pose_base.position.y;
        const double distance = std::hypot(local_x, local_y);
        const double reverse_heading_error = normalizePi(std::atan2(local_y, local_x) + M_PI);
        const double final_yaw_error = normalizePi(tf2::getYaw(target_pose_base.orientation) - final_yaw_offset_);
        RCLCPP_INFO(get_logger(),
                    "%s target error: local_x=%.4f local_y=%.4f dist=%.4f reverse_heading=%.4f rad final_yaw=%.4f rad",
                    prefix.c_str(), local_x, local_y, distance, reverse_heading_error, final_yaw_error);
    }

    void logTargetErrorThrottled(const geometry_msgs::msg::Pose& target_pose_base) {
        const double local_x = target_pose_base.position.x;
        const double local_y = target_pose_base.position.y;
        const double distance = std::hypot(local_x, local_y);
        const double reverse_heading_error = normalizePi(std::atan2(local_y, local_x) + M_PI);
        const double final_yaw_error = normalizePi(tf2::getYaw(target_pose_base.orientation) - final_yaw_offset_);
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "target error: local_x=%.4f local_y=%.4f dist=%.4f reverse_heading=%.4f rad final_yaw=%.4f rad", local_x,
            local_y, distance, reverse_heading_error, final_yaw_error);
    }

    double normalizePi(double value) const {
        while (value > M_PI) value -= 2.0 * M_PI;
        while (value < -M_PI) value += 2.0 * M_PI;
        return value;
    }

    geometry_msgs::msg::Pose averageTargetSamples() const {
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

    void publishZero() {
        geometry_msgs::msg::Twist zero;
        cmd_pub_->publish(zero);
    }

    void publishStaticTargetTf() {
        if (!locked_target_fixed_) {
            return;
        }

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now();
        tf.header.frame_id = fixed_frame_;
        tf.child_frame_id = static_target_frame_;
        tf.transform.translation.x = locked_target_fixed_->pose.position.x;
        tf.transform.translation.y = locked_target_fixed_->pose.position.y;
        tf.transform.translation.z = locked_target_fixed_->pose.position.z;
        tf.transform.rotation = locked_target_fixed_->pose.orientation;
        tf_broadcaster_->sendTransform(tf);
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

    ReverseDockingController controller_;
    std::string base_frame_;
    std::string target_base_frame_;
    std::string static_target_frame_;
    std::string target_topic_;
    std::string fixed_frame_;
    std::string cmd_vel_topic_;
    double target_lock_duration_;
    double refinement_lock_duration_;
    double second_stage_linear_scale_;
    double second_stage_angular_scale_;
    double second_stage_max_linear_speed_;
    double second_stage_max_angular_speed_;
    double final_yaw_offset_;
    TestStage test_stage_ = TestStage::INITIAL_SAMPLING;
    bool sampling_started_ = false;
    rclcpp::Time sample_start_time_;
    std::vector<geometry_msgs::msg::Pose> target_samples_;
    std::optional<geometry_msgs::msg::PoseStamped> locked_target_fixed_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReverseDockingControllerTester>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
