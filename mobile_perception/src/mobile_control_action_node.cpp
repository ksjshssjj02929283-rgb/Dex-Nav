// ----------------------------------------------------------------------------
// Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_interfaces/action/precise_control.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class MobileControlActionNode : public rclcpp::Node
{
public:
    using PreciseControl = nav_interfaces::action::PreciseControl;
    using GoalHandlePreciseControl = rclcpp_action::ServerGoalHandle<PreciseControl>;

    struct PidConfig
    {
        bool integrator_enable{true};
        double integrator_saturation_level{0.3};
        double kp{0.0};
        double ki{0.0};
        double kd{0.0};
    };

    struct PidState
    {
        double integral{0.0};
        double prev_error{0.0};
        bool has_prev{false};
    };

    enum class GoalMode
    {
        STRAIGHT,
        ROTATE
    };

    MobileControlActionNode() : Node("mobile_control_action_node")
    {
        this->declare_parameter("control_frequency", 20.0);
        this->declare_parameter("max_v", 0.35);
        this->declare_parameter("min_v", 0.0);
        this->declare_parameter("max_w", 1.0);
        this->declare_parameter("accel_x_max", 0.5);
        this->declare_parameter("straight_max_w_inc", 1.57);
        this->declare_parameter("rotate_to_heading_max_angular_accel", 1.0);
        this->declare_parameter("rotate_stop_yaw_error", 0.02);
        this->declare_parameter("k_rho", 0.8);
        this->declare_parameter("near_target_dist", 0.3);
        this->declare_parameter("cmd_vel_filter_alpha", 0.3);
        this->declare_parameter("min_angular_vel", 0.01);
        this->declare_parameter("xy_tolerance", 0.02);
        this->declare_parameter("yaw_tolerance", 0.04);
        this->declare_parameter("goal_mode_rotate_xy_threshold", 0.03);
        this->declare_parameter("pure_rotation.angular_pid.integrator_enable", true);
        this->declare_parameter("pure_rotation.angular_pid.integrator_saturation_level", 0.5);
        this->declare_parameter("pure_rotation.angular_pid.kp", 1.0);
        this->declare_parameter("pure_rotation.angular_pid.ki", 0.01);
        this->declare_parameter("pure_rotation.angular_pid.kd", 0.10);
        this->declare_parameter("straight.station_pid.integrator_enable", true);
        this->declare_parameter("straight.station_pid.integrator_saturation_level", 0.3);
        this->declare_parameter("straight.station_pid.kp", 1.0);
        this->declare_parameter("straight.station_pid.ki", 0.01);
        this->declare_parameter("straight.station_pid.kd", 0.10);
        this->declare_parameter("straight.high_speed_pid.integrator_enable", true);
        this->declare_parameter("straight.high_speed_pid.integrator_saturation_level", 0.3);
        this->declare_parameter("straight.high_speed_pid.kp", 1.0);
        this->declare_parameter("straight.high_speed_pid.ki", 0.01);
        this->declare_parameter("straight.high_speed_pid.kd", 0.10);
        this->declare_parameter("straight.low_speed_pid.integrator_enable", true);
        this->declare_parameter("straight.low_speed_pid.integrator_saturation_level", 0.3);
        this->declare_parameter("straight.low_speed_pid.kp", 1.0);
        this->declare_parameter("straight.low_speed_pid.ki", 0.01);
        this->declare_parameter("straight.low_speed_pid.kd", 0.10);
        this->declare_parameter("straight.reverse_station_pid.integrator_enable", true);
        this->declare_parameter("straight.reverse_station_pid.integrator_saturation_level", 0.5);
        this->declare_parameter("straight.reverse_station_pid.kp", 1.0);
        this->declare_parameter("straight.reverse_station_pid.ki", 0.01);
        this->declare_parameter("straight.reverse_station_pid.kd", 0.10);
        this->declare_parameter("straight.reverse_speed_pid.integrator_enable", true);
        this->declare_parameter("straight.reverse_speed_pid.integrator_saturation_level", 0.5);
        this->declare_parameter("straight.reverse_speed_pid.kp", 1.0);
        this->declare_parameter("straight.reverse_speed_pid.ki", 0.01);
        this->declare_parameter("straight.reverse_speed_pid.kd", 0.10);
        this->declare_parameter("straight.angular_pid.kp", 1.0);
        this->declare_parameter("straight.angular_pid.ki", 0.01);
        this->declare_parameter("straight.angular_pid.kd", 0.10);
        this->declare_parameter("straight.switch_speed", 0.1);

        control_frequency_ = this->get_parameter("control_frequency").as_double();
        max_v_ = this->get_parameter("max_v").as_double();
        min_v_ = this->get_parameter("min_v").as_double();
        max_w_ = this->get_parameter("max_w").as_double();
        accel_x_max_ = this->get_parameter("accel_x_max").as_double();
        straight_max_w_inc_ = this->get_parameter("straight_max_w_inc").as_double();
        rotate_to_heading_max_angular_accel_ =
            this->get_parameter("rotate_to_heading_max_angular_accel").as_double();
        rotate_stop_yaw_error_ = this->get_parameter("rotate_stop_yaw_error").as_double();
        k_rho_ = this->get_parameter("k_rho").as_double();
        near_target_dist_ = this->get_parameter("near_target_dist").as_double();
        filter_alpha_ = this->get_parameter("cmd_vel_filter_alpha").as_double();
        min_angular_vel_ = this->get_parameter("min_angular_vel").as_double();
        xy_tolerance_ = this->get_parameter("xy_tolerance").as_double();
        yaw_tolerance_ = this->get_parameter("yaw_tolerance").as_double();
        goal_mode_rotate_xy_threshold_ = this->get_parameter("goal_mode_rotate_xy_threshold").as_double();
        pure_rotation_angular_pid_conf_ = loadPidConfig("pure_rotation.angular_pid", 0.5);
        straight_station_pid_conf_ = loadPidConfig("straight.station_pid", 0.3);
        straight_high_speed_pid_conf_ = loadPidConfig("straight.high_speed_pid", 0.3);
        straight_low_speed_pid_conf_ = loadPidConfig("straight.low_speed_pid", 0.3);
        straight_reverse_station_pid_conf_ = loadPidConfig("straight.reverse_station_pid", 0.5);
        straight_reverse_speed_pid_conf_ = loadPidConfig("straight.reverse_speed_pid", 0.5);
        straight_angular_kp_ = this->get_parameter("straight.angular_pid.kp").as_double();
        straight_angular_ki_ = this->get_parameter("straight.angular_pid.ki").as_double();
        straight_angular_kd_ = this->get_parameter("straight.angular_pid.kd").as_double();
        straight_switch_speed_ = this->get_parameter("straight.switch_speed").as_double();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("control/cmd_vel", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "feedback/odom", 10,
            std::bind(&MobileControlActionNode::odomCallback, this, std::placeholders::_1));

        action_server_ = rclcpp_action::create_server<PreciseControl>(
            this,
            "control/precise_control",
            std::bind(&MobileControlActionNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MobileControlActionNode::handleCancel, this, std::placeholders::_1),
            std::bind(&MobileControlActionNode::handleAccepted, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_)),
            std::bind(&MobileControlActionNode::controlLoop, this));

        resetControlState();
    }

private:
    rclcpp_action::Server<PreciseControl>::SharedPtr action_server_;
    std::shared_ptr<GoalHandlePreciseControl> active_goal_handle_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::PoseStamped target_pose_odom_;
    geometry_msgs::msg::Twist pre_cmd_vel_;
    geometry_msgs::msg::Twist filtered_cmd_vel_;
    bool get_map_pose_{false};
    bool is_goal_{false};
    bool is_cancelling_{false};
    int stop_count_{0};
    GoalMode goal_mode_{GoalMode::STRAIGHT};

    double control_frequency_;
    double max_v_;
    double min_v_;
    double max_w_;
    double accel_x_max_;
    double straight_max_w_inc_;
    double rotate_to_heading_max_angular_accel_;
    double rotate_stop_yaw_error_;
    double k_rho_;
    double near_target_dist_;
    double filter_alpha_;
    double min_angular_vel_;
    double xy_tolerance_;
    double yaw_tolerance_;
    double goal_mode_rotate_xy_threshold_;
    PidConfig pure_rotation_angular_pid_conf_;
    PidConfig straight_station_pid_conf_;
    PidConfig straight_high_speed_pid_conf_;
    PidConfig straight_low_speed_pid_conf_;
    PidConfig straight_reverse_station_pid_conf_;
    PidConfig straight_reverse_speed_pid_conf_;
    PidState pure_rotation_angular_pid_state_;
    PidState straight_station_pid_state_;
    PidState straight_high_speed_pid_state_;
    PidState straight_low_speed_pid_state_;
    PidState straight_reverse_station_pid_state_;
    PidState straight_reverse_speed_pid_state_;
    double straight_angular_kp_;
    double straight_angular_ki_;
    double straight_angular_kd_;
    double straight_switch_speed_;
    double straight_angular_error_prev_{0.0};
    double straight_angular_integral_{0.0};

    PidConfig loadPidConfig(const std::string &prefix, double default_integrator_saturation_level)
    {
        PidConfig config;
        config.integrator_enable = this->get_parameter(prefix + ".integrator_enable").as_bool();
        config.integrator_saturation_level =
            this->get_parameter(prefix + ".integrator_saturation_level").as_double();
        config.kp = this->get_parameter(prefix + ".kp").as_double();
        config.ki = this->get_parameter(prefix + ".ki").as_double();
        config.kd = this->get_parameter(prefix + ".kd").as_double();
        if (config.integrator_saturation_level <= 0.0) {
            config.integrator_saturation_level = default_integrator_saturation_level;
        }
        return config;
    }

    void resetPidState(PidState &state)
    {
        state.integral = 0.0;
        state.prev_error = 0.0;
        state.has_prev = false;
    }

    void resetPidStates()
    {
        resetPidState(pure_rotation_angular_pid_state_);
        resetPidState(straight_station_pid_state_);
        resetPidState(straight_high_speed_pid_state_);
        resetPidState(straight_low_speed_pid_state_);
        resetPidState(straight_reverse_station_pid_state_);
        resetPidState(straight_reverse_speed_pid_state_);
        straight_angular_error_prev_ = 0.0;
        straight_angular_integral_ = 0.0;
    }

    void resetControlState()
    {
        get_map_pose_ = false;
        is_goal_ = false;
        is_cancelling_ = false;
        stop_count_ = 0;
        goal_mode_ = GoalMode::STRAIGHT;
        filtered_cmd_vel_.linear.x = 0.0;
        filtered_cmd_vel_.angular.z = 0.0;
        pre_cmd_vel_.linear.x = 0.0;
        pre_cmd_vel_.angular.z = 0.0;
        resetPidStates();
    }

    double runPid(const PidConfig &config, PidState &state, double error, double dt)
    {
        if (dt <= 0.0 || !std::isfinite(error)) {
            return 0.0;
        }

        if (config.integrator_enable) {
            state.integral += error * dt;
            state.integral = std::clamp(
                state.integral,
                -config.integrator_saturation_level,
                config.integrator_saturation_level);
        } else {
            state.integral = 0.0;
        }

        double derivative = 0.0;
        if (state.has_prev) {
            derivative = (error - state.prev_error) / dt;
        }

        state.prev_error = error;
        state.has_prev = true;
        return config.kp * error + config.ki * state.integral + config.kd * derivative;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        pre_cmd_vel_.linear.x = msg->twist.twist.linear.x;
        pre_cmd_vel_.angular.z = msg->twist.twist.angular.z;
    }

    bool transformTargetToMap(
        const geometry_msgs::msg::PoseStamped &target_pose,
        geometry_msgs::msg::PoseStamped &target_pose_map,
        std::string &error_msg)
    {
        try {
            if (target_pose.header.frame_id != "map") {
                target_pose_map = tf_buffer_->transform(target_pose, "map", tf2::durationFromSec(0.1));
            } else {
                target_pose_map = target_pose;
            }
        } catch (const tf2::TransformException &ex) {
            error_msg = ex.what();
            return false;
        }
        return true;
    }

    void setTarget(const geometry_msgs::msg::PoseStamped &target_pose)
    {
        target_pose_odom_ = target_pose;
        get_map_pose_ = true;
        is_goal_ = false;
        is_cancelling_ = false;
        stop_count_ = 0;
        filtered_cmd_vel_.linear.x = 0.0;
        filtered_cmd_vel_.angular.z = 0.0;
        resetPidStates();
    }

    void setGoalModeFromInput(const geometry_msgs::msg::PoseStamped &input_pose)
    {
        const double input_dx = input_pose.pose.position.x;
        const double input_dy = input_pose.pose.position.y;
        const double input_yaw = tf2::getYaw(input_pose.pose.orientation);
        const double input_dist = std::hypot(input_dx, input_dy);

        if (input_dist < goal_mode_rotate_xy_threshold_ && std::fabs(input_yaw) >= yaw_tolerance_) {
            goal_mode_ = GoalMode::ROTATE;
        } else {
            goal_mode_ = GoalMode::STRAIGHT;
        }
    }

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const PreciseControl::Goal> goal)
    {
        if (active_goal_handle_) {
            RCLCPP_WARN(this->get_logger(), "Rejecting precise control goal because another goal is active");
            return rclcpp_action::GoalResponse::REJECT;
        }

        geometry_msgs::msg::PoseStamped target_pose_map;
        std::string error_msg;
        if (!transformTargetToMap(goal->target_pose, target_pose_map, error_msg)) {
            RCLCPP_WARN(this->get_logger(), "Rejecting precise control goal: cannot transform goal to map: %s",
                        error_msg.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandlePreciseControl>)
    {
        RCLCPP_INFO(this->get_logger(), "Precise control cancel requested");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandlePreciseControl> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        geometry_msgs::msg::PoseStamped target_pose_map;
        std::string error_msg;
        if (!transformTargetToMap(goal->target_pose, target_pose_map, error_msg)) {
            auto result = std::make_shared<PreciseControl::Result>();
            result->success = false;
            result->message = "Cannot transform goal to map: " + error_msg;
            goal_handle->abort(result);
            return;
        }

        active_goal_handle_ = goal_handle;
        setGoalModeFromInput(goal->target_pose);
        setTarget(target_pose_map);
        RCLCPP_INFO(
            this->get_logger(),
            "Accepted precise control goal in map frame, mode=%s",
            goal_mode_ == GoalMode::STRAIGHT ? "STRAIGHT" : "ROTATE");
    }

    void publishFeedback(const std::string &status)
    {
        if (!active_goal_handle_) {
            return;
        }

        auto feedback = std::make_shared<PreciseControl::Feedback>();
        feedback->current_status = status;
        active_goal_handle_->publish_feedback(feedback);
    }

    void finishSucceeded()
    {
        if (!active_goal_handle_) {
            return;
        }

        auto result = std::make_shared<PreciseControl::Result>();
        result->success = true;
        result->message = "SUCCEEDED";
        active_goal_handle_->succeed(result);
        active_goal_handle_.reset();
        resetControlState();
    }

    void finishCanceled()
    {
        if (!active_goal_handle_) {
            resetControlState();
            return;
        }

        auto result = std::make_shared<PreciseControl::Result>();
        result->success = false;
        result->message = "CANCELLED";
        active_goal_handle_->canceled(result);
        active_goal_handle_.reset();
        resetControlState();
    }

    void startCancellation()
    {
        if (!is_cancelling_) {
            is_cancelling_ = true;
            get_map_pose_ = false;
            is_goal_ = false;
            stop_count_ = 0;
            resetPidStates();
            RCLCPP_INFO(this->get_logger(), "Precise control cancel accepted, starting smooth stop");
        }
    }

    geometry_msgs::msg::Twist computeRotateToHeadingCommand(double yaw_error, double dt)
    {
        geometry_msgs::msg::Twist cmd_vel;
        const double abs_error = std::fabs(yaw_error);

        if (abs_error < rotate_stop_yaw_error_) {
            resetPidState(pure_rotation_angular_pid_state_);
            return cmd_vel;
        }

        double angular_vel = runPid(
            pure_rotation_angular_pid_conf_, pure_rotation_angular_pid_state_, yaw_error, dt);
        angular_vel = std::clamp(angular_vel, -max_w_, max_w_);
        if (std::fabs(angular_vel) < min_angular_vel_) {
            angular_vel = std::copysign(min_angular_vel_, yaw_error);
        }

        cmd_vel.angular.z = angular_vel;
        return cmd_vel;
    }

    geometry_msgs::msg::Twist computeStraightPidCommand(
        const geometry_msgs::msg::Pose &target_pose,
        const geometry_msgs::msg::Twist &current_velocity,
        double dt,
        bool reverse_mode)
    {
        geometry_msgs::msg::Twist cmd_vel;
        const double err_x = target_pose.position.x;
        const double raw_target_yaw_in_base = std::atan2(
            std::sin(tf2::getYaw(target_pose.orientation)),
            std::cos(tf2::getYaw(target_pose.orientation)));
        (void)reverse_mode;
        const double yaw_error =
            std::fabs(raw_target_yaw_in_base) < yaw_tolerance_ ? 0.0 : raw_target_yaw_in_base;

        const double dist = std::fabs(err_x);
        double desired_linear_vel = std::min(max_v_, k_rho_ * dist);
        if (dist < near_target_dist_ && near_target_dist_ > 0.0) {
            desired_linear_vel *= std::max(0.2, dist / near_target_dist_);
        }
        desired_linear_vel = std::clamp(desired_linear_vel, min_v_, max_v_);
        if (reverse_mode) {
            desired_linear_vel = -desired_linear_vel;
        }

        const PidConfig &station_conf =
            reverse_mode ? straight_reverse_station_pid_conf_ : straight_station_pid_conf_;
        PidState &station_state =
            reverse_mode ? straight_reverse_station_pid_state_ : straight_station_pid_state_;
        const bool use_low_speed = std::fabs(current_velocity.linear.x) < straight_switch_speed_;
        const PidConfig &speed_conf = reverse_mode
            ? straight_reverse_speed_pid_conf_
            : (use_low_speed ? straight_low_speed_pid_conf_ : straight_high_speed_pid_conf_);
        PidState &speed_state = reverse_mode
            ? straight_reverse_speed_pid_state_
            : (use_low_speed ? straight_low_speed_pid_state_ : straight_high_speed_pid_state_);

        const double speed_error = desired_linear_vel - current_velocity.linear.x;
        const double speed_offset = runPid(station_conf, station_state, err_x, dt);
        const double speed_controller_input = speed_offset + speed_error;
        double acceleration_cmd = runPid(speed_conf, speed_state, speed_controller_input, dt);
        acceleration_cmd = std::clamp(acceleration_cmd, -accel_x_max_, accel_x_max_);

        double linear_output = current_velocity.linear.x + acceleration_cmd * dt;
        if (!std::isfinite(linear_output)) {
            linear_output = 0.0;
        }
        if (!reverse_mode) {
            linear_output = std::clamp(
                linear_output, min_v_, std::max(min_v_, desired_linear_vel * 1.5));
        } else {
            linear_output = std::clamp(
                linear_output, std::min(-min_v_, desired_linear_vel * 1.5), -min_v_);
        }
        if (dist <= xy_tolerance_) {
            linear_output = 0.0;
        }

        double desired_w = std::clamp(yaw_error / dt, -max_w_, max_w_);
        const double angular_error = desired_w - current_velocity.angular.z;
        straight_angular_integral_ += angular_error * dt;
        const double angular_derivative = (angular_error - straight_angular_error_prev_) / dt;
        straight_angular_error_prev_ = angular_error;

        double w_inc = straight_angular_kp_ * angular_error +
                       straight_angular_ki_ * straight_angular_integral_ +
                       straight_angular_kd_ * angular_derivative;
        w_inc = std::clamp(w_inc, -straight_max_w_inc_, straight_max_w_inc_);

        double angular_output = current_velocity.angular.z + w_inc;
        angular_output = std::clamp(angular_output, -max_w_, max_w_);
        if (!std::isfinite(angular_output)) {
            angular_output = 0.0;
        }

        const double yaw_error_abs = std::fabs(yaw_error);
        if (yaw_error_abs < M_PI / 8.0) {
            angular_output *= 0.25;
        } else if (yaw_error_abs < M_PI_4) {
            const double factor =
                1.0 - (M_PI_4 - yaw_error_abs) / (M_PI_4 - M_PI / 8.0) * 0.25;
            angular_output *= factor;
        }

        cmd_vel.linear.x = linear_output;
        cmd_vel.angular.z = angular_output;
        return cmd_vel;
    }

    void controlLoop()
    {
        if (!active_goal_handle_) {
            return;
        }

        if (active_goal_handle_->is_canceling()) {
            startCancellation();
        }

        if (is_cancelling_) {
            publishFeedback("CANCELLING");
            geometry_msgs::msg::Twist cmd_vel;
            const double dt = 1.0 / control_frequency_;
            const double linear_decel = accel_x_max_ * dt;
            const double angular_decel = rotate_to_heading_max_angular_accel_ * dt;

            if (pre_cmd_vel_.linear.x > 0.0) {
                cmd_vel.linear.x = std::max(0.0, pre_cmd_vel_.linear.x - linear_decel);
            } else if (pre_cmd_vel_.linear.x < 0.0) {
                cmd_vel.linear.x = std::min(0.0, pre_cmd_vel_.linear.x + linear_decel);
            }

            if (pre_cmd_vel_.angular.z > 0.0) {
                cmd_vel.angular.z = std::max(0.0, pre_cmd_vel_.angular.z - angular_decel);
            } else if (pre_cmd_vel_.angular.z < 0.0) {
                cmd_vel.angular.z = std::min(0.0, pre_cmd_vel_.angular.z + angular_decel);
            }

            if (std::fabs(cmd_vel.linear.x) < 0.01 && std::fabs(cmd_vel.angular.z) < 0.01) {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                cmd_vel_pub_->publish(cmd_vel);
                RCLCPP_INFO(this->get_logger(), "Precise control cancel completed, robot stopped");
                finishCanceled();
                return;
            }

            cmd_vel_pub_->publish(cmd_vel);
            return;
        }

        if (is_goal_ && stop_count_ >= 5) {
            publishFeedback("SUCCEEDED");
            finishSucceeded();
            return;
        }

        if (!get_map_pose_) {
            publishFeedback("IDLE");
            return;
        }

        publishFeedback("CONTROL");

        geometry_msgs::msg::PoseStamped target_pose_in_base;
        try {
            const geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "base_link", "map", tf2::TimePointZero);
            tf2::doTransform(target_pose_odom_, target_pose_in_base, transform);
        } catch (tf2::TransformException &) {
            return;
        }

        const double dist = std::hypot(
            target_pose_in_base.pose.position.x, target_pose_in_base.pose.position.y);
        const double x_error = target_pose_in_base.pose.position.x;
        const double yaw_error = tf2::getYaw(target_pose_in_base.pose.orientation);
        const double dt = 1.0 / control_frequency_;
        const bool is_reverse_mode =
            target_pose_in_base.pose.position.x < 0.0 &&
            std::fabs(target_pose_in_base.pose.position.y) < 0.5;

        geometry_msgs::msg::Twist cmd_vel;
        if (goal_mode_ == GoalMode::ROTATE && std::fabs(dist) < xy_tolerance_) {
            cmd_vel = computeRotateToHeadingCommand(yaw_error, dt);
        } else {
            cmd_vel = computeStraightPidCommand(
                target_pose_in_base.pose, pre_cmd_vel_, dt, is_reverse_mode);
        }

        filtered_cmd_vel_.linear.x =
            filter_alpha_ * cmd_vel.linear.x + (1.0 - filter_alpha_) * filtered_cmd_vel_.linear.x;
        filtered_cmd_vel_.angular.z =
            filter_alpha_ * cmd_vel.angular.z + (1.0 - filter_alpha_) * filtered_cmd_vel_.angular.z;

        if (goal_mode_ == GoalMode::STRAIGHT) {
            if (std::fabs(x_error) < xy_tolerance_ && std::fabs(yaw_error) < yaw_tolerance_) {
                is_goal_ = true;
            }
        } else if (std::fabs(dist) < xy_tolerance_ && std::fabs(yaw_error) < yaw_tolerance_) {
            is_goal_ = true;
        }

        if (is_goal_) {
            resetPidStates();
            filtered_cmd_vel_.linear.x = 0.0;
            filtered_cmd_vel_.angular.z = 0.0;
            stop_count_++;

            cmd_vel_pub_->publish(filtered_cmd_vel_);
            if (stop_count_ == 1) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Precise control goal reached with final error: x_error=%.4f m, dist=%.4f m, yaw=%.4f rad",
                    x_error, dist, yaw_error);
            }
        } else {
            stop_count_ = 0;
            cmd_vel_pub_->publish(filtered_cmd_vel_);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MobileControlActionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
