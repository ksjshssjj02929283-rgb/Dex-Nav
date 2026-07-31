// ----------------------------------------------------------------------------
// Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iomanip>
#include <memory>
#include <nav_interfaces/action/precise_control.hpp>
#include <nav_interfaces/msg/robot_trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "mobile_perception/mobile_control/goal_checker.hpp"
#include "mobile_perception/mobile_control/rotation_controller.hpp"
#include "mobile_perception/mobile_control/short_trajectory_builder.hpp"
#include "mobile_perception/mobile_control/speed_profile_planner.hpp"
#include "mobile_perception/mobile_control/straight_controller.hpp"

using namespace std::chrono_literals;

namespace {

constexpr double kPi = 3.14159265358979323846;

double normalizedAngle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

std::string makePlanId(const rclcpp_action::GoalUUID& goal_id) {
    std::ostringstream stream;
    stream << "mobile_control_";
    for (const auto byte : goal_id) {
        stream << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    }
    return stream.str();
}

}  // namespace

class MobileControlActionNode : public rclcpp::Node {
public:
    using PreciseControl = nav_interfaces::action::PreciseControl;
    using GoalHandlePreciseControl = rclcpp_action::ServerGoalHandle<PreciseControl>;
    using GoalMode = mobile_perception::mobile_control::MotionMode;
    using GoalCheckMode = mobile_perception::mobile_control::GoalCheckMode;
    using GoalControlState = mobile_perception::mobile_control::GoalControlState;
    using SpeedProfilePlanner = mobile_perception::mobile_control::SpeedProfilePlanner;

    MobileControlActionNode()
        : Node("mobile_control_action_node"),
          speed_planner_loader_("mobile_perception", "mobile_perception::mobile_control::SpeedProfilePlanner") {
        declareParameters();
        loadParameters();

        const std::string planner_plugin = this->get_parameter("speed_profile_plugin").as_string();
        try {
            speed_planner_ = speed_planner_loader_.createSharedInstance(planner_plugin);
        } catch (const pluginlib::PluginlibException& error) {
            throw std::runtime_error("Cannot load speed profile plugin '" + planner_plugin + "': " + error.what());
        }

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("control/cmd_vel", 10);
        const auto plan_qos = rclcpp::QoS(1).reliable().transient_local();
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("control/planned_path", plan_qos);
        trajectory_pub_ =
            this->create_publisher<nav_interfaces::msg::RobotTrajectory>("control/planned_trajectory", plan_qos);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "feedback/odom", 10, std::bind(&MobileControlActionNode::odomCallback, this, std::placeholders::_1));

        action_server_ = rclcpp_action::create_server<PreciseControl>(
            this, "control/precise_control",
            std::bind(&MobileControlActionNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MobileControlActionNode::handleCancel, this, std::placeholders::_1),
            std::bind(&MobileControlActionNode::handleAccepted, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / control_frequency_),
                                         std::bind(&MobileControlActionNode::controlLoop, this));

        const auto& checker_config = goal_checker_->config();
        RCLCPP_INFO(this->get_logger(),
                    "精控链路就绪：S曲线插件=%s，控制频率=%.1fHz，GoalChecker="
                    "(translation: longitudinal<=%.4fm + lateral<=%.4fm + yaw<=%.4frad; "
                    "rotation: yaw only; both: |v|<=%.4fm/s + |w|<=%.4frad/s + confirm=%.2fs), "
                    "geometry_frame=map, tracking_error_frame=base_link(latest TF), "
                    "velocity_feedback=feedback/odom.twist, "
                    "straight_control=S(v,a)+position_pid+speed_pid, straight_stop=(%.4fm->%.4fm/s), "
                    "rotation_min_w=%.4frad/s, "
                    "effective_terminal_w=%.4frad/s",
                    planner_plugin.c_str(), control_frequency_, checker_config.longitudinal_tolerance,
                    checker_config.lateral_tolerance, checker_config.yaw_tolerance,
                    checker_config.stopped_linear_velocity, checker_config.stopped_angular_velocity,
                    checker_config.confirm_duration, this->get_parameter("straight.stop_distance").as_double(),
                    this->get_parameter("straight.stop_speed").as_double(),
                    rotation_controller_config_.min_angular_velocity,
                    rotation_controller_config_.terminal_angular_velocity);
    }

private:
    rclcpp_action::Server<PreciseControl>::SharedPtr action_server_;
    std::shared_ptr<GoalHandlePreciseControl> active_goal_handle_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_interfaces::msg::RobotTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    pluginlib::ClassLoader<SpeedProfilePlanner> speed_planner_loader_;
    std::shared_ptr<SpeedProfilePlanner> speed_planner_;
    std::unique_ptr<mobile_perception::mobile_control::StraightController> straight_controller_;
    std::unique_ptr<mobile_perception::mobile_control::RotationController> rotation_controller_;
    std::unique_ptr<mobile_perception::mobile_control::PreciseGoalChecker> goal_checker_;

    geometry_msgs::msg::PoseStamped target_pose_map_;
    geometry_msgs::msg::Twist measured_velocity_;
    mobile_perception::mobile_control::ShortTrajectory active_trajectory_;
    GoalMode goal_mode_{GoalMode::Straight};
    bool reverse_{false};
    bool clockwise_{false};
    bool is_cancelling_{false};
    bool has_odom_{false};
    std::chrono::steady_clock::time_point last_odom_time_{};
    std::chrono::steady_clock::time_point goal_started_time_{};
    double initial_motion_{0.0};
    double continuous_rotation_yaw_error_{0.0};
    double trajectory_progress_{0.0};
    double last_longitudinal_error_{0.0};
    double last_lateral_error_{0.0};
    double last_yaw_error_{0.0};
    double last_goal_stable_duration_{0.0};
    bool goal_pose_reached_logged_{false};

    double control_frequency_{20.0};
    double odom_timeout_{0.5};
    double execution_timeout_{30.0};
    double goal_mode_rotate_xy_threshold_{0.05};
    double straight_max_initial_heading_error_{5.0 * kPi / 180.0};
    double trajectory_preview_time_{0.05};
    int64_t control_status_log_period_ms_{500};
    mobile_perception::mobile_control::SpeedProfileRequest linear_profile_request_;
    mobile_perception::mobile_control::SpeedProfileRequest angular_profile_request_;
    mobile_perception::mobile_control::TerminalSlowdownConstraint linear_terminal_slowdown_;
    mobile_perception::mobile_control::TerminalSlowdownConstraint angular_terminal_slowdown_;
    mobile_perception::mobile_control::RotationControllerConfig rotation_controller_config_;

    void declarePidParameters(
        const std::string& prefix, double default_kp, double default_ki, double default_kd, double default_saturation) {
        this->declare_parameter(prefix + ".integrator_enable", true);
        this->declare_parameter(prefix + ".integrator_saturation_level", default_saturation);
        this->declare_parameter(prefix + ".kp", default_kp);
        this->declare_parameter(prefix + ".ki", default_ki);
        this->declare_parameter(prefix + ".kd", default_kd);
    }

    void declareParameters() {
        this->declare_parameter("control_frequency", 20.0);
        this->declare_parameter("odom_timeout", 0.5);
        this->declare_parameter("execution_timeout", 30.0);
        this->declare_parameter("goal_mode_rotate_xy_threshold", 0.05);
        this->declare_parameter("trajectory_tracking.preview_time", 0.05);
        this->declare_parameter("logging.control_status_period", 0.5);
        this->declare_parameter("speed_profile_plugin", "mobile_perception::mobile_control::SCurveSpeedProfilePlanner");

        this->declare_parameter("s_curve.sample_period", 0.05);
        this->declare_parameter("s_curve.max_duration", 30.0);
        this->declare_parameter("s_curve.linear.max_velocity", 0.35);
        this->declare_parameter("s_curve.linear.max_acceleration", 0.5);
        this->declare_parameter("s_curve.linear.max_deceleration", 0.5);
        this->declare_parameter("s_curve.linear.max_jerk", 1.0);
        this->declare_parameter("s_curve.angular.max_velocity", 0.75);
        this->declare_parameter("s_curve.angular.max_acceleration", 1.0);
        this->declare_parameter("s_curve.angular.max_deceleration", 1.0);
        this->declare_parameter("s_curve.angular.max_jerk", 2.0);
        this->declare_parameter("goal_checker.longitudinal_tolerance", 0.005);
        this->declare_parameter("goal_checker.lateral_tolerance", 0.005);
        this->declare_parameter("goal_checker.yaw_tolerance", 0.008);
        this->declare_parameter("goal_checker.stopped_linear_velocity", 0.005);
        this->declare_parameter("goal_checker.stopped_angular_velocity", 0.01);
        this->declare_parameter("goal_checker.confirm_duration", 0.2);

        this->declare_parameter("straight.max_angular_velocity", 1.0);
        this->declare_parameter("straight.max_angular_acceleration", 1.57);
        this->declare_parameter("straight.max_initial_heading_error", 5.0 * kPi / 180.0);
        this->declare_parameter("straight.stop_distance", 0.05);
        this->declare_parameter("straight.stop_speed", 0.03);
        declarePidParameters("straight.position_pid", 1.5, 0.0, 0.0, 0.1);
        declarePidParameters("straight.speed_pid", 1.0, 0.01, 0.10, 0.3);
        declarePidParameters("straight.lateral_pid", 1.0, 0.01, 0.10, 0.3);
        declarePidParameters("straight.heading_pid", 1.0, 0.01, 0.10, 0.3);
        this->declare_parameter("rotation.yaw_kp", 2.0);
        this->declare_parameter("rotation.min_angular_velocity", 0.02);
        this->declare_parameter("rotation.terminal_slowdown_angle", 0.30);
        this->declare_parameter("rotation.terminal_angular_velocity", 0.01);
        declarePidParameters("rotation.angular_velocity_pid", 1.0, 0.01, 0.10, 0.5);
    }

    mobile_perception::mobile_control::PidConfig loadPidConfig(const std::string& prefix) {
        mobile_perception::mobile_control::PidConfig config;
        config.integrator_enable = this->get_parameter(prefix + ".integrator_enable").as_bool();
        config.integrator_saturation_level = this->get_parameter(prefix + ".integrator_saturation_level").as_double();
        config.kp = this->get_parameter(prefix + ".kp").as_double();
        config.ki = this->get_parameter(prefix + ".ki").as_double();
        config.kd = this->get_parameter(prefix + ".kd").as_double();
        return config;
    }

    void loadParameters() {
        control_frequency_ = std::max(1.0, this->get_parameter("control_frequency").as_double());
        odom_timeout_ = std::max(0.01, this->get_parameter("odom_timeout").as_double());
        execution_timeout_ = std::max(0.1, this->get_parameter("execution_timeout").as_double());
        goal_mode_rotate_xy_threshold_ =
            std::max(0.0, this->get_parameter("goal_mode_rotate_xy_threshold").as_double());
        straight_max_initial_heading_error_ =
            std::max(0.0, this->get_parameter("straight.max_initial_heading_error").as_double());
        trajectory_preview_time_ = std::max(0.0, this->get_parameter("trajectory_tracking.preview_time").as_double());
        control_status_log_period_ms_ = static_cast<int64_t>(
            1000.0 * std::max(0.1, this->get_parameter("logging.control_status_period").as_double()));

        const double sample_period = this->get_parameter("s_curve.sample_period").as_double();
        const double max_duration = this->get_parameter("s_curve.max_duration").as_double();
        linear_profile_request_.sample_period = sample_period;
        linear_profile_request_.max_duration = max_duration;
        linear_profile_request_.max_velocity = this->get_parameter("s_curve.linear.max_velocity").as_double();
        linear_profile_request_.max_acceleration = this->get_parameter("s_curve.linear.max_acceleration").as_double();
        linear_profile_request_.max_deceleration = this->get_parameter("s_curve.linear.max_deceleration").as_double();
        linear_profile_request_.max_jerk = this->get_parameter("s_curve.linear.max_jerk").as_double();
        angular_profile_request_.sample_period = sample_period;
        angular_profile_request_.max_duration = max_duration;
        angular_profile_request_.max_velocity = this->get_parameter("s_curve.angular.max_velocity").as_double();
        angular_profile_request_.max_acceleration = this->get_parameter("s_curve.angular.max_acceleration").as_double();
        angular_profile_request_.max_deceleration = this->get_parameter("s_curve.angular.max_deceleration").as_double();
        angular_profile_request_.max_jerk = this->get_parameter("s_curve.angular.max_jerk").as_double();

        mobile_perception::mobile_control::GoalCheckerConfig checker_config;
        checker_config.longitudinal_tolerance = this->get_parameter("goal_checker.longitudinal_tolerance").as_double();
        checker_config.lateral_tolerance = this->get_parameter("goal_checker.lateral_tolerance").as_double();
        checker_config.yaw_tolerance = this->get_parameter("goal_checker.yaw_tolerance").as_double();
        checker_config.stopped_linear_velocity =
            this->get_parameter("goal_checker.stopped_linear_velocity").as_double();
        checker_config.stopped_angular_velocity =
            this->get_parameter("goal_checker.stopped_angular_velocity").as_double();
        checker_config.confirm_duration = this->get_parameter("goal_checker.confirm_duration").as_double();
        goal_checker_ = std::make_unique<mobile_perception::mobile_control::PreciseGoalChecker>(checker_config);

        mobile_perception::mobile_control::StraightControllerConfig straight_config;
        straight_config.max_linear_velocity = linear_profile_request_.max_velocity;
        straight_config.max_angular_velocity = this->get_parameter("straight.max_angular_velocity").as_double();
        straight_config.max_linear_acceleration = linear_profile_request_.max_acceleration;
        straight_config.max_linear_deceleration = linear_profile_request_.max_deceleration;
        straight_config.max_angular_acceleration = this->get_parameter("straight.max_angular_acceleration").as_double();
        linear_terminal_slowdown_.motion = std::max(0.0, this->get_parameter("straight.stop_distance").as_double());
        linear_terminal_slowdown_.velocity = std::clamp(this->get_parameter("straight.stop_speed").as_double(), 0.0,
                                                        linear_profile_request_.max_velocity);
        straight_config.terminal_slowdown_distance = linear_terminal_slowdown_.motion;
        straight_config.terminal_linear_velocity = linear_terminal_slowdown_.velocity;
        straight_config.position_pid = loadPidConfig("straight.position_pid");
        straight_config.speed_pid = loadPidConfig("straight.speed_pid");
        straight_config.lateral_pid = loadPidConfig("straight.lateral_pid");
        straight_config.heading_pid = loadPidConfig("straight.heading_pid");
        straight_controller_ = std::make_unique<mobile_perception::mobile_control::StraightController>(straight_config);

        rotation_controller_config_.yaw_kp = this->get_parameter("rotation.yaw_kp").as_double();
        rotation_controller_config_.yaw_tolerance = checker_config.yaw_tolerance;
        rotation_controller_config_.min_angular_velocity =
            this->get_parameter("rotation.min_angular_velocity").as_double();
        rotation_controller_config_.terminal_slowdown_angle =
            this->get_parameter("rotation.terminal_slowdown_angle").as_double();
        rotation_controller_config_.terminal_angular_velocity =
            this->get_parameter("rotation.terminal_angular_velocity").as_double();
        rotation_controller_config_.max_angular_velocity = angular_profile_request_.max_velocity;
        rotation_controller_config_.min_angular_velocity = std::clamp(
            rotation_controller_config_.min_angular_velocity, 0.0, rotation_controller_config_.max_angular_velocity);
        if (rotation_controller_config_.terminal_angular_velocity < rotation_controller_config_.min_angular_velocity) {
            RCLCPP_WARN(this->get_logger(),
                        "rotation.terminal_angular_velocity=%.4frad/s is below the deadband-compensated "
                        "rotation.min_angular_velocity=%.4frad/s; using the executable minimum in the terminal zone",
                        rotation_controller_config_.terminal_angular_velocity,
                        rotation_controller_config_.min_angular_velocity);
            rotation_controller_config_.terminal_angular_velocity = rotation_controller_config_.min_angular_velocity;
        }
        rotation_controller_config_.terminal_slowdown_angle =
            std::max(rotation_controller_config_.yaw_tolerance, rotation_controller_config_.terminal_slowdown_angle);
        rotation_controller_config_.terminal_angular_velocity =
            std::clamp(rotation_controller_config_.terminal_angular_velocity, 0.0,
                       rotation_controller_config_.max_angular_velocity);
        angular_terminal_slowdown_.motion = rotation_controller_config_.terminal_slowdown_angle;
        angular_terminal_slowdown_.velocity = rotation_controller_config_.terminal_angular_velocity;
        rotation_controller_config_.max_angular_acceleration =
            std::min(angular_profile_request_.max_acceleration, angular_profile_request_.max_deceleration);
        rotation_controller_config_.angular_velocity_pid = loadPidConfig("rotation.angular_velocity_pid");
        rotation_controller_ =
            std::make_unique<mobile_perception::mobile_control::RotationController>(rotation_controller_config_);
    }

    void resetControllers() {
        straight_controller_->reset();
        rotation_controller_->reset();
        goal_checker_->reset();
    }

    void resetExecutionState() {
        is_cancelling_ = false;
        goal_mode_ = GoalMode::Straight;
        reverse_ = false;
        clockwise_ = false;
        initial_motion_ = 0.0;
        continuous_rotation_yaw_error_ = 0.0;
        trajectory_progress_ = 0.0;
        last_longitudinal_error_ = 0.0;
        last_lateral_error_ = 0.0;
        last_yaw_error_ = 0.0;
        last_goal_stable_duration_ = 0.0;
        goal_pose_reached_logged_ = false;
        active_trajectory_.path = nav_msgs::msg::Path();
        active_trajectory_.trajectory = nav_interfaces::msg::RobotTrajectory();
        active_trajectory_.motion_profile = mobile_perception::mobile_control::SpeedProfile{};
        active_trajectory_.mode = GoalMode::Straight;
        active_trajectory_.direction = 1.0;
        active_trajectory_.total_motion = 0.0;
        resetControllers();
    }

    double goalElapsedSeconds() const {
        if (!active_goal_handle_) {
            return -1.0;
        }
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - goal_started_time_).count();
    }

    bool odomIsFresh() const {
        return has_odom_ && std::chrono::duration<double>(std::chrono::steady_clock::now() - last_odom_time_).count() <=
                                odom_timeout_;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message) {
        measured_velocity_ = message->twist.twist;
        has_odom_ = true;
        last_odom_time_ = std::chrono::steady_clock::now();
    }

    bool transformTargetToMap(const geometry_msgs::msg::PoseStamped& target,
                              geometry_msgs::msg::PoseStamped& target_map,
                              std::string& error) {
        try {
            target_map = target.header.frame_id == "map"
                             ? target
                             : tf_buffer_->transform(target, "map", tf2::durationFromSec(0.1));
            target_map.header.frame_id = "map";
            // The goal is now a fixed map coordinate. All later base-relative
            // queries must use the latest robot transform, not the original
            // goal timestamp (which eventually leaves the TF cache).
            target_map.header.stamp = rclcpp::Time(0);
        } catch (const tf2::TransformException& exception) {
            error = exception.what();
            return false;
        }
        return true;
    }

    bool currentPoseInMap(geometry_msgs::msg::PoseStamped& pose, std::string& error) {
        try {
            const auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            pose.header = transform.header;
            pose.header.frame_id = "map";
            pose.pose.position.x = transform.transform.translation.x;
            pose.pose.position.y = transform.transform.translation.y;
            pose.pose.position.z = transform.transform.translation.z;
            pose.pose.orientation = transform.transform.rotation;
        } catch (const tf2::TransformException& exception) {
            error = exception.what();
            return false;
        }
        return true;
    }

    bool poseInBase(const geometry_msgs::msg::PoseStamped& pose_map,
                    geometry_msgs::msg::PoseStamped& pose_base,
                    std::string& error) {
        try {
            auto pose_at_latest = pose_map;
            pose_at_latest.header.stamp = rclcpp::Time(0);
            pose_base = tf_buffer_->transform(pose_at_latest, "base_link", tf2::durationFromSec(0.1));
        } catch (const tf2::TransformException& exception) {
            error = exception.what();
            return false;
        }
        return true;
    }

    bool makePlan(const geometry_msgs::msg::PoseStamped& start_pose,
                  const geometry_msgs::msg::PoseStamped& target_base,
                  const std::string& plan_id,
                  std::string& error) {
        const double distance = std::hypot(target_base.pose.position.x, target_base.pose.position.y);
        const double yaw_error = normalizedAngle(tf2::getYaw(target_base.pose.orientation));
        const auto& checker_config = goal_checker_->config();
        goal_mode_ = distance <= goal_mode_rotate_xy_threshold_ && std::fabs(yaw_error) > checker_config.yaw_tolerance
                         ? GoalMode::Rotation
                         : GoalMode::Straight;

        reverse_ = goal_mode_ == GoalMode::Straight && target_base.pose.position.x < 0.0;
        clockwise_ = goal_mode_ == GoalMode::Rotation && yaw_error < 0.0;
        initial_motion_ = goal_mode_ == GoalMode::Straight ? distance : std::fabs(yaw_error);
        const char* mode_name = goal_mode_ == GoalMode::Straight ? (reverse_ ? "REVERSE" : "STRAIGHT")
                                                                 : (clockwise_ ? "ROTATE_CW" : "ROTATE_CCW");
        const double direction = (goal_mode_ == GoalMode::Straight ? reverse_ : clockwise_) ? -1.0 : 1.0;
        continuous_rotation_yaw_error_ = goal_mode_ == GoalMode::Rotation ? direction * initial_motion_ : 0.0;
        const double start_yaw = tf2::getYaw(start_pose.pose.orientation);
        const double goal_yaw = tf2::getYaw(target_pose_map_.pose.orientation);

        if (goal_mode_ == GoalMode::Straight) {
            const double delta_x = target_pose_map_.pose.position.x - start_pose.pose.position.x;
            const double delta_y = target_pose_map_.pose.position.y - start_pose.pose.position.y;
            const double path_travel_yaw = distance > 1e-9 ? std::atan2(delta_y, delta_x) : start_yaw;
            const double straight_path_heading_map = normalizedAngle(path_travel_yaw + (reverse_ ? kPi : 0.0));
            const auto heading_check = mobile_perception::mobile_control::checkStraightHeadingAlignment(
                start_yaw, goal_yaw, straight_path_heading_map, straight_max_initial_heading_error_);
            const double radians_to_degrees = 180.0 / kPi;
            if (!heading_check.accepted) {
                RCLCPP_ERROR(this->get_logger(),
                             "[精控 2/7 控制前角度检查失败] plan_id=%s, mode=%s, "
                             "goal_to_robot=%.3fdeg, path_to_robot=%.3fdeg, goal_to_path=%.3fdeg, threshold=%.3fdeg",
                             plan_id.c_str(), mode_name, heading_check.goal_to_robot_error * radians_to_degrees,
                             heading_check.path_to_robot_error * radians_to_degrees,
                             heading_check.goal_to_path_error * radians_to_degrees,
                             straight_max_initial_heading_error_ * radians_to_degrees);
                std::ostringstream stream;
                stream << "ANGLE_CHECK_FAILED: straight heading misalignment exceeds "
                       << straight_max_initial_heading_error_ * radians_to_degrees << " deg";
                error = stream.str();
                return false;
            }
            RCLCPP_INFO(this->get_logger(),
                        "[精控 2/7 控制前角度检查通过] plan_id=%s, mode=%s, "
                        "goal_to_robot=%.3fdeg, path_to_robot=%.3fdeg, goal_to_path=%.3fdeg, threshold=%.3fdeg",
                        plan_id.c_str(), mode_name, heading_check.goal_to_robot_error * radians_to_degrees,
                        heading_check.path_to_robot_error * radians_to_degrees,
                        heading_check.goal_to_path_error * radians_to_degrees,
                        straight_max_initial_heading_error_ * radians_to_degrees);
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "[精控 2/7 控制前角度检查] plan_id=%s, mode=%s, skipped=true, reason=rotation_mode",
                        plan_id.c_str(), mode_name);
        }

        RCLCPP_INFO(this->get_logger(),
                    "[精控 2/7 路径规划完成] plan_id=%s, mode=%s, frame=map, "
                    "start=(%.4f, %.4f, %.4f), goal=(%.4f, %.4f, %.4f), path_points=2, motion=%.4f",
                    plan_id.c_str(), mode_name, start_pose.pose.position.x, start_pose.pose.position.y, start_yaw,
                    target_pose_map_.pose.position.x, target_pose_map_.pose.position.y, goal_yaw, initial_motion_);

        mobile_perception::mobile_control::SpeedProfile profile;
        if (goal_mode_ == GoalMode::Rotation) {
            auto request = angular_profile_request_;
            const double speed_along_motion = measured_velocity_.angular.z * direction;
            if (speed_along_motion < -checker_config.stopped_angular_velocity) {
                error = "robot is rotating opposite to the requested one-shot trajectory";
                return false;
            }
            request.distance = initial_motion_;
            request.initial_velocity = std::max(0.0, speed_along_motion);
            request.initial_acceleration = 0.0;
            request.target_velocity = 0.0;
            if (!mobile_perception::mobile_control::planSpeedProfileWithTerminalSlowdown(
                    *speed_planner_, request, angular_terminal_slowdown_, profile, error)) {
                return false;
            }

            double peak_velocity = 0.0;
            double peak_acceleration = 0.0;
            double peak_deceleration = 0.0;
            double peak_jerk = 0.0;
            for (const auto& point : profile.points) {
                peak_velocity = std::max(peak_velocity, std::fabs(point.velocity));
                peak_acceleration = std::max(peak_acceleration, point.acceleration);
                peak_deceleration = std::max(peak_deceleration, -point.acceleration);
                peak_jerk = std::max(peak_jerk, std::fabs(point.jerk));
            }
            RCLCPP_INFO(
                this->get_logger(),
                "[精控 3/7 S角速度规划完成] plan_id=%s, plugin=%s, points=%zu, duration=%.3fs, "
                "peak_w=%.4f, peak_accel=%.4f, peak_decel=%.4f, peak_jerk=%.4f, "
                "controller=ANGLE_OUTER+ANGULAR_VELOCITY_PID, yaw_kp=%.3f, "
                "pid=(kp=%.3f,ki=%.3f,kd=%.3f,i_sat=%.3f), min_w=%.3f, "
                "slowdown_point=%.3frad, terminal_zone=%.3frad, terminal_w=%.3frad/s, stop_yaw=%.3frad, "
                "max_w=%.3f, max_alpha=%.3f",
                plan_id.c_str(), this->get_parameter("speed_profile_plugin").as_string().c_str(), profile.points.size(),
                profile.duration, peak_velocity, peak_acceleration, peak_deceleration, peak_jerk,
                rotation_controller_config_.yaw_kp, rotation_controller_config_.angular_velocity_pid.kp,
                rotation_controller_config_.angular_velocity_pid.ki,
                rotation_controller_config_.angular_velocity_pid.kd,
                rotation_controller_config_.angular_velocity_pid.integrator_saturation_level,
                rotation_controller_config_.min_angular_velocity,
                std::max(0.0, initial_motion_ - angular_terminal_slowdown_.motion), angular_terminal_slowdown_.motion,
                angular_terminal_slowdown_.velocity, rotation_controller_config_.yaw_tolerance,
                rotation_controller_config_.max_angular_velocity, rotation_controller_config_.max_angular_acceleration);
        } else {
            const bool already_inside_goal =
                std::fabs(target_base.pose.position.x) <= checker_config.longitudinal_tolerance &&
                std::fabs(target_base.pose.position.y) <= checker_config.lateral_tolerance &&
                std::fabs(yaw_error) <= checker_config.yaw_tolerance;
            if (already_inside_goal) {
                profile.points.push_back(mobile_perception::mobile_control::SpeedProfilePoint{});
            } else {
                auto request = linear_profile_request_;
                const double direction = reverse_ ? -1.0 : 1.0;
                const double speed_along_motion = measured_velocity_.linear.x * direction;
                if (speed_along_motion < -checker_config.stopped_linear_velocity) {
                    error = "robot is moving opposite to the requested one-shot trajectory";
                    return false;
                }
                request.distance = initial_motion_;
                request.initial_velocity = std::max(0.0, speed_along_motion);
                request.initial_acceleration = 0.0;
                request.target_velocity = 0.0;
                if (!mobile_perception::mobile_control::planSpeedProfileWithTerminalSlowdown(
                        *speed_planner_, request, linear_terminal_slowdown_, profile, error)) {
                    return false;
                }
            }

            double peak_velocity = 0.0;
            double peak_acceleration = 0.0;
            double peak_deceleration = 0.0;
            double peak_jerk = 0.0;
            for (const auto& point : profile.points) {
                peak_velocity = std::max(peak_velocity, std::fabs(point.velocity));
                peak_acceleration = std::max(peak_acceleration, point.acceleration);
                peak_deceleration = std::max(peak_deceleration, -point.acceleration);
                peak_jerk = std::max(peak_jerk, std::fabs(point.jerk));
            }
            RCLCPP_INFO(this->get_logger(),
                        "[精控 3/7 S速度规划完成] plan_id=%s, plugin=%s, points=%zu, duration=%.3fs, "
                        "peak_v=%.4f, peak_accel=%.4f, peak_decel=%.4f, peak_j=%.4f, "
                        "slowdown_point=%.4fm, slowdown_distance=%.4fm, slowdown_speed=%.4fm/s, preview=%.3fs",
                        plan_id.c_str(), this->get_parameter("speed_profile_plugin").as_string().c_str(),
                        profile.points.size(), profile.duration, peak_velocity, peak_acceleration, peak_deceleration,
                        peak_jerk, std::max(0.0, initial_motion_ - linear_terminal_slowdown_.motion),
                        linear_terminal_slowdown_.motion, linear_terminal_slowdown_.velocity, trajectory_preview_time_);
        }

        active_trajectory_ = mobile_perception::mobile_control::ShortTrajectoryBuilder::build(
            start_pose, target_pose_map_, profile, goal_mode_, direction, plan_id);
        path_pub_->publish(active_trajectory_.path);
        trajectory_pub_->publish(active_trajectory_.trajectory);
        return true;
    }

    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& goal_id,
                                           std::shared_ptr<const PreciseControl::Goal> goal) {
        const std::string plan_id = makePlanId(goal_id);
        RCLCPP_INFO(this->get_logger(),
                    "[精控 1/7 收到Goal] plan_id=%s, frame=%s, stamp=%d.%09u, "
                    "target=(%.4f, %.4f, %.4f)",
                    plan_id.c_str(), goal->target_pose.header.frame_id.c_str(), goal->target_pose.header.stamp.sec,
                    goal->target_pose.header.stamp.nanosec, goal->target_pose.pose.position.x,
                    goal->target_pose.pose.position.y, tf2::getYaw(goal->target_pose.pose.orientation));
        if (active_goal_handle_) {
            RCLCPP_WARN(this->get_logger(),
                        "[精控 7/7 接口返回][目标拒绝] plan_id=%s, terminal=REJECTED, "
                        "reason=已有目标执行中, active_time=%.3fs",
                        plan_id.c_str(), goalElapsedSeconds());
            return rclcpp_action::GoalResponse::REJECT;
        }

        geometry_msgs::msg::PoseStamped target_map;
        std::string error;
        if (!transformTargetToMap(goal->target_pose, target_map, error)) {
            RCLCPP_WARN(this->get_logger(),
                        "[精控 7/7 接口返回][目标拒绝] plan_id=%s, terminal=REJECTED, "
                        "reason=目标无法转换到map, detail=%s",
                        plan_id.c_str(), error.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(), "[精控 1/7 Goal已接受] plan_id=%s", plan_id.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandlePreciseControl>) {
        RCLCPP_INFO(
            this->get_logger(), "[精控取消请求] plan_id=%s",
            active_trajectory_.trajectory.plan_id.empty() ? "pending" : active_trajectory_.trajectory.plan_id.c_str());
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandlePreciseControl> goal_handle) {
        const std::string plan_id = makePlanId(goal_handle->get_goal_id());
        if (!odomIsFresh()) {
            auto result = std::make_shared<PreciseControl::Result>();
            result->success = false;
            result->message = "Cannot plan: odometry is unavailable or stale";
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(),
                         "[精控 7/7 接口返回][规划失败] plan_id=%s, terminal=ABORTED, "
                         "success=false, reason=odom不可用或已超时",
                         plan_id.c_str());
            return;
        }

        geometry_msgs::msg::PoseStamped start_pose;
        geometry_msgs::msg::PoseStamped target_base;
        std::string error;
        if (!transformTargetToMap(goal_handle->get_goal()->target_pose, target_pose_map_, error) ||
            !currentPoseInMap(start_pose, error) || !poseInBase(target_pose_map_, target_base, error)) {
            auto result = std::make_shared<PreciseControl::Result>();
            result->success = false;
            result->message = "Cannot create one-shot path: " + error;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(),
                         "[精控 7/7 接口返回][规划失败] plan_id=%s, terminal=ABORTED, "
                         "success=false, reason=TF中断, detail=%s",
                         plan_id.c_str(), error.c_str());
            return;
        }

        resetExecutionState();
        if (!makePlan(start_pose, target_base, plan_id, error)) {
            auto result = std::make_shared<PreciseControl::Result>();
            result->success = false;
            result->message = "One-shot planning failed: " + error;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(),
                         "[精控 7/7 接口返回][规划失败] plan_id=%s, terminal=ABORTED, "
                         "success=false, reason=一次性规划失败, detail=%s",
                         plan_id.c_str(), error.c_str());
            return;
        }
        if (goal_mode_ == GoalMode::Rotation) {
            // Seed the rate limiter once at the goal boundary. During control,
            // odometry remains feedback only and never becomes command state.
            rotation_controller_->reset(measured_velocity_.angular.z);
        }

        active_goal_handle_ = goal_handle;
        goal_started_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(
            this->get_logger(), "[精控 4/7 控制开始] plan_id=%s, mode=%s, motion=%.4f, points=%zu, duration=%.3fs",
            active_trajectory_.trajectory.plan_id.c_str(),
            goal_mode_ == GoalMode::Straight ? (reverse_ ? "REVERSE" : "STRAIGHT")
                                             : (clockwise_ ? "ROTATE_CW" : "ROTATE_CCW"),
            initial_motion_, active_trajectory_.trajectory.points.size(), active_trajectory_.motion_profile.duration);
    }

    void publishFeedback(const std::string& status) {
        if (!active_goal_handle_) {
            return;
        }
        auto feedback = std::make_shared<PreciseControl::Feedback>();
        feedback->current_status = status;
        active_goal_handle_->publish_feedback(feedback);
    }

    void publishZeroVelocity() { cmd_vel_pub_->publish(geometry_msgs::msg::Twist{}); }

    void finishSucceeded() {
        if (!active_goal_handle_) {
            return;
        }
        publishZeroVelocity();
        auto result = std::make_shared<PreciseControl::Result>();
        result->success = true;
        result->message = "SUCCEEDED";
        const auto& config = goal_checker_->config();
        const bool position_required = goal_mode_ == GoalMode::Straight;
        const char* mode_name =
            position_required ? (reverse_ ? "REVERSE" : "STRAIGHT") : (clockwise_ ? "ROTATE_CW" : "ROTATE_CCW");
        if (position_required) {
            RCLCPP_INFO(this->get_logger(),
                        "[精控 6/7 最终到点精度] plan_id=%s, verdict=PASS, mode=%s, "
                        "checks=position+yaw+speed+dwell, longitudinal=%.4f/%.4fm, "
                        "lateral=%.4f/%.4fm, yaw=%.4f/%.4frad, "
                        "|v|=%.4f/%.4fm/s, |w|=%.4f/%.4frad/s, stable=%.3f/%.3fs",
                        active_trajectory_.trajectory.plan_id.c_str(), mode_name, last_longitudinal_error_,
                        config.longitudinal_tolerance, last_lateral_error_, config.lateral_tolerance, last_yaw_error_,
                        config.yaw_tolerance, std::fabs(measured_velocity_.linear.x), config.stopped_linear_velocity,
                        std::fabs(measured_velocity_.angular.z), config.stopped_angular_velocity,
                        last_goal_stable_duration_, config.confirm_duration);
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "[精控 6/7 最终到点精度] plan_id=%s, verdict=PASS, mode=%s, "
                        "checks=yaw+speed+dwell, longitudinal=%.4fm(diagnostic), "
                        "lateral=%.4fm(diagnostic), yaw=%.4f/%.4frad, "
                        "|v|=%.4f/%.4fm/s, |w|=%.4f/%.4frad/s, stable=%.3f/%.3fs",
                        active_trajectory_.trajectory.plan_id.c_str(), mode_name, last_longitudinal_error_,
                        last_lateral_error_, last_yaw_error_, config.yaw_tolerance,
                        std::fabs(measured_velocity_.linear.x), config.stopped_linear_velocity,
                        std::fabs(measured_velocity_.angular.z), config.stopped_angular_velocity,
                        last_goal_stable_duration_, config.confirm_duration);
        }
        RCLCPP_INFO(this->get_logger(),
                    "[精控 7/7 接口返回][成功] plan_id=%s, terminal=SUCCEEDED, success=true, "
                    "time=%.3fs, mode=%s, position_required=%s, "
                    "goal_longitudinal=%.4fm, goal_lateral=%.4fm, "
                    "yaw=%.4frad, v=%.4fm/s, w=%.4frad/s",
                    active_trajectory_.trajectory.plan_id.c_str(), goalElapsedSeconds(), mode_name,
                    position_required ? "true" : "false", last_longitudinal_error_, last_lateral_error_,
                    last_yaw_error_, measured_velocity_.linear.x, measured_velocity_.angular.z);
        active_goal_handle_->succeed(result);
        active_goal_handle_.reset();
        resetExecutionState();
    }

    void finishCanceled() {
        if (!active_goal_handle_) {
            return;
        }
        publishZeroVelocity();
        auto result = std::make_shared<PreciseControl::Result>();
        result->success = false;
        result->message = "CANCELLED";
        RCLCPP_INFO(this->get_logger(),
                    "[精控 7/7 接口返回][取消] plan_id=%s, terminal=CANCELED, success=false, "
                    "time=%.3fs, v=%.4fm/s, w=%.4frad/s",
                    active_trajectory_.trajectory.plan_id.c_str(), goalElapsedSeconds(), measured_velocity_.linear.x,
                    measured_velocity_.angular.z);
        active_goal_handle_->canceled(result);
        active_goal_handle_.reset();
        resetExecutionState();
    }

    void abortExecution(const std::string& message) {
        if (!active_goal_handle_) {
            return;
        }
        publishZeroVelocity();
        auto result = std::make_shared<PreciseControl::Result>();
        result->success = false;
        result->message = message;
        RCLCPP_ERROR(this->get_logger(),
                     "[精控中断][7/7 接口返回] plan_id=%s, terminal=ABORTED, success=false, "
                     "time=%.3fs, reason=%s",
                     active_trajectory_.trajectory.plan_id.c_str(), goalElapsedSeconds(), message.c_str());
        active_goal_handle_->abort(result);
        active_goal_handle_.reset();
        resetExecutionState();
    }

    void startCancellation() {
        if (!is_cancelling_) {
            is_cancelling_ = true;
            resetControllers();
            RCLCPP_INFO(this->get_logger(), "[精控取消] plan_id=%s, 开始按反馈速度平滑停车",
                        active_trajectory_.trajectory.plan_id.c_str());
        }
    }

    void runCancellation() {
        publishFeedback("CANCELLING");
        const double dt = 1.0 / control_frequency_;
        geometry_msgs::msg::Twist command;
        const auto decelerate = [dt](double velocity, double deceleration) {
            const double delta = deceleration * dt;
            return velocity > 0.0 ? std::max(0.0, velocity - delta) : std::min(0.0, velocity + delta);
        };
        command.linear.x = decelerate(measured_velocity_.linear.x, linear_profile_request_.max_deceleration);
        command.angular.z = decelerate(measured_velocity_.angular.z, angular_profile_request_.max_deceleration);

        const auto& config = goal_checker_->config();
        if (std::fabs(measured_velocity_.linear.x) <= config.stopped_linear_velocity &&
            std::fabs(measured_velocity_.angular.z) <= config.stopped_angular_velocity) {
            finishCanceled();
            return;
        }
        cmd_vel_pub_->publish(command);
    }

    void controlLoop() {
        if (!active_goal_handle_) {
            return;
        }
        if (active_goal_handle_->is_canceling()) {
            startCancellation();
        }
        if (is_cancelling_) {
            runCancellation();
            return;
        }

        if (!odomIsFresh()) {
            abortExecution("ODOM_INTERRUPTED: feedback/odom is unavailable or stale");
            return;
        }
        if (goalElapsedSeconds() > execution_timeout_) {
            abortExecution("EXECUTION_TIMEOUT: one-shot trajectory did not converge");
            return;
        }

        geometry_msgs::msg::PoseStamped target_base;
        std::string error;
        if (!poseInBase(target_pose_map_, target_base, error)) {
            abortExecution("TF_INTERRUPTED: " + error);
            return;
        }

        const double longitudinal_error = target_base.pose.position.x;
        const double lateral_error = target_base.pose.position.y;
        const double yaw_error = normalizedAngle(tf2::getYaw(target_base.pose.orientation));
        last_longitudinal_error_ = longitudinal_error;
        last_lateral_error_ = lateral_error;
        last_yaw_error_ = yaw_error;
        const double dt = 1.0 / control_frequency_;

        const auto goal_check_mode =
            goal_mode_ == GoalMode::Straight ? GoalCheckMode::Translation : GoalCheckMode::Rotation;
        const auto goal_state =
            goal_checker_->update({longitudinal_error, lateral_error, yaw_error, measured_velocity_.linear.x,
                                   measured_velocity_.angular.z, dt, goal_check_mode});
        last_goal_stable_duration_ = goal_state.stable_duration;
        const auto goal_control_state = mobile_perception::mobile_control::selectGoalControlState(goal_state);
        if (goal_state.pose_reached) {
            goal_pose_reached_logged_ = true;
            const char* phase = goal_control_state == GoalControlState::Succeeded
                                    ? "SUCCEEDED"
                                    : (goal_control_state == GoalControlState::HoldZero
                                           ? "CONFIRMING_STOP"
                                           : (goal_mode_ == GoalMode::Straight ? "DECELERATING_S_CURVE"
                                                                               : "ANGLE_SPEED_CASCADE_SETTLING"));
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), control_status_log_period_ms_,
                "[精控 6/7 到点检查] plan_id=%s, phase=%s, position_required=%s, "
                "position_ok=%s, yaw_ok=%s, pose_ok=true, speed_ok=%s, "
                "stable=%.3f/%.3fs, longitudinal=%.4f/%.4fm, lateral=%.4f/%.4fm, "
                "yaw=%.4f/%.4frad, |v|=%.4f/%.4fm/s, |w|=%.4f/%.4frad/s",
                active_trajectory_.trajectory.plan_id.c_str(), phase, goal_state.position_required ? "true" : "false",
                goal_state.position_reached ? "true" : "false", goal_state.yaw_reached ? "true" : "false",
                goal_state.speed_reached ? "true" : "false", goal_state.stable_duration,
                goal_checker_->config().confirm_duration, longitudinal_error,
                goal_checker_->config().longitudinal_tolerance, lateral_error,
                goal_checker_->config().lateral_tolerance, yaw_error, goal_checker_->config().yaw_tolerance,
                std::fabs(measured_velocity_.linear.x), goal_checker_->config().stopped_linear_velocity,
                std::fabs(measured_velocity_.angular.z), goal_checker_->config().stopped_angular_velocity);
            if (goal_control_state == GoalControlState::Succeeded) {
                finishSucceeded();
                return;
            }
            if (goal_control_state == GoalControlState::HoldZero) {
                publishZeroVelocity();
                publishFeedback("GOAL_CHECK:CONFIRMING_STOP");
                return;
            }
        }
        if (!goal_state.pose_reached && goal_pose_reached_logged_) {
            RCLCPP_WARN(this->get_logger(), "[精控 6/7 到点检查] plan_id=%s, GoalChecker离开容差窗口，继续闭环修正",
                        active_trajectory_.trajectory.plan_id.c_str());
            goal_pose_reached_logged_ = false;
        }

        geometry_msgs::msg::Twist command;
        if (goal_mode_ == GoalMode::Straight) {
            const double direction = reverse_ ? -1.0 : 1.0;
            trajectory_progress_ = mobile_perception::mobile_control::computeStraightProgress(
                initial_motion_, longitudinal_error, reverse_);
            const auto reference = mobile_perception::mobile_control::sampleSpeedProfileForTracking(
                active_trajectory_.motion_profile, trajectory_progress_, goalElapsedSeconds(),
                trajectory_preview_time_);

            geometry_msgs::msg::PoseStamped tracking_pose_map;
            if (!mobile_perception::mobile_control::ShortTrajectoryBuilder::samplePoseByMotion(
                    active_trajectory_, reference.position, tracking_pose_map)) {
                abortExecution("TRAJECTORY_INVALID: cannot sample straight tracking point");
                return;
            }
            geometry_msgs::msg::PoseStamped tracking_pose_base;
            if (!poseInBase(tracking_pose_map, tracking_pose_base, error)) {
                abortExecution("TF_INTERRUPTED: tracking point transform failed: " + error);
                return;
            }
            const double tracking_longitudinal_error = tracking_pose_base.pose.position.x;
            const double tracking_lateral_error = tracking_pose_base.pose.position.y;
            const double goal_yaw_error = yaw_error;
            const auto controller_command = straight_controller_->computeCommand(
                {direction * reference.velocity, tracking_longitudinal_error, tracking_lateral_error, goal_yaw_error,
                 measured_velocity_.linear.x, measured_velocity_.angular.z, dt, reverse_,
                 direction * reference.acceleration, longitudinal_error, true});
            command.linear.x = controller_command.linear_x;
            command.angular.z = controller_command.angular_z;
            publishFeedback(goal_state.pose_reached
                                ? "GOAL_CHECK:PID_SETTLING"
                                : (reverse_ ? "TRACKING:S_CURVE_PID_REVERSE" : "TRACKING:S_CURVE_PID_STRAIGHT"));

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), control_status_log_period_ms_,
                                 "[精控 5/7 控制中] plan_id=%s, phase=STRAIGHT_PID, progress=%.4f/%.4f, "
                                 "goal_longitudinal=%.4f, goal_lateral=%.4f, tracking_longitudinal=%.4f, "
                                 "tracking_lateral=%.4f, goal_yaw=%.4f, ref_position=%.4f, ref_t=%.3f, ref_v=%.4f, "
                                 "ref_a=%.4f, plan_v_limit=%.4f, stop_limited=%s, "
                                 "position_pid=%.4f, target_v=%.4f, speed_pid=%.4f, "
                                 "cmd_accel=%.4f, measured_v=%.4f, measured_w=%.4f, cmd_v=%.4f, cmd_w=%.4f",
                                 active_trajectory_.trajectory.plan_id.c_str(), trajectory_progress_, initial_motion_,
                                 longitudinal_error, lateral_error, tracking_longitudinal_error, tracking_lateral_error,
                                 yaw_error, reference.position, reference.relative_time, direction * reference.velocity,
                                 direction * reference.acceleration, controller_command.planned_linear_velocity_limit,
                                 controller_command.terminal_velocity_applied ? "true" : "false",
                                 controller_command.position_pid_output, controller_command.target_linear_velocity,
                                 controller_command.speed_pid_output, controller_command.linear_acceleration_command,
                                 measured_velocity_.linear.x, measured_velocity_.angular.z, command.linear.x,
                                 command.angular.z);
        } else {
            const double direction = clockwise_ ? -1.0 : 1.0;
            continuous_rotation_yaw_error_ =
                mobile_perception::mobile_control::unwrapRotationError(yaw_error, continuous_rotation_yaw_error_);
            const double rotation_progress =
                std::clamp(initial_motion_ - direction * continuous_rotation_yaw_error_, 0.0, initial_motion_);
            const auto reference = mobile_perception::mobile_control::sampleSpeedProfileForTracking(
                active_trajectory_.motion_profile, rotation_progress, goalElapsedSeconds(), trajectory_preview_time_);
            const double tracking_yaw_error =
                normalizedAngle(continuous_rotation_yaw_error_ - direction * (initial_motion_ - reference.position));
            const auto controller_command = rotation_controller_->computeCommand(
                {direction * reference.velocity, direction * reference.acceleration, tracking_yaw_error,
                 continuous_rotation_yaw_error_, measured_velocity_.angular.z, dt});
            command.angular.z = controller_command.angular_z;
            if (command.angular.z < 0.0) {
                publishFeedback("TRACKING:S_CURVE_ANGLE_SPEED_CASCADE_CW");
            } else if (command.angular.z > 0.0) {
                publishFeedback("TRACKING:S_CURVE_ANGLE_SPEED_CASCADE_CCW");
            } else {
                publishFeedback("TRACKING:S_CURVE_ANGLE_SPEED_CASCADE_HOLD");
            }

            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), control_status_log_period_ms_,
                "[精控 5/7 控制中] plan_id=%s, phase=ANGLE_SPEED_CASCADE, "
                "goal_yaw=%.4f, control_yaw=%.4f, tracking_yaw=%.4f, "
                "goal_longitudinal=%.4f, goal_lateral=%.4f, "
                "ref_position=%.4f/%.4f, ref_t=%.3f, ref_w=%.4f, ref_alpha=%.4f, "
                "plan_w_limit=%.4f, brake_w=%.4f, brake_limited=%s, target_w=%.4f, speed_pid=%.4f, "
                "min_w_applied=%s, terminal_w_applied=%s, cmd_alpha=%.4f, "
                "measured_v=%.4f, measured_w=%.4f, cmd_w=%.4f",
                active_trajectory_.trajectory.plan_id.c_str(), yaw_error, continuous_rotation_yaw_error_,
                tracking_yaw_error, longitudinal_error, lateral_error, reference.position, initial_motion_,
                reference.relative_time, direction * reference.velocity, direction * reference.acceleration,
                controller_command.planned_angular_velocity_limit, controller_command.braking_angular_velocity_limit,
                controller_command.braking_limited ? "true" : "false", controller_command.target_angular_velocity,
                controller_command.angular_velocity_pid_output,
                controller_command.minimum_velocity_applied ? "true" : "false",
                controller_command.terminal_velocity_applied ? "true" : "false",
                controller_command.angular_acceleration_command, measured_velocity_.linear.x,
                measured_velocity_.angular.z, command.angular.z);
        }
        cmd_vel_pub_->publish(command);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<MobileControlActionNode>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("mobile_control_action_node"), "%s", error.what());
    }
    rclcpp::shutdown();
    return 0;
}
