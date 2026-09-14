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
#include <chrono>
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

    enum class RotationVelocityCurve
    {
        STEP,
        SINE
    };

    enum class StraightVelocityCurve
    {
        STEP,
        SINE
    };

    MobileControlActionNode() : Node("mobile_control_action_node")
    {
        this->declare_parameter("control_frequency", 20.0);
        this->declare_parameter("control_reference_frame", "odom");
        this->declare_parameter("stop_confirm_duration", 0.2);
        this->declare_parameter("max_v", 0.35);
        this->declare_parameter("min_v", 0.03);
        this->declare_parameter("max_w", 1.0);
        this->declare_parameter("accel_x_max", 0.5);
        this->declare_parameter("decel_x_max", 0.25);
        this->declare_parameter("straight_max_w_inc", 1.57);
        this->declare_parameter("rotate_to_heading_max_angular_accel", 1.0);
        this->declare_parameter("rotate_to_heading_max_angular_decel", 0.8);
        this->declare_parameter("rotate_stop_yaw_error", 0.02);
        this->declare_parameter("rotate_stop_angular_vel", 0.03);
        this->declare_parameter("pure_rotation.velocity_curve", "sine");
        this->declare_parameter("pure_rotation.max_angular_vel", 0.6);
        this->declare_parameter("pure_rotation.sine_ramp_duration", 0.8);
        this->declare_parameter("pure_rotation.sine_slowdown_yaw_error", 0.7);
        this->declare_parameter("pure_rotation.brake_safety_factor", 0.8);
        this->declare_parameter("straight.velocity_curve", "sine");
        this->declare_parameter("straight.sine_ramp_duration", 0.8);
        this->declare_parameter("straight.sine_slowdown_distance", 0.6);
        this->declare_parameter("straight.brake_safety_factor", 0.75);
        this->declare_parameter("straight.stop_linear_vel", 0.03);
        this->declare_parameter("k_rho", 0.8);
        this->declare_parameter("near_target_dist", 0.3);
        this->declare_parameter("near_target_min_speed_scale", 0.5);
        this->declare_parameter("cmd_vel_filter_alpha", 0.3);
        this->declare_parameter("min_angular_vel", 0.01);
        this->declare_parameter("xy_tolerance", 0.02);
        this->declare_parameter("yaw_tolerance", 0.04);
        this->declare_parameter("goal_mode_rotate_xy_threshold", 0.03);
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
        control_reference_frame_ =
            this->get_parameter("control_reference_frame").as_string();
        if (control_reference_frame_.empty()) {
            control_reference_frame_ = "odom";
            RCLCPP_WARN(
                this->get_logger(),
                "控制参考坐标系不能为空，本次回退为 odom。");
        }
        stop_confirm_duration_ =
            std::max(0.0, this->get_parameter("stop_confirm_duration").as_double());
        stop_confirm_cycles_ = std::max(
            1, static_cast<int>(std::ceil(stop_confirm_duration_ * control_frequency_)));
        max_v_ = std::max(1e-3, this->get_parameter("max_v").as_double());
        // 最低接近速度用于越过底盘低速死区；进入位置容差后仍会按减速度上限平滑停车。
        min_v_ = std::clamp(
            this->get_parameter("min_v").as_double(), 0.0, max_v_);
        max_w_ = this->get_parameter("max_w").as_double();
        accel_x_max_ =
            std::max(1e-3, this->get_parameter("accel_x_max").as_double());
        decel_x_max_ =
            std::max(1e-3, this->get_parameter("decel_x_max").as_double());
        straight_max_w_inc_ = this->get_parameter("straight_max_w_inc").as_double();
        rotate_to_heading_max_angular_accel_ =
            std::max(
                1e-3,
                this->get_parameter("rotate_to_heading_max_angular_accel").as_double());
        rotate_to_heading_max_angular_decel_ =
            std::max(
                1e-3,
                this->get_parameter("rotate_to_heading_max_angular_decel").as_double());
        rotate_stop_yaw_error_ =
            std::max(0.0, this->get_parameter("rotate_stop_yaw_error").as_double());
        rotate_stop_angular_vel_ =
            std::max(0.0, this->get_parameter("rotate_stop_angular_vel").as_double());
        const std::string rotation_velocity_curve =
            this->get_parameter("pure_rotation.velocity_curve").as_string();
        if (rotation_velocity_curve == "step") {
            rotation_velocity_curve_ = RotationVelocityCurve::STEP;
        } else if (rotation_velocity_curve == "sine") {
            rotation_velocity_curve_ = RotationVelocityCurve::SINE;
        } else {
            rotation_velocity_curve_ = RotationVelocityCurve::SINE;
            RCLCPP_WARN(
                this->get_logger(),
                "原地转向速度曲线配置无效：velocity_curve=%s，仅支持 step 或 sine，"
                "本次使用 sine。",
                rotation_velocity_curve.c_str());
        }
        rotation_max_w_ = std::clamp(
            this->get_parameter("pure_rotation.max_angular_vel").as_double(),
            1e-3, max_w_);
        rotation_sine_ramp_duration_ =
            std::max(
                1e-3,
                this->get_parameter("pure_rotation.sine_ramp_duration").as_double());
        rotation_sine_slowdown_yaw_error_ =
            std::max(
                1e-3,
                this->get_parameter("pure_rotation.sine_slowdown_yaw_error").as_double());
        rotation_brake_safety_factor_ = std::clamp(
            this->get_parameter("pure_rotation.brake_safety_factor").as_double(),
            0.05, 1.0);
        const std::string straight_velocity_curve =
            this->get_parameter("straight.velocity_curve").as_string();
        if (straight_velocity_curve == "step") {
            straight_velocity_curve_ = StraightVelocityCurve::STEP;
        } else if (straight_velocity_curve == "sine") {
            straight_velocity_curve_ = StraightVelocityCurve::SINE;
        } else {
            straight_velocity_curve_ = StraightVelocityCurve::SINE;
            RCLCPP_WARN(
                this->get_logger(),
                "直线速度曲线配置无效：velocity_curve=%s，仅支持 step 或 sine，"
                "本次使用 sine。",
                straight_velocity_curve.c_str());
        }
        straight_sine_ramp_duration_ = std::max(
            1e-3,
            this->get_parameter("straight.sine_ramp_duration").as_double());
        straight_sine_slowdown_distance_ = std::max(
            1e-3,
            this->get_parameter("straight.sine_slowdown_distance").as_double());
        straight_brake_safety_factor_ = std::clamp(
            this->get_parameter("straight.brake_safety_factor").as_double(),
            0.05, 1.0);
        straight_stop_linear_vel_ = std::max(
            0.0,
            this->get_parameter("straight.stop_linear_vel").as_double());
        k_rho_ = this->get_parameter("k_rho").as_double();
        near_target_dist_ = this->get_parameter("near_target_dist").as_double();
        near_target_min_speed_scale_ = std::clamp(
            this->get_parameter("near_target_min_speed_scale").as_double(), 0.0, 1.0);
        filter_alpha_ = this->get_parameter("cmd_vel_filter_alpha").as_double();
        min_angular_vel_ = std::clamp(
            this->get_parameter("min_angular_vel").as_double(), 0.0, max_w_);
        xy_tolerance_ = this->get_parameter("xy_tolerance").as_double();
        yaw_tolerance_ = this->get_parameter("yaw_tolerance").as_double();
        goal_mode_rotate_xy_threshold_ = this->get_parameter("goal_mode_rotate_xy_threshold").as_double();
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

        RCLCPP_INFO(
            this->get_logger(),
            "Action 控制参数：参考坐标系=%s，停稳确认=%.3fs（%d周期），控制频率=%.1fHz，"
            "近目标减速距离=%.3fm，近目标最低速度比例=%.2f，"
            "直线曲线=%s，最低接近速度=%.3fm/s，线加/减速度=%.3f/%.3fm/s²，"
            "原地转向曲线=%s，最大角速度=%.3frad/s，最大角加/减速度=%.3f/%.3frad/s²",
            control_reference_frame_.c_str(),
            stop_confirm_duration_, stop_confirm_cycles_, control_frequency_,
            near_target_dist_, near_target_min_speed_scale_,
            straight_velocity_curve_ == StraightVelocityCurve::STEP ? "step" : "sine",
            min_v_, accel_x_max_, decel_x_max_,
            rotation_velocity_curve_ == RotationVelocityCurve::STEP ? "step" : "sine",
            rotation_max_w_, rotate_to_heading_max_angular_accel_,
            rotate_to_heading_max_angular_decel_);

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

    geometry_msgs::msg::PoseStamped target_pose_control_frame_;
    geometry_msgs::msg::Twist pre_cmd_vel_;
    geometry_msgs::msg::Twist filtered_cmd_vel_;
    bool has_control_target_{false};
    bool is_goal_{false};
    bool is_cancelling_{false};
    std::chrono::steady_clock::time_point goal_started_time_{};
    bool goal_timer_started_{false};
    int stop_count_{0};
    int stop_confirm_cycles_{1};
    GoalMode goal_mode_{GoalMode::STRAIGHT};

    double control_frequency_;
    std::string control_reference_frame_;
    double stop_confirm_duration_;
    double max_v_;
    double min_v_;
    double max_w_;
    double accel_x_max_;
    double decel_x_max_;
    double straight_max_w_inc_;
    double rotate_to_heading_max_angular_accel_;
    double rotate_to_heading_max_angular_decel_;
    double rotate_stop_yaw_error_;
    double rotate_stop_angular_vel_;
    RotationVelocityCurve rotation_velocity_curve_{RotationVelocityCurve::SINE};
    double rotation_max_w_;
    double rotation_sine_ramp_duration_;
    double rotation_sine_slowdown_yaw_error_;
    double rotation_brake_safety_factor_;
    bool rotation_profile_started_{false};
    bool rotation_settling_{false};
    double requested_rotation_angle_{0.0};
    double previous_raw_rotation_error_{0.0};
    double continuous_rotation_error_{0.0};
    bool rotation_error_initialized_{false};
    std::chrono::steady_clock::time_point rotation_profile_started_time_{};
    StraightVelocityCurve straight_velocity_curve_{StraightVelocityCurve::SINE};
    double straight_sine_ramp_duration_;
    double straight_sine_slowdown_distance_;
    double straight_brake_safety_factor_;
    double straight_stop_linear_vel_;
    bool straight_profile_started_{false};
    std::chrono::steady_clock::time_point straight_profile_started_time_{};
    double k_rho_;
    double near_target_dist_;
    double near_target_min_speed_scale_;
    double filter_alpha_;
    double min_angular_vel_;
    double xy_tolerance_;
    double yaw_tolerance_;
    double goal_mode_rotate_xy_threshold_;
    PidConfig straight_station_pid_conf_;
    PidConfig straight_high_speed_pid_conf_;
    PidConfig straight_low_speed_pid_conf_;
    PidConfig straight_reverse_station_pid_conf_;
    PidConfig straight_reverse_speed_pid_conf_;
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
        resetPidState(straight_station_pid_state_);
        resetPidState(straight_high_speed_pid_state_);
        resetPidState(straight_low_speed_pid_state_);
        resetPidState(straight_reverse_station_pid_state_);
        resetPidState(straight_reverse_speed_pid_state_);
        straight_angular_error_prev_ = 0.0;
        straight_angular_integral_ = 0.0;
    }

    void resetRotationErrorTracking()
    {
        requested_rotation_angle_ = 0.0;
        previous_raw_rotation_error_ = 0.0;
        continuous_rotation_error_ = 0.0;
        rotation_error_initialized_ = false;
    }

    double updateContinuousRotationError(double raw_rotation_error)
    {
        if (!rotation_error_initialized_) {
            previous_raw_rotation_error_ = raw_rotation_error;
            continuous_rotation_error_ = requested_rotation_angle_;
            rotation_error_initialized_ = true;
            return continuous_rotation_error_;
        }

        // 目标固定在控制参考系后，相邻周期的角误差变化量应连续且远小于pi。
        // 对变化量做归一化，再累计到请求角度，可避开精确180度的正负边界。
        const double error_delta = std::atan2(
            std::sin(raw_rotation_error - previous_raw_rotation_error_),
            std::cos(raw_rotation_error - previous_raw_rotation_error_));
        continuous_rotation_error_ += error_delta;
        previous_raw_rotation_error_ = raw_rotation_error;
        return continuous_rotation_error_;
    }

    const char * rotationDirectionName() const
    {
        if (requested_rotation_angle_ > 0.0) {
            return "逆时针";
        }
        if (requested_rotation_angle_ < 0.0) {
            return "顺时针";
        }
        return "未设置";
    }

    void resetControlState()
    {
        has_control_target_ = false;
        is_goal_ = false;
        is_cancelling_ = false;
        stop_count_ = 0;
        goal_mode_ = GoalMode::STRAIGHT;
        rotation_profile_started_ = false;
        rotation_settling_ = false;
        resetRotationErrorTracking();
        straight_profile_started_ = false;
        filtered_cmd_vel_.linear.x = 0.0;
        filtered_cmd_vel_.angular.z = 0.0;
        pre_cmd_vel_.linear.x = 0.0;
        pre_cmd_vel_.angular.z = 0.0;
        goal_timer_started_ = false;
        resetPidStates();
    }

    double goalElapsedSeconds() const
    {
        if (!goal_timer_started_) {
            return -1.0;
        }
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - goal_started_time_)
            .count();
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

    bool transformTargetToControlFrame(
        const geometry_msgs::msg::PoseStamped &target_pose,
        geometry_msgs::msg::PoseStamped &target_pose_control_frame,
        std::string &error_msg)
    {
        try {
            if (target_pose.header.frame_id != control_reference_frame_) {
                target_pose_control_frame = tf_buffer_->transform(
                    target_pose, control_reference_frame_, tf2::durationFromSec(0.1));
            } else {
                target_pose_control_frame = target_pose;
            }
        } catch (const tf2::TransformException &ex) {
            error_msg = ex.what();
            return false;
        }
        return true;
    }

    void setTarget(const geometry_msgs::msg::PoseStamped &target_pose)
    {
        target_pose_control_frame_ = target_pose;
        has_control_target_ = true;
        is_goal_ = false;
        is_cancelling_ = false;
        stop_count_ = 0;
        rotation_profile_started_ = false;
        rotation_settling_ = false;
        resetRotationErrorTracking();
        straight_profile_started_ = false;
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
            requested_rotation_angle_ = input_yaw;
        } else {
            goal_mode_ = GoalMode::STRAIGHT;
        }
    }

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const PreciseControl::Goal> goal)
    {
        if (active_goal_handle_) {
            RCLCPP_WARN(
                this->get_logger(),
                "[底盘直控目标被拒绝] 已有目标正在执行，当前任务已运行 %.3fs。",
                goalElapsedSeconds());
            return rclcpp_action::GoalResponse::REJECT;
        }

        geometry_msgs::msg::PoseStamped target_pose_control_frame;
        std::string error_msg;
        if (!transformTargetToControlFrame(
                goal->target_pose, target_pose_control_frame, error_msg))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "[底盘直控目标被拒绝] 无法将目标从 %s 转换到 %s，原因=%s。",
                goal->target_pose.header.frame_id.c_str(),
                control_reference_frame_.c_str(), error_msg.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandlePreciseControl>)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "[底盘直控取消请求已接受] 当前任务已运行 %.3fs，接下来执行平滑停车。",
            goalElapsedSeconds());
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandlePreciseControl> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        geometry_msgs::msg::PoseStamped target_pose_control_frame;
        std::string error_msg;
        if (!transformTargetToControlFrame(
                goal->target_pose, target_pose_control_frame, error_msg))
        {
            auto result = std::make_shared<PreciseControl::Result>();
            result->success = false;
            result->message =
                "Cannot transform goal to " + control_reference_frame_ + ": " + error_msg;
            goal_handle->abort(result);
            RCLCPP_ERROR(
                this->get_logger(),
                "[底盘直控执行中止] goal 接受后再次转换 TF 失败，"
                "目标坐标系=%s，控制参考坐标系=%s，原因=%s。",
                goal->target_pose.header.frame_id.c_str(),
                control_reference_frame_.c_str(), error_msg.c_str());
            return;
        }

        active_goal_handle_ = goal_handle;
        goal_started_time_ = std::chrono::steady_clock::now();
        goal_timer_started_ = true;
        setTarget(target_pose_control_frame);
        setGoalModeFromInput(goal->target_pose);
        RCLCPP_INFO(
            this->get_logger(),
            "[底盘直控状态变化] IDLE -> CONTROL，模式=%s，"
            "输入目标=(x=%.3fm, y=%.3fm, yaw=%.3frad)，"
            "锁定旋转方向=%s，目标坐标系=%s。",
            goal_mode_ == GoalMode::STRAIGHT ? "直线" : "原地旋转",
            goal->target_pose.pose.position.x, goal->target_pose.pose.position.y,
            tf2::getYaw(goal->target_pose.pose.orientation),
            goal_mode_ == GoalMode::ROTATE ? rotationDirectionName() : "不适用",
            goal->target_pose.header.frame_id.c_str());
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
        RCLCPP_INFO(
            this->get_logger(),
            "[底盘直控最终结果] CONTROL -> SUCCEEDED，"
            "停稳确认完成，总耗时=%.3fs。",
            goalElapsedSeconds());
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
            has_control_target_ = false;
            is_goal_ = false;
            stop_count_ = 0;
            resetPidStates();
            RCLCPP_INFO(
                this->get_logger(),
                "[底盘直控状态变化] CONTROL -> CANCELLING，"
                "任务耗时=%.3fs，开始平滑减速，当前速度=(线速度 %.3fm/s, 角速度 %.3frad/s)。",
                goalElapsedSeconds(), pre_cmd_vel_.linear.x, pre_cmd_vel_.angular.z);
        }
    }

    geometry_msgs::msg::Twist computeRotateToHeadingCommand(double yaw_error, double dt)
    {
        geometry_msgs::msg::Twist cmd_vel;
        const double abs_error = std::fabs(yaw_error);

        if (!rotation_profile_started_) {
            rotation_profile_started_ = true;
            rotation_profile_started_time_ = std::chrono::steady_clock::now();
        }

        if (
            rotation_settling_ &&
            std::fabs(pre_cmd_vel_.angular.z) <= rotate_stop_angular_vel_ &&
            abs_error > yaw_tolerance_)
        {
            // 完全停稳后误差仍超限才开始下一次修正，禁止运动中直接反向。
            rotation_settling_ = false;
            rotation_profile_started_time_ = std::chrono::steady_clock::now();
        }
        if (!rotation_settling_ && abs_error <= rotate_stop_yaw_error_) {
            rotation_settling_ = true;
        }

        double target_angular_vel = 0.0;
        if (!rotation_settling_) {
            // 剩余角度越小，允许速度越低，给底盘预留制动距离以减少过冲。
            const double braking_error =
                std::max(0.0, abs_error - rotate_stop_yaw_error_);
            const double braking_speed_limit =
                rotation_brake_safety_factor_ *
                std::sqrt(
                    2.0 * rotate_to_heading_max_angular_decel_ * braking_error);

            double curve_speed_limit = rotation_max_w_;
            if (rotation_velocity_curve_ == RotationVelocityCurve::SINE) {
                const double elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - rotation_profile_started_time_)
                                           .count();
                const double ramp_ratio = std::clamp(
                    elapsed / rotation_sine_ramp_duration_, 0.0, 1.0);
                const double slowdown_ratio = std::clamp(
                    braking_error / rotation_sine_slowdown_yaw_error_, 0.0, 1.0);
                const double ramp_scale = std::sin(M_PI_2 * ramp_ratio);
                const double slowdown_scale = std::sin(M_PI_2 * slowdown_ratio);
                curve_speed_limit =
                    rotation_max_w_ * std::min(ramp_scale, slowdown_scale);
            }

            double target_speed_abs = std::min(curve_speed_limit, braking_speed_limit);
            // 仍在转动时允许命令平滑降到死区以下；确实停住且误差超限时，
            // 才使用最小角速度重新起转，减少末端反复启停和过冲。
            if (
                target_speed_abs < min_angular_vel_ &&
                std::fabs(pre_cmd_vel_.angular.z) <= rotate_stop_angular_vel_)
            {
                target_speed_abs = min_angular_vel_;
            }
            target_speed_abs = std::clamp(
                target_speed_abs, 0.0, rotation_max_w_);
            target_angular_vel = std::copysign(target_speed_abs, yaw_error);
        }

        const double previous_cmd = filtered_cmd_vel_.angular.z;
        const bool is_accelerating =
            target_angular_vel * previous_cmd >= 0.0 &&
            std::fabs(target_angular_vel) > std::fabs(previous_cmd);
        const double angular_rate_limit = is_accelerating
            ? rotate_to_heading_max_angular_accel_
            : rotate_to_heading_max_angular_decel_;
        const double max_delta = angular_rate_limit * std::max(dt, 0.0);
        const double delta = std::clamp(
            target_angular_vel - previous_cmd, -max_delta, max_delta);
        cmd_vel.angular.z = previous_cmd + delta;
        if (!std::isfinite(cmd_vel.angular.z)) {
            cmd_vel.angular.z = 0.0;
        }
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
        const double yaw_error =
            std::fabs(raw_target_yaw_in_base) < yaw_tolerance_ ? 0.0 : raw_target_yaw_in_base;

        const double dist = std::fabs(err_x);
        if (!straight_profile_started_) {
            straight_profile_started_ = true;
            straight_profile_started_time_ = std::chrono::steady_clock::now();
        }

        const double braking_distance =
            std::max(0.0, dist - xy_tolerance_);
        const double braking_speed_limit =
            straight_brake_safety_factor_ *
            std::sqrt(2.0 * decel_x_max_ * braking_distance);
        double curve_speed_limit = max_v_;
        if (straight_velocity_curve_ == StraightVelocityCurve::SINE) {
            const double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - straight_profile_started_time_)
                                       .count();
            const double ramp_ratio = std::clamp(
                elapsed / straight_sine_ramp_duration_, 0.0, 1.0);
            const double slowdown_ratio = std::clamp(
                braking_distance / straight_sine_slowdown_distance_, 0.0, 1.0);
            const double ramp_scale = std::sin(M_PI_2 * ramp_ratio);
            const double slowdown_scale = std::sin(M_PI_2 * slowdown_ratio);
            curve_speed_limit =
                max_v_ * std::min(ramp_scale, slowdown_scale);
        }

        double desired_linear_speed_abs = std::min(
            {max_v_, k_rho_ * dist, braking_speed_limit, curve_speed_limit});
        if (dist < near_target_dist_ && near_target_dist_ > 0.0) {
            const double distance_ratio = std::clamp(
                dist / near_target_dist_, 0.0, 1.0);
            const double speed_scale = near_target_min_speed_scale_ +
                (1.0 - near_target_min_speed_scale_) * distance_ratio;
            desired_linear_speed_abs *= speed_scale;
        }
        if (braking_distance > 0.0) {
            desired_linear_speed_abs = std::clamp(
                desired_linear_speed_abs, min_v_, max_v_);
        } else {
            desired_linear_speed_abs = 0.0;
        }
        double desired_linear_vel = desired_linear_speed_abs;
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
        acceleration_cmd = std::clamp(
            acceleration_cmd, -decel_x_max_, accel_x_max_);

        double pid_output = current_velocity.linear.x + acceleration_cmd * dt;
        if (!std::isfinite(pid_output)) {
            pid_output = 0.0;
        }
        if (!reverse_mode) {
            pid_output = std::clamp(
                pid_output, 0.0, desired_linear_speed_abs);
        } else {
            pid_output = std::clamp(
                pid_output, -desired_linear_speed_abs, 0.0);
        }

        // 速度包络可能随剩余距离快速下降，最终发布命令仍需单独限制变化率，
        // 不能再次出现进入容差后从较高速度直接清零。
        const double previous_linear_cmd = filtered_cmd_vel_.linear.x;
        const bool is_linear_accelerating =
            pid_output * previous_linear_cmd >= 0.0 &&
            std::fabs(pid_output) > std::fabs(previous_linear_cmd);
        const double linear_rate_limit =
            is_linear_accelerating ? accel_x_max_ : decel_x_max_;
        const double max_linear_delta =
            linear_rate_limit * std::max(dt, 0.0);
        const double linear_output = previous_linear_cmd + std::clamp(
            pid_output - previous_linear_cmd,
            -max_linear_delta,
            max_linear_delta);

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
            // 取消动作时沿用直线减速度上限，避免取消与正常到点采用不同的刹车强度。
            const double linear_decel = decel_x_max_ * dt;
            const double angular_decel = rotate_to_heading_max_angular_decel_ * dt;

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
                RCLCPP_INFO(
                    this->get_logger(),
                    "[底盘直控最终结果] CANCELLING -> CANCELLED，"
                    "平滑停车完成，总耗时=%.3fs，最终速度阈值内。",
                    goalElapsedSeconds());
                finishCanceled();
                return;
            }

            cmd_vel_pub_->publish(cmd_vel);
            return;
        }

        if (is_goal_ && stop_count_ >= stop_confirm_cycles_) {
            publishFeedback("SUCCEEDED");
            finishSucceeded();
            return;
        }

        if (!has_control_target_) {
            publishFeedback("IDLE");
            return;
        }

        publishFeedback("CONTROL");

        geometry_msgs::msg::PoseStamped target_pose_in_base;
        try {
            const geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "base_link", control_reference_frame_, tf2::TimePointZero);
            tf2::doTransform(target_pose_control_frame_, target_pose_in_base, transform);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "[底盘直控等待 TF] %s -> base_link 转换失败，"
                "任务耗时=%.3fs，控制指令暂不更新，原因=%s。",
                control_reference_frame_.c_str(), goalElapsedSeconds(), ex.what());
            return;
        }

        const double dist = std::hypot(
            target_pose_in_base.pose.position.x, target_pose_in_base.pose.position.y);
        const double x_error = target_pose_in_base.pose.position.x;
        const double raw_yaw_error = tf2::getYaw(target_pose_in_base.pose.orientation);
        const double yaw_error = goal_mode_ == GoalMode::ROTATE
            ? updateContinuousRotationError(raw_yaw_error)
            : raw_yaw_error;
        const double dt = 1.0 / control_frequency_;
        const bool is_reverse_mode =
            target_pose_in_base.pose.position.x < 0.0 &&
            std::fabs(target_pose_in_base.pose.position.y) < 0.5;

        geometry_msgs::msg::Twist cmd_vel;
        // 模式只在接受目标时判定一次。原地转向期间即使参考坐标系出现
        // 小幅位置漂移，也保持线速度为零，禁止中途切换到直线控制器。
        if (goal_mode_ == GoalMode::ROTATE) {
            cmd_vel = computeRotateToHeadingCommand(yaw_error, dt);
            cmd_vel.linear.x = 0.0;
        } else {
            cmd_vel = computeStraightPidCommand(
                target_pose_in_base.pose, pre_cmd_vel_, dt, is_reverse_mode);
        }

        if (goal_mode_ == GoalMode::ROTATE) {
            // 原地转向已由速度曲线和角加速度统一约束，不再叠加低通滤波延迟。
            filtered_cmd_vel_ = cmd_vel;
        } else {
            // 直线命令已经经过正弦包络和加减速度限制，不再叠加低通延迟。
            filtered_cmd_vel_.linear.x = cmd_vel.linear.x;
            filtered_cmd_vel_.angular.z =
                filter_alpha_ * cmd_vel.angular.z +
                (1.0 - filter_alpha_) * filtered_cmd_vel_.angular.z;
        }

        bool reached_goal = false;
        if (goal_mode_ == GoalMode::STRAIGHT) {
            reached_goal =
                std::fabs(x_error) < xy_tolerance_ &&
                std::fabs(yaw_error) < yaw_tolerance_ &&
                std::fabs(pre_cmd_vel_.linear.x) <= straight_stop_linear_vel_;
        } else {
            reached_goal =
                std::fabs(yaw_error) <= yaw_tolerance_ &&
                rotation_settling_ &&
                std::fabs(pre_cmd_vel_.angular.z) <= rotate_stop_angular_vel_;
        }
        // 停稳确认期间每周期重新判断，误差重新超限时继续控制，不能锁存成功状态。
        is_goal_ = reached_goal;

        if (is_goal_) {
            resetPidStates();
            filtered_cmd_vel_.linear.x = 0.0;
            filtered_cmd_vel_.angular.z = 0.0;
            stop_count_++;

            cmd_vel_pub_->publish(filtered_cmd_vel_);
            if (stop_count_ == 1) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "[底盘直控到达目标] 进入停稳确认，任务耗时=%.3fs，"
                    "最终误差：x=%.4fm，距离=%.4fm，yaw=%.4frad；"
                    "需连续确认 %d 个控制周期（%.3fs）后返回成功。",
                    goalElapsedSeconds(), x_error, dist, yaw_error,
                    stop_confirm_cycles_, stop_confirm_duration_);
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
