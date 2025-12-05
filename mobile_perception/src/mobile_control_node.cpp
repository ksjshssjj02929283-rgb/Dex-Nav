#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <cmath>
#include <algorithm>
#include <deque>

using namespace std::chrono_literals;

namespace controller_common {

struct PidConf {
  bool integrator_enable = false;
  double integrator_saturation_level = 0.0;
  double output_saturation_level = 0.0;
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double kaw = 0.0;
};

/**
 * @brief Simple PID Controller implementation
 */
class PIDController {
public:
  void Init(const PidConf &pid_conf) {
    previous_error_ = 0.0;
    previous_output_ = 0.0;
    integral_ = 0.0;
    first_hit_ = true;
    integrator_enabled_ = pid_conf.integrator_enable;
    integrator_saturation_high_ = std::fabs(pid_conf.integrator_saturation_level);
    integrator_saturation_low_ = -std::fabs(pid_conf.integrator_saturation_level);
    integrator_saturation_status_ = 0;
    integrator_hold_ = false;
    output_saturation_high_ = std::fabs(pid_conf.output_saturation_level);
    output_saturation_low_ = -std::fabs(pid_conf.output_saturation_level);
    output_saturation_status_ = 0;
    SetPID(pid_conf);
  }

  void SetPID(const PidConf &pid_conf) {
    kp_ = pid_conf.kp;
    ki_ = pid_conf.ki;
    kd_ = pid_conf.kd;
    kaw_ = pid_conf.kaw;
  }

  void Reset() {
    previous_error_ = 0.0;
    previous_output_ = 0.0;
    integral_ = 0.0;
    first_hit_ = true;
    integrator_saturation_status_ = 0;
    output_saturation_status_ = 0;
  }

  void Reset_integral() {
    integral_ = 0.0;
    integrator_saturation_status_ = 0;
  }

  double Control(const double error, const double dt) {
    if (dt <= 0) {
      return previous_output_;
    }
    
    double diff = 0;
    double output = 0;

    if (first_hit_) {
      first_hit_ = false;
    } else {
      diff = (error - previous_error_) / dt;
    }
    
    // integral hold
    if (!integrator_enabled_) {
      integral_ = 0;
    } else if (!integrator_hold_) {
      integral_ += error * dt * ki_;
      if (integral_ > integrator_saturation_high_) {
        integral_ = integrator_saturation_high_;
        integrator_saturation_status_ = 1;
      } else if (integral_ < integrator_saturation_low_) {
        integral_ = integrator_saturation_low_;
        integrator_saturation_status_ = -1;
      } else {
        integrator_saturation_status_ = 0;
      }
    }
    
    previous_error_ = error;
    output = error * kp_ + integral_ + diff * kd_;
    previous_output_ = output;
    return output;
  }

  bool IntegratorHold() const { return integrator_hold_; }
  void SetIntegratorHold(bool hold) { integrator_hold_ = hold; }

private:
  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double kaw_ = 0.0;
  double previous_error_ = 0.0;
  double previous_output_ = 0.0;
  double integral_ = 0.0;
  double integrator_saturation_high_ = 0.0;
  double integrator_saturation_low_ = 0.0;
  bool first_hit_ = false;
  bool integrator_enabled_ = false;
  bool integrator_hold_ = false;
  int integrator_saturation_status_ = 0;
  double output_saturation_high_ = 0.0;
  double output_saturation_low_ = 0.0;
  int output_saturation_status_ = 0;
};

}  // namespace controller_common

class MobileControlNode : public rclcpp::Node
{
public:
    MobileControlNode() : Node("mobile_control_node")
    {
        // 1. 参数声明
        this->declare_parameter("control_frequency", 20.0);
        this->declare_parameter("max_v", 0.35);
        this->declare_parameter("min_v", 0.0);
        this->declare_parameter("max_w", 1.0);
        this->declare_parameter("accel_x_max", 0.5);
        
        // LatLon controller specific parameters
        this->declare_parameter("p_window", 0.5);
        this->declare_parameter("long_precision", 0.02);
        this->declare_parameter("lat_precision", 0.04);
        this->declare_parameter("o_precision", 0.5);
        this->declare_parameter("k_theta", 0.5);
        this->declare_parameter("switch_speed", 0.1);
        this->declare_parameter("adjust_heading_dist", 0.25);
        this->declare_parameter("reverse_enabled", false);
        
        // PID parameters for high speed
        this->declare_parameter("high_speed_station_pid_conf.kp", 1.0);
        this->declare_parameter("high_speed_station_pid_conf.ki", 0.01);
        this->declare_parameter("high_speed_station_pid_conf.kd", 0.1);
        this->declare_parameter("high_speed_station_pid_conf.integrator_enable", true);
        this->declare_parameter("high_speed_station_pid_conf.integrator_saturation_level", 0.3);
        
        // PID parameters for low speed
        this->declare_parameter("low_speed_station_pid_conf.kp", 1.0);
        this->declare_parameter("low_speed_station_pid_conf.ki", 0.01);
        this->declare_parameter("low_speed_station_pid_conf.kd", 0.1);
        this->declare_parameter("low_speed_station_pid_conf.integrator_enable", true);
        this->declare_parameter("low_speed_station_pid_conf.integrator_saturation_level", 0.3);
        
        // PID parameters for high speed linear
        this->declare_parameter("high_speed_pid_conf.kp", 1.0);
        this->declare_parameter("high_speed_pid_conf.ki", 0.01);
        this->declare_parameter("high_speed_pid_conf.kd", 0.1);
        this->declare_parameter("high_speed_pid_conf.integrator_enable", true);
        this->declare_parameter("high_speed_pid_conf.integrator_saturation_level", 0.3);
        
        // PID parameters for low speed linear
        this->declare_parameter("low_speed_pid_conf.kp", 1.0);
        this->declare_parameter("low_speed_pid_conf.ki", 0.01);
        this->declare_parameter("low_speed_pid_conf.kd", 0.1);
        this->declare_parameter("low_speed_pid_conf.integrator_enable", true);
        this->declare_parameter("low_speed_pid_conf.integrator_saturation_level", 0.3);
        
        // PID parameters for angular
        this->declare_parameter("angular_station_pid_conf.kp", 1.0);
        this->declare_parameter("angular_station_pid_conf.ki", 0.01);
        this->declare_parameter("angular_station_pid_conf.kd", 0.1);
        this->declare_parameter("angular_station_pid_conf.integrator_enable", true);
        this->declare_parameter("angular_station_pid_conf.integrator_saturation_level", 0.5);

        // 获取参数
        control_frequency_ = this->get_parameter("control_frequency").as_double();
        accel_x_max_ = this->get_parameter("accel_x_max").as_double();
        max_v_ = this->get_parameter("max_v").as_double();
        min_v_ = this->get_parameter("min_v").as_double();
        max_w_ = this->get_parameter("max_w").as_double();
        
        p_window_ = this->get_parameter("p_window").as_double();
        long_precision_ = this->get_parameter("long_precision").as_double();
        lat_precision_ = this->get_parameter("lat_precision").as_double();
        o_precision_ = this->get_parameter("o_precision").as_double();
        k_theta_ = this->get_parameter("k_theta").as_double();
        switch_speed_ = this->get_parameter("switch_speed").as_double();
        adjust_heading_dist_ = this->get_parameter("adjust_heading_dist").as_double();
        reverse_enabled_ = this->get_parameter("reverse_enabled").as_bool();
        
        d_t_ = 1.0 / control_frequency_;

        // Load PID configurations
        controller_common::PidConf high_speed_station_pid_conf;
        high_speed_station_pid_conf.kp = this->get_parameter("high_speed_station_pid_conf.kp").as_double();
        high_speed_station_pid_conf.ki = this->get_parameter("high_speed_station_pid_conf.ki").as_double();
        high_speed_station_pid_conf.kd = this->get_parameter("high_speed_station_pid_conf.kd").as_double();
        high_speed_station_pid_conf.integrator_enable = this->get_parameter("high_speed_station_pid_conf.integrator_enable").as_bool();
        high_speed_station_pid_conf.integrator_saturation_level = this->get_parameter("high_speed_station_pid_conf.integrator_saturation_level").as_double();

        controller_common::PidConf low_speed_station_pid_conf;
        low_speed_station_pid_conf.kp = this->get_parameter("low_speed_station_pid_conf.kp").as_double();
        low_speed_station_pid_conf.ki = this->get_parameter("low_speed_station_pid_conf.ki").as_double();
        low_speed_station_pid_conf.kd = this->get_parameter("low_speed_station_pid_conf.kd").as_double();
        low_speed_station_pid_conf.integrator_enable = this->get_parameter("low_speed_station_pid_conf.integrator_enable").as_bool();
        low_speed_station_pid_conf.integrator_saturation_level = this->get_parameter("low_speed_station_pid_conf.integrator_saturation_level").as_double();

        controller_common::PidConf high_speed_pid_conf;
        high_speed_pid_conf.kp = this->get_parameter("high_speed_pid_conf.kp").as_double();
        high_speed_pid_conf.ki = this->get_parameter("high_speed_pid_conf.ki").as_double();
        high_speed_pid_conf.kd = this->get_parameter("high_speed_pid_conf.kd").as_double();
        high_speed_pid_conf.integrator_enable = this->get_parameter("high_speed_pid_conf.integrator_enable").as_bool();
        high_speed_pid_conf.integrator_saturation_level = this->get_parameter("high_speed_pid_conf.integrator_saturation_level").as_double();

        controller_common::PidConf low_speed_pid_conf;
        low_speed_pid_conf.kp = this->get_parameter("low_speed_pid_conf.kp").as_double();
        low_speed_pid_conf.ki = this->get_parameter("low_speed_pid_conf.ki").as_double();
        low_speed_pid_conf.kd = this->get_parameter("low_speed_pid_conf.kd").as_double();
        low_speed_pid_conf.integrator_enable = this->get_parameter("low_speed_pid_conf.integrator_enable").as_bool();
        low_speed_pid_conf.integrator_saturation_level = this->get_parameter("low_speed_pid_conf.integrator_saturation_level").as_double();

        controller_common::PidConf angular_station_pid_conf;
        angular_station_pid_conf.kp = this->get_parameter("angular_station_pid_conf.kp").as_double();
        angular_station_pid_conf.ki = this->get_parameter("angular_station_pid_conf.ki").as_double();
        angular_station_pid_conf.kd = this->get_parameter("angular_station_pid_conf.kd").as_double();
        angular_station_pid_conf.integrator_enable = this->get_parameter("angular_station_pid_conf.integrator_enable").as_bool();
        angular_station_pid_conf.integrator_saturation_level = this->get_parameter("angular_station_pid_conf.integrator_saturation_level").as_double();

        // Store PID configurations
        high_speed_station_pid_conf_ = high_speed_station_pid_conf;
        low_speed_station_pid_conf_ = low_speed_station_pid_conf;
        high_speed_pid_conf_ = high_speed_pid_conf;
        low_speed_pid_conf_ = low_speed_pid_conf;
        angular_station_pid_conf_ = angular_station_pid_conf;

        // Initialize PID controllers
        station_pid_controller_.Init(low_speed_station_pid_conf_);
        speed_pid_controller_.Init(low_speed_pid_conf_);
        angular_station_pid_controller_.Init(angular_station_pid_conf_);

        // 2. 初始化 TF 和 通信
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("control/cmd_vel", 10);
        
        // 订阅 base_link 坐标系下的目标点
        target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "control/ctrl_pose", 10, 
            std::bind(&MobileControlNode::targetCallback, this, std::placeholders::_1));

        // 订阅 odom 话题获取机器人实际速度
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "feedback/odom", 10, 
            std::bind(&MobileControlNode::odomCallback, this, std::placeholders::_1));

        // 3. 定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / control_frequency_)),
            std::bind(&MobileControlNode::controlLoop, this));

        // 状态初始化
        get_target_pose_ = false;
        target_pose_odom_initialized_ = false;
        curr_controller_stage_ = ControllerStage::FINISH;
        prev_controller_stage_ = ControllerStage::FINISH;
        desired_linear_vel_ = 0.0;
        stop_count_ = 0;
        robot_actual_vel_.linear.x = 0.0;
        robot_actual_vel_.angular.z = 0.0;
    }

private:
    enum ControllerStage {
        FINISH = 0,
        APPROACHING = 1,
        ADJUST_HEADING = 2,
        ADJUST_DISTANCE = 3,
        ROTATING_TO_GOAL = 4
    };

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::PoseStamped target_pose_base_link_;  // 接收到的相对目标点
    geometry_msgs::msg::PoseStamped target_pose_odom_;       // 转换到 odom 坐标系的绝对目标点
    geometry_msgs::msg::Twist robot_actual_vel_;             // 从 odom 话题获取的机器人实际速度
    bool get_target_pose_;
    bool target_pose_odom_initialized_;
    
    double control_frequency_;
    double accel_x_max_;
    double max_v_, min_v_, max_w_;
    
    // LatLon controller parameters
    double p_window_;
    double long_precision_, lat_precision_, o_precision_;
    double k_theta_;
    double switch_speed_;
    double adjust_heading_dist_;
    bool reverse_enabled_;
    double d_t_;
    double desired_linear_vel_;
    
    ControllerStage curr_controller_stage_;
    ControllerStage prev_controller_stage_;
    
    std::deque<double> theta_errors_;
    std::deque<double> longitude_errors_;
    
    // PID Controllers
    controller_common::PIDController station_pid_controller_;
    controller_common::PIDController speed_pid_controller_;
    controller_common::PIDController angular_station_pid_controller_;
    
    // PID Configurations
    controller_common::PidConf high_speed_station_pid_conf_;
    controller_common::PidConf low_speed_station_pid_conf_;
    controller_common::PidConf high_speed_pid_conf_;
    controller_common::PidConf low_speed_pid_conf_;
    controller_common::PidConf angular_station_pid_conf_;
    
    int stop_count_;

    void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Target is in base_link frame (relative position)
        if (msg->header.frame_id == "base_link") {
            target_pose_base_link_ = *msg;
            
            // Convert relative target in base_link to absolute target in odom frame
            try {
                // Get current robot pose in odom frame
                geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                    "odom", "base_link", tf2::TimePointZero);
                
                geometry_msgs::msg::PoseStamped target_in_odom;
                target_in_odom.header.frame_id = "base_link";
                target_in_odom.header.stamp = msg->header.stamp;
                target_in_odom.pose = msg->pose;
                
                // Transform target from base_link to odom
                tf2::doTransform(target_in_odom, target_pose_odom_, transform);
                target_pose_odom_.header.frame_id = "odom";
                
                get_target_pose_ = true;
                target_pose_odom_initialized_ = true;
                curr_controller_stage_ = ControllerStage::APPROACHING;
                stop_count_ = 0;
                theta_errors_.clear();
                longitude_errors_.clear();
                
                RCLCPP_INFO(this->get_logger(), 
                    "New target received in base_link frame. Converted to odom: (%.3f, %.3f, %.3f)",
                    target_pose_odom_.pose.position.x, 
                    target_pose_odom_.pose.position.y,
                    tf2::getYaw(target_pose_odom_.pose.orientation));
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Cannot transform target to odom: %s", ex.what());
                get_target_pose_ = false;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Target frame is %s, expected base_link", msg->header.frame_id.c_str());
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 从 odom 话题获取机器人实际速度
        robot_actual_vel_.linear.x = msg->twist.twist.linear.x;
        robot_actual_vel_.angular.z = msg->twist.twist.angular.z;
    }

    /**
     * @brief Regularize angle to [-PI, PI]
     */
    void regularizeAngle(double& angle) {
        angle = angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
    }

    /**
     * @brief Calculate mean of deque
     */
    double CalculateMean(const std::deque<double>& data) {
        if (data.empty()) return 0.0;
        double sum = 0.0;
        for (const auto& value : data) {
            sum += value;
        }
        return sum / static_cast<double>(data.size());
    }

    /**
     * @brief Check if should rotate to path
     */
    bool shouldRotateToPath(double angle_to_path) {
        return (std::fabs(angle_to_path) > M_PI / 6.0);  // 30 degrees
    }

    /**
     * @brief Rotate to heading
     */
    void rotateToHeading(double & linear_vel, double & angular_vel, const double & angle_to_path) {
        linear_vel = 0.0;
        const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
        angular_vel = sign * 0.6;
        angular_vel = std::clamp(angular_vel, -max_w_, max_w_);
    }

    /**
     * @brief Linear speed control using PID
     */
    double LinearSpeedControl(double current_vel, double err_x) {
        // Select PID configuration based on current velocity
        if (current_vel < switch_speed_) {
            station_pid_controller_.SetPID(low_speed_station_pid_conf_);
            speed_pid_controller_.SetPID(low_speed_pid_conf_);
        } else {
            station_pid_controller_.SetPID(high_speed_station_pid_conf_);
            speed_pid_controller_.SetPID(high_speed_pid_conf_);
        }

        double speed_error = desired_linear_vel_ - current_vel;
        double speed_offset = station_pid_controller_.Control(err_x, d_t_);
        double speed_controller_input = speed_offset + speed_error;
        double acceleration_cmd = speed_pid_controller_.Control(speed_controller_input, d_t_);
        double acceleration_cmd_limited = std::clamp(acceleration_cmd, -accel_x_max_, accel_x_max_);
        double v_output = current_vel + acceleration_cmd_limited * d_t_;

        if (!std::isfinite(v_output)) {
            v_output = 0.0;
        }

        v_output = std::clamp(v_output, min_v_, desired_linear_vel_ * 1.5);
        return v_output;
    }

    /**
     * @brief Angular speed control using PID
     */
    double AngularSpeedControl(double err_theta) {
        regularizeAngle(err_theta);
        double vel_angular_output = angular_station_pid_controller_.Control(err_theta, d_t_);
        
        if (!std::isfinite(vel_angular_output)) {
            vel_angular_output = 0.0;
        }
        
        vel_angular_output = std::clamp(vel_angular_output, -max_w_, max_w_);
        return vel_angular_output;
    }

    void controlLoop()
    {
        if (stop_count_ > 50) return;

        if (!get_target_pose_ || !target_pose_odom_initialized_) return;

        // Transform target from odom to current base_link frame
        geometry_msgs::msg::PoseStamped target_in_base_link;
        try {
            target_in_base_link = target_pose_odom_;
            target_in_base_link.header.stamp = rclcpp::Time(0);
            tf_buffer_->transform(target_in_base_link, target_in_base_link, "base_link");
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Cannot transform target to base_link: %s", ex.what());
            return;
        }

        // Extract errors directly from target in base_link frame
        // Since base_link origin is at robot center, errors are relative to robot
        double err_x = target_in_base_link.pose.position.x;  // longitudinal error
        double err_y = target_in_base_link.pose.position.y;  // lateral error
        double err_theta = tf2::getYaw(target_in_base_link.pose.orientation);
        
        regularizeAngle(err_theta);

        double dist = std::hypot(err_x, err_y);

        RCLCPP_INFO(this->get_logger(), 
            "Errors -> Longitudinal: %.3f m, Lateral: %.3f m, Yaw: %.3f rad, Dist: %.3f m",
            err_x, err_y, err_theta, dist);

        // State machine logic
        longitude_errors_.push_back(std::fabs(err_x));
        while (longitude_errors_.size() > 20) {
            longitude_errors_.pop_front();
        }

        double longitude_mean = std::numeric_limits<double>::max();
        if (longitude_errors_.size() >= 20) {
            longitude_mean = CalculateMean(longitude_errors_);
        }

        theta_errors_.push_back(std::fabs(err_theta));
        while (theta_errors_.size() > 50) {
            theta_errors_.pop_front();
        }

        double theta_mean = std::numeric_limits<double>::max();
        if (theta_errors_.size() >= 50) {
            theta_mean = CalculateMean(theta_errors_);
        }

        // State transitions
        if (dist > 0.2 || curr_controller_stage_ == ControllerStage::FINISH ||
            (dist > 0.05 && curr_controller_stage_ == ControllerStage::ROTATING_TO_GOAL)) {
            curr_controller_stage_ = ControllerStage::APPROACHING;
            theta_errors_.clear();
            longitude_errors_.clear();
        }

        // 如果只有角度偏差，直接进入旋转到目标阶段
        if (dist < long_precision_ && std::fabs(err_theta) > o_precision_) {
            curr_controller_stage_ = ControllerStage::ROTATING_TO_GOAL;
        }

        if (curr_controller_stage_ == ControllerStage::APPROACHING && dist < adjust_heading_dist_) {
            curr_controller_stage_ = ControllerStage::ADJUST_HEADING;
        }

        if (curr_controller_stage_ == ControllerStage::ADJUST_HEADING) {
            if (theta_mean < o_precision_) {
                curr_controller_stage_ = ControllerStage::ADJUST_DISTANCE;
            }
        }

        if (curr_controller_stage_ == ControllerStage::ADJUST_DISTANCE) {
            if (longitude_mean < long_precision_ && std::fabs(err_y) > lat_precision_) {
                curr_controller_stage_ = ControllerStage::ADJUST_HEADING;
            }
        }

        if (longitude_mean < long_precision_ * 0.5 &&
            std::fabs(err_x) < long_precision_ && std::fabs(err_y) < lat_precision_ * 0.75) {
            curr_controller_stage_ = ControllerStage::ROTATING_TO_GOAL;
        }

        if (curr_controller_stage_ != prev_controller_stage_) {
            RCLCPP_INFO(this->get_logger(), 
                "Controller Stage changed to: %d (0:FINISH, 1:APPROACH, 2:ADJUST_H, 3:ADJUST_D, 4:ROTATE)",
                static_cast<int>(curr_controller_stage_));
        }
        prev_controller_stage_ = curr_controller_stage_;

        // Compute velocity commands based on stage
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;

        double angle_to_path = std::atan2(err_y, err_x);
        
        if (curr_controller_stage_ != ControllerStage::ROTATING_TO_GOAL && 
            shouldRotateToPath(angle_to_path)) {
            // Need to rotate to face the target
            rotateToHeading(cmd_vel.linear.x, cmd_vel.angular.z, angle_to_path);
        } else {
            // Use PID-based control
            desired_linear_vel_ = max_v_;
            
            switch (curr_controller_stage_) {
                case ControllerStage::FINISH:
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    break;
                    
                case ControllerStage::APPROACHING:
                    cmd_vel.linear.x = LinearSpeedControl(robot_actual_vel_.linear.x, err_x);
                    cmd_vel.angular.z = AngularSpeedControl(err_theta);
                    break;
                    
                case ControllerStage::ADJUST_HEADING:
                    cmd_vel.linear.x = LinearSpeedControl(robot_actual_vel_.linear.x, err_x) * 0.75;
                    cmd_vel.angular.z = AngularSpeedControl(err_theta);
                    break;
                    
                case ControllerStage::ADJUST_DISTANCE:
                    cmd_vel.linear.x = LinearSpeedControl(robot_actual_vel_.linear.x, err_x) * 0.75;
                    cmd_vel.angular.z = AngularSpeedControl(err_theta);
                    break;
                    
                case ControllerStage::ROTATING_TO_GOAL:
                    // 如果只有角度偏差（距离很小），则只旋转不移动
                    if (dist < long_precision_) {
                        cmd_vel.linear.x = 0.0;
                    } else {
                        cmd_vel.linear.x = LinearSpeedControl(robot_actual_vel_.linear.x, err_x) * 0.25;
                    }
                    cmd_vel.angular.z = AngularSpeedControl(err_theta);
                    break;
            }
        }

        // Check if goal reached
        if (std::fabs(err_x) < long_precision_ && std::fabs(err_y) < lat_precision_ && 
            std::fabs(err_theta) < o_precision_) {
            curr_controller_stage_ = ControllerStage::FINISH;
        }
        
        // 如果在旋转阶段且只有角度偏差，当角度误差足够小时完成
        if (curr_controller_stage_ == ControllerStage::ROTATING_TO_GOAL && 
            dist < long_precision_ && std::fabs(err_theta) < o_precision_) {
            curr_controller_stage_ = ControllerStage::FINISH;
        }

        // Handle goal reached state
        if (curr_controller_stage_ == ControllerStage::FINISH) {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            stop_count_++;
            if (stop_count_ <= 50) {
                cmd_vel_pub_->publish(cmd_vel);
                if (stop_count_ % 10 == 0) {
                    RCLCPP_INFO(this->get_logger(), "Goal Reached. Stopping... %d", stop_count_);
                }
            }
        } else {
            stop_count_ = 0;
            cmd_vel_pub_->publish(cmd_vel);
            RCLCPP_INFO(this->get_logger(), 
                "Velocity Command -> Linear: %.3f m/s, Angular: %.3f rad/s",
                cmd_vel.linear.x, cmd_vel.angular.z);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MobileControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
