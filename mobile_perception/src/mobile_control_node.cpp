#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <std_srvs/srv/trigger.hpp>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

class MobileControlNode : public rclcpp::Node
{
public:
    enum class GoalMode
    {
        POSE,
        STRAIGHT,
        ROTATE
    };

    MobileControlNode() : Node("mobile_control_node")
    {
        // 参数声明（保持不变）
        this->declare_parameter("control_frequency", 20.0);
        this->declare_parameter("max_v", 0.35);
        this->declare_parameter("min_v", 0.0);
        this->declare_parameter("max_w", 1.0);
        this->declare_parameter("accel_x_max", 0.5);
        this->declare_parameter("rotate_to_heading_angular_vel", 0.4);
        this->declare_parameter("rotate_to_heading_max_angular_accel", 1.0);
        this->declare_parameter("rotate_to_heading_kp", 1.2);
        this->declare_parameter("rotate_stop_yaw_error", 0.04);
        
        this->declare_parameter("k_rho", 0.8);
        this->declare_parameter("k_alpha", 1.5);
        this->declare_parameter("k_beta", -1.0);
        this->declare_parameter("max_forward_angle_error", 1.0);

        this->declare_parameter("angle_deadzone", 0.05);
        this->declare_parameter("dist_deadzone", 0.02);
        this->declare_parameter("near_target_dist", 0.3);
        this->declare_parameter("cmd_vel_filter_alpha", 0.3);
        this->declare_parameter("min_angular_vel", 0.05);
        this->declare_parameter("xy_tolerance", 0.02);
        this->declare_parameter("yaw_tolerance", 0.04);
        this->declare_parameter("goal_mode_linear_yaw_threshold", 0.08);
        this->declare_parameter("goal_mode_linear_lateral_threshold", 0.03);
        this->declare_parameter("goal_mode_rotate_xy_threshold", 0.03);
        this->declare_parameter("straight_near_goal_angular_max", 0.10);

        control_frequency_ = this->get_parameter("control_frequency").as_double();
        accel_x_max_ = this->get_parameter("accel_x_max").as_double();
        rotate_to_heading_angular_vel_ = this->get_parameter("rotate_to_heading_angular_vel").as_double();
        rotate_to_heading_max_angular_accel_ = this->get_parameter("rotate_to_heading_max_angular_accel").as_double();
        rotate_to_heading_kp_ = this->get_parameter("rotate_to_heading_kp").as_double();
        rotate_stop_yaw_error_ = this->get_parameter("rotate_stop_yaw_error").as_double();
        
        k_rho_ = this->get_parameter("k_rho").as_double();
        k_alpha_ = this->get_parameter("k_alpha").as_double();
        k_beta_ = this->get_parameter("k_beta").as_double();
        max_forward_angle_error_ = this->get_parameter("max_forward_angle_error").as_double();

        angle_deadzone_ = this->get_parameter("angle_deadzone").as_double();
        dist_deadzone_ = this->get_parameter("dist_deadzone").as_double();
        near_target_dist_ = this->get_parameter("near_target_dist").as_double();
        filter_alpha_ = this->get_parameter("cmd_vel_filter_alpha").as_double();
        min_angular_vel_ = this->get_parameter("min_angular_vel").as_double();
        xy_tolerance_ = this->get_parameter("xy_tolerance").as_double();
        yaw_tolerance_ = this->get_parameter("yaw_tolerance").as_double();
        goal_mode_linear_yaw_threshold_ = this->get_parameter("goal_mode_linear_yaw_threshold").as_double();
        goal_mode_linear_lateral_threshold_ = this->get_parameter("goal_mode_linear_lateral_threshold").as_double();
        goal_mode_rotate_xy_threshold_ = this->get_parameter("goal_mode_rotate_xy_threshold").as_double();
        straight_near_goal_angular_max_ = this->get_parameter("straight_near_goal_angular_max").as_double();

        // 2. 初始化 TF 和 通信
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("control/cmd_vel", 10);
        
        // 订阅目标点
        target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "control/ctrl_pose", 10, 
            std::bind(&MobileControlNode::targetCallback, this, std::placeholders::_1));

        // ======= [新增] 订阅odom获取实际速度 =======
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "feedback/odom", 10,
            std::bind(&MobileControlNode::odomCallback, this, std::placeholders::_1));

        // 服务
        status_service_ = this->create_service<std_srvs::srv::Trigger>(
            "control/get_control_status",
            std::bind(&MobileControlNode::getStatusCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));

        cancel_control_service_ = this->create_service<std_srvs::srv::Trigger>(
            "control/cancel_chassis_control",
            std::bind(&MobileControlNode::cancelControlCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        // 3. 定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / control_frequency_)),
            std::bind(&MobileControlNode::controlLoop, this));

        // 状态初始化
        get_map_pose_ = false;
        is_goal_ = false;
        is_cancelling_ = false;
        succeed_returned_ = false;
        stop_count_ = 0;
        goal_mode_ = GoalMode::POSE;

        // 滤波速度初始化
        filtered_cmd_vel_.linear.x = 0.0;
        filtered_cmd_vel_.angular.z = 0.0;
        
        // 实际速度初始化
        pre_cmd_vel_.linear.x = 0.0;
        pre_cmd_vel_.angular.z = 0.0;
    }

private:
    // 通信接口
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;  // [新增]
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_control_service_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 目标与状态
    geometry_msgs::msg::PoseStamped target_pose_odom_; 
    geometry_msgs::msg::Twist pre_cmd_vel_;          // [现在] 存储实际速度
    geometry_msgs::msg::Twist filtered_cmd_vel_;
    bool get_map_pose_;
    bool is_goal_;
    bool is_cancelling_;
    bool succeed_returned_;
    int stop_count_;
    GoalMode goal_mode_;
    
    // 参数
    double control_frequency_;
    double accel_x_max_;
    double rotate_to_heading_angular_vel_;
    double rotate_to_heading_max_angular_accel_;
    double rotate_to_heading_kp_;
    double rotate_stop_yaw_error_;
    double k_rho_, k_alpha_, k_beta_;
    double max_forward_angle_error_;
    double angle_deadzone_;
    double dist_deadzone_;
    double near_target_dist_;
    double filter_alpha_;
    double min_angular_vel_;
    double xy_tolerance_;
    double yaw_tolerance_;
    double goal_mode_linear_yaw_threshold_;
    double goal_mode_linear_lateral_threshold_;
    double goal_mode_rotate_xy_threshold_;
    double straight_near_goal_angular_max_;

    // ======= [新增] odom回调函数 =======
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 用odom反馈的实际速度更新pre_cmd_vel_
        pre_cmd_vel_.linear.x = msg->twist.twist.linear.x;
        pre_cmd_vel_.angular.z = msg->twist.twist.angular.z;
    }

    void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        try {
            geometry_msgs::msg::PoseStamped msg_transformed;
            if (msg->header.frame_id != "map") {
                 msg_transformed = tf_buffer_->transform(*msg, "map", tf2::durationFromSec(0.1));
            } else {
                msg_transformed = *msg;
            }
            target_pose_odom_ = msg_transformed;
            get_map_pose_ = true;
            is_goal_ = false;
            succeed_returned_ = false;
            stop_count_ = 0;
            const double input_dx = msg->pose.position.x;
            const double input_dy = msg->pose.position.y;
            const double input_yaw = tf2::getYaw(msg->pose.orientation);
            const double input_dist = std::hypot(input_dx, input_dy);

            if (input_dist < goal_mode_rotate_xy_threshold_ &&
                std::fabs(input_yaw) >= yaw_tolerance_)
            {
                goal_mode_ = GoalMode::ROTATE;
            }
            else if (std::fabs(input_dy) < goal_mode_linear_lateral_threshold_ &&
                     std::fabs(input_yaw) < goal_mode_linear_yaw_threshold_)
            {
                goal_mode_ = GoalMode::STRAIGHT;
            }
            else
            {
                goal_mode_ = GoalMode::POSE;
            }

            RCLCPP_INFO(this->get_logger(),
                        "New target received in map frame, mode=%s",
                        goal_mode_ == GoalMode::STRAIGHT ? "STRAIGHT" :
                        (goal_mode_ == GoalMode::ROTATE ? "ROTATE" : "POSE"));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Cannot transform goal to map: %s", ex.what());
        }
    }

    void getStatusCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (is_cancelling_)
        {
            response->message = "CANCEL CONTROL";
            response->success = true;
        }
        else if (is_goal_ && stop_count_ > 20)
        {
            if (!succeed_returned_)
            {
                response->message = "SUCCEED";
                response->success = true;
                succeed_returned_ = true;
            }
            else
            {
                response->message = "IDLE";
                response->success = true;
            }
        }
        else if ((get_map_pose_ && !is_goal_) || (is_goal_ && stop_count_ <= 20))
        {
            response->message = "CONTROL";
            response->success = true;
        }
        else
        {
            response->message = "IDLE";
            response->success = true;
        }
        
        RCLCPP_INFO(this->get_logger(), "Status query: %s", response->message.c_str());
    }

    void cancelControlCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (get_map_pose_ && !is_goal_)
        {
            is_cancelling_ = true;
            get_map_pose_ = false;
            is_goal_ = false;
            succeed_returned_ = false;
            stop_count_ = 0;
            response->message = "开始取消控制，正在平滑停止";
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Cancel control requested, starting smooth stop");
        }
        else
        {
            response->message = "当前不在控制中，无需取消";
            response->success = true;
        }
    }

    geometry_msgs::msg::Twist calculateNoReverseGracefulVelocity(const geometry_msgs::msg::Pose &target_pose)
    {
        geometry_msgs::msg::Twist cmd;
        
        double dx = target_pose.position.x;
        double dy = target_pose.position.y;
        double dyaw = tf2::getYaw(target_pose.orientation);

        double rho = std::hypot(dx, dy);
        double alpha = std::atan2(dy, dx);
        double beta = dyaw - alpha;
        
        alpha = std::atan2(std::sin(alpha), std::cos(alpha));
        beta = std::atan2(std::sin(beta), std::cos(beta));

        // 死区处理
        if (rho < dist_deadzone_) rho = 0.0;
        if (std::abs(alpha) < angle_deadzone_) alpha = 0.0;
        if (std::abs(beta) < angle_deadzone_) beta = 0.0;

        // 距离自适应增益
        double gain_scale = 1.0;
        if (rho < near_target_dist_ && rho > 0.0) {
            gain_scale = std::max(0.2, rho / near_target_dist_);
        }

        double v = 0.0;
        double w = 0.0;

        if (std::abs(alpha) > max_forward_angle_error_)
        {
            v = 0.0;
            double w_p = 2.0 * alpha * gain_scale;
            double max_w = this->get_parameter("max_w").as_double();
            w = std::clamp(w_p, -max_w, max_w);
            
            if (std::abs(w) < min_angular_vel_ && rho > dist_deadzone_) {
                w = (w > 0 ? min_angular_vel_ : -min_angular_vel_);
            }
        }
        else
        {
            v = k_rho_ * rho * gain_scale;
            // w = (k_alpha_ * alpha + k_beta_ * beta) * gain_scale;
            
            if (v < 0.0) v = 0.0;
            
            double angle_factor = 1.0 - (std::abs(alpha) / max_forward_angle_error_);
            v *= angle_factor;
            
            // if (std::abs(w) < min_angular_vel_ && rho > dist_deadzone_) {
            //     w = (w > 0 ? min_angular_vel_ : -min_angular_vel_);
            // }
             // ⭐ 关键修改点 ⭐
            if (rho < near_target_dist_)
            {
                // 近距离：只修 alpha，不修 beta（解耦最终姿态）
                w = k_alpha_ * alpha * gain_scale;
            }
            else
            {
                // 远距离：完整 rho–alpha–beta 控制
                w = (k_alpha_ * alpha + k_beta_ * beta) * gain_scale;
            }
        }

        double max_v = this->get_parameter("max_v").as_double();
        double max_w = this->get_parameter("max_w").as_double();
        
        cmd.linear.x = std::clamp(v, 0.0, max_v);
        cmd.angular.z = std::clamp(w, -max_w, max_w);

        return cmd;
    }

    geometry_msgs::msg::Twist calculateReverseGracefulVelocity(const geometry_msgs::msg::Pose &target_pose)
    {
        geometry_msgs::msg::Twist cmd;
        
        double dx = target_pose.position.x;
        double dy = target_pose.position.y;
        double dyaw = tf2::getYaw(target_pose.orientation);

        double rho = std::hypot(dx, dy);
        double alpha = std::atan2(dy, dx);
        double rear_alpha = M_PI - alpha;
        rear_alpha = std::atan2(std::sin(rear_alpha), std::cos(rear_alpha));
        double beta = dyaw - rear_alpha;
        beta = std::atan2(std::sin(beta), std::cos(beta));

        if (rho < dist_deadzone_) rho = 0.0;
        if (std::abs(rear_alpha) < angle_deadzone_) rear_alpha = 0.0;
        if (std::abs(beta) < angle_deadzone_) beta = 0.0;

        double gain_scale = 1.0;
        if (rho < near_target_dist_ && rho > 0.0) {
            gain_scale = std::max(0.2, rho / near_target_dist_);
        }

        double v = 0.0;
        double w = 0.0;

        if (std::abs(rear_alpha) > max_forward_angle_error_)
        {
            v = 0.0;
            double w_p = 2.0 * rear_alpha * gain_scale;
            double max_w = this->get_parameter("max_w").as_double();
            w = std::clamp(w_p, -max_w, max_w);
            
            if (std::abs(w) < min_angular_vel_ && rho > dist_deadzone_) {
                w = (w > 0 ? min_angular_vel_ : -min_angular_vel_);
            }
        }
        else
        {
            v = -k_rho_ * rho * gain_scale;
            w = -(k_alpha_ * rear_alpha + k_beta_ * beta) * gain_scale;
            
            if (v > 0.0) v = 0.0;
            
            double angle_factor = 1.0 - (std::abs(rear_alpha) / max_forward_angle_error_);
            v *= angle_factor;
            
            // if (std::abs(w) < min_angular_vel_ && rho > dist_deadzone_) {
            //     w = (w > 0 ? min_angular_vel_ : -min_angular_vel_);
            // }
            if (rho < near_target_dist_)
            {
                // 近距离：只修 alpha，不修 beta（解耦最终姿态）
                w = -(k_alpha_ * rear_alpha) * gain_scale;
            }
            else
            {
                // 远距离：完整 rho–alpha–beta 控制
                w = -(k_alpha_ * rear_alpha + k_beta_ * beta) * gain_scale;
            }
        }

        double max_v = this->get_parameter("max_v").as_double();
        double max_w = this->get_parameter("max_w").as_double();
        
        cmd.linear.x = std::clamp(v, -max_v, 0.0);
        cmd.angular.z = std::clamp(w, -max_w, max_w);

        return cmd;
    }

    geometry_msgs::msg::Twist computeRotateToHeadingCommand(
        const double &angular_distance_to_heading,
        const geometry_msgs::msg::Twist &current_velocity,
        const double &dt)
    {
        geometry_msgs::msg::Twist cmd_vel;
        const double abs_error = std::fabs(angular_distance_to_heading);

        // Stop rotating inside a small yaw deadband to avoid overshoot and chatter near the goal.
        if (abs_error < rotate_stop_yaw_error_) {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            return cmd_vel;
        }

        const double sign = angular_distance_to_heading > 0.0 ? 1.0 : -1.0;
        double angular_vel = rotate_to_heading_kp_ * angular_distance_to_heading;
        angular_vel = std::clamp(angular_vel,
                                 -rotate_to_heading_angular_vel_,
                                 rotate_to_heading_angular_vel_);
        
        const double min_feasible = current_velocity.angular.z - rotate_to_heading_max_angular_accel_ * dt;
        const double max_feasible = current_velocity.angular.z + rotate_to_heading_max_angular_accel_ * dt;
        
        cmd_vel.angular.z = std::clamp(angular_vel, min_feasible, max_feasible);

        double max_vel_to_stop = std::sqrt(2 * rotate_to_heading_max_angular_accel_ * abs_error);
        
        if (std::fabs(cmd_vel.angular.z) > max_vel_to_stop) {
            cmd_vel.angular.z = sign * max_vel_to_stop;
        }
        
        cmd_vel.linear.x = 0.0;
        return cmd_vel;
    }

    void controlLoop()
    {
        // [修改] 取消控制中，pre_cmd_vel_ 由odom更新，此处不再修改它
        if (is_cancelling_)
        {
            geometry_msgs::msg::Twist cmd_vel;
            double dt = 1.0 / control_frequency_;
            
            // 从odom获取的实际速度
            double actual_vel_x = pre_cmd_vel_.linear.x;
            double actual_vel_w = pre_cmd_vel_.angular.z;
            
            // 平滑减速
            double linear_decel = accel_x_max_ * dt;
            if (actual_vel_x > 0.0)
            {
                cmd_vel.linear.x = std::max(0.0, actual_vel_x - linear_decel);
            }
            else if (actual_vel_x < 0.0)
            {
                cmd_vel.linear.x = std::min(0.0, actual_vel_x + linear_decel);
            }
            else
            {
                cmd_vel.linear.x = 0.0;
            }
            
            double angular_decel = rotate_to_heading_max_angular_accel_ * dt;
            if (actual_vel_w > 0.0)
            {
                cmd_vel.angular.z = std::max(0.0, actual_vel_w - angular_decel);
            }
            else if (actual_vel_w < 0.0)
            {
                cmd_vel.angular.z = std::min(0.0, actual_vel_w + angular_decel);
            }
            else
            {
                cmd_vel.angular.z = 0.0;
            }
            
            // 检查是否已经停止
            const double velocity_threshold = 0.01;
            if (std::abs(cmd_vel.linear.x) < velocity_threshold && 
                std::abs(cmd_vel.angular.z) < velocity_threshold)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                is_cancelling_ = false;
                succeed_returned_ = false;
                stop_count_ = 0;
                RCLCPP_INFO(this->get_logger(), "Cancel control completed, robot stopped");
            }
            
            cmd_vel_pub_->publish(cmd_vel);
            // [重要] 不再更新pre_cmd_vel_，它由odom回调更新
            return;
        }
        
        if (stop_count_ > 40)
        {
          return;  
        } 
        if(!get_map_pose_) return;

        geometry_msgs::msg::PoseStamped target_pose_in_base;
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "base_link", "map", tf2::TimePointZero);
            tf2::doTransform(target_pose_odom_, target_pose_in_base, transform);
        } catch (tf2::TransformException &ex) {
            return;
        }

        geometry_msgs::msg::Twist cmd_vel;
        double dist = std::hypot(target_pose_in_base.pose.position.x, 
                                target_pose_in_base.pose.position.y);
        double yaw_error = tf2::getYaw(target_pose_in_base.pose.orientation);

        bool is_reverse_mode = (target_pose_in_base.pose.position.x < 0.0 && 
                            std::abs(target_pose_in_base.pose.position.y) < 0.5);
        
        // 状态机
        if(goal_mode_ != GoalMode::STRAIGHT && std::fabs(dist) < xy_tolerance_)
        {
            double dt = 1.0 / control_frequency_;
            cmd_vel = computeRotateToHeadingCommand(yaw_error, pre_cmd_vel_, dt);
            RCLCPP_INFO(this->get_logger(), "[Rotate Mode] yaw_error: %.4f rad (%.2f deg), cmd_vel: linear=%.3f, angular=%.3f",
                        yaw_error, yaw_error * 180.0 / M_PI, cmd_vel.linear.x, cmd_vel.angular.z);
        }
        else
        {
            if(is_reverse_mode)
            {
                cmd_vel = calculateReverseGracefulVelocity(target_pose_in_base.pose);
            }
            else
            {
                cmd_vel = calculateNoReverseGracefulVelocity(target_pose_in_base.pose);
                // RCLCPP_INFO(this->get_logger(), "cmd_vel: %.3f, %.3f", cmd_vel.linear.x, cmd_vel.angular.z);
            }

            if (goal_mode_ == GoalMode::STRAIGHT && dist < near_target_dist_)
            {
                cmd_vel.angular.z = std::clamp(cmd_vel.angular.z,
                                               -straight_near_goal_angular_max_,
                                               straight_near_goal_angular_max_);
            }

            // 加速度平滑（基于实际速度 pre_cmd_vel_）
            double dt = 1.0 / control_frequency_;
            if (cmd_vel.linear.x < pre_cmd_vel_.linear.x)
            {
                cmd_vel.linear.x = std::max(cmd_vel.linear.x, pre_cmd_vel_.linear.x - accel_x_max_ * dt);
            }
            if (cmd_vel.linear.x >= pre_cmd_vel_.linear.x)
            {
                cmd_vel.linear.x = std::min(cmd_vel.linear.x, pre_cmd_vel_.linear.x + accel_x_max_ * dt);
            }

            if(cmd_vel.angular.z < pre_cmd_vel_.angular.z)
            {
                cmd_vel.angular.z = std::max(cmd_vel.angular.z, pre_cmd_vel_.angular.z - rotate_to_heading_max_angular_accel_ * dt);
            }
            if(cmd_vel.angular.z >= pre_cmd_vel_.angular.z)
            {
                cmd_vel.angular.z = std::min(cmd_vel.angular.z, pre_cmd_vel_.angular.z + rotate_to_heading_max_angular_accel_ * dt);
            }
        }
        // 输出低通滤波
        filtered_cmd_vel_.linear.x = filter_alpha_ * cmd_vel.linear.x + 
                                     (1.0 - filter_alpha_) * filtered_cmd_vel_.linear.x;
        filtered_cmd_vel_.angular.z = filter_alpha_ * cmd_vel.angular.z + 
                                      (1.0 - filter_alpha_) * filtered_cmd_vel_.angular.z;
        

        // 判断到达
        if(goal_mode_ == GoalMode::STRAIGHT)
        {
            if(std::fabs(dist) < xy_tolerance_)
            {
                is_goal_ = true;
            }
        }
        else if(std::fabs(dist) < xy_tolerance_ && std::fabs(yaw_error) < yaw_tolerance_)
        {
            is_goal_ = true;
        }

        if(is_goal_)
        {
            double decay_factor = std::max(0.0, 1.0 - stop_count_ / 20.0);
            filtered_cmd_vel_.linear.x = 0.0;
            filtered_cmd_vel_.angular.z = 0.0;
            
            stop_count_++;
            
            if (stop_count_ <= 20) {
                cmd_vel_pub_->publish(filtered_cmd_vel_);
                if (stop_count_ == 1) {
                    RCLCPP_INFO(this->get_logger(),
                                "Goal reached with final error: dist=%.4f m, yaw=%.4f rad (%.2f deg), tolerance: dist<=%.3f m, yaw<=%.3f rad",
                                dist, yaw_error, yaw_error * 180.0 / M_PI, xy_tolerance_, yaw_tolerance_);
                }
                if (stop_count_ % 10 == 0) {
                    RCLCPP_INFO(this->get_logger(), "Goal Reached. Smooth stopping... %d", stop_count_);
                }
            }
        } 
        else
        {
            stop_count_ = 0;
            cmd_vel_pub_->publish(filtered_cmd_vel_);
            
            static int loop_count = 0;
            if (loop_count++ % (int)(control_frequency_ / 2) == 0) {
                RCLCPP_INFO(this->get_logger(), "Vel: %.3f, Ang: %.3f", 
                           filtered_cmd_vel_.linear.x, filtered_cmd_vel_.angular.z);
            }
        }

        // [重要] 不再更新pre_cmd_vel_，它由odom回调更新
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
