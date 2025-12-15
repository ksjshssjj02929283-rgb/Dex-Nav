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
    MobileControlNode() : Node("mobile_control_node")
    {
        // 1. 参数声明
        this->declare_parameter("control_frequency", 20.0);
        this->declare_parameter("max_v", 0.35);
        this->declare_parameter("min_v", 0.0); // 最小非零速度
        this->declare_parameter("max_w", 1.0);
        this->declare_parameter("accel_x_max", 0.5);
        this->declare_parameter("rotate_to_heading_angular_vel", 0.4);
        this->declare_parameter("rotate_to_heading_max_angular_accel", 1.0);
        
        // Graceful Controller Gains
        this->declare_parameter("k_rho", 0.8);
        this->declare_parameter("k_alpha", 1.5);
        this->declare_parameter("k_beta", -1.0); 
        
        // [新增] 允许前进的最大朝向误差 (弧度)，超过这个角度则原地旋转
        // 默认 1.0 弧度 (约 57度)。意味着只有目标在前方 +-57度扇区内，车才会走动
        this->declare_parameter("max_forward_angle_error", 1.0); 

        // 获取参数
        control_frequency_ = this->get_parameter("control_frequency").as_double();
        accel_x_max_ = this->get_parameter("accel_x_max").as_double();
        rotate_to_heading_angular_vel_ = this->get_parameter("rotate_to_heading_angular_vel").as_double();
        rotate_to_heading_max_angular_accel_ = this->get_parameter("rotate_to_heading_max_angular_accel").as_double();
        
        k_rho_ = this->get_parameter("k_rho").as_double();
        k_alpha_ = this->get_parameter("k_alpha").as_double();
        k_beta_ = this->get_parameter("k_beta").as_double();
        max_forward_angle_error_ = this->get_parameter("max_forward_angle_error").as_double();

        // 2. 初始化 TF 和 通信
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("control/cmd_vel", 10);
        
        // 订阅 Odom 坐标系下的目标点
        target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "control/ctrl_pose", 10, 
            std::bind(&MobileControlNode::targetCallback, this, std::placeholders::_1));

        // 创建状态查询服务
        status_service_ = this->create_service<std_srvs::srv::Trigger>(
            "control/get_control_status",
            std::bind(&MobileControlNode::getStatusCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));

        // 创建取消控制服务
        cancel_control_service_ = this->create_service<std_srvs::srv::Trigger>(
            "control/cancel_control",
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
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_control_service_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::PoseStamped target_pose_odom_; 
    geometry_msgs::msg::Twist pre_cmd_vel_;
    bool get_map_pose_;
    bool is_goal_;
    bool is_cancelling_;
    bool succeed_returned_;  // 标记是否已返回过 SUCCEED
    int stop_count_;
    
    double control_frequency_;
    double accel_x_max_;
    double rotate_to_heading_angular_vel_;
    double rotate_to_heading_max_angular_accel_;
    double k_rho_, k_alpha_, k_beta_;
    double max_forward_angle_error_;

    void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        try {
            geometry_msgs::msg::PoseStamped msg_transformed;
            if (msg->header.frame_id != "odom") {
                 msg_transformed = tf_buffer_->transform(*msg, "odom", tf2::durationFromSec(0.1));
            } else {
                msg_transformed = *msg;
            }
            target_pose_odom_ = msg_transformed;
            get_map_pose_ = true;
            is_goal_ = false;
            succeed_returned_ = false;  // 新目标到达，重置SUCCEED一次性标志
            stop_count_ = 0;
            RCLCPP_INFO(this->get_logger(), "New target received in Odom frame");
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Cannot transform goal to odom: %s", ex.what());
        }
    }

    /**
     * @brief 状态查询服务回调函数
     * @param request 服务请求（Trigger服务不需要请求参数）
     * @param response 服务响应，包含状态信息
     */
    void getStatusCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        // 状态判断优先级：
        // 1. "取消控制中"：正在取消控制（is_cancelling_ == true）
        // 2. "SUCCEED"：已到达目标并停止，且第一次返回此状态（is_goal_ == true 且 stop_count_ > 20 且 !succeed_returned_）
        // 3. "IDLE"：已到达目标并停止，且已返回过 SUCCEED（is_goal_ == true 且 stop_count_ > 20 且 succeed_returned_）
        // 4. "CONTROL"：正在执行控制任务（get_map_pose_ == true 且 !is_goal_）或正在停止（is_goal_ && stop_count_ <= 20）
        if (is_cancelling_)
        {
            response->message = "CANCEL CONTROL";
            response->success = true;
        }
        else if (is_goal_ && stop_count_ > 20)
        {
            // 已到达目标并停止
            if (!succeed_returned_)
            {
                // 第一次返回 SUCCEED
                response->message = "SUCCEED";
                response->success = true;
                succeed_returned_ = true;
                RCLCPP_INFO(this->get_logger(), "Goal reached, returning SUCCEED");
            }
            else
            {
                // 之后的 call 返回 IDLE
                response->message = "IDLE";
                response->success = true;
                RCLCPP_INFO(this->get_logger(), "Already succeeded, returning IDLE");
            }
        }
        else if ((get_map_pose_ && !is_goal_) || (is_goal_ && stop_count_ <= 20))
        {
            response->message = "CONTROL";
            response->success = true;
        }
        else
        {
            // 默认空闲
            response->message = "IDLE";
            response->success = true;
        }
        
        RCLCPP_INFO(this->get_logger(), "Status query: %s", response->message.c_str());
    }

    /**
     * @brief 取消控制服务回调函数
     * @param request 服务请求（Trigger服务不需要请求参数）
     * @param response 服务响应
     */
    void cancelControlCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        // 如果机器人在控制中，则启动取消控制流程
        if (get_map_pose_ && !is_goal_)
        {
            is_cancelling_ = true;
            get_map_pose_ = false;  // 清除目标
            is_goal_ = false;
            succeed_returned_ = false;  // 重置SUCCEED标志
            stop_count_ = 0;
            response->message = "开始取消控制，正在平滑停止";
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Cancel control requested, starting smooth stop");
        }
        else
        {
            response->message = "当前不在控制中，无需取消";
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Cancel control requested but not in control state");
        }
    }

    /**
     * @brief 核心修改：禁止倒车的 Graceful Velocity 计算
     */
    geometry_msgs::msg::Twist calculateNoReverseGracefulVelocity(const geometry_msgs::msg::Pose &target_pose)
    {
        geometry_msgs::msg::Twist cmd;
        
        double dx = target_pose.position.x;
        double dy = target_pose.position.y;
        double dyaw = tf2::getYaw(target_pose.orientation);
        RCLCPP_INFO(this->get_logger(), "dyaw: %0.3f", dyaw);

        // 1. 计算误差变量
        double rho = std::hypot(dx, dy);      // 距离
        double alpha = std::atan2(dy, dx);    // 目标方位角（相对于当前车头）
        double beta = dyaw - alpha;           // 目标朝向误差
        RCLCPP_INFO(this->get_logger(), "The alpha: %0.3f, the beta %0.3f, the dx: %0.3f, dy: %0.3f",
                    alpha, beta, dx, dy);
        // 规范化角度到 -PI ~ PI
        alpha = std::atan2(std::sin(alpha), std::cos(alpha));
        beta = std::atan2(std::sin(beta), std::cos(beta));

        double v = 0.0;
        double w = 0.0;

        // 2. 判断目标是否在前方扇区内 (No Reverse Logic)
        // 如果 alpha 绝对值很大，说明目标在侧面或后面，此时不能给线速度
        if (std::abs(alpha) > max_forward_angle_error_)
        {
            // 【策略 A：原地转向】
            // 目标不在视野前方，强制线速度为0，只旋转对准目标
            v = 0.0;
            
            // 使用简单的 P 控制旋转，或者使用固定角速度
            double w_p = 2.0 * alpha; // P gain for rotation
            // 限制最大旋转速度
            double max_w = this->get_parameter("max_w").as_double();
            w = std::clamp(w_p, -max_w, max_w);
            
            // 如果角度误差非常大，确保角速度足够大以克服静摩擦，但也别太大
            if (std::abs(w) < 0.1) w = (alpha > 0 ? 0.1 : -0.1);
        }
        else
        {
            // 【策略 B：Graceful Control 平滑逼近】
            // 目标在前方，使用 Lyapunov 控制律
            v = k_rho_ * rho;
            w = k_alpha_ * alpha + k_beta_ * beta;

            // 再次确保线速度不为负 (Strict No Reverse)
            // 理论上 rho 总是正的，但在浮点数计算或极端 alpha 情况下做个防护
            if (v < 0.0) v = 0.0;

            // 当角度接近边界时，适当减速线速度，防止转弯过大
            // 简单的线性衰减：角度越大，允许的最大线速度越小
            double angle_factor = 1.0 - (std::abs(alpha) / max_forward_angle_error_);
            v *= angle_factor;
        }

        // 3. 速度限幅
        double max_v = this->get_parameter("max_v").as_double();
        double max_w = this->get_parameter("max_w").as_double();
        
        cmd.linear.x = std::clamp(v, 0.0, max_v); // 下限锁死为 0.0
        cmd.angular.z = std::clamp(w, -max_w, max_w);

        return cmd;
    }


        // [新增] 后退模式检测和后向控制
    geometry_msgs::msg::Twist calculateReverseGracefulVelocity(
        const geometry_msgs::msg::Pose &target_pose)
    {
        geometry_msgs::msg::Twist cmd;
        
        // 后退时：从"后保险杠视角"看目标
        // 在base_link下，x为负表示目标在后方
        double dx = target_pose.position.x;  // 在base_link下，x为负
        double dy = target_pose.position.y;
        double dyaw = tf2::getYaw(target_pose.orientation);

        // 距离（用绝对值，因为后退时x为负）
        double rho = std::hypot(dx, dy);
        // 计算相对于后退方向的角度（镜像前向逻辑）
        // 后退时"前方"是base_link的-x方向
        double alpha = std::atan2(dy, dx);  // 目标相对于当前朝向的角度
        
        // 关键：切换到"后视角"坐标系
        // 对于后退，我们假装机器人后方是前方
        // 所以角度需要反向，且我们关心的是目标相对于后退方向的偏差
        double rear_alpha = M_PI - alpha;  // 转换到后退视角
        rear_alpha = std::atan2(std::sin(rear_alpha), std::cos(rear_alpha));
        
        double beta = dyaw - rear_alpha;  // 目标朝向误差
        beta = std::atan2(std::sin(beta), std::cos(beta));

        RCLCPP_INFO(this->get_logger(), "Reverse alpha: %0.3f, the beta %0.3f, the dx: %0.3f, dy: %0.3f",
                    alpha, beta, dx, dy);

        double v = 0.0;
        double w = 0.0;

        // 后退控制逻辑：镜像前向但速度为负
        // 只允许在"后方扇区"内后退
        if (std::abs(rear_alpha) > max_forward_angle_error_)
        {
            // 目标不在后方视野内，原地旋转调整
            v = 0.0;
            double w_p = 2.0 * rear_alpha;
            double max_w = this->get_parameter("max_w").as_double();
            w = std::clamp(w_p, -max_w, max_w);
            
            if (std::abs(w) < 0.1) w = (rear_alpha > 0 ? 0.1 : -0.1);
        }
        else
        {
            // 在后方扇区内，执行后退
            v = -k_rho_ * rho;  // 速度为负！
            w = -k_alpha_ * rear_alpha - k_beta_ * beta;  // 转向镜像
            
            if (v > 0.0) v = 0.0;  // 确保不会变成正速度
            
            // 角度越大，后退速度越小
            double angle_factor = 1.0 - (std::abs(rear_alpha) / max_forward_angle_error_);
            v *= angle_factor;
        }

        // 速度限幅
        double max_v = this->get_parameter("max_v").as_double();
        double max_w = this->get_parameter("max_w").as_double();
        
        cmd.linear.x = std::clamp(v, -max_v, 0.0);  // 速度限制在负数范围
        cmd.angular.z = std::clamp(w, -max_w, max_w);

        return cmd;
    }


    geometry_msgs::msg::Twist computeRotateToHeadingCommand(
        const double & angular_distance_to_heading,
        const geometry_msgs::msg::Twist & current_velocity,
        const double & dt)
    {
        geometry_msgs::msg::Twist cmd_vel;
        const double sign = angular_distance_to_heading > 0.0 ? 1.0 : -1.0;
        
        double angular_vel = sign * rotate_to_heading_angular_vel_;
        
        const double min_feasible = current_velocity.angular.z - rotate_to_heading_max_angular_accel_ * dt;
        const double max_feasible = current_velocity.angular.z + rotate_to_heading_max_angular_accel_ * dt;
        
        cmd_vel.angular.z = std::clamp(angular_vel, min_feasible, max_feasible);

        double max_vel_to_stop = std::sqrt(2 * rotate_to_heading_max_angular_accel_ * std::fabs(angular_distance_to_heading));
        
        if (std::fabs(cmd_vel.angular.z) > max_vel_to_stop) {
            cmd_vel.angular.z = sign * max_vel_to_stop;
        }
        
        cmd_vel.linear.x = 0.0;
        return cmd_vel;
    }

        
    // [修改] 主循环中的检测逻辑
    void controlLoop()
    {
        // --- [新增] 取消控制检测和平滑减速 ---
        if (is_cancelling_)
        {
            geometry_msgs::msg::Twist cmd_vel;
            double dt = 1.0 / control_frequency_;
            
            // 平滑减速：线速度按加速度限制逐渐减到0
            double linear_decel = accel_x_max_ * dt;
            if (pre_cmd_vel_.linear.x > 0.0)
            {
                cmd_vel.linear.x = std::max(0.0, pre_cmd_vel_.linear.x - linear_decel);
            }
            else if (pre_cmd_vel_.linear.x < 0.0)
            {
                cmd_vel.linear.x = std::min(0.0, pre_cmd_vel_.linear.x + linear_decel);
            }
            else
            {
                cmd_vel.linear.x = 0.0;
            }
            
            // 平滑减速：角速度按加速度限制逐渐减到0
            double angular_decel = rotate_to_heading_max_angular_accel_ * dt;
            if (pre_cmd_vel_.angular.z > 0.0)
            {
                cmd_vel.angular.z = std::max(0.0, pre_cmd_vel_.angular.z - angular_decel);
            }
            else if (pre_cmd_vel_.angular.z < 0.0)
            {
                cmd_vel.angular.z = std::min(0.0, pre_cmd_vel_.angular.z + angular_decel);
            }
            else
            {
                cmd_vel.angular.z = 0.0;
            }
            
            // 检查是否已经停止（速度接近0）
            const double velocity_threshold = 0.01;
            if (std::abs(cmd_vel.linear.x) < velocity_threshold && 
                std::abs(cmd_vel.angular.z) < velocity_threshold)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                is_cancelling_ = false;  // 停止完成，退出取消控制状态
                succeed_returned_ = false;  // 重置SUCCEED标志
                stop_count_ = 0;
                RCLCPP_INFO(this->get_logger(), "Cancel control completed, robot stopped");
            }
            
            cmd_vel_pub_->publish(cmd_vel);
            pre_cmd_vel_ = cmd_vel;
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
                "base_link", "odom", tf2::TimePointZero);
            tf2::doTransform(target_pose_odom_, target_pose_in_base, transform);
        } catch (tf2::TransformException &ex) {
            return;
        }

        geometry_msgs::msg::Twist cmd_vel;
        double dist = std::hypot(target_pose_in_base.pose.position.x, 
                                target_pose_in_base.pose.position.y);
        double yaw_error = tf2::getYaw(target_pose_in_base.pose.orientation);

        // 容差设置
        double xy_tolerance = 0.03;
        double yaw_tolerance = 0.02;

        // --- [新增] 后退模式检测 ---
        // 判断条件：目标在base_link下x为负，且y接近0（纯后退）
        // 或者你可以通过topic标志位来触发
        bool is_reverse_mode = (target_pose_in_base.pose.position.x < 0.0 && 
                            std::abs(target_pose_in_base.pose.position.y) < 0.5);
        
        // --- 状态机 ---
        if(std::fabs(dist) < xy_tolerance)
        {
            // 到位后旋转调整朝向
            double dt = 1.0 / control_frequency_;
            cmd_vel = computeRotateToHeadingCommand(yaw_error, pre_cmd_vel_, dt);
        }
        else
        {
            // [修改] 根据模式选择控制函数
            if(is_reverse_mode)
            {
                cmd_vel = calculateReverseGracefulVelocity(target_pose_in_base.pose);
                RCLCPP_INFO(this->get_logger(), "[REVERSE MODE] x=%.2f, y=%.2f", 
                        target_pose_in_base.pose.position.x, 
                        target_pose_in_base.pose.position.y);
            }
            else
            {
                cmd_vel = calculateNoReverseGracefulVelocity(target_pose_in_base.pose);
            }

            // 加速度平滑
            if(cmd_vel.linear.x < pre_cmd_vel_.linear.x)
            {
                cmd_vel.linear.x = std::max(cmd_vel.linear.x, pre_cmd_vel_.linear.x - 0.05 * accel_x_max_);
            }
            if(cmd_vel.linear.x >= pre_cmd_vel_.linear.x)
            {
                cmd_vel.linear.x = std::min(cmd_vel.linear.x, pre_cmd_vel_.linear.x + 0.02 * accel_x_max_);
            }
        }

        // 判断到达
        if(std::fabs(dist) < xy_tolerance && std::fabs(yaw_error) < yaw_tolerance)
        {
            is_goal_ = true;
        }

        if(is_goal_)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            stop_count_++;
            if(stop_count_ <= 20) {
                cmd_vel_pub_->publish(cmd_vel);
                if(stop_count_ % 10 == 0) {
                    RCLCPP_INFO(this->get_logger(), "Goal Reached. Stopping... %d", stop_count_);
                }
            }
            RCLCPP_INFO(this->get_logger(), "stop count_ %d", stop_count_);
        } 
        else 
        {
            stop_count_ = 0;
            cmd_vel_pub_->publish(cmd_vel);
            RCLCPP_INFO(this->get_logger(), "vel=%0.3f, ang=%0.3f", cmd_vel.linear.x, cmd_vel.angular.z);
        }

        pre_cmd_vel_ = cmd_vel;
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
