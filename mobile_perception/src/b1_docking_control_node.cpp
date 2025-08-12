
#include <fstream>
#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
// #include "opennav_docking/controller.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_2d_utils/conversions.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <chrono> 
#include "tf2/utils.h"
#include <yaml-cpp/yaml.h>
#include "geometry_msgs/msg/twist.hpp"

#include <ament_index_cpp/get_package_prefix.hpp> 
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "nav2_graceful_controller/smooth_control_law.hpp"

class B1DockingControlNode : public rclcpp::Node
{
    public:
        B1DockingControlNode(const rclcpp::NodeOptions& options)
        : Node("b1_docking_control_node", options)
        {
            RCLCPP_INFO(this->get_logger(), "Starting b1_docking_control_node");

            // 获取用户主目录路径
            file_path_ = "/home/Projects/b1_main_v1/b1_devel/b1_perception/b1_docking_control_node.log";

            // 打开文件（追加模式）
            log_file_.open(file_path_, std::ios::app);
            if (!log_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open log file: %s", file_path_.c_str());
            throw std::runtime_error("Failed to open log file");
            }

            RCLCPP_INFO(this->get_logger(), "Logging data to: %s", file_path_.c_str());

            // 写入CSV表头（如果文件是新建的）
            if (log_file_.tellp() == 0) {
            log_file_ << "timestamp,counter,node_name" << std::endl;
            }

            // Get parameters from the node
            this->declare_parameter("controller.k_phi", 2.0);
            this->declare_parameter("controller.k_delta", 1.0);
            this->declare_parameter("controller.beta", 0.4);
            this->declare_parameter("controller.lambda", 2.0);
            this->declare_parameter("controller.v_linear_min", 0.1);
            this->declare_parameter("controller.v_linear_max", 0.25);
            this->declare_parameter("controller.v_angular_max", 0.75);
            this->declare_parameter("controller.slowdown_radius", 0.25);
            this->declare_parameter("controller.rotate_to_heading_angular_vel", 0.5);
            this->declare_parameter("controller.rotate_to_heading_max_angular_accel", 0.5);

            this->declare_parameter("goal_coffe.position.x", 0.0);
            this->declare_parameter("goal_coffe.position.y", 0.0);
            this->declare_parameter("goal_coffe.position.z", 0.0);
            this->declare_parameter("goal_coffe.orientation.x", 0.0);
            this->declare_parameter("goal_coffe.orientation.y", 0.0);
            this->declare_parameter("goal_coffe.orientation.z", 0.0);
            this->declare_parameter("goal_coffe.orientation.w", 1.0);

            this->declare_parameter("goal_table.position.x", 0.0);
            this->declare_parameter("goal_table.position.y", 0.0);
            this->declare_parameter("goal_table.position.z", 0.0);
            this->declare_parameter("goal_table.orientation.x", 0.0);
            this->declare_parameter("goal_table.orientation.y", 0.0);
            this->declare_parameter("goal_table.orientation.z", 0.0);
            this->declare_parameter("goal_table.orientation.w", 1.0);

            this->declare_parameter("is_coffe", true);
            accel_x_max_ = 0.3;
            
            SetParams();

            // Set parameters
            control_law_ = std::make_unique<nav2_graceful_controller::SmoothControlLaw>(
                k_phi_, k_delta_, beta_, lambda_, slowdown_radius_, v_linear_min_, v_linear_max_, v_angular_max_
            );
            RCLCPP_INFO(this->get_logger(), "****");
            //
            GetTarPose_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                            std::bind(&B1DockingControlNode::GetTargetPose, this));
            
            ControlLoop_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), 
                            std::bind(&B1DockingControlNode::ControllLoop, this));
            ControlLoop_timer_->cancel(); 

            target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/control/target_pose", 10, std::bind(&B1DockingControlNode::TargetPoseCallback, this, std::placeholders::_1)
            );
            cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/control/cmd_vel", 10);

            tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);


           
        }

        ~B1DockingControlNode()
        {
            if (log_file_.is_open()) {
                log_file_.close();
                RCLCPP_INFO(this->get_logger(), "Log file closed");
            }
        }
        bool CheckNewGoal(geometry_msgs::msg::Pose target_pose, geometry_msgs::msg::Pose current_pose)
        {
            // Implement your logic to check if the target pose is new
            // Return true if the target pose is new, false otherwise
            if(target_pose.position.x != current_pose.position.x || target_pose.position.y != current_pose.position.y
                || target_pose.orientation.x != current_pose.orientation.x || target_pose.orientation.y != current_pose.orientation.y
                || target_pose.orientation.z != current_pose.orientation.z || target_pose.orientation.w != current_pose.orientation.w)
            {
                return true;
            }
            return false;
        }
        void TargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {   
            if(!CheckNewGoal(msg->pose, target_map_pose_.pose))
            {
                return;
            }
            
            target_map_pose_ = *msg;
            RCLCPP_INFO(this->get_logger(), "Target pose received!");
            get_map_pose_ = true;
            is_goal_ = false;
            get_tar_pose_ =false;
            if(!ControlLoop_timer_->is_canceled())
            {
                ControlLoop_timer_->cancel();
            }
            if(!GetTarPose_timer_->is_canceled())
            {
                GetTarPose_timer_->cancel();
            }
            GetTarPose_timer_->reset();
            ControlLoop_timer_->reset();
        }

        void SetParams()
        {
            // TODO: Set params
            this->get_parameter("controller.k_phi", k_phi_);
            this->get_parameter("controller.k_delta", k_delta_);
            this->get_parameter("controller.beta", beta_);
            this->get_parameter("controller.lambda", lambda_);
            this->get_parameter("controller.v_linear_min", v_linear_min_);
            this->get_parameter("controller.v_linear_max", v_linear_max_);
            this->get_parameter("controller.v_angular_max", v_angular_max_);
            this->get_parameter("controller.slowdown_radius", slowdown_radius_);
            this->get_parameter("controller.rotate_to_heading_max_angular_accel", rotate_to_heading_max_angular_accel_);
            this->get_parameter("controller.rotate_to_heading_angular_vel", rotate_to_heading_angular_vel_);

            is_coffe_ = this->get_parameter("is_coffe").as_bool();
        }
        

        void GetTargetPose()
        {
            if(!get_map_pose_ )
            {
                RCLCPP_INFO(this->get_logger(), "No target received!");
                return;
            }
            if(is_goal_)
            {
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                GetTarPose_timer_->cancel();
                return;
            }
            target_map_pose_.header.frame_id = "map";
            target_map_pose_.header.stamp = rclcpp::Time(0);
            try{
                tf2_buffer_->transform(target_map_pose_, target_pose_, "base_link");
                RCLCPP_DEBUG(this->get_logger(), 
                            "Transformed target pose in base_link: x=%.2f, y=%.2f, theta=%.2f",
                            target_pose_.pose.position.x,
                            target_pose_.pose.position.y,
                            tf2::getYaw(target_pose_.pose.orientation));
                get_tar_pose_ = true;
            } catch(tf2::TransformException &ex){
                RCLCPP_ERROR(this->get_logger(), "Failed to transform target pose: %s", ex.what());
            }

        }

        void ControllLoop()
        {
            if(!get_map_pose_)
            {
                RCLCPP_INFO(this->get_logger(), "No target received, not controlling");
                return;
            }
            geometry_msgs::msg::Twist cmd_vel;
            geometry_msgs::msg::PoseStamped target_pose_tmp = target_pose_;
            // target_pose_tmp.pose.position.x += 0.25;
            RCLCPP_INFO(this->get_logger(), "Target pose in base_link frame: %f, %f, %f", target_pose_tmp.pose.position.x, target_pose_tmp.pose.position.y, target_pose_tmp.pose.position.z);
            double dis = hypot(target_pose_.pose.position.x, target_pose_.pose.position.y);
            if(fabs(dis) < 0.03)
            {
                RCLCPP_INFO(this->get_logger(), "In Rotate");
                cmd_vel = ComputeRotateToHeadingCommand(tf2::getYaw(target_pose_.pose.orientation), pre_cmd_vel_, 0.02);
            }
            else{
                cmd_vel = control_law_->calculateRegularVelocity(target_pose_tmp.pose, false);
                if(cmd_vel.linear.x < pre_cmd_vel_.linear.x)
                {
                    cmd_vel.linear.x = std::max(cmd_vel.linear.x, pre_cmd_vel_.linear.x - 0.02 * accel_x_max_);
                }
                if(cmd_vel.linear.x >= pre_cmd_vel_.linear.x)
                {
                    cmd_vel.linear.x = std::min(cmd_vel.linear.x, pre_cmd_vel_.linear.x + 0.02 * accel_x_max_);
                }
                // RCLCPP_INFO(this->get_logger(), "control cmd_vel: %f, %f", cmd_vel.linear.x, cmd_vel.angular.z);
            }
            if(fabs(dis) < 0.03 && fabs(tf2::getYaw(target_pose_.pose.orientation)) < 0.02)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;

                is_goal_ = true;
            }

            if(is_goal_ && get_tar_pose_)
            {
                RCLCPP_INFO(this->get_logger(), "****Goal Reached****");
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                cmd_vel_pub_->publish(cmd_vel);
                // 如何解决需要多次发布0速度才能停止的问题
                count++;
                if(count > 50)
                {
                    count = 0;
                    ControlLoop_timer_->cancel();
                    GetTarPose_timer_->cancel();
                    return;
                }

            }
            is_goal_ = false;
            log_file_ << "linear: " << cmd_vel.linear.x << " angular: " << cmd_vel.angular.z << std::endl;
            log_file_ << "Target pose in base_link frame: " << target_pose_tmp.pose.position.x << " " << target_pose_tmp.pose.position.y << " " << tf2::getYaw(target_pose_tmp.pose.orientation) << std::endl;
            cmd_vel_pub_->publish(cmd_vel);
            pre_cmd_vel_ = cmd_vel;
        }

        geometry_msgs::msg::Twist ComputeRotateToHeadingCommand(
        const double & angular_distance_to_heading,
        const geometry_msgs::msg::Twist & current_velocity,
        const double & dt)
        {
            geometry_msgs::msg::Twist cmd_vel;
            const double sign = angular_distance_to_heading > 0.0 ? 1.0 : -1.0;
            const double angular_vel = sign * rotate_to_heading_angular_vel_;
            const double min_feasible_angular_speed =
                current_velocity.angular.z - rotate_to_heading_max_angular_accel_ * dt;
            const double max_feasible_angular_speed =
                current_velocity.angular.z + rotate_to_heading_max_angular_accel_ * dt;
            cmd_vel.angular.z =
                std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);

            // Check if we need to slow down to avoid overshooting
            double max_vel_to_stop =
                std::sqrt(2 * rotate_to_heading_max_angular_accel_ * fabs(angular_distance_to_heading));
            if (fabs(cmd_vel.angular.z) > max_vel_to_stop) {
                cmd_vel.angular.z = sign * max_vel_to_stop;
            }
            cmd_vel.linear.x = 0.0;
            return cmd_vel;
        }
    private:

        // Smooth control law
        std::unique_ptr<nav2_graceful_controller::SmoothControlLaw> control_law_;
        double k_phi_, k_delta_, beta_, lambda_;
        double slowdown_radius_, v_linear_min_, v_linear_max_, v_angular_max_;
        double rotate_to_heading_angular_vel_, rotate_to_heading_max_angular_accel_;
        double accel_x_max_;
        
        geometry_msgs::msg::PoseStamped target_pose_;
        geometry_msgs::msg::PoseStamped target_map_pose_;

        rclcpp::TimerBase::SharedPtr GetTarPose_timer_;
        rclcpp::TimerBase::SharedPtr ControlLoop_timer_;
        

        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;

        geometry_msgs::msg::Twist pre_cmd_vel_;

        int count = 0;
    
        bool get_tar_pose_ = false;
        bool is_goal_ = false;
        bool is_coffe_ = true;
        bool get_map_pose_ = false;
        std::ofstream log_file_;
        std::string file_path_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::string package_name = "b1_perception";
    std::string yaml_file = "config/docking.yaml";
    std::string yaml_path = ament_index_cpp::get_package_share_directory(package_name) + "/" + yaml_file; 

    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", yaml_path});

    auto docking_control_node = std::make_shared<B1DockingControlNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(docking_control_node);
    while (rclcpp::ok())
    {
        executor.spin();
    }
    
    
    return 0;
}
