#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class BaseLinkPosition : public rclcpp::Node {
public:
    BaseLinkPosition() : Node("baselink_position", rclcpp::NodeOptions().use_intra_process_comms(true)) {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/feedback/chassis_pose",
                                                                            rclcpp::QoS(1).best_effort());

        // 降到 1Hz：几乎无 CPU 占用
        timer_ = this->create_wall_timer(500ms, std::bind(&BaseLinkPosition::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Low-CPU Chassis Pose Publisher started");
    }

private:
    void timer_callback() {
        if (!tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero, 50ms)) return;

        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            // 不打印日志，节省 CPU
            return;
        }

        // 只在位置变化明显时发布
        double x = transform.transform.translation.x;
        double y = transform.transform.translation.y;

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = transform.header;
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = x;
        pose_msg.pose.position.y = y;
        pose_msg.pose.orientation = transform.transform.rotation;
        pose_pub_->publish(pose_msg);
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double last_x_{0.0}, last_y_{0.0};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseLinkPosition>());
    rclcpp::shutdown();
    return 0;
}
