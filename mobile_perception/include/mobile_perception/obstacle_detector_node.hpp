#ifndef MOBILE_PERCEPTION__OBSTACLE_DETECTOR_NODE_HPP_
#define MOBILE_PERCEPTION__OBSTACLE_DETECTOR_NODE_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include <memory>
#include <string>
#include <vector>

namespace mobile_perception
{

class ObstacleDetectorNode : public rclcpp::Node
{
public:
    ObstacleDetectorNode();
    ~ObstacleDetectorNode() = default;

private:
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void detectionTimerCallback();
    bool checkObstacleInFrontRect();

    bool pointInRect(
        double px,
        double py,
        double rect_front_left_x,
        double rect_front_left_y,
        double rect_front_right_x,
        double rect_front_right_y,
        double rect_rear_left_x,
        double rect_rear_left_y,
        double rect_rear_right_x,
        double rect_rear_right_y) const;

    void publishObstacleMarkers(const std::vector<geometry_msgs::msg::Point> & points, const rclcpp::Time & stamp);

    void publishDetectionBoxMarker(
        double front_left_x,
        double front_left_y,
        double front_right_x,
        double front_right_y,
        double rear_left_x,
        double rear_left_y,
        double rear_right_x,
        double rear_right_y,
        const rclcpp::Time & stamp,
        bool has_obstacle);

    bool worldToMap(
        const nav_msgs::msg::OccupancyGrid & costmap,
        double wx,
        double wy,
        unsigned int & mx,
        unsigned int & my) const;

    unsigned char getCost(const nav_msgs::msg::OccupancyGrid & costmap, unsigned int mx, unsigned int my) const;
    bool updateTransform();

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr detection_box_pub_;
    rclcpp::TimerBase::SharedPtr detection_timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string costmap_topic_;
    std::string robot_base_frame_;
    double front_distance_;
    double left_width_;
    double right_width_;
    int obstacle_threshold_;
    double detection_frequency_;

    nav_msgs::msg::OccupancyGrid::SharedPtr current_costmap_;
    bool costmap_received_;

    std::string costmap_frame_;
    geometry_msgs::msg::TransformStamped base_to_costmap_transform_;
    bool transform_valid_;
};

}  // namespace mobile_perception

#endif  // MOBILE_PERCEPTION__OBSTACLE_DETECTOR_NODE_HPP_
