#include "mobile_perception/obstacle_detector_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <utility>

namespace mobile_perception
{

ObstacleDetectorNode::ObstacleDetectorNode()
: Node("obstacle_detector_node"), costmap_received_(false), transform_valid_(false)
{
    this->declare_parameter("costmap_topic", "local_costmap/costmap");
    this->declare_parameter("robot_base_frame", "base_link");
    this->declare_parameter("front_distance", 1.0);
    this->declare_parameter("left_width", 0.5);
    this->declare_parameter("right_width", 0.5);
    this->declare_parameter("obstacle_threshold", 50);
    this->declare_parameter("detection_frequency", 10.0);

    costmap_topic_ = this->get_parameter("costmap_topic").as_string();
    robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
    front_distance_ = this->get_parameter("front_distance").as_double();
    left_width_ = this->get_parameter("left_width").as_double();
    right_width_ = this->get_parameter("right_width").as_double();
    obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_int();
    detection_frequency_ = this->get_parameter("detection_frequency").as_double();

    if (detection_frequency_ <= 0.0) {
        RCLCPP_WARN(
            this->get_logger(),
            "Invalid detection_frequency %.3f, fallback to 10.0 Hz",
            detection_frequency_);
        detection_frequency_ = 10.0;
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        costmap_topic_,
        10,
        std::bind(&ObstacleDetectorNode::costmapCallback, this, std::placeholders::_1));

    obstacle_pub_ = this->create_publisher<std_msgs::msg::Bool>("obstacle_detector/has_obstacle", 10);
    obstacle_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>("obstacle_detector/markers", 10);
    detection_box_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>("obstacle_detector/detection_box", 10);

    detection_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / detection_frequency_)),
        std::bind(&ObstacleDetectorNode::detectionTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "ObstacleDetectorNode initialized");
    RCLCPP_INFO(this->get_logger(), "  costmap_topic: %s", costmap_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  front_distance: %.2f m", front_distance_);
    RCLCPP_INFO(this->get_logger(), "  left_width: %.2f m, right_width: %.2f m", left_width_, right_width_);
    RCLCPP_INFO(this->get_logger(), "  obstacle_threshold: %d", obstacle_threshold_);
    RCLCPP_INFO(this->get_logger(), "  detection_frequency: %.1f Hz", detection_frequency_);
}

void ObstacleDetectorNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    current_costmap_ = msg;
    costmap_received_ = true;

    if (costmap_frame_ != msg->header.frame_id) {
        costmap_frame_ = msg->header.frame_id;
        transform_valid_ = false;
        RCLCPP_INFO(this->get_logger(), "Costmap frame changed to: %s", costmap_frame_.c_str());
    }
}

void ObstacleDetectorNode::detectionTimerCallback()
{
    if (!costmap_received_ || !current_costmap_) {
        return;
    }

    if (!transform_valid_) {
        if (!updateTransform()) {
            return;
        }
    }

    const bool has_obstacle = checkObstacleInFrontRect();

    std_msgs::msg::Bool obstacle_msg;
    obstacle_msg.data = has_obstacle;
    obstacle_pub_->publish(obstacle_msg);

    if (has_obstacle) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Obstacle detected in front!");
    }
}

bool ObstacleDetectorNode::checkObstacleInFrontRect()
{
    const nav_msgs::msg::OccupancyGrid & costmap = *current_costmap_;

    geometry_msgs::msg::PoseStamped robot_pose_base;
    robot_pose_base.header.frame_id = robot_base_frame_;
    robot_pose_base.header.stamp = rclcpp::Time(0);
    robot_pose_base.pose.position.x = 0.0;
    robot_pose_base.pose.position.y = 0.0;
    robot_pose_base.pose.position.z = 0.0;
    robot_pose_base.pose.orientation.x = 0.0;
    robot_pose_base.pose.orientation.y = 0.0;
    robot_pose_base.pose.orientation.z = 0.0;
    robot_pose_base.pose.orientation.w = 1.0;

    geometry_msgs::msg::PoseStamped robot_pose;
    try {
        robot_pose = tf_buffer_->transform(robot_pose_base, costmap_frame_, tf2::durationFromSec(0.1));
    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Cannot transform robot pose: %s",
            ex.what());
        transform_valid_ = false;
        return false;
    }

    const double robot_x = robot_pose.pose.position.x;
    const double robot_y = robot_pose.pose.position.y;
    const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    const double cos_yaw = std::cos(robot_yaw);
    const double sin_yaw = std::sin(robot_yaw);

    auto transform_point = [&](double x_robot, double y_robot) -> std::pair<double, double> {
        const double x_map = robot_x + x_robot * cos_yaw - y_robot * sin_yaw;
        const double y_map = robot_y + x_robot * sin_yaw + y_robot * cos_yaw;
        return std::make_pair(x_map, y_map);
    };

    const std::pair<double, double> front_left = transform_point(front_distance_, left_width_);
    const std::pair<double, double> front_right = transform_point(front_distance_, -right_width_);
    const std::pair<double, double> rear_left = transform_point(0.0, left_width_);
    const std::pair<double, double> rear_right = transform_point(0.0, -right_width_);

    const double front_left_x = front_left.first;
    const double front_left_y = front_left.second;
    const double front_right_x = front_right.first;
    const double front_right_y = front_right.second;
    const double rear_left_x = rear_left.first;
    const double rear_left_y = rear_left.second;
    const double rear_right_x = rear_right.first;
    const double rear_right_y = rear_right.second;

    const double min_x = std::min(std::min(front_left_x, front_right_x), std::min(rear_left_x, rear_right_x));
    const double max_x = std::max(std::max(front_left_x, front_right_x), std::max(rear_left_x, rear_right_x));
    const double min_y = std::min(std::min(front_left_y, front_right_y), std::min(rear_left_y, rear_right_y));
    const double max_y = std::max(std::max(front_left_y, front_right_y), std::max(rear_left_y, rear_right_y));

    const double resolution = costmap.info.resolution;
    const double step = resolution * 2.0;
    bool has_obstacle = false;
    int checked_cells = 0;
    int obstacle_cells = 0;
    std::vector<geometry_msgs::msg::Point> obstacle_points;

    for (double wx = min_x; wx <= max_x; wx += step) {
        for (double wy = min_y; wy <= max_y; wy += step) {
            if (!pointInRect(
                    wx,
                    wy,
                    front_left_x,
                    front_left_y,
                    front_right_x,
                    front_right_y,
                    rear_left_x,
                    rear_left_y,
                    rear_right_x,
                    rear_right_y)) {
                continue;
            }

            unsigned int mx = 0;
            unsigned int my = 0;
            if (!worldToMap(costmap, wx, wy, mx, my)) {
                continue;
            }

            checked_cells++;

            const unsigned char cost = getCost(costmap, mx, my);
            if (cost != 255 && static_cast<int>(cost) >= obstacle_threshold_) {
                obstacle_cells++;
                has_obstacle = true;

                geometry_msgs::msg::Point point;
                point.x = wx;
                point.y = wy;
                point.z = 0.0;
                obstacle_points.push_back(point);
            }
        }
    }

    if (checked_cells > 0) {
        RCLCPP_DEBUG(
            this->get_logger(),
            "Checked %d cells, found %d obstacle cells (%.1f%%)",
            checked_cells,
            obstacle_cells,
            100.0 * obstacle_cells / checked_cells);
    }

    publishDetectionBoxMarker(
        front_left_x,
        front_left_y,
        front_right_x,
        front_right_y,
        rear_left_x,
        rear_left_y,
        rear_right_x,
        rear_right_y,
        costmap.header.stamp,
        has_obstacle);
    publishObstacleMarkers(obstacle_points, costmap.header.stamp);

    return has_obstacle;
}

bool ObstacleDetectorNode::pointInRect(
    double px,
    double py,
    double rect_front_left_x,
    double rect_front_left_y,
    double rect_front_right_x,
    double rect_front_right_y,
    double rect_rear_left_x,
    double rect_rear_left_y,
    double rect_rear_right_x,
    double rect_rear_right_y) const
{
    auto cross_product = [](double x1, double y1, double x2, double y2, double point_x, double point_y) {
        return (x2 - x1) * (point_y - y1) - (y2 - y1) * (point_x - x1);
    };

    const double c1 =
        cross_product(rect_rear_left_x, rect_rear_left_y, rect_front_left_x, rect_front_left_y, px, py);
    const double c2 =
        cross_product(rect_front_left_x, rect_front_left_y, rect_front_right_x, rect_front_right_y, px, py);
    const double c3 =
        cross_product(rect_front_right_x, rect_front_right_y, rect_rear_right_x, rect_rear_right_y, px, py);
    const double c4 =
        cross_product(rect_rear_right_x, rect_rear_right_y, rect_rear_left_x, rect_rear_left_y, px, py);

    return (c1 >= 0 && c2 >= 0 && c3 >= 0 && c4 >= 0) || (c1 <= 0 && c2 <= 0 && c3 <= 0 && c4 <= 0);
}

void ObstacleDetectorNode::publishObstacleMarkers(
    const std::vector<geometry_msgs::msg::Point> & points,
    const rclcpp::Time & stamp)
{
    if (!obstacle_marker_pub_) {
        return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = costmap_frame_;
    marker.header.stamp = stamp;
    marker.ns = "obstacle_detector";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;

    const double scale = current_costmap_ ? current_costmap_->info.resolution * 0.8 : 0.05;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = 0.01;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.9f;

    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.points = points;

    obstacle_marker_pub_->publish(marker);
}

void ObstacleDetectorNode::publishDetectionBoxMarker(
    double front_left_x,
    double front_left_y,
    double front_right_x,
    double front_right_y,
    double rear_left_x,
    double rear_left_y,
    double rear_right_x,
    double rear_right_y,
    const rclcpp::Time & stamp,
    bool has_obstacle)
{
    if (!detection_box_pub_) {
        return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = costmap_frame_;
    marker.header.stamp = stamp;
    marker.ns = "detection_box";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.03;

    if (has_obstacle) {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
    } else {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
    }
    marker.color.b = 0.0f;
    marker.color.a = 0.8f;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    geometry_msgs::msg::Point point;
    point.z = 0.0;

    point.x = rear_left_x;
    point.y = rear_left_y;
    marker.points.push_back(point);

    point.x = front_left_x;
    point.y = front_left_y;
    marker.points.push_back(point);

    point.x = front_right_x;
    point.y = front_right_y;
    marker.points.push_back(point);

    point.x = rear_right_x;
    point.y = rear_right_y;
    marker.points.push_back(point);

    point.x = rear_left_x;
    point.y = rear_left_y;
    marker.points.push_back(point);

    detection_box_pub_->publish(marker);
}

bool ObstacleDetectorNode::worldToMap(
    const nav_msgs::msg::OccupancyGrid & costmap,
    double wx,
    double wy,
    unsigned int & mx,
    unsigned int & my) const
{
    const double origin_x = costmap.info.origin.position.x;
    const double origin_y = costmap.info.origin.position.y;
    const double resolution = costmap.info.resolution;

    if (wx < origin_x || wy < origin_y || resolution <= 0.0) {
        return false;
    }

    mx = static_cast<unsigned int>((wx - origin_x) / resolution);
    my = static_cast<unsigned int>((wy - origin_y) / resolution);

    return mx < costmap.info.width && my < costmap.info.height;
}

unsigned char ObstacleDetectorNode::getCost(
    const nav_msgs::msg::OccupancyGrid & costmap,
    unsigned int mx,
    unsigned int my) const
{
    if (mx >= costmap.info.width || my >= costmap.info.height) {
        return 255;
    }

    const unsigned int index = my * costmap.info.width + mx;
    if (index >= costmap.data.size()) {
        return 255;
    }

    return static_cast<unsigned char>(costmap.data[index]);
}

bool ObstacleDetectorNode::updateTransform()
{
    if (costmap_frame_.empty()) {
        return false;
    }

    try {
        base_to_costmap_transform_ = tf_buffer_->lookupTransform(
            costmap_frame_,
            robot_base_frame_,
            tf2::TimePointZero,
            tf2::durationFromSec(0.1));
        transform_valid_ = true;
        return true;
    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Cannot get transform from %s to %s: %s",
            robot_base_frame_.c_str(),
            costmap_frame_.c_str(),
            ex.what());
        transform_valid_ = false;
        return false;
    }
}

}  // namespace mobile_perception

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mobile_perception::ObstacleDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
