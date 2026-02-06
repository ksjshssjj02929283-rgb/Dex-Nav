#ifndef MOBILE_PERCEPTION__OBSTACLE_DETECTOR_NODE_HPP_
#define MOBILE_PERCEPTION__OBSTACLE_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
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
    // 订阅局部代价地图回调（只缓存数据）
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
    // 定时器回调（执行检测）
    void detectionTimerCallback();
    
    // 检测前方是否有障碍物（矩形区域）
    bool checkObstacleInFrontRect();
    
    // 判断点是否在矩形内（使用叉积）
    bool pointInRect(double px, double py,
                     double rect_front_left_x, double rect_front_left_y,
                     double rect_front_right_x, double rect_front_right_y,
                     double rect_rear_left_x, double rect_rear_left_y,
                     double rect_rear_right_x, double rect_rear_right_y) const;

    // 发布可视化标记
    void publishObstacleMarkers(
        const std::vector<geometry_msgs::msg::Point> & points,
        const rclcpp::Time & stamp);
    
    // 将世界坐标转换为代价地图索引
    bool worldToMap(
        const nav_msgs::msg::OccupancyGrid & costmap,
        double wx, double wy,
        unsigned int & mx, unsigned int & my) const;
    
    // 获取代价地图中指定索引的代价值
    unsigned char getCost(
        const nav_msgs::msg::OccupancyGrid & costmap,
        unsigned int mx, unsigned int my) const;
    
    // 更新坐标系变换（只在costmap frame变化时调用）
    bool updateTransform();

    // ROS2 接口
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_marker_pub_;
    rclcpp::TimerBase::SharedPtr detection_timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 参数
    std::string costmap_topic_;
    std::string robot_base_frame_;
    double front_distance_;      // 前方检测距离（米）
    double left_width_;          // 左侧宽度（米）
    double right_width_;         // 右侧宽度（米）
    int obstacle_threshold_;     // 障碍物阈值（0-100，通常>=50或>=100表示障碍物）
    double detection_frequency_; // 检测频率（Hz）
    
    // 状态
    nav_msgs::msg::OccupancyGrid::SharedPtr current_costmap_;
    bool costmap_received_;
    
    // 缓存的坐标系信息
    std::string costmap_frame_;
    geometry_msgs::msg::TransformStamped base_to_costmap_transform_;
    bool transform_valid_;
};

}  // namespace mobile_perception

#endif  // MOBILE_PERCEPTION__OBSTACLE_DETECTOR_NODE_HPP_

