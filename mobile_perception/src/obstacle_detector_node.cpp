#include "mobile_perception/obstacle_detector_node.hpp"
#include <cmath>
#include <algorithm>

namespace mobile_perception
{

ObstacleDetectorNode::ObstacleDetectorNode()
: Node("obstacle_detector_node"),
  costmap_received_(false),
  transform_valid_(false)
{
    // 声明参数
    this->declare_parameter("costmap_topic", "local_costmap/costmap");
    this->declare_parameter("robot_base_frame", "base_link");
    this->declare_parameter("front_distance", 1.0);      // 前方检测距离（米）
    this->declare_parameter("left_width", 0.5);           // 左侧宽度（米）
    this->declare_parameter("right_width", 0.5);           // 右侧宽度（米）
    this->declare_parameter("obstacle_threshold", 50);    // 障碍物阈值（0-100）
    this->declare_parameter("detection_frequency", 10.0);  // 检测频率（Hz）
    
    // 获取参数
    costmap_topic_ = this->get_parameter("costmap_topic").as_string();
    robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
    front_distance_ = this->get_parameter("front_distance").as_double();
    left_width_ = this->get_parameter("left_width").as_double();
    right_width_ = this->get_parameter("right_width").as_double();
    obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_int();
    detection_frequency_ = this->get_parameter("detection_frequency").as_double();
    
    // 初始化TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // 订阅局部代价地图
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        costmap_topic_, 10,
        std::bind(&ObstacleDetectorNode::costmapCallback, this, std::placeholders::_1));
    
    // 发布障碍物检测结果
    obstacle_pub_ = this->create_publisher<std_msgs::msg::Bool>("obstacle_detector/has_obstacle", 10);
    obstacle_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("obstacle_detector/markers", 10);
    detection_box_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("obstacle_detector/detection_box", 10);
    
    // 创建定时器执行检测
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
    // 只缓存数据，不执行检测
    current_costmap_ = msg;
    costmap_received_ = true;
    
    // 检查坐标系是否变化，如果变化则更新变换
    if (costmap_frame_ != msg->header.frame_id)
    {
        costmap_frame_ = msg->header.frame_id;
        transform_valid_ = false;
        RCLCPP_INFO(this->get_logger(), "Costmap frame changed to: %s", costmap_frame_.c_str());
    }
}

void ObstacleDetectorNode::detectionTimerCallback()
{
    if (!costmap_received_ || !current_costmap_)
    {
        return;
    }
    
    // 更新坐标系变换（只在坐标系变化时更新一次）
    if (!transform_valid_)
    {
        if (!updateTransform())
        {
            return;
        }
    }
    
    // 执行检测（每次检测时使用最新的机器人位置）
    bool has_obstacle = checkObstacleInFrontRect();
    
    // 发布结果
    std_msgs::msg::Bool obstacle_msg;
    obstacle_msg.data = has_obstacle;
    obstacle_pub_->publish(obstacle_msg);
    
    if (has_obstacle)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Obstacle detected in front!");
    }
}

bool ObstacleDetectorNode::checkObstacleInFrontRect()
{
    const nav_msgs::msg::OccupancyGrid & costmap = *current_costmap_;
    
    // 获取机器人在代价地图坐标系中的位置（每次使用最新时间戳查询TF）
    geometry_msgs::msg::PoseStamped robot_pose_base;
    robot_pose_base.header.frame_id = robot_base_frame_;
    // 使用时间戳 0 获取最新 TF，避免仿真/系统时间差导致外推
    robot_pose_base.header.stamp = rclcpp::Time(0);
    robot_pose_base.pose.position.x = 0.0;
    robot_pose_base.pose.position.y = 0.0;
    robot_pose_base.pose.position.z = 0.0;
    robot_pose_base.pose.orientation.x = 0.0;
    robot_pose_base.pose.orientation.y = 0.0;
    robot_pose_base.pose.orientation.z = 0.0;
    robot_pose_base.pose.orientation.w = 1.0;
    
    geometry_msgs::msg::PoseStamped robot_pose;
    try
    {
        // 使用最新可用 TF
        robot_pose = tf_buffer_->transform(robot_pose_base, costmap_frame_, tf2::durationFromSec(0.1));
    }
    catch (tf2::TransformException & ex)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Cannot transform robot pose: %s", ex.what());
        transform_valid_ = false;
        return false;
    }
    
    // 获取机器人位置和朝向
    double robot_x = robot_pose.pose.position.x;
    double robot_y = robot_pose.pose.position.y;
    double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    
    // 计算前方矩形区域的四个角点（在机器人坐标系中）
    double cos_yaw = std::cos(robot_yaw);
    double sin_yaw = std::sin(robot_yaw);
    
    // 矩形四个角点在机器人坐标系中的坐标
    double front_left_x_robot = front_distance_;
    double front_left_y_robot = left_width_;
    double front_right_x_robot = front_distance_;
    double front_right_y_robot = -right_width_;
    double rear_left_x_robot = 0.0;
    double rear_left_y_robot = left_width_;
    double rear_right_x_robot = 0.0;
    double rear_right_y_robot = -right_width_;
    
    // 将矩形角点从机器人坐标系转换到代价地图坐标系
    auto transformPoint = [&](double x_robot, double y_robot) -> std::pair<double, double> {
        double x_map = robot_x + x_robot * cos_yaw - y_robot * sin_yaw;
        double y_map = robot_y + x_robot * sin_yaw + y_robot * cos_yaw;
        return std::make_pair(x_map, y_map);
    };
    
    std::pair<double, double> front_left = transformPoint(front_left_x_robot, front_left_y_robot);
    std::pair<double, double> front_right = transformPoint(front_right_x_robot, front_right_y_robot);
    std::pair<double, double> rear_left = transformPoint(rear_left_x_robot, rear_left_y_robot);
    std::pair<double, double> rear_right = transformPoint(rear_right_x_robot, rear_right_y_robot);
    
    double front_left_x = front_left.first;
    double front_left_y = front_left.second;
    double front_right_x = front_right.first;
    double front_right_y = front_right.second;
    double rear_left_x = rear_left.first;
    double rear_left_y = rear_left.second;
    double rear_right_x = rear_right.first;
    double rear_right_y = rear_right.second;
    
    // 获取代价地图分辨率
    double resolution = costmap.info.resolution;
    
    // 优化：只遍历矩形ROI内部区域，并且步长可以大于分辨率
    bool has_obstacle = false;
    int checked_cells = 0;
    int obstacle_cells = 0;
    std::vector<geometry_msgs::msg::Point> obstacle_points;
    
    // 计算矩形ROI在世界坐标下的轴对齐包围框
    double min_x = std::min(std::min(front_left_x, front_right_x), std::min(rear_left_x, rear_right_x));
    double max_x = std::max(std::max(front_left_x, front_right_x), std::max(rear_left_x, rear_right_x));
    double min_y = std::min(std::min(front_left_y, front_right_y), std::min(rear_left_y, rear_right_y));
    double max_y = std::max(std::max(front_left_y, front_right_y), std::max(rear_left_y, rear_right_y));

    // 采样步长：可以比分辨率大一些，减少计算量
    const double step = resolution * 2.0;  // 如需更密集采样，可改为 1.0 * resolution

    for (double wx = min_x; wx <= max_x; wx += step)
    {
        for (double wy = min_y; wy <= max_y; wy += step)
        {
            // 判断点是否在矩形ROI内（考虑矩形的旋转）
            if (!pointInRect(wx, wy,
                             front_left_x, front_left_y,
                             front_right_x, front_right_y,
                             rear_left_x, rear_left_y,
                             rear_right_x, rear_right_y))
            {
                continue;
            }

            // 将世界坐标转换为地图索引
            unsigned int mx, my;
            if (!worldToMap(costmap, wx, wy, mx, my))
            {
                continue;
            }

            checked_cells++;

            // 获取代价值
            unsigned char cost = getCost(costmap, mx, my);

            // 检查是否为障碍物
            if (cost != 255 && static_cast<int>(cost) >= obstacle_threshold_)
            {
                obstacle_cells++;
                has_obstacle = true;
                // 记录障碍物中心点（用于可视化）
                geometry_msgs::msg::Point p;
                p.x = wx;
                p.y = wy;
                p.z = 0.0;
                obstacle_points.push_back(p);
            }
        }
    }
    
    if (checked_cells > 0)
    {
        RCLCPP_DEBUG(this->get_logger(), 
            "Checked %d cells, found %d obstacle cells (%.1f%%)",
            checked_cells, obstacle_cells, 
            100.0 * obstacle_cells / checked_cells);
    }

    // 发布检测框可视化标记
    publishDetectionBoxMarker(
        front_left_x, front_left_y,
        front_right_x, front_right_y,
        rear_left_x, rear_left_y,
        rear_right_x, rear_right_y,
        costmap.header.stamp,
        has_obstacle);

    // 发布可视化标记
    publishObstacleMarkers(obstacle_points, costmap.header.stamp);
    
    return has_obstacle;
}

bool ObstacleDetectorNode::pointInRect(double px, double py,
                                       double rect_front_left_x, double rect_front_left_y,
                                       double rect_front_right_x, double rect_front_right_y,
                                       double rect_rear_left_x, double rect_rear_left_y,
                                       double rect_rear_right_x, double rect_rear_right_y) const
{
    // 使用叉积判断点是否在矩形内
    // 对于凸多边形，如果点在所有边的同一侧，则在多边形内
    auto crossProduct = [](double x1, double y1, double x2, double y2, double px, double py) {
        return (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1);
    };
    
    // 检查点是否在矩形的四条边构成的凸多边形内
    double c1 = crossProduct(rect_rear_left_x, rect_rear_left_y, 
                            rect_front_left_x, rect_front_left_y, px, py);
    double c2 = crossProduct(rect_front_left_x, rect_front_left_y, 
                            rect_front_right_x, rect_front_right_y, px, py);
    double c3 = crossProduct(rect_front_right_x, rect_front_right_y, 
                            rect_rear_right_x, rect_rear_right_y, px, py);
    double c4 = crossProduct(rect_rear_right_x, rect_rear_right_y, 
                            rect_rear_left_x, rect_rear_left_y, px, py);
    
    // 如果所有叉积符号相同（或为零），则点在多边形内
    return (c1 >= 0 && c2 >= 0 && c3 >= 0 && c4 >= 0) ||
           (c1 <= 0 && c2 <= 0 && c3 <= 0 && c4 <= 0);
}

void ObstacleDetectorNode::publishObstacleMarkers(
    const std::vector<geometry_msgs::msg::Point> & points,
    const rclcpp::Time & stamp)
{
    if (!obstacle_marker_pub_) return;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = costmap_frame_;
    marker.header.stamp = stamp;
    marker.ns = "obstacle_detector";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 设置点尺寸（使用栅格分辨率的 0.8 倍）
    double scale = current_costmap_ ? current_costmap_->info.resolution * 0.8 : 0.05;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = 0.01;

    // 红色点
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.9f;

    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.points = points;

    obstacle_marker_pub_->publish(marker);
}

void ObstacleDetectorNode::publishDetectionBoxMarker(
    double front_left_x, double front_left_y,
    double front_right_x, double front_right_y,
    double rear_left_x, double rear_left_y,
    double rear_right_x, double rear_right_y,
    const rclcpp::Time & stamp,
    bool has_obstacle)
{
    if (!detection_box_pub_) return;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = costmap_frame_;
    marker.header.stamp = stamp;
    marker.ns = "detection_box";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.03;

    // 有障碍物时红色，无障碍物时绿色
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

    // 矩形四个角点 + 闭合回到起点
    geometry_msgs::msg::Point p;
    p.z = 0.0;

    p.x = rear_left_x;  p.y = rear_left_y;
    marker.points.push_back(p);

    p.x = front_left_x;  p.y = front_left_y;
    marker.points.push_back(p);

    p.x = front_right_x;  p.y = front_right_y;
    marker.points.push_back(p);

    p.x = rear_right_x;  p.y = rear_right_y;
    marker.points.push_back(p);

    // 闭合
    p.x = rear_left_x;  p.y = rear_left_y;
    marker.points.push_back(p);

    detection_box_pub_->publish(marker);
}

bool ObstacleDetectorNode::worldToMap(
    const nav_msgs::msg::OccupancyGrid & costmap,
    double wx, double wy,
    unsigned int & mx, unsigned int & my) const
{
    // 代价地图原点
    double origin_x = costmap.info.origin.position.x;
    double origin_y = costmap.info.origin.position.y;
    double resolution = costmap.info.resolution;
    
    // 转换为地图坐标
    mx = static_cast<unsigned int>((wx - origin_x) / resolution);
    my = static_cast<unsigned int>((wy - origin_y) / resolution);
    
    // 检查是否在地图范围内
    if (mx >= costmap.info.width || my >= costmap.info.height)
    {
        return false;
    }
    
    return true;
}

unsigned char ObstacleDetectorNode::getCost(
    const nav_msgs::msg::OccupancyGrid & costmap,
    unsigned int mx, unsigned int my) const
{
    if (mx >= costmap.info.width || my >= costmap.info.height)
    {
        return 255;  // 越界返回未知
    }
    
    // OccupancyGrid数据按行存储：data[row * width + col]
    unsigned int index = my * costmap.info.width + mx;
    if (index >= costmap.data.size())
    {
        return 255;  // 越界返回未知
    }
    
    return static_cast<unsigned char>(costmap.data[index]);
}

bool ObstacleDetectorNode::updateTransform()
{
    if (costmap_frame_.empty())
    {
        return false;
    }
    
    try
    {
        // 获取从base_link到costmap_frame的变换（使用最新时间戳）
        // 局部代价地图的坐标系通常不变，所以只需要在坐标系变化时更新一次
        base_to_costmap_transform_ = tf_buffer_->lookupTransform(
            costmap_frame_, robot_base_frame_, 
            tf2::TimePointZero,  // 使用最新时间戳
            tf2::durationFromSec(0.1));
        transform_valid_ = true;
        return true;
    }
    catch (tf2::TransformException & ex)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Cannot get transform from %s to %s: %s",
            robot_base_frame_.c_str(), costmap_frame_.c_str(), ex.what());
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

