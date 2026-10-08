// Copyright (c) 2022 Samsung R&D Institute Russia
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_collision_monitor/polygon.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <utility>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include "nav2_util/node_utils.hpp"

#include "nav2_collision_monitor/kinematics.hpp"

namespace nav2_collision_monitor
{

Polygon::Polygon(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & polygon_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const tf2::Duration & transform_tolerance)
: node_(node), polygon_name_(polygon_name), action_type_(DO_NOTHING),
  slowdown_ratio_(0.0), footprint_sub_(nullptr), tf_buffer_(tf_buffer),
  base_frame_id_(base_frame_id), transform_tolerance_(transform_tolerance)
{
  RCLCPP_INFO(logger_, "[%s]: Creating Polygon", polygon_name_.c_str());
}

Polygon::~Polygon()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying Polygon", polygon_name_.c_str());
  poly_.clear();
}

bool Polygon::configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string polygon_pub_topic, footprint_topic;

  if (!getParameters(polygon_pub_topic, footprint_topic)) {
    return false;
  }

  if (!footprint_topic.empty()) {
    footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(
      node, footprint_topic, *tf_buffer_,
      base_frame_id_, tf2::durationToSec(transform_tolerance_));
  }

  if (visualize_) {
    // Fill polygon_ points for future usage
    std::vector<Point> poly;
    getPolygon(poly);
    for (const Point & p : poly) {
      geometry_msgs::msg::Point32 p_s;
      p_s.x = p.x;
      p_s.y = p.y;
      // p_s.z will remain 0.0
      polygon_.points.push_back(p_s);
    }

    rclcpp::QoS polygon_qos = rclcpp::SystemDefaultsQoS();  // set to default
    polygon_pub_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>(
      polygon_pub_topic, polygon_qos);
  }

  return true;
}

void Polygon::activate()
{
  if (visualize_) {
    polygon_pub_->on_activate();
  }
}

void Polygon::deactivate()
{
  if (visualize_) {
    polygon_pub_->on_deactivate();
  }
}

std::string Polygon::getName() const
{
  return polygon_name_;
}

ActionType Polygon::getActionType() const
{
  return action_type_;
}

int Polygon::getMaxPoints() const
{
  return max_points_;
}

double Polygon::getSlowdownRatio() const
{
  return slowdown_ratio_;
}

double Polygon::getTimeBeforeCollision() const
{
  return time_before_collision_;
}

void Polygon::getPolygon(std::vector<Point> & poly) const
{
  poly = poly_;
}

void Polygon::setPadding(double padding)
{
  // NaN 或负数 → 恢复默认（NaN 与任何值比较都为 false，这里显式判）
  const double applied = (std::isnan(padding) || padding < 0.0) ? default_padding_ : padding;
  padding_.store(applied, std::memory_order_relaxed);
}

double Polygon::getPadding() const
{
  return padding_.load(std::memory_order_relaxed);
}

double Polygon::getDefaultPadding() const
{
  return default_padding_;
}

void Polygon::setLateralPaddingOnly(bool enabled)
{
  lateral_padding_only_.store(enabled, std::memory_order_relaxed);
}

bool Polygon::isLateralPaddingOnly() const
{
  return lateral_padding_only_.load(std::memory_order_relaxed);
}

void Polygon::getEffectivePolygon(std::vector<Point> & poly) const
{
  const double padding = padding_.load(std::memory_order_relaxed);
  const std::size_t n = poly_.size();
  if (padding <= 0.0 || n < 3) {
    poly = poly_;
    return;
  }

  if (lateral_padding_only_.load(std::memory_order_relaxed)) {
    poly = poly_;
    for (Point & point : poly) {
      if (point.y > 0.0) {
        point.y += padding;
      } else if (point.y < 0.0) {
        point.y -= padding;
      }
    }
    return;
  }

  // 顶点沿相邻两边外法线的角平分线外推（凸角为精确 miter 接头）。
  // 仅用于可视化，凹角处会略微失真，判定不依赖这里。
  // 绕向：有向面积 > 0 为逆时针，外法线取 (dy, -dx)；顺时针则取 (-dy, dx)。
  double area2 = 0.0;
  for (std::size_t i = 0; i < n; i++) {
    const Point & a = poly_[i];
    const Point & b = poly_[(i + 1) % n];
    area2 += a.x * b.y - b.x * a.y;
  }
  const double sign = (area2 >= 0.0) ? 1.0 : -1.0;

  auto edge_normal = [sign](const Point & a, const Point & b) {
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double len = std::hypot(dx, dy);
      if (len < 1e-9) {
        return Point{0.0, 0.0};
      }
      return Point{sign * dy / len, -sign * dx / len};
    };

  poly.resize(n);
  for (std::size_t i = 0; i < n; i++) {
    const Point n1 = edge_normal(poly_[(i + n - 1) % n], poly_[i]);
    const Point n2 = edge_normal(poly_[i], poly_[(i + 1) % n]);
    double bx = n1.x + n2.x;
    double by = n1.y + n2.y;
    const double blen = std::hypot(bx, by);
    if (blen < 1e-9) {
      // 两边反向（退化的尖角），无法求角平分线，退回单边法线
      poly[i] = {poly_[i].x + n2.x * padding, poly_[i].y + n2.y * padding};
      continue;
    }
    bx /= blen;
    by /= blen;
    // miter 长度 = padding / cos(θ/2)，cos(θ/2) 即单位角平分线与边法线的夹角余弦
    const double cos_half = bx * n1.x + by * n1.y;
    // 夹角过尖时 miter 会飞出去，限幅到 5 倍 padding
    const double scale = padding / std::max(cos_half, 0.2);
    poly[i] = {poly_[i].x + bx * scale, poly_[i].y + by * scale};
  }
}

double Polygon::distanceToBoundary(const Point & point) const
{
  const std::size_t n = poly_.size();
  if (n == 0) {
    return std::numeric_limits<double>::infinity();
  }
  double min_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < n; i++) {
    const Point & a = poly_[i];
    const Point & b = poly_[(i + 1) % n];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len_sq = dx * dx + dy * dy;
    double t = 0.0;
    if (len_sq > 1e-18) {
      // 点在线段上的投影参数，夹到 [0,1] 即为线段最近点
      t = ((point.x - a.x) * dx + (point.y - a.y) * dy) / len_sq;
      t = std::max(0.0, std::min(1.0, t));
    }
    const double d = std::hypot(point.x - (a.x + t * dx), point.y - (a.y + t * dy));
    min_dist = std::min(min_dist, d);
  }
  return min_dist;
}

void Polygon::updatePolygon()
{
  if (footprint_sub_ != nullptr) {
    // Get latest robot footprint from footprint subscriber
    std::vector<geometry_msgs::msg::Point> footprint_vec;
    std_msgs::msg::Header footprint_header;
    footprint_sub_->getFootprintInRobotFrame(footprint_vec, footprint_header);

    std::size_t new_size = footprint_vec.size();
    poly_.resize(new_size);
    polygon_.points.resize(new_size);

    geometry_msgs::msg::Point32 p_s;
    for (std::size_t i = 0; i < new_size; i++) {
      poly_[i] = {footprint_vec[i].x, footprint_vec[i].y};
      p_s.x = footprint_vec[i].x;
      p_s.y = footprint_vec[i].y;
      polygon_.points[i] = p_s;
    }
  }
}

int Polygon::getPointsInside(const std::vector<Point> & points) const
{
  int num = 0;
  for (const Point & point : points) {
    if (isPointInside(point)) {
      num++;
    }
  }
  return num;
}

double Polygon::getCollisionTime(
  const std::vector<Point> & collision_points,
  const Velocity & velocity) const
{
  // Initial robot pose is {0,0} in base_footprint coordinates
  Pose pose = {0.0, 0.0, 0.0};
  Velocity vel = velocity;

  // Array of points transformed to the frame concerned with pose on each simulation step
  std::vector<Point> points_transformed;

  // Robot movement simulation
  for (double time = 0.0; time <= time_before_collision_; time += simulation_time_step_) {
    // Shift the robot pose towards to the vel during simulation_time_step_ time interval
    // NOTE: vel is changing during the simulation
    projectState(simulation_time_step_, pose, vel);
    // Transform collision_points to the frame concerned with current robot pose
    points_transformed = collision_points;
    transformPoints(pose, points_transformed);
    // If the collision occurred on this stage, return the actual time before a collision
    // as if robot was moved with given velocity
    if (getPointsInside(points_transformed) > max_points_) {
      return time;
    }
  }

  // There is no collision
  return -1.0;
}

void Polygon::publish() const
{
  if (!visualize_) {
    return;
  }

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Fill PolygonStamped struct
  std::unique_ptr<geometry_msgs::msg::PolygonStamped> poly_s =
    std::make_unique<geometry_msgs::msg::PolygonStamped>();
  poly_s->header.stamp = node->now();
  poly_s->header.frame_id = base_frame_id_;
  // 每次发布都按当前 padding 现算轮廓，这样 rviz 里看到的就是实际生效的检测范围
  // （本仓库相对上游 Nav2 的改动：上游直接发缓存的 polygon_）
  std::vector<Point> effective_poly;
  getEffectivePolygon(effective_poly);
  poly_s->polygon.points.reserve(effective_poly.size());
  for (const Point & p : effective_poly) {
    geometry_msgs::msg::Point32 p_s;
    p_s.x = p.x;
    p_s.y = p.y;
    // p_s.z will remain 0.0
    poly_s->polygon.points.push_back(p_s);
  }

  // Publish polygon
  polygon_pub_->publish(std::move(poly_s));
}

bool Polygon::getCommonParameters(std::string & polygon_pub_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  try {
    // Get action type.
    // Leave it not initialized: the will cause an error if it will not set.
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".action_type", rclcpp::PARAMETER_STRING);
    const std::string at_str =
      node->get_parameter(polygon_name_ + ".action_type").as_string();
    if (at_str == "stop") {
      action_type_ = STOP;
    } else if (at_str == "slowdown") {
      action_type_ = SLOWDOWN;
    } else if (at_str == "approach") {
      action_type_ = APPROACH;
    } else {  // Error if something else
      RCLCPP_ERROR(logger_, "[%s]: Unknown action type: %s", polygon_name_.c_str(), at_str.c_str());
      return false;
    }

    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".max_points", rclcpp::ParameterValue(3));
    max_points_ = node->get_parameter(polygon_name_ + ".max_points").as_int();

    if (action_type_ == SLOWDOWN) {
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".slowdown_ratio", rclcpp::ParameterValue(0.5));
      slowdown_ratio_ = node->get_parameter(polygon_name_ + ".slowdown_ratio").as_double();
    }

    if (action_type_ == APPROACH) {
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".time_before_collision", rclcpp::ParameterValue(2.0));
      time_before_collision_ =
        node->get_parameter(polygon_name_ + ".time_before_collision").as_double();
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".simulation_time_step", rclcpp::ParameterValue(0.1));
      simulation_time_step_ =
        node->get_parameter(polygon_name_ + ".simulation_time_step").as_double();
    }

    // 动态膨胀默认值（本仓库相对上游 Nav2 的改动）。
    // 基础形状（points / radius）配成底盘轮廓，padding 是其上的膨胀量；
    // 运行时可经 ~/set_polygon_padding 修改，收到 NaN 时回落到这里。
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".default_padding", rclcpp::ParameterValue(0.0));
    default_padding_ = node->get_parameter(polygon_name_ + ".default_padding").as_double();
    if (default_padding_ < 0.0) {
      RCLCPP_WARN(
        logger_, "[%s]: default_padding %.3f < 0, clamped to 0",
        polygon_name_.c_str(), default_padding_);
      default_padding_ = 0.0;
    }
    padding_.store(default_padding_, std::memory_order_relaxed);

    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".visualize", rclcpp::ParameterValue(false));
    visualize_ = node->get_parameter(polygon_name_ + ".visualize").as_bool();
    if (visualize_) {
      // Get polygon topic parameter in case if it is going to be published
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".polygon_pub_topic", rclcpp::ParameterValue(polygon_name_));
      polygon_pub_topic = node->get_parameter(polygon_name_ + ".polygon_pub_topic").as_string();
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      logger_,
      "[%s]: Error while getting common polygon parameters: %s",
      polygon_name_.c_str(), ex.what());
    return false;
  }

  return true;
}

bool Polygon::getParameters(std::string & polygon_pub_topic, std::string & footprint_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!getCommonParameters(polygon_pub_topic)) {
    return false;
  }

  try {
    if (action_type_ == APPROACH) {
      // Obtain the footprint topic to make a footprint subscription for approach polygon
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".footprint_topic",
        rclcpp::ParameterValue("local_costmap/published_footprint"));
      footprint_topic =
        node->get_parameter(polygon_name_ + ".footprint_topic").as_string();

      // This is robot footprint: do not need to get polygon points from ROS parameters.
      // It will be set dynamically later.
      return true;
    } else {
      // Make it empty otherwise
      footprint_topic.clear();
    }

    // Leave it not initialized: the will cause an error if it will not set
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".points", rclcpp::PARAMETER_DOUBLE_ARRAY);
    std::vector<double> poly_row =
      node->get_parameter(polygon_name_ + ".points").as_double_array();
    // Check for points format correctness
    if (poly_row.size() <= 6 || poly_row.size() % 2 != 0) {
      RCLCPP_ERROR(
        logger_,
        "[%s]: Polygon has incorrect points description",
        polygon_name_.c_str());
      return false;
    }

    // Obtain polygon vertices
    Point point;
    bool first = true;
    for (double val : poly_row) {
      if (first) {
        point.x = val;
      } else {
        point.y = val;
        poly_.push_back(point);
      }
      first = !first;
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      logger_,
      "[%s]: Error while getting polygon parameters: %s",
      polygon_name_.c_str(), ex.what());
    return false;
  }

  return true;
}

inline bool Polygon::isPointInside(const Point & point) const
{
  // Adaptation of Shimrat, Moshe. "Algorithm 112: position of point relative to polygon."
  // Communications of the ACM 5.8 (1962): 434.
  // Implementation of ray crossings algorithm for point in polygon task solving.
  // Y coordinate is fixed. Moving the ray on X+ axis starting from given point.
  // Odd number of intersections with polygon boundaries means the point is inside polygon.
  std::vector<Point> effective_poly;
  const std::vector<Point> * checked_poly = &poly_;
  const bool lateral_only = lateral_padding_only_.load(std::memory_order_relaxed);
  if (lateral_only) {
    getEffectivePolygon(effective_poly);
    checked_poly = &effective_poly;
  }
  const int poly_size = checked_poly->size();
  int i, j;  // Polygon vertex iterators
  bool res = false;  // Final result, initialized with already inverted value

  // Starting from the edge where the last point of polygon is connected to the first
  i = poly_size - 1;
  for (j = 0; j < poly_size; j++) {
    // Checking the edge only if given point is between edge boundaries by Y coordinates.
    // One of the condition should contain equality in order to exclude the edges
    // parallel to X+ ray.
    if ((point.y <= (*checked_poly)[i].y) == (point.y > (*checked_poly)[j].y)) {
      // Calculating the intersection coordinate of X+ ray
      const double x_inter = (*checked_poly)[i].x +
        (point.y - (*checked_poly)[i].y) *
        ((*checked_poly)[j].x - (*checked_poly)[i].x) /
        ((*checked_poly)[j].y - (*checked_poly)[i].y);
      // If intersection with checked edge is greater than point.x coordinate, inverting the result
      if (x_inter > point.x) {
        res = !res;
      }
    }
    i = j;
  }
  if (res) {
    return true;
  }

  if (lateral_only) {
    return false;
  }

  // 动态膨胀（本仓库相对上游 Nav2 的改动）：点在基础形状外时，再看它到边界的距离。
  // "在基础形状内 或 到边界距离 < padding" 精确等价于对基础形状做 padding 的
  // Minkowski 膨胀，对凹多边形同样成立，且不需要真的把多边形做大。
  const double padding = padding_.load(std::memory_order_relaxed);
  if (padding <= 0.0) {
    return false;
  }
  return distanceToBoundary(point) < padding;
}

}  // namespace nav2_collision_monitor
