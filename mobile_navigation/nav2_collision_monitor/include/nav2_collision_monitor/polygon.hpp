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

#ifndef NAV2_COLLISION_MONITOR__POLYGON_HPP_
#define NAV2_COLLISION_MONITOR__POLYGON_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"

#include "tf2/time.h"
#include "tf2_ros/buffer.h"

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"

#include "nav2_collision_monitor/types.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Basic polygon shape class.
 * For STOP/SLOWDOWN model it represents zone around the robot
 * while for APPROACH model it represents robot footprint.
 */
class Polygon
{
public:
  /**
   * @brief Polygon constructor
   * @param node Collision Monitor node pointer
   * @param polygon_name Name of polygon
   * @param tf_buffer Shared pointer to a TF buffer
   * @param base_frame_id Robot base frame ID
   * @param transform_tolerance Transform tolerance
   */
  Polygon(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & polygon_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const tf2::Duration & transform_tolerance);
  /**
   * @brief Polygon destructor
   */
  virtual ~Polygon();

  /**
   * @brief Shape configuration routine. Obtains ROS-parameters related to shape object
   * and creates polygon lifecycle publisher.
   * @return True in case of everything is configured correctly, or false otherwise
   */
  bool configure();
  /**
   * @brief Activates polygon lifecycle publisher
   */
  void activate();
  /**
   * @brief Deactivates polygon lifecycle publisher
   */
  void deactivate();

  /**
   * @brief Returns the name of polygon
   * @return Polygon name
   */
  std::string getName() const;
  /**
   * @brief Obtains polygon action type
   * @return Action type for current polygon
   */
  ActionType getActionType() const;
  /**
   * @brief Obtains polygon maximum points to enter inside polygon causing no action
   * @return Maximum points to enter to current polygon and take no action
   */
  int getMaxPoints() const;
  /**
   * @brief Obtains speed slowdown ratio for current polygon.
   * Applicable for SLOWDOWN model.
   * @return Speed slowdown ratio
   */
  double getSlowdownRatio() const;
  /**
   * @brief Obtains required time before collision for current polygon.
   * Applicable for APPROACH model.
   * @return Time before collision in seconds
   */
  double getTimeBeforeCollision() const;

  /**
   * @brief Gets polygon points
   * @param poly Output polygon points (vertices)
   */
  virtual void getPolygon(std::vector<Point> & poly) const;

  /** @brief Replace the internally owned base polygon (without padding). */
  virtual void setPolygon(const geometry_msgs::msg::Polygon & polygon);

  /**
   * @brief 取当前生效的形状（基础形状按 padding 向外膨胀后的轮廓）。
   *
   * 仅用于可视化发布。碰撞判定不走这里——判定在 isPointInside() 里用
   * "到基础形状边界的距离 < padding" 实现，那是精确的 Minkowski 和，
   * 对凹多边形同样正确；这里的顶点外推只是它的近似外轮廓。
   * @param poly 输出膨胀后的多边形顶点
   */
  virtual void getEffectivePolygon(std::vector<Point> & poly) const;

  /**
   * @brief 设置膨胀大小（本仓库相对上游 Nav2 的改动：动态停障距离）
   * @param padding 膨胀大小 m；NaN 或负数表示恢复 default_padding_
   */
  void setPadding(double padding);

  /**
   * @brief 取当前生效的膨胀大小
   */
  double getPadding() const;

  /**
   * @brief 设置前向额外膨胀大小（本仓库相对上游 Nav2 的改动：速度分档前向停障距离）
   * @param front_padding 前向额外外扩大小 m；NaN 或负数表示置 0
   */
  void setFrontPadding(const double front_padding);

  /**
   * @brief 取当前生效的前向外扩大小
   */
  double getFrontPadding() const;

  /**
   * @brief 设置后向额外膨胀大小（支持倒车按速度后向外扩停障距离）
   * @param rear_padding 后向额外外扩大小 m；NaN 或负数表示置 0
   */
  void setRearPadding(const double rear_padding);

  /**
   * @brief 取当前生效的后向外扩大小
   */
  double getRearPadding() const;

  /**
   * @brief 取配置里的默认膨胀大小
   */
  double getDefaultPadding() const;

  /**
   * @brief 取配置里的默认前向额外膨胀大小
   */
  double getDefaultFrontPadding() const;

  /**
   * @brief 取配置里的默认后向额外膨胀大小
   */
  double getDefaultRearPadding() const;

  /** @brief 切换仅横向膨胀模式（Y/左右膨胀，X/前后不膨胀）。 */
  void setLateralPaddingOnly(bool enabled);

  /** @brief 当前是否仅做横向膨胀。 */
  bool isLateralPaddingOnly() const;

  /**
   * @brief Updates polygon from footprint subscriber (if any)
   */
  void updatePolygon();
  bool missingFootprint() const {return footprint_sub_ != nullptr && poly_.size() < 3;}

  /**
   * @brief Gets number of points inside given polygon
   * @param points Input array of points to be checked
   * @param early_exit_threshold If > 0, stops counting and returns immediately when points reach this threshold
   * @return Number of points inside polygon. If there are no points,
   * returns zero value.
   */
  virtual int getPointsInside(
    const std::vector<Point> & points,
    int early_exit_threshold = -1) const;

  /**
   * @brief Gets number of points inside given polygon across multiple point buffers
   * (Zero-copy optimization: avoids concatenating point clouds).
   * @param point_buffers Vector of pointers to point cloud buffers
   * @param early_exit_threshold If > 0, stops counting and returns immediately when points reach this threshold
   * @return Number of points inside polygon.
   */
  virtual int getPointsInside(
    const std::vector<const std::vector<Point> *> & point_buffers,
    int early_exit_threshold = -1) const;

  /**
   * @brief Obtains estimated (simulated) time before a collision.
   * Applicable for APPROACH model.
   * @param collision_points Array of 2D obstacle points
   * @param velocity Simulated robot velocity
   * @return Estimated time before a collision. If there is no collision,
   * return value will be negative.
   */
  double getCollisionTime(
    const std::vector<Point> & collision_points,
    const Velocity & velocity) const;

  /**
   * @brief Publishes polygon message into a its own topic
   */
  void publish() const;

protected:
  /**
   * @brief Supporting routine obtaining ROS-parameters common for all shapes
   * @param polygon_pub_topic Output name of polygon publishing topic
   * @return True if all parameters were obtained or false in failure case
   */
  bool getCommonParameters(std::string & polygon_pub_topic);

  /**
   * @brief Supporting routine obtaining polygon-specific ROS-parameters
   * @param polygon_pub_topic Output name of polygon publishing topic
   * @param footprint_topic Output name of footprint topic. Empty, if no footprint subscription
   * @return True if all parameters were obtained or false in failure case
   */
  virtual bool getParameters(std::string & polygon_pub_topic, std::string & footprint_topic);

  /**
   * @brief Checks if point is inside polygon
   * @param point Given point to check
   * @return True if given point is inside polygon, otherwise false
   */
  bool isPointInside(const Point & point) const;

  /**
   * @brief 点到基础多边形边界的最短距离（不区分内外，取各边线段距离的最小值）
   * @param point 待测点
   * @return 最短距离 m；poly_ 为空时返回 +inf
   */
  double distanceToBoundary(const Point & point) const;

  // ----- Variables -----

  /// @brief Collision Monitor node
  nav2_util::LifecycleNode::WeakPtr node_;
  /// @brief Collision monitor node logger stored for further usage
  rclcpp::Logger logger_{rclcpp::get_logger("collision_monitor")};

  // Basic parameters
  /// @brief Name of polygon
  std::string polygon_name_;
  /// @brief Action type for the polygon
  ActionType action_type_;
  /// @brief Maximum number of data readings within a zone to not trigger the action
  int max_points_;
  /// @brief Robot slowdown (share of its actual speed)
  double slowdown_ratio_;
  /// @brief Time before collision in seconds
  double time_before_collision_;
  /// @brief Time step for robot movement simulation
  double simulation_time_step_;
  /// @brief Footprint subscriber
  std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;

  // Global variables
  /// @brief TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  /// @brief Base frame ID
  std::string base_frame_id_;
  /// @brief Transform tolerance
  tf2::Duration transform_tolerance_;

  // Visualization
  /// @brief Whether to publish the polygon
  bool visualize_;
  /// @brief Polygon points stored for later publishing
  geometry_msgs::msg::Polygon polygon_;
  /// @brief Polygon publisher for visualization purposes
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;

  /// @brief Polygon points (vertices)。始终是"基础形状"，不含 padding
  std::vector<Point> poly_;

  /// @brief 基础轮廓 AABB 包围盒（用于快速剪枝过滤远端点）
  double base_min_x_{0.0};
  double base_max_x_{0.0};
  double base_min_y_{0.0};
  double base_max_y_{0.0};
  void updateBoundingBox();

  /// @brief 上次可视化多边形发布时间戳（用于 5Hz 节流降频）
  mutable rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};

  // 动态膨胀（本仓库相对上游 Nav2 的改动）
  /// @brief 配置项 <name>.default_padding，收到 NaN 时回落到该值
  double default_padding_{0.0};
  /// @brief 配置项 <name>.front_padding，默认前向额外外扩大小
  double default_front_padding_{0.0};
  /// @brief 配置项 <name>.rear_padding，默认后向额外外扩大小
  double default_rear_padding_{0.0};
  /// @brief 当前生效的膨胀大小 m。service 线程写、process() 线程读，故用 atomic
  std::atomic<double> padding_{0.0};
  /// @brief true 时 padding 仅作用于 base_link 的 Y 方向
  std::atomic<bool> lateral_padding_only_{false};
  /// @brief 当前生效的前向额外膨胀大小 m。主控制循环写、isPointInside() 读，用 atomic
  std::atomic<double> front_padding_{0.0};
  /// @brief 当前生效的后向额外膨胀大小 m。主控制循环写、isPointInside() 读，用 atomic
  std::atomic<double> rear_padding_{0.0};
};  // class Polygon

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__POLYGON_HPP_
