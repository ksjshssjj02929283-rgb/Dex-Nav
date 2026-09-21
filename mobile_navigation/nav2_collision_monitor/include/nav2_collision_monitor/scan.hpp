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

#ifndef NAV2_COLLISION_MONITOR__SCAN_HPP_
#define NAV2_COLLISION_MONITOR__SCAN_HPP_

#include <memory>
#include <string>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav2_util/robot_utils.hpp"

#include "nav2_collision_monitor/source.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Implementation for laser scanner source
 */
class Scan : public Source
{
public:
  /**
   * @brief Scan constructor
   * @param node Collision Monitor node pointer
   * @param source_name Name of data source
   * @param tf_buffer Shared pointer to a TF buffer
   * @param base_frame_id Robot base frame ID. The output data will be transformed into this frame.
   * @param global_frame_id Global frame ID for correct transform calculation
   * @param transform_tolerance Maximum age of the latest dynamic base TF (no waiting)
   * @param source_timeout Maximum time interval in which data is considered valid
   * @param base_shift_correction Whether to correct source data towards to base frame movement,
   * from sensor acquisition time to the latest available base TF time
   */
  Scan(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & source_timeout,
    const bool base_shift_correction);
  /**
   * @brief Scan destructor
   */
  ~Scan();

  /**
   * @brief Data source configuration routine. Obtains ROS-parameters
   * and creates laser scanner subscriber.
   */
  void configure();

  /**
   * @brief Adds latest data from laser scanner to the data array.
   * @param curr_time Current node time for data interpolation
   * @param data Array where the data from source to be added.
   * Added data is cached in base_frame_id_ at the latest TF time at scan processing.
   */
  void getData(
    const rclcpp::Time & curr_time,
    std::vector<Point> & data) const;

protected:
  /**
   * @brief Laser scanner data callback
   * @param msg Shared pointer to LaserScan message
   */
  void dataCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

  // ----- Variables -----

  /// @brief Laser scanner data subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr data_sub_;

  // Retain the latest raw message for diagnostics and source-level compatibility. Detection uses
  // processed_data_; readers of data_ must use atomic shared_ptr operations.
  sensor_msgs::msg::LaserScan::ConstSharedPtr data_;

  struct ProcessedScan
  {
    rclcpp::Time source_stamp;
    std::vector<Point> points;
    bool transform_valid{false};
  };

  // A callback builds the complete base-frame point set and publishes it atomically. The
  // 100 Hz safety loop therefore never observes a partially converted scan and never performs TF.
  std::shared_ptr<const ProcessedScan> processed_data_;

  struct DirectionCache
  {
    float angle_min;
    float angle_increment;
    std::size_t size;
    std::vector<Point> unit_vectors;
  };

  // Immutable beam directions keyed by LaserScan geometry. Access through atomic shared_ptr
  // operations so concurrent snapshot tests and executor callbacks never observe a partial cache.
  std::shared_ptr<const DirectionCache> direction_cache_;
};  // class Scan

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__SCAN_HPP_
