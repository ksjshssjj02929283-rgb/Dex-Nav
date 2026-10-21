/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020 Samsung Research Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the <ORGANIZATION> nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *********************************************************************/

#ifndef NAV2_COSTMAP_2D__COSTMAP_FILTERS__COSTMAP_FILTER_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_FILTERS__COSTMAP_FILTER_HPP_

#include <string>
#include <mutex>
#include <atomic>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace nav2_costmap_2d
{

/**
 * @brief: CostmapFilter basic class. It is inherited from Layer in order to avoid
 * hidden problems when the shared handling of costmap_ resource (PR #1936)
 */
class CostmapFilter : public Layer
{
public:
  /**
   * @brief A constructor
   */
  CostmapFilter();
  /**
   * @brief A destructor
   */
  ~CostmapFilter();

  /**
   * @brief: Provide a typedef to ease future code maintenance
   */
  typedef std::recursive_mutex mutex_t;
  /**
   * @brief: returns pointer to a mutex
   */
  mutex_t * getMutex()
  {
    return access_;
  }

  /**
   * @brief Initialization process of layer on startup
   */
  void onInitialize() final;

  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) final;

  /**
   * @brief Update the costs in the master costmap in the window
   * @param master_grid The master costmap grid to update
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) final;

  /**
   * @brief Activate the layer
   */
  void activate() final;
  /**
   * @brief Deactivate the layer
   */
  void deactivate() final;
  /**
   * @brief Reset the layer
   */
  void reset() final;

  /**
   * @brief If clearing operations should be processed on this layer or not
   */
  bool isClearable() {return false;}

  /** CostmapFilter API **/
  /**
   * @brief: Initializes costmap filter. Creates subscriptions to filter-related topics
   * @param: Name of costmap filter info topic
   */
  virtual void initializeFilter(
    const std::string & filter_info_topic) = 0;

  /**
   * @brief: An algorithm for how to use that map's information. Fills the Costmap2D with
   *         calculated data and makes an action based on processed data
   * @param: Reference to a master costmap2d
   * @param: Low window map boundary OX
   * @param: Low window map boundary OY
   * @param: High window map boundary OX
   * @param: High window map boundary OY
   * @param: Robot 2D-pose
   */
  virtual void process(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j,
    const geometry_msgs::msg::Pose2D & pose) = 0;

  /**
   * @brief: Resets costmap filter. Stops all subscriptions
   */
  virtual void resetFilter() = 0;

protected:
  /**
   * @brief Costmap filter enabling/disabling callback
   * @param request_header Service request header
   * @param request Service request
   * @param response Service response
   */
  void enableCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /**
   * @brief Warn once that the filter mask has not been received yet.
   * The warning state is reset after a valid mask arrives.
   * @param filter_name Human-readable filter name to be shown in logs
   */
  void warnMaskNotReceivedOnce(const char * filter_name);

  /**
   * @brief Mark that a valid filter mask has been received and re-arm the one-shot warning.
   */
  void maskReceived();

  /**
   * @brief Reset the one-shot warning state when filter subscriptions are recreated.
   */
  void resetMaskWarningState();

  /**
   * @brief 请求下一轮 updateBounds 把更新范围扩到整张 costmap 一次。
   * 过滤器自身不上报更新范围，掩膜变化后只有其它图层本轮更新的窗口会重算；
   * 掩膜内容会改变 costmap 的过滤器（如 keepout）在收到新掩膜时调用，使新增、删除的区域全图生效。
   * 可在任意线程调用。
   */
  void requestFullUpdate();

  /**
   * @brief:  Transforms robot pose from current layer frame to mask frame
   * @param:  global_frame Costmap frame to transform from
   * @param:  global_pose Robot pose in costmap frame
   * @param:  mask_frame Filter mask frame to transform to
   * @param:  mask_pose Output robot pose in mask frame
   * @return: True if the transformation was successful, false otherwise
   */
  bool transformPose(
    const std::string global_frame,
    const geometry_msgs::msg::Pose2D & global_pose,
    const std::string mask_frame,
    geometry_msgs::msg::Pose2D & mask_pose) const;

  /**
   * @brief: Convert from world coordinates to mask coordinates.
     Similar to Costmap2D::worldToMap() method but works directly with OccupancyGrid-s.
   * @param  filter_mask Filter mask on which to convert
   * @param  wx The x world coordinate
   * @param  wy The y world coordinate
   * @param  mx Will be set to the associated mask x coordinate
   * @param  my Will be set to the associated mask y coordinate
   * @return True if the conversion was successful (legal bounds) false otherwise
   */
  bool worldToMask(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr filter_mask,
    double wx, double wy, unsigned int & mx, unsigned int & my) const;

  /**
   * @brief  Get the data of a cell in the filter mask
   * @param  filter_mask Filter mask to get the data from
   * @param  mx The x coordinate of the cell
   * @param  my The y coordinate of the cell
   * @return The data of the selected cell
   */
  inline int8_t getMaskData(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr filter_mask,
    const unsigned int mx, const unsigned int my) const
  {
    return filter_mask->data[my * filter_mask->info.width + mx];
  }

  /**
   * @brief: Name of costmap filter info topic
   */
  std::string filter_info_topic_;

  /**
   * @brief: Name of filter mask topic
   */
  std::string mask_topic_;

  /**
   * @brief: mask_frame_->global_frame_ transform tolerance
   */
  tf2::Duration transform_tolerance_;

  /**
   * @brief: A service to enable/disable costmap filter
   */
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;

private:
  /**
   * @brief: Latest robot position
   */
  geometry_msgs::msg::Pose2D latest_pose_;

  /**
   * @brief: Mutex for locking filter's resources
   */
  mutex_t * access_;

  /**
   * @brief: Tracks whether the "mask was not received" warning has already been emitted.
   */
  bool mask_not_received_warned_;

  /**
   * @brief: 是否在下一轮 updateBounds 扩到整张 costmap（requestFullUpdate 置位，消费后清除）
   */
  std::atomic<bool> full_update_requested_{false};
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COSTMAP_FILTERS__COSTMAP_FILTER_HPP_
