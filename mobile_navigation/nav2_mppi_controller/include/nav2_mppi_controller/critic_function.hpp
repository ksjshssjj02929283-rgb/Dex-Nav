// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#ifndef NAV2_MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_
#define NAV2_MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_

#include <cstddef>
#include <string>
#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/critic_data.hpp"

namespace mppi::critics
{

struct GoalCriticSettings
{
  float weight{0.0f};
  float threshold_to_consider{0.0f};
};

struct GoalLateralCriticSettings
{
  float weight{0.0f};
  float threshold_to_consider{0.0f};
  size_t last_n_points{0};
};

struct GoalAngleCriticSettings
{
  float weight{0.0f};
  float threshold_to_consider{0.0f};
};

struct PathAngleCriticSettings
{
  float weight{0.0f};
  float threshold_to_consider{0.0f};
  int offset_from_furthest{0};
  float max_angle_to_furthest{0.0f};
  bool forward_preference{true};
};

struct PathFollowCriticSettings
{
  float weight{0.0f};
  float threshold_to_consider{0.0f};
  int offset_from_furthest{0};
};

struct PathAlignCriticSettings
{
  float weight{0.0f};
  float threshold_to_consider{0.0f};
  int offset_from_furthest{0};
  int trajectory_point_step{0};
  float max_path_occupancy_ratio{0.0f};
};

struct PreferForwardCriticSettings
{
  float weight{0.0f};
  float threshold_to_consider{0.0f};
};

struct CriticProfileSettings
{
  GoalCriticSettings goal;
  GoalLateralCriticSettings goal_lateral;
  GoalAngleCriticSettings goal_angle;
  PathAngleCriticSettings path_angle;
  PathFollowCriticSettings path_follow;
  PathAlignCriticSettings path_align;
  PreferForwardCriticSettings prefer_forward;
};

/**
 * @class mppi::critics::CollisionCost
 * @brief Utility for storing cost information
 */
struct CollisionCost
{
  float cost{0};
  bool using_footprint{false};
};

/**
 * @class mppi::critics::CriticFunction
 * @brief Abstract critic objective function to score trajectories
 */
class CriticFunction
{
public:
  /**
    * @brief Constructor for mppi::critics::CriticFunction
    */
  CriticFunction() = default;

  /**
    * @brief Destructor for mppi::critics::CriticFunction
    */
  virtual ~CriticFunction() = default;

  /**
    * @brief Configure critic on bringup
    * @param parent WeakPtr to node
    * @param parent_name name of the controller
    * @param name Name of plugin
    * @param costmap_ros Costmap2DROS object of environment
    * @param dynamic_parameter_handler Parameter handler object
    */
  void on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
    const std::string & parent_name,
    const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    ParametersHandler * param_handler)
  {
    parent_ = parent;
    logger_ = parent_.lock()->get_logger();
    name_ = name;
    parent_name_ = parent_name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    parameters_handler_ = param_handler;

    auto getParam = parameters_handler_->getParamGetter(name_);
    getParam(enabled_, "enabled", true);

    initialize();
  }

  /**
    * @brief Main function to score trajectory
    * @param data Critic data to use in scoring
    */
  virtual void score(CriticData & data) = 0;

  /**
    * @brief Initialize critic
    */
  virtual void initialize() = 0;

  /**
    * @brief Apply runtime profile settings, if supported by the critic
    * @param settings Runtime critic profile settings
    */
  virtual void applyProfileSettings(const CriticProfileSettings & /* settings */) {}

  /**
    * @brief Get name of critic
    */
  std::string getName()
  {
    return name_;
  }

protected:
  bool enabled_;
  std::string name_, parent_name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  ParametersHandler * parameters_handler_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_
