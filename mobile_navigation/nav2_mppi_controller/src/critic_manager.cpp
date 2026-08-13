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

#include "nav2_mppi_controller/critic_manager.hpp"

#include <chrono>

namespace mppi
{

void CriticManager::on_configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros, ParametersHandler * param_handler)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  name_ = name;
  auto node = parent_.lock();
  logger_ = node->get_logger();
  parameters_handler_ = param_handler;

  getParams();
  loadCritics();
}

void CriticManager::getParams()
{
  auto node = parent_.lock();
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(critic_names_, "critics", std::vector<std::string>{}, ParameterType::Static);
}

void CriticManager::loadCritics()
{
  if (!loader_) {
    loader_ = std::make_unique<pluginlib::ClassLoader<critics::CriticFunction>>(
      "nav2_mppi_controller", "mppi::critics::CriticFunction");
  }

  critics_.clear();
  for (auto name : critic_names_) {
    std::string fullname = getFullName(name);
    auto instance = std::unique_ptr<critics::CriticFunction>(
      loader_->createUnmanagedInstance(fullname));
    critics_.push_back(std::move(instance));
    critics_.back()->on_configure(
      parent_, name_, name_ + "." + name, costmap_ros_,
      parameters_handler_);
    RCLCPP_INFO(logger_, "Critic loaded : %s", fullname.c_str());
  }
}

std::string CriticManager::getFullName(const std::string & name)
{
  return "mppi::critics::" + name;
}

void CriticManager::evalTrajectoriesScores(
  CriticData & data) const
{
  constexpr double kSlowCriticWarnMs = 100.0;
  for (size_t q = 0; q < critics_.size(); q++) {
    if (data.fail_flag) {
      break;
    }

    auto t_start = std::chrono::high_resolution_clock::now();
    critics_[q]->score(data);
    auto t_end = std::chrono::high_resolution_clock::now();
    const double critic_ms =
      std::chrono::duration<double, std::milli>(t_end - t_start).count();

    if (critic_ms >= kSlowCriticWarnMs) {
      RCLCPP_WARN(
        logger_, "Critic timing (ms): name=%s cost=%.3f fail=%d",
        critic_names_[q].c_str(), critic_ms, data.fail_flag);
    }
  }
}

void CriticManager::applyCriticProfileSettings(const critics::CriticProfileSettings & settings)
{
  for (auto & critic : critics_) {
    critic->applyProfileSettings(settings);
  }

  RCLCPP_DEBUG(
    logger_,
    "MPPI critic profile applied: Goal(weight=%.3f), "
    "GoalLateral(weight=%.3f), "
    "GoalLongitudinal(weight=%.3f), "
    "PathAlign(weight=%.3f offset=%d step=%d)",
    settings.goal.weight,
    settings.goal_lateral.weight,
    settings.goal_longitudinal.weight,
    settings.path_align.weight, settings.path_align.offset_from_furthest,
    settings.path_align.trajectory_point_step);
}

}  // namespace mppi
