// Copyright (c) 2026 lovebc
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

#include "nav2_costmap_2d/esdf_layer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_costmap_2d
{

void EsdfLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("influence_radius", rclcpp::ParameterValue(2.0));
  declareParameter("cost_scaling_factor", rclcpp::ParameterValue(3.0));
  declareParameter("use_exponential_decay", rclcpp::ParameterValue(true));
  declareParameter("treat_unknown_as_obstacle", rclcpp::ParameterValue(false));
  declareParameter("combination_method", rclcpp::ParameterValue(static_cast<int>(CombinationMethod::Max)));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"EsdfLayer::onInitialize: Failed to lock node"};
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "influence_radius", influence_radius_);
  node->get_parameter(name_ + "." + "cost_scaling_factor", cost_scaling_factor_);
  node->get_parameter(name_ + "." + "use_exponential_decay", use_exponential_decay_);
  node->get_parameter(name_ + "." + "treat_unknown_as_obstacle", treat_unknown_as_obstacle_);
  node->get_parameter(name_ + "." + "combination_method", combination_method_);
  applyParameterBounds();

  inscribed_radius_ = layered_costmap_->getInscribedRadius();
  setDefaultValue(NO_INFORMATION);
  matchSize();
  current_ = true;

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&EsdfLayer::dynamicParametersCallback, this, std::placeholders::_1));
}

void EsdfLayer::matchSize()
{
  CostmapLayer::matchSize();
  setDefaultValue(NO_INFORMATION);
  resetMaps();
  const std::size_t size =
    static_cast<std::size_t>(getSizeInCellsX()) * static_cast<std::size_t>(getSizeInCellsY());
  squared_distances_cells_.assign(size, kInfDistanceCells);
}

void EsdfLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  Costmap2D * master = layered_costmap_->getCostmap();
  *min_x = std::min(*min_x, master->getOriginX());
  *min_y = std::min(*min_y, master->getOriginY());
  *max_x = std::max(*max_x, master->getOriginX() + master->getSizeInMetersX());
  *max_y = std::max(*max_y, master->getOriginY() + master->getSizeInMetersY());
}

void EsdfLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }

  if (min_i >= max_i || min_j >= max_j) {
    return;
  }

  updateDistanceCache(master_grid);
  resetMap(
    static_cast<unsigned int>(min_i), static_cast<unsigned int>(min_j),
    static_cast<unsigned int>(max_i), static_cast<unsigned int>(max_j));

  const unsigned int size_x = master_grid.getSizeInCellsX();
  const double resolution = master_grid.getResolution();

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      const unsigned int mx = static_cast<unsigned int>(i);
      const unsigned int my = static_cast<unsigned int>(j);
      const unsigned int index = master_grid.getIndex(mx, my);

      if (isObstacleSite(master_grid.getCost(mx, my))) {
        continue;
      }

      const float squared_cells = squared_distances_cells_[index];
      if (!std::isfinite(squared_cells) || squared_cells >= kInfDistanceCells) {
        continue;
      }

      const double distance_m = std::sqrt(static_cast<double>(squared_cells)) * resolution;
      const unsigned char cost = distanceToCost(distance_m);
      if (cost == FREE_SPACE) {
        continue;
      }
      costmap_[static_cast<unsigned int>(j) * size_x + static_cast<unsigned int>(i)] = cost;
    }
  }

  updateWithCombination(master_grid, min_i, min_j, max_i, max_j);
  current_ = true;
}

void EsdfLayer::reset()
{
  resetMaps();
  current_ = false;
}

bool EsdfLayer::isClearable()
{
  return false;
}

void EsdfLayer::onFootprintChanged()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  inscribed_radius_ = layered_costmap_->getInscribedRadius();
  current_ = false;
  addExtraBounds(
    layered_costmap_->getCostmap()->getOriginX(),
    layered_costmap_->getCostmap()->getOriginY(),
    layered_costmap_->getCostmap()->getOriginX() + layered_costmap_->getCostmap()->getSizeInMetersX(),
    layered_costmap_->getCostmap()->getOriginY() + layered_costmap_->getCostmap()->getSizeInMetersY());
}

rcl_interfaces::msg::SetParametersResult EsdfLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    const std::string & param_name = parameter.get_name();
    if (param_name == name_ + "." + "enabled") {
      enabled_ = parameter.as_bool();
    } else if (param_name == name_ + "." + "influence_radius") {
      influence_radius_ = parameter.as_double();
    } else if (param_name == name_ + "." + "cost_scaling_factor") {
      cost_scaling_factor_ = parameter.as_double();
    } else if (param_name == name_ + "." + "use_exponential_decay") {
      use_exponential_decay_ = parameter.as_bool();
    } else if (param_name == name_ + "." + "treat_unknown_as_obstacle") {
      treat_unknown_as_obstacle_ = parameter.as_bool();
    } else if (param_name == name_ + "." + "combination_method") {
      combination_method_ = parameter.as_int();
    }
  }

  applyParameterBounds();
  current_ = false;
  addExtraBounds(
    layered_costmap_->getCostmap()->getOriginX(),
    layered_costmap_->getCostmap()->getOriginY(),
    layered_costmap_->getCostmap()->getOriginX() + layered_costmap_->getCostmap()->getSizeInMetersX(),
    layered_costmap_->getCostmap()->getOriginY() + layered_costmap_->getCostmap()->getSizeInMetersY());
  return result;
}

void EsdfLayer::updateDistanceCache(const nav2_costmap_2d::Costmap2D & master_grid)
{
  const std::size_t size =
    static_cast<std::size_t>(master_grid.getSizeInCellsX()) *
    static_cast<std::size_t>(master_grid.getSizeInCellsY());
  if (squared_distances_cells_.size() != size) {
    squared_distances_cells_.assign(size, kInfDistanceCells);
  }
  squared_distances_cells_ = computeDistanceTransform(master_grid);
}

std::vector<float> EsdfLayer::computeDistanceTransform(
  const nav2_costmap_2d::Costmap2D & master_grid) const
{
  const unsigned int width = master_grid.getSizeInCellsX();
  const unsigned int height = master_grid.getSizeInCellsY();
  const std::size_t size = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);

  std::vector<float> column_pass(size, kInfDistanceCells);
  std::vector<float> line(std::max(width, height), kInfDistanceCells);
  bool has_site = false;

  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      if (isObstacleSite(master_grid.getCost(x, y))) {
        has_site = true;
        break;
      }
    }
    if (has_site) {
      break;
    }
  }

  if (!has_site) {
    return column_pass;
  }

  for (unsigned int x = 0; x < width; ++x) {
    for (unsigned int y = 0; y < height; ++y) {
      line[y] = isObstacleSite(master_grid.getCost(x, y)) ? 0.0F : kInfDistanceCells;
    }
    const std::vector<float> transformed = distanceTransform1D(
      std::vector<float>(line.begin(), line.begin() + static_cast<std::ptrdiff_t>(height)));
    for (unsigned int y = 0; y < height; ++y) {
      column_pass[master_grid.getIndex(x, y)] = transformed[y];
    }
  }

  std::vector<float> output(size, kInfDistanceCells);
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      line[x] = column_pass[master_grid.getIndex(x, y)];
    }
    const std::vector<float> transformed = distanceTransform1D(
      std::vector<float>(line.begin(), line.begin() + static_cast<std::ptrdiff_t>(width)));
    for (unsigned int x = 0; x < width; ++x) {
      output[master_grid.getIndex(x, y)] = transformed[x];
    }
  }

  return output;
}

std::vector<float> EsdfLayer::distanceTransform1D(const std::vector<float> & source) const
{
  const int n = static_cast<int>(source.size());
  std::vector<float> output(source.size(), kInfDistanceCells);
  if (n == 0) {
    return output;
  }

  std::vector<int> lower_envelope(static_cast<std::size_t>(n), 0);
  std::vector<float> boundaries(static_cast<std::size_t>(n) + 1U, 0.0F);

  int first_site = -1;
  for (int i = 0; i < n; ++i) {
    if (source[static_cast<std::size_t>(i)] < kInfDistanceCells) {
      first_site = i;
      break;
    }
  }

  if (first_site < 0) {
    return output;
  }

  int k = 0;
  lower_envelope[0] = first_site;
  boundaries[0] = -kInfDistanceCells;
  boundaries[1] = kInfDistanceCells;

  for (int q = first_site + 1; q < n; ++q) {
    if (source[static_cast<std::size_t>(q)] >= kInfDistanceCells) {
      continue;
    }

    float intersection = 0.0F;
    do {
      const int p = lower_envelope[static_cast<std::size_t>(k)];
      intersection =
        ((source[static_cast<std::size_t>(q)] + static_cast<float>(q * q)) -
        (source[static_cast<std::size_t>(p)] + static_cast<float>(p * p))) /
        (2.0F * static_cast<float>(q - p));
      if (intersection <= boundaries[static_cast<std::size_t>(k)]) {
        --k;
      }
    } while (k >= 0 && intersection <= boundaries[static_cast<std::size_t>(k)]);

    ++k;
    lower_envelope[static_cast<std::size_t>(k)] = q;
    boundaries[static_cast<std::size_t>(k)] = intersection;
    boundaries[static_cast<std::size_t>(k + 1)] = kInfDistanceCells;
  }

  k = 0;
  for (int q = 0; q < n; ++q) {
    while (boundaries[static_cast<std::size_t>(k + 1)] < static_cast<float>(q)) {
      ++k;
    }
    const int p = lower_envelope[static_cast<std::size_t>(k)];
    const int diff = q - p;
    output[static_cast<std::size_t>(q)] =
      static_cast<float>(diff * diff) + source[static_cast<std::size_t>(p)];
  }

  return output;
}

unsigned char EsdfLayer::distanceToCost(double distance_m) const
{
  if (distance_m <= inscribed_radius_) {
    return INSCRIBED_INFLATED_OBSTACLE;
  }
  if (distance_m >= influence_radius_) {
    return FREE_SPACE;
  }

  const double clearance = distance_m - inscribed_radius_;
  const double cost_range = static_cast<double>(INSCRIBED_INFLATED_OBSTACLE - 1);
  double scaled_cost = 0.0;
  if (use_exponential_decay_) {
    scaled_cost = cost_range * std::exp(-cost_scaling_factor_ * clearance);
  } else {
    const double usable_radius = std::max(0.001, influence_radius_ - inscribed_radius_);
    scaled_cost = cost_range * (influence_radius_ - distance_m) / usable_radius;
  }

  const int cost = std::clamp(static_cast<int>(std::lround(scaled_cost)), 1, INSCRIBED_INFLATED_OBSTACLE - 1);
  return static_cast<unsigned char>(cost);
}

bool EsdfLayer::isObstacleSite(unsigned char cost) const
{
  if (cost == LETHAL_OBSTACLE) {
    return true;
  }
  return treat_unknown_as_obstacle_ && cost == NO_INFORMATION;
}

void EsdfLayer::updateWithCombination(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  switch (static_cast<CombinationMethod>(combination_method_)) {
    case CombinationMethod::Overwrite:
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case CombinationMethod::Addition:
      updateWithAddition(master_grid, min_i, min_j, max_i, max_j);
      break;
    case CombinationMethod::Max:
    default:
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
  }
}

void EsdfLayer::applyParameterBounds()
{
  influence_radius_ = std::max(0.01, influence_radius_);
  cost_scaling_factor_ = std::max(0.01, cost_scaling_factor_);
  if (combination_method_ < static_cast<int>(CombinationMethod::Overwrite) ||
    combination_method_ > static_cast<int>(CombinationMethod::Addition))
  {
    RCLCPP_WARN(
      logger_,
      "EsdfLayer combination_method=%d is invalid. Using Max combination.",
      combination_method_);
    combination_method_ = static_cast<int>(CombinationMethod::Max);
  }
}

}  // namespace nav2_costmap_2d

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::EsdfLayer, nav2_costmap_2d::Layer)
