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

#ifndef NAV2_COSTMAP_2D__ESDF_LAYER_HPP_
#define NAV2_COSTMAP_2D__ESDF_LAYER_HPP_

#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_costmap_2d
{

/**
 * @class EsdfLayer
 * @brief Builds a 2D Euclidean distance cost field from lethal cells in the master costmap.
 */
class EsdfLayer : public CostmapLayer
{
  friend class EsdfLayerTester;

public:
  EsdfLayer() = default;
  ~EsdfLayer() override = default;

  void onInitialize() override;
  void matchSize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override;
  void onFootprintChanged() override;

private:
  enum class CombinationMethod
  {
    Overwrite = 0,
    Max = 1,
    Addition = 2
  };

  static constexpr float kInfDistanceCells = 1.0e20F;

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  void updateDistanceCache(const nav2_costmap_2d::Costmap2D & master_grid);
  std::vector<float> computeDistanceTransform(
    const nav2_costmap_2d::Costmap2D & master_grid) const;
  std::vector<float> distanceTransform1D(const std::vector<float> & source) const;
  unsigned char distanceToCost(double distance_m) const;
  bool isObstacleSite(unsigned char cost) const;
  void updateWithCombination(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);
  void applyParameterBounds();

  double influence_radius_ = 2.0;
  double cost_scaling_factor_ = 3.0;
  bool use_exponential_decay_ = true;
  bool treat_unknown_as_obstacle_ = false;
  int combination_method_ = static_cast<int>(CombinationMethod::Max);
  double inscribed_radius_ = 0.0;

  std::vector<float> squared_distances_cells_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__ESDF_LAYER_HPP_
