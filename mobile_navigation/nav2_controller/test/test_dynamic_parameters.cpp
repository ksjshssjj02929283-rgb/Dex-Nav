// Copyright (c) 2021, Samsung Research America
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
// limitations under the License. Reserved.

#include <math.h>

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_controller/controller_server.hpp"
#include "rclcpp/rclcpp.hpp"

class ControllerShim : public nav2_controller::ControllerServer
{
public:
  ControllerShim()
  : nav2_controller::ControllerServer(rclcpp::NodeOptions())
  {
  }

  // Since we cannot call configure/activate due to costmaps
  // requiring TF
  void setDynamicCallback()
  {
    auto node = shared_from_this();
    // Add callback for dynamic parameters
    dyn_params_handler_ = node->add_on_set_parameters_callback(
      std::bind(&ControllerShim::dynamicParamsShim, this, std::placeholders::_1));
  }

  rcl_interfaces::msg::SetParametersResult
  dynamicParamsShim(std::vector<rclcpp::Parameter> parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    dynamicParametersCallback(parameters);
    return result;
  }

  bool lateralStallEnabled() const {return near_goal_lateral_stall_enabled_;}
  double lateralStallLonTolerance() const
  {
    return near_goal_lateral_stall_lon_tolerance_;
  }
  double lateralStallYawTolerance() const
  {
    return near_goal_lateral_stall_yaw_tolerance_;
  }
  double lateralStallLatTolerance() const
  {
    return near_goal_lateral_stall_lat_tolerance_;
  }
  double lateralStallMinImprovement() const
  {
    return near_goal_lateral_stall_min_improvement_;
  }
  double lateralStallTimeAllowance() const
  {
    return near_goal_lateral_stall_time_allowance_.seconds();
  }
};

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(WPTest, test_dynamic_parameters)
{
  auto controller = std::make_shared<ControllerShim>();
  controller->setDynamicCallback();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    controller->get_node_base_interface(), controller->get_node_topics_interface(),
    controller->get_node_graph_interface(),
    controller->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("controller_frequency", 100.0),
      rclcpp::Parameter("min_x_velocity_threshold", 100.0),
      rclcpp::Parameter("min_y_velocity_threshold", 100.0),
      rclcpp::Parameter("min_theta_velocity_threshold", 100.0),
      rclcpp::Parameter("failure_tolerance", 5.0)});

  rclcpp::spin_until_future_complete(
    controller->get_node_base_interface(),
    results);

  EXPECT_EQ(controller->get_parameter("controller_frequency").as_double(), 100.0);
  EXPECT_EQ(controller->get_parameter("min_x_velocity_threshold").as_double(), 100.0);
  EXPECT_EQ(controller->get_parameter("min_y_velocity_threshold").as_double(), 100.0);
  EXPECT_EQ(controller->get_parameter("min_theta_velocity_threshold").as_double(), 100.0);
  EXPECT_EQ(controller->get_parameter("failure_tolerance").as_double(), 5.0);
}

TEST(WPTest, test_near_goal_lateral_stall_dynamic_parameters)
{
  auto controller = std::make_shared<ControllerShim>();
  controller->setDynamicCallback();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    controller->get_node_base_interface(), controller->get_node_topics_interface(),
    controller->get_node_graph_interface(),
    controller->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("near_goal_progress.lateral_stall_enabled", true),
      rclcpp::Parameter("near_goal_progress.lateral_stall_lon_tolerance", 0.02),
      rclcpp::Parameter("near_goal_progress.lateral_stall_yaw_tolerance", 0.05),
      rclcpp::Parameter("near_goal_progress.lateral_stall_lat_tolerance", 0.02),
      rclcpp::Parameter("near_goal_progress.lateral_stall_min_improvement", 0.005),
      rclcpp::Parameter("near_goal_progress.lateral_stall_time_allowance", 3.0)});

  rclcpp::spin_until_future_complete(
    controller->get_node_base_interface(), results);

  EXPECT_TRUE(controller->lateralStallEnabled());
  EXPECT_DOUBLE_EQ(controller->lateralStallLonTolerance(), 0.02);
  EXPECT_DOUBLE_EQ(controller->lateralStallYawTolerance(), 0.05);
  EXPECT_DOUBLE_EQ(controller->lateralStallLatTolerance(), 0.02);
  EXPECT_DOUBLE_EQ(controller->lateralStallMinImprovement(), 0.005);
  EXPECT_DOUBLE_EQ(controller->lateralStallTimeAllowance(), 3.0);
}
