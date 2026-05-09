#pragma once

#include <smacc2/smacc.hpp>

namespace smacc2_nav_mission
{

using namespace cl_nav2z;
using namespace smacc2::default_transition_tags;

struct StNavigateToPickup : smacc2::SmaccState<StNavigateToPickup, SmNavMission>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvActionSucceeded<ClNav2Z, OrNavigation>, StNavigateToDropoff, SUCCESS>,
    Transition<EvActionAborted<ClNav2Z, OrNavigation>,  StMissionComplete,   ABORT>
  > reactions;

  static void staticConfigure()
  {
    // 点位名称来自 nav_point_manager 的 points.json，运行时从 GetNavPoint 服务获取坐标
    // 默认点位名称 "pickup"，可通过 ROS 参数 "pickup_point_name" 覆盖
    configure_orthogonal<OrNavigation, CbNavigateToNavPoint>("pickup");
  }

  void runtimeConfigure()
  {
    // 支持通过参数动态指定点位名称
    auto node = getNode();
    std::string point_name = "pickup";
    node->declare_parameter("pickup_point_name", point_name);
    node->get_parameter("pickup_point_name", point_name);

    auto * cb = this->template getOrthogonalClientBehavior<OrNavigation, CbNavigateToNavPoint>();
    if (cb) {
      cb->setPointName(point_name);
    }
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StNavigateToPickup] navigating to pickup point via nav_point_manager");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "[StNavigateToPickup] exit");
  }
};

}  // namespace smacc2_nav_mission
