#pragma once

#include <smacc2/smacc.hpp>

namespace smacc2_nav_mission
{

using namespace cl_nav2z;
using namespace smacc2::default_transition_tags;

struct StNavigateToDropoff : smacc2::SmaccState<StNavigateToDropoff, SmNavMission>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvActionSucceeded<ClNav2Z, OrNavigation>, StMissionComplete, SUCCESS>,
    Transition<EvActionAborted<ClNav2Z, OrNavigation>,  StMissionComplete, ABORT>
  > reactions;

  static void staticConfigure()
  {
    // 点位名称来自 nav_point_manager 的 points.json，可通过参数 "dropoff_point_name" 覆盖
    configure_orthogonal<OrNavigation, CbNavigateToNavPoint>("dropoff");
  }

  void runtimeConfigure()
  {
    auto node = getNode();
    std::string point_name = "dropoff";
    node->declare_parameter("dropoff_point_name", point_name);
    node->get_parameter("dropoff_point_name", point_name);

    auto * cb = this->template getOrthogonalClientBehavior<OrNavigation, CbNavigateToNavPoint>();
    if (cb) {
      cb->setPointName(point_name);
    }
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StNavigateToDropoff] navigating to dropoff point via nav_point_manager");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "[StNavigateToDropoff] exit");
  }
};

}  // namespace smacc2_nav_mission
