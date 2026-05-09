#pragma once

#include <smacc2/smacc.hpp>

namespace smacc2_nav_mission
{

using namespace cl_nav2z;
using namespace smacc2::default_transition_tags;

struct StWaitNav2 : smacc2::SmaccState<StWaitNav2, SmNavMission>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbWaitNav2Nodes, OrNavigation>, StNavigateToPickup, SUCCESS>,
    Transition<EvCbFailure<CbWaitNav2Nodes, OrNavigation>, StMissionComplete, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbWaitNav2Nodes>();
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StWaitNav2] waiting nav2 servers ready");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "[StWaitNav2] exit");
  }
};

}  // namespace smacc2_nav_mission
