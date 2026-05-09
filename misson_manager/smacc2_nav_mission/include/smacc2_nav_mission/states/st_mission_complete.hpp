#pragma once

#include <smacc2/smacc.hpp>

namespace smacc2_nav_mission
{

// 终止状态——成功
struct StMissionSuccess : smacc2::SmaccState<StMissionSuccess, SmNavMission>
{
  using SmaccState::SmaccState;
  typedef mpl::list<> reactions;
  static void staticConfigure() {}
  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StMissionSuccess] mission completed successfully");
  }
};

// 终止状态——失败/中断
struct StMissionComplete : smacc2::SmaccState<StMissionComplete, SmNavMission>
{
  using SmaccState::SmaccState;
  typedef mpl::list<> reactions;
  static void staticConfigure() {}
  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StMissionComplete] mission aborted or failed");
  }
};

}  // namespace smacc2_nav_mission
