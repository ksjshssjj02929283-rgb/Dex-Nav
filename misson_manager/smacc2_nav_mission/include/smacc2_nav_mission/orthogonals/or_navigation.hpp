#pragma once

#include <cl_nav2z/cl_nav2z.hpp>
#include <cl_nav2z/components/amcl/cp_amcl.hpp>
#include <cl_nav2z/components/goal_checker_switcher/cp_goal_checker_switcher.hpp>
#include <cl_nav2z/components/odom_tracker/cp_odom_tracker.hpp>
#include <cl_nav2z/components/planner_switcher/cp_planner_switcher.hpp>
#include <cl_nav2z/components/pose/cp_pose.hpp>
#include <smacc2/smacc_orthogonal.hpp>

namespace smacc2_nav_mission
{

class OrNavigation : public smacc2::Orthogonal<OrNavigation>
{
public:
  void onInitialize() override
  {
    auto client = this->createClient<cl_nav2z::ClNav2Z>();
    client->createComponent<cl_nav2z::CpPose>("base_link", "map");
    client->createComponent<cl_nav2z::odom_tracker::CpOdomTracker>();
    client->createComponent<cl_nav2z::CpPlannerSwitcher>();
    client->createComponent<cl_nav2z::CpGoalCheckerSwitcher>();
    client->createComponent<cl_nav2z::CpAmcl>();
  }
};

}  // namespace smacc2_nav_mission
