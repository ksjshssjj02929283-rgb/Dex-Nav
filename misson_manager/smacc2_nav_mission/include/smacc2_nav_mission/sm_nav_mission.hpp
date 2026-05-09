#pragma once

#include <smacc2/smacc.hpp>

#include <cl_nav2z/cl_nav2z.hpp>
#include <cl_nav2z/client_behaviors/cb_wait_nav2_nodes.hpp>

#include "client_behaviors/cb_navigate_to_nav_point.hpp"
#include "orthogonals/or_navigation.hpp"

namespace smacc2_nav_mission
{

class StWaitNav2;
class StNavigateToPickup;
class StNavigateToDropoff;
class StMissionComplete;
class StMissionSuccess;

struct SmNavMission : public smacc2::SmaccStateMachineBase<SmNavMission, StWaitNav2>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrNavigation>();
  }
};

}  // namespace smacc2_nav_mission

#include "states/st_wait_nav2.hpp"
#include "states/st_navigate_to_pickup.hpp"
#include "states/st_navigate_to_dropoff.hpp"
#include "states/st_mission_complete.hpp"
