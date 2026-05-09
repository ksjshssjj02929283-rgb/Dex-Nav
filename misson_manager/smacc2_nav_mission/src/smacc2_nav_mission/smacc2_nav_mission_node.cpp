#include <smacc2_nav_mission/sm_nav_mission.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  smacc2::run<smacc2_nav_mission::SmNavMission>();
  return 0;
}
