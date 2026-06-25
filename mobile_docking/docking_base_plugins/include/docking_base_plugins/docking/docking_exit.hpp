// 出桩任务（ROS1 DockingExit 移植，DEC-011 Part2）：前进离桩单段
#ifndef DOCKING_BASE_PLUGINS__DOCKING_EXIT_HPP_
#define DOCKING_BASE_PLUGINS__DOCKING_EXIT_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "docking_base_core/docking_interface.hpp"

namespace docking_base_plugins
{

enum class ExitStage { FAILED = -1, INIT = 0, TO_EXIT_POINT = 1, SUCCESS = 2 };

class DockingExit : public docking_base_core::DockingInterface
{
public:
  DockingExit() = default;
  bool Init() override;
  docking_base_core::DockingStatus UpdateTaskStage() override;

protected:
  void GenerateCheckPoint() override;
  ExitStage curr_stage_{ExitStage::INIT};
  geometry_msgs::msg::PoseStamped exit_point_;
  double exit_distance_{0.4};
  double exit_velocity_{0.1};   // 出桩逼近速度（参数 exit_velocity）
};

}  // namespace docking_base_plugins

#endif  // DOCKING_BASE_PLUGINS__DOCKING_EXIT_HPP_
