#ifndef MISSION_MANAGER__ROBOT_STATUS_HPP_
#define MISSION_MANAGER__ROBOT_STATUS_HPP_

#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <vector>

namespace pnc {
// 当前机器人的状态
//   当前的位姿
//   当前的速度
//   当前的电量
//   是否急停
//   是否是自动模式

typedef struct {
    /* data */
    Eigen::Vector3d pose;         // 位姿
    Eigen::Vector3d velocity;     // 速度
    double soc;                   // 电量
    bool emergency_stop = false;  // 是否急停
    bool auto_mode = true;        // 是否是自动模式
    bool is_charging = false;     // 是否在充电
} RobotStatus;

inline std::ostream& operator<<(std::ostream& s, const RobotStatus& status) {
    s << "robot status: \n";
    s << "pose: " << status.pose.transpose() << std::endl;
    s << "velocity: " << status.velocity.transpose() << std::endl;
    s << "soc: " << status.soc << std::endl;
    s << "emergency_stop: " << status.emergency_stop << std::endl;
    s << "is_charging: " << status.is_charging << std::endl;
    return s;
}

}  // namespace pnc

#endif  // MISSION_MANAGER__ROBOT_STATUS_HPP_