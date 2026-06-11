#ifndef MISSION_MANAGER__MISSION_MANAGER_PARAM_HPP_
#define MISSION_MANAGER__MISSION_MANAGER_PARAM_HPP_

#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <vector>

namespace pnc {

typedef struct {
    double auto_charge_threshold = 30.0;
    double force_charge_threshold = 20.0;
    bool respawn_task = false;
} MissionManagerParam;

inline std::ostream& operator<<(std::ostream& s, const MissionManagerParam& param) {
    s << "mission manager param: \n";
    s << "auto_charge_threshold: " << param.auto_charge_threshold << std::endl;
    s << "force_charge_threshold: " << param.force_charge_threshold << std::endl;
    s << "respawn_task: " << param.respawn_task << std::endl;
    return s;
}

}  // namespace pnc

#endif  // MISSION_MANAGER__MISSION_MANAGER_PARAM_HPP_