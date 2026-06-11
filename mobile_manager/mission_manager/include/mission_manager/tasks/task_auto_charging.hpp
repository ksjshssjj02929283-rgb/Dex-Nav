#ifndef MISSION_MANAGER__TASK_AUTO_CHARGING_HPP_
#define MISSION_MANAGER__TASK_AUTO_CHARGING_HPP_

#include <memory>
#include <vector>

// #include "mission_manager/task.hpp"
// #include "mission_manager/mission_manager.hpp"

#include "mission_manager/mission_manager.hpp"
#include "mission_manager/task_interface.hpp"

namespace pnc {

// Task Interface
class TaskAutoCharging : public TaskInterface {
    enum class AutoChargingTaskStage {
        FAILED = -1,                 //!< Task failed
        INIT = 0,                    //!< Task initialized
        GOAL_SENDING = 1,            //!< Task is sending goal to the charging point
        NAVIGATING = 2,              //!< Task is navigating to the charging point
        CHARGING_POINT_REACHED = 3,  //!< Task is at the charging point
        DOCKING = 4,                 //!< Task is docking charging station
        FINISHED = 5,                //!< Task finished
        CANCELED = 6                 //!< Task canceled
    };

public:
    TaskAutoCharging();
    virtual ~TaskAutoCharging() = default;

    bool Init(const std::string& task_name) override;
    TaskStatus Execute() override;

    TaskStatus Pause() override;
    TaskStatus Resume() override;
    TaskStatus Restart() override;
    void ResetTask() override;

private:
    AutoChargingTaskStage curr_task_stage_;
};

}  // namespace pnc

#endif  // MISSION_MANAGER__TASK_AUTO_CHARGING_HPP_
