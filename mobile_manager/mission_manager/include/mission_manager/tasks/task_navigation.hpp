#ifndef MISSION_MANAGER__TASK_NAVIGATION_HPP_
#define MISSION_MANAGER__TASK_NAVIGATION_HPP_

#include <memory>
#include <vector>

// #include "mission_manager/task.hpp"
// #include "mission_manager/mission_manager.hpp"

#include "mission_manager/mission_manager.hpp"
#include "mission_manager/task_interface.hpp"

namespace pnc {

// Task Interface
class TaskNavigation : public TaskInterface {
    // TODO: 考虑在从充电状态中跳转到导航状态，需要先导航到最近的充电点，然后再进行导航

    enum class NavigationTaskStage {
        FAILED = -1,              //!< Task failed
        INIT = 0,                 //!< Task initialized
        EXIT_CHARGE_STATION = 1,  //!< Task is exiting charge station
        GOAL_SENDING = 2,         //!< Task is sending goal to the charging point
        NAVIGATING = 3,           //!< Task is navigating to the charging point
        GOAL_REACHED = 4,         //!< Task is at the charging point
        FINISHED = 5,             //!< Task finished
        CANCELED = 6              //!< Task canceled
    };

public:
    TaskNavigation();
    virtual ~TaskNavigation() = default;

    bool Init(const std::string& task_name) override;
    TaskStatus Execute() override;

    TaskStatus Pause() override;
    TaskStatus Resume() override;
    TaskStatus Restart() override;
    void ResetTask() override;

private:
    NavigationTaskStage curr_task_stage_;
};

}  // namespace pnc

#endif  // MISSION_MANAGER__TASK_NAVIGATION_HPP_
