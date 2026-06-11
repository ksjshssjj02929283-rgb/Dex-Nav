#ifndef MISSION_MANAGER__TASK_EXIT_HPP_
#define MISSION_MANAGER__TASK_EXIT_HPP_

#include <memory>
#include <vector>

// #include "mission_manager/task.hpp"
// #include "mission_manager/mission_manager.hpp"

#include "mission_manager/mission_manager.hpp"
#include "mission_manager/task_interface.hpp"

namespace pnc {

// Task Interface
class TaskExitPile : public TaskInterface {
    // TODO: 考虑在从充电状态中跳转到导航状态，需要先导航到最近的充电点，然后再进行导航

    enum class ExitPileTaskStage {
        FAILED = -1,              //!< Task failed
        INIT = 0,                 //!< Task initialized
        EXIT_CHARGE_STATION = 1,  //!< Task is exiting charge station
        FINISHED = 2,             //!< Task finished
        CANCELED = 3              //!< Task canceled
    };

public:
    TaskExitPile();
    virtual ~TaskExitPile() = default;

    bool Init(const std::string& task_name) override;
    TaskStatus Execute() override;

    TaskStatus Pause() override;
    TaskStatus Resume() override;
    TaskStatus Restart() override;
    void ResetTask() override;

private:
    ExitPileTaskStage curr_task_stage_;
};

}  // namespace pnc

#endif  // MISSION_MANAGER__TASK_EXIT_HPP_
