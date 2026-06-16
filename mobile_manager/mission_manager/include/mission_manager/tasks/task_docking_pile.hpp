#ifndef MISSION_MANAGER__TASK_DOCKING_PILE_HPP_
#define MISSION_MANAGER__TASK_DOCKING_PILE_HPP_

#include <memory>
#include <vector>

#include "mission_manager/mission_manager.hpp"
#include "mission_manager/nav_point.hpp"
#include "mission_manager/task_interface.hpp"

namespace pnc {

// Task Interface
class TaskDockingPile : public TaskInterface {
    enum class DockingPileTaskStage {
        FAILED = -1,         //!< Task failed
        INIT = 0,            //!< Task initialized
        NAV_TO_STATION = 1,  // !< Task is navigating to charging station
        DOCKING = 2,         //!< Task is docking charging station
        FINISHED = 3,        //!< Task finished
        CANCELED = 4         //!< Task canceled
    };

public:
    TaskDockingPile();
    virtual ~TaskDockingPile() = default;

    bool Init(const std::string& task_name) override;
    TaskStatus Execute() override;

    TaskStatus Pause() override;
    TaskStatus Resume() override;
    TaskStatus Restart() override;
    void ResetTask() override;

private:
    bool IsAtStation();
    bool IsNearStation();

private:
    DockingPileTaskStage curr_task_stage_;
    std::shared_ptr<NavPoint> charge_station_ = nullptr;
};

}  // namespace pnc

#endif  // MISSION_MANAGER__TASK_DOCKING_PILE_HPP_
