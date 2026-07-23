#ifndef MISSION_MANAGER_HPP_
#define MISSION_MANAGER_HPP_

#include <glog/logging.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "mission_manager/docking_info.hpp"
#include "mission_manager/mission_info.hpp"
#include "mission_manager/mission_manager_param.hpp"
#include "mission_manager/nav_util_adapter.hpp"
#include "mission_manager/robot_status.hpp"
#include "mission_manager/task_group.hpp"
#include "mission_manager/task_interface.hpp"

namespace pnc {

// // 当前执行的任务状态
// enum class MissionStatus
// {
//   ERROR = -1,
//   IDLE = 0,
//   NAVIGATING = 1,        // 导航中
//   DOCKING_EXIT = 2,      // 出桩中
//   DOCKING_CHARGING = 3,  // 充电对接任务中
// };

// 任务管理器的状态
enum class ManagerStatus { FAILED = -1, IDLE = 0, EXECUTING = 1, PAUSING = 2, WAITING = 3, FINISHED = 4 };

inline std::ostream& operator<<(std::ostream& os, const ManagerStatus& status) {
    switch (status) {
        case ManagerStatus::FAILED:
            os << "ManagerStatus::FAILED";
            break;
        case ManagerStatus::IDLE:
            os << "ManagerStatus::IDLE";
            break;
        case ManagerStatus::EXECUTING:
            os << "ManagerStatus::EXECUTING";
            break;
        case ManagerStatus::PAUSING:
            os << "ManagerStatus::PAUSING";
            break;
        case ManagerStatus::WAITING:
            os << "ManagerStatus::WAITING";
            break;
        case ManagerStatus::FINISHED:
            os << "ManagerStatus::FINISHED";
            break;
        default:
            os << "ManagerStatus::UNKNOWN";
    }
    return os;
}

class MissionManager {
public:
    // explicit MissionManager(const std::string& file_path, const bool load_from_file = true);
    MissionManager(const MissionManagerParam& param);
    ~MissionManager() = default;

    // 从nav_point_manager请求nav_points
    bool SetNavPoints(
        const std::vector<std::shared_ptr<NavPoint>>& nav_points);  // 每次任务前都要请求一次，防止任务点发生变化

    void UpdateRobotStatus(const std::shared_ptr<RobotStatus>& status);

    void Run();

    // --------------------------------------------------------
    // 可以增加一个task，也可以增加多个task
    bool AddTasks(const std::vector<std::shared_ptr<TaskInterface>>& task_in);

    bool AddTaskGroup(const std::shared_ptr<TaskGroup>& task_group);

    const std::shared_ptr<TaskInterface> GetCurrTask() const { return current_task_; }

    const std::shared_ptr<TaskGroup> GetCurrTaskGroup() const { return task_group_deque_.front(); }

    const uint32_t GetCurrGroupExecutionLeft();

    const std::vector<std::shared_ptr<TaskInterface>> GetTaskList();

    bool RemoveTask(const std::string& task_name);

    bool ClearAllTasks();  // 清空任务队列

    bool PauseCurrentTask();  // 暂停任务

    bool ResumeCurrentTask();  // 恢复任务

    bool CancelCurrentTask();  // 取消任务

    bool CancelCurrentTaskGroup();  // 取消当前任务组

    void EnableRespawnTask(const bool enable) { param_.respawn_task = enable; }
    bool IsRespawnTask() const { return param_.respawn_task; }
    const MissionManagerParam& GetParam() const { return param_; }

    // --------------------------------------------------------
    bool IsNeedNavigation() const { return mission_info_->is_need_navigation(); }
    bool IsNeedDocking() const { return mission_info_->is_need_docking(); }
    bool IsNeedPerception() const { return mission_info_->is_need_perception(); }
    bool IsNeedCancelNavigation() const { return mission_info_->is_need_cancel_navigation(); }
    bool IsNeedCancelDocking() const { return mission_info_->is_need_cancel_docking(); }
    bool IsNeedCancelPerception() const { return mission_info_->is_need_cancel_perception(); }

    bool is_navigating() const { return mission_info_->is_navigating(); }
    bool is_docking() const { return mission_info_->is_docking(); }

    void SetNavigating() { mission_info_->SetNavigating(); }
    void SetDocking() { mission_info_->SetDocking(); }

    void ResetMissionStatus() { mission_info_->ResetStatus(); }

    void SetNavigationSuccess(const bool is_success) { mission_info_->SetNavigationSuccess(is_success); }

    void SetDockingSuccess(const bool is_success) { mission_info_->SetDockingSuccess(is_success); }

    void SetPerceptionFinished() { mission_info_->SetPerceptionFinished(); }

    const std::shared_ptr<NavPoint> GetCurrentGoal() const;
    const std::shared_ptr<DockingInfo> GetDockingInfo() const;

    const ManagerStatus& GetManagerStatus();

private:
    void UpdateManagerStatus(const TaskStatus& task_status);

    // 将TaskGroup中的任务加入到任务队列中
    void MaybeLoadTaskFromGroup();
    void MaybeLoadCurrentTask();
    void MaybeAddAutoChargingTask();
    void MaybePauseCurrentTask();
    void MaybeRespawnCurrentTask(TaskStatus& task_status);
    void MaybeWaiting();
    void MaybeFinishCurrentTask();  // 完成任务
    // 根据条件，增加充电任务
    void AddAutoChargingTask();

    // 在充电流程中使用PublishNavPoint，发布导航点，具体的操作使用任务流控制
    // 检查是否需要自动充电
    bool IsNeedAutoCharging();

    std::shared_ptr<NavPoint> GetNearestNavPoint(const Eigen::Vector3d& pose, bool is_charging_point = false);

private:
    std::string file_path_;  // NavPoint文件的保存路径

    std::unordered_map<std::string, std::shared_ptr<NavPoint>> nav_point_umap_;

    // 任务队列
    std::deque<std::shared_ptr<TaskGroup>> task_group_deque_;
    std::deque<std::shared_ptr<TaskInterface>> valid_task_deque_;  // 当前的任务队列
    std::shared_ptr<TaskInterface> current_task_;

    std::shared_ptr<MissionInfo> mission_info_;

    ManagerStatus manager_status_ = ManagerStatus::IDLE;
    TaskStatus task_status_;

    MissionManagerParam param_;

    std::recursive_mutex status_mutex_;
    std::recursive_mutex task_mutex_;

    std::shared_ptr<nav_util::TicToc> waiting_tic_;
};

}  // namespace pnc

#endif  // MISSION_MANAGER_HPP_
