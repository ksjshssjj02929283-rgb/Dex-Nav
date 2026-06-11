#ifndef MISSION_MANAGER__TASK_INTERFACE_HPP_
#define MISSION_MANAGER__TASK_INTERFACE_HPP_

#include <iostream>
#include <memory>
#include <mutex>
#include <vector>

#include "mission_manager/docking_info.hpp"
#include "mission_manager/mission_info.hpp"
#include "mission_manager/nav_point.hpp"
#include "mission_manager/nav_util_adapter.hpp"
#include "mission_manager/robot_status.hpp"

namespace pnc {

enum class TaskPriority { NORMAL = 0, HIGH = 1, EMERGENCY = 2 };

enum class TaskType { UNKNOWN = -1, NAVIGATION = 0, DOCKING_PILE = 1, EXIT_PILE = 2, AUTO_CHARGING = 3 };

enum class TaskStatus { EXECUTING = 1, PAUSING = 2, FINISHED = 3, CANCELED = 4, FAILED = 5 };

// 为 TaskType 定义操作符<<
inline std::ostream& operator<<(std::ostream& os, const TaskType& type) {
    switch (type) {
        case TaskType::UNKNOWN:
            os << "TaskType::UNKNOWN";
            break;
        case TaskType::NAVIGATION:
            os << "TaskType::NAVIGATION";
            break;
        case TaskType::DOCKING_PILE:
            os << "TaskType::DOCKING_PILE";
            break;
        case TaskType::EXIT_PILE:
            os << "TaskType::EXIT_PILE";
            break;
        case TaskType::AUTO_CHARGING:
            os << "TaskType::AUTO_CHARGING";
            break;
        default:
            os << "TaskType::UNKNOWN";
            break;
    }
    return os;
}

// 为 TaskStatus 定义操作符<<
inline std::ostream& operator<<(std::ostream& os, const TaskStatus& status) {
    switch (status) {
        case TaskStatus::EXECUTING:
            os << "TaskStatus::EXECUTING";
            break;
        case TaskStatus::PAUSING:
            os << "TaskStatus::PAUSING";
            break;
        case TaskStatus::FINISHED:
            os << "TaskStatus::FINISHED";
            break;
        case TaskStatus::CANCELED:
            os << "TaskStatus::CANCELED";
            break;
        case TaskStatus::FAILED:
            os << "TaskStatus::FAILED";
            break;
        default:
            os << "TaskStatus::UNKNOWN";
            break;
    }
    return os;
}

// Task Interface
class TaskInterface {
public:
    TaskInterface() {}
    virtual ~TaskInterface() = default;

    // 初始化task，名称和配置文件路径
    virtual bool Init(const std::string& task_name) = 0;
    // 执行task，并返回执行状态
    virtual TaskStatus Execute() = 0;

    // 暂停或恢复task执行
    virtual TaskStatus Pause() = 0;
    virtual TaskStatus Resume() = 0;
    virtual TaskStatus Restart() = 0;  // 重新开始执行
    virtual void ResetTask() = 0;

    virtual TaskStatus Cancel() {
        std::lock_guard<std::mutex> lock(status_mutex_);
        task_status_ = TaskStatus::CANCELED;
        return TaskStatus::CANCELED;
    }

    void SetMissionInfo(const std::shared_ptr<MissionInfo>& info) { mission_info_ = info; }

    // 设置经过点
    void SetPassingBy(const std::vector<std::shared_ptr<NavPoint>>& passing_by) { passing_by_ = passing_by; }

    void SetWaitingTime(const double waiting_time) { waiting_time_ = waiting_time; }

    const std::vector<std::shared_ptr<NavPoint>> GetPassingBy() const { return passing_by_; }

    void SetPriority(const TaskPriority priority) { priority_ = priority; }

    const std::shared_ptr<NavPoint> GetCurrentGoal() const { return curr_goal_; }

    const std::shared_ptr<DockingInfo> GetDockingInfo() const { return docking_info_; }

    const std::string name() const { return name_; }
    const TaskType task_type() const { return task_type_; }
    const TaskStatus task_status() const { return task_status_; }
    const double progress() const { return task_progress_; }

    const TaskPriority priority() const { return priority_; }

    const double waiting_time() const { return waiting_time_; }

protected:
    std::string name_;        // 当前任务的名称
    TaskType task_type_;      // 当前任务的类型
    TaskStatus task_status_;  // 当前任务的状态
    TaskPriority priority_ = TaskPriority::NORMAL;
    uint32_t curr_goal_idx_ = 0;
    std::shared_ptr<NavPoint> curr_goal_;
    // 一个task可能经过多个nav_point
    std::vector<std::shared_ptr<NavPoint>> passing_by_;
    std::shared_ptr<DockingInfo> docking_info_;
    // 需要信息与mission manager进行交互
    std::shared_ptr<MissionInfo> mission_info_;

    double task_progress_ = 0.0;  // 任务的进度

    bool is_paussing_ = false;

    std::mutex status_mutex_;

    double waiting_time_ = 0.0;  // 完成任务后的等待时间
    std::shared_ptr<nav_util::TicToc> waiting_timer_;
};

}  // namespace pnc

#endif  // MISSION_MANAGER__TASK_INTERFACE_HPP_
