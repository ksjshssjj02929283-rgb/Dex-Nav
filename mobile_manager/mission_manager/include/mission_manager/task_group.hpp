#ifndef MISSION_MANAGER__TASK_GROUP_HPP_
#define MISSION_MANAGER__TASK_GROUP_HPP_

#include <iostream>
#include <memory>
#include <mutex>
#include <vector>

#include "mission_manager/nav_util_adapter.hpp"
#include "mission_manager/task_interface.hpp"

namespace pnc {

class TaskGroup {
public:
    TaskGroup(const std::string& name) : name_(name) {}

    ~TaskGroup() = default;
    void AddTask(std::shared_ptr<TaskInterface> task) { task_list_.push_back(task); }

    void StartWaiting() { wait_tic_.reset(new nav_util::TicToc()); }

    bool CanLoadNextCycle() {
        if (wait_tic_ == nullptr) {
            return true;
        }

        if (wait_tic_->toc() * 1e-3 > interval_) {
            return true;
        }
        return false;
    }

    bool IsFinished() const { return (execute_count_ >= max_execute_count_); }

    void SetMaxExecuteCount(const uint32_t count) {
        max_execute_count_ = count;
        if (max_execute_count_ == 0) {
            max_execute_count_ = 1;  // 任务至少执行一次
        }
    }

    void SetInverval(const double interval) { interval_ = interval; }

    void SetGroupName(const std::string& name) { name_ = name; }

    void IncreaseExecuteCount() {
        execute_count_++;
        std::cout << "execute_count_: " << execute_count_ << std::endl;
        std::cout << "max_execute_count_: " << max_execute_count_ << std::endl;
    }

    const std::string name() const { return name_; }

    const std::vector<std::shared_ptr<TaskInterface>>& GetTaskList() const { return task_list_; }

    const uint32_t max_execute_count() const { return max_execute_count_; }

    const uint32_t execute_count() const { return execute_count_; }

    const double interval() const { return interval_; }

private:
    std::string name_;
    std::vector<std::shared_ptr<TaskInterface>> task_list_;
    uint32_t execute_count_ = 0;
    uint32_t max_execute_count_ = 1;  // 默认这个 task group 执行一次
    double interval_ = 0.0;           // in seconds

    std::shared_ptr<nav_util::TicToc> wait_tic_;
};

}  // namespace pnc

#endif  // MISSION_MANAGER__TASK_GROUP_HPP_
