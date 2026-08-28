// Copyright (c) 2026 kuawei
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_BEHAVIOR_TREE__GLOBAL_REPLAN_CONTEXT_HPP_
#define NAV2_BEHAVIOR_TREE__GLOBAL_REPLAN_CONTEXT_HPP_

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "behaviortree_cpp_v3/blackboard.h"

namespace nav2_behavior_tree
{

inline constexpr char kGlobalReplanContextKey[] = "global_replan_context";

enum class GlobalReplanMode { DISABLED, OBSERVE, ACTIVE };

enum class GlobalReplanReason { FOLLOW_PATH_FAILED, BACK_UP_SUCCEEDED };

enum class GlobalReplanRequestState { PENDING, COMPLETED, CANCELED };

struct GlobalReplanRequestStatus
{
  std::atomic<GlobalReplanRequestState> state{GlobalReplanRequestState::PENDING};
};

class GlobalReplanToken
{
public:
  GlobalReplanToken() = default;
  GlobalReplanToken(const GlobalReplanToken &) = default;
  GlobalReplanToken & operator=(const GlobalReplanToken &) = default;

  uint64_t request_id{0};

private:
  GlobalReplanToken(
    uint64_t id,
    const std::shared_ptr<GlobalReplanRequestStatus> & status)
  : request_id(id), status_(status)
  {
  }

  std::shared_ptr<GlobalReplanRequestStatus> status_;

  friend class GlobalReplanContext;
};

struct GlobalReplanRequest
{
  GlobalReplanToken token;
  GlobalReplanReason reason{GlobalReplanReason::FOLLOW_PATH_FAILED};
};

inline bool parseGlobalReplanMode(const std::string & value, GlobalReplanMode & mode)
{
  if (value == "disabled") {
    mode = GlobalReplanMode::DISABLED;
    return true;
  }
  if (value == "observe") {
    mode = GlobalReplanMode::OBSERVE;
    return true;
  }
  if (value == "active") {
    mode = GlobalReplanMode::ACTIVE;
    return true;
  }
  return false;
}

inline bool parseGlobalReplanReason(const std::string & value, GlobalReplanReason & reason)
{
  if (value == "follow_path_failed") {
    reason = GlobalReplanReason::FOLLOW_PATH_FAILED;
    return true;
  }
  if (value == "back_up_succeeded") {
    reason = GlobalReplanReason::BACK_UP_SUCCEEDED;
    return true;
  }
  return false;
}

inline const char * toString(GlobalReplanMode mode)
{
  switch (mode) {
    case GlobalReplanMode::DISABLED:
      return "disabled";
    case GlobalReplanMode::OBSERVE:
      return "observe";
    case GlobalReplanMode::ACTIVE:
      return "active";
  }
  return "invalid";
}

inline const char * toString(GlobalReplanReason reason)
{
  switch (reason) {
    case GlobalReplanReason::FOLLOW_PATH_FAILED:
      return "follow_path_failed";
    case GlobalReplanReason::BACK_UP_SUCCEEDED:
      return "back_up_succeeded";
  }
  return "invalid";
}

inline const char * toString(GlobalReplanRequestState state)
{
  switch (state) {
    case GlobalReplanRequestState::PENDING:
      return "pending";
    case GlobalReplanRequestState::COMPLETED:
      return "completed";
    case GlobalReplanRequestState::CANCELED:
      return "canceled";
  }
  return "invalid";
}

/**
 * @brief 行为树内的全局重规划协调状态
 *
 * 单调请求编号用于区分规划开始前后的恢复事件，防止运行中的旧规划错误确认新请求。
 * 多个规划前已存在的请求可以由同一次有效规划统一确认。
 * 上下文只保留待处理请求；完成或取消状态随令牌保存，避免任务运行越久历史表越大。
 * 已取消请求保留独立状态，之后的规划完成也不能把它改判为完成。
 * 该对象只在当前行为树黑板内共享，不跨节点进程，也不替代 Planner Action 接口。
 */
class GlobalReplanContext
{
public:
  void setMode(GlobalReplanMode mode)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    mode_ = mode;
  }

  GlobalReplanMode mode() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return mode_;
  }

  GlobalReplanToken request(GlobalReplanReason reason)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++latest_request_id_;
    auto status = std::make_shared<GlobalReplanRequestStatus>();
    pending_requests_.emplace(latest_request_id_, RequestRecord{reason, status});
    return GlobalReplanToken(latest_request_id_, status);
  }

  std::optional<GlobalReplanRequest> latestPendingRequest() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_requests_.empty()) {
      return std::nullopt;
    }

    const auto & request = *pending_requests_.crbegin();
    return GlobalReplanRequest{
      GlobalReplanToken(request.first, request.second.status), request.second.reason};
  }

  bool isCompleted(const GlobalReplanToken & token) const
  {
    return hasState(token, GlobalReplanRequestState::COMPLETED);
  }

  bool isCanceled(const GlobalReplanToken & token) const
  {
    return hasState(token, GlobalReplanRequestState::CANCELED);
  }

  bool complete(const GlobalReplanToken & token)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto request = pending_requests_.find(token.request_id);
    if (request == pending_requests_.end() ||
      request->second.status != token.status_)
    {
      return false;
    }

    // 本次规划开始前积累的待处理请求都由同一条新路径满足。
    const auto completed_end = pending_requests_.upper_bound(token.request_id);
    for (auto pending_request = pending_requests_.begin();
      pending_request != completed_end; ++pending_request)
    {
      pending_request->second.status->state.store(GlobalReplanRequestState::COMPLETED);
    }
    pending_requests_.erase(pending_requests_.begin(), completed_end);
    return true;
  }

  bool cancel(const GlobalReplanToken & token)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto request = pending_requests_.find(token.request_id);
    if (request == pending_requests_.end() ||
      request->second.status != token.status_)
    {
      return false;
    }
    request->second.status->state.store(GlobalReplanRequestState::CANCELED);
    pending_requests_.erase(request);
    return true;
  }

private:
  struct RequestRecord
  {
    GlobalReplanReason reason;
    std::shared_ptr<GlobalReplanRequestStatus> status;
  };

  static bool hasState(
    const GlobalReplanToken & token,
    GlobalReplanRequestState expected_state)
  {
    return token.request_id != 0 && token.status_ &&
           token.status_->state.load() == expected_state;
  }

  mutable std::mutex mutex_;
  GlobalReplanMode mode_{GlobalReplanMode::DISABLED};
  uint64_t latest_request_id_{0};
  std::map<uint64_t, RequestRecord> pending_requests_;
};

inline std::shared_ptr<GlobalReplanContext> getOrCreateGlobalReplanContext(
  const BT::Blackboard::Ptr & blackboard)
{
  std::shared_ptr<GlobalReplanContext> context;
  if (!blackboard->get(kGlobalReplanContextKey, context) || !context) {
    context = std::make_shared<GlobalReplanContext>();
    blackboard->set(kGlobalReplanContextKey, context);
  }
  return context;
}

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__GLOBAL_REPLAN_CONTEXT_HPP_
