// Copyright 2026 DEXE
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_CONTROLLER__SNAPSHOT_VALIDATION_HPP_
#define NAV2_CONTROLLER__SNAPSHOT_VALIDATION_HPP_
#include <chrono>
#include <cmath>
#include <cstdint>
namespace nav2_controller
{
namespace snapshot
{
inline bool stallTimedOut(
  const std::chrono::steady_clock::time_point & started,
  const std::chrono::steady_clock::time_point & now,
  double timeout_s)
{
  if (!std::isfinite(timeout_s) || timeout_s <= 0.0) {
    return false;
  }
  const double elapsed_s = std::chrono::duration<double>(now - started).count();
  return std::isfinite(elapsed_s) && elapsed_s >= timeout_s;
}

inline bool usableResult(
  uint64_t session, uint64_t revision, uint64_t expected_session,
  uint64_t expected_revision, bool success, double age, double max_age)
{
  // 必须处于相同任务会话 session
  if (session != expected_session || !success || !std::isfinite(age) || age > max_age) {
    return false;
  }
  // 允许在同一 session 下新版本 revision 计算完成前，平滑复用旧版本 revision（revision <= expected_revision），
  // 避免同任务局部重规划计算耗时引发瞬间急刹。
  return revision <= expected_revision;
}

template<typename ResultT>
inline bool usableResult(
  const ResultT & result,
  uint64_t expected_session,
  uint64_t expected_revision,
  double max_age,
  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now())
{
  const double age = std::chrono::duration<double>(now - result.completed).count();
  return usableResult(
    result.session, result.revision, expected_session,
    expected_revision, result.success, age, max_age);
}
}  // namespace snapshot
}  // namespace nav2_controller
#endif
