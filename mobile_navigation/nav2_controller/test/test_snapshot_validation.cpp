// Copyright 2026 DEXE
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "nav2_controller/snapshot_validation.hpp"
using nav2_controller::snapshot::usableResult;
using nav2_controller::snapshot::stallTimedOut;
TEST(Snapshot, AcceptsStillValidPreviousPlanWhileNextPlanRuns)
{
  EXPECT_TRUE(usableResult(2, 3, 2, 3, true, .2, .3));
  // 同一 session 下，新 revision 规划中（expected_revision=4），旧 revision=3 在时效内仍可用
  EXPECT_TRUE(usableResult(2, 3, 2, 4, true, .2, .3));
}
TEST(Snapshot, RejectsLatePreviousSessionAndFutureRevision)
{
  EXPECT_FALSE(usableResult(1, 3, 2, 3, true, .01, .3));
  // 迟到的超前版本（或不匹配会话）拒绝
  EXPECT_FALSE(usableResult(2, 4, 2, 3, true, .01, .3));
}
TEST(Snapshot, FailureAndExpiryStopImmediately)
{
  EXPECT_FALSE(usableResult(2, 3, 2, 3, false, .01, .3));
  EXPECT_FALSE(usableResult(2, 3, 2, 3, true, .31, .3));
}

TEST(Snapshot, LocalControlStallTimeout)
{
  const auto started = std::chrono::steady_clock::now();
  EXPECT_FALSE(stallTimedOut(started, started, 30.0));
  EXPECT_FALSE(stallTimedOut(started, started + std::chrono::seconds(29), 30.0));
  EXPECT_TRUE(stallTimedOut(started, started + std::chrono::seconds(30), 30.0));
  EXPECT_FALSE(stallTimedOut(started, started + std::chrono::seconds(31), 0.0));
}

struct DummyResult
{
  uint64_t session{0};
  uint64_t revision{0};
  bool success{false};
  std::chrono::steady_clock::time_point completed;
};

TEST(Snapshot, ObjectOverloadValidation)
{
  const auto now = std::chrono::steady_clock::now();
  DummyResult res;
  res.session = 10;
  res.revision = 5;
  res.success = true;
  res.completed = now - std::chrono::milliseconds(100);

  // 100ms within 300ms limit -> TRUE
  EXPECT_TRUE(usableResult(res, 10, 5, 0.3, now));

  // Expired -> FALSE
  EXPECT_FALSE(usableResult(res, 10, 5, 0.05, now));

  // Session mismatch -> FALSE
  EXPECT_FALSE(usableResult(res, 11, 5, 0.3, now));

  // Revision transition: res.revision=5, expected=6 is allowed if within age
  EXPECT_TRUE(usableResult(res, 10, 6, 0.3, now));
  // But future revision in result (expected=4 < res.revision=5) -> FALSE
  EXPECT_FALSE(usableResult(res, 10, 4, 0.3, now));

  // Unsuccessful -> FALSE
  res.success = false;
  EXPECT_FALSE(usableResult(res, 10, 5, 0.3, now));
}

TEST(Snapshot, TaskCancellationSessionIncrementRejectsActivePlan)
{
  const auto now = std::chrono::steady_clock::now();
  DummyResult active_plan;
  active_plan.session = 42;
  active_plan.revision = 1;
  active_plan.success = true;
  active_plan.completed = now - std::chrono::milliseconds(50);

  // 1. 任务正常运行中，活跃轨迹完全可用 (session=42, revision=1)
  EXPECT_TRUE(usableResult(active_plan, 42, 1, 0.5, now));

  // 2. 外部触发取消任务 (Action Cancel)：内部机制原子递增 session (42 -> 43)
  const uint64_t canceled_new_session = 43;

  // 3. 校验：递增 session 后，原任务的活跃轨迹被即刻判定为不可用 (拒绝执行与发速)
  EXPECT_FALSE(usableResult(active_plan, canceled_new_session, 1, 0.5, now));

  // 4. 即便后台迟到返回了属于被取消 session 的规划，也坚决拒绝
  EXPECT_FALSE(usableResult(42, 1, canceled_new_session, 1, true, 0.05, 0.5));
}

