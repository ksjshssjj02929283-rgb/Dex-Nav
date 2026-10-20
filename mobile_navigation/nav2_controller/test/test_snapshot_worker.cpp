// Copyright 2026 DEXE
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "nav2_controller/snapshot_runtime.hpp"
using namespace std::chrono_literals;
class FakeSpeed : public nav2_core::SpeedPlanner
{
public:
  std::atomic<int> calls{0};
  std::atomic<bool> block{false}, fail{false};
  std::mutex input_mutex;
  nav_msgs::msg::Path last_input;
  nav_msgs::msg::Path inputPath()
  {
    std::lock_guard<std::mutex> lock(input_mutex);
    return last_input;
  }
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &, std::string,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}
  void setPlan(const nav_msgs::msg::Path &) override {}
  void setSpeedLimit(const double &, const bool &) override {}
  nav_interfaces::msg::RobotTrajectory computeSpeedPlan(
    const nav_msgs::msg::Path & path,
    const nav2_core::InitialMotionState &) override
  {
    {std::lock_guard<std::mutex> lock(input_mutex); last_input = path;}
    ++calls;
    while (block.load()) {std::this_thread::sleep_for(1ms);}
    nav_interfaces::msg::RobotTrajectory t;
    if (!fail.load()) {
      t.header.frame_id = path.header.frame_id;
      nav_interfaces::msg::TrajectoryPoint p; p.pose = path.poses.front();
      t.points.push_back(p);
    }
    return t;
  }
  void requestCancel() override {block = false;}
};

// 模拟已经走过重复路段的裁剪器；setPlan会清空进度，与真实插件的契约一致。
class TailLocalPlanner : public nav2_core::LocalPlanner
{
public:
  std::atomic<int> set_plan_calls{0};
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &, std::string,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}
  void setPlan(const nav_msgs::msg::Path & path) override
  {
    plan_ = path;
    ++set_plan_calls;
  }
  bool computeLocalPlan(
    const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::Twist &,
    nav_msgs::msg::Path & local) override
  {
    local = plan_;
    // 首条路径已有三个点被消费；重设完整路径会丢失这段进度。
    if (set_plan_calls == 1 && local.poses.size() > 3) {
      local.poses.erase(local.poses.begin(), local.poses.begin() + 3);
    }
    return !local.poses.empty();
  }
private:
  nav_msgs::msg::Path plan_;
};

class ProbeServer : public nav2_controller::ControllerServer
{
public:
  std::shared_ptr<FakeSpeed> fake = std::make_shared<FakeSpeed>();
  void start()
  {
    configureSnapshotControl(); current_speed_planner_ = "fake";
    current_local_planner_.clear(); speed_planners_["fake"] = fake;
    startSnapshotWorker();
  }
  void stop() {stopSnapshotWorker();}
  void useLocalPlanner(const std::shared_ptr<TailLocalPlanner> & local)
  {
    current_local_planner_ = "tail";
    local_planners_["tail"] = local;
  }
  void request(
    uint64_t session, uint64_t revision,
    std::shared_ptr<const nav_msgs::msg::Path> path = nullptr,
    geometry_msgs::msg::PoseStamped locked_goal = {})
  {
    SnapshotRuntime::Request q;
    q.session = session; q.revision = revision;
    if (!path) {
      auto default_path = std::make_shared<nav_msgs::msg::Path>();
      default_path->header.frame_id = "map";
      geometry_msgs::msg::PoseStamped p; p.header.frame_id = "map";p.pose.orientation.w = 1;
      default_path->poses.push_back(p); path = default_path;
    }
    q.path = path;
    q.locked_position_goal = locked_goal;
    std::lock_guard<std::mutex> l(snapshot_runtime_->mutex);
    snapshot_runtime_->request = q; snapshot_runtime_->has_request = true;
    snapshot_runtime_->wake.notify_all();
  }
  void cancel()
  {
    std::lock_guard<std::mutex> l(snapshot_runtime_->mutex);
    snapshot_runtime_->has_request = false; snapshot_runtime_->result.reset();
    snapshot_runtime_->wake.notify_all();
  }
  void cancelSession()
  {
    {
      std::lock_guard<std::mutex> l(snapshot_runtime_->mutex);
      ++snapshot_runtime_->session;
      snapshot_runtime_->has_request = false;
      snapshot_runtime_->result.reset();
      snapshot_runtime_->wake.notify_all();
    }
    for (auto & planner : speed_planners_) {
      planner.second->requestCancel();
    }
  }
  uint64_t resultSession()
  {
    std::lock_guard<std::mutex> l(snapshot_runtime_->mutex);
    return snapshot_runtime_->result ? snapshot_runtime_->result->session : 0;
  }
  uint64_t resultRevision()
  {
    std::lock_guard<std::mutex> l(snapshot_runtime_->mutex);
    return snapshot_runtime_->result ? snapshot_runtime_->result->revision : 0;
  }
  bool success()
  {
    std::lock_guard<std::mutex> l(snapshot_runtime_->mutex);
    return snapshot_runtime_->result && snapshot_runtime_->result->success;
  }
  auto workerId() {return snapshot_runtime_->worker.get_id();}
};
template<class F> bool waitFor(F f)
{
  const auto end = std::chrono::steady_clock::now() + 3s;
  while (!f() && std::chrono::steady_clock::now() < end) {std::this_thread::sleep_for(5ms);}
  return f();
}
TEST(PersistentPlanner, IdleSlowPlanningCancelAndNextSession) {
  auto n = std::make_shared<ProbeServer>(); n->start();
  auto cleanup = std::shared_ptr<void>(nullptr, [n](void *) {n->stop();});
  auto id = n->workerId(); std::this_thread::sleep_for(120ms); EXPECT_EQ(n->fake->calls, 0);
  n->request(1, 1); ASSERT_TRUE(waitFor([&] {return n->success();}));
  n->fake->block = true; int calls = n->fake->calls;
  ASSERT_TRUE(waitFor([&] {return n->fake->calls > calls;}));
  EXPECT_TRUE(n->success()); // No clearing the previous result at planning start.
  const auto start = std::chrono::steady_clock::now();n->cancel();
  EXPECT_LT(std::chrono::steady_clock::now() - start, 50ms);
  EXPECT_EQ(n->resultSession(), 0u);
  n->request(2, 1); n->fake->block = false;
  ASSERT_TRUE(waitFor([&] {return n->resultSession() == 2;}));
  EXPECT_EQ(n->workerId(), id);
  n->fake->fail = true;
  ASSERT_TRUE(waitFor([&] {return !n->success();}));
  n->cancel(); std::this_thread::sleep_for(120ms);calls = n->fake->calls;
  std::this_thread::sleep_for(120ms);EXPECT_EQ(n->fake->calls, calls);
}
TEST(PersistentPlanner, LifecycleStopCancelsBlockedPlanner)
{
  auto n = std::make_shared<ProbeServer>();
  n->start();
  n->fake->block = true;
  n->request(1, 1);
  ASSERT_TRUE(waitFor([&] {return n->fake->calls > 0;}));
  const auto start = std::chrono::steady_clock::now();
  n->stop();
  EXPECT_LT(std::chrono::steady_clock::now() - start, 100ms);
}

TEST(PersistentPlanner, GoalCancellationInvalidatesActiveTrajectoryAndBlocksLateOutput)
{
  auto n = std::make_shared<ProbeServer>();
  n->start();
  auto cleanup = std::shared_ptr<void>(nullptr, [n](void *) {n->stop();});

  // 1. 正常发起规划请求 (session=1, revision=1) 并成功获取轨迹
  n->request(1, 1);
  ASSERT_TRUE(waitFor([&] {return n->success();}));
  EXPECT_EQ(n->resultSession(), 1u);

  // 2. 模拟长时间运行规划，在此期间外部触发任务取消 (finish 守卫触发 session 自增与状态清理)
  n->fake->block = true;
  n->request(1, 2);
  ASSERT_TRUE(waitFor([&] {return n->fake->calls > 1;}));

  // 执行取消操作：模拟 computeControlSnapshot 退出时的 RAII finish 清理
  n->cancelSession();

  // 验证现有快照已被彻底清空，旧 session 轨迹已被失效
  EXPECT_EQ(n->resultSession(), 0u);
  EXPECT_FALSE(n->success());

  // 3. 规划器恢复完成计算后，由于 session 已自增，迟到的旧规划结果绝不能被采纳
  n->fake->block = false;
  std::this_thread::sleep_for(50ms);

  // 验证此时前台仍然无有效结果 (迟到结果被 Worker 丢弃，没有赋给 result)
  EXPECT_NE(n->resultSession(), 1u);
}

TEST(PersistentPlanner, PositionLatchKeepsUnconsumedTailAcrossRevisionChanges)
{
  auto n = std::make_shared<ProbeServer>();
  auto local = std::make_shared<TailLocalPlanner>();
  n->start();
  auto cleanup = std::shared_ptr<void>(nullptr, [n](void *) {n->stop();});
  // 工作线程尚无请求，可以安全安装测试裁剪器。
  n->useLocalPlanner(local);
  auto path = std::make_shared<nav_msgs::msg::Path>();
  path->header.frame_id = "odom";
  for (double x : {0.0, 1.0, 0.0, 2.0, 3.0}) {
    geometry_msgs::msg::PoseStamped point;
    point.header.frame_id = "odom";
    point.pose.position.x = x;
    point.pose.orientation.w = 1.0;
    path->poses.push_back(point);
  }
  n->request(1, 1, path);
  ASSERT_TRUE(waitFor([&] {return n->resultRevision() == 1;}));
  ASSERT_EQ(n->fake->inputPath().poses.size(), 2u);
  EXPECT_EQ(local->set_plan_calls, 1);

  auto goal = path->poses.back();
  goal.pose.position.x = 3.01;
  n->request(1, 2, path, goal);
  ASSERT_TRUE(waitFor([&] {return n->resultRevision() == 2;}));
  auto tail = n->fake->inputPath();
  ASSERT_EQ(tail.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(tail.poses.front().pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(tail.poses.back().pose.position.x, 3.01);
  EXPECT_EQ(local->set_plan_calls, 2);

  // 限速等revision变化不能再次setPlan，或恢复已走过的重复路段。
  n->request(1, 3, path, goal);
  ASSERT_TRUE(waitFor([&] {return n->resultRevision() == 3;}));
  EXPECT_EQ(local->set_plan_calls, 2);
  EXPECT_EQ(n->fake->inputPath().poses.size(), 2u);

  // 新任务即使复用同一份路径指针，也必须丢弃上一任务的锁存尾段。
  n->request(2, 4, path);
  ASSERT_TRUE(waitFor([&] {return n->resultRevision() == 4;}));
  EXPECT_EQ(local->set_plan_calls, 3);
  tail = n->fake->inputPath();
  ASSERT_EQ(tail.poses.size(), 5u);
  EXPECT_DOUBLE_EQ(tail.poses.back().pose.position.x, 3.0);
}
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();rclcpp::shutdown();return result;
}
