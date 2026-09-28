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
    ++calls;
    while (block.load()) {std::this_thread::sleep_for(1ms);}
    nav_interfaces::msg::RobotTrajectory t;
    if (!fail.load()) {
      t.header.frame_id = "map";
      nav_interfaces::msg::TrajectoryPoint p; p.pose = path.poses.front();
      t.points.push_back(p);
    }
    return t;
  }
  void requestCancel() override {block = false;}
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
  void request(uint64_t session, uint64_t revision)
  {
    SnapshotRuntime::Request q;
    q.session = session; q.revision = revision;
    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header.frame_id = "map";
    geometry_msgs::msg::PoseStamped p; p.header.frame_id = "map";p.pose.orientation.w = 1;
    path->poses.push_back(p); q.path = path;
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
  uint64_t resultSession()
  {
    std::lock_guard<std::mutex> l(snapshot_runtime_->mutex);
    return snapshot_runtime_->result ? snapshot_runtime_->result->session : 0;
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
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();rclcpp::shutdown();return result;
}
