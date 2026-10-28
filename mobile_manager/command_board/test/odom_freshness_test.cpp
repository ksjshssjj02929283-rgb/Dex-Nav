#include "status_monitor.hpp"

#include <gtest/gtest.h>

namespace command_board {
namespace {
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

TEST(OdomFreshness, NoMessageDoesNotConfirmStillness) {
    const StatusMonitor::Snapshot snap;
    EXPECT_FALSE(snap.HasFreshVelocity(Clock::time_point{} + 10s));
}

TEST(OdomFreshness, AcceptsOneSecondBoundaryAndRejectsExpiredZeroVelocity) {
    StatusMonitor::Snapshot snap;
    snap.velocity_valid = true;
    snap.odom_received_at = Clock::time_point{} + 10s;
    EXPECT_TRUE(snap.HasFreshVelocity(snap.odom_received_at));
    EXPECT_TRUE(snap.HasFreshVelocity(snap.odom_received_at + 1s));
    EXPECT_FALSE(snap.HasFreshVelocity(snap.odom_received_at + 1s + 1ns));
    EXPECT_FALSE(snap.HasFreshVelocity(snap.odom_received_at + 30s));
}

TEST(OdomFreshness, NewReceptionRestoresFreshnessWithoutChangingVelocity) {
    StatusMonitor::Snapshot snap;
    snap.velocity_valid = true;
    snap.odom_received_at = Clock::time_point{} + 10s;
    const auto now = snap.odom_received_at + 30s;
    EXPECT_FALSE(snap.HasFreshVelocity(now));
    snap.odom_received_at = now;
    EXPECT_TRUE(snap.HasFreshVelocity(now));
}

TEST(OdomFreshness, RejectsInvalidOrFutureReception) {
    StatusMonitor::Snapshot snap;
    snap.odom_received_at = Clock::time_point{} + 10s;
    EXPECT_FALSE(snap.HasFreshVelocity(snap.odom_received_at));
    snap.velocity_valid = true;
    EXPECT_FALSE(snap.HasFreshVelocity(snap.odom_received_at - 1ns));
}

TEST(OdomFreshness, ChassisOdomCallbackUpdatesVelocityAndReceptionTime) {
    rclcpp::init(0, nullptr);
    auto context = rclcpp::contexts::get_global_default_context();
    rclcpp::NodeOptions options;
    options.context(context);
    auto node = std::make_shared<rclcpp::Node>("odom_freshness_test", options);
    StatusMonitor monitor(node.get());
    auto publisher = node->create_publisher<nav_msgs::msg::Odometry>(
        "/feedback/odom", rclcpp::QoS(1).reliable());
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context;
    rclcpp::executors::SingleThreadedExecutor executor(executor_options);
    executor.add_node(node);

    nav_msgs::msg::Odometry msg;
    // 接收新鲜度不依赖消息时间戳（此处保留默认的零时间戳）。
    msg.twist.twist.linear.x = 0.12;
    msg.twist.twist.angular.z = -0.117;
    const auto started = Clock::now();
    StatusMonitor::Snapshot snap;
    while (Clock::now() - started < 5s) {
        publisher->publish(msg);
        executor.spin_once(20ms);
        snap = monitor.GetSnapshot();
        if (snap.velocity_valid) break;
    }
    EXPECT_TRUE(snap.velocity_valid);
    EXPECT_DOUBLE_EQ(snap.velocity.linear.x, msg.twist.twist.linear.x);
    EXPECT_DOUBLE_EQ(snap.velocity.angular.z, msg.twist.twist.angular.z);
    EXPECT_GE(snap.odom_received_at, started);
    EXPECT_TRUE(snap.HasFreshVelocity(Clock::now()));
    EXPECT_FALSE(snap.HasFreshVelocity(snap.odom_received_at + 1s + 1ns));
    executor.remove_node(node);
    context->shutdown("test complete");
}
}  // namespace
}  // namespace command_board
