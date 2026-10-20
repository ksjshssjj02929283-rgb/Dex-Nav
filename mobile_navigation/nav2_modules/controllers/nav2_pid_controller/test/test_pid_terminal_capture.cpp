// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "nav2_pid_controller/pid_controller.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace {

struct RclcppFixture {
    RclcppFixture() { rclcpp::init(0, nullptr); }
    ~RclcppFixture() { rclcpp::shutdown(); }
};
RclcppFixture g_rclcpp_fixture;

// 直接调用真实computeVelocityCommands和TF变换，覆盖轨迹刷新后的连续周期。
class TerminalCapturePID : public nav2_pid_controller::PIDController {
public:
    explicit TerminalCapturePID(bool enable_lock = true) {
        clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
        tf_ = std::make_shared<tf2_ros::Buffer>(clock_);
        // 测试直接注入静态TF，不启动监听线程。
        tf_->setUsingDedicatedThread(true);
        global_frame_ = "odom";
        base_frame_ = "base_link";
        verbose_ = 0;
        d_t_ = 0.1;
        retry_position_capture_cycles_ = 3;
        o_precision_ = 0.02;
        angular_station_pid_conf_.kp = 1.0;
        angular_station_nav2_pid_controller_.Init(angular_station_pid_conf_);
        nav2_controller_common::PidConf linear;
        linear.kp = 1.0;
        high_speed_station_pid_conf_ = low_speed_station_pid_conf_ = linear;
        reverse_station_pid_conf_ = linear;
        high_speed_pid_conf_ = low_speed_pid_conf_ = reverse_speed_pid_conf_ = linear;
        station_nav2_pid_controller_.Init(linear);
        speed_nav2_pid_controller_.Init(linear);
        nav2_pid_controller::EndLocalizationLockConfig config;
        config.switch_distance_m = enable_lock ? 0.05 : 0.0;
        config.switch_angle_rad = enable_lock ? 0.05 : 0.0;
        end_localization_lock_.configure(config);
        resetEndLocalization(1);
    }

    geometry_msgs::msg::Twist step(double x,
                                   double yaw,
                                   double goal_yaw,
                                   double measured_v = 0.0,
                                   double planned_v = 0.014,
                                   double measured_w = 0.0,
                                   bool provide_transform = true) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = global_frame_;
        pose.pose.position.x = x;
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, yaw);
        pose.pose.orientation = tf2::toMsg(orientation);
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = global_frame_;
        transform.child_frame_id = base_frame_;
        transform.header.stamp = clock_->now();
        transform.transform.translation.x = x;
        transform.transform.rotation = pose.pose.orientation;
        if (provide_transform) {
            tf_->setTransform(transform, "terminal_capture_test", true);
        }

        auto goal = pose;
        goal.pose.position.x = 0.0;
        orientation.setRPY(0.0, 0.0, goal_yaw);
        goal.pose.orientation = tf2::toMsg(orientation);
        nav_interfaces::msg::RobotTrajectory trajectory;
        trajectory.header.frame_id = global_frame_;
        for (double point_x : {x - 0.01, 0.02, 0.0}) {
            nav_interfaces::msg::TrajectoryPoint point;
            point.pose = goal;
            point.pose.pose.position.x = point_x;
            point.longitudinal_velocity = planned_v;
            point.region_type = "terminal_handoff";
            trajectory.points.push_back(point);
        }
        setTrajectory(trajectory);
        setEndLocalizationGoal(goal, getEndLocalizationSnapshot().goal_generation);
        geometry_msgs::msg::Twist velocity;
        velocity.linear.x = measured_v;
        velocity.angular.z = measured_w;
        return computeVelocityCommands(pose, velocity, nullptr).twist;
    }

    bool isBraking() const { return curr_controller_stage_ == ControllerStage::BRAKING_FOR_GOAL; }
    bool isRotating() const { return curr_controller_stage_ == ControllerStage::ROTATING_TO_GOAL; }
    bool isApproaching() const { return curr_controller_stage_ == ControllerStage::APPROACHING; }
    bool isRetrying() const {
        return curr_controller_stage_ == ControllerStage::RETRY_ALIGN_TO_POSITION ||
               curr_controller_stage_ == ControllerStage::RETRY_TRANSLATE;
    }
    double desiredSpeed() const { return desired_linear_vel_; }
    void setBaseFrameForTest(const std::string& frame) { base_frame_ = frame; }
    void setReverse(bool enabled) { setReverseEnabled(enabled); }
};

TEST(PidTerminalCapture, ResidualPlannedSpeedCannotBlockFinalYaw) {
    TerminalCapturePID pid;
    auto command = pid.step(-0.0004, 0.0, M_PI_2);
    EXPECT_TRUE(pid.isBraking());
    EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
    command = pid.step(-0.0004, 0.0, M_PI_2);
    EXPECT_TRUE(pid.isBraking());
    command = pid.step(-0.0004, 0.0, M_PI_2);
    EXPECT_GT(pid.desiredSpeed(), 0.01);
    EXPECT_TRUE(pid.isRotating());
    EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
    EXPECT_GT(command.angular.z, 0.0);
    double v = 0.0, a = 0.0;
    EXPECT_FALSE(pid.getLastTrajectoryTrackingTarget(v, a));
}

TEST(PidTerminalCapture, MovingRobotBrakesWithoutChasingPointBearing) {
    TerminalCapturePID pid;
    for (int i = 0; i < 5; ++i) {
        const auto command = pid.step(-0.001, 0.0, M_PI_2, 0.08, 0.05, -0.4);
        EXPECT_TRUE(pid.isBraking());
        EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
        EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
    }
}

TEST(PidTerminalCapture, StopConfirmationRestartsAfterVelocityExcursion) {
    TerminalCapturePID pid;
    pid.step(-0.001, 0.0, M_PI_2);
    pid.step(-0.001, 0.0, M_PI_2, 0.02);
    pid.step(-0.001, 0.0, M_PI_2);
    pid.step(-0.001, 0.0, M_PI_2);
    EXPECT_TRUE(pid.isBraking());
    pid.step(-0.001, 0.0, M_PI_2);
    EXPECT_TRUE(pid.isRotating());
}

TEST(PidTerminalCapture, OvershootDuringBrakingWaitsForStopBeforePositionRetry) {
    TerminalCapturePID pid;
    pid.step(-0.001, 0.0, M_PI_2, 0.1);
    const auto moving = pid.step(0.06, 0.0, M_PI_2, 0.1);
    EXPECT_TRUE(pid.isBraking());
    EXPECT_DOUBLE_EQ(moving.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(moving.angular.z, 0.0);
    for (int i = 0; i < 3; ++i) {
        pid.step(0.06, 0.0, M_PI_2);
    }
    EXPECT_TRUE(pid.isRetrying());
}

TEST(PidTerminalCapture, NewGoalAndRepeatedResetClearStopConfirmation) {
    TerminalCapturePID pid;
    pid.step(-0.001, 0.0, M_PI_2);
    pid.step(-0.001, 0.0, M_PI_2);
    pid.resetEndLocalization(2);
    pid.resetEndLocalization(2);
    pid.step(-0.001, 0.0, M_PI_2);
    EXPECT_TRUE(pid.isBraking());
    pid.step(-0.001, 0.0, M_PI_2);
    EXPECT_TRUE(pid.isBraking());
    pid.step(-0.001, 0.0, M_PI_2);
    EXPECT_TRUE(pid.isRotating());
}

TEST(PidTerminalCapture, InvalidVelocityCannotConfirmStop) {
    TerminalCapturePID pid;
    pid.step(-0.001, 0.0, M_PI_2);
    pid.step(-0.001, 0.0, M_PI_2, std::numeric_limits<double>::quiet_NaN());
    pid.step(-0.001, 0.0, M_PI_2);
    pid.step(-0.001, 0.0, M_PI_2);
    EXPECT_TRUE(pid.isBraking());
}

TEST(PidTerminalCapture, DisabledLocalizationLockStillCapturesPosition) {
    TerminalCapturePID pid(false);
    for (int i = 0; i < 3; ++i) {
        pid.step(-0.001, 0.0, M_PI_2, 0.0, 0.05);
    }
    EXPECT_TRUE(pid.isRotating());
    EXPECT_FALSE(pid.getEndLocalizationSnapshot().position_goal_latched);
}

TEST(PidTerminalCapture, FinalYawUsesShortestDirectionAcrossPi) {
    for (double sign : {1.0, -1.0}) {
        TerminalCapturePID pid;
        geometry_msgs::msg::Twist command;
        for (int i = 0; i < 3; ++i) {
            command = pid.step(-0.001, sign * 3.0, -sign * 3.0);
        }
        ASSERT_TRUE(pid.isRotating());
        EXPECT_GT(command.angular.z * sign, 0.0);
    }
}

TEST(PidTerminalCapture, PositionOutsideToleranceStillTracksPlannedMotion) {
    TerminalCapturePID pid;
    const auto command = pid.step(-0.06, 0.0, M_PI_2, 0.05, 0.05);
    EXPECT_TRUE(pid.isApproaching());
    EXPECT_GT(command.linear.x, 0.0);
}

TEST(PidTerminalCapture, ExactGoalNeverChasesUndefinedPointBearing) {
    TerminalCapturePID pid;
    geometry_msgs::msg::Twist command;
    for (int i = 0; i < 3; ++i) {
        command = pid.step(0.0, M_PI_2, M_PI_2);
    }
    EXPECT_TRUE(pid.isRotating());
    EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
}

TEST(PidTerminalCapture, ReverseTrackingUsesPhysicalFinalYawAfterStop) {
    TerminalCapturePID pid;
    pid.setReverse(true);
    geometry_msgs::msg::Twist command;
    for (int i = 0; i < 3; ++i) {
        command = pid.step(0.001, 0.0, M_PI_2);
    }
    EXPECT_TRUE(pid.isRotating());
    EXPECT_GT(command.angular.z, 0.0);
}

TEST(PidTerminalCapture, FinalPositionRefinementCanContinueAboveStopThreshold) {
    TerminalCapturePID pid;
    for (int i = 0; i < 3; ++i) {
        pid.step(-0.019, 0.0, 0.0);
    }
    ASSERT_TRUE(pid.isRotating());
    const auto command = pid.step(-0.019, 0.0, 0.0, 0.012);
    EXPECT_GT(command.linear.x, 0.0);
    EXPECT_LE(command.linear.x, 0.03);
}

TEST(PidTerminalCapture, TransformFailureBreaksConsecutiveStopConfirmation) {
    TerminalCapturePID pid;
    pid.step(-0.001, 0.0, M_PI_2);
    pid.setBaseFrameForTest("missing_base");
    const auto command = pid.step(-0.001, 0.0, M_PI_2, 0.0, 0.014, 0.0, false);
    EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
    pid.setBaseFrameForTest("base_link");
    pid.step(-0.001, 0.0, M_PI_2);
    pid.step(-0.001, 0.0, M_PI_2);
    EXPECT_TRUE(pid.isBraking());
    pid.step(-0.001, 0.0, M_PI_2);
    EXPECT_TRUE(pid.isRotating());
}

}  // namespace
