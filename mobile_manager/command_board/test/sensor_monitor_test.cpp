// SensorMonitor 单元测试（§5.8 S-01 ~ S-12）
//
// 用可注入的单调时钟（测试内为变量）驱动判定，取消回调 / 急停查询 / 故障上报
// 用桩函数计数，不依赖 DDS、不依赖真实时间。

#include "sensor_monitor.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace command_board {
namespace {

using MsgSensorState = nav_interfaces::msg::SensorState;

SensorMonitor::SensorConfig MakeSensor(const std::string& name,
                                       const double timeout_sec,
                                       const bool required,
                                       const bool suppress_on_estop = false) {
    SensorMonitor::SensorConfig sensor;
    sensor.name = name;
    sensor.topic = "/" + name;
    sensor.type = "sensor_msgs/msg/LaserScan";
    sensor.timeout_sec = timeout_sec;
    sensor.required = required;
    sensor.suppress_on_estop = suppress_on_estop;
    return sensor;
}

// 测试夹具：短参数（超时 0.2 s / 保持 0.3 s / 宽限 0.5 s），时钟由 now 变量驱动。
class SensorMonitorTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.enable = true;
        config_.check_period_sec = 0.05;
        config_.startup_grace_sec = 0.5;
        config_.recover_hold_sec = 0.3;
        config_.cancel_repeat_sec = 0.2;
        config_.estop_release_grace_sec = 0.3;
        config_.stats_window_sec = 10.0;
        config_.sensors = {
            MakeSensor("imu", 0.2, true, true),   MakeSensor("odom", 0.2, true, true),
            MakeSensor("lidar_front", 0.2, true), MakeSensor("lidar_rear", 0.2, true),
            MakeSensor("rgbd_front", 0.2, true),  MakeSensor("rgbd_rear", 0.2, true),
        };
        now = 100.0;
        Build();
    }

    void Build() {
        SensorMonitor::Hooks hooks;
        hooks.cancel_navigation = [this]() { ++cancel_count; };
        hooks.is_estop_active = [this]() { return estop; };
        hooks.publish_now = [this]() { ++publish_now_count; };
        hooks.set_nav_fault = [this](const int32_t code, const std::string& detail,
                                     const std::vector<MsgSensorState>& sensors) {
            last_code = code;
            last_detail = detail;
            last_sensors = sensors;
        };
        monitor_ = std::make_unique<SensorMonitor>(config_, std::move(hooks), [this]() { return now; });
    }

    // 让所有传感器都在最近收到一帧。
    void FeedAll() {
        for (const auto& sensor : config_.sensors) {
            monitor_->OnFrame(sensor.name);
        }
    }

    const MsgSensorState* FindState(const std::string& name) const {
        for (const auto& state : monitor_->SensorStates()) {
            if (state.name == name) {
                return &state;
            }
        }
        return nullptr;
    }

    SensorMonitor::Config config_;
    std::unique_ptr<SensorMonitor> monitor_;
    double now = 0.0;
    bool estop = false;
    int cancel_count = 0;
    int publish_now_count = 0;
    int32_t last_code = -1;
    std::string last_detail;
    std::vector<MsgSensorState> last_sensors;
};

TEST_F(SensorMonitorTest, S01AllHealthy) {
    // 100 Hz 连续 1 s，期间每 0.01 s 判定一次
    for (int i = 0; i < 100; ++i) {
        now += 0.01;
        FeedAll();
        monitor_->Tick();
    }
    EXPECT_FALSE(monitor_->NavFaultActive());
    EXPECT_EQ(0, monitor_->NavFaultCode());
    EXPECT_EQ(cancel_count, 0);
    ASSERT_EQ(monitor_->SensorStates().size(), config_.sensors.size());
    for (const auto& state : monitor_->SensorStates()) {
        EXPECT_EQ(state.state, MsgSensorState::STATE_OK) << state.name;
        EXPECT_GT(state.rate_hz, 50.0f) << state.name;
        EXPECT_LT(state.age_sec, 0.05f) << state.name;
    }
}

TEST_F(SensorMonitorTest, S02StaleSingleSensorFaultsAndCancelsOnce) {
    FeedAll();
    now += 0.01;
    monitor_->Tick();
    EXPECT_EQ(cancel_count, 0);

    // lidar_front 停发，其余继续
    for (int i = 0; i < 30; ++i) {
        now += 0.01;
        for (const auto& sensor : config_.sensors) {
            if (sensor.name != "lidar_front") {
                monitor_->OnFrame(sensor.name);
            }
        }
        monitor_->Tick();
    }

    EXPECT_TRUE(monitor_->NavFaultActive());
    EXPECT_EQ(monitor_->NavFaultCode(), SensorMonitor::kNavSensorFaultCode);
    EXPECT_EQ(last_code, SensorMonitor::kNavSensorFaultCode);
    EXPECT_EQ(cancel_count, 1);  // 边沿只取消一次
    EXPECT_NE(last_detail.find("lidar_front"), std::string::npos);
    const auto* state = FindState("lidar_front");
    ASSERT_NE(state, nullptr);
    EXPECT_EQ(state->state, MsgSensorState::STATE_FAULT);
}

TEST_F(SensorMonitorTest, S03FaultPersistsCancelsRepeatedly) {
    FeedAll();
    monitor_->Tick();
    for (int i = 0; i < 200; ++i) {  // 2.0 s
        now += 0.01;
        for (const auto& sensor : config_.sensors) {
            if (sensor.name != "lidar_rear") {
                monitor_->OnFrame(sensor.name);
            }
        }
        monitor_->Tick();
    }
    // 边沿 1 次 + 每 0.2 s 一次：约 1 + (2.0 - 0.2) / 0.2 = 10
    EXPECT_GE(cancel_count, 8);
    EXPECT_LE(cancel_count, 12);
}

TEST_F(SensorMonitorTest, S04NeverReceived) {
    // 宽限期内 UNKNOWN，不取消
    monitor_->Tick();
    EXPECT_FALSE(monitor_->NavFaultActive());
    EXPECT_EQ(FindState("imu")->state, MsgSensorState::STATE_UNKNOWN);
    EXPECT_EQ(cancel_count, 0);

    now += config_.startup_grace_sec + 0.1;
    monitor_->Tick();
    EXPECT_TRUE(monitor_->NavFaultActive());
    EXPECT_EQ(FindState("imu")->state, MsgSensorState::STATE_FAULT);
    EXPECT_NE(last_detail.find("never received"), std::string::npos);
    EXPECT_EQ(cancel_count, 1);
}

TEST_F(SensorMonitorTest, S05RecoverHoldThenClear) {
    FeedAll();
    monitor_->Tick();
    for (int i = 0; i < 30; ++i) {
        now += 0.01;
        for (const auto& sensor : config_.sensors) {
            if (sensor.name != "imu") {
                monitor_->OnFrame(sensor.name);
            }
        }
        monitor_->Tick();
    }
    EXPECT_TRUE(monitor_->NavFaultActive());

    // 恢复：连续 0.2 s 无故障仍保持（hold=0.3 s），之后解除
    for (int i = 0; i < 20; ++i) {
        now += 0.01;
        FeedAll();
        monitor_->Tick();
    }
    EXPECT_TRUE(monitor_->NavFaultActive());
    for (int i = 0; i < 15; ++i) {
        now += 0.01;
        FeedAll();
        monitor_->Tick();
    }
    EXPECT_FALSE(monitor_->NavFaultActive());
    EXPECT_EQ(monitor_->NavFaultCode(), 0);
    EXPECT_EQ(last_code, 0);
    EXPECT_TRUE(last_detail.empty());

    // 解除后不再取消
    const int cancels_after_clear = cancel_count;
    for (int i = 0; i < 30; ++i) {
        now += 0.01;
        FeedAll();
        monitor_->Tick();
    }
    EXPECT_EQ(cancel_count, cancels_after_clear);
}

TEST_F(SensorMonitorTest, S06FaultDuringHoldResetsTimer) {
    FeedAll();
    monitor_->Tick();
    for (int i = 0; i < 30; ++i) {
        now += 0.01;
        for (const auto& sensor : config_.sensors) {
            if (sensor.name != "imu") {
                monitor_->OnFrame(sensor.name);
            }
        }
        monitor_->Tick();
    }
    ASSERT_TRUE(monitor_->NavFaultActive());

    // 恢复 0.25 s（< hold 0.3 s）
    for (int i = 0; i < 25; ++i) {
        now += 0.01;
        FeedAll();
        monitor_->Tick();
    }
    EXPECT_TRUE(monitor_->NavFaultActive());

    // 再次故障：计时清零
    for (int i = 0; i < 30; ++i) {
        now += 0.01;
        for (const auto& sensor : config_.sensors) {
            if (sensor.name != "imu") {
                monitor_->OnFrame(sensor.name);
            }
        }
        monitor_->Tick();
    }
    EXPECT_TRUE(monitor_->NavFaultActive());

    // 重新恢复：必须重新连续满足 hold
    for (int i = 0; i < 25; ++i) {
        now += 0.01;
        FeedAll();
        monitor_->Tick();
    }
    EXPECT_TRUE(monitor_->NavFaultActive());
    for (int i = 0; i < 15; ++i) {
        now += 0.01;
        FeedAll();
        monitor_->Tick();
    }
    EXPECT_FALSE(monitor_->NavFaultActive());
}

TEST_F(SensorMonitorTest, S07ShortGlitchBelowTimeout) {
    FeedAll();
    monitor_->Tick();
    now += 0.15;  // < timeout 0.2 s
    for (const auto& sensor : config_.sensors) {
        if (sensor.name != "lidar_front") {
            monitor_->OnFrame(sensor.name);
        }
    }
    monitor_->Tick();
    EXPECT_FALSE(monitor_->NavFaultActive());
    EXPECT_EQ(FindState("lidar_front")->state, MsgSensorState::STATE_OK);
}

TEST_F(SensorMonitorTest, S09EstopSuppressionAndReleaseGrace) {
    FeedAll();
    monitor_->Tick();

    // 急停：里程计 / IMU 停发 → SUPPRESSED，不故障
    estop = true;
    for (int i = 0; i < 40; ++i) {
        now += 0.01;
        for (const auto& sensor : config_.sensors) {
            if (!sensor.suppress_on_estop) {
                monitor_->OnFrame(sensor.name);
            }
        }
        monitor_->Tick();
    }
    EXPECT_FALSE(monitor_->NavFaultActive());
    EXPECT_EQ(FindState("imu")->state, MsgSensorState::STATE_SUPPRESSED);
    EXPECT_EQ(FindState("odom")->state, MsgSensorState::STATE_SUPPRESSED);

    // 解除急停，宽限内仍 OK；宽限内恢复 → 保持 OK
    estop = false;
    now += 0.01;
    monitor_->Tick();
    EXPECT_EQ(FindState("imu")->state, MsgSensorState::STATE_OK);
    FeedAll();
    for (int i = 0; i < 40; ++i) {
        now += 0.01;
        FeedAll();
        monitor_->Tick();
    }
    EXPECT_FALSE(monitor_->NavFaultActive());

    // 再一次：解除急停后宽限内不恢复 → 宽限后 FAULT
    estop = true;
    now += 0.01;
    monitor_->Tick();
    estop = false;
    for (int i = 0; i < 20; ++i) {  // 0.2 s < grace 0.3 s
        now += 0.01;
        for (const auto& sensor : config_.sensors) {
            if (!sensor.suppress_on_estop) {
                monitor_->OnFrame(sensor.name);
            }
        }
        monitor_->Tick();
    }
    EXPECT_EQ(FindState("imu")->state, MsgSensorState::STATE_OK);
    for (int i = 0; i < 15; ++i) {  // 越过 0.3 s 宽限
        now += 0.01;
        for (const auto& sensor : config_.sensors) {
            if (!sensor.suppress_on_estop) {
                monitor_->OnFrame(sensor.name);
            }
        }
        monitor_->Tick();
    }
    EXPECT_EQ(FindState("imu")->state, MsgSensorState::STATE_FAULT);
    EXPECT_TRUE(monitor_->NavFaultActive());
}

TEST_F(SensorMonitorTest, S10OptionalSensorDoesNotTriggerNavFault) {
    config_.sensors.push_back(MakeSensor("optional_cam", 0.2, false));
    Build();
    FeedAll();
    monitor_->Tick();
    for (int i = 0; i < 30; ++i) {
        now += 0.01;
        for (const auto& sensor : config_.sensors) {
            if (sensor.name != "optional_cam") {
                monitor_->OnFrame(sensor.name);
            }
        }
        monitor_->Tick();
    }
    EXPECT_EQ(FindState("optional_cam")->state, MsgSensorState::STATE_FAULT);
    EXPECT_FALSE(monitor_->NavFaultActive());
    EXPECT_EQ(cancel_count, 0);
    EXPECT_EQ(monitor_->NavFaultCode(), 0);
}

TEST_F(SensorMonitorTest, S11ConfigValidation) {
    std::string err;

    auto config = config_;
    config.sensors[0].topic.clear();
    EXPECT_FALSE(ValidateSensorMonitorConfig(config, err));
    EXPECT_NE(err.find("topic"), std::string::npos);

    config = config_;
    config.sensors[0].type.clear();
    EXPECT_FALSE(ValidateSensorMonitorConfig(config, err));

    config = config_;
    config.sensors[0].timeout_sec = 0.0;
    EXPECT_FALSE(ValidateSensorMonitorConfig(config, err));

    config = config_;
    std::string dup = config.sensors[1].name;
    config.sensors[2].name = dup;
    EXPECT_FALSE(ValidateSensorMonitorConfig(config, err));
    EXPECT_NE(err.find("duplicate"), std::string::npos);

    // check_period > min(timeout)/2
    config = config_;
    config.check_period_sec = 0.2;
    EXPECT_FALSE(ValidateSensorMonitorConfig(config, err));

    config = config_;
    EXPECT_TRUE(ValidateSensorMonitorConfig(config, err)) << err;
}

TEST_F(SensorMonitorTest, S12DisabledDoesNothing) {
    config_.enable = false;
    Build();
    now += 10.0;
    FeedAll();
    monitor_->Tick();
    EXPECT_FALSE(monitor_->NavFaultActive());
    EXPECT_EQ(monitor_->NavFaultCode(), 0);
    EXPECT_TRUE(monitor_->SensorStates().empty());
    EXPECT_EQ(cancel_count, 0);
    EXPECT_FALSE(monitor_->Enabled());
}

}  // namespace
}  // namespace command_board
