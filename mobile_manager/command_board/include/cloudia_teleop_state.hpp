#pragma once

#include <cmath>
#include <cstdint>
#include <string>

namespace command_board {

// Pure policy; caller serializes access. All times are monotonic seconds.
// The joystick remains responsible for speed generation and soft stopping.
class CloudiaTeleopState {
public:
    enum class Phase { Idle, StoppingTask, Teleop, Releasing, Fault, Error };
    struct Velocity { double x{0.0}, y{0.0}, yaw{0.0}; };
    struct Remote {
        bool valid{false};
        bool running{false};
        bool ready{false};
        bool fault{false};
        bool stopped{false};
        double received{-1.0};
        int64_t task_id{0};
        bool navigation_blocked{false};  // 是否因为导航停障停车，为了后续细分底盘停车情况避免所有停车都进入故障处理
        bool control_unavailable{false};  // 初始化或充电：不是故障，但不允许遥控
    };
    struct Config {
        double input_timeout{0.5};
        double safety_timeout{1.0};
        double remote_timeout{1.5};
        double takeover_timeout{8.0};
    };
    struct Output {
        bool publish{false};
        Velocity velocity;
    };

    explicit CloudiaTeleopState(Config config) : config_(config) {}

    void OnJoy(int16_t value, double now) {
        joy_ = value;
        joy_time_ = now;
        // joy_in=1 包括松开使能后的平滑停止
        if (value != 1 && phase_ == Phase::Teleop) {
            Enter(Phase::Releasing, now, "joystick released");
        }
    }

    void OnVelocity(Velocity velocity, double now) {
        if (!std::isfinite(velocity.x) || !std::isfinite(velocity.y) ||
            !std::isfinite(velocity.yaw)) {
            Fail(now, "non-finite joystick velocity");
            return;
        }
        velocity_ = velocity;
        velocity_time_ = now;
    }

    void UpdateSafety(bool fault, double now) {
        safety_seen_ = true;
        safety_time_ = now;
        safety_fault_ = fault;
        if (fault && phase_ != Phase::Fault) {
            Enter(Phase::Fault, now, "robot safety fault");
        }
    }

    void RequestStop(double now, const std::string& reason) {
        // 普通停车请求不能解除已有故障.
        if (phase_ != Phase::Fault && phase_ != Phase::Error) {
            Enter(Phase::Releasing, now, reason);
        }
        //废除之前的旧状态，重新查询当前底盘所处状态用于判断是否已经停稳
        InvalidateRemote();
    }

    void SetRemote(Remote remote) { remote_ = remote; }
    void InvalidateRemote() {
        remote_.valid = false;
        ++revision_;
    }
    void Fail(double now, const std::string& reason) {
        if (phase_ != Phase::Fault) Enter(Phase::Error, now, reason);
    }

    bool SafetyOK(double now) const {
        return safety_seen_ && !safety_fault_ && now - safety_time_ <= config_.safety_timeout;
    }
    bool RemoteFresh(double now) const {
        return remote_.valid && now - remote_.received <= config_.remote_timeout;
    }
    bool Quiescent(double now) const {
        //普通导航停障不会阻止遥控接管
        return RemoteFresh(now) && !remote_.fault && !remote_.control_unavailable &&
               (remote_.ready || remote_.navigation_blocked) &&
               !remote_.running && remote_.stopped;
    }
    bool CanReleaseControl(double now) const {
        return Quiescent(now);
    }
    Output Tick(double now) {
        if (safety_seen_ && !SafetyOK(now) && phase_ != Phase::Fault) {
            Enter(Phase::Fault, now, "robot safety state stale or faulty");
        }
        if (RemoteFresh(now) && remote_.fault && phase_ != Phase::Fault) {
            Enter(Phase::Fault, now, "Cloudia reports fault");
        }
        // 待命时只等待；已经开始接管或遥控时，失去控制条件必须保护停车。
        if (RemoteFresh(now) && remote_.control_unavailable &&
            (phase_ == Phase::StoppingTask || phase_ == Phase::Teleop)) {
            Fail(now, "Cloudia initializing or charging during control");
        }

        const bool joy_fresh = joy_time_ >= 0.0 && now - joy_time_ <= config_.input_timeout;
        const bool raw_fresh = velocity_time_ >= 0.0 && now - velocity_time_ <= config_.input_timeout;
        const bool wants_control = joy_fresh && joy_ == 1;
        if (phase_ == Phase::Idle && wants_control && SafetyOK(now) &&
            RemoteFresh(now) && !remote_.fault && !remote_.control_unavailable) {
            Enter(Phase::StoppingTask, now, "teleop requested");
        }
        if (phase_ == Phase::StoppingTask && now - phase_begin_ > config_.takeover_timeout) {
            Fail(now, "teleop takeover timeout");
        }
        if (phase_ == Phase::StoppingTask && Quiescent(now) && SafetyOK(now)) {
            if (wants_control) {
                Enter(Phase::Teleop, now, "Cloudia stopped; teleop granted");
                activated_at_ = now;
            } else {
                EnterIdle(now);
            }
        }
        if (phase_ == Phase::Teleop) {
            if (!joy_fresh || !raw_fresh || joy_ < 0) {
                Fail(now, "joystick input lost");
            } else if (!RemoteFresh(now) || remote_.running || remote_.fault || !SafetyOK(now)) {
                Fail(now, "Cloudia state stale, task restarted, or safety active");
            } else if (joy_ != 1) {
                Enter(Phase::Releasing, now, "joystick released");
            }
        }

        if ((phase_ == Phase::Releasing || phase_ == Phase::Fault || phase_ == Phase::Error) &&
            SafetyOK(now) && CanReleaseControl(now) && joy_ != 1) {
            EnterIdle(now);
        }

        Output output;
        output.publish = phase_ != Phase::Idle || zero_tail_ > 0;
        if (zero_tail_ > 0) --zero_tail_;
        // 永远接收最新的遥控指令，不会执行停止时缓存的旧遥控信息，避免一开始发送高速信息底盘缓存但因为某些原因没响应，后续链路通了突然开始高速行驶的情况
        if (phase_ == Phase::Teleop && raw_fresh && velocity_time_ > activated_at_ && wants_control) {
            output.velocity = velocity_;
        }
        return output;
    }

    bool NeedsTaskStop() const {
        return phase_ == Phase::StoppingTask || phase_ == Phase::Releasing ||
               phase_ == Phase::Fault || phase_ == Phase::Error;
    }
    Phase phase() const { return phase_; }
    uint64_t revision() const { return revision_; }
    const Remote& remote() const { return remote_; }
    const std::string& reason() const { return reason_; }
    static const char* Name(Phase phase) {
        switch (phase) {
            case Phase::Idle: return "IDLE";
            case Phase::StoppingTask: return "STOPPING_TASK";
            case Phase::Teleop: return "TELEOP";
            case Phase::Releasing: return "RELEASING";
            case Phase::Fault: return "FAULT";
            case Phase::Error: return "ERROR";
        }
        return "UNKNOWN";
    }

private:
    void Enter(Phase phase, double now, const std::string& reason) {
        if (phase_ == phase && reason_ == reason) return;
        phase_ = phase;
        phase_begin_ = now;
        reason_ = reason;
        ++revision_;
        if (phase != Phase::Teleop && phase != Phase::Idle) remote_.valid = false;
    }
    void EnterIdle(double now) {
        Enter(Phase::Idle, now, "idle");
        zero_tail_ = 5;
        velocity_ = {};
        velocity_time_ = -1.0;
    }

    Config config_;
    Phase phase_{Phase::Idle};
    Remote remote_;
    Velocity velocity_;
    int16_t joy_{-2};
    double joy_time_{-1.0}, velocity_time_{-1.0}, safety_time_{-1.0};
    double phase_begin_{0.0}, activated_at_{0.0};
    bool safety_seen_{false}, safety_fault_{false};
    uint64_t revision_{0};
    int zero_tail_{0};
    std::string reason_{"waiting for safety/remote state"};
};

}  // namespace command_board
