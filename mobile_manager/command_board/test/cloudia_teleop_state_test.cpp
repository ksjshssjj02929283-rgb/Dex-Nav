#include <gtest/gtest.h>
#include "cloudia_teleop_state.hpp"

using State = command_board::CloudiaTeleopState;
using Phase = State::Phase;

namespace {
State::Remote Stopped(double now) { return {true, false, true, false, true, now}; }
void Ready(State& state, double now) {
    state.UpdateSafety(false, now);
    state.SetRemote(Stopped(now));
}
void Activate(State& state, double now = 1.0) {
    Ready(state, now);
    state.OnJoy(1, now);
    state.OnVelocity({0.3, 0.0, 0.0}, now);
    state.Tick(now, false, false);
    ASSERT_EQ(state.phase(), Phase::StoppingTask);
    state.SetRemote(Stopped(now + 0.01));
    state.Tick(now + 0.01, false, false);
    ASSERT_EQ(state.phase(), Phase::Teleop);
}
}  // namespace

TEST(CloudiaTeleop, DoesNotGrantBeforeSafetyAndFreshRemoteState) {
    State state({});
    EXPECT_FALSE(state.CanSubmit(1.0, false));
    Ready(state, 1.0);
    EXPECT_TRUE(state.CanSubmit(1.0, false));
    EXPECT_FALSE(state.CanSubmit(1.0, true));
    EXPECT_FALSE(state.Tick(1.0, false, false).publish);
}

TEST(CloudiaTeleop, WaitsForSubmissionResultsNotTaskCompletion) {
    State state({});
    Ready(state, 1.0);
    state.OnJoy(1, 1.0);
    state.OnVelocity({0.4, 0.0, 0.0}, 1.0);
    auto output = state.Tick(1.0, true, false);
    EXPECT_EQ(state.phase(), Phase::WaitingSubmissions);
    EXPECT_TRUE(output.publish);
    EXPECT_EQ(output.velocity.x, 0.0);
    EXPECT_FALSE(state.NeedsTaskStop());  // normal takeover waits for the HTTP result
    EXPECT_FALSE(state.CanSubmit(1.0, true));
    state.Tick(1.02, false, false);
    EXPECT_EQ(state.phase(), Phase::StoppingTask);
    EXPECT_TRUE(state.NeedsTaskStop());
    state.SetRemote({true, true, false, false, false, 1.03});
    EXPECT_EQ(state.Tick(1.03, false, false).velocity.x, 0.0);
    EXPECT_EQ(state.phase(), Phase::StoppingTask);
}

TEST(CloudiaTeleop, OnlyForwardsFreshPostGrantVelocity) {
    State state({});
    Activate(state);
    EXPECT_EQ(state.Tick(1.02, false, false).velocity.x, 0.0);
    state.OnVelocity({0.2, -0.1, 0.3}, 1.03);
    auto output = state.Tick(1.03, false, false);
    EXPECT_DOUBLE_EQ(output.velocity.x, 0.2);
    EXPECT_DOUBLE_EQ(output.velocity.y, -0.1);
    EXPECT_DOUBLE_EQ(output.velocity.yaw, 0.3);
}

TEST(CloudiaTeleop, ExistingJoyOneIncludesSoftStopAndZeroTailReleasesMux) {
    State state({});
    Activate(state);
    state.OnVelocity({0.1, 0.0, 0.0}, 1.03);  // joystick, not this policy, computes decay
    EXPECT_DOUBLE_EQ(state.Tick(1.03, false, false).velocity.x, 0.1);
    state.OnVelocity({}, 1.04);
    EXPECT_EQ(state.Tick(1.04, false, false).velocity.x, 0.0);
    state.OnJoy(0, 1.05);
    EXPECT_EQ(state.phase(), Phase::Releasing);
    state.SetRemote(Stopped(1.06));
    EXPECT_TRUE(state.Tick(1.06, false, false).publish);
    EXPECT_EQ(state.phase(), Phase::Idle);
    for (int i = 0; i < 5; ++i) state.Tick(1.07 + i * 0.02, false, false);
    EXPECT_FALSE(state.Tick(1.2, false, false).publish);
}

TEST(CloudiaTeleop, FaultStopsImmediatelyEvenWithPendingSubmission) {
    State state({});
    Activate(state);
    state.OnVelocity({0.4, 0.0, 0.0}, 1.03);
    state.UpdateSafety(true, 1.04);
    EXPECT_EQ(state.phase(), Phase::Fault);
    EXPECT_TRUE(state.NeedsTaskStop());
    EXPECT_EQ(state.Tick(1.04, true, false).velocity.x, 0.0);
    EXPECT_FALSE(state.CanSubmit(1.04, true));
    state.UpdateSafety(false, 1.05);
    state.SetRemote(Stopped(1.05));
    EXPECT_EQ(state.Tick(1.05, false, false).velocity.x, 0.0);
    EXPECT_EQ(state.phase(), Phase::Fault);  // held key cannot rearm
    state.OnJoy(0, 1.06);
    state.SetRemote(Stopped(1.06));
    state.Tick(1.06, false, false);
    EXPECT_EQ(state.phase(), Phase::Idle);
}

TEST(CloudiaTeleop, UncertainSubmissionNeverBecomesAnImplicitGrant) {
    State state({});
    Ready(state, 1.0);
    state.OnJoy(1, 1.0);
    state.Tick(1.0, true, true);
    EXPECT_EQ(state.phase(), Phase::Error);
    state.OnJoy(0, 2.0);
    Ready(state, 2.0);
    state.Tick(2.0, true, true);
    EXPECT_EQ(state.phase(), Phase::Error);
    EXPECT_TRUE(state.NeedsTaskStop());
    EXPECT_FALSE(state.CanSubmit(2.0, true));
}

TEST(CloudiaTeleop, LossOfInputOrLateTaskRevokesForwarding) {
    State state({});
    Activate(state);
    state.OnVelocity({0.4, 0.0, 0.0}, 1.03);
    state.SetRemote({true, true, false, false, false, 1.04});
    EXPECT_EQ(state.Tick(1.04, false, false).velocity.x, 0.0);
    EXPECT_EQ(state.phase(), Phase::Error);
    State lost({});
    Activate(lost);
    EXPECT_EQ(lost.Tick(1.6, false, false).velocity.x, 0.0);
    EXPECT_EQ(lost.phase(), Phase::Error);
}

TEST(CloudiaTeleop, MissingSafetyHeartbeatAndNonFiniteVelocityBlockMotion) {
    State state({});
    Activate(state);
    state.OnJoy(1, 2.1);
    state.OnVelocity({0.4, 0.0, 0.0}, 2.1);
    state.SetRemote(Stopped(2.1));
    EXPECT_EQ(state.Tick(2.1, false, false).velocity.x, 0.0);
    EXPECT_EQ(state.phase(), Phase::Fault);
    State invalid({});
    Activate(invalid);
    invalid.OnVelocity({NAN, 0.0, 0.0}, 1.03);
    EXPECT_EQ(invalid.phase(), Phase::Error);
}

TEST(CloudiaTeleop, TotalTakeoverDeadlineCannotReleasePendingTask) {
    State::Config config;
    config.takeover_timeout = 0.2;
    State state(config);
    Ready(state, 1.0);
    state.OnJoy(1, 1.0);
    state.Tick(1.0, true, false);
    state.OnJoy(1, 1.3);
    state.UpdateSafety(false, 1.3);
    state.Tick(1.3, true, false);
    EXPECT_EQ(state.phase(), Phase::Error);
    EXPECT_FALSE(state.CanSubmit(1.3, true));
}

TEST(CloudiaTeleop, NoActivationWhenReleaseFinishesWhileWaiting) {
    State state({});
    Ready(state, 1.0);
    state.OnJoy(1, 1.0);
    state.Tick(1.0, true, false);
    state.OnJoy(0, 1.1);
    state.Tick(1.1, false, false);
    state.SetRemote(Stopped(1.2));
    EXPECT_EQ(state.Tick(1.2, false, false).velocity.x, 0.0);
    EXPECT_EQ(state.phase(), Phase::Idle);
}

TEST(CloudiaTeleop, NewStopCannotReuseOldStoppedSnapshotToReopenSubmission) {
    State state({});
    Ready(state, 1.0);
    state.UpdateSafety(true, 1.0);
    state.SetRemote(Stopped(1.01));
    state.UpdateSafety(false, 1.02);
    state.RequestStop(1.02, "another stop queued");
    state.Tick(1.02, false, false);
    EXPECT_EQ(state.phase(), Phase::Fault);
    EXPECT_FALSE(state.CanSubmit(1.02, false));
    state.SetRemote(Stopped(1.03));
    state.Tick(1.03, false, false);
    EXPECT_EQ(state.phase(), Phase::Idle);
}
