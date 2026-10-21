#include <gtest/gtest.h>

#include "speed_limit_arbiter.hpp"

using command_board::SpeedLimitValue;
using command_board::arbitrateSpeedLimit;
using command_board::needsSpeedReference;

namespace {
const SpeedLimitValue kNone{0.0, false};
SpeedLimitValue mps(double v) { return {v, false}; }
SpeedLimitValue pct(double v) { return {v, true}; }
}  // namespace

TEST(SpeedLimitArbiter, NoLimitWhenBothInactive) {
    EXPECT_EQ(arbitrateSpeedLimit(kNone, kNone, 0.6), kNone);
    EXPECT_EQ(arbitrateSpeedLimit(pct(0.0), kNone, 0.6), kNone);   // X-Wiz 用百分比解除
}

TEST(SpeedLimitArbiter, SingleSourceForwardedAsIs) {
    EXPECT_EQ(arbitrateSpeedLimit(mps(0.4), kNone, 0.6), mps(0.4));
    EXPECT_EQ(arbitrateSpeedLimit(pct(50.0), kNone, 0.6), pct(50.0));
    EXPECT_EQ(arbitrateSpeedLimit(kNone, mps(0.3), 0.6), mps(0.3));
}

TEST(SpeedLimitArbiter, LeavingAreaRestoresGlobal) {
    // 出减速区 SpeedFilter 发 0：恢复 X-Wiz 限速，而不是解除
    EXPECT_EQ(arbitrateSpeedLimit(mps(0.4), mps(0.0), 0.6), mps(0.4));
}

TEST(SpeedLimitArbiter, SameUnitTakesStricter) {
    EXPECT_EQ(arbitrateSpeedLimit(mps(0.4), mps(0.3), 0.6), mps(0.3));
    EXPECT_EQ(arbitrateSpeedLimit(mps(0.2), mps(0.3), 0.6), mps(0.2));   // 区域更宽松时不提速
    EXPECT_EQ(arbitrateSpeedLimit(pct(30.0), pct(50.0), 0.6), pct(30.0));
}

TEST(SpeedLimitArbiter, MixedUnitsUseReference) {
    // 50% × 0.6 = 0.30 m/s
    EXPECT_EQ(arbitrateSpeedLimit(pct(50.0), mps(0.2), 0.6), mps(0.2));
    EXPECT_EQ(arbitrateSpeedLimit(pct(50.0), mps(0.4), 0.6), pct(50.0));
    EXPECT_EQ(arbitrateSpeedLimit(mps(0.4), pct(50.0), 0.6), pct(50.0));
    EXPECT_EQ(arbitrateSpeedLimit(pct(50.0), mps(0.3), 0.6), pct(50.0));   // 相等取百分比一方
}

TEST(SpeedLimitArbiter, MixedUnitsWithoutReferenceIsConservative) {
    const auto r = arbitrateSpeedLimit(pct(50.0), mps(0.4), 0.0);
    EXPECT_FALSE(r.percentage);
    EXPECT_DOUBLE_EQ(r.value, 0.2);
    // 不大于任一真实上限（基准 0.6 时真实结果为 0.30）
    EXPECT_LE(r.value, 0.6 * 0.5);
    EXPECT_LE(r.value, 0.4);
}

TEST(SpeedLimitArbiter, NeedsReferenceOnlyForActiveMixedUnits) {
    EXPECT_TRUE(needsSpeedReference(pct(50.0), mps(0.3)));
    EXPECT_FALSE(needsSpeedReference(pct(50.0), mps(0.0)));
    EXPECT_FALSE(needsSpeedReference(mps(0.4), mps(0.3)));
    EXPECT_FALSE(needsSpeedReference(kNone, mps(0.3)));
}
