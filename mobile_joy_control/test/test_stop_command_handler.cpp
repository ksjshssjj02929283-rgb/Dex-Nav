#include <cstdint>
#include <vector>

#include "gtest/gtest.h"
#include "mobile_joy_control/stop_command_handler.hpp"

namespace mobile_joy_control {
namespace {

TEST(StopCommandHandler, DefaultMappingIsDisabled) {
    StopCommandHandler handler;
    EXPECT_EQ(handler.process(std::vector<int32_t>(12, 1)), StopCommand::NONE);
}

class StopCommandHandlerTest : public ::testing::Test {
protected:
    void SetUp() override { ASSERT_TRUE(handler_.configure(7, 4, 6)); }

    StopCommand process(bool rt, bool lb, bool lt) {
        std::vector<int32_t> buttons(12, 0);
        buttons[7] = rt;
        buttons[4] = lb;
        buttons[6] = lt;
        return handler_.process(buttons);
    }

    StopCommandHandler handler_;
};

TEST_F(StopCommandHandlerTest, SingleButtonsDoNotRequestStop) {
    EXPECT_EQ(process(true, false, false), StopCommand::NONE);
    EXPECT_EQ(process(false, true, false), StopCommand::NONE);
    EXPECT_EQ(process(false, false, true), StopCommand::NONE);
}

TEST_F(StopCommandHandlerTest, QuickStopAcceptsEitherButtonOrder) {
    EXPECT_EQ(process(false, true, false), StopCommand::NONE);
    EXPECT_EQ(process(true, true, false), StopCommand::QUICK_STOP);
    EXPECT_EQ(process(false, false, false), StopCommand::NONE);
    EXPECT_EQ(process(true, false, false), StopCommand::NONE);
    EXPECT_EQ(process(true, true, false), StopCommand::QUICK_STOP);
}

TEST_F(StopCommandHandlerTest, EmergencyStopAcceptsEitherButtonOrder) {
    EXPECT_EQ(process(false, false, true), StopCommand::NONE);
    EXPECT_EQ(process(true, false, true), StopCommand::EMERGENCY_STOP);
    EXPECT_EQ(process(false, false, false), StopCommand::NONE);
    EXPECT_EQ(process(true, false, false), StopCommand::NONE);
    EXPECT_EQ(process(true, false, true), StopCommand::EMERGENCY_STOP);
}

TEST_F(StopCommandHandlerTest, HoldingOrBouncingPartnerDoesNotRepeatCommand) {
    EXPECT_EQ(process(true, true, false), StopCommand::QUICK_STOP);
    for (int i = 0; i < 20; ++i) {
        EXPECT_EQ(process(true, true, false), StopCommand::NONE);
    }
    EXPECT_EQ(process(true, false, false), StopCommand::NONE);
    EXPECT_EQ(process(true, true, false), StopCommand::NONE);
}

TEST_F(StopCommandHandlerTest, EmergencyStopWinsSimultaneousCombinations) {
    EXPECT_EQ(process(true, true, true), StopCommand::EMERGENCY_STOP);
    EXPECT_EQ(process(true, true, false), StopCommand::NONE);
    EXPECT_EQ(process(true, true, true), StopCommand::NONE);
}

TEST_F(StopCommandHandlerTest, QuickStopCanEscalateWithoutReleasingEnable) {
    EXPECT_EQ(process(true, true, false), StopCommand::QUICK_STOP);
    EXPECT_EQ(process(true, true, true), StopCommand::EMERGENCY_STOP);
    EXPECT_EQ(process(true, true, false), StopCommand::NONE);
}

TEST_F(StopCommandHandlerTest, QuickStopCanEscalateAfterReleasingQuickStopButton) {
    EXPECT_EQ(process(true, true, false), StopCommand::QUICK_STOP);
    EXPECT_EQ(process(true, false, false), StopCommand::NONE);
    EXPECT_EQ(process(true, false, true), StopCommand::EMERGENCY_STOP);
}

TEST_F(StopCommandHandlerTest, QuickStopCanEscalateInLaterGesture) {
    EXPECT_EQ(process(true, true, false), StopCommand::QUICK_STOP);
    EXPECT_EQ(process(false, false, false), StopCommand::NONE);
    EXPECT_EQ(process(true, false, true), StopCommand::EMERGENCY_STOP);
}

TEST_F(StopCommandHandlerTest, NewGestureCanRequestQuickStopAfterEmergencyStop) {
    EXPECT_EQ(process(true, false, true), StopCommand::EMERGENCY_STOP);
    EXPECT_EQ(process(false, false, false), StopCommand::NONE);
    EXPECT_EQ(process(true, true, false), StopCommand::QUICK_STOP);
    EXPECT_EQ(process(true, false, true), StopCommand::EMERGENCY_STOP);
}

TEST_F(StopCommandHandlerTest, ReleasingEnableAllowsRetryWithoutRestart) {
    EXPECT_EQ(process(true, true, false), StopCommand::QUICK_STOP);
    EXPECT_EQ(process(false, true, false), StopCommand::NONE);
    EXPECT_EQ(process(true, true, false), StopCommand::QUICK_STOP);
}

TEST_F(StopCommandHandlerTest, MissingInputDoesNotRearmGesture) {
    EXPECT_EQ(process(true, false, true), StopCommand::EMERGENCY_STOP);
    EXPECT_EQ(handler_.process({}), StopCommand::NONE);
    EXPECT_EQ(handler_.process(std::vector<int32_t>(7, 1)), StopCommand::NONE);
    EXPECT_EQ(process(true, false, true), StopCommand::NONE);
}

TEST_F(StopCommandHandlerTest, InvalidEnableValueDoesNotRearmGesture) {
    EXPECT_EQ(process(true, true, false), StopCommand::QUICK_STOP);
    std::vector<int32_t> buttons(12, 0);
    buttons[7] = -1;
    EXPECT_EQ(handler_.process(buttons), StopCommand::NONE);
    buttons[7] = 2;
    EXPECT_EQ(handler_.process(buttons), StopCommand::NONE);
    EXPECT_EQ(process(true, true, false), StopCommand::NONE);
}

TEST_F(StopCommandHandlerTest, InvalidPartnerValueDoesNotRequestStop) {
    std::vector<int32_t> buttons(12, 0);
    buttons[7] = 1;
    buttons[4] = 2;
    buttons[6] = -1;
    EXPECT_EQ(handler_.process(buttons), StopCommand::NONE);
}

TEST_F(StopCommandHandlerTest, MissingStopEnableMustNotAllowMediumSpeedInput) {
    std::vector<int32_t> buttons(6, 0);
    buttons[4] = 1;
    buttons[5] = 1;
    EXPECT_EQ(handler_.process(buttons), StopCommand::NONE);
    EXPECT_FALSE(handler_.has_valid_input(buttons));
}

TEST_F(StopCommandHandlerTest, NormalControlRequiresBinaryStopButtonValues) {
    std::vector<int32_t> buttons(12, 0);
    EXPECT_TRUE(handler_.has_valid_input(buttons));
    for (const auto index : {4, 6, 7}) {
        buttons[index] = -1;
        EXPECT_FALSE(handler_.has_valid_input(buttons));
        buttons[index] = 2;
        EXPECT_FALSE(handler_.has_valid_input(buttons));
        buttons[index] = 1;
        EXPECT_TRUE(handler_.has_valid_input(buttons));
        buttons[index] = 0;
    }
}

TEST_F(StopCommandHandlerTest, DisabledStopMappingDoesNotRequireStopInput) {
    EXPECT_TRUE(handler_.configure(-1, 4, 6));
    EXPECT_TRUE(handler_.has_valid_input({}));
    EXPECT_TRUE(handler_.configure(7, -1, -1));
    EXPECT_TRUE(handler_.has_valid_input({}));
    EXPECT_FALSE(handler_.configure(7, 7, 6));
    EXPECT_TRUE(handler_.has_valid_input({}));
}

TEST_F(StopCommandHandlerTest, NormalControlDoesNotRequireDisabledPartner) {
    EXPECT_TRUE(handler_.configure(7, 4, -1));
    std::vector<int32_t> buttons(8, 0);
    buttons[6] = -1;
    EXPECT_TRUE(handler_.has_valid_input(buttons));
    EXPECT_TRUE(handler_.configure(7, -1, 6));
    buttons[4] = -1;
    buttons[6] = 0;
    EXPECT_TRUE(handler_.has_valid_input(buttons));
}

TEST_F(StopCommandHandlerTest, ValidEmergencyCombinationPrecedesOrdinaryInputValidation) {
    EXPECT_TRUE(handler_.configure(7, 10, 6));
    std::vector<int32_t> buttons(8, 0);
    buttons[6] = 1;
    buttons[7] = 1;
    EXPECT_FALSE(handler_.has_valid_input(buttons));
    EXPECT_EQ(handler_.process(buttons), StopCommand::EMERGENCY_STOP);
}

TEST_F(StopCommandHandlerTest, StopNeedsOnlyButtonsNotJoystickAxes) {
    std::vector<int32_t> buttons(8, 0);
    buttons[6] = 1;
    buttons[7] = 1;
    EXPECT_EQ(handler_.process(buttons), StopCommand::EMERGENCY_STOP);
}

TEST_F(StopCommandHandlerTest, DisabledMappingPreservesNormalInput) {
    EXPECT_TRUE(handler_.configure(-1, -1, -1));
    EXPECT_EQ(process(true, true, true), StopCommand::NONE);
}

TEST_F(StopCommandHandlerTest, CombinationsCanBeDisabledIndividually) {
    EXPECT_TRUE(handler_.configure(7, -1, 6));
    EXPECT_EQ(process(true, true, false), StopCommand::NONE);
    EXPECT_EQ(process(true, false, true), StopCommand::EMERGENCY_STOP);

    StopCommandHandler quick_only;
    EXPECT_TRUE(quick_only.configure(7, 4, -1));
    std::vector<int32_t> buttons(12, 1);
    EXPECT_EQ(quick_only.process(buttons), StopCommand::QUICK_STOP);
}

TEST_F(StopCommandHandlerTest, InvalidMappingDisablesAllCombinations) {
    for (const auto& indices :
         std::vector<std::vector<int>>{{-2, 4, 6}, {7, -2, 6}, {7, 4, -2}, {7, 7, 6}, {7, 4, 7}, {7, 4, 4}}) {
        EXPECT_FALSE(handler_.configure(indices[0], indices[1], indices[2]));
        EXPECT_EQ(process(true, true, true), StopCommand::NONE);
    }
}

TEST_F(StopCommandHandlerTest, OutOfRangeIndicesDoNotReadPastInput) {
    EXPECT_TRUE(handler_.configure(100, 4, 6));
    EXPECT_EQ(process(true, true, true), StopCommand::NONE);
    EXPECT_TRUE(handler_.configure(7, 100, 101));
    EXPECT_EQ(process(true, true, true), StopCommand::NONE);
}

TEST_F(StopCommandHandlerTest, ReconfigurationResetsGesture) {
    EXPECT_EQ(process(true, true, false), StopCommand::QUICK_STOP);
    EXPECT_TRUE(handler_.configure(-1, -1, -1));
    EXPECT_EQ(process(true, true, true), StopCommand::NONE);
    EXPECT_TRUE(handler_.configure(7, 4, 6));
    EXPECT_EQ(process(true, true, false), StopCommand::QUICK_STOP);
}

TEST_F(StopCommandHandlerTest, QuickStopCombinationReservesMediumSpeedButton) {
    std::vector<int32_t> buttons(12, 0);
    buttons[4] = 1;
    EXPECT_FALSE(handler_.is_quick_stop_combination_pressed(buttons));
    buttons[7] = 1;
    EXPECT_TRUE(handler_.is_quick_stop_combination_pressed(buttons));
    EXPECT_EQ(handler_.process(buttons), StopCommand::QUICK_STOP);
    EXPECT_TRUE(handler_.is_quick_stop_combination_pressed(buttons));
    EXPECT_EQ(handler_.process(buttons), StopCommand::NONE);
    buttons[6] = 1;
    EXPECT_TRUE(handler_.is_quick_stop_combination_pressed(buttons));
    EXPECT_EQ(handler_.process(buttons), StopCommand::EMERGENCY_STOP);
    buttons[7] = 0;
    EXPECT_FALSE(handler_.is_quick_stop_combination_pressed(buttons));
}

TEST_F(StopCommandHandlerTest, DisabledQuickStopDoesNotReserveMediumSpeedButton) {
    EXPECT_TRUE(handler_.configure(7, -1, 6));
    EXPECT_FALSE(handler_.is_quick_stop_combination_pressed(std::vector<int32_t>(12, 1)));
    EXPECT_TRUE(handler_.configure(-1, 4, 6));
    EXPECT_FALSE(handler_.is_quick_stop_combination_pressed(std::vector<int32_t>(12, 1)));
}

}  // namespace
}  // namespace mobile_joy_control
