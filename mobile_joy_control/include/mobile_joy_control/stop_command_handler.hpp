#ifndef MOBILE_JOY_CONTROL__STOP_COMMAND_HANDLER_HPP_
#define MOBILE_JOY_CONTROL__STOP_COMMAND_HANDLER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace mobile_joy_control {

// 区分未请求、软急停和硬急停，协议数值由发布端映射。
enum class StopCommand { NONE, QUICK_STOP, EMERGENCY_STOP };

// 组合键每次按住使能键只触发一次，允许软急停升级为硬急停。
class StopCommandHandler {
public:
    // -1关闭对应组合；其他负值或重叠键位会关闭全部组合。
    bool configure(int enable_button, int quick_stop_button, int emergency_stop_button) {
        const bool is_valid =
            enable_button >= -1 && quick_stop_button >= -1 && emergency_stop_button >= -1 &&
            (enable_button < 0 || quick_stop_button < 0 || enable_button != quick_stop_button) &&
            (enable_button < 0 || emergency_stop_button < 0 || enable_button != emergency_stop_button) &&
            (quick_stop_button < 0 || emergency_stop_button < 0 || quick_stop_button != emergency_stop_button);
        enable_button_ = is_valid ? enable_button : -1;
        quick_stop_button_ = is_valid ? quick_stop_button : -1;
        emergency_stop_button_ = is_valid ? emergency_stop_button : -1;
        gesture_command_ = StopCommand::NONE;
        return is_valid;
    }

    StopCommand process(const std::vector<std::int32_t>& buttons) {
        // 缺失或异常的使能键数据不能解除本次手势的去重状态。
        if (!has_valid_button(buttons, enable_button_)) {
            return StopCommand::NONE;
        }
        if (buttons[enable_button_] == 0) {
            gesture_command_ = StopCommand::NONE;
            return StopCommand::NONE;
        }

        const StopCommand command =
            is_pressed(buttons, emergency_stop_button_)
                ? StopCommand::EMERGENCY_STOP
                : (is_pressed(buttons, quick_stop_button_) ? StopCommand::QUICK_STOP : StopCommand::NONE);
        if (command == StopCommand::NONE || command == gesture_command_ ||
            (command == StopCommand::QUICK_STOP && gesture_command_ == StopCommand::EMERGENCY_STOP)) {
            return StopCommand::NONE;
        }

        gesture_command_ = command;
        return command;
    }

    // 组合键占用提速键时，仅发送停止请求，避免同一输入也触发提速。
    bool is_quick_stop_combination_pressed(const std::vector<std::int32_t>& buttons) const {
        return is_pressed(buttons, enable_button_) && is_pressed(buttons, quick_stop_button_);
    }

    // 普通控制必须能判断全部已启用的急停键，避免缺失RT时把LB解释为提速。
    bool has_valid_input(const std::vector<std::int32_t>& buttons) const {
        if (enable_button_ < 0 || (quick_stop_button_ < 0 && emergency_stop_button_ < 0)) {
            return true;
        }
        return has_valid_button(buttons, enable_button_) &&
               (quick_stop_button_ < 0 || has_valid_button(buttons, quick_stop_button_)) &&
               (emergency_stop_button_ < 0 || has_valid_button(buttons, emergency_stop_button_));
    }

private:
    static bool has_button(const std::vector<std::int32_t>& buttons, int index) {
        return index >= 0 && static_cast<std::size_t>(index) < buttons.size();
    }

    static bool is_pressed(const std::vector<std::int32_t>& buttons, int index) {
        return has_button(buttons, index) && buttons[index] == 1;
    }

    static bool has_valid_button(const std::vector<std::int32_t>& buttons, int index) {
        return has_button(buttons, index) && (buttons[index] == 0 || buttons[index] == 1);
    }

    int enable_button_ = -1;
    int quick_stop_button_ = -1;
    int emergency_stop_button_ = -1;
    StopCommand gesture_command_ = StopCommand::NONE;
};

}  // namespace mobile_joy_control

#endif  // MOBILE_JOY_CONTROL__STOP_COMMAND_HANDLER_HPP_
