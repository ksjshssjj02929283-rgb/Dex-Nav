#ifndef LOOP_NAVIGATION_BT_LOGGING_HPP_
#define LOOP_NAVIGATION_BT_LOGGING_HPP_

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace loop_navigation_bt {

// 使用固定 UTC+8 生成统一日志时间，不依赖设备系统时区。
inline std::string formatUtc8Time(const std::chrono::system_clock::time_point& time_point) {
    const auto utc8_time = time_point + std::chrono::hours(8);
    const std::time_t time = std::chrono::system_clock::to_time_t(utc8_time);
    std::tm time_info{};
    gmtime_r(&time, &time_info);

    const auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(time_point.time_since_epoch()).count() % 1000;
    std::ostringstream stream;
    stream << std::put_time(&time_info, "%Y-%m-%d %H:%M:%S") << '.' << std::setfill('0') << std::setw(3)
           << milliseconds;
    return stream.str();
}

inline std::string currentUtc8Time() {
    return formatUtc8Time(std::chrono::system_clock::now());
}

}  // namespace loop_navigation_bt

#define LOOP_RCLCPP_DEBUG(logger, format, ...) \
    RCLCPP_DEBUG(logger, "[%s] " format, ::loop_navigation_bt::currentUtc8Time().c_str(), ##__VA_ARGS__)
#define LOOP_RCLCPP_INFO(logger, format, ...) \
    RCLCPP_INFO(logger, "[%s] " format, ::loop_navigation_bt::currentUtc8Time().c_str(), ##__VA_ARGS__)
#define LOOP_RCLCPP_WARN(logger, format, ...) \
    RCLCPP_WARN(logger, "[%s] " format, ::loop_navigation_bt::currentUtc8Time().c_str(), ##__VA_ARGS__)
#define LOOP_RCLCPP_ERROR(logger, format, ...) \
    RCLCPP_ERROR(logger, "[%s] " format, ::loop_navigation_bt::currentUtc8Time().c_str(), ##__VA_ARGS__)

#define LOOP_RCLCPP_INFO_ONCE(logger, format, ...) \
    RCLCPP_INFO_ONCE(logger, "[%s] " format, ::loop_navigation_bt::currentUtc8Time().c_str(), ##__VA_ARGS__)
#define LOOP_RCLCPP_INFO_THROTTLE(logger, clock, duration, format, ...) \
    RCLCPP_INFO_THROTTLE(logger, clock, duration, "[%s] " format, \
                         ::loop_navigation_bt::currentUtc8Time().c_str(), ##__VA_ARGS__)
#define LOOP_RCLCPP_WARN_THROTTLE(logger, clock, duration, format, ...) \
    RCLCPP_WARN_THROTTLE(logger, clock, duration, "[%s] " format, \
                         ::loop_navigation_bt::currentUtc8Time().c_str(), ##__VA_ARGS__)

// 用于到位、计时开始等需要保留事件实际发生时刻的日志。
#define LOOP_RCLCPP_INFO_AT(logger, time_text, format, ...) \
    RCLCPP_INFO(logger, "[%s] " format, time_text.c_str(), ##__VA_ARGS__)
#define LOOP_RCLCPP_WARN_AT(logger, time_text, format, ...) \
    RCLCPP_WARN(logger, "[%s] " format, time_text.c_str(), ##__VA_ARGS__)

#endif  // LOOP_NAVIGATION_BT_LOGGING_HPP_
