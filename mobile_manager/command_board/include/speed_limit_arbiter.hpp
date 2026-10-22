// 导航门面的限速仲裁：X-Wiz 全局限速（nav_command SET_SPEED_LIMIT）与减速区限速（costmap SpeedFilter）取更严者，
// 结果由门面作为 nav2 speed_limit 话题的唯一发布者发出。纯函数，不依赖 ROS，便于单测。
#pragma once

#include <algorithm>

namespace command_board {

struct SpeedLimitValue {
    double value{0.0};        // 0 = 不限速（nav2 NO_SPEED_LIMIT）
    bool percentage{false};   // true：最大速度百分比 0~100；false：m/s

    bool active() const { return value > 0.0; }
    bool operator==(const SpeedLimitValue& o) const { return value == o.value && percentage == o.percentage; }
    bool operator!=(const SpeedLimitValue& o) const { return !(*this == o); }
};

// 两方都生效且单位不同（一方百分比、一方 m/s）时才需要比较基准。
inline bool needsSpeedReference(const SpeedLimitValue& global, const SpeedLimitValue& area) {
    return global.active() && area.active() && global.percentage != area.percentage;
}

// 返回应发给 controller 的限速：
// - 只有一方生效：原样转发；都不生效：不限速
// - 同单位：取数值小的一方
// - 单位不同：用 ref_max_speed（百分比的 100% 对应速度，即当前速度规划器的 max_speed）把百分比换成 m/s 再比，
//   输出更严一方的原始值，保持该来源原本的单位语义
// - 单位不同且 ref_max_speed 未知（<= 0）：输出 m/s 值 × 百分比。只要减速区限速不超过最大速度，
//   这个值不大于任何一方的真实上限，宁慢勿快；拿到基准后会重新仲裁
inline SpeedLimitValue arbitrateSpeedLimit(const SpeedLimitValue& global, const SpeedLimitValue& area, double ref_max_speed) {
    if (!global.active()) return area.active() ? area : SpeedLimitValue{};
    if (!area.active()) return global;
    if (global.percentage == area.percentage) return global.value <= area.value ? global : area;

    const SpeedLimitValue& pct = global.percentage ? global : area;
    const SpeedLimitValue& abs = global.percentage ? area : global;
    if (ref_max_speed > 0.0) return ref_max_speed * pct.value / 100.0 <= abs.value ? pct : abs;
    return SpeedLimitValue{abs.value * std::min(pct.value, 100.0) / 100.0, false};
}

}  // namespace command_board
