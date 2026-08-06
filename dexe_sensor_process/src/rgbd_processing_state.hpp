#pragma once

#include <cstdint>
#include <optional>

namespace dexe_sensor_process {

class FrameDivisionState {
public:
    explicit FrameDivisionState(bool enabled) : enabled_(enabled) {}

    void setEnabled(bool enabled) {
        enabled_ = enabled;
        process_next_ = true;
    }

    bool shouldProcess() {
        if (!enabled_) {
            return true;
        }

        const bool should_process = process_next_;
        process_next_ = !process_next_;
        return should_process;
    }

private:
    bool enabled_;
    bool process_next_{true};
};

class SourceStampIntervalTracker {
public:
    float update(std::int64_t stamp_nanoseconds) {
        if (stamp_nanoseconds <= 0) {
            last_stamp_nanoseconds_.reset();
            return 0.0F;
        }

        if (!last_stamp_nanoseconds_ || stamp_nanoseconds <= *last_stamp_nanoseconds_) {
            last_stamp_nanoseconds_ = stamp_nanoseconds;
            return 0.0F;
        }

        constexpr double nanoseconds_per_second = 1'000'000'000.0;
        const auto interval_nanoseconds = stamp_nanoseconds - *last_stamp_nanoseconds_;
        last_stamp_nanoseconds_ = stamp_nanoseconds;
        return static_cast<float>(static_cast<double>(interval_nanoseconds) / nanoseconds_per_second);
    }

private:
    std::optional<std::int64_t> last_stamp_nanoseconds_;
};

}  // namespace dexe_sensor_process
