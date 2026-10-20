#ifndef PICO_COPTER_SBUS_PARSER_HPP
#define PICO_COPTER_SBUS_PARSER_HPP
#include <stdint.h>

// Pure C++: can be unit tested without the Pico SDK.
namespace sbus {
constexpr uint32_t kLinkTimeoutUs = 100000; // 100 ms
constexpr uint32_t kInterByteGapUs = 5000;
constexpr uint8_t kFrameLostFlag = 0x04;
constexpr uint8_t kFailsafeFlag = 0x08;
struct Frame {
    uint16_t channels[16]{};
    uint8_t flags = 0;
    uint8_t end_byte = 0;
};
inline bool valid_end(uint8_t value) {
    return value == 0x00 || value == 0x04 || value == 0x14
        || value == 0x24 || value == 0x34;
}
inline bool healthy_frame(const Frame& frame) {
    return !(frame.flags & (kFrameLostFlag | kFailsafeFlag));
}
inline bool link_fresh(uint32_t now_us, uint32_t last_us, bool received) {
    return received && static_cast<uint32_t>(now_us - last_us) <= kLinkTimeoutUs;
}
class Decoder {
public:
    bool push(uint8_t byte, uint32_t now_us, Frame& result) {
        if (count_ != 0 && static_cast<uint32_t>(now_us - last_byte_us_) > kInterByteGapUs) count_ = 0;
        last_byte_us_ = now_us;
        if (count_ == 0 && byte != 0x0f) return false;
        data_[count_++] = byte;
        if (count_ != 25) return false;
        count_ = 0;
        if (!valid_end(data_[24])) return false;
        // SBUS packs 16 x 11-bit channel values into bytes 1..22.
        for (uint32_t i = 0; i < 16; ++i) {
            const uint32_t bit = i * 11;
            const uint32_t pos = 1 + bit / 8;
            const uint32_t shift = bit % 8;
            const uint32_t packed = static_cast<uint32_t>(data_[pos])
                | (static_cast<uint32_t>(data_[pos + 1]) << 8)
                | (static_cast<uint32_t>(data_[pos + 2]) << 16);
            result.channels[i] = static_cast<uint16_t>((packed >> shift) & 0x07ff);
        }
        result.flags = data_[23];
        result.end_byte = data_[24];
        return true;
    }
private:
    uint8_t data_[25]{};
    uint8_t count_ = 0;
    uint32_t last_byte_us_ = 0;
};

// Failsafe clears only after 80 uninterrupted 400 Hz ticks (200 ms)
// of fresh RC, low throttle, and an arm gesture that is NOT being held.
class ReArmLatch {
public:
    bool update(bool healthy, bool failsafe_event, bool throttle_low, bool arm_released) {
        if (!healthy || failsafe_event) {
            latched_ = true;
            recovery_ticks_ = 0;
        } else if (latched_) {
            if (throttle_low && arm_released) {
                if (++recovery_ticks_ >= kRecoveryTicks) {
                    latched_ = false;
                    recovery_ticks_ = 0;
                }
            } else {
                recovery_ticks_ = 0;
            }
        }
        return !latched_;
    }
    bool latched() const { return latched_; }
    static constexpr uint16_t kRecoveryTicks = 80;
private:
    bool latched_ = true; // Power-on is inhibited, including when RC is absent.
    uint16_t recovery_ticks_ = 0;
};
} // namespace sbus
#endif
