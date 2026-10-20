#ifndef PICO_COPTER_LOG_CAPTURE_GATE_HPP
#define PICO_COPTER_LOG_CAPTURE_GATE_HPP

// Deliberately gated recording. Arm does NOT start recording; CH5 must be
// observed OFF after arm, then turned ON. Each subsequent OFF freezes the log,
// and each new ON starts a fresh ring recording.
class Ch5CaptureGate {
public:
    enum class Action { None, Start, Capture, Stop };

    void on_arm() {
        saw_off_since_arm_ = false;
        recording_ = false;
    }

    void stop() { recording_ = false; }

    Action update(bool ch5_high) {
        if (!ch5_high) {
            saw_off_since_arm_ = true;
            if (recording_) {
                recording_ = false;
                return Action::Stop;
            }
            return Action::None;
        }
        if (!saw_off_since_arm_) return Action::None;
        if (!recording_) {
            recording_ = true;
            return Action::Start;
        }
        return Action::Capture;
    }

    bool recording() const { return recording_; }
private:
    bool saw_off_since_arm_ = false;
    bool recording_ = false;
};
#endif
