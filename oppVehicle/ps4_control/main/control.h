#pragma once
#include <stdint.h>

// Percentages are PWM commands, not measured vehicle speed or wheel angle.
namespace hosim {
constexpr int kStop = 1000;
constexpr int kMaxThrottle = 1180;  // Reduced command cap; actual speed is unmeasured.
constexpr int kCenter = 1600;  // User confirmed wheels approximately straight.
constexpr int kSteerMin = 1400;
constexpr int kSteerMax = 1800;
static_assert(kSteerMin < kCenter && kCenter < kSteerMax, "Steering center must be inside endpoints");
constexpr int kSteerSign = -1;  // Reversed to match observed HOSIM steering linkage.
constexpr uint32_t kBootHoldMs = 5000;
constexpr uint32_t kTimeoutMs = 250;
constexpr int kTriggerDeadzone = 30;
constexpr int kStickDeadzone = 40;

struct Input {
    int x = 0;
    int driveY = 0;  // Right stick: negative = up/forward, positive = stop.
    bool enable = false;
    bool stop = false;
};

struct Control {
    int motorUs = kStop;
    int steeringUs = kCenter;
    bool ready = false;
    bool seen = false;
    uint32_t lastPacket = 0;
    const char* state = "WAIT_CONTROLLER";

    void halt(const char* reason) {
        motorUs = kStop;
        steeringUs = kCenter;
        ready = false;
        state = reason;
    }
    void disconnect() {
        seen = false;
        halt("DISCONNECTED");
    }
    void tick(uint32_t now) {
        if (seen && uint32_t(now - lastPacket) >= kTimeoutMs)
            halt("INPUT_TIMEOUT");
    }
    void packet(uint32_t now, uint32_t sinceBoot, const Input& in) {
        // Expiry is checked BEFORE a recovered packet can renew the lease.
        tick(now);
        lastPacket = now;
        seen = true;
        if (in.x < -512 || in.x > 512 || in.driveY < -512 || in.driveY > 512) {
            halt("INVALID_INPUT");
            return;
        }
        if (sinceBoot < kBootHoldMs) {
            halt("BOOT_HOLD");
            return;
        }
        if (in.stop) {
            halt("STOP_BUTTON");
            return;
        }
        if (!in.enable) {
            motorUs = kStop;
            steeringUs = kCenter;
            // Releasing L1 re-arms regardless of the right-stick position.
            ready = true;
            state = "READY_PRESS_L1";
            return;
        }
        if (!ready) {
            halt("RELEASE_L1");
            return;
        }
        const int magnitude = in.x < 0 ? -in.x : in.x;
        const int direction = kSteerSign * (in.x < 0 ? -1 : 1);
        const int span = direction > 0 ? kSteerMax - kCenter : kCenter - kSteerMin;
        // Normalized travel u -> 0.3*u + 0.7*u^3: finer near center,
        // with the same endpoints. Use 64-bit intermediates to avoid overflow.
        const int64_t range = 512 - kStickDeadzone;
        const int64_t travel = magnitude <= kStickDeadzone ? 0 : magnitude - kStickDeadzone;
        const int offset = static_cast<int>(span * travel *
            (3 * range * range + 7 * travel * travel) / (10 * range * range * range));
        steeringUs = kCenter + direction * offset;
        const int forward = -in.driveY;
        motorUs = forward <= kStickDeadzone ? kStop :
            kStop + (forward - kStickDeadzone) * (kMaxThrottle - kStop) /
            (512 - kStickDeadzone);
        state = motorUs > kStop ? "DRIVE" : "STEERING_ONLY";
    }
};
}  // namespace hosim
