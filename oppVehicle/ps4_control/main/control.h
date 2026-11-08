#pragma once
#include <stdint.h>

// Percentages are PWM commands, not measured vehicle speed or wheel angle.
namespace hosim {
constexpr int kStop = 1000;
constexpr int kFixedThrottleUs = 1150;  // Intermediate calibration trial after 1140 failed to start; ground speed unmeasured.
static_assert(kStop < kFixedThrottleUs && kFixedThrottleUs <= 1200, "Recalibrate before raising fixed throttle");
constexpr int kCenter = 1600;  // Retained until initial heading / servo trim are distinguished physically.
constexpr int kSteerMin = 1400;
constexpr int kSteerMax = 1800;
static_assert(kSteerMin < kCenter && kCenter < kSteerMax, "Steering center must be inside endpoints");
constexpr uint32_t kBootHoldMs = 5000;
constexpr uint32_t kTimeoutMs = 250;
constexpr int kTriggerDeadzone = 30;

struct Input {
    int x = 0;
    int driveY = 0;  // Stick values validated, but ignored in fixed straight mode.
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
        // L1 alone commands straight travel, independent of either stick.
        steeringUs = kCenter;
        motorUs = kFixedThrottleUs;
        state = "FIXED_STRAIGHT";
    }
};
}  // namespace hosim
