#pragma once
#include <stdint.h>

namespace opp {
// Version 1 is physically inhibited: accepted requests NEVER reach the motor.
constexpr bool kActuationEnabled = false;
constexpr uint32_t kMagic = 0x3150504f;
constexpr uint32_t kTimeoutMs = 250;
constexpr uint16_t kStop = 1000, kCenter = 1600, kMaxThrottle = 1180;
enum Type : uint8_t { Hello = 1, Command = 2, Status = 3 };
enum State : uint8_t { Unpaired = 0, WaitingRelease = 1, Ready = 2, Running = 3, Latched = 4 };
#pragma pack(push, 1)
struct Frame {
  uint32_t magic = kMagic;
  uint8_t version = 1, type = Hello, flags = 0, state = 0;
  uint64_t boot = 0, session = 0;
  uint32_t sequence = 0;
  uint16_t motor = kStop, steering = kCenter;
  uint32_t tick = 0;
};
#pragma pack(pop)
static_assert(sizeof(Frame) == 36, "wire layout");

struct Core {
  uint64_t boot, session = 0;
  uint32_t sequence = 0, last = 0, armedAt = 0;
  uint16_t requested = kStop;
  State state = Unpaired;
  explicit Core(uint64_t b) : boot(b) {}
  void stop() { requested = kStop; state = Latched; }
  void tick(uint32_t now) {
    if ((state == Ready || state == Running) && uint32_t(now-last) >= kTimeoutMs) stop();
    if (state == Running && uint32_t(now-armedAt) >= 5000) stop(); // bench window only
  }
  bool accept(const Frame& f, uint32_t now) {
    tick(now); // an expired lease must not be revived by a late packet
    if (f.magic != kMagic || f.version != 1 || f.state != 0 || f.session == 0) return false;
    if (f.type == Hello) {
      // Discovery is read-only. A new process cannot replace the active session.
      return f.boot == 0 && f.sequence == 0 && f.flags == 0 && f.motor == kStop && f.steering == kCenter;
    }
    if (f.type != Command || f.boot != boot || (f.flags & ~1u) || f.sequence == 0) return false;
    const int32_t age = int32_t(now-f.tick);
    if (age < -20 || age > 100) return false;
    if (session == 0) {
      if (f.flags != 0 || f.motor != kStop || f.steering != kCenter) return false;
      session = f.session; state = WaitingRelease;
    }
    if (f.session != session || f.sequence <= sequence) return false;
    // Invalid authenticated commands stop immediately and do not renew the lease.
    if (f.motor < kStop || f.motor > kMaxThrottle || f.steering != kCenter ||
        (!f.flags && f.motor != kStop)) { stop(); return false; }
    sequence = f.sequence;
    if (state == Latched) return true; // reset/reboot required for another bench run
    if (now < 5000) { state = WaitingRelease; requested = kStop; return true; }
    if (!f.flags) {
      requested = kStop;
      if (state == Running) stop();
      else { state = Ready; last = now; }
      return true;
    }
    if (state != Ready && state != Running) return false;
    if (f.motor == kStop) { stop(); return true; }
    if (state == Ready) armedAt = now;
    state = Running; requested = f.motor; last = now;
    return true;
  }
  uint16_t applied() const { return kActuationEnabled ? requested : kStop; }
};
}
