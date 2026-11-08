#pragma once
#include <cmath>

namespace frenet {
// Nominal period is 50 ms. Only a completed, otherwise validated command may
// use the 30 ms jitter allowance. No previous command is replayed here.
class MpcTimingBudget {
  unsigned consecutive_ = 0;
public:
  static constexpr double soft_seconds() { return .045; }
  static constexpr double hard_seconds() { return .075; }
  static constexpr unsigned max_consecutive() { return 3; }
  void reset() { consecutive_ = 0; }
  unsigned consecutive() const { return consecutive_; }
  const char *observe(double elapsed) {
    if (!std::isfinite(elapsed) || elapsed < 0)
      return "joint_control_timing_invalid";
    consecutive_ = elapsed > soft_seconds() ? consecutive_ + 1 : 0;
    if (elapsed > hard_seconds()) return "joint_control_deadline";
    if (consecutive_ >= max_consecutive()) return "joint_control_overload";
    return nullptr;
  }
};
} // namespace frenet
