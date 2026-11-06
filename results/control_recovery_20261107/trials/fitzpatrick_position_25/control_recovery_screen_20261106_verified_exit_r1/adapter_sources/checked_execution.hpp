#pragma once
#include "joint_control.hpp"
#include "takeover_sim/dynamic_overtaking.hpp"

namespace takeover {

// Simulation adapter only. The physical-v8 controller and bounds are unchanged.
inline frenet::TimedReference execution_reference(const Candidate &candidate,
    double plan_stamp, double control_stamp) {
  if (!std::isfinite(plan_stamp+control_stamp) || control_stamp<plan_stamp-1e-9)
    throw std::runtime_error("execution_clock_invalid");
  frenet::TimedReference timed;
  timed.age=std::max(0.,control_stamp-plan_stamp);
  for (const auto &p:candidate.timed) timed.points.push_back({p.t,p.pose,p.speed});
  timed.validate();
  timed.at(1.20); // Same horizon/expiry in preview, retry and live execution.
  return timed;
}

struct ExecutionAttempt {
  std::string first_rejection;
  std::string final_rejection;
  Json::Value rejected_diagnostic;
  bool cold_retry=false;
  bool cold_recovered=false;
};

// Nothing is published here. A rejected solve may already have advanced its
// command/servo/observer state: roll back to the pre-solve physical runtime,
// not the rejected state and not an artificial standstill. Only the numerical
// warm start is discarded. All retries require the SAME complete validator.
template<class Compute,class Validate>
frenet::Control checked_execution(frenet::JointControl &live,
    frenet::JointControl &snapshot, Compute compute, Validate validate,
    ExecutionAttempt &attempt, bool allow_cold_retry=true) {
  attempt=ExecutionAttempt{};
  snapshot.copy_runtime_from(live);
  try {
    const auto command=compute(live);
    validate(live);
    return command;
  } catch(const std::exception &e) {
    attempt.first_rejection=e.what();
    attempt.rejected_diagnostic=live.diag;
    live.copy_runtime_from(snapshot);
    if(!allow_cold_retry || !retryable_control_rejection(e.what())) {
      attempt.final_rejection=e.what();
      throw;
    }
  }
  attempt.cold_retry=true;
  try {
    const auto command=compute(live);
    validate(live);
    attempt.cold_recovered=true;
    return command;
  } catch(const std::exception &e) {
    attempt.final_rejection=e.what();
    live.copy_runtime_from(snapshot);
    throw;
  }
}
} // namespace takeover
