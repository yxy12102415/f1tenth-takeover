// Fixed-work native implementation of the existing Python MPC iteration.
// No ROS, actuator access, static mutable state, heap allocation or fast-math.
#include <algorithm>
#include <cmath>

extern "C" int mpc_iteration_abi() { return 1; }

extern "C" int mpc_projected_iterations(
    int n, int maximum_iterations, const double* hessian, const double* linear,
    const double* warm_start, double angle, double steering_limit,
    double first_step, double future_step, double lipschitz,
    double* output) {
  constexpr int capacity = 64;
  if (n < 1 || n > capacity || maximum_iterations < 1 ||
      maximum_iterations > 1000 || !hessian || !linear || !warm_start || !output ||
      !std::isfinite(angle) || !std::isfinite(steering_limit) ||
      steering_limit <= 0.0 || std::abs(angle) > steering_limit ||
      !std::isfinite(first_step) || first_step < 0.0 ||
      !std::isfinite(future_step) || future_step < 0.0 ||
      !std::isfinite(lipschitz) || lipschitz <= 0.0) return -1;
  double current[capacity], next[capacity];
  for (int i = 0; i < n; ++i) {
    if (!std::isfinite(linear[i]) || !std::isfinite(warm_start[i])) return -1;
    current[i] = warm_start[i];
    for (int j = 0; j < n; ++j)
      if (!std::isfinite(hessian[i*n+j])) return -1;
  }
  int completed = 0;
  for (int iteration = 0; iteration < maximum_iterations; ++iteration) {
    for (int i = 0; i < n; ++i) {
      double dot = 0.0;
      for (int j = 0; j < n; ++j) dot += hessian[i*n+j]*current[j];
      double value = current[i] - 2.0*(dot+linear[i])/lipschitz;
      if (!std::isfinite(value)) return -2;
      next[i] = std::min(steering_limit, std::max(-steering_limit, value));
    }
    next[0] = std::min(angle+first_step, std::max(angle-first_step, next[0]));
    for (int i = 1; i < n; ++i)
      next[i] = std::min(next[i-1]+future_step,
                         std::max(next[i-1]-future_step, next[i]));
    double change = 0.0;
    for (int i = 0; i < n; ++i) {
      change = std::max(change, std::abs(next[i]-current[i]));
      current[i] = next[i];
    }
    completed = iteration+1;
    if (change < 1e-6) break;
  }
  for (int i = 0; i < n; ++i) output[i] = current[i];
  return completed;
}
