// Standalone regression checks; no ROS or hardware access.
#include <cassert>
#include <cmath>
#include <iostream>
#include "takeover_sim/dynamic_overtaking.hpp"

int main() {
  takeover::OvertakeObservationGate gate;
  takeover::Track observation;
  observation.valid=observation.measured=true;
  observation.velocity={1.,0.};observation.velocity_variance=.04;
  for(int i=0;i<4;++i) {
    observation.observation_stamp=1.+i*.2;
    assert(gate.update(observation,1.+i*.2)==(i==3));
  }
  // Stale messages and a velocity jump invalidate admission, not safety checks.
  assert(!gate.update(observation,2.));
  observation.observation_stamp=2.;observation.velocity={.5,0.};
  assert(!gate.update(observation,2.));
  observation.measured=false;
  assert(!gate.update(observation,2.1));
  // Admission depends on convergence, not an uncalibrated covariance cutoff.
  // This tracker's steady-state velocity variance is approximately 0.51.
  takeover::OvertakeObservationGate settled;
  observation.measured=true;observation.velocity_variance=.52;
  for(int i=0;i<4;++i) {
    observation.observation_stamp=3.+i*.21;
    assert(settled.update(observation,3.+i*.21)==(i==3));
  }
  takeover::Candidate left, right;
  left.valid = right.valid = true;
  left.minimum_map_clearance = 0.673;
  right.minimum_map_clearance = 0.800;
  left.minimum_body_clearance = 0.087;
  right.minimum_body_clearance = 0.044;
  assert(takeover::candidate_bottleneck_margin(left, 0.05) >
         takeover::candidate_bottleneck_margin(right, 0.05));
  // Prefer the other side if it is genuinely less constrained, not a fixed
  // left-pass rule tied to a particular map.
  std::swap(left.minimum_body_clearance, right.minimum_body_clearance);
  assert(takeover::candidate_bottleneck_margin(right, 0.05) >
         takeover::candidate_bottleneck_margin(left, 0.05));
  right.minimum_map_clearance = 0.06;
  assert(takeover::candidate_bottleneck_margin(right, 0.05) < 0.011);
  left.valid = false;
  assert(takeover::candidate_bottleneck_margin(left, 0.05) < -1e6);
  left.valid = right.valid = true;
  left.minimum_map_clearance = right.minimum_map_clearance = 2.;
  left.minimum_body_clearance = .20; right.minimum_body_clearance = 1.;
  left.target_offset = .82; right.target_offset = -.82;
  left.timed = {{0., 10., 0., 1., {10,0,0}}, {4., 15., .82, 1., {15,.82,0}}};
  right.timed = {{0., 10., 0., 1., {10,0,0}}, {4., 14., -.82, 1., {14,-.82,0}}};
  // Surplus space must not beat better progress on a common time horizon.
  assert(takeover::adaptive_candidate_score(left,.05) > takeover::adaptive_candidate_score(right,.05));
  std::swap(left.timed, right.timed);
  assert(takeover::adaptive_candidate_score(right,.05) > takeover::adaptive_candidate_score(left,.05));
  right.valid = false;
  assert(takeover::adaptive_candidate_score(right,.05) < -1e6);
  assert(takeover::polygons_overlap(
      takeover::rectangle({0, 0, 0}, 0.58, 0.31),
      takeover::rectangle({0.58, 0, 0}, 0.58, 0.31)));
  std::cout << "Candidate scoring and footprint regression checks passed\n";
}
