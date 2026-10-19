// Standalone regression checks; no ROS or hardware access.
#include <cassert>
#include <cmath>
#include <iostream>
#include "takeover_sim/dynamic_overtaking.hpp"

int main() {
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
  assert(takeover::polygons_overlap(
      takeover::rectangle({0, 0, 0}, 0.58, 0.31),
      takeover::rectangle({0.58, 0, 0}, 0.58, 0.31)));
  std::cout << "5 candidate/footprint regression checks passed\n";
}
