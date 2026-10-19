// Offline diagnostic only. Inspect both candidates without publishing commands.
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include "core.hpp"
// Deliberately confined to this offline diagnostic translation unit.
#define private public
#include "takeover_sim/dynamic_overtaking.hpp"
#undef private
#include <iostream>

int main(int argc, char **argv) {
  if (argc != 2) return 2;
  frenet::Assets assets(argv[1]);
  Json::Value input;
  std::cin >> input;
  takeover::DynamicOvertakingPlanner planner(assets);
  const frenet::Pose ego{input["x"].asDouble(), input["y"].asDouble(), input["yaw"].asDouble()};
  const auto projection = takeover::project(assets, ego.xy());
  planner.maneuver_start_s_ = projection.s;
  planner.maneuver_start_d_ = projection.d;
  takeover::Track track;
  track.valid = true;
  track.position = {input["tx"].asDouble(), input["ty"].asDouble()};
  track.velocity = {input["vx"].asDouble(), input["vy"].asDouble()};
  track.yaw = input["tyaw"].asDouble();
  track.position_variance = std::pow(input["sigma"].asDouble(), 2);
  track.velocity_variance = 0.0;
  Json::Value out;
  for (int side : {1, -1}) {
    auto candidate = planner.build_candidate(ego, input["speed"].asDouble(), track,
        side, takeover::Behavior::OVERTAKE, std::max(0.70, track.velocity.norm() + 0.05));
    Json::Value row;
    row["valid"] = candidate.valid;
    row["reason"] = candidate.reason;
    row["map_clearance_m"] = candidate.minimum_map_clearance;
    row["body_clearance_m"] = candidate.minimum_body_clearance;
    row["curvature"] = candidate.maximum_curvature;
    out[side == 1 ? "left" : "right"] = row;
  }
  std::cout << out << '\n';
}
