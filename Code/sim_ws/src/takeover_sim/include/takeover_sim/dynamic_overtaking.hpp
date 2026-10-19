#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "core.hpp"

namespace takeover {

using frenet::Assets;
using frenet::INF;
using frenet::Path;
using frenet::Point;
using frenet::Pose;
using frenet::clip;
using frenet::wrap;

enum class Behavior {
  CRUISE,
  FOLLOW,
  PREPARE_OVERTAKE,
  OVERTAKE,
  REJOIN,
  WAIT,
  CANCEL
};

inline const char *behavior_name(Behavior value) {
  switch (value) {
  case Behavior::CRUISE: return "CRUISE";
  case Behavior::FOLLOW: return "FOLLOW";
  case Behavior::PREPARE_OVERTAKE: return "PREPARE_OVERTAKE";
  case Behavior::OVERTAKE: return "OVERTAKE";
  case Behavior::REJOIN: return "REJOIN";
  case Behavior::WAIT: return "WAIT";
  case Behavior::CANCEL: return "CANCEL";
  }
  return "UNKNOWN";
}

struct Track {
  int id = 1;
  Point position = Point::Zero();
  Point velocity = Point::Zero();
  double yaw = 0.0;
  double length = 0.58;
  double width = 0.31;
  double stamp = -INF;
  double observation_stamp = -INF;
  double position_variance = 0.04;
  double velocity_variance = 0.25;
  bool valid = false;
  bool measured = false;
};

struct Prediction {
  double t = 0.0;
  Pose pose;
  double sigma = 0.0;
};

struct TimedState {
  double t = 0.0;
  double s = 0.0;
  double d = 0.0;
  double speed = 0.0;
  Pose pose;
};

struct Projection {
  double s = 0.0;
  double d = 0.0;
  int segment = 0;
  Pose pose;
};

inline Projection project(const Assets &assets, const Point &query) {
  Projection best;
  double distance = INF;
  for (size_t i = 1; i < assets.route.size(); ++i) {
    const Point a = assets.route[i - 1].xy();
    const Point edge = assets.route[i].xy() - a;
    const double length2 = edge.squaredNorm();
    if (length2 < 1e-12) continue;
    const double u = clip((query - a).dot(edge) / length2, 0.0, 1.0);
    const Point point = a + u * edge;
    const double candidate = (query - point).squaredNorm();
    if (candidate < distance) {
      distance = candidate;
      const double yaw = assets.route[i - 1].yaw +
          u * wrap(assets.route[i].yaw - assets.route[i - 1].yaw);
      best.s = assets.arc[i - 1] + u * std::sqrt(length2);
      best.d = Point(-std::sin(yaw), std::cos(yaw)).dot(query - point);
      best.segment = static_cast<int>(i - 1);
      best.pose = {point.x(), point.y(), yaw};
    }
  }
  return best;
}

inline Pose route_pose(const Assets &assets, double s) {
  const double total = assets.arc.back();
  if (assets.closed_single_lap && total > 0.0) {
    s = std::fmod(s, total);
    if (s < 0.0) s += total;
  } else {
    s = clip(s, 0.0, total);
  }
  size_t hi = std::min(assets.route.size() - 1,
      static_cast<size_t>(std::upper_bound(assets.arc.begin(), assets.arc.end(), s) - assets.arc.begin()));
  const size_t lo = hi ? hi - 1 : 0;
  const double span = std::max(1e-9, assets.arc[hi] - assets.arc[lo]);
  const double u = hi == lo ? 0.0 : (s - assets.arc[lo]) / span;
  return {
      assets.route[lo].x + u * (assets.route[hi].x - assets.route[lo].x),
      assets.route[lo].y + u * (assets.route[hi].y - assets.route[lo].y),
      assets.route[lo].yaw + u * wrap(assets.route[hi].yaw - assets.route[lo].yaw)};
}

inline double signed_progress(double other_s, double ego_s, double length) {
  double delta = other_s - ego_s;
  if (length > 0.0) {
    while (delta > 0.5 * length) delta -= length;
    while (delta < -0.5 * length) delta += length;
  }
  return delta;
}

inline std::vector<Prediction> predict_cv(const Track &track, double horizon,
                                          double dt, double process_sigma_rate) {
  std::vector<Prediction> output;
  const double speed = track.velocity.norm();
  const double yaw = speed > 0.08 ? std::atan2(track.velocity.y(), track.velocity.x()) : track.yaw;
  for (double t = 0.0; t <= horizon + 1e-9; t += dt) {
    Prediction p;
    p.t = t;
    p.pose = {track.position.x() + track.velocity.x() * t,
              track.position.y() + track.velocity.y() * t, yaw};
    p.sigma = std::sqrt(std::max(0.0, track.position_variance +
        t * t * track.velocity_variance + process_sigma_rate * process_sigma_rate * t * t));
    output.push_back(p);
  }
  return output;
}

// Optional map-assisted prediction. The filter still estimates Cartesian
// position/velocity; this is a separate forecast with an explicit route prior.
inline std::vector<Prediction> predict_track(const Assets &assets, const Track &track,
    double horizon, double dt, double noise, bool assisted, double age = 0.0) {
  const Projection q = project(assets, track.position);
  const double v = track.velocity.norm();
  const Point tangent(std::cos(q.pose.yaw), std::sin(q.pose.yaw));
  const Point normal(-tangent.y(), tangent.x());
  const double heading = v > .15 ? wrap(std::atan2(track.velocity.y(), track.velocity.x()) - q.pose.yaw) : 0.;
  const bool on_route = assisted && std::abs(q.d) < 1.2 && std::abs(heading) < .65;
  const double curvature = wrap(route_pose(assets, q.s + .15).yaw -
                                route_pose(assets, q.s - .15).yaw) / .30;
  const double metric = std::max(.5, 1. - curvature * q.d);
  const double vs = track.velocity.dot(tangent) / metric;
  const double vd = track.velocity.dot(normal);
  std::vector<Prediction> output;
  for (double t = 0.; t <= horizon + 1e-9; t += dt) {
    const double tau = std::max(0., age) + t;
    Prediction p;
    p.t = t;
    if (on_route) {
      const double d = q.d + vd * (1. - std::exp(-tau));
      p.pose = route_pose(assets, q.s + vs * tau);
      p.pose.x -= std::sin(p.pose.yaw) * d;
      p.pose.y += std::cos(p.pose.yaw) * d;
      p.pose.yaw += std::atan2(vd * std::exp(-tau), std::max(.05, track.velocity.dot(tangent)));
    } else {
      const Point xy = track.position + tau * track.velocity;
      p.pose = {xy.x(), xy.y(), v > .08 ? std::atan2(track.velocity.y(), track.velocity.x()) : track.yaw};
    }
    p.sigma = std::sqrt(std::max(0., track.position_variance + tau*tau *
        (track.velocity_variance + noise*noise + (on_route ? .02*.02 : 0.))));
    output.push_back(p);
  }
  return output;
}

using Polygon = std::array<Point, 4>;

inline Polygon rectangle(const Pose &pose, double length, double width) {
  const Point longitudinal(std::cos(pose.yaw), std::sin(pose.yaw));
  const Point lateral(-std::sin(pose.yaw), std::cos(pose.yaw));
  const Point center(pose.x, pose.y);
  const Point a = 0.5 * length * longitudinal;
  const Point b = 0.5 * width * lateral;
  return {center + a + b, center + a - b, center - a - b, center - a + b};
}

inline bool overlap_on_axis(const Polygon &a, const Polygon &b, Point axis) {
  const double norm = axis.norm();
  if (norm < 1e-12) return true;
  axis /= norm;
  double amin = INF, amax = -INF, bmin = INF, bmax = -INF;
  for (const auto &p : a) { amin = std::min(amin, p.dot(axis)); amax = std::max(amax, p.dot(axis)); }
  for (const auto &p : b) { bmin = std::min(bmin, p.dot(axis)); bmax = std::max(bmax, p.dot(axis)); }
  return !(amax < bmin || bmax < amin);
}

inline bool polygons_overlap(const Polygon &a, const Polygon &b) {
  for (const Polygon *polygon : {&a, &b}) {
    for (size_t i = 0; i < polygon->size(); ++i) {
      const Point edge = (*polygon)[(i + 1) % polygon->size()] - (*polygon)[i];
      if (!overlap_on_axis(a, b, Point(-edge.y(), edge.x()))) return false;
    }
  }
  return true;
}

inline double point_segment_distance(const Point &p, const Point &a, const Point &b) {
  const Point edge = b - a;
  const double u = edge.squaredNorm() < 1e-12 ? 0.0 :
      clip((p - a).dot(edge) / edge.squaredNorm(), 0.0, 1.0);
  return (p - (a + u * edge)).norm();
}

inline double polygon_distance(const Polygon &a, const Polygon &b) {
  if (polygons_overlap(a, b)) return 0.0;
  double result = INF;
  for (const Polygon *first : {&a, &b}) {
    const Polygon &second = first == &a ? b : a;
    for (const Point &p : *first)
      for (size_t j = 0; j < second.size(); ++j)
        result = std::min(result, point_segment_distance(
            p, second[j], second[(j + 1) % second.size()]));
  }
  return result;
}

struct Candidate {
  bool valid = false;
  std::string reason = "uninitialized";
  int side = 0;
  double speed_cap = 0.7;
  double transition_length = 0.;
  double minimum_body_clearance = INF;
  double minimum_map_clearance = INF;
  double maximum_curvature = 0.0;
  Path path;
  std::vector<TimedState> timed;
};

inline double candidate_bottleneck_margin(const Candidate &candidate, double map_margin) {
  if (!candidate.valid) return -INF;
  // Rank by the more restrictive clearance, after mandatory inflation/margins.
  // Wall-only ranking selected corridors almost touching the predicted lead
  // envelope even when the other passing side had substantially more room.
  return std::min(candidate.minimum_map_clearance - map_margin,
                  candidate.minimum_body_clearance);
}

struct Decision {
  Behavior behavior = Behavior::WAIT;
  Candidate candidate;
  std::vector<Prediction> target_prediction;
  int committed_side = 0;
  bool stop = true;
  bool target_fresh = false;
  double target_age = INF;
  double target_gap = INF;
  double target_speed = 0.0;
  double planning_ms = 0.0;
  int overtakes_completed = 0;
  std::string transition = "initialized";
};

// The control clock can cross a freshness boundary between planner ticks.
// Re-evaluate behavior first, using the same timeout; never bypass expiry in
// the MPC or extend the age of an observation. A completed rejoin may become
// CRUISE, while an unfinished maneuver must become WAIT.
inline bool freshness_replan_required(const Decision &last, const Track &track,
    double now, double normal_timeout, double committed_timeout) {
  if (!last.target_fresh || !track.valid) return false;
  const bool committed = last.behavior == Behavior::OVERTAKE ||
      last.behavior == Behavior::REJOIN || last.behavior == Behavior::CANCEL;
  return now - track.observation_stamp > (committed ? committed_timeout : normal_timeout);
}

class DynamicOvertakingPlanner {
public:
  explicit DynamicOvertakingPlanner(const Assets &assets) : assets_(assets) {}

  double cruise_speed = 1.5;
  double horizon = 4.0;
  double step = 0.10;
  double desired_gap = 1.15;
  double detection_timeout = 1.20;
  double committed_prediction_timeout = 4.0;
  double vehicle_length = 0.58;
  double vehicle_width = 0.31;
  // 0.82 m clears two 0.31 m bodies plus the worst-case prediction inflation,
  // while retaining the v8 controller's 0.25 m centre-to-wall clearance in
  // the 2.20 m benchmark corridor.
  double pass_offset = 0.82;
  double lane_change_length = 2.6;
  double rejoin_length = 2.8;
  double map_margin = 0.05;
  double body_margin = 0.08;
  double uncertainty_sigma = 2.0;
  double process_sigma_rate = 0.08;
  double acceleration_max = 1.0;
  double deceleration_max = 0.8;
  bool track_assisted_prediction = true;
  bool timed_control = true;
  int allowed_pass_side = 0; // 0: automatic; +/-1: restrict candidates for regression
  double initial_command = 0.;
  double initial_acceleration = 0.;

  Decision plan(Pose ego, double ego_speed, const Track &track, double now) {
    const double begin = frenet::mono();
    Decision decision;
    decision.behavior = behavior_;
    decision.committed_side = committed_side_;
    decision.overtakes_completed = overtakes_completed_;
    decision.target_age = track.valid ? std::max(0.0, now - track.observation_stamp) : INF;
    const bool committed = behavior_ == Behavior::OVERTAKE ||
                           behavior_ == Behavior::REJOIN || behavior_ == Behavior::CANCEL;
    const double allowed_prediction_age = committed
        ? committed_prediction_timeout : detection_timeout;
    decision.target_fresh = track.valid &&
                            decision.target_age <= allowed_prediction_age;

    const Projection ego_projection = project(assets_, ego.xy());
    Projection target_projection;
    if (track.valid) {
      target_projection = project(assets_, track.position);
      decision.target_gap = signed_progress(target_projection.s, ego_projection.s,
                                            assets_.arc.back());
      decision.target_speed = track.velocity.norm();
      decision.target_prediction = predict_track(assets_, track, horizon, step,
          process_sigma_rate, track_assisted_prediction);
    }

    // Complete an already-committed return BEFORE checking target expiry.
    // The ego can cross both the rejoin threshold and the age threshold
    // between 5 Hz ticks. Stopping first strands a fully rejoined car in the
    // departed lead's lane. Keep the same position, heading and separation
    // criteria; do not extend the target timeout or ignore unfinished loss.
    if(behavior_==Behavior::REJOIN && track.valid &&
        std::abs(ego_projection.d)<.10 &&
        std::abs(wrap(ego.yaw-ego_projection.pose.yaw))<.15 &&
        decision.target_gap < -1.20) {
      ++overtakes_completed_;
      committed_side_=0;prepare_start_=-INF;aborting_=false;
      set_behavior(Behavior::CRUISE,"reference_route_rejoined");
    }

    // After a completed pass the slower vehicle is behind the forward LiDAR
    // by construction.  Treating that expected loss as an emergency stop
    // leaves the ego stationary in the lead vehicle's lane and creates a
    // rear-end collision.  Continue route cruise; a newly observed target
    // ahead will re-enter the ordinary FOLLOW logic below.
    const bool post_pass_cruise = overtakes_completed_ > 0 &&
                                  behavior_ == Behavior::CRUISE;
    if (!decision.target_fresh && !post_pass_cruise) {
      set_behavior(Behavior::WAIT, "target_stale");
      decision.behavior = behavior_;
      decision.stop = true;
      decision.transition = transition_;
      decision.planning_ms = (frenet::mono() - begin) * 1000.0;
      return decision;
    }

    const double gap = decision.target_gap;
    const double lead_speed = decision.target_speed;
    const bool lead_ahead = decision.target_fresh && gap > -0.4 && gap < 7.0;
    const bool slower = lead_speed + 0.18 < cruise_speed;

    if (behavior_ == Behavior::WAIT) {
      if(committed_side_ && std::abs(ego_projection.d)>.10) {
        aborting_=true; cancel_start_=now;
        set_behavior(Behavior::CANCEL,"fresh_target_recovered_mid_maneuver");
      } else {
        set_behavior(lead_ahead ? Behavior::FOLLOW : Behavior::CRUISE,
                     "fresh_target_recovered");
      }
    }
    // An aborted manoeuvre keeps its side until a feasible rejoin completes.
    // Do not restart side selection every cycle from a partially shifted pose.
    if (behavior_ == Behavior::CANCEL && now - cancel_start_ > .5) {
      if (std::abs(ego_projection.d) < .10 &&
          std::abs(wrap(ego.yaw - ego_projection.pose.yaw)) < .15) {
        committed_side_ = 0;
        aborting_ = false;
        prepare_start_ = -INF;
        set_behavior(lead_ahead ? Behavior::FOLLOW : Behavior::CRUISE, "abort_rejoined");
      } else if (gap < -1.2 || gap > desired_gap + .8) {
        maneuver_start_s_ = ego_projection.s;
        set_behavior(Behavior::REJOIN, "abort_rejoin_clear");
      }
    }
    if (behavior_ == Behavior::CRUISE && lead_ahead) {
      set_behavior(Behavior::FOLLOW, "lead_vehicle_ahead");
    }
    if (behavior_ == Behavior::FOLLOW && (!lead_ahead || gap < -0.8)) {
      set_behavior(Behavior::CRUISE, "lead_not_ahead");
    }
    if (behavior_ == Behavior::FOLLOW && slower && gap < 4.5 && gap > 1.35) {
      if (prepare_start_ < 0.0) prepare_start_ = now;
      set_behavior(Behavior::PREPARE_OVERTAKE, "slower_lead_confirmed");
    }
    if (behavior_ == Behavior::PREPARE_OVERTAKE && (!slower || !lead_ahead)) {
      prepare_start_ = -INF;
      set_behavior(Behavior::FOLLOW, "overtake_trigger_cleared");
    }

    double cap = cruise_speed;
    if (behavior_ == Behavior::FOLLOW || behavior_ == Behavior::PREPARE_OVERTAKE) {
      cap = clip(lead_speed + 0.65 * (gap - desired_gap), 0.70, cruise_speed);
    }

    if (behavior_ == Behavior::PREPARE_OVERTAKE && now - prepare_start_ >= 0.40) {
      maneuver_start_s_ = ego_projection.s;
      maneuver_start_d_ = ego_projection.d;
      Candidate left = build_candidate(ego, ego_speed, track, +1, Behavior::OVERTAKE,
                                       std::max(0.70, lead_speed + 0.05));
      Candidate right = build_candidate(ego, ego_speed, track, -1, Behavior::OVERTAKE,
                                        std::max(0.70, lead_speed + 0.05));
      if (allowed_pass_side < 0) left.valid = false;
      if (allowed_pass_side > 0) right.valid = false;
      if (left.valid || right.valid) {
        Candidate selected = !right.valid || (left.valid &&
            candidate_bottleneck_margin(left, map_margin) >=
            candidate_bottleneck_margin(right, map_margin)) ? left : right;
        committed_side_ = selected.side;
        aborting_ = false;
        set_behavior(Behavior::OVERTAKE, selected.side > 0 ?
                     "left_corridor_committed" : "right_corridor_committed");
      }
    }

    const Projection current_projection = project(assets_, ego.xy());
    if(behavior_==Behavior::OVERTAKE && !slower && gap>1.35 &&
        std::abs(current_projection.d)<.35) {
      aborting_=true;cancel_start_=now;
      set_behavior(Behavior::CANCEL,"speed_advantage_disappeared");
    }
    if (behavior_ == Behavior::OVERTAKE) {
      const double offset_progress = std::abs(current_projection.d);
      // Build lateral separation at approximately the lead speed.  Closing
      // during the lane-entry transient repeatedly invalidates an otherwise
      // safe pass and produces a speed saw-tooth.  Once footprint separation
      // exists, the same time-aligned check below permits the full pass speed.
      cap = offset_progress < 0.68
          ? std::min(cruise_speed, std::max(0.70, lead_speed + 0.05))
          : cruise_speed;
      // Start returning as soon as full longitudinal body separation exists
      // AND a complete inflated, time-aligned return is feasible. Waiting for
      // a fixed extra metre wastes the bounded prediction window on the outer
      // side of a bend. Completion still requires gap < -1.20 and alignment.
      const double return_gap = .5 * (vehicle_length + track.length) + body_margin;
      if (gap < -return_gap) {
        const double old_start_s = maneuver_start_s_;
        const double old_start_d = maneuver_start_d_;
        maneuver_start_s_ = current_projection.s;
        maneuver_start_d_ = current_projection.d;
        const auto returning = build_candidate(ego, ego_speed, track, committed_side_,
                                               Behavior::REJOIN, cruise_speed);
        if (returning.valid) {
          set_behavior(Behavior::REJOIN, "ego_body_clear_return_feasible");
        } else {
          maneuver_start_s_ = old_start_s;
          maneuver_start_d_ = old_start_d;
        }
      }
    }
    if (behavior_ == Behavior::REJOIN) {
      cap = aborting_ && lead_ahead ?
          clip(lead_speed + .65*(gap-desired_gap), .70, cruise_speed) : cruise_speed;
      if (std::abs(current_projection.d) < 0.10 &&
          std::abs(wrap(ego.yaw-current_projection.pose.yaw)) < .15 &&
          (gap < -1.20 || aborting_)) {
        const bool passed = gap < -1.20;
        if (passed) ++overtakes_completed_;
        committed_side_ = 0;
        prepare_start_ = -INF;
        aborting_ = false;
        set_behavior(passed ? Behavior::CRUISE : Behavior::FOLLOW,
                     passed ? "reference_route_rejoined" : "abort_rejoined_behind_lead");
      }
    }
    if (behavior_ == Behavior::CANCEL)
      // Create a longitudinal gap for recovery instead of matching the lead
      // indefinitely while sitting alongside it. Retain the locked side.
      cap = clip(lead_speed + .5*(gap-desired_gap-1.0), .70,
                 std::min(cruise_speed,std::max(.70,lead_speed)));

    Track collision_track = track;
    if (post_pass_cruise && !decision.target_fresh) collision_track.valid = false;
    Candidate candidate = build_candidate(ego, ego_speed, collision_track, committed_side_,
                                          behavior_, cap);
    if (!candidate.valid && behavior_ == Behavior::PREPARE_OVERTAKE) {
      candidate = build_candidate(ego, ego_speed, track, 0, Behavior::FOLLOW, cap);
    }
    if (!candidate.valid && behavior_ == Behavior::OVERTAKE) {
      // A full-speed candidate can be temporarily unsafe while the bodies are
      // still longitudinally adjacent.  Keep the committed side and retry at
      // lead speed before cancelling; side switching here is prohibited.
      const double conservative_cap =
          std::min(cruise_speed, std::max(0.70, lead_speed + 0.05));
      Candidate conservative = build_candidate(
          ego, ego_speed, track, committed_side_, Behavior::OVERTAKE,
          conservative_cap);
      if (conservative.valid) candidate = std::move(conservative);
    }
    if (!candidate.valid && (behavior_ == Behavior::OVERTAKE || behavior_ == Behavior::REJOIN)) {
      cancel_start_ = now;
      aborting_ = true;
      set_behavior(Behavior::CANCEL, "committed_corridor_invalid:"+candidate.reason);
      candidate = build_candidate(ego, ego_speed, track, committed_side_,
                                  Behavior::CANCEL, 0.70);
    }

    decision.behavior = behavior_;
    decision.candidate = candidate;
    decision.committed_side = committed_side_;
    decision.stop = !candidate.valid || behavior_ == Behavior::WAIT ||
                    (lead_ahead && lead_speed < 0.25 && gap < desired_gap + 0.35 &&
                     std::abs(current_projection.d) < 0.35);
    decision.overtakes_completed = overtakes_completed_;
    decision.transition = transition_;
    decision.planning_ms = (frenet::mono() - begin) * 1000.0;
    return decision;
  }

private:
  const Assets &assets_;
  Behavior behavior_ = Behavior::CRUISE;
  int committed_side_ = 0;
  int overtakes_completed_ = 0;
  double prepare_start_ = -INF;
  double maneuver_start_s_ = 0.0;
  double maneuver_start_d_ = 0.0;
  double cancel_start_ = -INF;
  bool aborting_ = false;
  std::string transition_ = "initialized";

  void set_behavior(Behavior value, const std::string &reason) {
    if (behavior_ != value) transition_ = reason;
    behavior_ = value;
  }

  static double smoothstep(double value) {
    value = clip(value, 0.0, 1.0);
    return value * value * (3.0 - 2.0 * value);
  }

  Candidate build_candidate(Pose ego, double ego_speed, const Track &track,
                            int side, Behavior behavior, double speed_cap) const {
    Candidate best;
    // Search longer physically executable blends instead of relaxing the
    // curvature limit or resetting to the same impossible short return path.
    for (double scale : {1., 1.4, 2., 3.}) {
      best = build_candidate_scaled(ego, ego_speed, track, side, behavior, speed_cap, scale);
      if (best.valid) return best;
      if (best.reason != "curvature_limit" && scale == 1.) return best;
    }
    return best;
  }

  Candidate build_candidate_scaled(Pose ego, double ego_speed, const Track &track,
                            int side, Behavior behavior, double speed_cap, double scale) const {
    Candidate candidate;
    candidate.side = side;
    candidate.speed_cap = clip(speed_cap, 0.70, cruise_speed);
    const Projection start = project(assets_, ego.xy());
    const std::vector<Prediction> prediction =
        predict_track(assets_, track, horizon, step, process_sigma_rate, track_assisted_prediction);
    const double goal_d = behavior == Behavior::CANCEL ? start.d :
        behavior == Behavior::OVERTAKE ? side * pass_offset : 0.0;
    const double transition = behavior == Behavior::REJOIN ? rejoin_length : lane_change_length;
    double s = start.s;
    double speed = std::max(0.0, ego_speed);
    double command = initial_command, acceleration = initial_acceleration;
    candidate.path.push_back(ego);
    candidate.timed.push_back({0.0, s, start.d, speed, ego});
    double previous_yaw = ego.yaw;
    const bool committed_maneuver = behavior == Behavior::OVERTAKE ||
                                    behavior == Behavior::REJOIN ||
                                    behavior == Behavior::CANCEL;
    const double blend_start_s = committed_maneuver ? maneuver_start_s_ : start.s;
    const double current_distance = std::max(
        0.0, signed_progress(start.s, blend_start_s, assets_.arc.back()));
    // A geometrically steerable return can still exceed the MPC's lateral
    // acceleration limit at 2 m/s. Lengthen the blend before solving, rather
    // than resetting physical actuator states when the QP rejects it.
    const double design_speed=std::max({.70,ego_speed,candidate.speed_cap});
    const double curvature_budget=std::min(.60,1.20/(design_speed*design_speed));
    const double remaining_distance=scale*std::max({1.5,transition-current_distance,
        std::sqrt(6.0*std::abs(goal_d-start.d)/curvature_budget)});
    candidate.transition_length=remaining_distance;
    const Point normal(-std::sin(start.pose.yaw),std::cos(start.pose.yaw));
    const Point anchor_error=ego.xy()-(start.pose.xy()+start.d*normal);
    const double initial_slope=std::tan(clip(wrap(ego.yaw-start.pose.yaw),-.65,.65));
    auto spatial_pose=[&](double progress) {
      const double u=clip((progress-start.s)/remaining_distance,0.,1.);
      const double d=start.d+(goal_d-start.d)*smoothstep(u)+
          remaining_distance*initial_slope*(u*u*u-2*u*u+u);
      Pose p=route_pose(assets_,progress);
      p.x-=std::sin(p.yaw)*d; p.y+=std::cos(p.yaw)*d;
      // Projection uses a discretized route tangent. Its reconstruction can
      // miss the measured pose by millimetres: fade that residual continuously
      // instead of inserting a kink at the first near-zero-speed sample.
      p.x+=(1-smoothstep(u))*anchor_error.x();
      p.y+=(1-smoothstep(u))*anchor_error.y();
      return p;
    };

    for (size_t i = 1; i < prediction.size(); ++i) {
      const double requested_acceleration = candidate.speed_cap > speed ? acceleration_max : -deceleration_max;
      const double next_speed = candidate.speed_cap > speed
          ? std::min(candidate.speed_cap, speed + requested_acceleration * step)
          : std::max(candidate.speed_cap, speed + requested_acceleration * step);
      double travel = 0.;
      if (timed_control) {
        double local_cap=candidate.speed_cap;
        if(track.valid && (behavior==Behavior::FOLLOW || behavior==Behavior::PREPARE_OVERTAKE)) {
          // Re-evaluate the gap throughout the forecast, not only at t=0.
          // Holding today's catch-up speed for four seconds predicts a
          // preventable rear-end conflict even when gradual following exists.
          const double lead_s=project(assets_,prediction[i-1].pose.xy()).s;
          const double future_gap=signed_progress(lead_s,s,assets_.arc.back());
          local_cap=std::min(local_cap,clip(track.velocity.norm()+.65*(future_gap-desired_gap),
                                         .70,cruise_speed));
        }
        for(double ahead=0.;ahead<=1.5;ahead+=.25) {
          // Forward-only samples: there is no planned pre-start trajectory.
          const Point a=spatial_pose(s+ahead).xy();
          const Point b=spatial_pose(s+ahead+.08).xy();
          const Point c=spatial_pose(s+ahead+.16).xy();
          const Point ab=b-a,bc=c-b,ac=c-a;
          const double den=ab.norm()*bc.norm()*ac.norm();
          const double k=den>1e-8?2*std::abs(ab.x()*bc.y()-ab.y()*bc.x())/den:0.;
          local_cap=std::min(local_cap,std::sqrt(1.20/std::max(.01,k)));
        }
        if(local_cap<.70) {candidate.reason="curvature_limit";return candidate;}
        // Match MPC command integration, jerk limits and longitudinal lag.
        const int substeps = std::max(1, int(std::round(step / .05)));
        const double h = step / substeps;
        for (int j = 0; j < substeps; ++j) {
          const double desired_a = clip(2.0 * (local_cap - command),
                                         -deceleration_max, acceleration_max);
          acceleration += clip(desired_a - acceleration, -4*h, 4*h);
          command = clip(command + h * acceleration, 0., cruise_speed);
          const double vnew = speed + h / (.15+h) * (command-speed);
          travel += .5*(speed+vnew)*h;
          speed = vnew;
        }
      } else { travel = .5*(speed+next_speed)*step; speed = next_speed; }
      // Anchor at the measured lateral position, retaining enough remaining
      // transition distance to finish without a residual-position snap.
      // Convert travelled path length into reference-route progress. An offset
      // path has different arc length, especially through the Fitz bends.
      const double local_u = clip((s-start.s)/remaining_distance, 0., 1.);
      const double slope0 = std::tan(clip(wrap(ego.yaw-start.pose.yaw), -.65, .65));
      const double deriv = (goal_d-start.d)*6*local_u*(1-local_u)/remaining_distance +
          slope0*(3*local_u*local_u-4*local_u+1);
      const double current_d = start.d + (goal_d-start.d)*smoothstep(local_u) +
          remaining_distance*slope0*(local_u*local_u*local_u-2*local_u*local_u+local_u);
      const double k = wrap(route_pose(assets_, s+.15).yaw-route_pose(assets_, s-.15).yaw)/.30;
      const double path_metric = std::sqrt(std::pow(1-k*current_d,2)+deriv*deriv);
      s += travel / (timed_control ? std::max(.5,path_metric) : 1.);
      // Finish residual tracking error over a finite distance. A completed
      // blend must not snap the very next 5 cm sample onto the offset: that
      // artificial kink caused curvature failures and repeated braking on Fitz.
      const double u = clip(std::max(0.0, s - start.s) / remaining_distance, 0., 1.);
      // Cubic Hermite boundary conditions preserve measured start heading.
      // Restarting every plan with zero lateral slope kinks the path whenever
      // the vehicle is already turning into/out of its offset corridor.
      const double d = start.d + (goal_d - start.d) * smoothstep(u) +
          remaining_distance * initial_slope * (u*u*u - 2*u*u + u);
      Pose reference = spatial_pose(s);
      if (!candidate.path.empty()) {
        const Point delta = reference.xy() - candidate.path.back().xy();
        if (delta.norm() > 1e-8) previous_yaw += wrap(std::atan2(delta.y(), delta.x()) - previous_yaw);
      }
      reference.yaw = previous_yaw;
      candidate.path.push_back(reference);
      candidate.timed.push_back({prediction[i].t, s, d, speed, reference});
    }

    // The first point is the measured position, but its measured yaw is not
    // necessarily the tangent of the newly generated local path. Geometry's
    // curvature is derived from stored path yaw, so recompute every tangent
    // from positions. Otherwise a 5 mm soft-start segment can turn an ordinary
    // 0.08 rad heading error into a fictitious >10 1/m curvature spike.
    for (size_t i = 0; i < candidate.path.size(); ++i) {
      const size_t lo = i ? i - 1 : i;
      const size_t hi = std::min(i + 1, candidate.path.size() - 1);
      const Point delta = candidate.path[hi].xy() - candidate.path[lo].xy();
      if (delta.norm() > 1e-8) candidate.path[i].yaw = std::atan2(delta.y(), delta.x());
      candidate.timed[i].pose.yaw = candidate.path[i].yaw;
    }

    for (size_t i = 1; i + 1 < candidate.path.size(); ++i) {
      const double a = std::atan2(candidate.path[i].y - candidate.path[i - 1].y,
                                  candidate.path[i].x - candidate.path[i - 1].x);
      const double b = std::atan2(candidate.path[i + 1].y - candidate.path[i].y,
                                  candidate.path[i + 1].x - candidate.path[i].x);
      const double ds = std::max(1e-6,
          0.5 * ((candidate.path[i].xy() - candidate.path[i - 1].xy()).norm() +
                 (candidate.path[i + 1].xy() - candidate.path[i].xy()).norm()));
      candidate.maximum_curvature = std::max(candidate.maximum_curvature,
                                              std::abs(wrap(b - a)) / ds);
    }

    // Both vehicles move in the same direction. Inter-sample collision growth
    // is governed by closing speed, not the sum of their speeds.
    const double relative_speed = std::abs(ego_speed - track.velocity.norm());
    for (size_t i = 0; i < candidate.timed.size(); ++i) {
      const Pose &pose = candidate.timed[i].pose;
      const Polygon ego_body = rectangle(pose, vehicle_length, vehicle_width);
      for (const Point &corner : ego_body)
        candidate.minimum_map_clearance = std::min(candidate.minimum_map_clearance,
                                                    assets_.clearance(corner));
      candidate.minimum_map_clearance = std::min(candidate.minimum_map_clearance,
                                                  assets_.clearance(pose.xy()));
      if (candidate.minimum_map_clearance < map_margin) {
        candidate.reason = "map_or_static_footprint";
        return candidate;
      }

      if (!track.valid) continue;

      const Prediction &opponent = prediction[std::min(i, prediction.size() - 1)];
      const double sampling_margin = 0.5 * relative_speed * step;
      const double uncertainty = std::min(0.30, uncertainty_sigma * opponent.sigma);
      const double inflation = body_margin + sampling_margin + uncertainty;
      const Polygon opponent_body = rectangle(opponent.pose,
          track.length + 2.0 * inflation, track.width + 2.0 * inflation);
      const double clearance = polygon_distance(ego_body, opponent_body);
      candidate.minimum_body_clearance = std::min(candidate.minimum_body_clearance,
                                                   clearance);
      if (polygons_overlap(ego_body, opponent_body)) {
        candidate.reason = "time_aligned_body_conflict";
        return candidate;
      }
    }

    const double steering_curvature_limit = std::tan(0.269) / 0.25;
    if (candidate.maximum_curvature > steering_curvature_limit + 0.05) {
      candidate.reason = "curvature_limit";
      return candidate;
    }
    candidate.valid = candidate.path.size() >= 10;
    candidate.reason = candidate.valid ? "ok" : "trajectory_too_short";
    return candidate;
  }
};

} // namespace takeover
