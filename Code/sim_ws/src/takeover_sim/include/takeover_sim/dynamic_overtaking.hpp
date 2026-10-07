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
  double minimum_body_clearance = INF;
  double minimum_map_clearance = INF;
  double maximum_curvature = 0.0;
  Path path;
  std::vector<TimedState> timed;
};

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

  Decision plan(Pose ego, double ego_speed, const Track &track, double now) {
    const double begin = frenet::mono();
    Decision decision;
    decision.behavior = behavior_;
    decision.committed_side = committed_side_;
    decision.overtakes_completed = overtakes_completed_;
    decision.target_age = track.valid ? std::max(0.0, now - track.observation_stamp) : INF;
    const bool committed = behavior_ == Behavior::OVERTAKE ||
                           behavior_ == Behavior::REJOIN;
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
      decision.target_prediction = predict_cv(track, horizon, step, process_sigma_rate);
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
    const bool lead_ahead = gap > -0.4 && gap < 7.0;
    const bool slower = lead_speed + 0.18 < cruise_speed;

    if (behavior_ == Behavior::WAIT || behavior_ == Behavior::CANCEL) {
      set_behavior(lead_ahead ? Behavior::FOLLOW : Behavior::CRUISE,
                   "fresh_target_recovered");
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
      if (left.valid || right.valid) {
        Candidate selected = !right.valid || (left.valid &&
            left.minimum_map_clearance >= right.minimum_map_clearance) ? left : right;
        committed_side_ = selected.side;
        set_behavior(Behavior::OVERTAKE, selected.side > 0 ?
                     "left_corridor_committed" : "right_corridor_committed");
      }
    }

    const Projection current_projection = project(assets_, ego.xy());
    if (behavior_ == Behavior::OVERTAKE) {
      const double offset_progress = std::abs(current_projection.d);
      // Build lateral separation at approximately the lead speed.  Closing
      // during the lane-entry transient repeatedly invalidates an otherwise
      // safe pass and produces a speed saw-tooth.  Once footprint separation
      // exists, the same time-aligned check below permits the full pass speed.
      cap = offset_progress < 0.68
          ? std::min(cruise_speed, std::max(0.70, lead_speed + 0.05))
          : cruise_speed;
      if (gap < -1.0) {
        maneuver_start_s_ = current_projection.s;
        maneuver_start_d_ = current_projection.d;
        set_behavior(Behavior::REJOIN, "ego_body_clear_of_lead");
      }
    }
    if (behavior_ == Behavior::REJOIN) {
      cap = cruise_speed;
      if (std::abs(current_projection.d) < 0.10 && gap < -1.20) {
        ++overtakes_completed_;
        committed_side_ = 0;
        prepare_start_ = -INF;
        set_behavior(Behavior::CRUISE, "reference_route_rejoined");
      }
    }

    Candidate candidate = build_candidate(ego, ego_speed, track, committed_side_,
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
      set_behavior(Behavior::CANCEL, "committed_corridor_invalid");
      candidate = build_candidate(ego, ego_speed, track, committed_side_,
                                  Behavior::CANCEL, 0.70);
    }

    decision.behavior = behavior_;
    decision.candidate = candidate;
    decision.committed_side = committed_side_;
    decision.stop = !candidate.valid || behavior_ == Behavior::WAIT ||
                    (lead_speed < 0.25 && gap < desired_gap + 0.35 &&
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
    Candidate candidate;
    candidate.side = side;
    candidate.speed_cap = clip(speed_cap, 0.70, cruise_speed);
    const Projection start = project(assets_, ego.xy());
    const std::vector<Prediction> prediction =
        predict_cv(track, horizon, step, process_sigma_rate);
    const double goal_d = behavior == Behavior::OVERTAKE || behavior == Behavior::CANCEL
        ? side * pass_offset : 0.0;
    const double transition = behavior == Behavior::REJOIN ? rejoin_length : lane_change_length;
    double s = start.s;
    double speed = std::max(0.0, ego_speed);
    candidate.path.push_back(ego);
    candidate.timed.push_back({0.0, s, start.d, speed, ego});
    double previous_yaw = ego.yaw;
    const bool committed_maneuver = behavior == Behavior::OVERTAKE ||
                                    behavior == Behavior::REJOIN ||
                                    behavior == Behavior::CANCEL;
    const double blend_start_s = committed_maneuver ? maneuver_start_s_ : start.s;
    const double current_distance = std::max(
        0.0, signed_progress(start.s, blend_start_s, assets_.arc.back()));
    const double current_blend = smoothstep(
        current_distance / std::max(0.2, transition));

    for (size_t i = 1; i < prediction.size(); ++i) {
      const double requested_acceleration = candidate.speed_cap > speed ? acceleration_max : -deceleration_max;
      const double next_speed = candidate.speed_cap > speed
          ? std::min(candidate.speed_cap, speed + requested_acceleration * step)
          : std::max(candidate.speed_cap, speed + requested_acceleration * step);
      s += 0.5 * (speed + next_speed) * step;
      speed = next_speed;
      const double distance = std::max(
          0.0, signed_progress(s, blend_start_s, assets_.arc.back()));
      const double absolute_blend = smoothstep(
          distance / std::max(0.2, transition));
      // Anchor the receding-horizon path at the measured lateral position,
      // then consume the remaining portion of the original manoeuvre.  This
      // is continuous even when tracking lags, but still reaches the fixed
      // lateral goal at the fixed Frenet end station.
      const double remaining_blend = current_blend < 1.0 - 1e-6
          ? clip((absolute_blend - current_blend) / (1.0 - current_blend), 0.0, 1.0)
          : 1.0;
      const double d = start.d + (goal_d - start.d) * remaining_blend;
      Pose reference = route_pose(assets_, s);
      reference.x -= std::sin(reference.yaw) * d;
      reference.y += std::cos(reference.yaw) * d;
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
