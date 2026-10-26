#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
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

// Admission only: never stop or suppress tracking during an existing pass.
// Fresh positions alone do not establish an opponent's velocity at startup
// or reacquisition. Require distinct observations with stable velocity before
// selecting a passing side. Uses estimates only, not opponent truth.
struct OvertakeObservationGate {
  double since=-INF, stamp=-INF;
  double variance=0.;
  int id=-1, samples=0;
  Point velocity=Point::Zero();
  bool update(const Track &track,double now) {
    if(!track.valid || !track.measured || now-track.observation_stamp>.25 ||
       !track.velocity.allFinite() || !std::isfinite(track.velocity_variance) ||
       track.velocity_variance<0.) {
      samples=0;since=-INF;stamp=-INF;return false;
    }
    if(track.id!=id || track.observation_stamp<stamp ||
       track.observation_stamp-stamp>.45 || (track.velocity-velocity).norm()>.20 ||
       std::abs(track.velocity_variance-variance)>.20*std::max(.05,variance)) {
      since=now;samples=0;
    }
    if(track.observation_stamp>stamp)++samples;
    stamp=track.observation_stamp;id=track.id;velocity=track.velocity;
    variance=track.velocity_variance;
    return samples>=3 && now-since>=.60;
  }
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
  // Hermite interpolation preserves route tangent continuity at waypoints.
  // Linear position interpolation made a millimetre soft-start step crossing
  // a knot appear to have arbitrarily large curvature, preventing recovery.
  const auto &a=assets.route[lo], &b=assets.route[hi];
  const Point m0=span*Point(std::cos(a.yaw),std::sin(a.yaw));
  const Point m1=span*Point(std::cos(b.yaw),std::sin(b.yaw));
  const double u2=u*u,u3=u2*u;
  const Point xy=(2*u3-3*u2+1)*a.xy()+(u3-2*u2+u)*m0+
      (-2*u3+3*u2)*b.xy()+(u3-u2)*m1;
  const Point tangent=(6*u2-6*u)*a.xy()+(3*u2-4*u+1)*m0+
      (-6*u2+6*u)*b.xy()+(3*u2-2*u)*m1;
  return {
      xy.x(), xy.y(), a.yaw+wrap(std::atan2(tangent.y(),tangent.x())-a.yaw)};
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
  double target_offset = 0.;
  int evaluated_candidates = 0;
  double minimum_body_clearance = INF;
  double minimum_map_clearance = INF;
  double maximum_curvature = 0.0;
  Path path;
  std::vector<TimedState> timed;
};

inline bool return_clearance_reserved(double inflated_body_clearance) {
  return inflated_body_clearance >= .10;
}

inline bool return_alignment_before(const Assets &assets, const Candidate &candidate,
                                    double deadline) {
  for(const auto &p:candidate.timed)
    if(p.t<=deadline && std::abs(p.d)<.10 &&
        std::abs(wrap(p.pose.yaw-route_pose(assets,p.s).yaw))<.15) return true;
  return false;
}

inline double candidate_bottleneck_margin(const Candidate &candidate, double map_margin) {
  if (!candidate.valid) return -INF;
  // Rank by the more restrictive clearance, after mandatory inflation/margins.
  // Wall-only ranking selected corridors almost touching the predicted lead
  // envelope even when the other passing side had substantially more room.
  return std::min(candidate.minimum_map_clearance - map_margin,
                  candidate.minimum_body_clearance);
}

inline double adaptive_candidate_score(const Candidate &candidate, double map_margin) {
  if (!candidate.valid || candidate.timed.empty()) return -INF;
  // Use the SAME preference for offset search and left/right entry selection.
  // Once adequate clearance is available, reward actual route progress over
  // the common time horizon; a wide outer bend can otherwise win despite
  // making too little progress to pass/rejoin before observation expiry.
  // These are ranking weights only: every hard footprint/map check precedes it.
  const double progress = candidate.timed.back().s - candidate.timed.front().s;
  return std::min(.15, candidate_bottleneck_margin(candidate, map_margin))
      + .04 * progress - .025 * std::abs(candidate.target_offset)
      - .01 * candidate.maximum_curvature;
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
  bool adaptive_offset = false; // opt-in; retain the frozen fixed-offset baseline
  // Optional upstream SQP lateral optimizer. Other behaviors, the predictor,
  // time parameterization, collision checks and MPC remain ours, not upstream.
  std::function<Json::Value(const Json::Value &)> sqp_optimizer;
  mutable Json::Value sqp_geometry;
  double lane_change_length = 2.6;
  double rejoin_length = 2.8;
  double map_margin = 0.05;
  double body_margin = 0.08;
  double uncertainty_sigma = 2.0;
  double process_sigma_rate = 0.08;
  double acceleration_max = 1.0;
  double deceleration_max = 0.8;
  double wheelbase = .25;
  double maximum_steering = .269;
  double acceleration_response = .15, braking_response = .15;
  bool track_assisted_prediction = true;
  bool timed_control = true;
  bool require_target = true; // false only for explicitly empty simulation scenes
  int allowed_pass_side = 0; // 0: automatic; +/-1: restrict candidates for regression
  double initial_command = 0.;
  double initial_acceleration = 0.;
  double initial_curvature = std::numeric_limits<double>::quiet_NaN();
  std::function<bool(const Candidate &)> return_feasibility;
  std::function<bool(const Candidate &)> overtake_feasibility;

  double following_gap(double ego_speed,double lead_speed) const {
    if(!adaptive_offset || cruise_speed<=2.)return desired_gap;
    const double closing=std::max(0.,ego_speed-lead_speed);
    // High-speed waiting must leave room for longitudinal response and for
    // braking relative to a moving lead, not keep the low-speed fixed gap.
    return desired_gap+.45*lead_speed+.40*closing+
        closing*closing/(2*std::max(.1,deceleration_max));
  }

  Decision plan(Pose ego, double ego_speed, const Track &track, double now) {
    const double begin = frenet::mono();
    Decision decision;
    decision.behavior = behavior_;
    decision.committed_side = committed_side_;
    decision.overtakes_completed = overtakes_completed_;
    decision.target_age = track.valid ? std::max(0.0, now - track.observation_stamp) : INF;
    const bool observation_ready=!sqp_optimizer || observation_gate_.update(track,now);
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
        (decision.target_gap < -1.20 || (aborting_ && decision.target_gap>7.))) {
      const bool passed=decision.target_gap < -1.20;
      if(passed)++overtakes_completed_;
      committed_side_=0;prepare_start_=-INF;aborting_=false;
      set_behavior(Behavior::CRUISE,passed?"reference_route_rejoined":"abort_rejoined_lead_far_ahead");
    }

    // After a completed pass the slower vehicle is behind the forward LiDAR
    // by construction.  Treating that expected loss as an emergency stop
    // leaves the ego stationary in the lead vehicle's lane and creates a
    // rear-end collision.  Continue route cruise; a newly observed target
    // ahead will re-enter the ordinary FOLLOW logic below.
    const bool post_pass_cruise = overtakes_completed_ > 0 &&
                                  behavior_ == Behavior::CRUISE;
    if (!decision.target_fresh && !post_pass_cruise && require_target) {
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
      cap = clip(lead_speed + 0.65 * (gap - following_gap(ego_speed,lead_speed)), 0.70, cruise_speed);
    }

    if (behavior_ == Behavior::PREPARE_OVERTAKE && !observation_ready)
      transition_="waiting_for_stable_target_motion";
    if (behavior_ == Behavior::PREPARE_OVERTAKE && now - prepare_start_ >= 0.40 &&
        observation_ready) {
      maneuver_start_s_ = ego_projection.s;
      maneuver_start_d_ = ego_projection.d;
      Candidate left = build_candidate(ego, ego_speed, track, +1, Behavior::OVERTAKE,
                                       std::max(0.70, lead_speed + 0.05));
      Candidate right = build_candidate(ego, ego_speed, track, -1, Behavior::OVERTAKE,
                                        std::max(0.70, lead_speed + 0.05));
      if (allowed_pass_side < 0) left.valid = false;
      if (allowed_pass_side > 0) right.valid = false;
      if (left.valid || right.valid) {
        const auto entry_score = [&](const Candidate &c) {
          return adaptive_offset ? adaptive_candidate_score(c, map_margin)
                                 : candidate_bottleneck_margin(c, map_margin);
        };
        Candidate selected = !right.valid || (left.valid &&
            entry_score(left) >= entry_score(right)) ? left : right;
        committed_side_ = selected.side;
        committed_offset_ = selected.target_offset;
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
      // Separation is relative to the predicted target, not the raceline.
      const double offset_progress = adaptive_offset
          ? committed_side_ * (current_projection.d - target_projection.d)
          : std::abs(current_projection.d);
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
      // This only triggers a return CANDIDATE. Do not require an axis-aligned
      // worst-case inflated gap before even considering a gradual diagonal
      // return: the full time-aligned inflated-body check below, 0.10 m return
      // reserve and non-actuating MPC preview already validate the maneuver.
      const double return_gap = .5 * (vehicle_length + track.length) + body_margin;
      if (gap < -return_gap) {
        const double old_start_s = maneuver_start_s_;
        const double old_start_d = maneuver_start_d_;
        maneuver_start_s_ = current_projection.s;
        maneuver_start_d_ = current_projection.d;
        const auto returning = build_candidate(ego, ego_speed, track, committed_side_,
                                               Behavior::REJOIN, cruise_speed);
        if (returning.valid && (!adaptive_offset ||
            return_clearance_reserved(returning.minimum_body_clearance))) {
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
      // A recovery is not a new catch-up maneuver. Keep its forecast at or
      // below lead speed until aligned, instead of accelerating into a bend
      // simply because aborting created a larger longitudinal gap.
      if (adaptive_offset && aborting_ && lead_ahead)
        cap = std::min(cap, std::max(.70, lead_speed));
      if (std::abs(current_projection.d) < 0.10 &&
          std::abs(wrap(ego.yaw-current_projection.pose.yaw)) < .15 &&
          (gap < -1.20 || aborting_)) {
        const bool passed = gap < -1.20;
        if (passed) ++overtakes_completed_;
        committed_side_ = 0;
        prepare_start_ = -INF;
        aborting_ = false;
        set_behavior(passed || !lead_ahead ? Behavior::CRUISE : Behavior::FOLLOW,
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
    if (adaptive_offset && candidate.valid && behavior_ == Behavior::OVERTAKE)
      committed_offset_ = candidate.target_offset;
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
  double committed_offset_ = 0.;
  int overtakes_completed_ = 0;
  double prepare_start_ = -INF;
  double maneuver_start_s_ = 0.0;
  double maneuver_start_d_ = 0.0;
  double cancel_start_ = -INF;
  bool aborting_ = false;
  std::string transition_ = "initialized";
  struct SqpMemory { double s=0., length=0.; std::vector<double> d; };
  mutable std::array<SqpMemory,2> sqp_memory_;
  OvertakeObservationGate observation_gate_;

  void set_behavior(Behavior value, const std::string &reason) {
    if (behavior_ != value) transition_ = reason;
    behavior_ = value;
  }

  static double smoothstep(double value) {
    value = clip(value, 0.0, 1.0);
    return value * value * (3.0 - 2.0 * value);
  }

  // A slow entry's four-second forecast sees only a few metres. Before
  // committing, also check the known STATIC map over the estimated passing
  // distance, so a locally wide side is not selected just before it narrows.
  // This conservative constant-offset corridor is not a longer target
  // prediction and never extends the observation timeout.
  bool entry_corridor_clear(Pose ego, const Track &track, double offset,
                            double reserve=0.) const {
    const double start = project(assets_, ego.xy()).s;
    const double gap = track.valid ? std::max(0., signed_progress(
        project(assets_, track.position).s, start, assets_.arc.back())) : desired_gap;
    double progress_speed=cruise_speed;
    const auto target_projection=project(assets_,track.position);
    const double target_curvature=wrap(route_pose(assets_,target_projection.s+.10).yaw-
        route_pose(assets_,target_projection.s-.10).yaw)/.20;
    // A maximum-speed distance estimate alone is optimistic on an outer bend:
    // retiming slows the ego, while the lead still advances. Tighten the
    // admission budget using the actual offset's curvature/arc-length metric.
    // This is STATIC map/capability lookahead, not a longer trusted target
    // prediction or permission to extend observation freshness.
    const bool speed_budget=sqp_optimizer && cruise_speed>2.;
    const double lead_progress_speed=track.velocity.norm()/(speed_budget?
        std::max(.5,1-target_curvature*target_projection.d):1.);
    for(int iteration=0;iteration<(speed_budget?5:1);++iteration) {
      const double advantage=progress_speed-lead_progress_speed;
      if(speed_budget && advantage<.20)return false;
      const double distance=lane_change_length+rejoin_length+
          progress_speed*(gap+1.2)/std::max(.18,advantage);
      if(speed_budget && distance>30.)return false;
      double available_progress_speed=progress_speed;
      for (double ds=0.; ds<=distance+.05; ds+=.10) {
        Pose p = route_pose(assets_, start+ds);
        p.x -= std::sin(p.yaw)*offset;
        p.y += std::cos(p.yaw)*offset;
        Pose before=route_pose(assets_,start+ds-.10),after=route_pose(assets_,start+ds+.10);
        before.x-=std::sin(before.yaw)*offset;before.y+=std::cos(before.yaw)*offset;
        after.x-=std::sin(after.yaw)*offset;after.y+=std::cos(after.yaw)*offset;
        const double chord=(after.xy()-before.xy()).norm();
        const double curvature=std::abs(wrap(after.yaw-before.yaw))/std::max(.001,chord);
        if(chord<.02 || curvature>std::tan(maximum_steering)/wheelbase) return false;
        if(speed_budget) {
          const double metric=chord/.20;
          const double speed=std::min(cruise_speed,std::sqrt(1.20/std::max(.01,curvature)));
          // Ten percent progress reserve for entry, response and tracking;
          // the complete timed/body checks and MPC still decide execution.
          available_progress_speed=std::min(available_progress_speed,.90*speed/metric);
        }
        if (assets_.clearance(p.xy()) < .25+reserve) return false;
        for (const auto &corner : rectangle(p, vehicle_length, vehicle_width))
          if (assets_.clearance(corner) < map_margin+reserve) return false;
      }
      if(!speed_budget || progress_speed-available_progress_speed<.02)return true;
      progress_speed=available_progress_speed;
    }
    return false; // Unsettled budget is not an admission certificate.
  }

  Candidate build_candidate(Pose ego, double ego_speed, const Track &track,
                            int side, Behavior behavior, double speed_cap) const {
    if (sqp_optimizer && behavior == Behavior::OVERTAKE && side != 0) {
      const double target_d = track.valid ? project(assets_, track.position).d : 0.;
      Candidate c;
      int evaluated=0;
      for(double scale:{1.,1.4,2.}) {
        c=build_candidate_scaled(ego,ego_speed,track,side,behavior,speed_cap,scale,
            side*clip(side*target_d+pass_offset,.50,1.25));
        ++evaluated;
        if(c.valid)break;
      }
      c.evaluated_candidates = evaluated;
      sqp_geometry["reason"]=c.reason;
      sqp_geometry["maximum_discrete_curvature"]=c.maximum_curvature;
      if(c.valid && behavior_ == Behavior::PREPARE_OVERTAKE &&
         !entry_corridor_clear(ego, track, c.target_offset,.10)) {
        c.valid=false;c.reason="sqp_entry_corridor_blocked";
      }
      return c; // Never silently fall back to the finite-offset baseline.
    }
    if (adaptive_offset && behavior == Behavior::OVERTAKE && side != 0) {
      const double target_d = track.valid ? project(assets_, track.position).d : 0.;
      std::vector<double> offsets{side*pass_offset, side*.62, side*1.02, side*1.22,
          side*clip(side*target_d+pass_offset, .50, 1.25)};
      if (committed_side_ == side && side*committed_offset_ > 0.)
        offsets.push_back(committed_offset_);
      Candidate selected, previous;
      selected.reason = "no_safe_same_side_offset";
      double best_score = -INF, previous_score = -INF;
      int evaluated = 0;
      for (double offset : offsets) {
        // A low-speed entry can be collision-free while too narrow to ever
        // pass. Require room for the existing worst-case inflated body at
        // cruise closing speed, not merely today's still-distant target.
        const double passing_separation = .5*(vehicle_width+track.width) +
            body_margin + .30 + .5*std::abs(cruise_speed-track.velocity.norm())*step + .04;
        if (track.valid && side*(offset-target_d) < passing_separation) continue;
        if (behavior_ == Behavior::PREPARE_OVERTAKE &&
            !entry_corridor_clear(ego, track, offset)) {
          if (!selected.valid) selected.reason = "entry_corridor_blocked";
          continue;
        }
        for (double scale : {1., 1.4, 2., 3.}) {
          auto c = build_candidate_scaled(ego, ego_speed, track, side, behavior,
                                           speed_cap, scale, offset);
          ++evaluated;
          if (!c.valid) {
            if (!selected.valid) selected = c;
            continue;
          }
          // Extra clearance beyond 15 cm is not rewarded indefinitely at the
          // expense of an unnecessarily large, hard-to-track lane change.
          double score = adaptive_candidate_score(c, map_margin);
          if (committed_side_ == side)
            score -= .10*std::abs(offset-committed_offset_);
          if (score > best_score) { selected = c; best_score = score; }
          if (committed_side_ == side && std::abs(offset-committed_offset_) < 1e-6) {
            previous = c; previous_score = score;
          }
          break;
        }
      }
      // Hysteresis never keeps an infeasible candidate. Side remains locked.
      if (previous.valid && previous_score >= best_score-.02) selected = previous;
      selected.evaluated_candidates = evaluated;
      return selected;
    }
    Candidate best;
    // Search longer physically executable blends instead of relaxing the
    // curvature limit or resetting to the same impossible short return path.
    for (double scale : {1., 1.4, 2., 3.}) {
      best = build_candidate_scaled(ego, ego_speed, track, side, behavior, speed_cap, scale, side*pass_offset);
      if(best.valid && adaptive_offset && behavior==Behavior::REJOIN &&
          return_feasibility && !return_feasibility(best)) {
        best.valid=false;
        best.reason="mpc_return_preview_rejected";
      }
      if (best.valid) return best;
      if (best.reason != "curvature_limit" && best.reason != "mpc_return_preview_rejected" &&
          scale == 1.) return best;
    }
    return best;
  }

  Candidate build_candidate_scaled(Pose ego, double ego_speed, const Track &track,
                            int side, Behavior behavior, double speed_cap, double scale,
                            double pass_target) const {
    Candidate candidate;
    candidate.side = side;
    candidate.speed_cap = clip(speed_cap, 0.70, cruise_speed);
    const Projection start = project(assets_, ego.xy());
    const std::vector<Prediction> prediction =
        predict_track(assets_, track, horizon, step, process_sigma_rate, track_assisted_prediction);
    const double goal_d = behavior == Behavior::CANCEL ? start.d :
        behavior == Behavior::OVERTAKE ? pass_target : 0.0;
    candidate.target_offset = goal_d;
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
    const double design_speed=adaptive_offset ? std::max(.70,ego_speed) :
        std::max({.70,ego_speed,candidate.speed_cap});
    const double curvature_budget=std::min(.60,1.20/(design_speed*design_speed));
    const bool use_sqp=bool(sqp_optimizer) && behavior==Behavior::OVERTAKE && side!=0;
    // A waypoint on the controller's hard map limit leaves no room for
    // tracking error. Tighten SQP corridors by 10 cm; never relax the final
    // live MPC/map checks. The original candidate backend remains unchanged.
    const double map_reserve=use_sqp?.10:0.;
    const double remaining_distance=scale*std::max({use_sqp?2.8:1.5,(use_sqp?4.5:transition)-current_distance,
        std::sqrt(6.0*std::abs(goal_d-start.d)/curvature_budget)});
    candidate.transition_length=remaining_distance;
    const Pose start_reference=route_pose(assets_,start.s);
    const Point normal(-std::sin(start_reference.yaw),std::cos(start_reference.yaw));
    const Point anchor_error=ego.xy()-(start_reference.xy()+start.d*normal);
    const auto route_curvature = [&](double at) {
      return wrap(route_pose(assets_,at+.15).yaw-route_pose(assets_,at-.15).yaw)/.30;
    };
    const double k0=route_curvature(start.s);
    const double metric0=std::max(.5,1-k0*start.d);
    const bool curvature_continuous=adaptive_offset && (behavior==Behavior::REJOIN || use_sqp ||
        (sqp_optimizer && behavior==Behavior::CANCEL));
    const double initial_slope=(curvature_continuous?metric0:1.)*
        std::tan(clip(wrap(ego.yaw-start_reference.yaw),-.65,.65));
    const double kprime=(route_curvature(start.s+.15)-route_curvature(start.s-.15))/.30;
    const double actual_k=std::isfinite(initial_curvature)?initial_curvature:k0/metric0;
    const double second_derivative=(actual_k*std::pow(metric0*metric0+initial_slope*initial_slope,1.5)
        -k0*(metric0*metric0+2*initial_slope*initial_slope)-kprime*start.d*initial_slope)/metric0;
    // Adaptive returns preserve current curvature as well as position/heading.
    // A cubic return switched lateral acceleration instantly on a Fitz bend,
    // although its centerline was collision-free. The quintic keeps the
    // unchanged MPC's steering lag/rate limits executable at that transition.
    std::function<std::array<double,2>(double)> lateral_state=[&](double u) {
      const double L=remaining_distance, delta=goal_d-start.d;
      if(!curvature_continuous) return std::array<double,2>{
          start.d+delta*smoothstep(u)+L*initial_slope*(u*u*u-2*u*u+u),
          delta*6*u*(1-u)/L+initial_slope*(3*u*u-4*u+1)};
      const double u2=u*u,u3=u2*u,u4=u3*u,u5=u4*u;
      return std::array<double,2>{
          start.d+delta*(10*u3-15*u4+6*u5)+L*initial_slope*(u-6*u3+8*u4-3*u5)
              +.5*L*L*second_derivative*(u2-3*u3+3*u4-u5),
          delta*(30*u2-60*u3+30*u4)/L+initial_slope*(1-18*u2+32*u3-15*u4)
              +.5*L*second_derivative*(2*u-9*u2+12*u3-5*u4)};
    };
    std::vector<std::array<double,4>> sqp_coefficients;
    if(use_sqp) {
      sqp_geometry=Json::Value(Json::objectValue);
      sqp_geometry["start_d"]=start.d;sqp_geometry["initial_slope"]=initial_slope;
      sqp_geometry["initial_second_derivative"]=second_derivative;
      sqp_geometry["length"]=remaining_distance;sqp_geometry["route_curvature"]=k0;
      Json::Value request;
      const int n=21;
      const double ds=remaining_distance/(n-1);
      const auto &memory=sqp_memory_[side>0?1:0];
      const double advance=signed_progress(start.s,memory.s,assets_.arc.back());
      const bool warm=committed_side_==side && behavior_==Behavior::OVERTAKE &&
          !memory.d.empty() && advance>=-.05 && advance<3.;
      const auto previous_d=[&](double x) {
        const double index=clip((x+std::max(0.,advance))/memory.length,0.,1.)*(memory.d.size()-1);
        const size_t lo=std::min(memory.d.size()-1,size_t(index));
        const size_t hi=std::min(memory.d.size()-1,lo+1);
        return memory.d[lo]+(index-lo)*(memory.d[hi]-memory.d[lo]);
      };
      request["initial_slope"]=initial_slope;
      request["initial_second_derivative"]=second_derivative;
      // Geometry must be steerable, not drivable everywhere at cruise speed.
      // The timed trajectory below reduces speed for curvature and the MPC
      // independently enforces lateral acceleration. Using 1.2/v_cruise^2
      // here made even a measured bend state infeasible before retiming.
      request["max_curvature"]=std::max(curvature_budget,
          std::abs(k0+second_derivative))+.03;
      for(const char *key:{"s","initial","lateral_reference","bounds","curvature","obstacle_indices",
                            "obstacle_d","obstacle_clearance"})
        request[key]=Json::Value(Json::arrayValue);
      for(int i=0;i<n;++i) {
        const double x=i*ds;
        const Pose ref=route_pose(assets_,start.s+x);
        const auto map_clear=[&](double d) {
          Pose p=ref;p.x-=std::sin(ref.yaw)*d;p.y+=std::cos(ref.yaw)*d;
          if(assets_.clearance(p.xy())<.25+map_reserve)return false;
          for(const auto &corner:rectangle(p,vehicle_length,vehicle_width))
            if(assets_.clearance(corner)<map_margin+map_reserve)return false;
          return true;
        };
        double lo=0.,hi=0.;
        if(!map_clear(0.)){candidate.reason="sqp_reference_corridor_blocked";return candidate;}
        for(double d=.025;d<=1.35;d+=.025){if(!map_clear(d))break;hi=d;}
        for(double d=-.025;d>=-1.35;d-=.025){if(!map_clear(d))break;lo=d;}
        const double t=std::min(horizon,x/std::max(.70,speed_cap));
        const auto &pred=prediction[std::min(prediction.size()-1,size_t(t/step))];
        const auto lead=project(assets_,pred.pose.xy());
        const double sampling=std::max(.5*std::abs(cruise_speed-track.velocity.norm())*step,
            .5*(cruise_speed+track.velocity.norm())*.05);
        const double separation=.5*(vehicle_width+track.width)+body_margin+.30+sampling+.10;
        const double longitudinal=std::abs(signed_progress(lead.s,start.s+x,assets_.arc.back()));
        // The endpoint is a passing corridor, not a premature raceline return.
        // Its d remains an optimization variable within map and body bounds.
        const bool passing_zone=i>=16 || (i>=3 && longitudinal<.9);
        if(track.valid && passing_zone) {
          if(side>0)lo=std::max(lo,lead.d+separation);
          else hi=std::min(hi,lead.d-separation);
          request["obstacle_indices"].append(i);
          request["obstacle_d"].append(lead.d);
          request["obstacle_clearance"].append(separation);
        }
        if(lo>hi){candidate.reason="sqp_empty_lateral_bounds";return candidate;}
        Json::Value bound(Json::arrayValue);bound.append(lo);bound.append(hi);
        request["bounds"].append(bound);request["s"].append(x);
        request["initial"].append(warm && i>0?previous_d(x):lateral_state(double(i)/(n-1))[0]);
        // Soft progress/continuity cost, not a hard tube: a shifted opponent
        // may require a different safe shape. Boundary/body constraints win.
        const double seed_d=lateral_state(double(i)/(n-1))[0];
        // Keep a stable maneuver goal as well as rolling continuity. Merely
        // tracking the last optimum lets the raceline cost shrink the passing
        // offset on every cycle until execution clearance disappears.
        request["lateral_reference"].append(warm?
            side*std::max(side*previous_d(x),side*seed_d):seed_d);
        // Our d is positive left; upstream's k_global-d'' convention needs
        // the opposite route-curvature sign. Absolute curvature is checked
        // again on Cartesian samples, not trusted from this approximation.
        if(i<n-2) {
          const double route_k=route_curvature(start.s+x+ds);
          request["curvature"].append(-route_k);
          request["max_curvature"]=std::max(request["max_curvature"].asDouble(),
              std::abs(route_k)+.05);
        }
      }
      request["max_curvature"]=std::min(std::tan(maximum_steering)/wheelbase,request["max_curvature"].asDouble());
      const auto solved=sqp_optimizer(request);
      if(!solved["ok"].asBool() || solved["coefficients"].size()!=unsigned(n-1)) {
        candidate.reason="upstream_sqp_rejected:"+solved["message"].asString();return candidate;
      }
      for(const auto &row:solved["coefficients"]) {
        if(row.size()!=4){candidate.reason="sqp_invalid_coefficients";return candidate;}
        std::array<double,4> coeff;
        for(int j=0;j<4;++j){coeff[j]=row[j].asDouble();
          if(!std::isfinite(coeff[j])){candidate.reason="sqp_nonfinite_coefficients";return candidate;}}
        sqp_coefficients.push_back(coeff);
      }
      candidate.target_offset=solved["d"][n-1].asDouble();
      auto cubic=[&,ds,n](double x) {
        x=clip(x,0.,remaining_distance);
        const int index=std::min(n-2,int(x/ds));
        const double z=x-index*ds;
        const auto &c=sqp_coefficients[index];
        return std::array<double,3>{((c[0]*z+c[1])*z+c[2])*z+c[3],
                                   (3*c[0]*z+2*c[1])*z+c[2],6*c[0]*z+2*c[1]};
      };
      lateral_state=[&,cubic](double u) {
        const double x=clip(u,0.,1.)*remaining_distance;
        const auto v=cubic(x);return std::array<double,2>{v[0],v[1]};
      };
    }
    auto spatial_pose=[&](double progress) {
      const double u=clip((progress-start.s)/remaining_distance,0.,1.);
      const double d=lateral_state(u)[0];
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
          local_cap=std::min(local_cap,clip(track.velocity.norm()+.65*(future_gap-
                                         following_gap(speed,track.velocity.norm())),
                                         .70,cruise_speed));
        }
        const double brake_speed=std::max(speed,command);
        const double braking_preview=cruise_speed>2.0 ?
            std::max(1.5,brake_speed*brake_speed/(2*.60)+brake_speed*.4+1.) : 1.5;
        for(double ahead=0.;ahead<=braking_preview;ahead+=.25) {
          // Forward-only samples: there is no planned pre-start trajectory.
          const Point a=spatial_pose(s+ahead).xy();
          const Point b=spatial_pose(s+ahead+.08).xy();
          const Point c=spatial_pose(s+ahead+.16).xy();
          const Point ab=b-a,bc=c-b,ac=c-a;
          const double den=ab.norm()*bc.norm()*ac.norm();
          const double k=den>1e-8?2*std::abs(ab.x()*bc.y()-ab.y()*bc.x())/den:0.;
          if(use_sqp && k>sqp_geometry.get("maximum_lookahead_curvature",0.).asDouble()) {
            sqp_geometry["maximum_lookahead_curvature"]=k;
            sqp_geometry["curvature_at_relative_s"]=s+ahead-start.s;
          }
          const double bend_speed_squared=1.20/std::max(.01,k);
          // A 3 m/s approach needs several metres to brake. Include command
          // jerk/longitudinal response before spending braking distance.
          // Original <=2 m/s timing remains the frozen regression profile.
          const double braking_distance=cruise_speed>2.0 ?
              std::max(0.,ahead-brake_speed*.4) : 0.;
          local_cap=std::min(local_cap,std::sqrt(bend_speed_squared+2*.60*braking_distance));
        }
        if(local_cap<.70) {candidate.reason="curvature_limit";return candidate;}
        // Match MPC command integration, jerk limits and longitudinal lag.
        const int substeps = std::max(1, int(std::round(step / .05)));
        const double h = step / substeps;
        for (int j = 0; j < substeps; ++j) {
          const double desired_a = clip(2.0 * (local_cap - command),
                                         -deceleration_max, acceleration_max);
          // MPC permits 4 m/s^3 only in its first wall-time step, but 3 in
          // subsequent 50 ms stages (0.15 m/s^2). Use the common stricter bound.
          acceleration += clip(desired_a - acceleration, -3*h, 3*h);
          command = clip(command + h * acceleration, 0., cruise_speed);
          const double response=command>=speed?acceleration_response:braking_response;
          const double vnew = speed + h / (response+h) * (command-speed);
          travel += .5*(speed+vnew)*h;
          speed = vnew;
        }
      } else { travel = .5*(speed+next_speed)*step; speed = next_speed; }
      // Anchor at the measured lateral position, retaining enough remaining
      // transition distance to finish without a residual-position snap.
      // Convert travelled path length into reference-route progress. An offset
      // path has different arc length, especially through the Fitz bends.
      const double local_u = clip((s-start.s)/remaining_distance, 0., 1.);
      const auto local_lateral=lateral_state(local_u);
      const double deriv=local_lateral[1], current_d=local_lateral[0];
      const double k = wrap(route_pose(assets_, s+.15).yaw-route_pose(assets_, s-.15).yaw)/.30;
      const double path_metric = std::sqrt(std::pow(1-k*current_d,2)+deriv*deriv);
      s += travel / (timed_control ? std::max(.5,path_metric) : 1.);
      // Finish residual tracking error over a finite distance. A completed
      // blend must not snap the very next 5 cm sample onto the offset: that
      // artificial kink caused curvature failures and repeated braking on Fitz.
      const double u = clip(std::max(0.0, s - start.s) / remaining_distance, 0., 1.);
      const double d=lateral_state(u)[0];
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
      // Match the unchanged v8 MPC swept center clearance, including between
      // reference samples. Otherwise a nominally valid wide candidate can
      // send the controller into an irrecoverable wall-side stop.
      if (adaptive_offset) {
        const Pose &previous = candidate.timed[i ? i-1 : i].pose;
        const int n = std::max(1, int(std::ceil((pose.xy()-previous.xy()).norm()/.02)));
        for (int j=0; j<=n; ++j) {
          const Point p = previous.xy()+(pose.xy()-previous.xy())*(double(j)/n);
          if (assets_.clearance(p) < .25+map_reserve) {
            candidate.reason = "mpc_center_map_clearance";
            return candidate;
          }
        }
      }
      const Polygon ego_body = rectangle(pose, vehicle_length, vehicle_width);
      for (const Point &corner : ego_body)
        candidate.minimum_map_clearance = std::min(candidate.minimum_map_clearance,
                                                    assets_.clearance(corner));
      candidate.minimum_map_clearance = std::min(candidate.minimum_map_clearance,
                                                  assets_.clearance(pose.xy()));
      if (candidate.minimum_map_clearance < map_margin+map_reserve) {
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

    const double steering_curvature_limit = std::tan(maximum_steering) / wheelbase;
    if (candidate.maximum_curvature > steering_curvature_limit + 0.05) {
      candidate.reason = "curvature_limit";
      return candidate;
    }
    candidate.valid = candidate.path.size() >= 10;
    candidate.reason = candidate.valid ? "ok" : "trajectory_too_short";
    if(candidate.valid && use_sqp && overtake_feasibility && !overtake_feasibility(candidate)) {
      candidate.valid=false;
      candidate.reason="mpc_sqp_preview_rejected";
    }
    if(candidate.valid && use_sqp) {
      auto &memory=sqp_memory_[side>0?1:0];
      memory.s=start.s;memory.length=remaining_distance;memory.d.clear();
      for(int i=0;i<=100;++i)memory.d.push_back(lateral_state(i/100.)[0]);
    }
    return candidate;
  }
};

} // namespace takeover
