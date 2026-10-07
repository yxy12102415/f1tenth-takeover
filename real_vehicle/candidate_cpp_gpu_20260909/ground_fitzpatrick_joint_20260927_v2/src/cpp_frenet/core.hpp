#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
namespace frenet {
constexpr double INF = std::numeric_limits<double>::infinity();
inline double clip(double x, double a, double b) {
  return std::max(a, std::min(b, x));
}
inline double wrap(double x) { return std::atan2(std::sin(x), std::cos(x)); }
inline double mono() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
using Point = Eigen::Vector2d;
struct Pose {
  double x = 0, y = 0, yaw = 0;
  Point xy() const { return {x, y}; }
};
using Path = std::vector<Pose>;
inline Json::Value json_read(const std::string &name) {
  std::ifstream f(name);
  Json::Value j;
  if (!f || !(f >> j))
    throw std::runtime_error("asset_read_failed");
  return j;
}
inline std::string dump(const Json::Value &j) {
  Json::StreamWriterBuilder b;
  b["indentation"] = "";
  return Json::writeString(b, j);
}
inline Path path_read(const Json::Value &j) {
  Path p;
  for (const auto &r : j) {
    if (r.size() != 3)
      throw std::runtime_error("path_shape");
    Pose v{r[0].asDouble(), r[1].asDouble(), r[2].asDouble()};
    if (!std::isfinite(v.x + v.y + v.yaw))
      throw std::runtime_error("path_nonfinite");
    p.push_back(v);
  }
  return p;
}
inline Json::Value path_json(const Path &p) {
  Json::Value j(Json::arrayValue);
  for (auto v : p) {
    Json::Value r(Json::arrayValue);
    r.append(v.x);
    r.append(v.y);
    r.append(v.yaw);
    j.append(r);
  }
  return j;
}
inline double interp(const std::vector<double> &a, const std::vector<double> &b,
                     double x) {
  if (x <= a.front())
    return b.front();
  if (x >= a.back())
    return b.back();
  size_t h = std::lower_bound(a.begin(), a.end(), x) - a.begin();
  double f = (x - a[h - 1]) / std::max(1e-12, a[h] - a[h - 1]);
  return b[h - 1] + f * (b[h] - b[h - 1]);
}
inline double curvature(const Path &p) {
  double k = 0;
  for (size_t i = 0; i + 10 < p.size(); ++i) {
    Point a = p[i + 5].xy() - p[i].xy(), b = p[i + 10].xy() - p[i].xy();
    double den = a.norm() * b.norm() * (p[i + 10].xy() - p[i + 5].xy()).norm();
    if (den > 1e-9)
      k = std::max(k, 2 * std::abs(a.x() * b.y() - a.y() * b.x()) / den);
  }
  return k;
}
struct Assets {
  Path route;
  bool closed_single_lap = false;
  std::vector<double> ds, arc, speed, grid;
  double res, ox, oy;
  int width, height;
  explicit Assets(const std::string &filename) {
    auto j = json_read(filename);
    route = path_read(j["route"]);
    closed_single_lap = j.get("route_mode","open_stop_at_end").asString()=="closed_single_lap_stop";
    if (route.size() < 40)
      throw std::runtime_error("route_short");
    res = j["resolution"].asDouble();
    ox = j["origin"][0].asDouble();
    oy = j["origin"][1].asDouble();
    width = j["width"].asInt();
    height = j["height"].asInt();
    if (!std::isfinite(res + ox + oy) || res <= 0 || width <= 0 ||
        height <= 0 || width > 10000 || height > 10000)
      throw std::runtime_error("map_invalid");
    for (auto v : j["clearance"]) {
      double value = v.asDouble();
      if (!std::isfinite(value) || value < 0)
        throw std::runtime_error("map_clearance_invalid");
      grid.push_back(value);
    }
    if (grid.size() != size_t(width * height))
      throw std::runtime_error("map_shape");
    arc.push_back(0);
    for (size_t i = 1; i < route.size(); ++i) {
      ds.push_back((route[i].xy() - route[i - 1].xy()).norm());
      if (ds.back() < 1e-6)
        throw std::runtime_error("route_duplicate");
      arc.push_back(arc.back() + ds.back());
    }
    ds.push_back(0);
    for (auto v : j["speed"])
      speed.push_back(v.asDouble());
    if (speed.size() != route.size() || speed.front() != 0 || speed.back() != 0)
      throw std::runtime_error("profile_shape");
    for (double v : speed)
      if (!std::isfinite(v) || v < 0 || v > 2.0)
        throw std::runtime_error("profile_limits");
    double seam = (route.front().xy() - route.back().xy()).norm();
    if (closed_single_lap) {
      if (seam>1e-8 || std::abs(wrap(route.back().yaw-route.front().yaw))>1e-8)
        throw std::runtime_error("single_lap_seam_invalid");
    } else if (seam<=.08) throw std::runtime_error("closed_route_mode_required");
  }
  double clearance(Point p) const {
    int col = int(std::floor((p.x() - ox) / res)),
        row = height - 1 - int(std::floor((p.y() - oy) / res));
    return col >= 0 && col < width && row >= 0 && row < height
               ? grid[row * width + col]
               : 0.;
  }
};
struct Candidate {
  bool valid = false, collision = false;
  double offset = 0, transition = 0, ret = 0, score = INF, stat = 0, obs = 0,
         k = 0, jerk = 0, footprint = INF;
  Path path;
  std::string reason = "all_candidates_blocked";
  Json::Value terms;
};
inline Json::Value candidate_json(const Candidate &c) {
  Json::Value j;
  j["valid"] = c.valid;
  j["reason"] = c.reason;
  j["offset_m"] = c.offset;
  j["transition_m"] = c.transition;
  j["return_m"] = c.ret;
  j["score"] = std::isfinite(c.score) ? Json::Value(c.score) : Json::Value();
  j["minimum_static_clearance_m"] = c.stat;
  j["minimum_obstacle_clearance_m"] =
      std::isfinite(c.obs) ? Json::Value(c.obs) : Json::Value();
  j["maximum_curvature_1pm"] = c.k;
  j["lateral_jerk_cost"] = c.jerk;
  j["footprint_collision"] = c.collision;
  j["minimum_footprint_point_distance_m"] =
      std::isfinite(c.footprint) ? Json::Value(c.footprint) : Json::Value();
  j["score_terms"] = c.terms;
  j["path"] = path_json(c.path);
  return j;
}
inline std::array<double, 6> quintic(double p, double v, double c, double end,
                                     double l) {
  double l2 = l * l, l3 = l2 * l, l4 = l3 * l, l5 = l4 * l;
  return {p,
          v,
          .5 * c,
          (20 * (end - p) - 12 * v * l - 3 * c * l2) / (2 * l3),
          (30 * (p - end) + 16 * v * l + 3 * c * l2) / (2 * l4),
          (12 * (end - p) - 6 * v * l - c * l2) / (2 * l5)};
}
inline std::pair<double, double> eval(const std::array<double, 6> &a,
                                      double s) {
  double s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
  return {a[0] + a[1] * s + a[2] * s2 + a[3] * s3 + a[4] * s4 + a[5] * s5,
          6 * a[3] + 24 * a[4] * s + 60 * a[5] * s2};
}
struct Planner {
  const Assets &a;
  int progress = 0, last_index = -1, clear_scans = 0, commit_age = 0,
      switches = 0;
  double offset = 0, transition = 0, ret = 0, last_slope = 0,
         last_curvature = 0;
  std::string mode = "FOLLOWING", switch_reason = "initialized";
  Candidate cache;
  bool reused = false;
  std::vector<Candidate> candidates;
  double tracking_reserve;
  explicit Planner(const Assets &v, double reserve = .03) : a(v), tracking_reserve(reserve) {}
  int nearest(Pose p) {
    int begin = std::max(0, progress - 8),
        end = std::min(int(a.route.size()), progress + 81);
    double best = INF;
    for (int i = begin; i < end; ++i) {
      double d = (a.route[i].xy() - p.xy()).squaredNorm();
      if (d < best) {
        best = d;
        progress = i;
      }
    }
    return progress;
  }
  void clear() {
    offset = transition = ret = 0;
    clear_scans = commit_age = 0;
    mode = "RETURNING";
  }
  void distances(Candidate &c, const std::vector<Point> &obs) const {
    c.stat = INF;
    c.obs = INF;
    double footprint = INF;
    for (const auto &p : c.path) {
      c.stat = std::min(c.stat, a.clearance(p.xy()));
      Point tangent(std::cos(p.yaw), std::sin(p.yaw));
      for (const auto &o : obs) {
        c.obs = std::min(c.obs, (p.xy() - o).norm());
        for (double shift : {-.08, 0., .08})
          footprint =
              std::min(footprint, (p.xy() + shift * tangent - o).norm());
      }
    }
    c.collision = footprint < .15;
    c.footprint = footprint;
  }
  Candidate make(double d, double length, double back, double start,
                 double slope, double curve, const Path &base,
                 const std::vector<double> &s, const std::vector<Point> &obs,
                 double last_obs) {
    Candidate c;
    c.offset = d;
    c.transition = length;
    c.ret = back;
    bool terminal = (base.back().xy() - a.route.back().xy()).norm() < 1e-6;
    if (std::abs(d) < 1e-9 && terminal)
      c.transition = length = std::min(length, std::max(.001, s.back() - .25));
    auto outbound = quintic(start, slope, curve, d, length),
         inbound = quintic(d, 0, 0, 0, back);
    std::vector<double> jerks;
    double ret_start = std::max(length, last_obs + .25);
    for (size_t i = 0; i < base.size(); ++i) {
      auto value = eval(outbound, std::min(s[i], length));
      if (s[i] >= length)
        value = {d, 0};
      if (std::isfinite(last_obs) && std::abs(d) > 1e-9 && s[i] >= ret_start)
        value = eval(inbound, std::min(s[i] - ret_start, back));
      c.path.push_back({base[i].x - std::sin(base[i].yaw) * value.first,
                        base[i].y + std::cos(base[i].yaw) * value.first,
                        base[i].yaw});
      jerks.push_back(value.second);
    }
    if (c.path.size() > 1)
      for (size_t i = 0; i < c.path.size(); ++i) {
        size_t lo = i ? i - 1 : i, hi = std::min(i + 1, c.path.size() - 1);
        Point v = c.path[hi].xy() - c.path[lo].xy();
        c.path[i].yaw = std::atan2(v.y(), v.x());
      }
    distances(c, obs);
    c.k = curvature(c.path);
    double limit = std::max(std::tan(.27) / .25 + .03, curvature(base) + .03);
    bool heading_ok =
        !terminal || std::abs(d) > 1e-9 ||
        std::abs(wrap(c.path.back().yaw - a.route.back().yaw)) <= .10;
    c.valid = c.path.size() >= 2 && c.stat >= .25 && !c.collision &&
              c.obs >= .15 && c.k <= limit && heading_ok;
    c.reason = c.valid             ? "ok"
               : c.path.size() < 2 ? "terminal_path_too_short"
               : c.stat < .25      ? "static_clearance"
               : c.collision       ? "obstacle_footprint"
               : c.obs < .15       ? "obstacle_clearance"
               : !heading_ok       ? "terminal_heading"
                                   : "curvature";
    for (size_t i = 1; i < s.size(); ++i)
      c.jerk += (jerks[i] * jerks[i] + jerks[i - 1] * jerks[i - 1]) * .5 *
                (s[i] - s[i - 1]);
    double ob = std::isfinite(c.obs)
                    ? 4 * std::pow(std::max(0., .35 - (c.obs - .15)), 2)
                    : 0,
           wall = 5 * std::pow(std::max(0., .35 - c.stat), 2);
    double continuity =
        1.5 * std::abs(d - offset) +
        (transition <= 0 ? 0 : .2 * std::abs(length - transition)) +
        (ret <= 0 ? 0 : .1 * std::abs(back - ret)) +
        .04 * std::abs(length - 1.8) + .02 * std::abs(back - 1.6);
    c.terms["reference"] = std::abs(d);
    c.terms["curvature_jerk"] = .35 * c.k + .003 * c.jerk;
    c.terms["obstacle"] = ob;
    c.terms["wall"] = wall;
    c.terms["continuity"] = continuity;
    c.score = std::abs(d) + .35 * c.k + .003 * c.jerk + ob + wall + continuity;
    return c;
  }
  Candidate plan(Pose pose, std::vector<Point> obs, double horizon = 3.2) {
    reused = false;
    candidates.clear();
    nearest(pose);
    auto b = a.route[progress];
    double start =
        clip((pose.xy() - b.xy()).dot(Point(-std::sin(b.yaw), std::cos(b.yaw))),
             -.6, .6);
    int next = std::min(progress + 1, int(a.route.size()) - 1),
        prev = std::max(0, progress - 1);
    double rk =
        progress == next
            ? wrap(b.yaw - a.route[prev].yaw) / std::max(1e-6, a.ds[prev])
            : wrap(a.route[next].yaw - b.yaw) / std::max(1e-6, a.ds[progress]);
    double slope = clip(std::tan(clip(wrap(pose.yaw - b.yaw), -.6, .6)) *
                            (1 - rk * start),
                        -.8, .8),
           curve = 0;
    if (last_index >= 0) {
      int forward = progress - last_index;
      if (forward == 0)
        curve = last_curvature;
      else if (forward > 0 && forward <= 12)
        curve =
            .5 * last_curvature +
            .5 * clip((slope - last_slope) /
                          std::max(.05, a.arc[progress] - a.arc[last_index]),
                      -.6, .6);
    }
    last_index = progress;
    last_slope = slope;
    last_curvature = curve;
    obs.erase(std::remove_if(obs.begin(), obs.end(),
                             [&](Point p) { return a.clearance(p) <= .25; }),
              obs.end());
    Path base{b};
    std::vector<double> s{0};
    for (int i = progress; s.back() < std::min(3.2, std::max(0., horizon)) &&
                           i + 1 < int(a.route.size());
         ++i) {
      base.push_back(a.route[i + 1]);
      s.push_back(s.back() + a.ds[i]);
    }
    std::vector<Point> local;
    double last_obs = -INF;
    for (auto o : obs) {
      size_t idx = 0;
      double best = INF;
      for (size_t i = 0; i < base.size(); ++i) {
        double d = (base[i].xy() - o).norm();
        if (d < best) {
          best = d;
          idx = i;
        }
      }
      if (best < .8 && s[idx] <= 2.) {
        local.push_back(o);
        last_obs = std::max(last_obs, s[idx]);
      }
    }
    for (double l : {1.2, 1.5, 1.8, 2.1, 2.4})
      candidates.push_back(
          make(0, l, 1.2, start, slope, curve, base, s, local, last_obs));
    Candidate center = candidates.front();
    bool found = false;
    for (auto c : candidates)
      if (c.valid) {
        if (!found || c.score < center.score)
          center = c;
        found = true;
      }
    if (!found)
      for (auto c : candidates)
        if (c.score < center.score)
          center = c;
    if (cache.valid && std::abs(offset) < 1e-9) {
      size_t idx = 0;
      double best = INF;
      for (size_t i = 0; i < cache.path.size(); ++i) {
        double dist = (cache.path[i].xy() - pose.xy()).norm();
        if (dist < best) {
          best = dist;
          idx = i;
        }
      }
      Candidate copy = cache;
      copy.path.erase(copy.path.begin(),
                      copy.path.begin() + (idx ? idx - 1 : 0));
      double length = 0;
      for (size_t i = 1; i < copy.path.size(); ++i)
        length += (copy.path[i].xy() - copy.path[i - 1].xy()).norm();
      bool terminal =
          (copy.path.back().xy() - a.route.back().xy()).norm() < 1e-6;
      if (copy.path.size() >= 2 && best <= .15 && (length >= .8 || terminal)) {
        distances(copy, obs);
        if (copy.stat >= .25 && copy.obs >= .15 && !copy.collision &&
            copy.footprint >= .15 + tracking_reserve) {
          center = copy;
          reused = true;
        }
      }
    }
    auto finish = [&](Candidate c) {
      cache = (c.valid && std::abs(c.offset) < 1e-9) ? c : Candidate{};
      return c;
    };
    bool proactive = center.valid && std::abs(offset) < 1e-9 &&
                     !local.empty() && center.obs <= .22;
    if (center.valid && std::abs(offset) < 1e-9 && !proactive) {
      clear_scans = commit_age = 0;
      mode = "FOLLOWING";
      switch_reason = "center_clear";
      return finish(center);
    }
    for (double d : {-.4, -.3, -.2, -.15, -.1, .1, .15, .2, .3, .4})
      for (double l : {1.2, 1.5, 1.8, 2.1, 2.4})
        for (double back : {1.2, 1.6, 2.})
          candidates.push_back(
              make(d, l, back, start, slope, curve, base, s, local, last_obs));
    std::vector<Candidate> allowed;
    for (auto c : candidates)
      if (c.valid && std::abs(c.offset) > 1e-9 &&
          (std::abs(offset) < 1e-9 || c.offset * offset > 0))
        allowed.push_back(c);
    if (std::abs(offset) > 1e-9) {
      if (center.valid) {
        ++clear_scans;
        if (clear_scans >= 5 || allowed.empty()) {
          std::string why = clear_scans >= 5
                                ? "center_clear_hold_satisfied"
                                : "committed_side_invalid_center_safe";
          clear();
          switch_reason = why;
          return finish(center);
        }
      } else
        clear_scans = 0;
    } else {
      std::vector<Candidate> decisive;
      for (auto c : allowed)
        if (std::abs(c.offset) + 1e-9 >= .15)
          decisive.push_back(c);
      if (!decisive.empty())
        allowed = decisive;
    }
    if (allowed.empty()) {
      if (center.valid) {
        clear_scans = commit_age = 0;
        mode = "FOLLOWING";
        switch_reason = "proactive_clearance_no_valid_avoidance";
        return finish(center);
      }
      mode = "BLOCKED";
      switch_reason = "all_candidates_blocked";
      Candidate invalid;
      invalid.offset = offset;
      return finish(invalid);
    }
    // Prefer room for tracking error before hysteresis can retain a path
    // grazing the hard footprint boundary. The current pose can already be
    // inside the preferred buffer: never mislabel that as a hard collision.
    std::vector<Candidate> buffered;
    for (auto c : allowed)
      if (c.footprint >= .15 + tracking_reserve) buffered.push_back(c);
    if (!buffered.empty()) allowed = buffered;
    Candidate best = allowed.front();
    for (auto c : allowed)
      if (c.score < best.score)
        best = c;
    Candidate selected = best;
    bool incumbent_found = false;
    Candidate incumbent;
    if (std::abs(offset) > 1e-9)
      for (auto c : allowed)
        if (std::abs(c.offset - offset) < 1e-9 &&
            std::abs(c.transition - transition) < 1e-9 &&
            std::abs(c.ret - ret) < 1e-9) {
          if (!incumbent_found || c.score < incumbent.score)
            incumbent = c;
          incumbent_found = true;
        }
    if (incumbent_found &&
        (commit_age < 3 || best.score + .12 >= incumbent.score)) {
      selected = incumbent;
      switch_reason = "incumbent_hysteresis";
    } else if (std::abs(offset) > 1e-9)
      switch_reason = incumbent_found ? "score_improvement_exceeded_margin"
                                      : "incumbent_invalid";
    else
      switch_reason = proactive ? "proactive_clearance_select_avoidance"
                                : "center_blocked_select_avoidance";
    if (std::abs(selected.offset - offset) > 1e-9 ||
        std::abs(selected.transition - transition) > 1e-9 ||
        std::abs(selected.ret - ret) > 1e-9) {
      ++switches;
      commit_age = 0;
    } else
      ++commit_age;
    offset = selected.offset;
    transition = selected.transition;
    ret = selected.ret;
    mode = "AVOIDING";
    if (!center.valid)
      clear_scans = 0;
    return finish(selected);
  }
};
struct Progress {
  int previous = -1;
  double distance = 0;
  void update(int index, const Assets &a) {
    if (previous < 0) {
      if (a.closed_single_lap && a.arc[index]>.30)
        throw std::runtime_error("single_lap_start_not_at_anchor");
      previous = index;
      distance = a.arc[index];
      return;
    }
    int n = index - previous;
    if (n < 0) {
      if (-n <= 8)
        return;
      throw std::runtime_error("progress_jump");
    }
    if (n > 80)
      throw std::runtime_error("progress_jump");
    distance += a.arc[index] - a.arc[previous];
    previous = index;
  }
};
inline bool endpoint_match(const Assets &a, Pose p) {
  return (p.xy() - a.route.back().xy()).norm() <= .1 &&
         std::abs(wrap(p.yaw - a.route.back().yaw)) <= .2;
}
} // namespace frenet
