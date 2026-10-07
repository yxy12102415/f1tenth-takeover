#pragma once
#include "core.hpp"
#include <deque>
#include <tuple>
namespace frenet {
struct TimedPose {
  double stamp;
  Pose pose;
};
struct PoseHistory {
  std::deque<TimedPose> values;
  void add(TimedPose p) {
    if (!values.empty() && p.stamp <= values.back().stamp)
      throw std::runtime_error("pose_history_nonincreasing");
    values.push_back(p);
    if (values.size() > 80)
      values.pop_front();
  }
  Pose at(double source, double &span) const {
    if (values.empty())
      throw std::runtime_error("scan_pose_unsynchronized");
    for (size_t i = 0; i < values.size(); ++i) {
      auto right = values[i];
      if (right.stamp < source)
        continue;
      if (!i) {
        span = right.stamp - source;
        if (span <= .075)
          return right.pose;
        break;
      }
      auto left = values[i - 1];
      span = right.stamp - left.stamp;
      if (span <= 0 || span > .15)
        break;
      double f = (source - left.stamp) / span;
      return {left.pose.x + f * (right.pose.x - left.pose.x),
              left.pose.y + f * (right.pose.y - left.pose.y),
              left.pose.yaw + f * wrap(right.pose.yaw - left.pose.yaw)};
    }
    if (source >= values.back().stamp && source - values.back().stamp <= .075) {
      span = source - values.back().stamp;
      return values.back().pose;
    }
    throw std::runtime_error("scan_pose_unsynchronized");
  }
};
struct Cluster {
  std::vector<Point> points;
  Point center;
  double extent;
};
struct Track {
  Cluster cluster;
  int id = 0, hits = 1, misses = 0;
  double stamp = 0;
  bool confirmed = false;
};
struct Perception {
  const Assets &a;
  std::vector<Track> tracks;
  int next_id = 1, raw_count = 0, dynamic_count = 0, cluster_count = 0;
  double last_stamp = -INF;
  explicit Perception(const Assets &v) : a(v) {}
  std::vector<Cluster> cluster(const std::vector<Point> &p,
                               const std::vector<int> &beam) {
    std::vector<Cluster> result;
    size_t start = 0;
    for (size_t end = 1; end <= p.size(); ++end) {
      bool split = end == p.size() || beam[end] - beam[end - 1] <= 0 ||
                   beam[end] - beam[end - 1] > 2 ||
                   (p[end] - p[end - 1]).norm() > .18;
      if (!split)
        continue;
      if (end - start >= 3) {
        Cluster c;
        c.center = Point::Zero();
        Point lo = p[start], hi = p[start];
        for (size_t i = start; i < end; ++i) {
          c.points.push_back(p[i]);
          c.center += p[i];
          lo = lo.cwiseMin(p[i]);
          hi = hi.cwiseMax(p[i]);
        }
        c.center /= double(c.points.size());
        c.extent = (hi - lo).norm();
        if (c.extent >= .025 && c.extent <= .8)
          result.push_back(c);
      }
      start = end;
    }
    return result;
  }
  std::vector<Point> update(const std::vector<Cluster> &clusters,
                            double stamp) {
    if (!std::isfinite(stamp) || stamp <= last_stamp)
      throw std::runtime_error("obstacle_tracker_stamp_nonincreasing");
    last_stamp = stamp;
    std::vector<std::tuple<double, size_t, size_t>> pairs;
    for (size_t i = 0; i < tracks.size(); ++i)
      for (size_t j = 0; j < clusters.size(); ++j) {
        double d = (tracks[i].cluster.center - clusters[j].center).norm();
        if (d <= .22)
          pairs.emplace_back(d, i, j);
      }
    std::sort(pairs.begin(), pairs.end());
    std::vector<bool> mt(tracks.size()), mc(clusters.size());
    for (auto item : pairs) {
      size_t i = std::get<1>(item), j = std::get<2>(item);
      if (mt[i] || mc[j])
        continue;
      auto &t = tracks[i];
      t.cluster = clusters[j];
      ++t.hits;
      t.misses = 0;
      t.stamp = stamp;
      t.confirmed = t.confirmed || t.hits >= 2;
      mt[i] = mc[j] = true;
    }
    std::vector<Track> surviving;
    for (size_t i = 0; i < tracks.size(); ++i) {
      auto t = tracks[i];
      if (!mt[i])
        ++t.misses;
      if (t.confirmed ? (t.misses <= 1 && stamp - t.stamp <= .30 + 1e-9)
                      : t.misses == 0)
        surviving.push_back(t);
    }
    for (size_t j = 0; j < clusters.size(); ++j)
      if (!mc[j])
        surviving.push_back({clusters[j], next_id++, 1, 0, stamp, false});
    tracks = std::move(surviving);
    std::vector<Point> points;
    for (auto t : tracks)
      if (t.confirmed)
        points.insert(points.end(), t.cluster.points.begin(),
                      t.cluster.points.end());
    return points;
  }
  std::vector<Point> scan(const std::vector<float> &ranges, double amin,
                          double inc, double rmin, double rmax, double stamp,
                          Pose p) {
    raw_count = 0;
    std::vector<Point> dynamic;
    std::vector<int> beams;
    Point laser = p.xy() + .27 * Point(std::cos(p.yaw), std::sin(p.yaw));
    for (size_t i = 0; i < ranges.size(); ++i) {
      double r = ranges[i];
      if (!std::isfinite(r) || r < rmin || r > std::min(rmax, 7.))
        continue;
      ++raw_count;
      double angle = p.yaw + amin + i * inc;
      Point dir(std::cos(angle), std::sin(angle)), point = laser + r * dir;
      if (a.clearance(point) <= .25)
        continue;
      bool wall = false;
      for (int step = -9; step <= 9; ++step)
        if (step && a.clearance(point + (.05 * step) * dir) <= .05) {
          wall = true;
          break;
        }
      if (!wall) {
        dynamic.push_back(point);
        beams.push_back(int(i));
      }
    }
    dynamic_count = int(dynamic.size());
    auto clusters = cluster(dynamic, beams);
    cluster_count = int(clusters.size());
    return update(clusters, stamp);
  }
  Json::Value diagnostic() const {
    Json::Value j;
    j["raw_scan_point_count"] = raw_count;
    j["dynamic_scan_point_count"] = dynamic_count;
    j["detected_cluster_count"] = cluster_count;
    j["confirmed_obstacle_tracks"] = Json::Value(Json::arrayValue);
    int n = 0;
    for (auto t : tracks)
      if (t.confirmed) {
        Json::Value v;
        v["id"] = t.id;
        v["hits"] = t.hits;
        v["misses"] = t.misses;
        v["x"] = t.cluster.center.x();
        v["y"] = t.cluster.center.y();
        v["extent_m"] = t.cluster.extent;
        j["confirmed_obstacle_tracks"].append(v);
        n += int(t.cluster.points.size());
      }
    j["confirmed_dynamic_point_count"] = n;
    j["confirmed_cluster_count"] = int(j["confirmed_obstacle_tracks"].size());
    return j;
  }
};
} // namespace frenet
