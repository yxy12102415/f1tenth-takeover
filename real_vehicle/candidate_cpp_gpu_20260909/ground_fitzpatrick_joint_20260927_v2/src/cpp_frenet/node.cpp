#include "perception.hpp"
#include "joint_control.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <visualization_msgs/msg/marker_array.hpp>
using namespace frenet;
namespace {
double stamp(const builtin_interfaces::msg::Time &t) {
  return double(t.sec) + 1e-9 * t.nanosec;
}
double yaw_checked(const geometry_msgs::msg::Quaternion &q) {
  double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!std::isfinite(norm) || std::abs(norm - 1) > .02 || std::abs(q.x) > .01 ||
      std::abs(q.y) > .01)
    throw std::runtime_error("invalid_quaternion");
  return 2 * std::atan2(q.z, q.w);
}
struct Input {
  Pose pose;
  PoseHistory history;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
  double ps = -INF, pr = -INF, ss = -INF, sr = -INF, os = -INF, orr = -INF,
         qr = -INF, er = -INF, speed = 0, lateral = 0, rate = 0;
  bool quality = false, enabled = false;
};
} // namespace
class Runtime : public rclcpp::Node {
  bool planning;
  std::string tracker_type;
  std::unique_ptr<Assets> assets;
  std::unique_ptr<Planner> planner;
  std::unique_ptr<Perception> perception;
  std::unique_ptr<Tracker> tracker;
  std::unique_ptr<JointControl> joint;
  std::mutex input_mutex;
  Input input;
  std::thread worker;
  std::atomic<bool> stopping{false};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub, odom_sub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr quality_sub, enable_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr plan_sub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr plan_pub, status_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
      drive_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      obstacles_pub;
  rclcpp::TimerBase::SharedPtr control_timer, watchdog_timer;
  Json::Value plan_message;
  Json::Value control_input, rejected_control;
  Path local_path;
  double plan_receipt = -INF, plan_source = -INF, last_tick = -INF,
         last_control = -INF;
  bool complete = false;
  bool terminal_stop_requested = false;
  double terminal_still_since = -INF;
  Pose terminal_still_pose;
  int logged_plan_id = -1;
  uint64_t sequence = 0, plan_sequence = 0;
  Input snapshot() {
    std::lock_guard<std::mutex> g(input_mutex);
    return input;
  }
  bool fresh(double source, double receipt, double now, double ros) const {
    return -0.02 <= ros - source && ros - source <= .505 &&
           now - receipt >= 0 && now - receipt <= .50;
  }
  bool ready(const Input &i, double now) {
    double ros = get_clock()->now().seconds();
    return i.scan && fresh(i.ps, i.pr, now, ros) &&
           fresh(i.ss, i.sr, now, ros) && fresh(i.os, i.orr, now, ros) &&
           i.quality && now - i.qr >= 0 && now - i.qr <= .50;
  }
  bool permit(const Input &i, double now) const {
    return i.enabled && now - i.er >= 0 && now - i.er <= .50;
  }
  Json::Value ages(const Input &i) {
    double now = mono(), ros = get_clock()->now().seconds();
    Json::Value j;
    auto one = [&](double source, double receipt) {
      Json::Value v;
      v["source_age_s"] = std::isfinite(source) ? ros - source : 1e9;
      v["receipt_age_s"] = std::isfinite(receipt) ? now - receipt : 1e9;
      v["timeout_s"] = .50;
      return v;
    };
    j["pose"] = one(i.ps, i.pr);
    j["scan"] = one(i.ss, i.sr);
    j["odom"] = one(i.os, i.orr);
    return j;
  }
  void emit(double speed, double angle, bool valid, const std::string &reason) {
    if (planning)
      return;
    double now = mono();
    if (!std::isfinite(speed + angle) || speed < 0 || speed > 2.0 ||
        std::abs(angle) > .269) {
      speed = angle = 0;
      valid = false;
    }
    ackermann_msgs::msg::AckermannDriveStamped drive;
    drive.header.stamp = get_clock()->now();
    drive.header.frame_id = "base_link";
    drive.drive.speed = float(speed);
    drive.drive.steering_angle = float(angle);
    drive_pub->publish(drive);
    std_msgs::msg::Bool ok;
    ok.data = valid;
    valid_pub->publish(ok);
    Json::Value j =
        plan_message.isObject() ? plan_message : Json::Value(Json::objectValue);
    // Retain one exact local-path snapshot per planner generation for replay.
    int plan_id = j.get("plan_id", -1).asInt();
    if (plan_id >= 0 && plan_id != logged_plan_id && j["path"].isArray()) {
      j["local_path_snapshot"] = j["path"];
      logged_plan_id = plan_id;
    }
    j.removeMember("path");
    j["valid"] = valid;
    j["complete"] = complete;
    j["reason"] = reason;
    j["controller"] = "closed_loop_frenet_cpp_" + tracker_type;
    j["implementation"] = "cpp_separate_planner_controller";
    j["speed_command"] = speed;
    j["steering_command"] = angle;
    j["tracker_diagnostics"] = tracker->diag;
    if (control_input.isObject()) j["control_input_snapshot"] = control_input;
    if (rejected_control.isObject()) j["rejected_control"] = rejected_control;
    j["tracker_config"]["type"] = tracker_type;
    j["tracker_config"]["state_source"] =
        "vesc_odometry_vy_zero_yaw_estimated_from_speed_and_servo";
    j["tracker_config"]["state_dimension"] =
        tracker_type == "dynamic_mpc" ? 10 : 0;
    j["runtime_timing"]["sequence"] = Json::UInt64(++sequence);
    j["runtime_timing"]["tick_start_mono"] = now;
    j["runtime_timing"]["publish_mono"] = mono();
    auto i = snapshot();
    j["input_age"] = ages(i);
    j["endpoint_distance_m"] = (i.pose.xy() - assets->route.back().xy()).norm();
    j["endpoint_heading_error_rad"] =
        wrap(i.pose.yaw - assets->route.back().yaw);
    j["planning_active"] = false;
    j["plan_interval_s"] = .2;
    j["control_period_s"] = .05;
    std_msgs::msg::String msg;
    msg.data = dump(j);
    status_pub->publish(msg);
  }
  void stop(const std::string &why) {
    tracker->reset();
    if(joint)joint->reset();
    last_tick = -INF;
    emit(0, 0, false, why);
  }
  void tick() {
    auto i = snapshot();
    double now = mono();
    control_input = Json::Value();
    rejected_control = Json::Value();
    try {
      if (!ready(i, now))
        throw std::runtime_error("input_stale");
      if (now - plan_receipt > .5 || now - plan_source > .5)
        throw std::runtime_error("planning_timeout");
      if (plan_message.get("valid", false).asBool() &&
          (get_clock()->now().seconds() -
                   plan_message["input_scan_stamp"].asDouble() >
               .505 ||
           get_clock()->now().seconds() -
                   plan_message["input_pose_stamp"].asDouble() >
               .505))
        throw std::runtime_error("planned_input_stale");
      if (!plan_message.get("valid", false).asBool())
        throw std::runtime_error(
            plan_message.get("reason", "planner_invalid").asString());
      if (complete) {
        emit(0, 0, true, "route_complete");
        return;
      }
      // A requested terminal coast remains zero even after the independent
      // motion window latches. Arrival is acknowledged only after standstill,
      // with the original 10cm / 0.2rad endpoint tolerances.
      if (terminal_stop_requested) {
        if (i.speed > .03) terminal_still_since = -INF;
        else if (!std::isfinite(terminal_still_since) ||
                 (i.pose.xy()-terminal_still_pose.xy()).norm()>.02 ||
                 std::abs(wrap(i.pose.yaw-terminal_still_pose.yaw))>.05) {
          terminal_still_since=now;terminal_still_pose=i.pose;
        }
        if (std::isfinite(terminal_still_since) && now-terminal_still_since>=.2) {
          if (!endpoint_match(*assets,i.pose))
            throw std::runtime_error("terminal_stop_outside_endpoint");
          complete=true;
        }
        last_control=now;
        emit(0,0,true,complete?"route_complete":"terminal_coast");
        return;
      }
      if (!permit(i, now)) {
        tracker->reset();
    if(joint)joint->reset();
        last_tick = -INF;
        last_control = now;
        emit(0, 0, true, "waiting_for_operator");
        return;
      }
      double progress = plan_message["lap_progress_m"].asDouble(),
             remaining = std::max(0., assets->arc.back() - progress);
      if (remaining <= .05) {
        if (!endpoint_match(*assets, i.pose))
          throw std::runtime_error("endpoint_pose_mismatch");
        if (joint) terminal_stop_requested=true;
        else complete = true;
        tracker->reset();
    if(joint)joint->reset();
        emit(0, 0, true, complete?"route_complete":"terminal_coast");
        return;
      }
      if (local_path.size() < 2)
        throw std::runtime_error("tracker_path_invalid");
      // Skip catch-up callbacks; otherwise an abnormally short dt makes the
      // first input-rate bounds tighter than the previously planned sequence.
      // Existing input/plan/enable checks above still execute on every callback.
      if(joint && std::isfinite(last_tick) && now-last_tick<.04)return;
      double dt=std::isfinite(last_tick)?clip(now-last_tick,.001,.05):.05;
      double speed=0, angle=0;
      std::string state=plan_message.get("state","FOLLOWING").asString();
      double cap=(state=="CONFIRM_BLOCKAGE" || state=="WAIT_SCAN_POSE")?.7:2.0;
      if(joint) {
        if(!plan_message["obstacle_points"].isArray())throw std::runtime_error("joint_obstacles_missing");
        std::vector<Point> obstacles;
        for(auto o:plan_message["obstacle_points"])obstacles.emplace_back(o[0].asDouble(),o[1].asDouble());
        control_input["pose"] = path_json(Path{i.pose})[0];
        control_input["pose_stamp"] = i.ps;
        control_input["odom_stamp"] = i.os;
        control_input["speed"] = i.speed;
        control_input["lateral"] = i.lateral;
        control_input["yaw_rate"] = i.rate;
        control_input["dt"] = dt;
        control_input["remaining"] = remaining;
        control_input["cap"] = cap;
        control_input["plan_id"] = plan_message["plan_id"];
        auto control=joint->compute(*assets,local_path,i.pose,i.speed,i.lateral,i.rate,remaining,cap,dt,*tracker,obstacles);
        speed=control.speed;angle=control.angle;tracker->diag=joint->diag;
        if(joint->terminal_coasting()) terminal_stop_requested=true;
      } else {
        angle=tracker->compute(local_path,i.pose,dt,i.speed,i.lateral,i.rate);
        double profile=interp(assets->arc,assets->speed,std::min(assets->arc.back()-.05,progress+.2));
        speed=std::min(profile,1.5-.9*std::min(1.,std::abs(angle)/.269));
        if(state=="AVOIDING" || state=="RETURNING")speed=std::min(speed,1.2);
        speed=std::min({speed,cap,std::sqrt(.5*remaining)});
      }
      last_tick=now;
      auto latest = snapshot();
      if (!ready(latest, mono()))
        throw std::runtime_error("input_stale");
      if (mono() - plan_source > .5 || mono() - plan_receipt > .5)
        throw std::runtime_error("planning_timeout");
      if (get_clock()->now().seconds() -
                  plan_message["input_scan_stamp"].asDouble() >
              .505 ||
          get_clock()->now().seconds() -
                  plan_message["input_pose_stamp"].asDouble() >
              .505)
        throw std::runtime_error("planned_input_stale");
      if (!permit(latest, mono())) {
        tracker->reset();
    if(joint)joint->reset();
        emit(0, 0, true, "waiting_for_operator");
        return;
      }
      last_control = mono();
      plan_message["velocity_policy"] = joint?"controller_longitudinal":"legacy_profile";
      emit(speed, angle, true, "ok");
    } catch (const std::exception &e) {
      if (joint && control_input.isObject()) {
        rejected_control["reason"] = e.what();
        rejected_control["diagnostics"] = joint->diag;
      }
      stop(e.what());
    }
  }
  void publish_geometry(const Path &p) {
    nav_msgs::msg::Path msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = "map";
    for (auto v : p) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position.x = v.x;
      pose.pose.position.y = v.y;
      pose.pose.orientation.z = std::sin(v.yaw / 2);
      pose.pose.orientation.w = std::cos(v.yaw / 2);
      msg.poses.push_back(pose);
    }
    path_pub->publish(msg);
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header = msg.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    for (auto t : perception->tracks)
      if (t.confirmed) {
        auto m = clear;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.ns = "confirmed_obstacles";
        m.id = t.id;
        m.pose.position.x = t.cluster.center.x();
        m.pose.position.y = t.cluster.center.y();
        m.pose.position.z = .1;
        m.pose.orientation.w = 1;
        Point low = t.cluster.points.front(), high = low;
        for (auto v : t.cluster.points) {
          low = low.cwiseMin(v);
          high = high.cwiseMax(v);
        }
        m.scale.x = std::max(.05, high.x() - low.x() + .04);
        m.scale.y = std::max(.05, high.y() - low.y() + .04);
        m.scale.z = .2;
        m.color.r = 1;
        m.color.g = .25;
        m.color.a = .8;
        markers.markers.push_back(m);
      }
    obstacles_pub->publish(markers);
  }
  void plan_loop() {
    Progress progress;
    Candidate last_safe;
    Json::Value last_plan_envelope;
    double used_scan = -INF, blocked_since = -INF;
    int blocked_count = 0;
    while (rclcpp::ok() && !stopping) {
      double begin = mono();
      auto i = snapshot();
      Json::Value out;
      bool waiting = false;
      try {
        if (!ready(i, begin))
          throw std::runtime_error("input_stale");
        if (i.ss <= used_scan) {
          // No new sensor sample is not a new plan. Keep the old lease intact.
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          continue;
        }
        double span = 0;
        Pose scan_pose = i.history.at(i.ss, span);
        double scan_begin = mono();
        auto points = perception->scan(
            i.scan->ranges, i.scan->angle_min, i.scan->angle_increment,
            i.scan->range_min, i.scan->range_max, i.ss, scan_pose);
        used_scan = i.ss;
        double scan_time = mono() - scan_begin;
        double plan_begin = mono();
        auto candidate =
            planner->plan(i.pose, points,
                          std::max(0., assets->arc.back() - progress.distance));
        double plan_time = mono() - plan_begin;
        progress.update(planner->progress, *assets);
        double remaining = assets->arc.back() - progress.distance;
        std::string state = planner->mode;
        bool finished = remaining <= .05;
        if (finished && !endpoint_match(*assets, i.pose))
          throw std::runtime_error("endpoint_pose_mismatch");
        double nearest = INF;
        for (auto p : points)
          nearest = std::min(nearest, (p - i.pose.xy()).norm());
        if (candidate.valid) {
          last_safe = candidate;
          blocked_since = -INF;
          blocked_count = 0;
        } else if (!finished) {
          if (candidate.reason == "all_candidates_blocked") {
            if (!std::isfinite(blocked_since))
              blocked_since = mono();
            ++blocked_count;
            double required = nearest <= .5 ? .2 : .75;
            if (mono() - blocked_since + 1e-9 < required && last_safe.valid) {
              candidate = last_safe;
              state = "CONFIRM_BLOCKAGE";
            } else
              last_safe = Candidate{};
          } else {
            blocked_since = -INF;
            blocked_count = 0;
            last_safe = Candidate{};
          }
        }
        if (mono() - begin > .5)
          throw std::runtime_error("planning_timeout");
        if (!ready(snapshot(), mono()))
          throw std::runtime_error("input_stale");
        out = perception->diagnostic();
        out["valid"] = candidate.valid || finished;
        out["reason"] = finished ? "route_complete" : candidate.reason;
        out["path"] = path_json(candidate.path);
        out["obstacle_points"]=Json::Value(Json::arrayValue);
        for(auto p:points){Json::Value o(Json::arrayValue);o.append(p.x());o.append(p.y());out["obstacle_points"].append(o);}
        out["state"] = state;
        out["side"] = candidate.offset > 0   ? "left"
                      : candidate.offset < 0 ? "right"
                                             : "center";
        out["offset_m"] = candidate.offset;
        out["transition_m"] = candidate.transition;
        out["return_m"] = candidate.ret;
        out["candidate_switch_count"] = planner->switches;
        out["commit_age_scans"] = planner->commit_age;
        out["switch_reason"] = planner->switch_reason;
        out["center_path_reused"] = planner->reused;
        out["candidate_score"] = std::isfinite(candidate.score)
                                     ? Json::Value(candidate.score)
                                     : Json::Value();
        out["candidate_score_terms"] = candidate.terms;
        out["lateral_jerk_cost"] = candidate.jerk;
        out["footprint_collision"] = candidate.collision;
        out["minimum_footprint_point_distance_m"] = std::isfinite(candidate.footprint)
            ? Json::Value(candidate.footprint) : Json::Value();
        out["preferred_tracking_reserve_m"] = planner->tracking_reserve;
        out["tracking_reserve_satisfied"] = candidate.footprint >= .15 + planner->tracking_reserve;
        out["minimum_static_clearance_m"] = candidate.stat;
        out["minimum_obstacle_clearance_m"] = std::isfinite(candidate.obs)
                                                  ? Json::Value(candidate.obs)
                                                  : Json::Value();
        out["nearest_dynamic_point_m"] =
            std::isfinite(nearest) ? Json::Value(nearest) : Json::Value();
        out["progress_index"] = planner->progress;
        out["lap_progress_m"] = progress.distance;
        out["lap_length_m"] = assets->arc.back();
        out["route_mode"] = assets->closed_single_lap ? "closed_single_lap_stop" : "open_stop_at_end";
        out["scan_pose_alignment_span_s"] = span;
        out["last_plan_duration_s"] = mono() - begin;
        out["last_plan_stages"]["scan_processing_s"] = scan_time;
        out["last_plan_stages"]["lattice_search_s"] = plan_time;
        out["last_plan_stages"]["compute_wall_s"] = mono() - begin;
        out["last_plan_stages"]["total_wall_s"] = mono() - begin;
        out["candidate_reason_counts"] = Json::Value(Json::objectValue);
        for (auto c : planner->candidates)
          out["candidate_reason_counts"][c.reason] =
              out["candidate_reason_counts"].get(c.reason, 0).asInt() + 1;
        out["blocked_confirmation_scans"] = blocked_count;
        out["local_path_endpoint_xy"] = Json::Value(Json::arrayValue);
        if (!candidate.path.empty()) {
          out["local_path_endpoint_xy"].append(candidate.path.back().x);
          out["local_path_endpoint_xy"].append(candidate.path.back().y);
        }
        publish_geometry(candidate.path);
      } catch (const std::exception &e) {
        // A scan can precede its localization sample. As in v3, wait on the
        // last safe path without refreshing either plan or input timestamps.
        waiting =
            std::string(e.what()) == "scan_pose_unsynchronized" &&
            last_safe.valid &&
            last_plan_envelope.get("valid", false).asBool() &&
            mono() - last_plan_envelope["generated_mono"].asDouble() <= .5;
        if (waiting) {
          out = last_plan_envelope;
          out["state"] = "WAIT_SCAN_POSE";
          out["waiting_for_scan_pose"] = true;
        } else {
          out["valid"] = false;
          out["reason"] = e.what();
          out["path"] = Json::Value(Json::arrayValue);
          last_safe = Candidate{};
        }
      }
      if (!waiting) {
        out["generated_mono"] = mono();
        out["input_scan_stamp"] =
            std::isfinite(i.ss) ? Json::Value(i.ss) : Json::Value();
        out["input_pose_stamp"] =
            std::isfinite(i.ps) ? Json::Value(i.ps) : Json::Value();
        out["frame_id"] = "map";
        out["plan_id"] = Json::UInt64(++plan_sequence);
        last_plan_envelope = out;
      }
      std_msgs::msg::String msg;
      msg.data = dump(out);
      try {
        if (rclcpp::ok())
          plan_pub->publish(msg);
      } catch (const std::exception &) {
        if (!rclcpp::ok())
          break;
      }
      for (int t = 0; t < (waiting ? 1 : 20) && !stopping && rclcpp::ok(); ++t)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

public:
  explicit Runtime(bool planner_role)
      : Node(planner_role ? "frenet_cpp_planner" : "frenet_cpp_controller"),
        planning(planner_role) {
    std::string file = declare_parameter<std::string>("assets_file", "");
    tracker_type = declare_parameter<std::string>("tracker", "dynamic_mpc");
    assets.reset(new Assets(file));
    tracker.reset(new Tracker(tracker_type));
      if(!planning && tracker_type!="pp")joint.reset(new JointControl(tracker_type=="dynamic_mpc"));
    planner.reset(new Planner(*assets, (tracker_type=="pid" || tracker_type=="dynamic_mpc") ? .03 : 0.));
    perception.reset(new Perception(*assets));
    const char *domain = std::getenv("ROS_DOMAIN_ID");
    const char *local = std::getenv("ROS_LOCALHOST_ONLY");
    if (!domain || !local || std::string(local) != "1" ||
        (std::string(domain) != "83" && std::string(domain) != "85"))
      throw std::runtime_error("runtime_domain_refused");
    if (std::string(domain) == "85" &&
        (!std::getenv("F1TENTH_GROUND_AVOIDANCE_TEST") ||
         std::string(std::getenv("F1TENTH_GROUND_AVOIDANCE_TEST")) !=
             "bounded_90s_L1"))
      throw std::runtime_error("physical_scope_missing");
    pose_sub = create_subscription<nav_msgs::msg::Odometry>(
        "/baseline/localization_pose", 1,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> g(input_mutex);
          try {
            double source = stamp(msg->header.stamp);
            Pose p{msg->pose.pose.position.x, msg->pose.pose.position.y,
                   yaw_checked(msg->pose.pose.orientation)};
            if (msg->header.frame_id != "map" ||
                msg->child_frame_id != "base_link" ||
                !std::isfinite(source + p.x + p.y + p.yaw) ||
                source <= input.ps)
              throw std::runtime_error("pose_invalid");
            input.history.add({source, p});
            input.pose = p;
            input.ps = source;
            input.pr = mono();
          } catch (...) {
            input.pr = -INF;
          }
        });
    odom_sub = create_subscription<nav_msgs::msg::Odometry>(
        "/baseline/odom", 1,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> g(input_mutex);
          try {
            double s = stamp(msg->header.stamp), v = msg->twist.twist.linear.x,
                   vy = msg->twist.twist.linear.y,
                   r = msg->twist.twist.angular.z;
            yaw_checked(msg->pose.pose.orientation);
            if (msg->header.frame_id != "odom" ||
                msg->child_frame_id != "base_link" || s <= input.os ||
                !std::isfinite(s + v + vy + r) || std::abs(vy) > 2.5 ||
                std::abs(r) > 8)
              throw std::runtime_error("odom_invalid");
            input.os = s;
            input.orr = mono();
            input.speed = std::abs(v);
            input.lateral = vy;
            input.rate = r;
          } catch (...) {
            input.orr = -INF;
          }
        });
    scan_sub = create_subscription<sensor_msgs::msg::LaserScan>(
        "/baseline/scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> g(input_mutex);
          double s = stamp(msg->header.stamp);
          if (msg->header.frame_id != "laser" || s <= input.ss ||
              !std::isfinite(s + msg->angle_min + msg->angle_increment +
                             msg->range_min + msg->range_max) ||
              msg->angle_increment <= 0 || msg->range_min < 0 ||
              msg->range_min >= msg->range_max || msg->ranges.empty() ||
              msg->ranges.size() > 10000) {
            input.scan.reset();
            return;
          }
          input.scan = msg;
          input.ss = s;
          input.sr = mono();
        });
    quality_sub = create_subscription<std_msgs::msg::Bool>(
        "/baseline/localization_valid", 1,
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> g(input_mutex);
          input.quality = msg->data;
          input.qr = mono();
        });
    enable_sub = create_subscription<std_msgs::msg::Bool>(
        "/baseline/enable", 1, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
          {
            std::lock_guard<std::mutex> g(input_mutex);
            input.enabled = msg->data;
            input.er = mono();
          }
          if (!planning && !msg->data) {
            tracker->reset();
    if(joint)joint->reset();
            last_tick = -INF;
            auto i = snapshot();
            emit(0, 0,
                 ready(i, mono()) && mono() - plan_receipt <= .5 &&
                     plan_message.get("valid", false).asBool(),
                 terminal_stop_requested?"terminal_coast":"waiting_for_operator");
          }
        });
    if (planning) {
      plan_pub = create_publisher<std_msgs::msg::String>(
          "/baseline/local_plan_cpp", 1);
      path_pub = create_publisher<nav_msgs::msg::Path>(
          "/baseline/lattice_selected", 1);
      obstacles_pub = create_publisher<visualization_msgs::msg::MarkerArray>(
          "/baseline/confirmed_obstacles", 1);
      worker = std::thread([this] { plan_loop(); });
    } else {
      drive_pub = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
          "/baseline/drive_nominal", 1);
      valid_pub =
          create_publisher<std_msgs::msg::Bool>("/baseline/planner_valid", 1);
      status_pub = create_publisher<std_msgs::msg::String>(
          "/baseline/controller_status", 1);
      plan_sub = create_subscription<std_msgs::msg::String>(
          "/baseline/local_plan_cpp", 1,
          [this](std_msgs::msg::String::ConstSharedPtr msg) {
            try {
              if (msg->data.size() > 1000000)
                throw std::runtime_error("plan_size");
              Json::CharReaderBuilder b;
              Json::Value j;
              std::string errors;
              std::istringstream s(msg->data);
              if (!Json::parseFromStream(b, s, &j, &errors) ||
                  j["frame_id"] != "map")
                throw std::runtime_error("plan_invalid");
              double source = j["generated_mono"].asDouble();
              if (j.get("waiting_for_scan_pose", false).asBool()) {
                if (source == plan_source &&
                    j["plan_id"] == plan_message["plan_id"] &&
                    mono() - plan_source <= .5 &&
                    plan_message.get("valid", false).asBool()) {
                  plan_message["state"] = "WAIT_SCAN_POSE";
                }
                // A wait notification never refreshes the command lease or
                // path.
                return;
              }
              if (!std::isfinite(source) || source <= plan_source ||
                  mono() - source < -.02 || mono() - source > .5)
                throw std::runtime_error("plan_stale");
              auto p = path_read(j["path"]);
              if (p.size() > 1000)
                throw std::runtime_error("plan_size");
              if (!j["valid"].isBool())
                throw std::runtime_error("plan_validity_missing");
              if (j["valid"].asBool()) {
                if (!j["lap_progress_m"].isNumeric() ||
                    !j["input_scan_stamp"].isNumeric() ||
                    !j["input_pose_stamp"].isNumeric())
                  throw std::runtime_error("plan_metadata_missing");
                double progress = j["lap_progress_m"].asDouble(),
                       ros = get_clock()->now().seconds();
                if (!std::isfinite(progress) || progress < 0 ||
                    progress > assets->arc.back() + 1e-6)
                  throw std::runtime_error("plan_progress_invalid");
                for (const char *key :
                     {"input_scan_stamp", "input_pose_stamp"}) {
                  double age = ros - j[key].asDouble();
                  if (!std::isfinite(age) || age < -.02 || age > .505)
                    throw std::runtime_error("planned_input_stale");
                }
                if (p.size() < 2 && assets->arc.back() - progress > .05)
                  throw std::runtime_error("plan_path_invalid");
              }
              plan_source = source;
              plan_receipt = mono();
              plan_message = j;
              local_path = std::move(p);
            } catch (...) {
              plan_receipt = -INF;
            }
          });
      control_timer =
          create_wall_timer(std::chrono::milliseconds(50), [this] { tick(); });
      watchdog_timer = create_wall_timer(std::chrono::milliseconds(20), [this] {
        auto i = snapshot();
        if (!ready(i, mono()))
          stop("input_stale");
        else if (mono() - plan_receipt > .5 || mono() - plan_source > .5)
          stop("planning_timeout");
      });
    }
  }
  ~Runtime() {
    stopping = true;
    if (worker.joinable())
      worker.join();
  }
  void shutdown_output() {
    if (!planning && rclcpp::ok()) {
      complete = false;
      emit(0, 0, false, "shutdown");
    }
  }
};
int main(int argc, char **argv) {
  try {
    rclcpp::init(argc, argv);
    bool planner = std::string(argv[0]).find("planner") != std::string::npos;
    auto node = std::make_shared<Runtime>(planner);
    rclcpp::spin(node);
    node->shutdown_output();
    node.reset();
    if (rclcpp::ok())
      rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
