#include "perception.hpp"
#include "joint_control.hpp"
#include "generic_avoidance.hpp"
#include "prearm_path_refresh.hpp"
#include "lead_identity.hpp"
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
  Json::Value observation;double observation_receipt=-INF;
  double window_receipt=-INF, window_source=-INF, window_started=-INF;
  bool window_latched=false;
  double terminal_receipt=-INF,terminal_source=-INF;
  bool controller_zero_latched=false;
};
} // namespace
class Runtime : public rclcpp::Node {
  bool planning;
  bool require_motion_window=false;
  std::string tracker_type;
  std::unique_ptr<Assets> assets;
  std::unique_ptr<Planner> planner;
  std::unique_ptr<Perception> perception;
  std::unique_ptr<Tracker> tracker;
  std::unique_ptr<JointControl> joint;
  std::unique_ptr<GenericAvoidance> generic;
  std::unique_ptr<LeadIdentity> lead_identity;
  std::mutex input_mutex;
  Input input;
  std::thread worker;
  std::mutex observation_mutex;Json::Value observed;
  std::vector<Track> observed_tracks;double observed_scan=-INF;
  std::atomic<bool> stopping{false};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub, odom_sub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr quality_sub, enable_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr plan_sub, window_sub, controller_state_sub, observation_sub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr plan_pub, status_pub, observation_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
      drive_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      obstacles_pub;
  rclcpp::TimerBase::SharedPtr control_timer, watchdog_timer, observation_timer;
  Json::Value plan_message;
  Json::Value control_input, rejected_control;
  Path local_path;
  ReferenceSpeedTable reference_speed_table;
  double plan_receipt = -INF, plan_source = -INF, last_tick = -INF,
         last_control = -INF;
  bool complete = false;
  bool terminal_stop_requested = false;
  double terminal_still_since = -INF;
  Pose terminal_still_pose;
  int logged_plan_id = -1;
  PrearmPathRefresh prearm_path_refresh;
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
    // Evaluate after the input snapshot, including calls whose C++ argument
    // evaluation order could otherwise sample mono() before snapshot().
    now=mono();
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
    const double drive_publish_begin=mono();
    drive_pub->publish(drive);
    const double drive_publish_end=mono();
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
    j["runtime_timing"]["drive_publish_ms"]=(drive_publish_end-drive_publish_begin)*1000.;
    if(reason=="ok" && tracker->diag.isMember("control_callback_start_mono"))
      j["runtime_timing"]["control_to_drive_publish_ms"]=(drive_publish_end-tracker->diag["control_callback_start_mono"].asDouble())*1000.;
    auto i = snapshot();
    j["input_age"] = ages(i);
    j["endpoint_distance_m"] = (i.pose.xy() - assets->route.back().xy()).norm();
    j["endpoint_heading_error_rad"] =
        wrap(i.pose.yaw - assets->route.back().yaw);
    j["observation_age_s"]=i.observation.isObject()?get_clock()->now().seconds()-i.observation["source_scan_stamp"].asDouble():1e9;
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
    const double tick_begin=mono();
    Json::Value stages(Json::objectValue);Path control_prediction;
    auto i = snapshot();
    double now = mono();
    control_input = Json::Value();
    rejected_control = Json::Value();
    try {
      if (!ready(i, now))
        throw std::runtime_error("input_stale");
      if (complete) {
        emit(0, 0, true, "route_complete");
        return;
      }
      // A requested terminal coast remains zero even after the independent
      // motion window latches. Arrival is acknowledged only after standstill,
      // with the validated position tolerance; final yaw is diagnostic only.
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
      if (now - plan_receipt > 1. || now - plan_source > 1.)
        throw std::runtime_error("planning_timeout");
      if(!observation_ready(i))throw std::runtime_error("observation_stale");
      if (!plan_message.get("valid", false).asBool())
        throw std::runtime_error(
            plan_message.get("reason", "planner_invalid").asString());
      if (!permit(i, now)) {
        tracker->reset();
    if(joint)joint->reset();
        last_tick = -INF;
        last_control = now;
        emit(0, 0, true, "waiting_for_operator");
        return;
      }
      // Operator enable precedes actual motor admission during the arm handshake.
      // Keep solving from rest until the output window completes its initial hold.
      // This preserves readiness without accumulating an unapplied speed ramp.
      bool launch_preview=false;
      if(require_motion_window && joint) {
        if(now-i.window_receipt>.5 || now-i.window_source>.5 || i.window_source>now+.02)
          throw std::runtime_error("motion_window_stale");
        if(i.window_latched) throw std::runtime_error("motion_window_latched");
        launch_preview=!std::isfinite(i.window_started) || now-i.window_started<.10;
        if(launch_preview)joint->reset();
      }
      double progress = plan_message["lap_progress_m"].asDouble(),
             remaining = std::max(0., assets->arc.back() - progress);
      if(generic && local_path.size()>1) { Geometry g(local_path,i.pose);remaining=std::max(0.,g.arc.back()-g.nearest_s); }
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
      const double speed_policy_begin=mono();
      double cap=generic->cap-(tracker_type=="pid"?.15:0.);
      double braking_cap=INF,controller_reference_cap=plan_message.get("following_speed_cap_mps",2.).asDouble();
      if(joint && generic->map_speed_feasibility) {
        const double path_speed_begin=mono();
        Geometry speed_geometry(local_path,i.pose,&*assets,generic->running_floor,&reference_speed_table);
        double ceiling=speed_geometry.target(speed_geometry.nearest_s,1e6,generic->cap-.15);
        stages["path_speed_preview_ms"]=(mono()-path_speed_begin)*1000.;
        const double state_coast_begin=mono();
        double state_ceiling=generic->state_coast_speed_cap(i.pose,joint->current_servo_estimate(),generic->cap-.15);
        stages["state_coast_cap_ms"]=(mono()-state_coast_begin)*1000.;
        ceiling=std::min(ceiling,std::max(generic->running_floor,state_ceiling-generic->speed_response_headroom));
        // Keep the absolute maximum independent of a changing reserve preview.
        // MPC turns the preview into a jerk-feasible stage braking envelope;
        // exact current-state coast and newest-observation guards still decide
        // whether any resulting command may be published.
        if(tracker_type=="dynamic_mpc") {
          braking_cap=state_ceiling;
          controller_reference_cap=std::min(controller_reference_cap,ceiling);
        } else cap=std::min(cap,ceiling);
        control_input["state_coast_advisory_cap_mps"]=state_ceiling;
        control_input["map_speed_ceiling_mps"]=ceiling;
      }
      stages["speed_policy_ms"]=(mono()-speed_policy_begin)*1000.;
      if(joint) {
        if(!plan_message["obstacle_points"].isArray())throw std::runtime_error("joint_obstacles_missing");
        std::vector<Point> obstacles;
        for(auto o:i.observation["static_obstacle_points"])obstacles.emplace_back(o[0].asDouble(),o[1].asDouble());
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
        double full_control_begin=speed_policy_begin;
        std::function<std::pair<Pose,double>(Pose,double)> dynamic_corridor;
        if(tracker_type=="dynamic_mpc") {
          const double prediction_now=get_clock()->now().seconds();
          control_input["mpc_prediction_now"]=prediction_now;
          control_input["braking_cap_mps"]=braking_cap;
          control_input["controller_reference_cap_mps"]=controller_reference_cap;
          const double preparation_begin=mono();
          auto prepared=generic->prepare_observations(i.observation["occupancies"],prediction_now);
          stages["observation_preparation_ms"]=(mono()-preparation_begin)*1000.;
          dynamic_corridor=[&,prepared=std::move(prepared)](Pose reference,double future){return generic->prediction_corridor_prepared(reference,prepared,future);};
        }
        // Preserve the actual solver observation even when compute throws;
        // post-solve guard time is still sampled independently below.
        control_input["collision_guard_observation"]=i.observation;
        auto control=joint->compute(*assets,local_path,i.pose,i.speed,i.lateral,i.rate,remaining,cap,dt,*tracker,obstacles,controller_reference_cap,dynamic_corridor,generic->map_speed_feasibility?&reference_speed_table:nullptr,braking_cap);
        control_input["collision_guard_now"]=get_clock()->now().seconds();
        control_prediction=path_read(joint->diag["predicted_path"]);
        const double prediction_begin=mono();
        generic->check_prediction(control_prediction,i.observation["occupancies"],control_input["collision_guard_now"].asDouble());
        stages["primary_prediction_guard_ms"]=(mono()-prediction_begin)*1000.;
        const double stop_speed=std::max(std::abs(i.speed),control.speed);
        const double stop_servo=joint->diag.get("sweep_initial_servo_rad",0.).asDouble();
        joint->diag["emergency_coast_check_speed_mps"]=stop_speed;
        const double emergency_begin=mono();
        joint->diag["emergency_coast_min_map_reserve_m"]=generic->check_emergency_map_coast(i.pose,stop_speed,stop_servo);
        stages["emergency_coast_guard_ms"]=(mono()-emergency_begin)*1000.;
        joint->diag["controller_and_dynamic_guard_ms"]=(mono()-full_control_begin)*1000.;
        joint->diag["full_control_hard_deadline_ms"]=1000*MpcTimingBudget::hard_seconds();
        if(mono()-full_control_begin>MpcTimingBudget::hard_seconds())throw std::runtime_error("compact_control_deadline");
        speed=control.speed;angle=control.angle;
        joint->diag["launch_preview_only"]=launch_preview;
        joint->diag["motion_window_required"]=require_motion_window;
        tracker->diag=joint->diag;
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
      if(mono()-plan_source>1. || mono()-plan_receipt>1.)throw std::runtime_error("planning_timeout");
      if(!observation_ready(latest))throw std::runtime_error("observation_stale");
      // Revalidate against the newest observation even if it arrived while
      // the controller was solving. No timestamp-only path lease renewal.
      if(joint) {
        control_input["collision_guard_observation"]=latest.observation;
        control_input["collision_guard_now"]=get_clock()->now().seconds();
        const double newest_begin=mono();
        generic->check_prediction(control_prediction,latest.observation["occupancies"],control_input["collision_guard_now"].asDouble());
        stages["newest_prediction_guard_ms"]=(mono()-newest_begin)*1000.;
      }
      if (!permit(latest, mono())) {
        tracker->reset();
    if(joint)joint->reset();
        emit(0, 0, true, "waiting_for_operator");
        return;
      }
      last_control = mono();
      plan_message["velocity_policy"] = joint?"controller_longitudinal":"legacy_profile";
      if(joint) {
        joint->diag["node_stage_ms"]=stages;
        joint->diag["control_callback_start_mono"]=tick_begin;
        joint->diag["control_to_emit_ms"]=(mono()-tick_begin)*1000.;
        tracker->diag=joint->diag;
        // Include the final newest-observation revalidation, not just the
        // earlier primary guard. DDS transport is measured separately.
        if(mono()-tick_begin>MpcTimingBudget::hard_seconds())throw std::runtime_error("compact_control_deadline");
      }
      emit(speed, angle, true, "ok");
    } catch (const std::exception &e) {
      if (joint && control_input.isObject()) {
        joint->diag["node_stage_ms"]=stages;
        joint->diag["control_to_emit_ms"]=(mono()-tick_begin)*1000.;
        rejected_control["reason"] = e.what();
        rejected_control["diagnostics"] = joint->diag;
        if(std::string(e.what())=="generic_predicted_occupancy_conflict")
          rejected_control["collision"]=generic->collision_diagnostic;
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
    std::vector<Track> marker_tracks;{std::lock_guard<std::mutex> g(observation_mutex);marker_tracks=observed_tracks;}
    for (auto t : marker_tracks)
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
  void observe_tick() {
    auto i=snapshot();double now=mono();
    if(!ready(i,now) || i.ss<=observed_scan)return;
    try {
      double span=0;Pose q=i.history.at(i.ss,span);
      auto points=perception->scan(i.scan->ranges,i.scan->angle_min,i.scan->angle_increment,
        i.scan->range_min,i.scan->range_max,i.ss,q);
      generic->observe(perception->tracks,i.ss,i.scan->ranges,i.scan->angle_min,
        i.scan->angle_increment,i.scan->range_min,i.scan->range_max,q);
      Json::Value o=perception->diagnostic();o["valid"]=true;o["frame_id"]="map";
      o["source_scan_stamp"]=i.ss;o["source_pose_stamp"]=i.ps;
      o["source_scan_mono"]=i.sr;o["generated_mono"]=mono();
      o["scan_pose"]=path_json(Path{q})[0];o["scan_pose_span_s"]=span;
      const double identity_begin=mono();
      o["occupancies"]=lead_identity->apply(generic->snapshot(),points,q,i.ss);
      o["lead_identity"]=lead_identity->status();
      o["lead_identity_ms"]=(mono()-identity_begin)*1000.;
      o["memory"]=generic->memory_diagnostic();
      o["obstacle_points"]=Json::Value(Json::arrayValue);
      o["static_obstacle_points"]=Json::Value(Json::arrayValue);
      for(auto point:points){
        Json::Value v(Json::arrayValue);v.append(point.x());v.append(point.y());o["obstacle_points"].append(v);
        bool moving_support=false;
        for(const auto& item:o["occupancies"]){
          if(item.get("visibility_hazard",false).asBool() || !item.get("has_moved",false).asBool() || item.get("observation_age_s",1.).asDouble()>.08)continue;
          if(point.x()>=item["xmin"].asDouble()-.02 && point.x()<=item["xmax"].asDouble()+.02 && point.y()>=item["ymin"].asDouble()-.02 && point.y()<=item["ymax"].asDouble()+.02){moving_support=true;break;}
        }
        if(!moving_support)o["static_obstacle_points"].append(v);
      }
      // Moving supports are omitted ONLY from the static future sweep.
      // Full observed occupancy is mandatory in time-aligned validation.

      {
        std::lock_guard<std::mutex> g(observation_mutex);observed=o;observed_tracks=perception->tracks;
      }
      observed_scan=i.ss;std_msgs::msg::String msg;msg.data=dump(o);observation_pub->publish(msg);
    } catch(const std::exception& e) {
      // A pending scan/pose alignment does not renew the previous lease.
      // Every consumer expires the unchanged source stamps within 500ms.
      if(std::string(e.what())!="scan_pose_unsynchronized"){
        Json::Value o;o["valid"]=false;o["reason"]=e.what();o["generated_mono"]=mono();
        std_msgs::msg::String msg;msg.data=dump(o);observation_pub->publish(msg);
      }
    }
  }
  bool observation_ready(const Input& i) {
    if(!i.observation.isObject() || !i.observation.get("valid",false).asBool())return false;
    double now=mono(),ros=get_clock()->now().seconds();
    return now-i.observation_receipt>=0 && now-i.observation_receipt<=.5 &&
      ros-i.observation["source_scan_stamp"].asDouble()>=-.02 && ros-i.observation["source_scan_stamp"].asDouble()<=.5 &&
      ros-i.observation["source_pose_stamp"].asDouble()>=-.02 && ros-i.observation["source_pose_stamp"].asDouble()<=.5;
  }
  void plan_loop() {
    Progress progress;
    Candidate last_safe;
    Json::Value last_plan_envelope;
    double used_scan = -INF;
    int blocked_count = 0;
    while (rclcpp::ok() && !stopping) {
      auto i = snapshot();
      double begin = mono();
      Json::Value out;
      double search_scan=-INF,search_pose=-INF;
      bool waiting = false;
      try {
        if (!ready(i, begin))
          throw std::runtime_error("input_stale");
        if (i.ss <= used_scan) {
          // No new sensor sample is not a new plan. Keep the old lease intact.
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          continue;
        }
        Json::Value observation;
        {std::lock_guard<std::mutex> g(observation_mutex);observation=observed;}
        if(!observation.isObject() || !observation.get("valid",false).asBool())
          throw std::runtime_error("observation_not_ready");
        if(get_clock()->now().seconds()-observation["source_scan_stamp"].asDouble()>.5)
          throw std::runtime_error("observation_stale");
        if(observation["source_scan_stamp"].asDouble()<=used_scan){std::this_thread::sleep_for(std::chrono::milliseconds(10));continue;}
        used_scan=observation["source_scan_stamp"].asDouble();
        search_scan=used_scan;search_pose=observation["source_pose_stamp"].asDouble();
        double span=observation["scan_pose_span_s"].asDouble();
        Json::Value poses(Json::arrayValue);poses.append(observation["scan_pose"]);
        Pose scan_pose=path_read(poses).at(0);
        std::vector<Point> points;for(auto v:observation["obstacle_points"])points.emplace_back(v[0].asDouble(),v[1].asDouble());
        double scan_time=0.,plan_begin=mono();
        const bool refresh_prearm=prearm_path_refresh.update(i.enabled,i.speed,i.pose);
        if(refresh_prearm)generic->refresh_prearm_path();
        Candidate candidate=generic->plan_occupancies(*planner,i.pose,used_scan,i.speed,observation["occupancies"],generic->preferred_gap);
        Json::Value generic_status=observation;
        generic_status["generic_avoidance"]=true;
        generic_status["route_aligned_opponent_prior"]=generic->route_aligned_opponent_prior;
        generic_status["wall_margin_m"]=assets->wall_margin;
        generic_status["planning_pose"]=path_json(Path{i.pose})[0];
        generic_status["planning_speed_mps"]=i.speed;
        generic_status["prearm_path_refreshed"]=refresh_prearm;
        generic_status["prearm_refresh_policy"]="initial_or_pose_change_3cm_0_05rad;revalidate_every_scan";
        generic_status["obstacle_motion_model"]="zero_static_growth_route_bounded_2cm_1cm_with_unknown_fallback";
        generic_status["static_process_inflation_m"]=0.;
        generic_status["static_occlusion_policy"]="all_confirmed_static_hold_until_motion_or_free_ray_clearance";
        generic_status["route_moving_longitudinal_inflation_max_m"]=.02;
        generic_status["route_moving_lateral_inflation_max_m"]=.01;
        generic_status["revalidated_on_newest_scan"]=false;
        generic_status["command_cap_mps"]=generic->cap-.15;
        generic_status["max_lateral_offset_m"]=generic->max_offset;
        generic_status["desired_body_gap_m"]=generic->preferred_gap;
        generic_status["running_floor_mps"]=generic->running_floor;
        generic_status["corridor_seed_reserve_m"]=generic->seed_reserve;
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
          blocked_count = 0;
        } else if (!finished) {
          last_safe=Candidate{};
          blocked_count++;
        }
        Json::Value prepared_speed_table;
        const double speed_table_begin=mono();
        if(candidate.valid && candidate.path.size()>1 && generic->map_speed_feasibility) {
          Geometry geometry(candidate.path,i.pose,&*assets,generic->running_floor);
          prepared_speed_table=geometry.precompute_reference_limits();
        }
        const double speed_table_ms=(mono()-speed_table_begin)*1000.;
        if (mono() - begin > .5)
          throw std::runtime_error("planning_timeout");
        if (!ready(snapshot(), mono()))
          throw std::runtime_error("input_stale");
        out = Json::Value(Json::objectValue);
        for(const auto& key:generic_status.getMemberNames())out[key]=generic_status[key];
        out["valid"] = candidate.valid || finished;
        out["reason"] = finished ? "route_complete" : candidate.reason;
        out["path"] = path_json(candidate.path);
        out["reference_speed_table"]=prepared_speed_table;
        out["reference_speed_preparation_ms"]=speed_table_ms;
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
        out["following_speed_cap_mps"]=candidate.terms.get("following_speed_cap_mps",2.);
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
        out["route_mode"] = "open_stop_at_end";
        out["scan_pose_alignment_span_s"] = span;
        out["scan_pose_at_source"] = path_json(Path{scan_pose})[0];
        out["scan_pose_source_stamp"] = search_scan;
        out["last_plan_duration_s"] = mono() - begin;
        out["last_plan_stages"]["scan_processing_s"] = scan_time;
        out["last_plan_stages"]["lattice_search_s"] = plan_time;
        out["last_plan_stages"]["compute_wall_s"] = mono() - begin;
        out["last_plan_stages"]["total_wall_s"] = mono() - begin;
        out["candidate_reason_counts"] = Json::Value(Json::objectValue);
        out["candidate_collision_diagnostics"]=Json::Value(Json::arrayValue);
        for(const auto& c:planner->candidates)
          if(c.terms.isMember("collision_diagnostic") && out["candidate_collision_diagnostics"].size()<4)
            out["candidate_collision_diagnostics"].append(c.terms["collision_diagnostic"]);
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
          {std::lock_guard<std::mutex> g(observation_mutex);out=observed;}
          out["valid"] = false;
          out["reason"] = e.what();
          out["path"] = Json::Value(Json::arrayValue);
          last_safe = Candidate{};
        }
      }
      if (!waiting) {
        out["generated_mono"] = mono();
        out["input_scan_stamp"] =
            std::isfinite(search_scan) ? Json::Value(search_scan) : Json::Value();
        out["input_pose_stamp"] =
            std::isfinite(search_pose) ? Json::Value(search_pose) : Json::Value();
        out["frame_id"] = "map";
        out["plan_id"] = Json::UInt64(++plan_sequence);
        if(out["reference_speed_table"].isObject())out["reference_speed_table"]["plan_id"]=out["plan_id"];
        last_plan_envelope = out;
      }
      if(out["input_scan_stamp"].isNumeric() && out["input_pose_stamp"].isNumeric()) {
        const double publish_ros=get_clock()->now().seconds();
        out["source_age_at_publication_s"]["scan"]=publish_ros-out["input_scan_stamp"].asDouble();
        out["source_age_at_publication_s"]["pose"]=publish_ros-out["input_pose_stamp"].asDouble();
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
      // A 200 ms start-to-start period, not 200 ms EXTRA after computing.
      // Slow lattice searches must not add an idle delay that ages the last
      // valid plan beyond its unchanged 0.5 s source-data lease.
      const double next_plan = begin + (waiting ? .01 : .2);
      while (!stopping && rclcpp::ok() && mono() < next_plan)
        std::this_thread::sleep_for(std::chrono::duration<double>(
            std::min(.01, std::max(0., next_plan - mono()))));
    }
  }

public:
  explicit Runtime(bool planner_role)
      : Node(planner_role ? "frenet_cpp_planner" : "frenet_cpp_controller"),
        planning(planner_role) {
    std::string file = declare_parameter<std::string>("assets_file", "");
    tracker_type = declare_parameter<std::string>("tracker", "dynamic_mpc");
    require_motion_window=declare_parameter<bool>("require_motion_window",false);
    assets.reset(new Assets(file));
    generic.reset(new GenericAvoidance(*assets));
    const auto runtime_config=json_read(file.substr(0,file.find_last_of('/')+1)+"compact_config.json");
    lead_identity.reset(new LeadIdentity(*assets,runtime_config["lead_identity_v1"]));
    generic->running_floor=runtime_config.get("generic_running_floor_mps",.5).asDouble();
    if(!std::isfinite(generic->running_floor) || generic->running_floor<.5 || generic->running_floor>1.)
      throw std::runtime_error("generic_running_floor");
    const double effective_cap=runtime_config.get("generic_first_trial_effective_cap_mps",1.25).asDouble();
    const double lateral_limit=runtime_config.get("generic_max_lateral_offset_m",.75).asDouble();
    if(!std::isfinite(effective_cap+lateral_limit) || effective_cap<generic->running_floor || effective_cap>2. || lateral_limit<.3 || lateral_limit>1.0)
      throw std::runtime_error("generic_commissioning_profile");
    generic->cap=effective_cap+.15;generic->max_offset=lateral_limit;
    generic->stationary_scene=runtime_config.get("generic_stationary_scene_v1",false).asBool();
    generic->route_aligned_opponent_prior=runtime_config.get("route_aligned_opponent_prior_v1",false).asBool();
    if(generic->stationary_scene && runtime_config.get("first_physical_scope","").asString()!="stationary_obstacle_only")
      throw std::runtime_error("stationary_scene_scope_mismatch");
    generic->map_speed_feasibility=runtime_config["mpc_tracking_v10"].get("map_speed_feasibility",false).asBool();
    generic->speed_response_headroom=runtime_config.get("generic_speed_response_headroom_mps",.20).asDouble();
    if(!std::isfinite(generic->speed_response_headroom) || generic->speed_response_headroom<0 || generic->speed_response_headroom>.4)
      throw std::runtime_error("generic_speed_response_headroom");
    generic->preferred_gap=runtime_config["adaptive_corridor_v2"].get("desired_body_gap_m",.14).asDouble();
    if(!std::isfinite(generic->preferred_gap) || generic->preferred_gap<.08 || generic->preferred_gap>.20)
      throw std::runtime_error("generic_preferred_body_gap");
    generic->seed_reserve=runtime_config.get("generic_corridor_seed_reserve_m",.04).asDouble();
    if(!std::isfinite(generic->seed_reserve) || generic->seed_reserve<.02 || generic->seed_reserve>.12)
      throw std::runtime_error("generic_corridor_seed_reserve");
    if(tracker_type!="pid" && tracker_type!="dynamic_mpc")throw std::runtime_error("generic_requires_pid_or_dynamic_mpc");
    if(planning)controller_state_sub=create_subscription<std_msgs::msg::String>(
      "/baseline/controller_status",1,[this](std_msgs::msg::String::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> g(input_mutex);
        try {
          Json::Value v;Json::Reader reader;
          if(!reader.parse(msg->data,v) || !v["runtime_timing"]["publish_mono"].isNumeric())
            throw std::runtime_error("terminal_state_schema");
          double source=v["runtime_timing"]["publish_mono"].asDouble();
          if(!std::isfinite(source) || source<=input.terminal_source || source>mono()+.02)
            throw std::runtime_error("terminal_state_stamp");
          input.terminal_source=source;input.terminal_receipt=mono();
          input.controller_zero_latched=v.get("valid",false).asBool() &&
            v["speed_command"].isNumeric() && v["speed_command"].asDouble()==0. &&
            (v.get("reason","").asString()=="terminal_coast" ||
             v.get("reason","").asString()=="route_complete");
        } catch(...) {input.controller_zero_latched=false;}
      });
    tracker.reset(new Tracker(tracker_type));
    if(!planning && tracker_type!="pp") {
      auto tuning=runtime_config["mpc_tracking_v10"];
      tuning["minimum_drivable_speed_mps"]=generic->running_floor;
      joint.reset(new JointControl(tracker_type=="dynamic_mpc",runtime_config.get("joint_feasibility_recovery_v9",false).asBool(),tuning));
    }
    planner.reset(new Planner(*assets, (tracker_type=="pid" || tracker_type=="dynamic_mpc") ? .03 : 0.));
    perception.reset(new Perception(*assets,json_read(file.substr(0,file.find_last_of('/')+1)+"compact_config.json").get("ego_body_filter_v11",false).asBool(),true));
    const char *domain = std::getenv("ROS_DOMAIN_ID");
    const char *local = std::getenv("ROS_LOCALHOST_ONLY");
    if (!domain || !local || std::string(local) != "1" ||
        (std::string(domain) != "83" && std::string(domain) != "85"))
      throw std::runtime_error("runtime_domain_refused");
    if(std::string(domain)=="85" && !json_read(file.substr(0,file.find_last_of('/')+1)+"compact_config.json").get("physical_commissioning_enabled",false).asBool())
      throw std::runtime_error("compact_physical_not_commissioned");
    if (std::string(domain) == "85" &&
        (!std::getenv("F1TENTH_GROUND_AVOIDANCE_TEST") ||
         std::string(std::getenv("F1TENTH_GROUND_AVOIDANCE_TEST")) !=
             "bounded_25s_L1"))
      throw std::runtime_error("physical_scope_missing");
    if(std::string(domain)=="85" && !require_motion_window)
      throw std::runtime_error("physical_motion_window_required");
    if(!planning && require_motion_window) window_sub=create_subscription<std_msgs::msg::String>(
      "/stand_pf/window_status",1,[this](std_msgs::msg::String::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> g(input_mutex);
        try {
          Json::Value v;Json::Reader reader;
          if(!reader.parse(msg->data,v) || !v["published_mono"].isNumeric() || !v["latched"].isBool()
             || !(v["started"].isNull() || v["started"].isNumeric()))
            throw std::runtime_error("window_status_invalid");
          double source=v["published_mono"].asDouble();
          double started=v["started"].isNull()?-INF:v["started"].asDouble();
          if(!std::isfinite(source) || source<=input.window_source ||
             (!v["started"].isNull() && (!std::isfinite(started) || started>source+.02)))
            throw std::runtime_error("window_timestamp_invalid");
          input.window_source=source;input.window_receipt=mono();
          input.window_started=started;input.window_latched=v["latched"].asBool();
        }catch(...){input.window_receipt=-INF;}
      });
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
    if(!planning)observation_sub=create_subscription<std_msgs::msg::String>(
      "/baseline/observation_cpp",1,[this](std_msgs::msg::String::ConstSharedPtr msg){
        try{Json::Value v;Json::Reader reader;if(msg->data.size()>1000000 || !reader.parse(msg->data,v))throw std::runtime_error("observation_schema");
          if(!v.get("valid",false).asBool())throw std::runtime_error("observation_invalid");
          if(!v["source_scan_stamp"].isNumeric() || !v["source_pose_stamp"].isNumeric())throw std::runtime_error("observation_schema");
          if(!v["occupancies"].isArray() || v["occupancies"].size()>128 || !v["obstacle_points"].isArray() || !v["static_obstacle_points"].isArray() || v["static_obstacle_points"].size()>10000)throw std::runtime_error("observation_schema");
          for(const auto& point:v["obstacle_points"])if(!point.isArray() || point.size()!=2 || !point[0].isNumeric() || !point[1].isNumeric() || !std::isfinite(point[0].asDouble()+point[1].asDouble()))throw std::runtime_error("observation_points");
          if(v["obstacle_points"].size()>10000)throw std::runtime_error("observation_capacity");
          double source=v["source_scan_stamp"].asDouble(),pose=v["source_pose_stamp"].asDouble(),ros=get_clock()->now().seconds();
          if(!std::isfinite(source+pose) || ros-source>.5 || ros-pose>.5 || ros-source<-.02 || ros-pose<-.02)throw std::runtime_error("observation_stale");
          std::lock_guard<std::mutex> g(input_mutex);
          if(input.observation.isObject() && source<=input.observation["source_scan_stamp"].asDouble())return;
          input.observation=v;input.observation_receipt=mono();
        }catch(...){std::lock_guard<std::mutex> g(input_mutex);input.observation_receipt=-INF;}
      });
    if (planning) {
      observation_pub=create_publisher<std_msgs::msg::String>("/baseline/observation_cpp",1);
      observation_timer=create_wall_timer(std::chrono::milliseconds(50),[this]{observe_tick();});
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
                  // Keep the admitted path's original driving state and cap.
                  // Waiting for scan alignment does not constitute a new
                  // blockage plan. A sudden lower hard command bound can be
                  // unreachable under the unchanged acceleration/jerk limits.
                  // Preserve all source/receipt timestamps: tick() still stops
                  // on the original 0.5 s plan/input lease expiry.
                  plan_message["waiting_for_scan_pose"] = true;
                }
                // A wait notification never refreshes the command lease or
                // path.
                return;
              }
              if (!std::isfinite(source) || source <= plan_source ||
                  mono() - source < -.02 || mono() - source > 1.)
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
                for(const char* key:{"input_scan_stamp","input_pose_stamp"}){
                  double age=ros-j[key].asDouble();if(!std::isfinite(age)||age<-.02||age>1.)throw std::runtime_error("proposal_input_stale");
                }
                if (p.size() < 2 && assets->arc.back() - progress > .05)
                  throw std::runtime_error("plan_path_invalid");
              }
              ReferenceSpeedTable admitted;
              if(j["valid"].asBool() && p.size()>1 && generic && generic->map_speed_feasibility) {
                if(j["reference_speed_table"]["plan_id"]!=j["plan_id"])throw std::runtime_error("reference_speed_table_version");
                admitted=Geometry::admit_reference_limits(j["reference_speed_table"],p,*assets,generic->running_floor);
              }
              reference_speed_table=std::move(admitted);
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
        else if (!complete && !terminal_stop_requested && (mono() - plan_receipt > 1. || mono() - plan_source > 1.))
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
