#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "joint_control.hpp"
#include "takeover_sim/dynamic_overtaking.hpp"

namespace {

double stamp_seconds(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

} // namespace

class TakeoverControllerNode : public rclcpp::Node {
public:
  TakeoverControllerNode() : Node("takeover_controller") {
    declare_parameter<std::string>("assets_path", "");
    declare_parameter<std::string>("mode", "takeover");
    declare_parameter<std::string>("ego_odom_topic", "/ego_racecar/odom");
    declare_parameter<std::string>("target_topic", "/ego_racecar/opp_odom_ekf");
    declare_parameter<std::string>("target_status_topic", "/ego_tracker/status");
    declare_parameter<std::string>("drive_topic", "/drive_nominal");
    declare_parameter<double>("cruise_speed", 1.5);
    declare_parameter<double>("control_rate_hz", 20.0);
    declare_parameter<double>("planning_rate_hz", 5.0);
    declare_parameter<double>("target_timeout", 1.2);
    declare_parameter<double>("pass_offset", 0.82);
    declare_parameter<double>("vehicle_length", 0.58);
    declare_parameter<double>("vehicle_width", 0.31);
    declare_parameter<bool>("track_assisted_prediction", true);
    declare_parameter<bool>("timed_control", true);
    declare_parameter<int>("allowed_pass_side", 0);

    const std::string assets_path = get_parameter("assets_path").as_string();
    if (assets_path.empty()) throw std::runtime_error("assets_path_required");
    mode_ = get_parameter("mode").as_string();
    if (mode_ != "takeover" && mode_ != "static_v8")
      throw std::runtime_error("mode_must_be_takeover_or_static_v8");

    assets_ = std::make_unique<frenet::Assets>(assets_path);
    static_planner_ = std::make_unique<frenet::Planner>(*assets_, 0.03);
    dynamic_planner_ = std::make_unique<takeover::DynamicOvertakingPlanner>(*assets_);
    dynamic_planner_->cruise_speed = get_parameter("cruise_speed").as_double();
    dynamic_planner_->detection_timeout = get_parameter("target_timeout").as_double();
    dynamic_planner_->pass_offset = get_parameter("pass_offset").as_double();
    dynamic_planner_->vehicle_length = get_parameter("vehicle_length").as_double();
    dynamic_planner_->vehicle_width = get_parameter("vehicle_width").as_double();
    dynamic_planner_->track_assisted_prediction = get_parameter("track_assisted_prediction").as_bool();
    dynamic_planner_->timed_control = get_parameter("timed_control").as_bool();
    dynamic_planner_->allowed_pass_side = get_parameter("allowed_pass_side").as_int();
    if (std::abs(dynamic_planner_->allowed_pass_side) > 1)
      throw std::runtime_error("allowed_pass_side_must_be_minus1_0_or_1");
    controller_ = std::make_unique<frenet::JointControl>(true);
    tracker_ = std::make_unique<frenet::Tracker>("dynamic_mpc");

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        get_parameter("drive_topic").as_string(), 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/takeover/status", 10);
    reference_pub_ = create_publisher<nav_msgs::msg::Path>("/takeover/reference_path", 10);
    lead_prediction_pub_ = create_publisher<nav_msgs::msg::Path>("/takeover/lead_prediction", 10);

    ego_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        get_parameter("ego_odom_topic").as_string(), 10,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { ego_ = msg; });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
          if (!msg->ranges.empty()) last_scan_time_ = now().seconds();
        });
    target_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        get_parameter("target_topic").as_string(), 10,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { update_track(*msg); });
    target_status_sub_ = create_subscription<std_msgs::msg::String>(
        get_parameter("target_status_topic").as_string(), 10,
        [this](std_msgs::msg::String::SharedPtr msg) { update_tracker_status(msg->data); });

    const double control_rate = get_parameter("control_rate_hz").as_double();
    planning_period_ = 1.0 / get_parameter("planning_rate_hz").as_double();
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / control_rate),
                               [this]() { control_tick(); });
    RCLCPP_INFO(get_logger(), "Loaded v8 assets from %s in %s mode",
                assets_path.c_str(), mode_.c_str());
  }

private:
  std::string mode_;
  std::unique_ptr<frenet::Assets> assets_;
  std::unique_ptr<frenet::Planner> static_planner_;
  std::unique_ptr<takeover::DynamicOvertakingPlanner> dynamic_planner_;
  std::unique_ptr<frenet::JointControl> controller_;
  std::unique_ptr<frenet::Tracker> tracker_;
  nav_msgs::msg::Odometry::SharedPtr ego_;
  takeover::Track track_;
  takeover::Track measured_track_;
  std::string tracker_mode_ = "UNINITIALIZED";
  double tracker_measurement_age_ = frenet::INF;
  double last_plan_time_ = -frenet::INF;
  double last_control_time_ = -frenet::INF;
  double latest_target_message_time_ = -frenet::INF;
  double last_scan_time_ = -frenet::INF;
  double planning_period_ = 0.2;
  bool stopped_ = true;
  int controller_failures_ = 0;
  int planner_failures_ = 0;
  int solver_timeouts_ = 0;
  int infeasible_count_ = 0;
  int solver_recoveries_ = 0;
  int rejected_track_jumps_ = 0;
  int freshness_triggered_replans_ = 0;
  takeover::Decision decision_;
  frenet::Path reference_path_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr lead_prediction_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void update_track(const nav_msgs::msg::Odometry &msg) {
    const double receive_time = now().seconds();
    const takeover::Point measured_position{
        msg.pose.pose.position.x, msg.pose.pose.position.y};
    const bool committed = decision_.behavior == takeover::Behavior::OVERTAKE ||
                           decision_.behavior == takeover::Behavior::REJOIN ||
                           decision_.overtakes_completed > 0;
    if (track_.valid && committed && latest_target_message_time_ > 0.0 &&
        receive_time-track_.observation_stamp < dynamic_planner_->committed_prediction_timeout) {
      const takeover::Point expected = prediction_track(receive_time).position;
      // Reject a global re-acquisition jump while the passing side is locked.
      // This uses only innovation against the accepted CV track, never truth.
      if ((measured_position - expected).norm() > 0.90) {
        ++rejected_track_jumps_;
        return;
      }
    }
    latest_target_message_time_ = receive_time;
    track_.position = measured_position;
    track_.velocity = {msg.twist.twist.linear.x, msg.twist.twist.linear.y};
    track_.yaw = yaw_from_quaternion(msg.pose.pose.orientation);
    track_.stamp = stamp_seconds(msg.header.stamp);
    track_.position_variance = std::max({0.0, msg.pose.covariance[0], msg.pose.covariance[7]});
    track_.velocity_variance = std::max({0.0, msg.twist.covariance[0], msg.twist.covariance[7]});
    track_.valid = true;
    track_.measured = tracker_mode_ == "TRACKED";
    track_.observation_stamp = latest_target_message_time_ - tracker_measurement_age_;
    if (track_.measured && tracker_measurement_age_ < .15) measured_track_ = track_;
  }

  void update_tracker_status(const std::string &text) {
    Json::CharReaderBuilder builder;
    Json::Value value;
    std::string error;
    std::istringstream input(text);
    if (!Json::parseFromStream(builder, input, &value, &error)) return;
    tracker_mode_ = value.get("mode", "INVALID").asString();
    tracker_measurement_age_ = value["measurement_age_s"].isNumeric()
        ? std::max(0.0, value["measurement_age_s"].asDouble()) : frenet::INF;
    track_.measured = tracker_mode_ == "TRACKED";
    // observation_stamp is updated only alongside an accepted state message;
    // a status-only TRACKED transition must not refresh a rejected jump.
  }

  frenet::Pose ego_pose() const {
    return {ego_->pose.pose.position.x, ego_->pose.pose.position.y,
            yaw_from_quaternion(ego_->pose.pose.orientation)};
  }

  double ego_speed() const {
    return std::hypot(ego_->twist.twist.linear.x, ego_->twist.twist.linear.y);
  }

  takeover::Track prediction_track(double time) const {
    // During a dropout, forecast from the last observed state along the track,
    // not from the filter's repeatedly straight-extrapolated Cartesian output.
    auto track = (dynamic_planner_->track_assisted_prediction &&
                  tracker_mode_ != "TRACKED" && measured_track_.valid) ? measured_track_ : track_;
    if (!track.valid) return track;
    const auto forecast = takeover::predict_track(*assets_, track, .1, .1,
        dynamic_planner_->process_sigma_rate, dynamic_planner_->track_assisted_prediction,
        std::max(0.,time-track.stamp));
    track.position=forecast[0].pose.xy();
    track.velocity=(forecast[1].pose.xy()-forecast[0].pose.xy())/.1;
    track.yaw=forecast[0].pose.yaw;
    track.position_variance=forecast[0].sigma*forecast[0].sigma;
    track.stamp=time;
    return track;
  }

  void plan(double time) {
    const frenet::Pose pose = ego_pose();
    if (mode_ == "static_v8" || !track_.valid) {
      const double begin = frenet::mono();
      auto candidate = static_planner_->plan(pose, {}, 3.2);
      decision_ = takeover::Decision{};
      decision_.behavior = candidate.valid ? takeover::Behavior::CRUISE : takeover::Behavior::WAIT;
      decision_.candidate.valid = candidate.valid;
      decision_.candidate.reason = candidate.reason;
      decision_.candidate.path = candidate.path;
      decision_.candidate.speed_cap = get_parameter("cruise_speed").as_double();
      decision_.candidate.side = candidate.offset > 0 ? 1 : candidate.offset < 0 ? -1 : 0;
      decision_.candidate.minimum_map_clearance = candidate.stat;
      decision_.stop = !candidate.valid;
      decision_.planning_ms = (frenet::mono() - begin) * 1000.0;
      decision_.transition = static_planner_->switch_reason;
    } else {
      dynamic_planner_->initial_command=controller_->command_speed();
      dynamic_planner_->initial_acceleration=controller_->command_acceleration();
      decision_ = dynamic_planner_->plan(pose, ego_speed(), prediction_track(time), time);
    }
    reference_path_ = decision_.candidate.path;
    if (!decision_.candidate.valid) ++planner_failures_;
    publish_path(reference_pub_, reference_path_);
    publish_lead_prediction(decision_.target_prediction);
    last_plan_time_ = time;
  }

  void control_tick() {
    const double time = now().seconds();
    if (!ego_) {
      publish_stop("waiting_for_ego_odometry");
      return;
    }
    if (time - last_scan_time_ > 0.3 || time - stamp_seconds(ego_->header.stamp) > 0.3) {
      publish_stop("sensor_stale");
      return;
    }
    const bool freshness_replan = mode_ != "static_v8" && track_.valid &&
        takeover::freshness_replan_required(decision_, prediction_track(time), time,
            dynamic_planner_->detection_timeout,
            dynamic_planner_->committed_prediction_timeout);
    if (freshness_replan) ++freshness_triggered_replans_;
    if (time - last_plan_time_ >= planning_period_ || freshness_replan) plan(time);
    if (decision_.stop || !decision_.candidate.valid || reference_path_.size() < 2) {
      publish_stop(decision_.candidate.reason.empty() ? "planner_stop" : decision_.candidate.reason);
      publish_status(time, Json::Value());
      return;
    }

    const double dt = last_control_time_ > 0.0 ? frenet::clip(time - last_control_time_, 0.01, 0.10) : 0.05;
    last_control_time_ = time;
    Json::Value diagnostic;
    try {
      const auto &twist = ego_->twist.twist;
      const double speed = ego_speed();
      const double yaw = ego_pose().yaw;
      // Gym odometry twist is already in child_frame_id (vehicle body).
      const double lateral = twist.linear.y;
      const double remaining = 100.0;
      frenet::TimedReference timed;
      const bool use_timed = dynamic_planner_->timed_control && !decision_.candidate.timed.empty();
      if(use_timed) {
        timed.age=std::max(0.,time-last_plan_time_);
        for(const auto &p:decision_.candidate.timed) timed.points.push_back({p.t,p.pose,p.speed});
      }
      const double command_cap=use_timed?dynamic_planner_->cruise_speed:decision_.candidate.speed_cap;
      frenet::Control command;
      try {
        command = controller_->compute(
            *assets_, reference_path_, ego_pose(), speed, lateral, twist.angular.z,
            remaining, command_cap, dt, *tracker_, {}, use_timed?&timed:nullptr);
      } catch (const std::exception &first_error) {
        // acados status 4 can be a stale warm-start QP after a local-path
        // refresh. A cold retry is accepted only if the complete unchanged
        // constraints solve successfully; otherwise the outer catch performs
        // the same fail-stop as v8.
        if (std::string(first_error.what()) != "joint_mpc_solver_4") throw;
        controller_->cold_retry(speed, use_timed);
        command = controller_->compute(
            *assets_, reference_path_, ego_pose(), speed, lateral, twist.angular.z,
            remaining, command_cap, dt, *tracker_, {}, use_timed?&timed:nullptr);
        ++solver_recoveries_;
        controller_->diag["cold_retry_recovered"] = true;
      }
      if(use_timed) validate_mpc_motion(time);
      publish_drive(command.speed, command.angle);
      diagnostic = controller_->diag;
      stopped_ = false;
    } catch (const std::exception &error) {
      ++controller_failures_;
      const std::string message = error.what();
      if (message.find("deadline") != std::string::npos) ++solver_timeouts_;
      if (message.find("infeasible") != std::string::npos ||
          message.find("solver_") != std::string::npos) ++infeasible_count_;
      diagnostic = controller_->diag;
      diagnostic["controller_error"] = message;
      publish_stop(message);
    }
    publish_status(time, diagnostic);
  }

  void publish_drive(double speed, double steering) {
    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header.stamp = now();
    command.header.frame_id = "ego_racecar/base_link";
    command.drive.speed = speed;
    command.drive.steering_angle = steering;
    drive_pub_->publish(command);
  }

  void validate_mpc_motion(double time) {
    const auto &path=controller_->accepted_prediction();
    if(path.size()<2) throw std::runtime_error("mpc_prediction_missing");
    auto target=prediction_track(time);
    const double age=target.valid?time-target.observation_stamp:frenet::INF;
    // Freshness must use this control tick, not the preceding planner tick.
    // Once fully rejoined, expected rear-FOV loss follows the same cruise
    // policy as the planner; a new fresh lead is still checked normally.
    if(decision_.overtakes_completed>0 && decision_.behavior==takeover::Behavior::CRUISE &&
        age>dynamic_planner_->detection_timeout) target.valid=false;
    const auto lead=takeover::predict_track(*assets_,target,(path.size()-1)*.05,.05,
        dynamic_planner_->process_sigma_rate,dynamic_planner_->track_assisted_prediction);
    double minimum=frenet::INF;
    for(size_t i=0;i<path.size();++i) {
      const auto body=takeover::rectangle(path[i],.58,.31);
      for(const auto &corner:body)
        if(assets_->clearance(corner)<dynamic_planner_->map_margin)
          throw std::runtime_error("mpc_actual_map_conflict");
      if(!target.valid) continue;
      if(age>dynamic_planner_->committed_prediction_timeout)
        throw std::runtime_error("mpc_target_expired");
      const double inflation=dynamic_planner_->body_margin+
          std::min(.30,dynamic_planner_->uncertainty_sigma*lead[i].sigma)+
          .5*(ego_speed()+target.velocity.norm())*.05;
      const auto other=takeover::rectangle(lead[i].pose,target.length+2*inflation,target.width+2*inflation);
      minimum=std::min(minimum,takeover::polygon_distance(body,other));
      if(takeover::polygons_overlap(body,other))
        throw std::runtime_error("mpc_actual_dynamic_conflict");
    }
    controller_->diag["mpc_dynamic_check_passed"]=true;
    controller_->diag["mpc_minimum_predicted_body_clearance_m"]=std::isfinite(minimum)?Json::Value(minimum):Json::Value();
  }

  void publish_stop(const std::string &reason) {
    publish_drive(0.0, 0.0);
    // A rejected solve can update internal command state even before the
    // first drive publication. Never carry that rejected state into retry.
    controller_->reset();
    stopped_ = true;
    stop_reason_ = reason;
  }

  void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &publisher,
                    const frenet::Path &path) {
    nav_msgs::msg::Path output;
    output.header.stamp = now();
    output.header.frame_id = "map";
    for (const auto &point : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = output.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.orientation = quaternion_from_yaw(point.yaw);
      output.poses.push_back(pose);
    }
    publisher->publish(output);
  }

  void publish_lead_prediction(const std::vector<takeover::Prediction> &prediction) {
    frenet::Path path;
    for (const auto &point : prediction) path.push_back(point.pose);
    publish_path(lead_prediction_pub_, path);
  }

  void publish_status(double time, const Json::Value &controller_diagnostic) {
    Json::Value value;
    value["stamp"] = time;
    value["plan_stamp"] = last_plan_time_;
    value["freshness_triggered_replans"] = freshness_triggered_replans_;
    value["mode"] = mode_;
    value["behavior"] = takeover::behavior_name(decision_.behavior);
    value["transition"] = decision_.transition;
    value["committed_side"] = decision_.committed_side;
    value["stop"] = decision_.stop || stopped_;
    value["stop_reason"] = stopped_ ? stop_reason_ : "none";
    value["target_fresh"] = decision_.target_fresh;
    value["target_age_s"] = std::isfinite(decision_.target_age) ?
        Json::Value(decision_.target_age) : Json::Value();
    value["target_gap_m"] = std::isfinite(decision_.target_gap) ?
        Json::Value(decision_.target_gap) : Json::Value();
    value["target_speed_mps"] = decision_.target_speed;
    value["tracker_mode"] = tracker_mode_;
    value["planner_valid"] = decision_.candidate.valid;
    value["planner_reason"] = decision_.candidate.reason;
    value["planning_ms"] = decision_.planning_ms;
    value["speed_cap_mps"] = decision_.candidate.speed_cap;
    value["prediction_model"] = dynamic_planner_->track_assisted_prediction?"track_assisted_with_cv_fallback":"cartesian_cv";
    value["timed_control"] = dynamic_planner_->timed_control;
    value["transition_length_m"] = decision_.candidate.transition_length;
    value["minimum_predicted_body_clearance_m"] =
        std::isfinite(decision_.candidate.minimum_body_clearance) ?
        Json::Value(decision_.candidate.minimum_body_clearance) : Json::Value();
    value["minimum_predicted_map_clearance_m"] =
        std::isfinite(decision_.candidate.minimum_map_clearance) ?
        Json::Value(decision_.candidate.minimum_map_clearance) : Json::Value();
    value["maximum_candidate_curvature_1pm"] = decision_.candidate.maximum_curvature;
    value["overtakes_completed"] = decision_.overtakes_completed;
    value["planner_failures"] = planner_failures_;
    value["controller_failures"] = controller_failures_;
    value["solver_timeouts"] = solver_timeouts_;
    value["infeasible_count"] = infeasible_count_;
    value["solver_recoveries"] = solver_recoveries_;
    value["rejected_track_jumps"] = rejected_track_jumps_;
    value["controller"] = controller_diagnostic;
    value["reference_path"] = frenet::path_json(reference_path_);
    value["timed_reference"] = Json::Value(Json::arrayValue);
    for (const auto &point : decision_.candidate.timed) {
      Json::Value row(Json::arrayValue);
      row.append(point.t); row.append(point.s); row.append(point.d); row.append(point.speed);
      value["timed_reference"].append(row);
    }
    value["mpc_prediction"] = stopped_ ? Json::Value(Json::arrayValue) :
        frenet::path_json(controller_->accepted_prediction());
    value["target_prediction"] = Json::Value(Json::arrayValue);
    for (const auto &point : decision_.target_prediction) {
      Json::Value row(Json::arrayValue);
      row.append(point.t); row.append(point.pose.x); row.append(point.pose.y);
      row.append(point.pose.yaw); row.append(point.sigma);
      value["target_prediction"].append(row);
    }
    status_pub_->publish(std_msgs::msg::String().set__data(frenet::dump(value)));
  }

  std::string stop_reason_ = "initializing";
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TakeoverControllerNode>());
  } catch (const std::exception &error) {
    std::fprintf(stderr, "takeover_controller fatal: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
