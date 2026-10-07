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
  std::string tracker_mode_ = "UNINITIALIZED";
  double tracker_measurement_age_ = frenet::INF;
  double last_plan_time_ = -frenet::INF;
  double last_control_time_ = -frenet::INF;
  double latest_target_message_time_ = -frenet::INF;
  double planning_period_ = 0.2;
  bool stopped_ = true;
  int controller_failures_ = 0;
  int planner_failures_ = 0;
  int solver_timeouts_ = 0;
  int infeasible_count_ = 0;
  int solver_recoveries_ = 0;
  int rejected_track_jumps_ = 0;
  takeover::Decision decision_;
  frenet::Path reference_path_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr lead_prediction_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_sub_;
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
    if (track_.valid && committed && latest_target_message_time_ > 0.0) {
      const double dt = frenet::clip(
          receive_time - latest_target_message_time_, 0.0, 0.25);
      const takeover::Point expected = track_.position + dt * track_.velocity;
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

  void plan(double time) {
    const frenet::Pose pose = ego_pose();
    if (mode_ == "static_v8") {
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
      decision_ = dynamic_planner_->plan(pose, ego_speed(), track_, time);
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
    if (time - last_plan_time_ >= planning_period_) plan(time);
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
      const double lateral = -std::sin(yaw) * twist.linear.x + std::cos(yaw) * twist.linear.y;
      const double remaining = 100.0;
      frenet::Control command;
      try {
        command = controller_->compute(
            *assets_, reference_path_, ego_pose(), speed, lateral, twist.angular.z,
            remaining, decision_.candidate.speed_cap, dt, *tracker_, {});
      } catch (const std::exception &first_error) {
        // acados status 4 can be a stale warm-start QP after a local-path
        // refresh. A cold retry is accepted only if the complete unchanged
        // constraints solve successfully; otherwise the outer catch performs
        // the same fail-stop as v8.
        if (std::string(first_error.what()) != "joint_mpc_solver_4") throw;
        controller_->cold_retry(speed);
        command = controller_->compute(
            *assets_, reference_path_, ego_pose(), speed, lateral, twist.angular.z,
            remaining, decision_.candidate.speed_cap, dt, *tracker_, {});
        ++solver_recoveries_;
        controller_->diag["cold_retry_recovered"] = true;
      }
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

  void publish_stop(const std::string &reason) {
    publish_drive(0.0, 0.0);
    if (!stopped_) controller_->reset();
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
