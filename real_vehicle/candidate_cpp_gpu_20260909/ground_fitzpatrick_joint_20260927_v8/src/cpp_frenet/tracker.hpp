#pragma once
#include "core.hpp"
extern "C" int dynamic_mpc_solve(const double *, const double *, const double *,
                                 const double *, const double *, const double *,
                                 double, double, double, double, double, int,
                                 double *, double *);
namespace frenet {
struct Tracker {
  std::string type;
  double angle = 0, error = 0, integral = 0, derivative = 0;
  bool initialized = false;
  double pid_kp = 1.8, pid_preview_gain = 0., pid_kd = .9;
  double pid_prediction_s = .20, pid_servo_estimate = 0;
  std::array<double, 16> previous{};
  Json::Value diag;
  explicit Tracker(std::string value) : type(value) {
    if (type != "pp" && type != "pid" && type != "dynamic_mpc")
      throw std::runtime_error("unknown_tracker");
  }
  void reset() {
    angle = error = integral = derivative = 0;
    initialized = false;
    pid_servo_estimate=0;
    previous.fill(0);
    diag = Json::Value();
  }
  double compute(const Path &p, Pose pose, double dt, double speed,
                 double lateral = 0, double yaw_rate = 0) {
    double started = mono();
    if (p.size() < 2)
      throw std::runtime_error("tracker_path_invalid");
    if (!std::isfinite(pose.x + pose.y + pose.yaw + dt + speed + lateral +
                       yaw_rate))
      throw std::runtime_error("mpc_non_finite");
    if (std::abs(lateral) > 2.5 || std::abs(yaw_rate) > 8)
      throw std::runtime_error("dynamic_state_out_of_range");
    std::vector<double> arc{0}, yaw{p[0].yaw};
    size_t nearest = 0;
    double best = INF;
    for (size_t i = 0; i < p.size(); ++i) {
      if (!std::isfinite(p[i].x + p[i].y + p[i].yaw))
        throw std::runtime_error("tracker_path_non_finite");
      double dist = (p[i].xy() - pose.xy()).norm();
      if (dist < best) {
        best = dist;
        nearest = i;
      }
      if (i) {
        arc.push_back(arc.back() +
                      std::max(1e-5, (p[i].xy() - p[i - 1].xy()).norm()));
        yaw.push_back(yaw.back() + wrap(p[i].yaw - p[i - 1].yaw));
      }
    }
    double requested = 0;
    dt = clip(dt, type == "pid" ? .001 : 0, .05);
    diag = Json::Value();
    if (type == "pp") {
      size_t target = p.size() - 1;
      for (size_t i = 0; i < p.size(); ++i)
        if ((p[i].xy() - pose.xy()).norm() >= .65) {
          target = i;
          break;
        }
      Point d = p[target].xy() - pose.xy();
      double lx = std::cos(pose.yaw) * d.x() + std::sin(pose.yaw) * d.y(),
             ly = -std::sin(pose.yaw) * d.x() + std::cos(pose.yaw) * d.y();
      if (lx <= 0 || d.squaredNorm() < 1e-6)
        throw std::runtime_error("target_behind");
      requested = clip(std::atan2(.5 * ly, d.squaredNorm()), -.269, .269);
      angle += clip(requested - angle, -dt, dt);
      diag["target_index"] = int(target);
    } else if (type == "pid") {
      Pose ref = p[nearest];
      // Project onto segments rather than stepping between 5 cm waypoints.
      double reference_s = arc[nearest], projection_best = INF;
      for (size_t i = 0; i + 1 < p.size(); ++i) {
        Point edge = p[i+1].xy() - p[i].xy();
        if (edge.squaredNorm() < 1e-10) continue;
        double f = clip((pose.xy()-p[i].xy()).dot(edge)/edge.squaredNorm(),0,1);
        Point q = p[i].xy()+f*edge;
        double distance = (pose.xy()-q).squaredNorm();
        if (distance < projection_best) {
          projection_best=distance;
          ref={q.x(),q.y(),yaw[i]+f*(yaw[i+1]-yaw[i])};
          reference_s=arc[i]+f*(arc[i+1]-arc[i]); nearest=i;
        }
      }
      Point diff = pose.xy() - ref.xy();
      double cross =
                 -std::sin(ref.yaw) * diff.x() + std::cos(ref.yaw) * diff.y(),
             heading = wrap(pose.yaw - ref.yaw);
      if (std::abs(cross) > .25)
        throw std::runtime_error("pid_cross_track_error");
      if (std::abs(heading) > .7)
        throw std::runtime_error("pid_heading_error");
      // Predict the pose during steering response using the previous bounded
      // command and a first-order servo estimate. No gyro measurement is
      // claimed: the existing odometry yaw rate is itself servo-derived.
      pid_servo_estimate+=(angle-pid_servo_estimate)*dt/(.10+dt);
      Pose predicted_pose=pose;
      double predicted_servo=pid_servo_estimate;
      int steps=std::max(1,int(std::ceil(pid_prediction_s/.025)));
      double prediction_dt=pid_prediction_s/steps;
      for(int step=0;step<steps;++step) {
        predicted_servo+=(angle-predicted_servo)*prediction_dt/(.10+prediction_dt);
        double rate=std::abs(speed)/.25*std::tan(predicted_servo);
        double mid=predicted_pose.yaw+.5*prediction_dt*rate;
        predicted_pose.x+=prediction_dt*std::abs(speed)*std::cos(mid);
        predicted_pose.y+=prediction_dt*std::abs(speed)*std::sin(mid);
        predicted_pose.yaw=wrap(predicted_pose.yaw+prediction_dt*rate);
      }
      double feedback_cross=cross, feedback_heading=heading, feedback_s=reference_s;
      projection_best=INF;
      for(size_t i=0;i+1<p.size();++i) {
        Point edge=p[i+1].xy()-p[i].xy();if(edge.squaredNorm()<1e-10)continue;
        double f=clip((predicted_pose.xy()-p[i].xy()).dot(edge)/edge.squaredNorm(),0,1);
        Point q=p[i].xy()+f*edge;double dist=(predicted_pose.xy()-q).squaredNorm();
        if(dist<projection_best) {
          projection_best=dist;double h=yaw[i]+f*(yaw[i+1]-yaw[i]);
          Point diff=predicted_pose.xy()-q;
          feedback_cross=-std::sin(h)*diff.x()+std::cos(h)*diff.y();
          feedback_heading=wrap(predicted_pose.yaw-h);
          feedback_s=arc[i]+f*(arc[i+1]-arc[i]);
        }
      }
      // Curvature feedforward supplies the nominal turn. Feedback acts on
      // lateral error, with a kinematic error-rate estimate from heading.
      // Unlike differencing a lookahead bearing, this has no derivative kick
      // when the planner replaces a path. Ki is intentionally zero.
      double preview=clip(std::abs(speed)*pid_preview_gain,0,.25);
      double center_s=feedback_s+preview;
      double lo_s=std::max(0.,center_s-.15), hi_s=std::min(arc.back(),center_s+.15);
      double k=hi_s-lo_s>.02 ? (interp(arc,yaw,hi_s)-interp(arc,yaw,lo_s))/(hi_s-lo_s) : 0.;
      k=clip(k,-std::tan(.269)/.25,std::tan(.269)/.25);
      double feedforward=std::atan(.25*k);
      double error_rate=std::abs(speed)*std::sin(feedback_heading);
      double effective_kd=pid_kd/std::max(.5,std::abs(speed));
      double p_term=-pid_kp*feedback_cross, d_term=-effective_kd*error_rate;
      double raw=feedforward+p_term+d_term;
      requested=clip(raw,-.269,.269);
      angle+=clip(requested-angle,-1.1*dt,1.1*dt);
      error=-cross;derivative=-error_rate;initialized=true;
      diag["lateral_control_law"]="curvature_feedforward_lateral_error_pd";
      diag["cross_track_error_m"]=cross;diag["heading_error_rad"]=heading;
      diag["predicted_pose"]=Json::Value(Json::arrayValue);
      diag["predicted_pose"].append(predicted_pose.x);diag["predicted_pose"].append(predicted_pose.y);diag["predicted_pose"].append(predicted_pose.yaw);
      diag["predicted_cross_track_m"]=feedback_cross;diag["predicted_heading_rad"]=feedback_heading;
      diag["response_prediction_s"]=pid_prediction_s;diag["servo_estimate_rad"]=pid_servo_estimate;
      diag["pid_error_m"]=-feedback_cross;diag["pid_error_rate_mps"]=-error_rate;
      diag["pid_effective_kd_s_per_m"]=effective_kd;
      diag["pid_p_rad"]=p_term;diag["pid_d_rad"]=d_term;
      diag["curvature_feedforward_rad"]=feedforward;
      diag["curvature_reference_1pm"]=k;diag["curvature_preview_m"]=preview;
      diag["pid_raw_steering_rad"]=raw;diag["reference_arc_m"]=reference_s;
    } else {
      Pose ref;
      double reference_s = 0, min_dist = INF;
      bool valid = false;
      for (size_t i = 0; i + 1 < p.size(); ++i) {
        Point edge = p[i + 1].xy() - p[i].xy();
        double sq = edge.squaredNorm();
        if (sq <= 1e-10)
          continue;
        double f = clip((pose.xy() - p[i].xy()).dot(edge) / sq, 0, 1);
        Point xy = p[i].xy() + f * edge;
        double dist = (pose.xy() - xy).squaredNorm();
        if (dist < min_dist) {
          valid = true;
          min_dist = dist;
          nearest = i;
          ref = {xy.x(), xy.y(), yaw[i] + f * (yaw[i + 1] - yaw[i])};
          reference_s = arc[i] + f * (arc[i + 1] - arc[i]);
        }
      }
      if (!valid)
        throw std::runtime_error("tracker_path_invalid");
      Point diff = pose.xy() - ref.xy();
      double cross =
                 -std::sin(ref.yaw) * diff.x() + std::cos(ref.yaw) * diff.y(),
             heading = wrap(pose.yaw - ref.yaw);
      if (std::abs(cross) > .25)
        throw std::runtime_error("mpc_cross_track_error");
      if (std::abs(heading) > .7)
        throw std::runtime_error("mpc_heading_error");
      std::vector<double> k(p.size());
      for (size_t i = 0; i < p.size(); ++i) {
        if (i == 0)
          k[i] = (yaw[1] - yaw[0]) / (arc[1] - arc[0]);
        else if (i + 1 == p.size())
          k[i] = (yaw[i] - yaw[i - 1]) / (arc[i] - arc[i - 1]);
        else {
          double left = arc[i] - arc[i - 1], right = arc[i + 1] - arc[i];
          k[i] = -right / (left * (left + right)) * yaw[i - 1] +
                 (right - left) / (left * right) * yaw[i] +
                 left / (right * (left + right)) * yaw[i + 1];
        }
        k[i] = clip(k[i], -std::tan(.27) / .25, std::tan(.27) / .25);
      }
      std::array<double, 16> speeds, curv, control;
      std::array<double, 4> maxima;
      double v = clip(std::abs(speed), .5, 1.5);
      for (int i = 0; i < 16; ++i) {
        speeds[i] = v;
        curv[i] = interp(arc, k, reference_s + i * v * .05);
      }
      double state[] = {cross, heading, lateral, yaw_rate},
             model[] = {3.74, .04712, .12, .13, 90, 96},
             weights[] = {140, 70, 1, 3, 210, 105, 1.5, 4.5, 1.2, 16};
      int iterations = dynamic_mpc_solve(
          state, speeds.data(), curv.data(), model, weights, previous.data(),
          angle, .269, dt, .05, .05, 80, control.data(), maxima.data());
      if (iterations < 0)
        throw std::runtime_error("mpc_native_failure");
      for (double value : control)
        if (!std::isfinite(value))
          throw std::runtime_error("mpc_non_finite");
      for (int i = 0; i < 4; ++i)
        if (!std::isfinite(maxima[i]) ||
            maxima[i] > std::array<double, 4>{.25, .7, 2.5, 8}[i])
          throw std::runtime_error("mpc_infeasible");
      previous = control;
      angle = control[0];
      diag["cross_track_error_m"] = cross;
      diag["heading_error_rad"] = heading;
      diag["reference_arc_m"] = reference_s;
      diag["state_dimension"] = 4;
      diag["mpc_iterations"] = iterations;
      diag["mpc_solve_ms"] = (mono() - started) * 1000;
      diag["mpc_predicted_max_cross_track_m"] = maxima[0];
      diag["mpc_predicted_max_heading_rad"] = maxima[1];
      diag["lateral_speed_mps"] = lateral;
      diag["yaw_rate_rad_s"] = yaw_rate;
      diag["prediction_speed_mps"] = v;
    }
    diag["nearest_local_index"] = int(nearest);
    diag["measured_speed_mps"] = speed;
    diag["controller_compute_ms"] = (mono() - started) * 1000;
    return angle;
  }
};
} // namespace frenet
