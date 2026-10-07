#pragma once
#include "tracker.hpp"
extern "C" {
#include "acados_solver_joint_vehicle.h"
}
namespace frenet {
struct Control { double speed=0, angle=0; };
struct Geometry {
  Path path;
  std::vector<double> arc, yaw, curvature;
  double nearest_s=0, cross=0, heading=0;
  Geometry(const Path &p, Pose pose):path(p) {
    if(p.size()<2) throw std::runtime_error("tracker_path_invalid");
    arc.push_back(0); yaw.push_back(p[0].yaw);
    double best=INF;
    for(size_t i=1;i<p.size();++i) {
      double length=(p[i].xy()-p[i-1].xy()).norm();
      if(length<1e-6) throw std::runtime_error("tracker_path_duplicate");
      arc.push_back(arc.back()+length);
      yaw.push_back(yaw.back()+wrap(p[i].yaw-p[i-1].yaw));
      Point edge=p[i].xy()-p[i-1].xy();
      double t=clip((pose.xy()-p[i-1].xy()).dot(edge)/edge.squaredNorm(),0,1);
      Point d=pose.xy()-(p[i-1].xy()+t*edge);
      if(d.squaredNorm()<best) {
        best=d.squaredNorm();nearest_s=arc[i-1]+t*length;
        double h=yaw[i-1]+t*(yaw[i]-yaw[i-1]);
        cross=-std::sin(h)*d.x()+std::cos(h)*d.y();heading=wrap(pose.yaw-h);
      }
    }
    for(size_t i=0;i<p.size();++i) {
      size_t lo=i?i-1:i, hi=std::min(i+1,p.size()-1);
      curvature.push_back((yaw[hi]-yaw[lo])/(arc[hi]-arc[lo]));
    }
    if(std::abs(cross)>.25) throw std::runtime_error("tracking_cross_track_error");
    if(std::abs(heading)>.7) throw std::runtime_error("tracking_heading_error");
  }
  Pose at(double s) const {
    s=clip(s,0,arc.back());
    size_t hi=std::min(path.size()-1,size_t(std::upper_bound(arc.begin(),arc.end(),s)-arc.begin()));
    size_t lo=hi?hi-1:0;
    double f=hi==lo?0:(s-arc[lo])/(arc[hi]-arc[lo]);
    return {path[lo].x+f*(path[hi].x-path[lo].x),path[lo].y+f*(path[hi].y-path[lo].y),yaw[lo]+f*(yaw[hi]-yaw[lo])};
  }
  double target(double s,double remaining,double cap) const {
    double k=0;
    for(size_t i=0;i<arc.size();++i)
      if(arc[i]>=s-.05 && arc[i]<=s+.75) k=std::max(k,std::abs(curvature[i]));
    // Curvature/terminal feasibility is explicit; no fixed AVOIDING speed dip.
    return std::min({cap,std::sqrt(1.2/std::max(.01,k)),std::sqrt(std::max(0.,remaining)*.8)});
  }
};

inline double corridor(const Assets &a,Pose p,const std::vector<Point>&obs) {
  double margin=std::min(.20,a.clearance(p.xy())-.25);
  Point tangent(std::cos(p.yaw),std::sin(p.yaw));
  for(auto o:obs) for(double shift:{-.08,0.,.08})
    margin=std::min(margin,(p.xy()+shift*tangent-o).norm()-.15);
  return clip(margin*.7,.003,.20);
}
inline void check_swept(const Assets&a,const Path&prediction,const std::vector<Point>&obs) {
  for(size_t i=0;i<prediction.size();++i) {
    Pose prev=i?prediction[i-1]:prediction[i];
    int count=std::max(1,int(std::ceil((prev.xy()-prediction[i].xy()).norm()/.02)));
    for(int j=0;j<=count;++j) {
      double f=double(j)/count;
      Pose p{prev.x+f*(prediction[i].x-prev.x),prev.y+f*(prediction[i].y-prev.y),prev.yaw+f*wrap(prediction[i].yaw-prev.yaw)};
      if(a.clearance(p.xy())<.25) throw std::runtime_error("predicted_static_clearance");
      Point tangent(std::cos(p.yaw),std::sin(p.yaw));
      for(auto o:obs) for(double shift:{-.08,0.,.08})
        if((p.xy()+shift*tangent-o).norm()<.15) throw std::runtime_error("predicted_obstacle_footprint");
    }
  }
}

class JointControl {
  joint_vehicle_solver_capsule *capsule=nullptr;
  ocp_nlp_config *config=nullptr; ocp_nlp_dims *dims=nullptr;
  ocp_nlp_in *in=nullptr; ocp_nlp_out *out=nullptr;
  double command=0, acceleration=0, steering=0, servo=0, integral=0;
  double continuous_yaw=0, lateral_estimate=0, yaw_rate_estimate=0;
  bool initialized=false, floor_reached=false, coasting=false;
  std::array<std::array<double,10>,25> predicted{};
public:
  Json::Value diag;
  explicit JointControl(bool mpc) {
    if(mpc) {
      capsule=joint_vehicle_acados_create_capsule();
      if(!capsule || joint_vehicle_acados_create(capsule)) throw std::runtime_error("joint_solver_create");
      config=joint_vehicle_acados_get_nlp_config(capsule);dims=joint_vehicle_acados_get_nlp_dims(capsule);
      in=joint_vehicle_acados_get_nlp_in(capsule);out=joint_vehicle_acados_get_nlp_out(capsule);
      double W[100]={0},We[36]={0};
      const double w[10]={3.,160.,140.,18.,1.,.5,.3,1.2,2.,180.};
      const double we[6]={5.,240.,210.,30.,1.,.5};
      for(int i=0;i<10;++i)W[i*10+i]=w[i];
      for(int i=0;i<6;++i)We[i*6+i]=we[i];
      for(int i=0;i<24;++i)ocp_nlp_cost_model_set(config,dims,in,i,"W",W);
      ocp_nlp_cost_model_set(config,dims,in,24,"W",We);
    }
  }
  ~JointControl(){if(capsule){joint_vehicle_acados_free(capsule);joint_vehicle_acados_free_capsule(capsule);}}
  bool terminal_coasting() const {return coasting;}
  void reset(){floor_reached=coasting=false;command=acceleration=steering=servo=integral=0; initialized=false;continuous_yaw=lateral_estimate=yaw_rate_estimate=0;diag=Json::Value();if(capsule)joint_vehicle_acados_reset(capsule,1);}
  // Reset only the numerical warm start after an acados QP failure.  Seeding
  // the longitudinal state from measured speed avoids an artificial zero-speed
  // command; steering/servo estimates remain the physical actuator state.
  void cold_retry(double measured) {
    command=clip(measured,0.,2.0);acceleration=0.;initialized=false;
    floor_reached=command>=.50;coasting=false;
    continuous_yaw=lateral_estimate=yaw_rate_estimate=0.;diag=Json::Value();
    if(capsule)joint_vehicle_acados_reset(capsule,1);
  }
  Control compute(const Assets&a,const Path&p,Pose pose,double measured,double lateral,double yaw_rate,double remaining,double cap,double dt,Tracker&tracker,const std::vector<Point>&obs) {
    double begin=mono();
    diag=Json::Value();
    if(!std::isfinite(measured+lateral+yaw_rate+remaining+cap+dt) || measured<0 || measured>2.4 || cap<.70 || cap>2.0 || std::abs(lateral)>2.5 || std::abs(yaw_rate)>8 || dt<=0 || dt>.1)
      throw std::runtime_error("joint_state_invalid");
    // Localization wraps yaw at +/-pi. Keep the solver state, references and
    // its retained warm-start trajectory on the same continuous angle branch.
    // This changes only the representation, never steering/error thresholds.
    if(capsule) {
      if(initialized) pose.yaw=continuous_yaw+wrap(pose.yaw-continuous_yaw);
      continuous_yaw=pose.yaw;
    }
    Geometry g(p,pose);dt=std::min(.05,dt);
    diag=Json::Value();Path trajectory{pose};
    if(capsule) {
      diag["continuous_yaw_rad"]=pose.yaw;
      // This vehicle's odometry angular velocity is inferred from the servo
      // COMMAND, not measured by an IMU. It anticipates the actuator response.
      // Carry lateral speed and yaw rate through the MPC's own dynamic model,
      // using measured longitudinal speed and accepted steering commands.
      // These are model estimates, not independent inertial measurements.
      // Preserve the raw odometry input for diagnosis.
      diag["command_derived_yaw_rate_rad_s"]=yaw_rate;
      diag["input_lateral_speed_mps"]=lateral;
      lateral=lateral_estimate;
      yaw_rate=yaw_rate_estimate;
      diag["model_yaw_rate_rad_s"]=yaw_rate;
      diag["model_lateral_speed_mps"]=lateral;
      diag["initial_servo_estimate_rad"]=servo;
      diag["yaw_rate_source"]="continuous_dynamic_model_observer";
      diag["predicted_lateral_acceleration_limit_mps2"]=1.65;
    }
    if(!capsule) {
      // Heading PID and longitudinal PI remain explicitly separate loops.
      steering=tracker.compute(p,pose,dt,measured,lateral,yaw_rate);
      // Planner progress is sampled at 5 Hz. Reproject the current pose onto
      // the terminal path every control tick to avoid stale braking distance.
      double control_remaining=remaining;
      if((p.back().xy()-a.route.back().xy()).norm()<.02)
        control_remaining=std::min(remaining,std::max(0.,g.arc.back()-g.nearest_s));
      const double floor=.50, brake=.55, response=.30, floor_settle_distance=.25;
      if(cap<floor) throw std::runtime_error("pid_speed_cap_below_drivable_floor");
      // Reach the minimum drivable speed before the final coast zone. The
      // command itself enters the feedback bound: missing/zero low-speed
      // feedback cannot integrate a command above a falling speed target.
      double ramp_distance=std::max(0.,control_remaining-floor_settle_distance);
      double terminal_target=std::sqrt(std::pow(brake*response,2)+2*brake*ramp_distance)-brake*response;
      terminal_target=std::max(floor,terminal_target);
      double geometry_target=g.target(g.nearest_s,1e6,cap);
      if(geometry_target<.70) throw std::runtime_error("pid_curve_below_drivable_floor");
      double target=std::min(geometry_target,terminal_target);
      double feedback=std::max(measured,command),error=target-feedback;
      double blend=clip((geometry_target-terminal_target)/.15,0,1);
      double feedforward=terminal_target>floor+1e-6
          ? -blend*brake*measured/std::max(.05,terminal_target+brake*response) : 0.;
      double next_i=clip(integral+error*dt,-.4,.4);
      double requested=feedforward+2.0*error+.5*next_i;
      double bounded=clip(requested,-.8,1.0);
      if(std::abs(requested-bounded)<1e-9 || requested*error<0) integral=next_i;
      acceleration+=clip(bounded-acceleration,-4*dt,4*dt);
      command=clip(command+acceleration*dt,0,cap);
      if(command>=floor) floor_reached=true;
      if(floor_reached && command<floor) {command=floor;acceleration=integral=0.;}
      if((command>=cap && error>0)||(command<=0 && error<0))integral=0;
      // Calibrated nominal coast estimate; sensitivity checked at 0.4--0.8
      // m/s^2. A 50ms command/transport allowance precedes zero-current coast.
      // Cut only after the commanded speed has settled to the floor. STOP
      // latches: it must never turn into another low-speed launch attempt.
      double stopping_speed=std::max(measured,command);
      double coast_distance=stopping_speed*stopping_speed/(2*.60)+.05*stopping_speed;
      if((floor_reached && command<=floor+.015 && control_remaining<=coast_distance)
          || control_remaining<=.05) coasting=true;
      if(coasting) {command=steering=acceleration=integral=0.;}
      // Short executable-command sweep, not a claim of future PID optimality.
      Pose q=pose;double v=measured,s=servo;
      for(int i=0;i<10;++i) {
        v=coasting?std::max(0.,v-.05*.4):v+.05*clip((command-v)/.15,-.8,1.0);s+=(steering-s)*.05/.15;
        q.yaw+=.05*v/.25*std::tan(s);q.x+=.05*v*std::cos(q.yaw);q.y+=.05*v*std::sin(q.yaw);trajectory.push_back(q);
      }
      diag=tracker.diag;diag["longitudinal_controller"]="velocity_pi_acceleration_jerk_limited";
      diag["speed_target_mps"]=target;diag["speed_error_mps"]=error;diag["speed_integral"]=integral;
      diag["control_remaining_m"]=control_remaining;diag["terminal_target_mps"]=terminal_target;
      diag["terminal_acceleration_feedforward_mps2"]=feedforward;
      diag["minimum_drivable_speed_mps"]=floor;
      diag["speed_feedback_bound_mps"]=feedback;
      diag["terminal_coasting"]=coasting;
      diag["coast_cut_distance_m"]=coast_distance;
      diag["speed_phase"]=coasting?"terminal_coast":(floor_reached?"driving":"soft_start");
    } else {
      double control_remaining=remaining;
      if((p.back().xy()-a.route.back().xy()).norm()<.02)
        control_remaining=std::min(remaining,std::max(0.,g.arc.back()-g.nearest_s));
      const double floor=.50, brake=.55, response=.30, settle=.25;
      if(cap<floor) throw std::runtime_error("mpc_speed_cap_below_drivable_floor");
      double stopping_speed=std::max(measured,command);
      double coast_distance=stopping_speed*stopping_speed/(2*.60)+.05*stopping_speed;
      if((floor_reached && command<=floor+.10 && control_remaining<=coast_distance)
          || control_remaining<=.05) coasting=true;
      auto speed_target=[&](double s,double rem) {
        double curve=g.target(s,1e6,cap);
        if(curve<.70) throw std::runtime_error("mpc_curve_below_drivable_floor");
        double terminal=std::max(floor,std::sqrt(std::pow(brake*response,2)+2*brake*std::max(0.,rem-settle))-brake*response);
        return std::min(curve,terminal);
      };
      diag["control_remaining_m"]=control_remaining;
      diag["minimum_drivable_speed_mps"]=floor;
      diag["coast_cut_distance_m"]=coast_distance;
      diag["terminal_coasting"]=coasting;
      diag["speed_target_mps"]=speed_target(g.nearest_s,control_remaining);
      if(coasting) {
        command=steering=acceleration=0.;
        Pose q=pose;double v=measured,s=servo;
        // Check the complete conservative coast with the same steering
        // response estimate used by the driving model, including its tail.
        int coast_steps=int(std::ceil(measured/(.4*.05)))+2;
        for(int i=0;i<coast_steps;++i) {
          v=std::max(0.,v-.05*.4);s+=(steering-s)*.05/(.20+.05);
          q.yaw+=.05*v/.25*std::tan(s);q.x+=.05*v*std::cos(q.yaw);q.y+=.05*v*std::sin(q.yaw);trajectory.push_back(q);
        }
      } else {
      // Keep the minimum command inside the optimization constraints. Tighten
      // future command bounds only after launch; never clip a solved steering
      // or floor-speed command and silently invalidate its prediction.
      double lbx[4]={0.,-2.5,-8.,floor_reached?floor:0.};
      double ubx[4]={2.3,2.5,8.,cap};
      for(int i=1;i<=24;++i) {
        ocp_nlp_constraints_model_set(config,dims,in,i,"lbx",lbx);
        ocp_nlp_constraints_model_set(config,dims,in,i,"ubx",ubx);
      }
      double x[10]={pose.x,pose.y,pose.yaw,measured,lateral,yaw_rate,acceleration,steering,command,servo};
      ocp_nlp_constraints_model_set(config,dims,in,0,"lbx",x);
      ocp_nlp_constraints_model_set(config,dims,in,0,"ubx",x);
      double s=g.nearest_s, guess_v=measured;
      for(int i=0;i<=24;++i) {
        Pose ref=g.at(s);ref.yaw=pose.yaw+wrap(ref.yaw-pose.yaw);
        double target=speed_target(s,std::max(0.,control_remaining-(s-g.nearest_s)));
        double param[5]={ref.x,ref.y,ref.yaw,target,corridor(a,ref,obs)};
        joint_vehicle_acados_update_params(capsule,i,param,5);
        if(!initialized) {
          double seed[10]={ref.x,ref.y,ref.yaw,guess_v,0,guess_v/.25*std::tan(steering),acceleration,steering,std::min(cap,guess_v),servo};
          if(i==0)std::copy(x,x+10,seed);
          ocp_nlp_out_set(config,dims,out,i,"x",seed);
          if(i<24){double u[2]={0,steering};ocp_nlp_out_set(config,dims,out,i,"u",u);}
        }
        double forward=initialized?predicted[std::min(i+1,24)][3]:guess_v;
        forward=clip(forward,0,cap);
        s=std::min(g.arc.back(),s+.05*std::max(.1,forward));
        guess_v=std::min(target,guess_v+.05);
      }
      // Current localization can lie outside a newly switched narrow corridor;
      // constrain predicted future states, not an impossible change of x0.
      double lh0[5]={-1e3,-.7,-10.,-4*dt,-dt},uh0[5]={1e3,.7,10.,4*dt,dt};
      ocp_nlp_constraints_model_set(config,dims,in,0,"lh",lh0);
      ocp_nlp_constraints_model_set(config,dims,in,0,"uh",uh0);
      int status=0, iterations=0;
      double equality=INF, inequality=INF;
      auto solver=joint_vehicle_acados_get_nlp_solver(capsule);
      for(int iteration=0;iteration<4;++iteration) {
        status=joint_vehicle_acados_solve(capsule);++iterations;
        if(status!=0)throw std::runtime_error("joint_mpc_solver_"+std::to_string(status));
        ocp_nlp_eval_residuals(solver,in,out);
        ocp_nlp_get(config,solver,"res_eq",&equality);
        ocp_nlp_get(config,solver,"res_ineq",&inequality);
        if(!std::isfinite(equality+inequality))throw std::runtime_error("joint_nonfinite_residual");
        if(equality<.001 && inequality<.001)break;
        if(mono()-begin>.035)break;
      }
      if(equality>=.001 || inequality>=.001)throw std::runtime_error("joint_mpc_nonlinear_infeasible eq="+std::to_string(equality)+" ineq="+std::to_string(inequality));
      double u[2];ocp_nlp_out_get(config,dims,out,0,"u",u);
      for(int i=0;i<=24;++i) {
        ocp_nlp_out_get(config,dims,out,i,"x",predicted[i].data());
        for(double value:predicted[i])if(!std::isfinite(value))throw std::runtime_error("joint_nonfinite");
        if(i)trajectory.push_back({predicted[i][0],predicted[i][1],predicted[i][2]});
      }
      if(!std::isfinite(u[0]+u[1]) || std::abs(u[1])>.26901 || u[0]<-.80001 || u[0]>1.00001 || std::abs(u[1]-steering)>dt+1e-4 || std::abs(u[0]-acceleration)>4*dt+1e-4)
        throw std::runtime_error("joint_control_constraint a="+std::to_string(u[0])+" delta="+std::to_string(u[1])+" previous="+std::to_string(acceleration)+","+std::to_string(steering)+" dt="+std::to_string(dt));
      double next=command+u[0]*dt;
      if(next<-.0001 || next>2.0001)throw std::runtime_error("joint_speed_constraint");
      if(floor_reached && next<floor-1e-5)throw std::runtime_error("joint_speed_below_drivable_floor");
      command=clip(next,0,2.0);acceleration=u[0];steering=u[1];initialized=true;
      if(command>=floor)floor_reached=true;
      diag["longitudinal_controller"]="joint_dynamic_mpc";diag["state_dimension"]=10;
      diag["mpc_solver_status"]=status;diag["mpc_iterations"]=iterations;diag["equality_residual"]=equality;diag["inequality_residual"]=inequality;diag["mpc_solve_ms"]=(mono()-begin)*1000;
      }
      diag["longitudinal_controller"]="joint_dynamic_mpc";diag["state_dimension"]=10;
      diag["speed_phase"]=coasting?"terminal_coast":(floor_reached?"driving":"soft_start");
      diag["predicted_path"]=path_json(trajectory);
    }
    // Persist the actual rejected command and sweep before collision checks.
    // Exception reporting must not lose the inputs needed for offline replay.
    diag["predicted_path"]=path_json(trajectory);
    diag["proposed_speed_mps"]=command;
    diag["proposed_steering_rad"]=steering;
    diag["sweep_initial_servo_rad"]=servo;
    check_swept(a,trajectory,obs);
    if(command>cap){command=cap;acceleration=std::min(0.,acceleration);diag["independent_speed_cap_applied"]=true;}
    servo+=(steering-servo)*dt/((capsule?.20:.10)+dt);
    if(capsule) {
      // Propagate unmeasured lateral states with the SAME backward-Euler
      // bicycle model used by the optimizer. Resetting vy=0 and substituting
      // kinematic yaw rate each tick can make an accepted next state infeasible.
      double v=std::max(measured,.35),m=3.74,iz=.04712,lf=.12,lr=.13,cf=90.,cr=96.;
      Eigen::Matrix2d A;
      A << -(cf+cr)/(m*v), -v-(cf*lf-cr*lr)/(m*v),
           -(cf*lf-cr*lr)/(iz*v), -(cf*lf*lf+cr*lr*lr)/(iz*v);
      Eigen::Vector2d b(cf/m,lf*cf/iz), z(lateral,yaw_rate);
      z=(Eigen::Matrix2d::Identity()-dt*A).inverse()*(z+dt*b*servo);
      double blend=clip((measured-.25)/.30,0.,1.);blend=blend*blend*(3-2*blend);
      double next_v=measured+dt/(.15+dt)*(command-measured);
      lateral_estimate=blend*z[0];
      yaw_rate_estimate=blend*z[1]+(1-blend)*next_v/.25*std::tan(servo);
      if(!std::isfinite(lateral_estimate+yaw_rate_estimate))
        throw std::runtime_error("joint_observer_nonfinite");
      diag["next_model_yaw_rate_rad_s"]=yaw_rate_estimate;
      diag["next_model_lateral_speed_mps"]=lateral_estimate;
    }
    diag["cross_track_error_m"]=g.cross;diag["heading_error_rad"]=g.heading;
    diag["measured_speed_mps"]=measured;diag["acceleration_command_mps2"]=acceleration;
    diag["speed_command_mps"]=command;diag["controller_compute_ms"]=(mono()-begin)*1000;
    if(mono()-begin>.045)throw std::runtime_error("joint_control_deadline");
    return {command,steering};
  }
};
} // namespace frenet
