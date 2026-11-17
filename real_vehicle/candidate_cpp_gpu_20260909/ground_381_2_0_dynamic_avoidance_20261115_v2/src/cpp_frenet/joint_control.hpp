#pragma once
#include "tracker.hpp"
#include "speed_geometry.hpp"
#include "control_timing.hpp"
#include "speed_transition.hpp"
#include <ctime>
#include <functional>
extern "C" {
#include "acados_solver_joint_vehicle.h"
#include "joint_vehicle_model/joint_vehicle_model.h"
}
namespace frenet {
struct Control { double speed=0, angle=0; };

inline double corridor(const Assets &a,Pose p,const std::vector<Point>&obs) {
  double margin=std::min(.20,body_map_reserve(a,p));
  Point tangent(std::cos(p.yaw),std::sin(p.yaw));
  for(auto o:obs) for(double shift:{-.08,0.,.08})
    margin=std::min(margin,(p.xy()+shift*tangent-o).norm()-.15);
  return clip(margin*.9,.003,.20);
}
inline void check_swept(const Assets&a,const Path&prediction,const std::vector<Point>&obs) {
  for(size_t i=0;i<prediction.size();++i) {
    Pose prev=i?prediction[i-1]:prediction[i];
    int count=std::max(1,int(std::ceil((prev.xy()-prediction[i].xy()).norm()/.02)));
    for(int j=0;j<=count;++j) {
      double f=double(j)/count;
      Pose p{prev.x+f*(prediction[i].x-prev.x),prev.y+f*(prediction[i].y-prev.y),prev.yaw+f*wrap(prediction[i].yaw-prev.yaw)};
      if(body_map_reserve(a,p)<0) throw std::runtime_error("predicted_full_body_static_clearance");
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
  double minimum_drivable_speed=.5;
  bool initialized=false, floor_reached=false, coasting=false;
  bool feasibility_recovery=false,map_speed_feasibility=false;
  // Private simulation copies use the same PID/PI update without nesting a sweep.
  bool prediction_only=false;
  MpcTimingBudget timing_budget;
  Json::Value tracking_costs;
  std::array<std::array<double,10>,25> predicted{};
public:
  Json::Value diag;
  double current_servo_estimate() const {return servo;}
#ifdef FRENET_DIAGNOSTIC_PROBE
  int diagnostic_mask=0;
  void diagnostic_restore_actuator_state(const Json::Value& state) {
    if(!state.isArray() || state.size()!=10)throw std::runtime_error("diagnostic_state_shape");
    for(const auto& x:state)if(!x.isNumeric() || !std::isfinite(x.asDouble()))throw std::runtime_error("diagnostic_state_nonfinite");
    acceleration=state[6].asDouble();steering=state[7].asDouble();command=state[8].asDouble();servo=state[9].asDouble();
    floor_reached=command>=minimum_drivable_speed;coasting=false;initialized=false;timing_budget.reset();
  }
#endif
  explicit JointControl(bool mpc,bool recovery=false,const Json::Value& tuning=Json::Value()):feasibility_recovery(recovery) {
    map_speed_feasibility=tuning.get("map_speed_feasibility",false).asBool();
    minimum_drivable_speed=tuning.get("minimum_drivable_speed_mps",.5).asDouble();
    if(!std::isfinite(minimum_drivable_speed) || minimum_drivable_speed<.5 || minimum_drivable_speed>1.)
      throw std::runtime_error("joint_drivable_floor");
    if(mpc) {
      capsule=joint_vehicle_acados_create_capsule();
      if(!capsule || joint_vehicle_acados_create(capsule)) throw std::runtime_error("joint_solver_create");
      config=joint_vehicle_acados_get_nlp_config(capsule);dims=joint_vehicle_acados_get_nlp_dims(capsule);
      in=joint_vehicle_acados_get_nlp_in(capsule);out=joint_vehicle_acados_get_nlp_out(capsule);
      double W[100]={0},We[36]={0};
      double w[10]={3.,160.,140.,18.,1.,.5,.3,1.2,2.,180.};
      double we[6]={5.,240.,210.,30.,1.,.5};
      if(!tuning.isNull()) {
        auto read=[&](const char* key,double low,double high,double fallback) {
          if(!tuning[key].isNull() && !tuning[key].isNumeric())throw std::runtime_error("mpc_tuning_schema");
          double v=tuning.get(key,fallback).asDouble();
          if(!std::isfinite(v)||v<low||v>high)throw std::runtime_error("mpc_tuning_range");
          return v;
        };
        w[1]=read("cross_track_weight",80.,200.,160.);we[1]=1.5*w[1];
        w[2]=read("heading_weight",100.,240.,140.);we[2]=1.5*w[2];
        w[5]=read("yaw_rate_weight",.5,6.,.5);we[5]=w[5];
        w[9]=read("steering_change_weight",180.,900.,180.);
      }
      for(int i=0;i<10;++i)W[i*10+i]=w[i];
      for(int i=0;i<6;++i)We[i*6+i]=we[i];
      tracking_costs["cross_track_weight"]=w[1];
      tracking_costs["heading_weight"]=w[2];
      tracking_costs["yaw_rate_weight"]=w[5];
      tracking_costs["steering_change_weight"]=w[9];
      for(int i=0;i<24;++i){
        ocp_nlp_cost_model_set(config,dims,in,i,"W",W);
        double ubu[2]={1.,.269};ocp_nlp_constraints_model_set(config,dims,in,i,"ubu",ubu);
      }
      ocp_nlp_cost_model_set(config,dims,in,24,"W",We);
    }
  }
  ~JointControl(){if(capsule){joint_vehicle_acados_free(capsule);joint_vehicle_acados_free_capsule(capsule);}}
  bool terminal_coasting() const {return coasting;}
  void enable_feasibility_recovery(){feasibility_recovery=true;}
  // Solver iterate only. Preserve applied-command/actuator estimates, terminal
  // latch, speed floor and timing history; never invent a vehicle-at-rest state.
  void reset_solver_iterate() {if(capsule)joint_vehicle_acados_reset(capsule,1);initialized=false;}
  void reset(){timing_budget.reset();floor_reached=coasting=false;command=acceleration=steering=servo=integral=0; initialized=false;diag=Json::Value();if(capsule)joint_vehicle_acados_reset(capsule,1);}
  Control compute(const Assets&a,const Path&p,Pose pose,double measured,double lateral,double yaw_rate,double remaining,double cap,double dt,Tracker&tracker,const std::vector<Point>&obs,double reference_cap=INF,const std::function<std::pair<Pose,double>(Pose,double)>& dynamic_corridor={},const ReferenceSpeedTable* reference_table=nullptr,double braking_cap=INF) {
    double begin=mono();
    timespec cpu_begin{};
    const bool cpu_clock_valid=clock_gettime(CLOCK_THREAD_CPUTIME_ID,&cpu_begin)==0;
    diag=Json::Value();
    if(!std::isfinite(measured+lateral+yaw_rate+remaining+cap+dt) || measured<0 || measured>2.2 || std::abs(lateral)>2.5 || std::abs(yaw_rate)>8 || dt<=0 || dt>.1)
      throw std::runtime_error("joint_state_invalid");
    const double geometry_begin=mono();
    Geometry g(p,pose,map_speed_feasibility?&a:nullptr,minimum_drivable_speed,reference_table);dt=std::min(.05,dt);
    const double geometry_ms=(mono()-geometry_begin)*1000.;
    diag=Json::Value();diag["tracking_costs"]=tracking_costs;Path trajectory{pose};
    diag["geometry_setup_ms"]=geometry_ms;
    if(!capsule) {
      // Lateral PD/feedforward and longitudinal PI are separate loops.
      cap=std::min(cap,map_speed_feasibility?2.:1.85);
      diag["effective_command_cap_mps"]=cap;
      steering=tracker.compute(p,pose,dt,measured,lateral,yaw_rate);
      // Planner progress is sampled at 5 Hz. Reproject the current pose onto
      // the terminal path every control tick to avoid stale braking distance.
      double control_remaining=remaining;
      if((p.back().xy()-a.route.back().xy()).norm()<.02)
        control_remaining=std::min(remaining,std::max(0.,g.arc.back()-g.nearest_s));
      const double floor=minimum_drivable_speed, brake=.55, response=.30, floor_settle_distance=.25;
      if(cap<floor) throw std::runtime_error("pid_speed_cap_below_drivable_floor");
      // Reach the minimum drivable speed before the final coast zone. The
      // command itself enters the feedback bound: missing/zero low-speed
      // feedback cannot integrate a command above a falling speed target.
      double ramp_distance=std::max(0.,control_remaining-floor_settle_distance);
      double terminal_target=std::sqrt(std::pow(brake*response,2)+2*brake*ramp_distance)-brake*response;
      terminal_target=std::max(floor,terminal_target);
      double geometry_target=g.target(g.nearest_s,1e6,cap);
      if(geometry_target<floor) throw std::runtime_error("pid_curve_below_drivable_floor");
      double target=std::min({geometry_target,terminal_target,reference_cap});
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
      // Roll out copies of the actual PID/PI policy. Holding today's turn
      // for the entire horizon falsely curls the prediction into the lead.
      // Copies never change live integrators, actuator estimates or commands.
      int sweep_steps=coasting?int(std::ceil(measured/(.4*.05)))+2:10;
      if(!prediction_only) {
        JointControl simulated(false);
        simulated.prediction_only=true;
        simulated.minimum_drivable_speed=minimum_drivable_speed;
        simulated.command=command;simulated.acceleration=acceleration;
        simulated.steering=steering;simulated.servo=servo;simulated.integral=integral;
        simulated.floor_reached=floor_reached;simulated.coasting=coasting;
        Tracker simulated_tracker=tracker;
        Pose q=pose;double v=measured,s=servo;
        for(int i=0;i<sweep_steps;++i) {
          if(i && !simulated.coasting) {
            Geometry future_geometry(p,q);
            double rem=std::max(0.,control_remaining-(future_geometry.nearest_s-g.nearest_s));
            simulated.compute(a,p,q,v,0.,v/.25*std::tan(s),rem,cap,.05,simulated_tracker,{},reference_cap,{},reference_table);
          }
          if(simulated.coasting)
            sweep_steps=std::max(sweep_steps,i+int(std::ceil(v/(.4*.05)))+2);
          v=simulated.coasting?std::max(0.,v-.05*.4):v+.05*clip((simulated.command-v)/.15,-.8,1.0);
          s+=(simulated.steering-s)*.05/.15;
          double rate=v/.25*std::tan(s),mid=q.yaw+.025*rate;
          q.x+=.05*v*std::cos(mid);q.y+=.05*v*std::sin(mid);q.yaw+=.05*rate;
          trajectory.push_back(q);
          if(simulated.coasting && v<=1e-9)break;
        }
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
      diag["effective_command_cap_mps"]=cap;
      diag["prediction_policy"]="closed_loop_pid_pi_copied_state";
      diag["prediction_horizon_s"]=.05*(trajectory.size()-1);
      if(prediction_only)return {command,steering};
    } else {
      double control_remaining=remaining;
      if((p.back().xy()-a.route.back().xy()).norm()<.02)
        control_remaining=std::min(remaining,std::max(0.,g.arc.back()-g.nearest_s));
      const double floor=minimum_drivable_speed, brake=.55, response=.30, settle=.25;
      const double command_cap=std::max(floor,cap-.15);
      const double requested_cap=std::min(command_cap,braking_cap);
      if(!std::isfinite(requested_cap) || requested_cap<floor)throw std::runtime_error("mpc_braking_cap_below_floor");
      diag["requested_braking_cap_mps"]=requested_cap;
      diag["absolute_command_cap_mps"]=command_cap;
      diag["command_headroom_mps"]=.15;diag["effective_command_cap_mps"]=command_cap;
      if(cap<floor) throw std::runtime_error("mpc_speed_cap_below_drivable_floor");
      double stopping_speed=std::max(measured,command);
      double coast_distance=stopping_speed*stopping_speed/(2*.60)+.05*stopping_speed;
      if((floor_reached && command<=floor+.10 && control_remaining<=coast_distance)
          || control_remaining<=.05) coasting=true;
      auto speed_target=[&](double s,double rem) {
        double curve=g.target(s,1e6,command_cap);
        if(curve<floor) throw std::runtime_error("mpc_curve_below_drivable_floor");
        if(map_speed_feasibility)curve=std::max(floor,curve-.20);
        double terminal=std::max(floor,std::sqrt(std::pow(brake*response,2)+2*brake*std::max(0.,rem-settle))-brake*response);
        return std::min({curve,terminal,reference_cap,requested_cap});
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
      double ubx[4]={2.1,2.5,8.,command_cap};
#ifdef FRENET_DIAGNOSTIC_PROBE
      if(diagnostic_mask&4)lbx[3]=0.;
#endif
      const auto speed_transition=reachable_speed_transition(command,acceleration,requested_cap,command_cap,lbx[3],dt);
      diag["speed_bound_recovery_active"]=speed_transition.active;
      diag["speed_bound_recovery_stages"]=speed_transition.recovery_stages;
      diag["stage_command_upper_mps"]=Json::Value(Json::arrayValue);
      for(int i=1;i<=24;++i) {
        ubx[3]=speed_transition.upper[i];diag["stage_command_upper_mps"].append(ubx[3]);
        ocp_nlp_constraints_model_set(config,dims,in,i,"lbx",lbx);
        ocp_nlp_constraints_model_set(config,dims,in,i,"ubx",ubx);
      }
      double x[10]={pose.x,pose.y,pose.yaw,measured,lateral,yaw_rate,acceleration,steering,command,servo};
      diag["initial_state"]=Json::Value(Json::arrayValue);for(double v:x)diag["initial_state"].append(v);
      if(feasibility_recovery) {
        // The field sample already exceeded the old 1.5 v*r limit. Its exact
        // model/rate bounds cannot bring the NEXT state below 1.5. Do not fake
        // actuator state zero or request an instantaneous state change.
        // Plan normally below 1.2. Only an observed excess gets a <=250 ms
        // restoration envelope from an exact, rate-feasible braking/centering
        // rollout. Every future stage stays <=2.0, then returns to 1.2; all
        // other nonlinear, command, map/body, residual and timing guards remain.
        const double observed=std::abs(measured*yaw_rate);
        if(observed>2.0)throw std::runtime_error("joint_lateral_state_above_recovery_cap");
        diag["lateral_nominal_limit_mps2"]=1.2;
        diag["lateral_recovery_hard_cap_mps2"]=2.0;
        diag["lateral_recovery_active"]=observed>1.2;
        diag["lateral_stage_limits_mps2"]=Json::Value(Json::arrayValue);
        std::array<double,10> recovery_state{};std::copy(x,x+10,recovery_state.begin());
        for(int stage=1;stage<=24;++stage) {
          double limit=1.2;
          if(observed>1.2 && stage<=5) {
          const double da=stage==1?4*dt:.15,dd=stage==1?dt:.04;
          double a=std::max(-.8,recovery_state[6]-da);
          a=std::max(a,((floor_reached?floor:0.)-recovery_state[8])/.05);
          const double upper=std::min({2.,recovery_state[6]+da,(command_cap-recovery_state[8])/.05});
          if(a>upper+1e-8)throw std::runtime_error("joint_recovery_rollout_infeasible");
          double delta=recovery_state[7]-clip(recovery_state[7],-dd,dd);
          double u[2]={a,delta},par[5]={0,0,0,0,.2};std::array<double,10> next{};
          const double* input[3]={recovery_state.data(),u,par};double* output[1]={next.data()};
          if(joint_vehicle_dyn_disc_phi_fun(input,output,nullptr,nullptr,nullptr))
            throw std::runtime_error("joint_recovery_dynamics");
          for(double value:next)if(!std::isfinite(value))throw std::runtime_error("joint_recovery_nonfinite");
          recovery_state=next;
          limit=std::min(2.0,std::max(1.2,std::abs(next[3]*next[5])+.03));
          }
#ifdef FRENET_DIAGNOSTIC_PROBE
          if(diagnostic_mask&2)limit=2.;
#endif
          diag["lateral_stage_limits_mps2"].append(limit);
          if(stage<24) {
            double lo[5]={-1.,-.7,-limit,-.15,-.04},hi[5]={1.,.7,limit,.15,.04};
            ocp_nlp_constraints_model_set(config,dims,in,stage,"lh",lo);
            ocp_nlp_constraints_model_set(config,dims,in,stage,"uh",hi);
          } else {
            double lo[3]={-1.,-.7,-limit},hi[3]={1.,.7,limit};
            ocp_nlp_constraints_model_set(config,dims,in,stage,"lh",lo);
            ocp_nlp_constraints_model_set(config,dims,in,stage,"uh",hi);
          }
        }
      }
      ocp_nlp_constraints_model_set(config,dims,in,0,"lbx",x);
      ocp_nlp_constraints_model_set(config,dims,in,0,"ubx",x);
      double s=g.nearest_s, guess_v=measured;
      const double reference_begin=mono();double tube_ms=0.,reference_speed_ms=0.,static_corridor_ms=0.,parameter_update_ms=0.;
      diag["dynamic_corridor_enabled"]=bool(dynamic_corridor);
      diag["minimum_dynamic_corridor_radius_m"]=.20;
      diag["maximum_dynamic_reference_shift_m"]=0.;
      for(int i=0;i<=24;++i) {
        Pose ref=g.at(s);ref.yaw=pose.yaw+wrap(ref.yaw-pose.yaw);
        const double speed_begin=mono();
        double target=speed_target(s,std::max(0.,control_remaining-(s-g.nearest_s)));
        reference_speed_ms+=(mono()-speed_begin)*1000.;
        const double corridor_begin=mono();
        double param[5]={ref.x,ref.y,ref.yaw,target,corridor(a,ref,obs)};
        static_corridor_ms+=(mono()-corridor_begin)*1000.;
        if(dynamic_corridor) {
          const double tube_begin=mono();
          auto tube=dynamic_corridor(ref,.05*i);double radius=tube.second;
          tube_ms+=(mono()-tube_begin)*1000.;
          diag["maximum_dynamic_reference_shift_m"]=std::max(diag["maximum_dynamic_reference_shift_m"].asDouble(),(tube.first.xy()-ref.xy()).norm());
          param[0]=tube.first.x;param[1]=tube.first.y;
          param[4]=std::min(param[4],radius);
          diag["minimum_dynamic_corridor_radius_m"]=std::min(diag["minimum_dynamic_corridor_radius_m"].asDouble(),radius);
        }
#ifdef FRENET_DIAGNOSTIC_PROBE
        if(diagnostic_mask&1)param[4]=1.;
#endif
        const double update_begin=mono();
        joint_vehicle_acados_update_params(capsule,i,param,5);
        parameter_update_ms+=(mono()-update_begin)*1000.;
        if(!initialized) {
          double seed[10]={ref.x,ref.y,ref.yaw,guess_v,0,guess_v/.25*std::tan(steering),acceleration,steering,std::min(cap,guess_v),servo};
          if(i==0)std::copy(x,x+10,seed);
          ocp_nlp_out_set(config,dims,out,i,"x",seed);
          if(i<24){double u[2]={0,steering};ocp_nlp_out_set(config,dims,out,i,"u",u);}
        }
        double forward=initialized?predicted[std::min(i+1,24)][3]:guess_v;
        forward=clip(forward,0,cap);
        s=std::min(g.arc.back(),s+.05*std::max(.1,forward));
        guess_v=std::min(target,guess_v+1.0*.05);
      }
      diag["mpc_reference_setup_ms"]=(mono()-reference_begin)*1000.;
      diag["mpc_tube_setup_ms"]=tube_ms;
      diag["mpc_reference_speed_ms"]=reference_speed_ms;
      diag["mpc_static_corridor_ms"]=static_corridor_ms;
      diag["mpc_parameter_update_ms"]=parameter_update_ms;
      diag["reference_speed_table_used"]=bool(reference_table);
      const double seed_begin=mono();
      {
        // Advance the previous control sequence by one tick and re-integrate
        // from the CURRENT measured state using the exact generated discrete
        // model. Reusing the old unshifted state trajectory leaves an x0/model
        // defect after each real pose/velocity update, especially during launch.
        std::array<std::array<double,2>,24> shifted{};
        if(initialized)for(int i=0;i<24;++i)
          ocp_nlp_out_get(config,dims,out,std::min(i+1,23),"u",shifted[i].data());
        std::array<double,10> seed{};std::copy(x,x+10,seed.begin());
        double unused_parameters[5]={0,0,0,0,.2};
        for(int i=0;i<=24;++i) {
          ocp_nlp_out_set(config,dims,out,i,"x",seed.data());
          if(i==24)break;
          const double da=i?.15:4*dt,dd=i?.04:dt;
          if(!initialized){shifted[i][0]=2.*(diag["speed_target_mps"].asDouble()-seed[8]);shifted[i][1]=steering;}
          if(speed_transition.active)shifted[i][0]=speed_transition.acceleration[i];
          shifted[i][0]=clip(shifted[i][0],std::max(-.8,seed[6]-da),std::min(1.,seed[6]+da));
          shifted[i][1]=clip(shifted[i][1],std::max(-.269,seed[7]-dd),std::min(.269,seed[7]+dd));
          ocp_nlp_out_set(config,dims,out,i,"u",shifted[i].data());
          std::array<double,10> next{};
          const double* input[3]={seed.data(),shifted[i].data(),unused_parameters};
          double* output[1]={next.data()};
          if(joint_vehicle_dyn_disc_phi_fun(input,output,nullptr,nullptr,nullptr))
            throw std::runtime_error("joint_warm_start_dynamics");
          for(double value:next)if(!std::isfinite(value))throw std::runtime_error("joint_warm_start_nonfinite");
          seed=next;
        }
        diag["warm_start_policy"]=initialized?"shift_controls_rollout_exact_generated_dynamics_from_measurement":"cold_controls_rollout_exact_generated_dynamics_from_measurement";
      }
      diag["mpc_seed_rollout_ms"]=(mono()-seed_begin)*1000.;
      // Current localization can lie outside a newly switched narrow corridor;
      // constrain predicted future states, not an impossible change of x0.
      double lh0[5]={-1e3,-.7,-10.,-4*dt,-dt},uh0[5]={1e3,.7,10.,4*dt,dt};
      ocp_nlp_constraints_model_set(config,dims,in,0,"lh",lh0);
      ocp_nlp_constraints_model_set(config,dims,in,0,"uh",uh0);
#ifdef FRENET_DIAGNOSTIC_PROBE
      if(diagnostic_mask&8) {
        double l0[5]={-1e3,-.7,-10.,-2.,-1.},u0[5]={1e3,.7,10.,2.,1.};
        ocp_nlp_constraints_model_set(config,dims,in,0,"lh",l0);ocp_nlp_constraints_model_set(config,dims,in,0,"uh",u0);
        for(int k=1;k<24;++k){double l[5]={-1.,-.7,-diag["lateral_stage_limits_mps2"][k-1].asDouble(),-2.,-1.},u[5]={1.,.7,diag["lateral_stage_limits_mps2"][k-1].asDouble(),2.,1.};ocp_nlp_constraints_model_set(config,dims,in,k,"lh",l);ocp_nlp_constraints_model_set(config,dims,in,k,"uh",u);}
      }
#endif
      int status=0, iterations=0;
      double equality=INF, inequality=INF;
      auto solver=joint_vehicle_acados_get_nlp_solver(capsule);
      double pure_solver_ms=0.,solver_residual_ms=0.,independent_residual_ms=0.;
      for(int iteration=0;iteration<8;++iteration) {
        const double solve_begin=mono();
        status=joint_vehicle_acados_solve(capsule);++iterations;
        pure_solver_ms+=(mono()-solve_begin)*1000.;
        diag["mpc_pure_solver_ms"]=pure_solver_ms;
        diag["mpc_solver_status"]=status;diag["mpc_iterations"]=iterations;
        diag["mpc_solve_ms"]=(mono()-begin)*1000;
        if(status!=0){int qp=0;ocp_nlp_get(config,solver,"qp_status",&qp);diag["qp_status"]=qp;throw std::runtime_error("joint_mpc_solver_"+std::to_string(status));}
        const double residual_begin=mono();
        ocp_nlp_eval_residuals(solver,in,out);
        ocp_nlp_get(config,solver,"res_eq",&equality);
        ocp_nlp_get(config,solver,"res_ineq",&inequality);
        solver_residual_ms+=(mono()-residual_begin)*1000.;
        diag["mpc_solver_residual_ms"]=solver_residual_ms;
        // Independently evaluate solved-state defects. Do not accept a seed
        // merely because the SQP implementation reports a small cached norm.
        const double independent_begin=mono();
        double dynamics_defect=0.,lateral_violation=0.;
        double parameters[5]={0,0,0,0,.2};
        for(int k=0;k<24;++k) {
          double state[10],next_state[10],control[2],integrated[10];
          ocp_nlp_out_get(config,dims,out,k,"x",state);
          ocp_nlp_out_get(config,dims,out,k+1,"x",next_state);
          if(feasibility_recovery)
            lateral_violation=std::max(lateral_violation,
              std::abs(next_state[3]*next_state[5])-diag["lateral_stage_limits_mps2"][k].asDouble());
          ocp_nlp_out_get(config,dims,out,k,"u",control);
          const double* input[3]={state,control,parameters};double* output[1]={integrated};
          if(joint_vehicle_dyn_disc_phi_fun(input,output,nullptr,nullptr,nullptr))
            throw std::runtime_error("joint_residual_dynamics");
          for(int kx=0;kx<10;++kx) {
            if(!std::isfinite(integrated[kx]+next_state[kx]))throw std::runtime_error("joint_nonfinite_residual");
            dynamics_defect=std::max(dynamics_defect,std::abs(integrated[kx]-next_state[kx]));
          }
        }
        equality=std::max(equality,dynamics_defect);
        independent_residual_ms+=(mono()-independent_begin)*1000.;
        diag["mpc_independent_residual_ms"]=independent_residual_ms;
        inequality=std::max(inequality,lateral_violation);
        if(feasibility_recovery)diag["independent_lateral_violation"]=lateral_violation;
        diag["independent_dynamics_defect"]=dynamics_defect;
        diag["equality_residual"]=equality;diag["inequality_residual"]=inequality;
        if(!std::isfinite(equality+inequality))throw std::runtime_error("joint_nonfinite_residual");
        if(equality<.001 && inequality<.001)break;
        if(mono()-begin>std::min(.085,MpcTimingBudget::hard_seconds()-.015))break;
      }
      if(equality>=.001 || inequality>=.001)throw std::runtime_error("joint_mpc_nonlinear_infeasible eq="+std::to_string(equality)+" ineq="+std::to_string(inequality));
      double u[2];ocp_nlp_out_get(config,dims,out,0,"u",u);
      for(int i=0;i<=24;++i) {
        ocp_nlp_out_get(config,dims,out,i,"x",predicted[i].data());
        for(double value:predicted[i])if(!std::isfinite(value))throw std::runtime_error("joint_nonfinite");
        if(i && (predicted[i][8]>speed_transition.upper[i]+.001 || predicted[i][8]>command_cap+.001 || predicted[i][8]<lbx[3]-.001))
          throw std::runtime_error("joint_speed_transition_violation");
        if(feasibility_recovery && i) {
          double load=std::abs(predicted[i][3]*predicted[i][5]);
          if(load>diag["lateral_stage_limits_mps2"][i-1].asDouble()+.001)
            throw std::runtime_error("joint_lateral_recovery_constraint");
          diag["max_predicted_lateral_load_mps2"]=std::max(diag.get("max_predicted_lateral_load_mps2",0.).asDouble(),load);
        }
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
    diag["cross_track_error_m"]=g.cross;diag["heading_error_rad"]=g.heading;
    diag["measured_speed_mps"]=measured;diag["acceleration_command_mps2"]=acceleration;
    diag["speed_command_mps"]=command;
    timespec cpu_end{};
    if(cpu_clock_valid && clock_gettime(CLOCK_THREAD_CPUTIME_ID,&cpu_end)==0) {
      double cpu=(cpu_end.tv_sec-cpu_begin.tv_sec)+(cpu_end.tv_nsec-cpu_begin.tv_nsec)*1e-9;
      diag["controller_thread_cpu_ms"]=cpu*1000;
      diag["controller_wait_wall_ms"]=std::max(0.,mono()-begin-cpu)*1000;
    }
    const double elapsed=mono()-begin;
    diag["controller_compute_ms"]=elapsed*1000;
    diag["compute_hard_deadline_ms"]=1000*MpcTimingBudget::hard_seconds();
    if(capsule) {
      const char *failure=timing_budget.observe(elapsed);
      diag["compute_soft_budget_ms"]=1000*MpcTimingBudget::soft_seconds();
      diag["compute_hard_deadline_ms"]=1000*MpcTimingBudget::hard_seconds();
      diag["compute_soft_budget_missed"]=elapsed>MpcTimingBudget::soft_seconds();
      diag["compute_consecutive_soft_misses"]=timing_budget.consecutive();
      diag["compute_max_consecutive_soft_misses"]=MpcTimingBudget::max_consecutive();
      if(failure)throw std::runtime_error(failure);
    } else if(elapsed>MpcTimingBudget::hard_seconds())throw std::runtime_error("joint_control_deadline");
    return {command,steering};
  }
};
} // namespace frenet
