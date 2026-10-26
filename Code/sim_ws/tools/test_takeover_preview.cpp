// Separate solver instances only: no ROS, publishers, devices or actuation.
#include "joint_control.hpp"
#include <cassert>
#include <iostream>

int main(int argc,char **argv) {
  assert(argc==2);
  auto *capsule=joint_vehicle_acados_create_capsule();
  assert(joint_vehicle_acados_create(capsule)==0);
  double upper[2],lower[2];
  ocp_nlp_constraints_model_get(joint_vehicle_acados_get_nlp_config(capsule),
      joint_vehicle_acados_get_nlp_dims(capsule),joint_vehicle_acados_get_nlp_in(capsule),0,"ubu",upper);
  ocp_nlp_constraints_model_get(joint_vehicle_acados_get_nlp_config(capsule),
      joint_vehicle_acados_get_nlp_dims(capsule),joint_vehicle_acados_get_nlp_in(capsule),0,"lbu",lower);
  assert(std::abs(upper[1]-frenet::joint_model::steering_limit)<1e-9);
  assert(std::abs(lower[1]+frenet::joint_model::steering_limit)<1e-9);
  // Inspect actual generated bounds, including every future shooting stage.
  for(int stage=0;stage<24;++stage) {
    ocp_nlp_constraints_model_get(joint_vehicle_acados_get_nlp_config(capsule),
        joint_vehicle_acados_get_nlp_dims(capsule),joint_vehicle_acados_get_nlp_in(capsule),stage,"ubu",upper);
    ocp_nlp_constraints_model_get(joint_vehicle_acados_get_nlp_config(capsule),
        joint_vehicle_acados_get_nlp_dims(capsule),joint_vehicle_acados_get_nlp_in(capsule),stage,"lbu",lower);
    assert(std::abs(upper[0]-frenet::joint_model::acceleration_limit)<1e-9);
    assert(std::abs(lower[0]+frenet::joint_model::deceleration_limit)<1e-9);
  }
#ifdef TAKEOVER_OFFICIAL_GYM_MODEL
  assert(frenet::joint_model::acceleration_limit==3.0);
  assert(frenet::joint_model::deceleration_limit==3.0);
#else
  assert(frenet::joint_model::acceleration_limit==1.0);
  assert(frenet::joint_model::deceleration_limit==.8);
#endif
  joint_vehicle_acados_free(capsule);joint_vehicle_acados_free_capsule(capsule);
  frenet::Assets assets(argv[1]);
  assets.route.clear();assets.arc.clear();assets.closed_single_lap=false;
  for(int i=0;i<200;++i) {assets.route.push_back({i*.05,0,0});assets.arc.push_back(i*.05);}
  assets.res=1;assets.ox=-10;assets.oy=-15;assets.width=130;assets.height=30;
  assets.grid.assign(3900,5.);
  frenet::JointControl live(true),preview(true);
  frenet::Tracker tracker("dynamic_mpc");
  frenet::TimedReference timed;
  for(int i=0;i<=40;++i) timed.points.push_back({i*.1,{i*.1,0,0},1.});
  live.compute(assets,assets.route,{0,0,0},0,0,0,100,1.5,.05,tracker,{},&timed);
  const double command=live.command_speed(),accel=live.command_acceleration();
  const auto diag=frenet::dump(live.diag);
  preview.copy_runtime_from(live);
  const auto proposed=preview.compute(assets,assets.route,{0,0,0},0,0,0,100,1.5,.05,tracker,{},&timed);
  assert(std::isfinite(proposed.speed+proposed.angle));
  assert(preview.command_speed()>command);
  assert(live.command_speed()==command && live.command_acceleration()==accel);
  assert(frenet::dump(live.diag)==diag);
  bool self_copy=false;
  try {live.copy_runtime_from(live);} catch(const std::runtime_error &) {self_copy=true;}
  assert(self_copy && live.command_speed()==command);
  // Runtime limits must reach every shooting stage, including the terminal
  // state. A 2.8 m/s state is infeasible under the generated default bounds.
  frenet::JointControl fast(true,3.0),fast_preview(true,3.0);
  fast.cold_retry(2.8);
  frenet::TimedReference fast_ref;
  for(int i=0;i<=40;++i)fast_ref.points.push_back({i*.1,{i*.28,0,0},2.8});
  const auto fast_command=fast.compute(assets,assets.route,{0,0,0},2.8,0,0,100,3.0,.05,tracker,{},&fast_ref);
  assert(fast_command.speed>2.7 && fast_command.speed<=3.0);
  assert(fast.diag["configured_speed_limit_mps"].asDouble()==3.0);
  fast_preview.copy_runtime_from(fast);
  bool default_rejected=false;
  try {live.compute(assets,assets.route,{0,0,0},2.8,0,0,100,3.0,.05,tracker,{},&fast_ref);}
  catch(const std::runtime_error &e) {default_rejected=std::string(e.what())=="joint_state_invalid";}
  assert(default_rejected);
  frenet::JointControl measured_yaw(true,3.0,true);
  measured_yaw.cold_retry(2.8);
  measured_yaw.compute(assets,assets.route,{0,0,0},2.8,0,.1,100,3.,.05,tracker,{},&fast_ref);
  assert(measured_yaw.diag["model_yaw_rate_rad_s"].asDouble()==.1);
  assert(measured_yaw.diag["yaw_rate_source"].asString()=="measured_simulation_odom");
  std::cout<<"Non-actuating MPC preview preserves the live controller state\n";
}
