// No ROS graph, sensors, or actuators. Uses synthetic routes in loaded assets.
#include "joint_control.hpp"
#define private public
#include "takeover_sim/dynamic_overtaking.hpp"
#undef private
#include <cassert>
#include <iostream>

int main(int argc, char **argv) {
  assert(argc == 2);
  frenet::Assets assets(argv[1]);
  assets.route.clear(); assets.arc.clear();
  constexpr double pi=3.14159265358979323846;
  for(int i=0;i<=1000;++i) {
    const double angle=2*pi*i/1000.;
    assets.route.push_back({10+8*std::cos(angle),10+8*std::sin(angle),angle+pi/2});
    assets.arc.push_back(8*angle);
  }
  assets.closed_single_lap=true;
  takeover::Track target;
  target.valid=true; target.position={18,10}; target.velocity={0,1}; target.yaw=pi/2;
  const auto curve=takeover::predict_track(assets,target,4,.1,.08,true);
  const auto line=takeover::predict_track(assets,target,4,.1,.08,false);
  assert(std::abs((curve.back().pose.xy()-takeover::Point(10,10)).norm()-8)<.002);
  assert((curve.front().pose.xy()-target.position).norm()<.001);
  assert(std::abs(line.back().pose.x-18)<1e-9 && std::abs(line.back().pose.y-14)<1e-8);
  assert(curve.back().sigma>curve.front().sigma);
  // A restart crossing a route knot must see a continuous tangent.
  for(int i:{1,200,500,999}) {
    const double s=assets.arc[i],eps=.001;
    const auto lo=takeover::route_pose(assets,s-eps);
    const auto hi=takeover::route_pose(assets,s+eps);
    assert(std::abs(takeover::wrap(hi.yaw-lo.yaw))<.001);
  }
  // A crossing vehicle must not be forced to follow the route prior.
  target.velocity={1,0};
  const auto crossing=takeover::predict_track(assets,target,1,.1,.08,true);
  assert(std::abs(crossing.back().pose.x-19)<1e-8);
  target.position={18.5,10}; target.velocity={0,1};
  const auto offset=takeover::predict_track(assets,target,1,.1,.08,true);
  assert(std::abs((offset.back().pose.xy()-takeover::Point(10,10)).norm()-8.5)<.003);

  frenet::TimedReference reference;
  reference.points={{0,{0,0,3.1},1},{1,{1,0,-3.1},2},{2,{2,0,-3.0},2}};
  reference.age=.2;
  reference.validate();
  assert(std::abs(reference.at(.3).pose.x-.5)<1e-12);
  assert(std::abs(reference.at(.3).speed-1.5)<1e-12);
  assert(std::abs(std::abs(reference.at(.3).pose.yaw)-pi)<.001);
  bool expired=false;
  try { reference.at(2.); } catch(const std::runtime_error&) { expired=true; }
  assert(expired);
  auto malformed=reference;
  malformed.points[1].t=0;
  bool invalid=false;
  try { malformed.validate(); } catch(const std::runtime_error&) { invalid=true; }
  assert(invalid);
  malformed=reference;malformed.points[1].speed=std::numeric_limits<double>::quiet_NaN();
  invalid=false;
  try { malformed.validate(); } catch(const std::runtime_error&) { invalid=true; }
  assert(invalid);

  // Straight wide road, with no wall constraint hiding recovery behavior.
  assets.route.clear(); assets.arc.clear(); assets.closed_single_lap=false;
  for(int i=0;i<2000;++i) {assets.route.push_back({i*.05,0,0});assets.arc.push_back(i*.05);}
  assets.res=1;assets.ox=-10;assets.oy=-15;assets.width=130;assets.height=30;
  assets.grid.assign(3900,5.);
  takeover::DynamicOvertakingPlanner planner(assets);
  planner.initial_command=1.5; planner.initial_acceleration=0;
  takeover::Track absent;
  auto braking=planner.build_candidate({10,0,0},1.5,absent,0,takeover::Behavior::FOLLOW,.8);
  assert(braking.valid && braking.timed.size()==41);
  assert(braking.timed[1].speed>1.45); // no instantaneous cap truncation
  assert(braking.timed.back().speed<.9);
  double integrated=0;
  for(size_t i=1;i<braking.timed.size();++i)
    integrated+=.05*(braking.timed[i-1].speed+braking.timed[i].speed);
  assert(std::abs(braking.timed.back().s-10-integrated)<.01);
  planner.step=.05;
  const auto jerk_limited=planner.build_candidate({10,0,0},1.5,absent,0,takeover::Behavior::FOLLOW,.8);
  assert(jerk_limited.valid);
  double previous_command=1.5,previous_acceleration=0.;
  for(size_t i=1;i<jerk_limited.timed.size();++i) {
    const double before=jerk_limited.timed[i-1].speed,after=jerk_limited.timed[i].speed;
    const double command=before+(after-before)*(.15+.05)/.05;
    const double acceleration=(command-previous_command)/.05;
    assert(std::abs(acceleration-previous_acceleration)/.05<=3.+1e-7);
    previous_command=command;previous_acceleration=acceleration;
  }
  planner.step=.1;

  // High-speed braking must begin BEFORE a bend, retaining the measured
  // initial speed and actuator acceleration/jerk rather than clipping it.
  auto bend_assets=assets;
  bend_assets.route.clear();bend_assets.arc.clear();
  for(int i=0;i<=400;++i)bend_assets.route.push_back({i*.05,0,0});
  for(int i=1;i<=100;++i) {
    const double angle=pi*.5*i/100.;
    bend_assets.route.push_back({20+2*std::sin(angle),2-2*std::cos(angle),angle});
  }
  for(int i=1;i<=300;++i)bend_assets.route.push_back({22,2+i*.05,pi*.5});
  bend_assets.arc.push_back(0.);
  for(size_t i=1;i<bend_assets.route.size();++i)
    bend_assets.arc.push_back(bend_assets.arc.back()+
        (bend_assets.route[i].xy()-bend_assets.route[i-1].xy()).norm());
  takeover::DynamicOvertakingPlanner fast_planner(bend_assets);
  fast_planner.cruise_speed=3.;fast_planner.initial_command=3.;
  const auto bend=fast_planner.build_candidate({15,0,0},3.,absent,0,takeover::Behavior::CRUISE,3.);
  assert(bend.valid && bend.timed.front().speed==3.);
  assert(bend.timed[1].speed>2.9 && bend.timed[1].speed<3.);
  assert(bend.timed[30].speed<1.9);
  fast_planner.require_target=false;
  assert(!fast_planner.plan({15,0,0},3.,absent,5.).stop);
  fast_planner.require_target=true;
  assert(fast_planner.plan({15,0,0},3.,absent,5.).stop);

  target.position={14,0};target.velocity={.8,0};target.yaw=0;
  target.observation_stamp=2.;target.position_variance=.0001;target.velocity_variance=.0001;
  planner.behavior_=takeover::Behavior::CANCEL;planner.cancel_start_=0;
  planner.committed_side_=1;planner.aborting_=true;
  auto recovering=planner.plan({10,.65,0},.8,target,2.);
  assert(recovering.behavior==takeover::Behavior::REJOIN);
  assert(recovering.committed_side==1 && recovering.candidate.valid);
  auto recovered=planner.plan({10,0,0},.8,target,2.);
  assert(recovered.overtakes_completed==0);
  assert(recovered.behavior==takeover::Behavior::FOLLOW && recovered.committed_side==0);
  takeover::DynamicOvertakingPlanner recovery_speed(assets);
  recovery_speed.adaptive_offset=true; recovery_speed.cruise_speed=1.8;
  recovery_speed.behavior_=takeover::Behavior::CANCEL;
  recovery_speed.cancel_start_=0.; recovery_speed.committed_side_=1;
  recovery_speed.aborting_=true; recovery_speed.initial_command=.8;
  const auto controlled_recovery=recovery_speed.plan({10,.65,0},.8,target,2.);
  assert(controlled_recovery.behavior==takeover::Behavior::REJOIN);
  assert(controlled_recovery.candidate.valid && controlled_recovery.candidate.speed_cap<=.8);
  recovery_speed.overtakes_completed_=1;recovery_speed.behavior_=takeover::Behavior::REJOIN;
  recovery_speed.aborting_=true;recovery_speed.committed_side_=1;
  target.position={19,0};target.observation_stamp=0.;
  const auto far_recovery=recovery_speed.plan({10,0,0},1.,target,2.);
  assert(!far_recovery.stop && far_recovery.behavior==takeover::Behavior::CRUISE);
  assert(far_recovery.overtakes_completed==1 && far_recovery.committed_side==0);
  target.position={14,0};
  // Longer observation loss still stops; route assistance is not a sensor.
  target.observation_stamp=-10.;
  auto stale=planner.plan({10,0,0},.8,target,2.);
  assert(stale.stop && stale.behavior==takeover::Behavior::WAIT);
  // Curvature recovery changes geometry, never the steering threshold.
  // Same-side adaptation must search offsets without changing the locked side.
  takeover::DynamicOvertakingPlanner adaptive(assets);
  adaptive.adaptive_offset=true;adaptive.initial_command=1.5;
  adaptive.committed_side_=1;adaptive.committed_offset_=.82;
  adaptive.maneuver_start_s_=10.;
  target.position={12.5,.4};target.velocity={.8,0};
  auto flexible=adaptive.build_candidate({10,.82,0},1.5,target,1,takeover::Behavior::OVERTAKE,1.5);
  std::cout<<"Adaptive shifted-target candidate: "<<flexible.reason<<" offset="<<flexible.target_offset<<std::endl;
  assert(flexible.valid && flexible.side==1 && flexible.target_offset>.82);
  assert(flexible.evaluated_candidates>1);
  target.position={14.,0.};
  auto entry=adaptive.build_candidate({10,0,0},.85,target,1,takeover::Behavior::OVERTAKE,.85);
  assert(entry.valid && entry.target_offset>=.82);
  target.position={10,0};
  auto impossible=adaptive.build_candidate({10,0,0},1.5,target,1,takeover::Behavior::OVERTAKE,1.5);
  assert(!impossible.valid); // sampling more candidates cannot erase a collision
  target.position={14.,0.};
  assets.grid.assign(3900,.20);
  auto narrow=adaptive.build_candidate({10,0,0},.85,target,1,takeover::Behavior::OVERTAKE,.85);
  assert(!narrow.valid && narrow.reason=="mpc_center_map_clearance");
  assets.grid.assign(3900,5.);
  // A downstream wall beyond the slow-entry horizon must prevent commitment
  // on that side, without suppressing an otherwise open opposite corridor.
  for(int row=0;row<assets.height;++row) for(int col=0;col<assets.width;++col) {
    const double x=assets.ox+col*assets.res;
    const double y=assets.oy+(assets.height-1-row)*assets.res;
    if(x>=18 && x<=22 && y<0) assets.grid[row*assets.width+col]=0.;
  }
  assert(!adaptive.entry_corridor_clear({10,0,0},target,-.82));
  assert(adaptive.entry_corridor_clear({10,0,0},target,.82));
  assets.grid.assign(3900,5.);
  // Beginning a return must not instantly reverse the vehicle's measured
  auto tight_assets=assets;tight_assets.route.clear();tight_assets.arc.clear();
  tight_assets.closed_single_lap=true;
  for(int i=0;i<=1000;++i) {
    const double angle=2*pi*i/1000.;
    tight_assets.route.push_back({10+1.6*std::cos(angle),10+1.6*std::sin(angle),angle+pi/2});
    tight_assets.arc.push_back(1.6*angle);
  }
  takeover::DynamicOvertakingPlanner tight(tight_assets);
  tight.cruise_speed=3.;tight.wheelbase=.3302;tight.maximum_steering=.4189;
  auto curve_lead=target;curve_lead.position={11.6,10};curve_lead.velocity={0,1.8};
  assert(!tight.entry_corridor_clear({11.6,10,pi/2},curve_lead,.9));
  assert(tight.entry_corridor_clear({11.6,10,pi/2},curve_lead,-.9));
  // A wide, steerable outer bend is not necessarily fast enough to pass.
  // The optional high-speed SQP admission must account for offset arc length
  // and curvature speed, not assume the maximum cruise speed throughout.
  tight.sqp_optimizer=[](const Json::Value &) {return Json::Value();};
  curve_lead.velocity={0,1.8};
  assert(!tight.entry_corridor_clear({11.6,10,pi/2},curve_lead,-.9));
  takeover::DynamicOvertakingPlanner straight_budget(assets);
  straight_budget.cruise_speed=3.;straight_budget.sqp_optimizer=tight.sqp_optimizer;
  auto straight_lead=target;straight_lead.position={12,0};straight_lead.velocity={1.8,0};
  assert(straight_budget.entry_corridor_clear({10,0,0},straight_lead,.82));
  assert(straight_budget.following_gap(3.,1.8)==straight_budget.desired_gap);
  straight_budget.adaptive_offset=true;
  assert(straight_budget.following_gap(1.8,1.8)>1.9);
  assert(straight_budget.following_gap(3.,1.8)>3.3);
  straight_budget.cruise_speed=2.;
  assert(straight_budget.following_gap(2.,1.8)==straight_budget.desired_gap);
  // Beginning a return must not instantly reverse the vehicle's measured
  // curvature. Use fine time steps only in this local derivative regression.
  adaptive.step=.01;adaptive.initial_curvature=.2;adaptive.maneuver_start_s_=10.;
  auto curved_return=adaptive.build_candidate({10,.7,0},1.5,absent,1,
      takeover::Behavior::REJOIN,1.5);
  assert(curved_return.valid);
  const takeover::Point e1=curved_return.path[1].xy()-curved_return.path[0].xy();
  const takeover::Point e2=curved_return.path[2].xy()-curved_return.path[1].xy();
  const double initial_k=2*(e1.x()*e2.y()-e1.y()*e2.x())/
      (e1.norm()*e2.norm()*(e1+e2).norm());
  assert(initial_k>.10 && initial_k<.30);
  adaptive.cruise_speed=3.;
  const auto fast_cap_return=adaptive.build_candidate({10,.7,0},1.5,absent,1,
      takeover::Behavior::REJOIN,3.);
  assert(fast_cap_return.valid && fast_cap_return.transition_length<4.);
  adaptive.cruise_speed=1.5;
  adaptive.sqp_optimizer=[](const Json::Value &)->Json::Value {
    throw std::runtime_error("cancel_must_not_call_overtake_optimizer");
  };
  const auto curved_cancel=adaptive.build_candidate({10,.7,0},1.5,absent,1,
      takeover::Behavior::CANCEL,.7);
  assert(curved_cancel.valid);
  const takeover::Point c1=curved_cancel.path[1].xy()-curved_cancel.path[0].xy();
  const takeover::Point c2=curved_cancel.path[2].xy()-curved_cancel.path[1].xy();
  const double cancel_k=2*(c1.x()*c2.y()-c1.y()*c2.x())/
      (c1.norm()*c2.norm()*(c1+c2).norm());
  std::cout<<"Cancel initial curvature "<<cancel_k<<" transition "<<curved_cancel.transition_length<<std::endl;
  assert(cancel_k>.10 && cancel_k<.30);
  adaptive.sqp_optimizer={};
  int preview_checks=0;
  adaptive.return_feasibility=[&](const takeover::Candidate &) {return ++preview_checks>1;};
  auto longer_return=adaptive.build_candidate({10,.7,0},1.5,absent,1,
      takeover::Behavior::REJOIN,1.5);
  assert(longer_return.valid && preview_checks>1 &&
      longer_return.transition_length>curved_return.transition_length);
  adaptive.return_feasibility=[](const takeover::Candidate &) {return false;};
  auto rejected_return=adaptive.build_candidate({10,.7,0},1.5,absent,1,
      takeover::Behavior::REJOIN,1.5);
  assert(!rejected_return.valid && rejected_return.reason=="mpc_return_preview_rejected");
  adaptive.return_feasibility={};
  assert(!takeover::return_clearance_reserved(.0035));
  assert(!takeover::return_clearance_reserved(std::numeric_limits<double>::quiet_NaN()));
  assert(takeover::return_clearance_reserved(.10));
  takeover::Candidate bounded_return;
  bounded_return.timed={{0.,10.,.7,1.5,{10,.7,0}}, {2.,13.,0.,1.5,{13,0,0}}};
  assert(!takeover::return_alignment_before(assets,bounded_return,1.9));
  assert(takeover::return_alignment_before(assets,bounded_return,2.0));
  bounded_return.timed.back().pose.yaw=.2;
  assert(!takeover::return_alignment_before(assets,bounded_return,3.0));
  adaptive.step=.10;adaptive.initial_curvature=std::numeric_limits<double>::quiet_NaN();
  adaptive.adaptive_offset=false;
  auto fixed=adaptive.build_candidate({10,0,0},.85,target,1,takeover::Behavior::OVERTAKE,.85);
  assert(fixed.valid && std::abs(fixed.target_offset-.82)<1e-9 && fixed.evaluated_candidates==0);
  // A mock constant-lane SQP isolates downstream admission, not optimization.
  // Rejected MPC previews must never fall back to old overtaking candidates.
  takeover::DynamicOvertakingPlanner sqp(assets);
  sqp.adaptive_offset=true;sqp.initial_command=1.5;sqp.initial_curvature=0.;
  sqp.behavior_=takeover::Behavior::OVERTAKE;sqp.committed_side_=1;
  sqp.maneuver_start_s_=10.;
  sqp.sqp_optimizer=[](const Json::Value &request) {
    Json::Value out;out["ok"]=true;
    for(unsigned i=0;i<request["s"].size();++i) {
      out["d"].append(.82);
      if(i+1<request["s"].size()) {
        Json::Value row;row.append(0.);row.append(0.);row.append(0.);row.append(.82);
        out["coefficients"].append(row);
      }
    }
    return out;
  };
  takeover::Track distant=target;distant.position={14.,0.};distant.velocity={.8,0.};distant.valid=true;
  int sqp_checks=0;
  sqp.overtake_feasibility=[&](const takeover::Candidate &) {++sqp_checks;return false;};
  auto rejected_sqp=sqp.build_candidate({10,.82,0},1.5,distant,1,takeover::Behavior::OVERTAKE,1.5);
  assert(!rejected_sqp.valid && rejected_sqp.reason=="mpc_sqp_preview_rejected" && sqp_checks==3);
  sqp.overtake_feasibility=[](const takeover::Candidate &) {return true;};
  assert(sqp.build_candidate({10,.82,0},1.5,distant,1,takeover::Behavior::OVERTAKE,1.5).valid);
  assets.grid.assign(3900,.30);
  assert(!sqp.build_candidate({10,.82,0},1.5,distant,1,takeover::Behavior::OVERTAKE,1.5).valid);
  assets.grid.assign(3900,5.);
  int rescued=0;
  planner.maneuver_start_s_=8.;
  for(double d:{.2,.4,.7}) for(double h:{-.6,-.4,-.2,.2,.4,.6}) {
    auto short_path=planner.build_candidate_scaled({10,d,h},1.2,absent,1,
        takeover::Behavior::REJOIN,1.2,1.,planner.pass_offset);
    auto searched=planner.build_candidate({10,d,h},1.2,absent,1,
        takeover::Behavior::REJOIN,1.2);
    if(short_path.reason=="curvature_limit" && searched.valid) {
      ++rescued;
      assert(searched.transition_length>short_path.transition_length);
      assert(searched.maximum_curvature<=std::tan(.269)/.25+.05);
    }
  }
  assert(rescued>0);
  // A small speed advantage at the current gap must not be held open-loop
  // over the whole horizon. Following adapts the future catch-up speed.
  planner.cruise_speed=.9;planner.initial_command=.9;planner.initial_acceleration=0.;
  target.position={11.3,0};target.velocity={.8,0};target.observation_stamp=2.;
  auto following=planner.build_candidate({10,0,0},.9,target,0,takeover::Behavior::FOLLOW,.9);
  assert(following.valid && following.timed.back().speed<.86);
  planner.behavior_=takeover::Behavior::OVERTAKE;planner.committed_side_=1;
  planner.aborting_=false;target.position={12,0};
  auto no_advantage=planner.plan({10,.15,0},.8,target,2.);
  assert(no_advantage.behavior==takeover::Behavior::CANCEL);
  assert(no_advantage.committed_side==1 && no_advantage.candidate.valid);
  // A single tick can cross both rejoin and observation-age thresholds. The
  // completed pass must be recognized before applying unfinished-loss stop.
  planner.behavior_=takeover::Behavior::REJOIN;planner.committed_side_=1;
  planner.aborting_=false;target.position={7,0};target.observation_stamp=-2.05;
  takeover::Decision previous_tick;
  previous_tick.behavior=takeover::Behavior::REJOIN;
  previous_tick.target_fresh=true;
  // At 20 Hz the target expires before the next 5 Hz plan. Request that plan
  // immediately, rather than emitting an MPC fail-stop from stale behavior.
  assert(!takeover::freshness_replan_required(previous_tick,target,1.94,1.2,4.));
  assert(takeover::freshness_replan_required(previous_tick,target,2.,1.2,4.));
  auto rejoined_at_expiry=planner.plan({10,.03,.10},.9,target,2.);
  assert(rejoined_at_expiry.behavior==takeover::Behavior::CRUISE);
  assert(rejoined_at_expiry.overtakes_completed==1 && !rejoined_at_expiry.stop);
  assert(!takeover::freshness_replan_required(rejoined_at_expiry,target,2.05,1.2,4.));
  // Same expired observation while still offset MUST retain fail-stop.
  takeover::DynamicOvertakingPlanner unfinished(assets);
  unfinished.behavior_=takeover::Behavior::REJOIN;unfinished.committed_side_=1;
  auto still_offset=unfinished.plan({10,.4,.10},.9,target,2.);
  assert(still_offset.stop && still_offset.behavior==takeover::Behavior::WAIT);
  assert(!takeover::freshness_replan_required(still_offset,target,2.05,1.2,4.));
  previous_tick.behavior=takeover::Behavior::FOLLOW;target.observation_stamp=0.;
  assert(!takeover::freshness_replan_required(previous_tick,target,1.19,1.2,4.));
  assert(takeover::freshness_replan_required(previous_tick,target,1.21,1.2,4.));
  // Start a footprint-validated return once bodies are longitudinally clear;
  // do not wait for the larger completion gap before even beginning to merge.
  takeover::DynamicOvertakingPlanner return_planner(assets);
  return_planner.behavior_=takeover::Behavior::OVERTAKE;
  return_planner.committed_side_=1;return_planner.maneuver_start_s_=5.;
  return_planner.initial_command=1.5;target.observation_stamp=2.;
  target.position={10.,0};target.velocity={.8,0};
  auto return_early=return_planner.plan({10.75,.82,0},1.5,target,2.);
  assert(return_early.behavior==takeover::Behavior::REJOIN && return_early.candidate.valid);
  assert(return_early.overtakes_completed==0); // not yet aligned/completed
  takeover::DynamicOvertakingPlanner right_only(assets);
  right_only.allowed_pass_side=-1;right_only.prepare_start_=0.;
  right_only.behavior_=takeover::Behavior::PREPARE_OVERTAKE;
  target.position={13.,0};
  auto right_entry=right_only.plan({10.,0,0},.8,target,2.);
  assert(right_entry.behavior==takeover::Behavior::OVERTAKE);
  assert(right_entry.committed_side==-1 && right_entry.candidate.valid);
  frenet::Assets fitz(argv[1]);
  takeover::DynamicOvertakingPlanner restart(fitz);
  restart.cruise_speed=3.;restart.adaptive_offset=true;
  const auto restarted=restart.build_candidate({1.5841287679,15.7142618943,-3.0568555153},
      0.,absent,0,takeover::Behavior::CRUISE,3.);
  std::cout<<"Fitz restart "<<restarted.reason<<" curvature "<<restarted.maximum_curvature<<std::endl;
  assert(restarted.valid); // measured stranded pose from the preserved failed trial
  std::cout<<"Prediction, timing, longitudinal lag, cancellation and stale-target regressions passed\n";
}
