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
  // Longer observation loss still stops; route assistance is not a sensor.
  target.observation_stamp=-10.;
  auto stale=planner.plan({10,0,0},.8,target,2.);
  assert(stale.stop && stale.behavior==takeover::Behavior::WAIT);
  // Curvature recovery changes geometry, never the steering threshold.
  int rescued=0;
  planner.maneuver_start_s_=8.;
  for(double d:{.2,.4,.7}) for(double h:{-.6,-.4,-.2,.2,.4,.6}) {
    auto short_path=planner.build_candidate_scaled({10,d,h},1.2,absent,1,
        takeover::Behavior::REJOIN,1.2,1.);
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
  std::cout<<"Prediction, timing, longitudinal lag, cancellation and stale-target regressions passed\n";
}
