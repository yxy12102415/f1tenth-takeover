#include <iostream>
#include <fstream>
#include "takeover_sim/dynamic_overtaking.hpp"
int main(int argc,char**argv){
 if(argc!=2 && argc!=3)return 2;
 const bool slow=argc==3 && std::string(argv[2])=="slow_lead";
 const std::vector<double> ego_caps=slow?std::vector<double>{1.5,1.85,2.0}:std::vector<double>{1.5,1.8,2.0};
 const std::vector<double> leads=slow?std::vector<double>{.1,.15,.3,.5}:std::vector<double>{.5,.6};
 const std::vector<double> gaps=slow?std::vector<double>{.8,1.,1.5}:std::vector<double>{1.5,2.,2.25};
 frenet::Assets a(argv[1]);Json::Value rows(Json::arrayValue);
 for(double cruise:ego_caps)for(double lead:leads)for(double gap:gaps) {
  takeover::DynamicOvertakingPlanner p(a);p.cruise_speed=cruise;p.adaptive_offset=true;p.fast_pass_geometry=true;p.verified_exit_recovery=true;p.predictive_speed_profile=true;
  p.wheelbase=.25;p.maximum_steering=.269;p.acceleration_max=1.;p.deceleration_max=.8;
  takeover::Track target;auto pose=takeover::route_pose(a,gap);target.position=pose.xy();target.yaw=pose.yaw;target.velocity=lead*frenet::Point(cos(pose.yaw),sin(pose.yaw));target.position_variance=.0025;target.velocity_variance=.04;target.valid=target.measured=true;
  takeover::Decision d;
  // Stationary-pose, perfect-estimate admission screen: intentionally optimistic.
  // No truth-fed driving, actual trajectory execution, or physical acceptance.
  for(int i=0;i<6;++i){double t=1.+.2*i;target.stamp=target.observation_stamp=t;d=p.plan(a.route.front(),0.,target,t);}
  Json::Value row;row["ego_cap_mps"]=cruise;row["lead_mps"]=lead;row["gap_m"]=gap;row["behavior"]=takeover::behavior_name(d.behavior);row["valid"]=d.candidate.valid;row["reason"]=d.candidate.reason;row["side"]=d.candidate.side;row["planning_ms"]=d.planning_ms;row["ideal_straight_distance_m"]=cruise*(gap+1.2)/(cruise-lead);row["planner_nominal_corridor_budget_m"]=p.lane_change_length+p.rejoin_length+row["ideal_straight_distance_m"].asDouble();rows.append(row);
 }
 Json::Value report;report["route_length_m"]=a.arc.back();report["scope"]="Optimistic native Frenet entry screen only; no SQP callback, no MPC preview, no motion. Passing this does not admit execution.";report["cases"]=rows;std::cout<<report<<std::endl;
}
