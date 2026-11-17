#define FRENET_DIAGNOSTIC_PROBE
#include "joint_control.hpp"
#include "generic_avoidance.hpp"
#include <iostream>
using namespace frenet;
int main(int argc,char**argv){
 JointControl* controller=nullptr;Json::Value out;
 try{Assets a(argv[1]);auto f=json_read(argv[2]);auto config=json_read(argv[3]);auto input=f["input"];Pose pose=path_read(f["pose_path"])[0];
 Path path=path_read(f["path"]);Geometry geometry(path,pose,&a,.6);auto wire=geometry.precompute_reference_limits();auto table=Geometry::admit_reference_limits(wire,path,a,.6);
 JointControl joint(true,true,config["mpc_tracking_v10"]);controller=&joint;joint.diagnostic_restore_actuator_state(f["state"]);Tracker tracker("dynamic_mpc");GenericAvoidance generic(a);generic.running_floor=.6;generic.route_aligned_opponent_prior=true;
 std::vector<Point> points;for(auto p:f["static_obstacle_points"])points.emplace_back(p[0].asDouble(),p[1].asDouble());
 auto prepared=generic.prepare_observations(f["occupancies"],f["prediction_now"].asDouble());
 auto corridor=[&](Pose p,double future){return generic.prediction_corridor_prepared(p,prepared,future);};
 try{auto result=joint.compute(a,path,pose,input["speed"].asDouble(),input["lateral"].asDouble(),input["yaw_rate"].asDouble(),input["remaining"].asDouble(),2.15,input["dt"].asDouble(),tracker,points,input["map_speed_ceiling_mps"].asDouble(),corridor,&table,f["braking_cap"].asDouble());
 out["speed"]=result.speed;out["steering"]=result.angle;out["valid"]=true;out["diagnostics"]=joint.diag;
 generic.check_prediction(path_read(joint.diag["predicted_path"]),f["occupancies"],f["prediction_now"].asDouble());
 out["predicted_body_guard_passed"]=true;out["nominal_coast_reserve_m"]=generic.check_emergency_map_coast(pose,std::max(input["speed"].asDouble(),result.speed),f["state"][9].asDouble());
 }catch(const std::exception& e){out["valid"]=false;out["reason"]=e.what();out["diagnostics"]=joint.diag;}
 controller=nullptr;
 }catch(const std::exception& e){out["valid"]=false;out["reason"]=e.what();}
 std::cout<<dump(out)<<std::endl;return out.get("valid",false).asBool()?0:1;
}
