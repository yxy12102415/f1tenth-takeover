#include "../src/cpp_frenet/generic_avoidance.hpp"
#include <iostream>
using namespace frenet;
int main(int argc,char**argv) {
  try {
    Assets a(argv[1]);GenericAvoidance g(a);g.route_aligned_opponent_prior=true;
    auto f=json_read(argv[2]);auto collision=f["rejected_control"]["collision"];
    Json::Value poses(Json::arrayValue);poses.append(collision["ego_pose"]);
    Pose ref=path_read(poses)[0];
    const auto occupancy=f["control_input_snapshot"]["collision_guard_observation"]["occupancies"];
    double now=collision["now"].asDouble(),future=collision["future_s"].asDouble();
    bool rejected=false;try{g.pose_check(ref,occupancy,now,future,.08);}catch(...){rejected=true;}
    if(!rejected)throw std::runtime_error("fixture no longer overlaps");
    auto tube=g.prediction_corridor(ref,occupancy,now,future);double shift=(tube.first.xy()-ref.xy()).norm();
    if(shift<=0 || shift>.080001 || tube.second<.018)throw std::runtime_error("invalid free tube");
    Point n(-std::sin(ref.yaw),std::cos(ref.yaw));
    for(int side:{-1,0,1}){Pose p=tube.first;p.x+=side*tube.second*n.x();p.y+=side*tube.second*n.y();g.pose_check(p,occupancy,now,future,.083);}
    Json::Value out;out["passed"]=true;out["physical_motion"]=false;out["original_reference_rejected"]=true;out["shift_m"]=shift;out["radius_m"]=tube.second;out["tube_centre_and_edges_pass_exact_body_guard"]=true;std::cout<<dump(out)<<std::endl;
  }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}
}
