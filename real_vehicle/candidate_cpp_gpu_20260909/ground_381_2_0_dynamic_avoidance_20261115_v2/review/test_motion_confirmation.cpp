#include "tracked_occupancy.hpp"
#include <iostream>
using namespace frenet;
void need(bool b,const char* s){if(!b)throw std::runtime_error(s);}
Track box(double stamp,double x,double y,double far=0){Track t;t.stamp=stamp;t.hits=5;t.cluster.points={Point(x,y-.15),Point(x+.3+far,y-.15),Point(x+.3+far,y+.15),Point(x,y+.15)};return t;}
int main(int argc,char**argv){try{
 Assets a(argv[1]);std::fill(a.grid.begin(),a.grid.end(),2.);std::vector<float> blocked(1081,.3);Pose pose{0,0,0};
 auto step=[&](DynamicMemory& m,double ts,double x,double y,double far=0){m.observe({box(ts,x,y,far)},ts,blocked,-2.35619449,4.71238898/1080,.05,10.,pose);auto v=m.snapshot();need(v.size()==1,"one_continuous_target");return v[0];};
 DynamicMemory jitter(a);Json::Value last;
 for(int k=0;k<120;++k){double ts=100+.05*k;last=step(jitter,ts,2.+.01*std::sin(k*1.7),.009*std::cos(k*.9));need(!last["has_moved"].asBool(),"bounded_jitter_invents_motion");if(k>20){need(last["fixed_world"].asBool(),"jitter_revokes_static");need(last["vx"].asDouble()==0 && last["vy"].asDouble()==0,"jitter_leaks_prediction_velocity");}}
 // Static latch must still update the measured geometry, not pin its position.
 need(std::abs(last["xmin"].asDouble()-2.)<.03,"measurement_not_updated");
 Json::Value delays(Json::arrayValue);
 for(double speed:{.15,.5,.75}){
  DynamicMemory moving(a);for(int k=0;k<25;++k)step(moving,200+k*.05,2.,0.);
  double detected=-1.;
  for(int k=0;k<32;++k){auto o=step(moving,201.25+k*.05,2.+speed*k*.05,0.);if(detected<0 && o["has_moved"].asBool()){detected=k*.05;need(!o["fixed_world"].asBool(),"moving_target_keeps_static_latch");}}
  need(detected>=0 && detected<=(speed>=.5?.45:.75),"real_motion_detection_delayed");delays.append(detected);
  for(int k=0;k<35;++k)last=step(moving,202.85+k*.05,2.+speed*31*.05,0.);
  need(!last["has_moved"].asBool() && last["fixed_world"].asBool(),"stopped_mover_never_settles");
  for(int k=0;k<20;++k)last=step(moving,204.60+k*.05,2.+speed*31*.05+.5*k*.05,0.);
  need(last["has_moved"].asBool()&&!last["fixed_world"].asBool(),"restart_hidden_by_static_latch");
 }
 DynamicMemory lateral(a);for(int k=0;k<25;++k)step(lateral,300+k*.05,2.,0.);double lateral_delay=-1;
 for(int k=0;k<25;++k){last=step(lateral,301.25+k*.05,2.,-.5*k*.05);if(lateral_delay<0 && last["has_moved"].asBool())lateral_delay=.05*k;}
 need(lateral_delay>=0 && lateral_delay<=.45,"lateral_motion_not_detected");
 Json::Value out;out["passed"]=true;out["physical_motion"]=false;out["bounded_1cm_jitter_remains_static"]=true;out["start_detection_delays_s"]=delays;out["speeds_mps"]=Json::Value(Json::arrayValue);for(double v:{.15,.5,.75})out["speeds_mps"].append(v);out["lateral_detection_delay_s"]=lateral_delay;out["moving_stop_restart_passed"]=true;std::cout<<dump(out)<<std::endl;
}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}}
