#include "tracked_occupancy.hpp"
#include <iostream>
using namespace frenet;
Track make_track(double t,double x,double y,int id=1){Track tr;tr.stamp=t;tr.id=id;tr.hits=5;tr.misses=0;tr.cluster.points={Point(x-.10,y-.15),Point(x+.10,y-.15),Point(x+.10,y+.15),Point(x-.10,y+.15)};return tr;}
void require(bool v,const char*why){if(!v)throw std::runtime_error(why);}
int main(int argc,char**argv){try{Assets a(argv[1]);Pose p{0,0,0};std::vector<float> blocked(1081,.3),free(1081,9.);
  DynamicMemory still(a);for(int n=0;n<1800;n++){double t=100+n*.05;auto tr=make_track(t,1.5+.006*std::sin(n),.006*std::cos(n),n%9);still.observe({tr},t,blocked,-2.35619449,4.71238898/1080,.05,10,p);}
  auto s=still.snapshot();require(s.size()==1,"stationary_duplicate_tracks");require(!s[0]["has_moved"].asBool(),"stationary_false_motion");require(s[0]["xmax"].asDouble()-s[0]["xmin"].asDouble()<.25,"stationary_shape_growth");
  DynamicMemory moving(a);for(int n=0;n<40;n++){double t=100+n*.05;moving.observe({make_track(t,1.+.5*(t-100),0,n%7)},t,blocked,-2.35619449,4.71238898/1080,.05,10,p);}
  s=moving.snapshot();require(s.size()==1,"moving_duplicate_tracks");require(s[0]["has_moved"].asBool(),"moving_not_classified");require(std::abs(s[0]["vx"].asDouble()-.5)<.10,"velocity_error");double stamp=s[0]["stamp"].asDouble();
  for(int n=1;n<=35;n++)moving.observe({},101.95+n*.05,blocked,-2.35619449,4.71238898/1080,.05,10,p);
  s=moving.snapshot();require(s.size()==1 && s[0]["visibility_hazard"].asBool(),"occlusion_forgot_obstacle");require(s[0]["stamp"].asDouble()==stamp,"occlusion_renewed_source");double xmin=s[0]["xmin"].asDouble();
  for(int n=36;n<=200;n++)moving.observe({},101.95+n*.05,blocked,-2.35619449,4.71238898/1080,.05,10,p);
  s=moving.snapshot();require(s.size()==1 && s[0]["xmin"].asDouble()==xmin,"occlusion_unbounded_growth");
  for(int n=201;n<=205;n++)moving.observe({},101.95+n*.05,free,-2.35619449,4.71238898/1080,.05,10,p);
  require(moving.snapshot().empty(),"ray_clearing_failed");
  for(double speed:{.2,.65}){
    DynamicMemory m(a);
    for(int n=0;n<40;n++){double tm=100+n*.05;m.observe({make_track(tm,1.+speed*(tm-100),0,n%11)},tm,blocked,-2.35619449,4.71238898/1080,.05,10,p);}
    auto z=m.snapshot();require(z.size()==1 && z[0]["has_moved"].asBool(),"motion_range_tracking");require(std::abs(z[0]["vx"].asDouble()-speed)<.10,"motion_range_velocity");
  }
  DynamicMemory wheels(a);
  for(int n=0;n<30;n++){
    double tm=100+n*.05;std::vector<Track> ts;
    for(int k:{0,3,1,2}){double x=1.5+.5*(tm-100)+(k&1?.1:-.1),y=k&2?.12:-.12;Track tr=make_track(tm,x,y,n*4+k);tr.cluster.points={Point(x-.04,y-.04),Point(x+.04,y+.04)};ts.push_back(tr);}
    wheels.observe(ts,tm,blocked,-2.35619449,4.71238898/1080,.05,10,p);
  }
  auto w=wheels.snapshot();require(w.size()==1 && w[0]["has_moved"].asBool(),"wheel_components_order");
  Path hook{{0,0,0},{.05,0,0},{.05,.05,0}};require(curvature(hook)>20,"short_path_curvature_bypass");
  Json::Value out;out["passed"]=true;out["stationary_90s"]=true;out["id_changes_0_5mps"]=true;out["occlusion_source_not_renewed"]=true;out["bounded_unknown"]=true;out["ray_clear"]=true;out["terminal_dense_curvature"]=true;out["motion_range_0_2_to_0_65"]=true;out["wheel_fragment_order"]=true;std::cout<<dump(out)<<std::endl;
}catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}}
