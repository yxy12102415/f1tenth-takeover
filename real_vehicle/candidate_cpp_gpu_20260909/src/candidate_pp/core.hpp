#pragma once
// Original implementation of open-route PP. Callback architecture inspired by
// CL2-UWaterloo/f1tenth_ws; geometry and limits follow this vehicle's Python core.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
namespace candidate {
inline double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
struct Point { double x,y,yaw,s=0.; };
struct Decision { double speed=0., angle=0.; bool valid=false, complete=false; std::string reason; };
inline std::vector<Point> load(const std::string &path) {
  std::ifstream f(path); std::string line;
  if (!std::getline(f,line) || line!="x,y,yaw") throw std::runtime_error("route schema must be x,y,yaw, NOT x,y,velocity");
  std::vector<Point> r;
  while(std::getline(f,line)) {
    std::replace(line.begin(),line.end(),',',' '); std::istringstream ss(line); Point p; std::string extra;
    if(!(ss>>p.x>>p.y>>p.yaw) || (ss>>extra) || !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw)) throw std::runtime_error("invalid route row");
    if(!r.empty()) {
      double d=std::hypot(p.x-r.back().x,p.y-r.back().y);
      if(d<=1.e-4) throw std::runtime_error("duplicate route point");
      p.s=r.back().s+d;
    }
    r.push_back(p);
  }
  if(r.size()<4) throw std::runtime_error("route too short");
  // Same nonuniform central derivative as numpy.gradient(unwrapped_yaw,s).
  std::vector<double> yaw(r.size()); yaw[0]=r[0].yaw;
  for(size_t i=1;i<r.size();++i) yaw[i]=yaw[i-1]+wrap(r[i].yaw-r[i-1].yaw);
  for(size_t i=0;i<r.size();++i) {
    double k;
    if(i==0) k=(yaw[1]-yaw[0])/(r[1].s-r[0].s);
    else if(i+1==r.size()) k=(yaw[i]-yaw[i-1])/(r[i].s-r[i-1].s);
    else { double a=r[i].s-r[i-1].s,b=r[i+1].s-r[i].s;
      k=-b/(a*(a+b))*yaw[i-1]+(b-a)/(a*b)*yaw[i]+a/(b*(a+b))*yaw[i+1]; }
    if(std::abs(k)>std::tan(.27)/.25+.03) throw std::runtime_error("route curvature exceeds authority");
  }
  return r;
}
class Core {
 public:
  std::vector<Point> route; size_t index=0; bool started=false,complete=false; double last_speed=0.;
  explicit Core(std::vector<Point> r):route(std::move(r)) {}
  Decision invalid(const std::string &why) {last_speed=0.; Decision d;d.reason=why;return d;}
  Decision step(double x,double y,double yaw,double measured,double dt=.05) {
    if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw)||!std::isfinite(measured)||!std::isfinite(dt)||dt<=0.) return invalid("non_finite_state");
    if(complete) {Decision d;d.valid=d.complete=true;d.reason="route_complete";return d;}
    if(!started) {
      if(std::hypot(x-route[0].x,y-route[0].y)>.30) return invalid("outside_start_distance");
      if(std::abs(wrap(yaw-route[0].yaw))>.45) return invalid("outside_start_heading");
      started=true;
    }
    size_t nearest=index; double dist=INFINITY;
    for(size_t i=index>2?index-2:0;i<std::min(route.size(),index+36);++i) {
      double d=std::hypot(x-route[i].x,y-route[i].y); if(d<dist){dist=d;nearest=i;}
    }
    index=std::max(index,nearest); const auto &p=route[index];
    if(std::abs(-std::sin(p.yaw)*(x-p.x)+std::cos(p.yaw)*(y-p.y))>.25) return invalid("cross_track_error");
    if(std::abs(wrap(yaw-p.yaw))>.70) return invalid("heading_error");
    double remaining=route.back().s-p.s;
    if(remaining<=.16) {complete=true;index=route.size()-1;last_speed=0.;Decision d;d.valid=d.complete=true;d.reason="route_complete";return d;}
    double target_s=std::min(route.back().s,p.s+.4);
    auto hi=std::lower_bound(route.begin(),route.end(),target_s,[](const Point &a,double s){return a.s<s;});
    auto lo=hi-1; double t=(target_s-lo->s)/(hi->s-lo->s);
    double dx=lo->x+t*(hi->x-lo->x)-x,dy=lo->y+t*(hi->y-lo->y)-y;
    double lx=std::cos(yaw)*dx+std::sin(yaw)*dy,ly=-std::sin(yaw)*dx+std::cos(yaw)*dy;
    if(lx<=0.||dx*dx+dy*dy<1.e-8) return invalid("mpc_infeasible");
    Decision d;d.valid=true;
    d.angle=std::max(-.27,std::min(.27,std::atan2(.5*ly,dx*dx+dy*dy)));
    double target=.5*std::sqrt(std::min(1.,std::max(0.,(remaining-.16)/.80)));
    // Cap elapsed time: a delayed callback never causes a large speed jump.
    double delta=std::min(.05,dt);
    d.speed=std::max(0.,std::min(.5,last_speed+std::max(-delta,std::min(delta,target-last_speed))));
    last_speed=d.speed;return d;
  }
};
struct Freshness {
  double stamp=-1.,receipt=-1.;
  bool accept(double s,double ros,double mono) {
    if(!std::isfinite(s)||s<=0.||s<=stamp||ros-s<-.02||ros-s>.2) return false;
    stamp=s;receipt=mono;return true;
  }
  bool fresh(double ros,double mono) const {return stamp>0.&&mono>=receipt&&mono-receipt<=.2&&ros-stamp>=-.02&&ros-stamp<=.2;}
};
inline float encode(double x,double limit) {
  if(!std::isfinite(x)||std::abs(x)>limit) throw std::runtime_error("invalid command");
  float out=static_cast<float>(x);if(std::abs(static_cast<double>(out))>limit)out=std::nextafter(out,0.f);return out;
}
}
