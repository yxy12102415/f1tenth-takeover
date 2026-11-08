#include "src/ground_pp/core.hpp"
#include <iostream>
int main(int argc,char**argv){
  if(argc!=2)return 2;
  auto route=candidate::load(argv[1]);
  if(route.front().s!=0.||route.back().s<=0.)return 1;
  // Regression for a real car held at the start by static friction: commands
  // must keep climbing at 1 m/s^2 even before localization reports movement.
  candidate::Core stationary(route);
  double previous=0.;
  for(int i=0;i<10;++i){
    auto d=stationary.step(route[0].x,route[0].y,route[0].yaw,0.,.02);
    if(!d.valid||d.complete||d.speed<=previous||d.speed-previous>.0200001)return 1;
    previous=d.speed;
  }
  if(previous<.199999||previous>.200001)return 1;
  candidate::Core c(route);
  double x=route[0].x,y=route[0].y,yaw=route[0].yaw,v=0.,max_error=0.,max_angle=0.,max_speed=0.;
  bool done=false;int i=0;
  for(;i<1200;++i){
    auto d=c.step(x,y,yaw,v,.025);
    if(!d.valid){std::cerr<<d.reason<<" at "<<i<<"\n";return 1;}
    if(d.speed<0.||d.speed>1.75||std::abs(d.angle)>.27)return 1;
    max_speed=std::max(max_speed,d.speed);
    auto p=route[c.index];max_error=std::max(max_error,std::abs(-std::sin(p.yaw)*(x-p.x)+std::cos(p.yaw)*(y-p.y)));
    max_angle=std::max(max_angle,std::abs(d.angle));
    if(d.complete){done=true;break;}
    v+=.025/.15*(d.speed-v);
    x+=.025*v*std::cos(yaw);y+=.025*v*std::sin(yaw);yaw+=.025*v/.25*std::tan(d.angle);
  }
  if(!done||max_error>.1||max_angle<.01||max_speed<=.5)return 1;
  auto stop=c.step(route[0].x,route[0].y,route[0].yaw,0.,.025);
  if(!stop.complete||stop.speed!=0.||stop.angle!=0.||c.index!=route.size()-1)return 1;
  std::cout<<"{\"passed\":true,\"simulation_seconds\":"<<i*.025<<",\"max_cte_m\":"<<max_error<<",\"max_steering_rad\":"<<max_angle<<",\"max_speed_mps\":"<<max_speed<<",\"endpoint_latched\":true}\n";
}
