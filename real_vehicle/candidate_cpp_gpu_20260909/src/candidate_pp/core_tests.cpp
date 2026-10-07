#include "core.hpp"
#include <iostream>
#include <limits>
static void check(bool yes){if(!yes)throw std::runtime_error("core assertion failed");}
int main(int argc,char **argv){
 try{
  if(argc!=2)return 2;
  auto r=candidate::load(argv[1]);auto p=r[0];
  for(double v:{std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()}){
    candidate::Core c(r);auto d=c.step(v,p.y,p.yaw,0.);check(!d.valid&&d.speed==0.);
    try{candidate::encode(v,.5);return 3;}catch(const std::exception &){}
  }
  check(candidate::encode(.27,.27)<=.27);check(candidate::encode(-.27,.27)>=-.27);
  candidate::Freshness f;check(!f.fresh(100.,1.));check(f.accept(100.,100.,1.));
  check(!f.accept(100.,100.,1.01));check(!f.accept(99.,100.,1.01));check(!f.accept(101.,100.,1.01));
  check(f.fresh(100.1,1.1));check(!f.fresh(100.21,1.21));check(!f.fresh(100.,.9));
  candidate::Core c(r);auto d=c.step(p.x,p.y,p.yaw,0.,.01);check(d.valid&&d.speed<=.01000001);
  d=c.step(p.x,p.y,p.yaw,0.,1.);check(d.valid&&d.speed<=.06000001);
  for(auto q:r){d=c.step(q.x,q.y,q.yaw,0.);check(d.valid&&d.speed>=0.&&d.speed<=.5&&std::abs(d.angle)<=.27);}
  check(d.complete&&d.speed==0.);d=c.step(p.x,p.y,p.yaw,0.);check(d.complete&&d.speed==0.);
  std::cout<<"PASS: finite checks, float32 limits, source/receipt freshness, replay rejection, elapsed slew, bounds, endpoint latch\n";
 }catch(const std::exception &e){std::cerr<<e.what()<<"\n";return 1;}
}
