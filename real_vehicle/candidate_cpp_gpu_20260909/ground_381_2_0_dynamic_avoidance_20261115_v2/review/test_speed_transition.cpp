#include "speed_transition.hpp"
#include <iostream>
#include <cassert>
using namespace frenet;
int main(){
 auto check=[](double v,double a,double target,double floor,double dt){auto r=reachable_speed_transition(v,a,target,2.,floor,dt);double current=v,old=a;
 for(unsigned i=0;i<24;++i){double change=i?.15:4*dt;assert(r.acceleration[i]>=-.8000001 && r.acceleration[i]<=1.0000001);assert(std::abs(r.acceleration[i]-old)<=change+1e-7);current+=.05*r.acceleration[i];assert(current>=floor-1e-8 && current<=r.upper[i+1]+1e-8 && r.upper[i+1]<=2.+1e-9);old=r.acceleration[i];}
 return r;};
 auto field=check(1.1065302685761107,.04242902877010183,.99375,.6,.05);
 assert(field.active && field.recovery_stages==4);assert(std::abs(field.upper[1]-1.0986527200146158)<1e-9);assert(field.upper[5]==.99375);
 unsigned cases=0;
 for(double v:{.6,.7,1.1,1.5})for(double a:{0.,.4,.8})for(double target:{.6,.8,1.2,2.})for(double dt:{.04,.05}){check(v,a,target,.6,dt);++cases;}
 auto floor=check(.61,0.,.6,.6,.05);assert(floor.upper[24]>=.6);
 auto steady=check(.9,0.,1.2,.6,.05);assert(!steady.active);for(unsigned i=1;i<=24;++i)assert(steady.upper[i]==1.2);
 bool refused=false;try{reachable_speed_transition(.6,-.8,.6,2.,.6,.05);}catch(const std::runtime_error&){refused=true;}assert(refused);
 refused=false;try{reachable_speed_transition(1.99,1.,1.,2.,.6,.05);}catch(const std::runtime_error&){refused=true;}assert(refused);
 std::cout<<"{\"passed\":true,\"physical_motion\":false,\"grid_cases\":"<<cases<<",\"recorded_drop_feasible\":true,\"floor_viability_kept\":true,\"absolute_max_kept\":true,\"unrecoverable_states_rejected\":true}\n";
}
