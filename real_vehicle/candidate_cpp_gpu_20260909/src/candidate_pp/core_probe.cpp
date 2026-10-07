#include "core.hpp"
#include <iostream>
#include <iomanip>
int main(int argc,char **argv) {
  try {
    if(argc!=2) throw std::runtime_error("route argument required");
    candidate::Core c(candidate::load(argv[1]));double x,y,yaw,v,dt;
    while(std::cin>>x>>y>>yaw>>v>>dt){auto d=c.step(x,y,yaw,v,dt);
      std::cout<<std::setprecision(17)<<d.speed<<","<<d.angle<<","<<d.valid<<","<<d.complete<<","<<c.index<<","<<d.reason<<"\n";}
  }catch(const std::exception &e){std::cerr<<e.what()<<"\n";return 2;}
}
