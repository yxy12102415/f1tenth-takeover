#include "speed_geometry.hpp"
#include <iostream>
using namespace frenet;
int main(int argc,char**argv){Assets a(argv[1]);auto f=json_read(argv[2]);Path p=path_read(f["path"]);auto q=f["input"]["pose"];Pose pose{q[0].asDouble(),q[1].asDouble(),q[2].asDouble()};Geometry g(p,pose,&a,.6);
#ifndef REFERENCE_BASELINE
 auto wire=g.precompute_reference_limits();auto table=Geometry::admit_reference_limits(wire,p,a,.6);Geometry cached(p,pose,&a,.6,&table);
#else
 auto& cached=g;
#endif
 double start=mono(),station=g.nearest_s;Json::Value out;
 for(int stage=0;stage<25;++stage){out["targets"].append(cached.target(station,1e6,1.85));station=std::min(g.arc.back(),station+.05*f["input"]["speed"].asDouble());}
 out["reference_speed_ms"]=(mono()-start)*1000.;std::cout<<dump(out)<<std::endl;
}
