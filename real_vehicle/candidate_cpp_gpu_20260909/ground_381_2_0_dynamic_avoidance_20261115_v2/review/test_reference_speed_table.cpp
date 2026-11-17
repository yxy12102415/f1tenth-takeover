#include "speed_geometry.hpp"
#include <iostream>
using namespace frenet;
int main(int argc,char**argv){
 Assets a(argv[1]);auto fixture=json_read(argv[2]);Path path=path_read(fixture["path"]);
 Pose pose;
 // The fixture pose is a single [x,y,yaw], rather than a Path.
 pose={fixture["input"]["pose"][0].asDouble(),fixture["input"]["pose"][1].asDouble(),fixture["input"]["pose"][2].asDouble()};
 Geometry original(path,pose,&a,.6);double begin=mono();auto wire=original.precompute_reference_limits();double prep=(mono()-begin)*1000;
 begin=mono();auto table=Geometry::admit_reference_limits(wire,path,a,.6);double admission=(mono()-begin)*1000;
 Geometry cached(path,pose,&a,.6,&table);int comparisons=0,rejections=0;double lookup_ms=0.;
 for(double cap:{.7,1.,1.85,2.})for(int j=0;j<=100;++j){
  double s=original.arc.back()*j/100.;double expected=0,actual=0;std::string err1,err2;
  try{expected=original.target(s,1e6,cap);}catch(const std::exception&e){err1=e.what();}
  double start=mono();try{actual=cached.target(s,1e6,cap);}catch(const std::exception&e){err2=e.what();}lookup_ms+=(mono()-start)*1000;
  if(expected!=actual||err1!=err2)throw std::runtime_error("target_parity_failure");++comparisons;
 }
 auto reject=[&](Json::Value bad,Path p,const Assets& map,double floor){try{Geometry::admit_reference_limits(bad,p,map,floor);}catch(...){++rejections;return;}throw std::runtime_error("invalid_table_admitted");};
 auto bad=wire;bad["policy_version"]=2;reject(bad,path,a,.6);
 bad=wire;bad["map_version"]="different";reject(bad,path,a,.6);
 bad=wire;bad["limits"][0]=2.1;reject(bad,path,a,.6);
 bad=wire;bad["limits"]=Json::Value(Json::arrayValue);reject(bad,path,a,.6);
 auto different=path;different[0].x+=.001;reject(wire,different,a,.6);
 reject(wire,path,a,.7);auto map=a;map.wall_margin+=.001;reject(wire,path,map,.6);
 Json::Value out;out["passed"]=true;out["target_parity_queries"]=comparisons;out["mismatch_rejections"]=rejections;out["preparation_ms"]=prep;out["admission_ms"]=admission;out["cached_query_mean_ms"]=lookup_ms/comparisons;out["points"]=int(path.size());out["physical_motion"]=false;std::cout<<dump(out)<<std::endl;
}
