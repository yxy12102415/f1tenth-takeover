#include "compact_overtake.hpp"
#include <iostream>
#include <sstream>
using namespace frenet;
int main(int argc,char**argv) {
  if(argc!=3)return 2;
  Assets assets(argv[1]);CompactOvertake compact(assets,argv[2]);Perception perception(assets);
  for(std::string line;std::getline(std::cin,line);) {
    Json::Value result;
    try {
      std::istringstream in(line);Json::Value q;in>>q;
      if(q["op"]=="scan") {
        std::vector<float> ranges;
        for(auto r:q["ranges"])ranges.push_back(r.isNull()?float(INF):r.asFloat());
        Pose pose=path_read(q["pose"])[0];double stamp=q["stamp"].asDouble();
        perception.scan(ranges,q["angle_min"].asDouble(),q["angle_increment"].asDouble(),.05,10.,stamp,pose);
        result["perception"]=perception.diagnostic();
        if(q.get("debug_bounds",false).asBool()) {
          result["observed_bounds"]=Json::Value(Json::arrayValue);
          for(const auto& tr:perception.tracks)if(tr.confirmed && !tr.misses) {
            double xmin=INF,xmax=-INF,ymin=INF,ymax=-INF;
            for(auto pt:tr.cluster.points){auto v=compact.local({pt.x(),pt.y(),0});
              xmin=std::min(xmin,v.x);xmax=std::max(xmax,v.x);
              ymin=std::min(ymin,v.y);ymax=std::max(ymax,v.y);}
            Json::Value row;row["id"]=tr.id;row["xmin"]=xmin;row["xmax"]=xmax;
            row["ymin"]=ymin;row["ymax"]=ymax;result["observed_bounds"].append(row);
          }
        }
        result["compact"]=compact.update(perception.tracks,stamp,pose,false);
      } else if(q["op"]=="check") {
        compact.check_prediction(path_read(q["prediction"]),LeadBounds::read(q["lead"]),q["now"].asDouble());
      } else {
        std::vector<Track> tracks;
        for(auto row:q["tracks"]) {
          Track t;t.confirmed=true;t.hits=5;t.id=row["id"].asInt();t.stamp=q["stamp"].asDouble();
          for(auto p:row["points"])t.cluster.points.emplace_back(p[0].asDouble(),p[1].asDouble());
          tracks.push_back(t);
        }
        result=compact.update(tracks,q["stamp"].asDouble(),path_read(q["pose"])[0],q.get("enable",false).asBool(),q.get("stationary",false).asBool(),q.get("terminal_zero_latched",false).asBool());
      }
      result["ok"]=true;
    }catch(const std::exception&e){result["ok"]=false;result["error"]=e.what();}
    std::cout<<dump(result)<<std::endl;
  }
}
