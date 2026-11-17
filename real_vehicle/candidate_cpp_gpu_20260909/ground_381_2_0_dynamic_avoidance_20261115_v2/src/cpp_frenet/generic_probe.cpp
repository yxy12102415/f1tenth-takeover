#include "generic_avoidance.hpp"
#include <iostream>
using namespace frenet;
namespace frenet {struct GenericProbeAccess {
  static void restore(GenericAvoidance& g,const Json::Value& input) {
    g.incumbent=path_read(input["initial_incumbent"]);
    g.incumbent_offset=input.get("initial_offset",0.).asDouble();
  }
  static Json::Value forecast(GenericAvoidance& g,const Json::Value& input) {
    auto o=g.read(input["forecast_occupancy"]);Point lo,hi;double station=0;Pose p;
    double now=input["stamp"].asDouble(),future=input.get("future_s",1.2).asDouble();
    bool eligible=g.route_prior_projection(o,now,station,p);
    g.bounds(o,now,future,lo,hi);Json::Value out;
    out["route_prior_applied"]=eligible;out["valid"]=true;
    for(double x:{lo.x(),lo.y(),hi.x(),hi.y()})out["box"].append(x);
    return out;
  }
};}
int main(int argc,char**argv) {
  try {
    Assets a(argv[1]);GenericAvoidance generic(a);Planner planner(a);Perception perception(a,true,true);
    std::string asset_file(argv[1]);
    const auto config=json_read(asset_file.substr(0,asset_file.find_last_of('/')+1)+"compact_config.json");
    Json::Value input;
    std::string line;
    while(std::getline(std::cin,line)) {
      Json::Reader reader;if(!reader.parse(line,input))return 2;
      generic.running_floor=input.get("running_floor",config.get("generic_running_floor_mps",.5).asDouble()).asDouble();
      generic.stationary_scene=input.get("stationary_scene",false).asBool();
      generic.route_aligned_opponent_prior=input.get("route_prior",config.get("route_aligned_opponent_prior_v1",false).asBool()).asBool();
      generic.map_speed_feasibility=input.get("map_speed_feasibility",config["mpc_tracking_v10"].get("map_speed_feasibility",false).asBool()).asBool();
      generic.seed_reserve=input.get("seed_reserve",config.get("generic_corridor_seed_reserve_m",.04).asDouble()).asDouble();
      generic.preferred_gap=input.get("preferred_gap",config["adaptive_corridor_v2"].get("desired_body_gap_m",.14).asDouble()).asDouble();
      if(!std::isfinite(generic.preferred_gap) || generic.preferred_gap<.08 || generic.preferred_gap>.20)throw std::runtime_error("generic_preferred_body_gap");
      generic.allow_envelope_seeds=!input.get("qp_only",false).asBool();
      generic.cap=input.get("cap",1.4).asDouble();generic.max_offset=input.get("max_offset",.75).asDouble();
      if(input.get("refresh_prearm_path",false).asBool())generic.refresh_prearm_path();
      if(input["initial_incumbent"].isArray()) {
        GenericProbeAccess::restore(generic,input);
      }
      Json::Value result;
      try {
        double stamp=input["stamp"].asDouble();Pose pose=path_read(input["poses"])[0];
        if(input["forecast_occupancy"].isObject()) {
          result=GenericProbeAccess::forecast(generic,input);
        } else if(input["map_limit_path"].isArray()) {
          Geometry geometry(path_read(input["map_limit_path"]),pose,&a);
          for(size_t k=0;k<geometry.path.size();++k)result["limits"].append(geometry.map_speed_limit(k));
          result["valid"]=true;
        } else if(input.get("state_coast_cap",false).asBool()) {
          result["cap"]=generic.state_coast_speed_cap(pose,input.get("servo",0.).asDouble(),input.get("upper",2.).asDouble());result["valid"]=true;
        } else if(input["arrival_path"].isArray()) {
          auto values=generic.arrival_times(path_read(input["arrival_path"]),pose,input.get("speed",0.).asDouble(),input.get("arrival_horizon",1e9).asDouble());
          for(double t:values)result["times"].append(t);result["valid"]=true;
        } else if(input.get("coast_map",false).asBool()) {
          double speed=input.get("speed",0.).asDouble(),servo=input.get("servo",0.).asDouble();
          result["minimum_map_reserve_m"]=generic.check_emergency_map_coast(pose,speed,servo);
          result["path"]=path_json(emergency_coast_path(pose,speed,servo));result["valid"]=true;
        } else if(input["ego_mask_points"].isArray()) {
          result["masked"]=Json::Value(Json::arrayValue);
          for(auto p:input["ego_mask_points"])result["masked"].append(Perception::in_ego_interior(p[0].asDouble(),p[1].asDouble()));
          result["valid"]=true;
        } else if(input["check_prediction"].isArray()) {
          generic.check_prediction(path_read(input["check_prediction"]),input["occupancies"],stamp);
          result["valid"]=true;
        } else if(input["plan_occupancies"].isArray()) {
          auto c=generic.plan_occupancies(planner,pose,stamp,input.get("speed",0.).asDouble(),input["plan_occupancies"],generic.preferred_gap,input.get("allow_search",true).asBool(),mono()+input.get("search_budget_ms",200.).asDouble()/1000.);
          result=candidate_json(c);
          Json::Value reasons;for(const auto& v:planner.candidates)reasons[v.reason]=reasons.get(v.reason,0).asInt()+1;
          result["candidate_reasons"]=reasons;
          if(input.get("include_candidates",false).asBool())for(const auto& v:planner.candidates)result["candidates"].append(candidate_json(v));
        } else if(input["ranges"].isArray()) {
          std::vector<float> ranges;
          for(auto v:input["ranges"])ranges.push_back(v.isNull()?std::numeric_limits<float>::infinity():v.asFloat());
          double amin=input["angle_min"].asDouble(),inc=input["angle_increment"].asDouble();
          double rmin=input["range_min"].asDouble(),rmax=input["range_max"].asDouble();
          auto points=perception.scan(ranges,amin,inc,rmin,rmax,stamp,pose);
          generic.observe(perception.tracks,stamp,ranges,amin,inc,rmin,rmax,pose);
          auto c=generic.plan(planner,pose,stamp,input.get("speed",0.).asDouble(),input.get("allow_search",true).asBool());
          result=candidate_json(c);result["occupancies"]=generic.snapshot();
          result["perception"]=perception.diagnostic();
        } else {
          std::vector<Track> tracks;
          for(auto v:input["clusters"]) {
            Track t;t.id=v["id"].asInt();t.stamp=stamp;t.hits=v.get("hits",5).asInt();t.confirmed=true;
            t.cluster.center=Point::Zero();
            for(auto p:v["points"]) {Point q(p[0].asDouble(),p[1].asDouble());t.cluster.points.push_back(q);t.cluster.center+=q;}
            t.cluster.center/=double(t.cluster.points.size());tracks.push_back(t);
          }
          std::vector<float> ranges(1081,input.get("free_range",100.).asDouble());
          generic.observe(tracks,stamp,ranges,-2.35619449,4.71238898/1080,.05,10.,pose);
          auto c=generic.plan(planner,pose,stamp,input.get("speed",0.).asDouble(),input.get("allow_search",true).asBool());
          result=candidate_json(c);result["occupancies"]=generic.snapshot();
        }
      }catch(const std::exception&e){result["valid"]=false;result["reason"]=e.what();}
      std::cout<<dump(result)<<std::endl;
    }
  } catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}
}
