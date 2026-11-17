#pragma once
#include "core.hpp"
#include <map>
#include <deque>
namespace frenet {
// A table belongs to one exact path and immutable numerical map. It contains
// reference-only limits; current-state coast and obstacle guards stay live.
struct ReferenceSpeedTable {
  std::vector<double> limits;
  Json::Value certificate;
};
struct Geometry {
  Path path;
  std::vector<double> arc, yaw, curvature;
  const Assets* map_assets=nullptr;
  double nearest_s=0, cross=0, heading=0;
  double running_floor=.5;
  const ReferenceSpeedTable* reference_table=nullptr;
  Geometry(const Path &p, Pose pose,const Assets* map=nullptr,double floor=.5,const ReferenceSpeedTable* table=nullptr):path(p),map_assets(map),running_floor(floor),reference_table(table) {
    if(p.size()<2) throw std::runtime_error("tracker_path_invalid");
    arc.push_back(0); yaw.push_back(p[0].yaw);
    double best=INF;
    for(size_t i=1;i<p.size();++i) {
      double length=(p[i].xy()-p[i-1].xy()).norm();
      if(length<1e-6) throw std::runtime_error("tracker_path_duplicate");
      arc.push_back(arc.back()+length);
      yaw.push_back(yaw.back()+wrap(p[i].yaw-p[i-1].yaw));
      Point edge=p[i].xy()-p[i-1].xy();
      double t=clip((pose.xy()-p[i-1].xy()).dot(edge)/edge.squaredNorm(),0,1);
      Point d=pose.xy()-(p[i-1].xy()+t*edge);
      if(d.squaredNorm()<best) {
        best=d.squaredNorm();nearest_s=arc[i-1]+t*length;
        double h=yaw[i-1]+t*(yaw[i]-yaw[i-1]);
        cross=-std::sin(h)*d.x()+std::cos(h)*d.y();heading=wrap(pose.yaw-h);
      }
    }
    for(size_t i=0;i<p.size();++i) {
      size_t lo=i?i-1:i, hi=std::min(i+1,p.size()-1);
      curvature.push_back((yaw[hi]-yaw[lo])/(arc[hi]-arc[lo]));
    }
    if(std::abs(cross)>.25) throw std::runtime_error("tracking_cross_track_error");
    if(std::abs(heading)>.7) throw std::runtime_error("tracking_heading_error");
  }
  Pose at(double s) const {
    s=clip(s,0,arc.back());
    size_t hi=std::min(path.size()-1,size_t(std::upper_bound(arc.begin(),arc.end(),s)-arc.begin()));
    size_t lo=hi?hi-1:0;
    double f=hi==lo?0:(s-arc[lo])/(arc[hi]-arc[lo]);
    return {path[lo].x+f*(path[hi].x-path[lo].x),path[lo].y+f*(path[hi].y-path[lo].y),yaw[lo]+f*(yaw[hi]-yaw[lo])};
  }
  double map_speed_limit(size_t index) const {
    if(reference_table) {
      const double limit=reference_table->limits.at(index);
      if(limit==0.)throw std::runtime_error("map_coast_below_drivable_floor");
      return limit;
    }
    // Assets are immutable for the life of a runtime. Limits depend on this
    // exact pose and steering, not the path's length or passed prefix. Keep
    // them when a held path is trimmed; never quantize collision geometry.
    using Key=std::array<double,7>;
    struct Cache { const Assets* assets=nullptr;std::map<Key,double> limits;std::deque<Key> order; };
    static thread_local Cache cache;
    if(cache.assets!=map_assets){cache.assets=map_assets;cache.limits.clear();cache.order.clear();}
    const double steer=clip(std::atan(.25*curvature[index]),-.27,.27);
    // During steering recovery the actual servo retains recent curvature.
    // Anticipate that lag in the reference speed instead of waiting for the
    // independent current-state coast guard to stop an already fast vehicle.
    double steer_low=steer,steer_high=steer;
    for(size_t j=index+1;j-->0;) {
      if(arc[index]-arc[j]>.75)break;
      double prior=clip(std::atan(.25*curvature[j]),-.27,.27);
      steer_low=std::min(steer_low,prior);steer_high=std::max(steer_high,prior);
    }
    const Key key{{path[index].x,path[index].y,path[index].yaw,steer,steer_low,steer_high,running_floor}};
    const auto known=cache.limits.find(key);
    if(known!=cache.limits.end())return known->second;
    auto feasible=[&](double velocity) {
      // Circumscribed body circle + map-cell, pose and between-step cover.
      // This inexpensive envelope is conservative; the independent exact
      // oriented-body coast check still verifies every emitted command.
      // Partition the 0.58 m body into three 0.1933 m rectangles.
      // A circle at each partition center covers its entire rectangle. This
      // remains conservative without imposing the diagonal as side width.
      const double segment=.58/3.;
      const double radius=std::hypot(segment/2.,.155)+map_assets->wall_margin+map_assets->res/std::sqrt(2.)+.02+velocity*.025;
      for(double dh:{-.05,0.,.05}) {
        Pose q=path[index];q.yaw+=dh;
        for(double lagged_steer:{steer_low,steer,steer_high})
          for(auto point:emergency_coast_path(q,velocity,lagged_steer)){
            Point tangent(std::cos(point.yaw),std::sin(point.yaw));
            for(double shift:{-segment,0.,segment})
              if(map_assets->clearance(point.xy()+shift*tangent)<radius)return false;
          }
      }
      return true;
    };
    if(!feasible(running_floor))throw std::runtime_error("map_coast_below_drivable_floor");
    double lo=running_floor,hi=2.;
    for(int i=0;i<6;++i){double middle=(lo+hi)/2;if(feasible(middle))lo=middle;else hi=middle;}
    if(cache.order.size()>=16384){cache.limits.erase(cache.order.front());cache.order.pop_front();}
    cache.limits[key]=lo;cache.order.push_back(key);return lo;
  }
  Json::Value precompute_reference_limits() const {
    Json::Value result;
    result["policy_version"]=1;result["map_version"]=map_assets->map_version;
    result["floor_mps"]=running_floor;result["wall_margin_m"]=map_assets->wall_margin;
    result["path"]=path_json(path);result["limits"]=Json::Value(Json::arrayValue);
    for(size_t i=0;i<path.size();++i) {
      // The existing target() never consults the final 40 cm. Keep its exact
      // terminal behavior; do not add cruising feasibility at the endpoint.
      double limit=2.;
      if(arc.back()-arc[i]>=.40)try{limit=map_speed_limit(i);}
      catch(const std::runtime_error& e){
        if(std::string(e.what())!="map_coast_below_drivable_floor")throw;
        limit=0.; // Preserve rejection if a future query reaches this point.
      }
      result["limits"].append(limit);
    }
    return result;
  }
  static ReferenceSpeedTable admit_reference_limits(const Json::Value& table,const Path& path,const Assets& a,double floor) {
    auto reject=[](){throw std::runtime_error("reference_speed_table_mismatch");};
    if(table["policy_version"]!=1 || table["map_version"]!=a.map_version ||
       !table["floor_mps"].isNumeric() || table["floor_mps"].asDouble()!=floor ||
       !table["wall_margin_m"].isNumeric() || table["wall_margin_m"].asDouble()!=a.wall_margin ||
       !table["path"].isArray() || !table["limits"].isArray() ||
       table["path"].size()!=path.size() || table["limits"].size()!=path.size())reject();
    Path original=path_read(table["path"]);ReferenceSpeedTable result;
    for(size_t i=0;i<path.size();++i) {
      if(original[i].x!=path[i].x || original[i].y!=path[i].y || original[i].yaw!=path[i].yaw || !table["limits"][int(i)].isNumeric())reject();
      double v=table["limits"][int(i)].asDouble();
      if(!std::isfinite(v) || (v!=0. && (v<floor || v>2.)))reject();
      result.limits.push_back(v);
    }
    result.certificate=table;return result;
  }
  double target(double s,double remaining,double cap) const {
    double k=0;
    for(size_t i=0;i<arc.size();++i)
      if(arc[i]>=s-.05 && arc[i]<=s+.75) k=std::max(k,std::abs(curvature[i]));
    // Curvature/terminal feasibility is explicit; no fixed AVOIDING speed dip.
    double result=std::min({cap,std::sqrt(1.2/std::max(.01,k)),std::sqrt(std::max(0.,remaining)*.8)});
    if(map_assets) {
      double last=-INF;
      for(size_t i=0;i<arc.size();++i) {
        // Terminal floor-to-zero coast is handled by the separate latched
        // terminal policy; it must not require cruising at the endpoint.
        if(arc.back()-arc[i]<.40)continue;
        if(arc[i]<s-.05 || arc[i]>s+cap*cap/(2*.55)+.3*cap+.3)continue;
        if(arc[i]-last<.20 && i+1<arc.size())continue;
        last=arc[i];double safe=map_speed_limit(i),distance=std::max(0.,arc[i]-s-.10);
        double reachable=std::sqrt(safe*safe+2*.55*distance);
        result=std::min(result,reachable);
      }
    }
    return result;
  }
};

} // namespace frenet
