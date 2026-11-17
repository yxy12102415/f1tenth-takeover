#pragma once
#include "perception.hpp"
#include <deque>
namespace frenet {
// Convex support preserves an extended wall contour without occupying every
// empty corner of its axis-aligned box. The hull also retains measured solid
// interior; this is not a point-only obstacle representation.
inline std::vector<Point> measured_support_hull(std::vector<Point> points) {
  if(points.size()<3)return points;
  std::sort(points.begin(),points.end(),[](Point a,Point b){return a.x()<b.x() || (a.x()==b.x() && a.y()<b.y());});
  points.erase(std::unique(points.begin(),points.end(),[](Point a,Point b){return (a-b).squaredNorm()<1e-18;}),points.end());
  if(points.size()<3)return points;
  auto cross=[](Point a,Point b,Point c){Point u=b-a,v=c-a;return u.x()*v.y()-u.y()*v.x();};
  std::vector<Point> hull(2*points.size());size_t n=0;
  for(auto q:points){while(n>=2 && cross(hull[n-2],hull[n-1],q)<=0)--n;hull[n++]=q;}
  size_t lower=n+1;
  for(int i=int(points.size())-2;i>=0;--i){auto q=points[i];while(n>=lower && cross(hull[n-2],hull[n-1],q)<=0)--n;hull[n++]=q;}
  hull.resize(n-1);return hull;
}

struct Occupancy {
  int id=0,hits=0,clear_scans=0,motion_score=0;
  bool has_moved=false,fixed_world=false,map_supported=false,hazard=false;
  bool route_prior_observed=true;
  bool designated_route_lead=false;
  double stamp=0;
  Point lo=Point::Zero(),hi=Point::Zero(),velocity=Point::Zero();
  std::vector<Point> samples;
  Point sensor=Point::Zero(),velocity_lo=Point::Zero(),velocity_hi=Point::Zero();
};
class DynamicMemory {
  const Assets& a;
  struct Frame {double stamp;Point lo,hi;std::vector<Point> hull;};
  struct Item {
    Occupancy o;std::deque<Frame> history;
    Point filtered_velocity=Point::Zero();
    bool static_confirmed=false;
  };
  std::vector<Item> active;
  std::vector<Occupancy> hazards;
  double last=-INF;
  int serial=1,retired=0,cleared=0;
  static Point center(const Occupancy& o){return .5*(o.lo+o.hi);}
  bool visible_free(Point lo,Point hi,const std::vector<float>& ranges,double amin,double inc,double rmin,double rmax,Pose pose) const {
    Point laser=pose.xy()+.27*Point(std::cos(pose.yaw),std::sin(pose.yaw));
    // Clear with actual rays, not elapsed time. Occupied map cells remain
    // represented by the independent static-map layer, not phantom movers.
    int cells=0;
    for(double x=lo.x();x<=hi.x()+.025;x+=.075)for(double y=lo.y();y<=hi.y()+.025;y+=.075){
      if(++cells>4096)return false;
      Point q(x,y);if(a.clearance(q)<=.05)continue;
      Point d=q-laser;int k=int(std::round((wrap(std::atan2(d.y(),d.x())-pose.yaw)-amin)/inc));
      if(k<1 || k+1>=int(ranges.size()))return false;
      for(int j=k-1;j<=k+1;++j)if(!std::isfinite(ranges[j]) || ranges[j]<rmin || ranges[j]>rmax || ranges[j]<d.norm()+.14)return false;
    }
    return true;
  }
public:
  explicit DynamicMemory(const Assets& assets):a(assets){}
  void observe(const std::vector<Track>& tracks,double source,const std::vector<float>& ranges,double amin,double inc,double rmin,double rmax,Pose pose,bool stationary=false){
    if(!std::isfinite(source) || source<=last || !std::isfinite(inc) || inc<=0)throw std::runtime_error("occupancy_scan_stamp");
    last=source;std::vector<Occupancy> observations;
    Point sensor=pose.xy()+.27*Point(std::cos(pose.yaw),std::sin(pose.yaw));
    for(const auto& t:tracks){
      if(t.stamp!=source || t.misses || t.cluster.points.empty())continue;
      Occupancy o;o.stamp=source;o.hits=1;o.sensor=sensor;o.samples=t.cluster.points;o.lo=o.hi=o.samples.front();int mapped=0;
      for(auto p:o.samples){o.lo=o.lo.cwiseMin(p);o.hi=o.hi.cwiseMax(p);if(a.clearance(p)<.23)++mapped;}
      o.map_supported=mapped>=int(std::ceil(.8*o.samples.size()));
      bool joined=false;
      for(auto& v:observations){
        Point gap=(o.lo-v.hi).cwiseMax(v.lo-o.hi).cwiseMax(Point::Zero()),lo=o.lo.cwiseMin(v.lo),hi=o.hi.cwiseMax(v.hi);
        if(v.map_supported==o.map_supported && gap.norm()<=.17 && (hi-lo).maxCoeff()<=.60){v.lo=lo;v.hi=hi;v.samples.insert(v.samples.end(),o.samples.begin(),o.samples.end());joined=true;break;}
      }
      if(!joined)observations.push_back(o);
    }
    bool fused=true;
    while(fused){
      fused=false;
      for(size_t i=0;i<observations.size()&&!fused;++i)for(size_t j=i+1;j<observations.size();++j){
        auto& v=observations[i];const auto& o=observations[j];
        Point gap=(o.lo-v.hi).cwiseMax(v.lo-o.hi).cwiseMax(Point::Zero());
        Point lo=o.lo.cwiseMin(v.lo),hi=o.hi.cwiseMax(v.hi);
        if(v.map_supported==o.map_supported && gap.norm()<=.170001 && (hi-lo).maxCoeff()<=.60){
          v.lo=lo;v.hi=hi;v.samples.insert(v.samples.end(),o.samples.begin(),o.samples.end());
          observations.erase(observations.begin()+j);fused=true;break;
        }
      }
    }
    std::vector<std::tuple<double,size_t,size_t>> pairs;
    for(size_t i=0;i<active.size();++i)for(size_t j=0;j<observations.size();++j){
      const auto& old=active[i].o;double age=source-old.stamp;if(age>1.)continue;
      Point expected=center(old)+old.velocity*age;
      double dist=(expected-center(observations[j])).norm();
      double shape=((old.hi-old.lo)-(observations[j].hi-observations[j].lo)).norm();
      if(dist<.28+.65*age && shape<.65)
        pairs.emplace_back(dist+.2*shape,i,j);
    }
    std::sort(pairs.begin(),pairs.end());std::vector<bool> used_old(active.size()),used_new(observations.size());
    for(auto pair:pairs){
      size_t i=std::get<1>(pair),j=std::get<2>(pair);if(used_old[i]||used_new[j])continue;
      auto& item=active[i];auto next=observations[j];const auto old=item.o;
      next.id=old.id;next.hits=old.hits+1;next.velocity=item.filtered_velocity;next.has_moved=old.has_moved;
      next.motion_score=old.motion_score;
      item.history.push_back({source,next.lo,next.hi,measured_support_hull(next.samples)});
      while(item.history.size()>1 && source-item.history.front().stamp>.6)item.history.pop_front();
      const auto& anchor=item.history.front();double dt=source-anchor.stamp;
      Point displacement=center(next)-.5*(anchor.lo+anchor.hi);
      if(dt>=.18){
        Point support_shift;
        for(int axis=0;axis<2;++axis){
          double dl=next.lo[axis]-anchor.lo[axis],dh=next.hi[axis]-anchor.hi[axis];
          support_shift[axis]=dl*dh>0?(std::abs(dl)<std::abs(dh)?dl:dh):0.;
        }
        // Newly visible sides expand only one edge. That is shape revelation,
        // not translation: both support edges must agree on signed motion.
        Point observed=support_shift/dt;double shape=((next.hi-next.lo)-(anchor.hi-anchor.lo)).norm();
        if(observed.norm()>.75)observed*=.75/observed.norm();
        // Long-baseline support displacement avoids differentiating 50ms
        // LiDAR phase / PF noise. Shape changes lower motion confidence.
        // An expanding silhouette provides weak evidence of zero common
        // translation, not grounds to freeze an old velocity indefinitely.
        // Coherent translating support retains the previous shape gate.
        double gain=shape<.04?.55:(support_shift.norm()<.02?.15:0.);
        item.filtered_velocity=(1-gain)*item.filtered_velocity+gain*observed;
        next.velocity=item.filtered_velocity;
        bool coherent=((next.lo-anchor.lo)-(next.hi-anchor.hi)).norm()<.06;
        // Distinguish persistent translation from a one-frame speed spike.
        // Both support edges must translate coherently over >=300 ms, with
        // 3 consecutive supporting scans. Retain the internal velocity while
        // publishing zero for established static objects, so real motion can
        // still revoke the latch rather than repeatedly restarting a filter.
        const bool evidence=dt>=.30 && support_shift.norm()>.045 &&
          coherent && next.velocity.norm()>.08;
        next.motion_score=evidence?std::min(3,old.motion_score+1):0;
        const bool fast_translation=dt>=.18 && coherent &&
          support_shift.norm()>.10 && observed.norm()>.35;
        if(next.motion_score>=3 || fast_translation){next.has_moved=true;item.static_confirmed=false;}
        if(!next.has_moved && dt>=.30 && support_shift.norm()<.025 && displacement.norm()<.04)
          item.static_confirmed=true;
        // Shape revelation / PF noise can move the box centre by several
        // centimetres while its common support remains stationary. Require
        // both low filtered speed AND a quiet shared support baseline before
        // settling; a past movement flag must not last forever after stopping.
        if(displacement.norm()<.05 && support_shift.norm()<.025 && dt>=.45 && next.velocity.norm()<.06){
          next.velocity.setZero();item.filtered_velocity.setZero();next.has_moved=false;
          next.motion_score=0;item.static_confirmed=true;
        }
      }
      if(stationary){next.velocity.setZero();next.has_moved=false;}
      else if(item.static_confirmed && !next.has_moved)next.velocity.setZero();
      else if(next.map_supported && !next.has_moved)next.velocity.setZero();
      next.fixed_world=stationary || (item.static_confirmed && !next.has_moved);
      Point raw_lo=next.lo,raw_hi=next.hi;
      // Learn only a bounded history of measured support, translated into
      // this frame. No all-time box union and no inherited raw cluster ID.
      if(!next.map_supported)for(const auto& h:item.history){
        Point motion=next.velocity*(source-h.stamp);
        Point lo=h.lo+motion,hi=h.hi+motion;
        if(((hi-lo)-(raw_hi-raw_lo)).norm()<.25){next.lo=next.lo.cwiseMin(lo);next.hi=next.hi.cwiseMax(hi);for(auto q:h.hull)next.samples.push_back(q+motion);}
      }
      next.samples=measured_support_hull(next.samples);
      item.o=std::move(next);used_old[i]=true;used_new[j]=true;
    }
    for(size_t j=0;j<observations.size();++j)if(!used_new[j]){
      auto o=observations[j];o.id=serial++;o.fixed_world=stationary;
      o.samples=measured_support_hull(o.samples);
      active.push_back({o,{{source,o.lo,o.hi,o.samples}}});
    }
    std::vector<Item> keep;
    for(auto& item:active){
      auto& o=item.o;double age=source-o.stamp;
      if(age<=.001){keep.push_back(std::move(item));continue;}
      if(visible_free(o.lo-Point::Constant(.04),o.hi+Point::Constant(.04),ranges,amin,inc,rmin,rmax,pose))++o.clear_scans;else o.clear_scans=0;
      if(o.clear_scans>=3){++cleared;continue;}
      if(age<=(o.hits>=4?1.:.35)){keep.push_back(std::move(item));continue;}
      ++retired;
      // Map proximity alone does not prove the obstacle is part of the wall.
      // Preserve an occluded measured object until free-ray evidence clears it.
      if(o.hits<4)continue;
      // Long occlusion is explicit bounded unknown occupancy, not a CV box
      // extrapolated for minutes. It can only be removed by ray evidence.
      const bool known_static=o.fixed_world || (o.hits>=4 && !o.has_moved && o.velocity.norm()<=.03);
      Occupancy unknown=o;
      if(!known_static) {
        Point shift=o.velocity*std::min(1.,age);
        unknown.lo+=shift-Point::Constant(.12);unknown.hi+=shift+Point::Constant(.12);
      }
      unknown.velocity.setZero();unknown.hazard=true;unknown.fixed_world=known_static;
      // Preserve actual movement evidence: a moving map-adjacent object must
      // not become a static wall just because visibility was lost.
      unknown.has_moved=o.has_moved || (!known_static && o.velocity.norm()>.08);
      hazards.push_back(unknown);
    }
    active=std::move(keep);
    for(auto& o:hazards){if(visible_free(o.lo,o.hi,ranges,amin,inc,rmin,rmax,pose))++o.clear_scans;else o.clear_scans=0;}
    hazards.erase(std::remove_if(hazards.begin(),hazards.end(),[&](const Occupancy& o){if(o.clear_scans>=3){++cleared;return true;}return false;}),hazards.end());
    // A reobserved component supersedes its old unknown envelope under real
    // geometric support, rather than retaining duplicate wheel hypotheses.
    hazards.erase(std::remove_if(hazards.begin(),hazards.end(),[&](const Occupancy& old){
      for(const auto& item:active){
        if(item.o.stamp!=source)continue;
        Point gap=(old.lo-item.o.hi).cwiseMax(item.o.lo-old.hi).cwiseMax(Point::Zero());
        if(gap.norm()<.12 && (center(old)-center(item.o)).norm()<.45)return true;
      }
      return false;
    }),hazards.end());
    if(active.size()+hazards.size()>128)throw std::runtime_error("occupancy_capacity");
  }
  Json::Value snapshot() const {
    Json::Value out(Json::arrayValue);
    auto append=[&](const Occupancy& o){
      Json::Value v;v["id"]=o.id;v["stamp"]=o.stamp;v["hits"]=o.hits;v["has_moved"]=o.has_moved;v["fixed_world"]=o.fixed_world;v["map_supported"]=o.map_supported;v["visibility_hazard"]=o.hazard;
      v["memory_state"]=o.hazard?(o.fixed_world?"occluded_static":"occluded_unknown"):(last-o.stamp>.08?"predicted_hold":"observed");
      v["observation_age_s"]=std::max(0.,last-o.stamp);v["xmin"]=o.lo.x();v["xmax"]=o.hi.x();v["ymin"]=o.lo.y();v["ymax"]=o.hi.y();v["vx"]=o.velocity.x();v["vy"]=o.velocity.y();v["sensor_x"]=o.sensor.x();v["sensor_y"]=o.sensor.y();v["clear_scans"]=o.clear_scans;v["stationary_history"]=o.fixed_world || (o.hits>=4&&!o.has_moved&&o.velocity.norm()<.08);
      v["motion_confirmation_scans"]=o.motion_score;
      v["motion_policy"]="support_translation_300ms_three_scans_with_static_latch";
      // Compact objects keep the physical hidden-body completion. Extended,
      // quiet contours use their measured convex support, including history.
      if((!o.hazard || o.fixed_world) && !o.has_moved && o.velocity.norm()<=.08 && (o.hi-o.lo).maxCoeff()>.65 && o.samples.size()>=2 && o.samples.size()<=128) {
        v["support_hull"]=Json::Value(Json::arrayValue);
        for(auto q:o.samples){Json::Value point(Json::arrayValue);point.append(q.x());point.append(q.y());v["support_hull"].append(point);}
        v["shape_model"]="extended_measured_convex_support";
      }
      out.append(v);
    };
    for(const auto& item:active){append(item.o);}
    for(const auto& o:hazards){append(o);}
    return out;
  }
  Json::Value diagnostic() const {Json::Value d;d["active_tracks"]=int(active.size());d["occlusion_hazards"]=int(hazards.size());d["retired_tracks"]=retired;d["ray_cleared_tracks"]=cleared;d["active_lifetime_s"]=1.;d["shape_history_s"]=.6;return d;}
};
} // namespace frenet
