#pragma once
#include "perception.hpp"

// Experimental straight-course maneuver. The trajectory is fixed after offline
// validation; fresh LiDAR observations bound the opponent's occupied region.
// This is NOT the general SQP overtaking planner. No extrapolated observation
// is relabelled as a new measurement and no loss/rejection restarts the maneuver.
namespace frenet {
struct LeadBounds {
  double xmin=0, xmax=0, ymin=0, ymax=0, stamp=-INF;
  double half_length=.18, half_width=.13, vmax=.35;
  bool passed=false;
  bool bounded_association=false;
  Json::Value json() const {
    Json::Value j;
    j["xmin"]=xmin;j["xmax"]=xmax;j["ymin"]=ymin;j["ymax"]=ymax;
    j["stamp"]=stamp;j["half_length"]=half_length;j["half_width"]=half_width;
    j["vmax"]=vmax;j["passed"]=passed;
    j["bounded_association"]=bounded_association;return j;
  }
  static LeadBounds read(const Json::Value& j) {
    for(auto key:{"xmin","xmax","ymin","ymax","stamp","half_length","half_width","vmax"})
      if(!j[key].isNumeric() || !std::isfinite(j[key].asDouble()))
        throw std::runtime_error("compact_lead_schema");
    LeadBounds b;
    b.xmin=j["xmin"].asDouble();b.xmax=j["xmax"].asDouble();
    b.ymin=j["ymin"].asDouble();b.ymax=j["ymax"].asDouble();b.stamp=j["stamp"].asDouble();
    b.half_length=j["half_length"].asDouble();b.half_width=j["half_width"].asDouble();
    b.vmax=j["vmax"].asDouble();b.passed=j.get("passed",false).asBool();
    b.bounded_association=j.get("bounded_association",false).asBool();
    if(b.xmin>b.xmax || b.ymin>b.ymax || b.xmax-b.xmin>.65 || b.ymax-b.ymin>.40 ||
       b.half_length<.18 || b.half_length>.3 || b.half_width<.13 || b.half_width>.2 ||
       b.vmax<.35 || b.vmax>.55)throw std::runtime_error("compact_lead_bounds");
    return b;
  }
};

class CompactOvertake {
  const Assets& a;
  Pose origin;
  Point axis,normal;
  LeadBounds lead;
  bool acquired=false, committed=false, failed=false, admitted=false;
  double acquired_at=-INF,last_scan=-INF;
  int hits=0;
  std::vector<std::array<double,4>> nominal;
  double margin=.08;
  double lead_vmax=.35,initial_center=1.6,initial_tolerance=.05;
  double final_gap=1.2,entry_end=1.8,pass_end=2.7,rejoin_end=5.1,command_cap=1.25;
  bool rolling_admission=false;
  bool bounded_association=false;
  bool observed_merge_v4=false;
  double rear_visibility_limit=.65;
  Json::Value association;
  static bool overlap(Pose ego,double xmin,double xmax,double ymin,double ymax,double extra) {
    // SAT: ego rectangle vs the full reachable axis-aligned opponent region.
    double c=std::cos(ego.yaw),s=std::sin(ego.yaw);
    double hx=.29+extra/2,hy=.155+extra/2;
    double ox=(xmax-xmin)/2+extra/2,oy=(ymax-ymin)/2+extra/2;
    double dx=(xmin+xmax)/2-ego.x,dy=(ymin+ymax)/2-ego.y;
    return std::abs(dx)<=hx*std::abs(c)+hy*std::abs(s)+ox &&
           std::abs(dy)<=hx*std::abs(s)+hy*std::abs(c)+oy &&
           std::abs(dx*c+dy*s)<=hx+ox*std::abs(c)+oy*std::abs(s) &&
           std::abs(-dx*s+dy*c)<=hy+ox*std::abs(s)+oy*std::abs(c);
  }
public:
  Path path;
  double cap=1.40; // Existing native controller reserves 0.15 m/s headroom.
  explicit CompactOvertake(const Assets& assets,const std::string& file):a(assets),origin(a.route.front()) {
    axis=Point(std::cos(origin.yaw),std::sin(origin.yaw));normal=Point(-axis.y(),axis.x());
    auto j=json_read(file);
    const bool v2=j["version"]==2;
    bounded_association=v2 && j.get("bounded_association_v3",false).asBool();
    if(v2) {
      const bool geometry_v4=j.get("compact_geometry_v4",false).asBool();
      lead_vmax=.55;initial_center=1.6;initial_tolerance=.05;rolling_admission=true;
      final_gap=.60;pass_end=3.8;rejoin_end=5.6;command_cap=1.85;
      if(geometry_v4) {
        observed_merge_v4=true;
        rear_visibility_limit=.90;
        initial_center=1.8;
        entry_end=j["geometry"]["entry_m"].asDouble();
        pass_end=entry_end+j["geometry"]["hold_m"].asDouble();
        const double exit=j["geometry"]["exit_m"].asDouble();
        const double offset=j["geometry"]["offset_m"].asDouble();
        rejoin_end=pass_end+exit;
        if(entry_end<1.6 || entry_end>2.1 || exit<1.6 || exit>2.1 ||
           offset>-.50 || offset<-.55 || pass_end<entry_end ||
           rejoin_end<5.6 || rejoin_end>5.900000001)
          throw std::runtime_error("compact_v4_geometry_unvalidated");
      }
      const double route_length=geometry_v4?(a.route.back().xy()-a.route.front().xy()).norm():5.85;
      if(geometry_v4 && (route_length<5.849999 || route_length>6.150001 || rejoin_end>route_length-.20+1e-6))
        throw std::runtime_error("compact_v4_stopping_section");
      if(std::abs(j["lead_initial_center_distance_m"].asDouble()-initial_center)>1e-8 ||
         std::abs(j["lead_initial_center_tolerance_m"].asDouble()-initial_tolerance)>1e-8 ||
         std::abs(j["minimum_final_center_gap_m"].asDouble()-final_gap)>1e-8 ||
         std::abs(j["route_length_m"].asDouble()-route_length)>1e-8)
        throw std::runtime_error("compact_config_unvalidated");
    }
    cap=command_cap+.15;
    if((j["version"]!=1 && !v2) || j["lead_bound_size_m"].size()!=2 ||
       std::abs(j["lead_bound_size_m"][0].asDouble()-.34)>1e-8 ||
       std::abs(j["lead_bound_size_m"][1].asDouble()-.22)>1e-8 ||
       std::abs(j["lead_speed_bound_mps"].asDouble()-lead_vmax)>1e-8 ||
       std::abs(j["body_margin_m"].asDouble()-.08)>1e-8 ||
       std::abs(j["command_cap_mps"].asDouble()-command_cap)>1e-8)
      throw std::runtime_error("compact_config_unvalidated");
    path=path_read(j["path"]);
    if(path.size()<40 || path.size()>500 || (path.front().xy()-a.route.front().xy()).norm()>.001 ||
       (path.back().xy()-a.route.back().xy()).norm()>.001 ||
       std::abs(wrap(path.front().yaw-origin.yaw))>.001 ||
       std::abs(wrap(path.back().yaw-origin.yaw))>.001)
      throw std::runtime_error("compact_endpoint_invalid");
    for(auto p:a.route)if(std::abs(local(p).y)>.002)throw std::runtime_error("compact_requires_straight");
    for(auto row:j["nominal_trace"]) {
      if(row.size()!=4)throw std::runtime_error("compact_trace_schema");
      std::array<double,4> p;
      for(int k=0;k<4;++k){p[k]=row[k].asDouble();if(!std::isfinite(p[k]))throw std::runtime_error("compact_trace_nonfinite");}
      if(!nominal.empty() && (p[0]<=nominal.back()[0] || p[0]-nominal.back()[0]>.051))throw std::runtime_error("compact_trace_timing");
      nominal.push_back(p);
    }
    if(nominal.size()<100)throw std::runtime_error("compact_trace_short");
    for(auto p:path)check_map(p);
  }
  Pose local(Pose p) const {
    Point d=p.xy()-origin.xy();return {d.dot(axis),d.dot(normal),wrap(p.yaw-origin.yaw)};
  }
  Json::Value diagnostic() const {return association;}
  double offset_m() const {return local(path[path.size()/2]).y;}
  double entry_m() const {return entry_end;}
  double return_m() const {return rejoin_end-pass_end;}
  bool rear_visibility_gap(Pose p,const LeadBounds& b,double age) const {
    if(!b.bounded_association || age<0 || age>rear_visibility_limit)return false;
    const Pose q=local(p);
    if(q.x<2.7 || q.x<=b.xmax+b.vmax*age)return false;
    const double lx=q.x+.27*std::cos(q.yaw),ly=q.y+.27*std::sin(q.yaw);
    const double cx=(b.xmin+b.xmax+b.vmax*age)/2,cy=(b.ymin+b.ymax)/2;
    // 270-degree LiDAR: allow a bounded visibility gap only near its rear
    // edge (129 degrees, with a 6-degree edge allowance), while ahead of the
    // entire reachable center interval. Every swept-body check still runs.
    return std::abs(wrap(std::atan2(cy-ly,cx-lx)-q.yaw))>=2.25;
  }
  void check_map(Pose p) const {
    Point t(std::cos(p.yaw),std::sin(p.yaw)),n(-t.y(),t.x());
    // Interior grid plus half-diagonal covers space between body samples.
    const double coverage=std::hypot(.58/12,.31/8)/2;
    for(int i=0;i<=12;++i)for(int j=0;j<=8;++j)
      if(a.clearance(p.xy()+(-.29+i*.58/12)*t+(-.155+j*.31/8)*n)<.05+coverage)
        throw std::runtime_error("compact_full_body_map");
  }
  void check_pose(Pose p,const LeadBounds& b,double at) const {
    if(!std::isfinite(at) || at<b.stamp-.02)throw std::runtime_error("compact_prediction_time");
    double dt=std::max(0.,at-b.stamp);
    const double correction=b.bounded_association?.02:0.;
    const double lateral=b.bounded_association?.10*dt:0.;
    const double ymin=b.bounded_association?std::max(-.10,b.ymin-lateral):b.ymin;
    const double ymax=b.bounded_association?std::min(.10,b.ymax+lateral):b.ymax;
    // V2's fixed 0.05 m allowance represented lateral course uncertainty.
    // V3 propagates that uncertainty explicitly inside the bounded corridor;
    // do not add a second copy of the same lateral drift allowance.
    const double course=b.bounded_association?0.:.05;
    // Fixed straight lead: any forward velocity in [0, vmax], and +/-0.05m
    // lateral course uncertainty. Bounds include shape and measurement error.
    if(overlap(local(p),b.xmin-b.half_length-correction,b.xmax+b.half_length+b.vmax*dt+correction,
               ymin-b.half_width-course-correction,ymax+b.half_width+course+correction,margin))
      throw std::runtime_error("compact_dynamic_body_conflict");
    check_map(p);
  }
  void check_prediction(const Path& prediction,const LeadBounds& b,double now) const {
    if(prediction.empty() || prediction.size()>100)throw std::runtime_error("compact_prediction_empty");
    double age=now-b.stamp;
    const bool visibility_hold=rear_visibility_gap(prediction.front(),b,age);
    if(age<-.02 || age>(b.passed?8.:(visibility_hold?rear_visibility_limit:.5)))throw std::runtime_error("compact_target_stale");
    const Pose here=local(prediction.front());
    const double rear=.29*std::abs(std::cos(here.yaw))+.155*std::abs(std::sin(here.yaw));
    // For the attended v2 trial, after a fully completed merge the lead is a
    // following vehicle controlled by its own operator (release after ~3 s).
    // Keep its CURRENT reachable body and all map checks; do not turn a
    // hypothetical future rear-end of the parked ego into a failed overtake.
    // Never use this exception while changing lanes or alongside the lead.
    // A v4 car can already be physically aligned a few cm before the nominal
    // quintic endpoint. Recognize that observed merge with TIGHTER pose bounds
    // and full longitudinal body clearance, rather than an arbitrary station.
    const bool merge_pose=observed_merge_v4
      ? (here.x>=pass_end && std::abs(here.y)<.03 && std::abs(here.yaw)<.08)
      : (here.x>=rejoin_end && std::abs(here.y)<.10 && std::abs(here.yaw)<.15);
    const bool merged_rear_only=rolling_admission && b.passed && merge_pose &&
      here.x-rear>b.xmax+b.vmax*std::max(0.,age)+b.half_length+margin;
    for(size_t i=0;i<prediction.size();++i) {
      Pose prev=i?prediction[i-1]:prediction[i];
      int count=std::max(1,int(std::ceil((prediction[i].xy()-prev.xy()).norm()/.02)));
      for(int k=0;k<=count;++k) {
        double f=double(k)/count;
        Pose q{prev.x+f*(prediction[i].x-prev.x),prev.y+f*(prediction[i].y-prev.y),prev.yaw+f*wrap(prediction[i].yaw-prev.yaw)};
        check_pose(q,b,now+(merged_rear_only?0.:.05*(i?double(i-1)+f:0.)));
      }
    }
  }
  Json::Value update(const std::vector<Track>& tracks,double source,Pose ego,bool enable,bool stationary=false,bool terminal_zero_latched=false) {
    if(failed)throw std::runtime_error("compact_latched_failure");
    try {
      if(!std::isfinite(source) || source<=last_scan)throw std::runtime_error("compact_scan_stamp");
      last_scan=source;
      Pose ep=local(ego);
      // Once this already-committed maneuver has actually stopped at its
      // endpoint, report arrival rather than expanding an unseen rear vehicle
      // into a fictitious new drive conflict. The controller's terminal latch
      // remains zero and requires its independent standstill dwell.
      if(rolling_admission && committed && lead.passed &&
         (stationary || (observed_merge_v4 && terminal_zero_latched)) &&
         (ego.xy()-path.back().xy()).norm()<=.10 &&
         std::abs(wrap(ego.yaw-path.back().yaw))<=.20) {
        check_map(ego);
        Json::Value out;out["lead_bounds"]=lead.json();out["state"]="COMPACT_FINISH";
        out["lead_confirmed_hits"]=hits;out["compact_overtake"]=true;
        out["command_cap_mps"]=command_cap;out["observed_endpoint_stationary"]=stationary;
        out["endpoint_zero_latched"]=terminal_zero_latched;
        return out;
      }
      if(enable){committed=true;if(!admitted)throw std::runtime_error("compact_not_admitted");}
      // During stationary localization initialization, an early association can
      // precede the settled map pose. Reacquire only BEFORE any motion enable;
      // this never clears a committed failure or invents a fresh observation.
      if(!committed && acquired && source-lead.stamp>.5) {
        acquired=false;admitted=false;hits=0;acquired_at=-INF;lead=LeadBounds{};
      }
      std::vector<LeadBounds> matches;
      association=Json::Value(Json::objectValue);
      association["bounded_association_v3"]=bounded_association;
      association["rear_visibility_limit_s"]=rear_visibility_limit;
      association["source_stamp"]=source;
      association["previous_lead"]=acquired?lead.json():Json::Value();
      association["candidates"]=Json::Value(Json::arrayValue);
      for(const auto& tr:tracks) {
        if(!tr.confirmed || tr.misses || tr.stamp!=source || tr.cluster.points.size()<3)continue;
        double xmin=INF,xmax=-INF,ymin=INF,ymax=-INF;
        for(auto p:tr.cluster.points) {
          Pose q=local({p.x(),p.y(),0});xmin=std::min(xmin,q.x);xmax=std::max(xmax,q.x);
          ymin=std::min(ymin,q.y);ymax=std::max(ymax,q.y);
        }
        // Bounding center intervals follow from every measured point being in
        // the physical rectangle. Unlike a radial offset this also covers side
        // views and rear/front surface changes during the pass.
        LeadBounds b;
        b.vmax=lead_vmax;
        b.bounded_association=bounded_association;
        b.xmin=xmax-b.half_length-.02;b.xmax=xmin+b.half_length+.02;
        b.ymin=ymax-b.half_width-.02;b.ymax=ymin+b.half_width+.02;b.stamp=source;
        if(b.xmin>b.xmax || b.ymin>b.ymax || ymin<-.28 || ymax>.28)continue;
        if(!acquired) {
          b.xmin=std::max(b.xmin,initial_center-initial_tolerance);
          b.xmax=std::min(b.xmax,initial_center+initial_tolerance);
          b.ymin=std::max(b.ymin,-.05);b.ymax=std::min(b.ymax,.05);
        } else {
          double dt=source-lead.stamp;
          if(dt<0 || dt>.5)continue;
          // Intersect a propagated prior, not an infinitely rigid historical
          // intersection. The previous implementation shrank lateral support
          // below 0.2 mm, rejecting a still-visible vehicle after tiny scan/PF
          // shifts. Correction is bounded; a new timestamp requires real hits.
          const double correction=bounded_association?.02:0.;
          const double lateral=bounded_association?.10*dt:0.;
          // The lead only moves forward: keep that longitudinal prior.
          // Applying a per-scan negative allowance here would accumulate
          // fictitious backward motion even while waiting stationary.
          b.xmin=std::max(b.xmin,lead.xmin);
          b.xmax=std::min(b.xmax,lead.xmax+lead.vmax*dt);
          b.ymin=std::max(b.ymin,lead.ymin-lateral-correction);
          b.ymax=std::min(b.ymax,lead.ymax+lateral+correction);
          if(bounded_association) {
            b.ymin=std::max(b.ymin,-.10);b.ymax=std::min(b.ymax,.10);
          }
        }
        Json::Value diagnostic;
        diagnostic["id"]=tr.id;diagnostic["observed_xmin"]=xmin;diagnostic["observed_xmax"]=xmax;
        diagnostic["observed_ymin"]=ymin;diagnostic["observed_ymax"]=ymax;
        diagnostic["accepted"]=b.xmin<=b.xmax && b.ymin<=b.ymax;
        diagnostic["intersection"]=b.json();association["candidates"].append(diagnostic);
        if(b.xmin<=b.xmax && b.ymin<=b.ymax)matches.push_back(b);
      }
      if(matches.size()>1) {
        // Wheels/chassis can appear as disconnected laser clusters. They may
        // represent one opponent only when ALL returns fit the SAME bounded
        // rectangle. Incompatible groups remain an ambiguity, not an identity
        // switch or a closest-cluster guess.
        LeadBounds combined=matches.front();
        for(size_t k=1;k<matches.size();++k) {
          combined.xmin=std::max(combined.xmin,matches[k].xmin);
          combined.xmax=std::min(combined.xmax,matches[k].xmax);
          combined.ymin=std::max(combined.ymin,matches[k].ymin);
          combined.ymax=std::min(combined.ymax,matches[k].ymax);
        }
        if(combined.xmin>combined.xmax || combined.ymin>combined.ymax)
          throw std::runtime_error("compact_target_ambiguous");
        matches={combined};
      }
      if(matches.empty()) {
        if(!acquired)throw std::runtime_error("compact_waiting_for_target_at_configured_gap");
        // The 270-degree scanner loses a slower lead in its rear blind sector
        // before the ego has finished merging. Once the ego's REAR has cleared
        // the entire reachable lead FRONT (+ body margin), retain the original
        // source stamp and conservative forward reachable region. This marks
        // longitudinal body clearance, not completion of the whole maneuver.
        double upper=lead.xmax+lead.vmax*(source-lead.stamp)+(bounded_association?.02:0.);
        const double rear_extent=.29*std::abs(std::cos(ep.yaw))+
                                 .155*std::abs(std::sin(ep.yaw));
        if(!lead.passed && ep.x-rear_extent>=upper+lead.half_length+margin &&
           ep.x>=2.7)lead.passed=true;
        const bool visibility_hold=rear_visibility_gap(ego,lead,source-lead.stamp);
        association["rear_visibility_hold"]=visibility_hold;
        if(!lead.passed && !visibility_hold && source-lead.stamp>.30)throw std::runtime_error("compact_target_lost");
      } else {
        bool was_passed=lead.passed;lead=matches.front();lead.passed=was_passed;
        if(!acquired){acquired=true;acquired_at=source;}++hits;
      }
      if(hits<5 || source-acquired_at<.8)throw std::runtime_error("compact_target_confirming");
      if(!committed) {
        if(std::hypot(ep.x,ep.y)>.10 || std::abs(ep.yaw)>.15)throw std::runtime_error("compact_start_pose");
        // V1 admitted a full open-loop reachable tube to the endpoint. V2 is
        // an attended receding-horizon trial: map-check the whole maneuver,
        // check the launch horizon, then guard every actual MPC prediction.
        // It makes no claim that a moving lead will allow eventual merging;
        // an unsafe future sweep still causes a latched stop.
        for(auto p:nominal) {
          check_map({p[1],p[2],p[3]});
          if(!rolling_admission || p[0]<=1.2)
            check_pose({p[1],p[2],p[3]},lead,source+p[0]);
        }
        double endx=local(path.back()).x;
        if(!rolling_admission && endx-lead.xmax-lead.vmax*nominal.back()[0]<final_gap)throw std::runtime_error("compact_insufficient_rejoin_gap");
        admitted=true;
      }
      check_pose(ego,lead,source);
      Json::Value out;
      out["lead_bounds"]=lead.json();out["lead_confirmed_hits"]=hits;
      out["state"]=ep.x<entry_end?"COMPACT_ENTRY":ep.x<pass_end?"COMPACT_PASS":ep.x<rejoin_end?"COMPACT_REJOIN":"COMPACT_FINISH";
      out["compact_overtake"]=true;out["command_cap_mps"]=command_cap;
      out["compact_association"]=association;return out;
    } catch(const std::exception&) {
      if(committed)failed=true;
      throw;
    }
  }
};
} // namespace frenet
