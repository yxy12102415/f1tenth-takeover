#pragma once
#include "perception.hpp"
#include "corridor.hpp"
#include "speed_geometry.hpp"
#include "tracked_occupancy.hpp"
#include <utility>
#include <map>
namespace frenet {
class GenericAvoidance {
  friend struct GenericProbeAccess;
  const Assets& a;
  DynamicMemory memory;
  Path incumbent;
  double incumbent_offset=0,incumbent_entry=0,incumbent_return=0,incumbent_score=0;
  Json::Value diagnostics,incumbent_terms;
  static bool overlap(Pose p,Point lo,Point hi,double extra) {
    double c=std::cos(p.yaw),s=std::sin(p.yaw),hx=.29+extra/2,hy=.155+extra/2;
    Point center=(lo+hi)/2,half=(hi-lo)/2+Point::Constant(extra/2),d=center-p.xy();
    return std::abs(d.x())<=hx*std::abs(c)+hy*std::abs(s)+half.x() &&
      std::abs(d.y())<=hx*std::abs(s)+hy*std::abs(c)+half.y() &&
      std::abs(d.x()*c+d.y()*s)<=hx+half.x()*std::abs(c)+half.y()*std::abs(s) &&
      std::abs(-d.x()*s+d.y()*c)<=hy+half.x()*std::abs(s)+half.y()*std::abs(c);
  }
  static bool quiet_support(const Occupancy& o) {
    const bool mapped_static=o.map_supported && o.hits>=4 && !o.has_moved && o.velocity.norm()<=.03;
    return o.fixed_world || ((!o.hazard || mapped_static) && !o.has_moved && o.velocity.norm()<=.08);
  }
  static bool overlap_occupancy(Pose p,const Occupancy& o,Point lo,Point hi,double extra) {
    if(!overlap(p,lo,hi,extra))return false;
    if(!quiet_support(o) || o.samples.size()<2)return true;
    Point tangent(std::cos(p.yaw),std::sin(p.yaw)),normal(-tangent.y(),tangent.x());
    auto separated=[&](Point axis){
      double length=axis.norm();if(length<1e-10)return false;axis/=length;
      double low=INF,high=-INF;for(auto q:o.samples){double v=q.dot(axis);low=std::min(low,v);high=std::max(high,v);}
      // Same split clearance as box SAT: half on the ego rectangle and
      // half on the obstacle in map axes. Vehicle dimensions stay unchanged.
      double radius=(.29+extra/2)*std::abs(axis.dot(tangent))+(.155+extra/2)*std::abs(axis.dot(normal));
      double padding=extra/2*(std::abs(axis.x())+std::abs(axis.y())),center=p.xy().dot(axis);
      return center+radius<low-padding || center-radius>high+padding;
    };
    if(separated(tangent)||separated(normal)||separated(Point(1,0))||separated(Point(0,1)))return false;
    for(size_t i=0;i<o.samples.size();++i){Point edge=o.samples[(i+1)%o.samples.size()]-o.samples[i];if(separated(Point(-edge.y(),edge.x())))return false;}
    return true;
  }
  static Occupancy read(const Json::Value& v) {
    Occupancy o;o.id=v["id"].asInt();o.stamp=v["stamp"].asDouble();o.hits=v["hits"].asInt();o.has_moved=v.get("has_moved",false).asBool();o.fixed_world=v.get("fixed_world",false).asBool();o.map_supported=v.get("map_supported",false).asBool();o.hazard=v.get("visibility_hazard",false).asBool();
    o.lo=Point(v["xmin"].asDouble(),v["ymin"].asDouble());
    o.hi=Point(v["xmax"].asDouble(),v["ymax"].asDouble());
    o.velocity=Point(v["vx"].asDouble(),v["vy"].asDouble());
    o.sensor=Point(v["sensor_x"].asDouble(),v["sensor_y"].asDouble());
    o.route_prior_observed=v.get("memory_state","observed").asString()=="observed";
    o.designated_route_lead=v.get("semantic_role","")=="lead_vehicle" && v.get("lead_route_aligned",false).asBool();
    for(auto key:{"stamp","xmin","xmax","ymin","ymax","vx","vy","sensor_x","sensor_y"})
      if(!v[key].isNumeric() || !std::isfinite(v[key].asDouble()))throw std::runtime_error("occupancy_schema");
    if((o.lo.array()>o.hi.array()).any() || o.velocity.norm()>.8)throw std::runtime_error("occupancy_bounds");
    if(v.isMember("support_hull")) {
      if(!v["support_hull"].isArray() || v["support_hull"].size()>128)throw std::runtime_error("occupancy_hull_schema");
      for(const auto& q:v["support_hull"]) {
        if(q.size()!=2 || !q[0].isNumeric() || !q[1].isNumeric())throw std::runtime_error("occupancy_hull_schema");
        Point point(q[0].asDouble(),q[1].asDouble());
        if(!std::isfinite(point.sum()) || (point.array()<o.lo.array()-1e-6).any() || (point.array()>o.hi.array()+1e-6).any())throw std::runtime_error("occupancy_hull_bounds");
        o.samples.push_back(point);
      }
      o.samples=measured_support_hull(o.samples);
    }
    return o;
  }
public:
  mutable Json::Value collision_diagnostic;
  void refresh_prearm_path() {
    // Only the stationary, disabled planner calls this. Observation memory
    // remains intact; discard a warm-up route chosen before PF/shape settled.
    incumbent.clear();incumbent_terms=Json::Value();
    incumbent_offset=incumbent_entry=incumbent_return=incumbent_score=0;
  }
  int geometric_absorptions=0;
  bool map_speed_feasibility=false;
  bool stationary_scene=false; // Explicit stationary-obstacle commissioning scope only.
  bool route_aligned_opponent_prior=false;
  double speed_response_headroom=.20;
  double running_floor=.5;
  double preferred_gap=.14; // Desired swept-body gap; independent controller hard gap remains 8 cm.
  double seed_reserve=.04; // tracking / newly revealed surface allowance
  bool allow_envelope_seeds=true; // probe may isolate the QP for geometry tests
  double max_offset=.75;
  double cap=1.40; // First commissioning: effective cap = 1.25 m/s, acceleration unchanged.
  explicit GenericAvoidance(const Assets& assets):a(assets),memory(assets) {}
  void map_check(Pose p) const {
    if(body_map_reserve(a,p)<0)throw std::runtime_error("generic_full_body_map");
  }
  void observe(const std::vector<Track>& tracks,double source,const std::vector<float>& ranges,
               double amin,double inc,double rmin,double rmax,Pose pose) {
    memory.observe(tracks,source,ranges,amin,inc,rmin,rmax,pose,stationary_scene);
  }
  Json::Value snapshot() const {return memory.snapshot();}
  Json::Value memory_diagnostic() const {return memory.diagnostic();}
  // Controlled-track policy: quiet measured support has zero process growth.
  // Established route-aligned movers get <=2 cm longitudinal / <=1 cm lateral
  // uncertainty below. Unknown/lateral motion and occlusion retain fallback.
  static void unconstrained_bounds(const Occupancy& o,double now,double future,Point& lo,Point& hi) {
    double age=now-o.stamp;
    if(age<-.02 || !std::isfinite(age+future) || future<0)throw std::runtime_error("occupancy_prediction_time");
    age=clip(age,0.,1.); // Lifetime is managed by observed/occluded state, never unbounded CV drift.
    const double motion_confidence=clip((o.velocity.norm()-.03)/.15,0.,1.);
    const double total=age+future;
    const double velocity_uncertainty=o.hits<4?.35:(.03+.05*motion_confidence);
    // Fresh source age is ordinary prediction time, not instantly full-speed
    // motion uncertainty. Prolonged missed observations additionally expand
    // at the maximum unobserved-motion rate; this never throws on identity.
    // A tiny map-backed residual is a surface measurement, not an unseen
    // vehicle travelling at an arbitrary velocity. Keep its measured support
    // and padding; observed motion or visibility loss revokes this prior.
    const bool mapped_surface=o.map_supported && !o.hazard && !o.has_moved &&
      o.velocity.norm()<=.03 && (o.hi-o.lo).maxCoeff()<=.06;
    const bool stationary_history=quiet_support(o) || mapped_surface;
    // The explicitly commissioned stationary scene has zero obstacle process
    // motion. Keep observation-age/measurement padding; predicting arbitrary
    // acceleration for fixed objects contradicts that scene assumption.
    // Preserve a learned map-backed stationary class through visibility loss.
    // Otherwise every temporarily occluded wall becomes a hypothetical vehicle
    // accelerating into the corridor. Free-space unknowns and observed movers
    // retain the full unknown-motion envelope.
    // Confirmed static classification has priority over an occlusion marker.
    // Otherwise a non-map static object becomes a growing unknown solely
    // because visibility was lost, despite fixed_world remaining true.
    const double uncertainty=stationary_history ? 0. : o.hazard ? .75*future
      : std::min(.55*total,velocity_uncertainty*total+.04*total*total+.55*std::max(0.,age-.35));
    Point movement=stationary_history?Point::Zero().eval():Point(o.velocity*(age+future));
    if(o.hazard){movement.setZero();}
    // Complete hidden body AWAY from the measurement sensor. Symmetric
    // dilation in the sight direction invents body in LiDAR-observed free
    // space in front of the visible surface and falsely blocks approach.
    Point missing=o.map_supported || o.hazard || (quiet_support(o) && o.samples.size()>=2) ? Point::Zero().eval() : (Point::Constant(.36)-(o.hi-o.lo)).cwiseMax(Point::Zero()).eval();
    Point direction=(o.lo+o.hi)/2-o.sensor;
    int depth=std::abs(direction.x())>=std::abs(direction.y())?0:1;
    const double measurement_pad=stationary_history?0.:.02;
    Point low_pad=Point::Constant(measurement_pad)+missing/2,high_pad=low_pad;
    low_pad[depth]=measurement_pad+(direction[depth]<0?missing[depth]:0.);
    high_pad[depth]=measurement_pad+(direction[depth]>=0?missing[depth]:0.);
    lo=o.lo+movement-low_pad-Point::Constant(uncertainty);
    hi=o.hi+movement+high_pad+Point::Constant(uncertainty);
    // Intersect the CV uncertainty with the bounded-speed reachable region.
    // Do not add a full-speed uncertainty radius ON TOP of full-speed motion.
    const double reachable=std::max(.75,o.velocity.norm()+.10)*total;
    lo=lo.cwiseMax(o.lo-low_pad-Point::Constant(reachable));
    hi=hi.cwiseMin(o.hi+high_pad+Point::Constant(reachable));
  }
  bool route_prior_projection(const Occupancy& o,double now,double& station,Pose& reference) const {
    if(!route_aligned_opponent_prior || !o.route_prior_observed || o.hazard || o.map_supported || o.hits<4 ||
       now-o.stamp<-.02 || now-o.stamp>.50 ||
       (o.hi-o.lo).maxCoeff()<.12 || (o.hi-o.lo).maxCoeff()>.65)return false;
    // Immutable route and exact measured support: no spatial quantization,
    // source-stamp renewal or extrapolation cache. Freshness gates stay above.
    using Key=std::array<double,9>;
    struct Projection {double station;Pose reference;bool aligned;};
    struct Cache {const Assets* assets=nullptr;std::map<Key,Projection> values;};
    static thread_local Cache cache;
    if(cache.assets!=&a){cache.assets=&a;cache.values.clear();}
    Key key{{o.stamp,o.lo.x(),o.lo.y(),o.hi.x(),o.hi.y(),o.velocity.x(),o.velocity.y(),double(o.id),double(o.designated_route_lead)}};
    auto known=cache.values.find(key);
    if(known!=cache.values.end()){station=known->second.station;reference=known->second.reference;return known->second.aligned;}
    Point centre=.5*(o.lo+o.hi);double best=INF;
    for(size_t j=1;j<a.route.size();++j) {
      Point e=a.route[j].xy()-a.route[j-1].xy();
      double f=clip((centre-a.route[j-1].xy()).dot(e)/e.squaredNorm(),0.,1.);
      Point q=a.route[j-1].xy()+f*e;double d=(centre-q).squaredNorm();
      if(d<best){best=d;station=a.arc[j-1]+f*e.norm();reference={q.x(),q.y(),a.route[j-1].yaw+f*wrap(a.route[j].yaw-a.route[j-1].yaw)};}
    }
    Point t(std::cos(reference.yaw),std::sin(reference.yaw)),n(-t.y(),t.x());
    // This is an explicit course assumption, never an identity requirement.
    // Clearly lateral/reversing motion, wall supports and visibility loss
    // immediately revoke it; their original uncertainty stays intact.
    const double route_band=o.designated_route_lead?.65:.35;
    bool aligned=best<=route_band*route_band && std::abs(o.velocity.dot(n))<=.08 && o.velocity.dot(t)>=-.08;
    if(cache.values.size()>=256)cache.values.clear();
    cache.values.emplace(key,Projection{station,reference,aligned});
    return aligned;
  }
  void bounds(const Occupancy& o,double now,double future,Point& lo,Point& hi) const {
    unconstrained_bounds(o,now,future,lo,hi);
    double station=0;Pose ref;
    if(!route_prior_projection(o,now,station,ref))return;
    const double age=clip(now-o.stamp,0.,1.),total=age+future;
    const bool stationary=o.fixed_world || (!o.has_moved && o.velocity.norm()<=.08);
    Point t(std::cos(ref.yaw),std::sin(ref.yaw)),n(-t.y(),t.x());
    double along=stationary?0.:o.velocity.dot(t),across=stationary?0.:o.velocity.dot(n);
    double target=clip(station+along*total,0.,a.arc.back());
    // Do not continue a constrained prediction past the reference endpoint.
    if(station+along*total<0 || station+along*total>a.arc.back())return;
    size_t h=std::min(a.route.size()-1,size_t(std::upper_bound(a.arc.begin(),a.arc.end(),target)-a.arc.begin()));
    size_t l=h?h-1:0;double f=h==l?0:(target-a.arc[l])/(a.arc[h]-a.arc[l]);
    Pose next{a.route[l].x+f*(a.route[h].x-a.route[l].x),a.route[l].y+f*(a.route[h].y-a.route[l].y),a.route[l].yaw+f*wrap(a.route[h].yaw-a.route[l].yaw)};
    Point nt(std::cos(next.yaw),std::sin(next.yaw)),nn(-nt.y(),nt.x());
    Point centre=.5*(o.lo+o.hi);double offset=(centre-ref.xy()).dot(n)+across*total;
    const double tangent_residual=(centre-ref.xy()).dot(t);
    Point destination=next.xy()+offset*nn+tangent_residual*nt;
    Occupancy support=o;support.fixed_world=true;support.has_moved=false;support.velocity.setZero();
    Point low,high;unconstrained_bounds(support,o.stamp,0,low,high);
    double c=std::cos(next.yaw-ref.yaw),s=std::sin(next.yaw-ref.yaw);
    lo=Point::Constant(INF);hi=Point::Constant(-INF);
    for(Point corner:{low,high,Point(low.x(),high.y()),Point(high.x(),low.y())}) {
      Point d=corner-centre,q=destination+Point(c*d.x()-s*d.y(),s*d.x()+c*d.y());
      lo=lo.cwiseMin(q);hi=hi.cwiseMax(q);
    }
    // Position advance follows the route; uncertainty is bounded rather than
    // accumulating an arbitrary lateral acceleration envelope on a known track.
    double longitudinal=stationary?0.:.02*clip(total/.5,0.,1.);
    const double lateral=stationary?0.:.01*clip(total/.5,0.,1.);
    Point pad=longitudinal*nt.cwiseAbs()+lateral*nn.cwiseAbs();
    lo-=pad;hi+=pad;
  }
  bool mapped_static_surface(const Occupancy& o) const {
    // Strong geometric wall prior, narrower than map_supported's 23 cm
    // association gate. Keep measured support; only unseen motion growth is
    // replaced by fixed-surface padding. Observed motion revokes the prior.
    if(!o.map_supported || o.has_moved || o.hits<4 || o.velocity.norm()>.03 ||
       (o.hi-o.lo).maxCoeff()>.60)return false;
    int near=0;
    for(int x=0;x<3;++x)for(int y=0;y<3;++y) {
      Point q(o.lo.x()+.5*x*(o.hi.x()-o.lo.x()),o.lo.y()+.5*y*(o.hi.y()-o.lo.y()));
      double gap=a.clearance(q);
      if(gap>.150001)return false;
      if(gap<=.100001)++near;
    }
    return near>=8;
  }
  enum class SceneGroup {All,Static,Moving};
  struct PreparedObject {
    Occupancy raw,support;Json::Value original;Point low,high;
    bool wall_prior=false,is_static=false,invariant=false;
  };
  struct PreparedScene {
    const Assets* assets=nullptr;double now=0;bool route_prior=false;
    std::vector<PreparedObject> objects;
    bool time_dependent=false;
  };
  PreparedScene prepare_observations(const Json::Value& occupancy,double now) const {
    if(!occupancy.isArray() || occupancy.size()>128)throw std::runtime_error("occupancy_missing");
    PreparedScene scene;scene.assets=&a;scene.now=now;scene.route_prior=route_aligned_opponent_prior;
    scene.objects.reserve(occupancy.size());
    for(const auto& value:occupancy) {
      PreparedObject row;row.original=value;row.raw=read(value);row.support=row.raw;
      row.is_static=row.raw.hazard || (!row.raw.has_moved && row.raw.velocity.norm()<=.08);
      row.wall_prior=mapped_static_surface(row.support);
      if(row.wall_prior){row.support.hazard=false;row.support.fixed_world=true;}
      row.invariant=quiet_support(row.support);scene.time_dependent|=!row.invariant;
      bounds(row.support,now,0.,row.low,row.high);scene.objects.push_back(std::move(row));
    }
    return scene;
  }
  void check_scene_identity(const PreparedScene& scene) const {
    if(scene.assets!=&a || scene.route_prior!=route_aligned_opponent_prior)
      throw std::runtime_error("prepared_observation_context_changed");
  }
  void forecast_region(const PreparedObject& row,const PreparedScene& scene,double future,Point& lo,Point& hi) const {
    if(!std::isfinite(future) || future<0)throw std::runtime_error("occupancy_prediction_time");
    if(row.invariant || future==0.){lo=row.low;hi=row.high;}
    else bounds(row.support,scene.now,future,lo,hi);
  }
  // Caller checks the map once at this exact pose. The immutable preparation
  // never survives this observation/time context or refreshes source stamps.
  void check_prepared_obstacles(Pose p,const PreparedScene& scene,double future,double clearance=.08,
       SceneGroup group=SceneGroup::All,bool skip_checked_static=false) const {
    check_scene_identity(scene);
    for(const auto& row:scene.objects) {
      if((group==SceneGroup::Static && !row.is_static) || (group==SceneGroup::Moving && row.is_static) ||
         (skip_checked_static && row.is_static && row.invariant))continue;
      Point lo,hi;forecast_region(row,scene,future,lo,hi);
      if(overlap_occupancy(p,row.support,lo,hi,clearance)) {
        collision_diagnostic=Json::Value(Json::objectValue);
        collision_diagnostic["occupancy"]=row.original;
        collision_diagnostic["mapped_static_surface_prior"]=row.wall_prior;
        collision_diagnostic["shape_model"]=quiet_support(row.support)&&row.support.samples.size()>=2?"measured_convex_support":"box";
        collision_diagnostic["now"]=scene.now;collision_diagnostic["future_s"]=future;
        collision_diagnostic["predicted_box"]=Json::Value(Json::arrayValue);
        for(double z:{lo.x(),lo.y(),hi.x(),hi.y()})collision_diagnostic["predicted_box"].append(z);
        collision_diagnostic["ego_pose"]=path_json(Path{p})[0];
        collision_diagnostic["clearance_m"]=clearance;
        throw std::runtime_error("generic_predicted_occupancy_conflict");
      }
    }
  }
  void pose_check(Pose p,const Json::Value& occupancy,double now,double future,double clearance=.08) const {
    map_check(p);auto scene=prepare_observations(occupancy,now);
    check_prepared_obstacles(p,scene,future,clearance);
  }
  double check_emergency_map_coast(Pose pose,double speed,double servo) const {
    auto path=emergency_coast_path(pose,speed,servo);double reserve=INF;
    for(size_t j=0;j<path.size();++j) {
      Pose before=j?path[j-1]:path[j];int steps=std::max(1,int(std::ceil((path[j].xy()-before.xy()).norm()/.02)));
      for(int k=0;k<=steps;++k) {
        double f=double(k)/steps;
        Pose q{before.x+f*(path[j].x-before.x),before.y+f*(path[j].y-before.y),before.yaw+f*wrap(path[j].yaw-before.yaw)};
        reserve=std::min(reserve,body_map_reserve(a,q));
        if(reserve<0)throw std::runtime_error("generic_emergency_coast_map_conflict");
      }
    }
    return reserve;
  }
  std::pair<Pose,double> prediction_corridor(Pose reference,const Json::Value& occupancy,double now,double future) const {
    // Symmetric local tube, checked at both edges with the same forecast and
    // full body as the independent guard. No observed object is discarded.
    return prediction_corridor_prepared(reference,prepare_observations(occupancy,now),future);
  }
  std::pair<Pose,double> prediction_corridor_prepared(Pose reference,const PreparedScene& scene,double future) const {
    check_scene_identity(scene);
    Point normal(-std::sin(reference.yaw),std::cos(reference.yaw));
    // All tube probes at this stage share the exact observation/time. Form
    // its forecast once, rather than projecting/parsing every object for each
    // lateral trial. Map and oriented-body checks still run at every trial.
    struct ForecastRegion {Occupancy support;Point lo,hi;};
    std::vector<ForecastRegion> boxes;
    for(const auto& row:scene.objects) {
      Point lo,hi;forecast_region(row,scene,future,lo,hi);boxes.push_back({row.support,lo,hi});
    }
    auto admissible=[&](Pose centre,double radius) {
      try {
        for(int side:{-1,1}) {
          Pose q=centre;q.x+=side*radius*normal.x();q.y+=side*radius*normal.y();
          map_check(q);
          for(const auto& box:boxes)if(overlap_occupancy(q,box.support,box.lo,box.hi,.083))return false;
        }
        return true;
      }catch(const std::exception&){return false;}
    };
    if(admissible(reference,.20))return {reference,.20};
    // Use the nearest free tube when the nominal path lies against its
    // predicted boundary. This small stage-local normal correction avoids
    // requiring perfect tracking of a zero-width centreline. The optimizer
    // still enforces dynamics/rates; independent swept-body checks remain.
    Pose centre=reference;bool found=false;
    for(int step=0;step<=16 && !found;++step)for(int side:{-1,1}) {
      Pose q=reference;double offset=side*.005*step;
      q.x+=offset*normal.x();q.y+=offset*normal.y();
      if(admissible(q,.02)){centre=q;found=true;break;}
    }
    if(!found)throw std::runtime_error("joint_prediction_corridor_unavailable");
    double lo=0.,hi=.20;
    for(int k=0;k<7;++k){double mid=.5*(lo+hi);if(admissible(centre,mid))lo=mid;else hi=mid;}
    return {centre,lo};
  }
  double state_coast_speed_cap(Pose pose,double servo,double upper) const {
    if(!std::isfinite(upper+servo) || upper<running_floor || upper>2.)throw std::runtime_error("coast_speed_cap_input");
    auto admissible=[&](double v) {
      try {
        for(int side:{-1,0,1}) {
          Pose q=pose;q.yaw+=side*.03;
          if(!coast_reserve_feasible(q,v,clip(servo+side*.02,-.27,.27),.04))return false;
        }
        return true;
      }catch(const std::exception&){return false;}
    };
    if(!admissible(running_floor))throw std::runtime_error("state_coast_below_drivable_floor");
    if(admissible(upper))return upper;
    double lo=running_floor,hi=upper;
    for(int i=0;i<5;++i){double middle=(lo+hi)/2;if(admissible(middle))lo=middle;else hi=middle;}
    return lo;
  }
  bool coast_reserve_feasible(Pose pose,double speed,double servo,double required) const {
    // Same swept poses and reserve as check_emergency_map_coast. The EDT is
    // 1-Lipschitz on cell centres. Include a full cell diagonal for query
    // quantization and the map boundary (outside-map queries return zero).
    // A positive lower bound certifies every body-grid sample; ambiguous
    // poses use the original exact reserve. No clearance is approximated up.
    const double radius=std::hypot(.29,.155),cover=std::hypot(.58/12,.31/8)/2;
    auto path=emergency_coast_path(pose,speed,servo);
    for(size_t j=0;j<path.size();++j) {
      Pose before=j?path[j-1]:path[j];
      int steps=std::max(1,int(std::ceil((path[j].xy()-before.xy()).norm()/.02)));
      for(int k=0;k<=steps;++k) {
        double f=double(k)/steps;
        Pose q{before.x+f*(path[j].x-before.x),before.y+f*(path[j].y-before.y),before.yaw+f*wrap(path[j].yaw-before.yaw)};
        double boundary=std::min({q.x-a.ox,a.ox+a.width*a.res-q.x,q.y-a.oy,a.oy+a.height*a.res-q.y});
        double lower=std::min(a.clearance(q.xy())-std::sqrt(2.)*a.res,boundary)-radius-a.wall_margin-cover;
        if(lower>=required+.000001)continue;
        if(body_map_reserve(a,q)<required)return false;
      }
    }
    return true;
  }
  void check_prediction(const Path& prediction,const Json::Value& occupancy,double now) const {
    collision_diagnostic=Json::Value();
    if(prediction.empty() || prediction.size()>300)throw std::runtime_error("generic_prediction_shape");
    map_check(prediction.front());const auto scene=prepare_observations(occupancy,now);
    for(size_t i=0;i<prediction.size();++i) {
      Pose prev=i?prediction[i-1]:prediction[i];
      int count=std::max(1,int(std::ceil((prediction[i].xy()-prev.xy()).norm()/.02)));
      for(int k=0;k<=count;++k) {
        double f=double(k)/count;
        Pose q{prev.x+f*(prediction[i].x-prev.x),prev.y+f*(prediction[i].y-prev.y),prev.yaw+f*wrap(prediction[i].yaw-prev.yaw)};
        map_check(q);check_prepared_obstacles(q,scene,.05*(i?double(i-1)+f:0.));
      }
    }
  }
  // Build obstacle-relative Frenet free intervals, then minimize displacement,
  // slope, curvature and curvature change inside them. There is no prescribed
  // passing offset or hold/return station. Front/rear extent enters the forbidden
  // region through the entire ego footprint, not a target-centre crossing test.
  Candidate corridor_candidate(Planner& planner,const Path& base,const std::vector<double>& station,
                               const std::vector<Point>& cloud,const PreparedScene& prepared,
                               double source,double speed,double start,double slope,
                               double clearance,int side,double smooth,double extra,const Path& heading_hint=Path()) const {
    Candidate c;c.reason="generic_corridor_infeasible";
    if(base.size()<6)return c;
    std::vector<double> knots;std::vector<Pose> ref;
    for(size_t j=0;j<base.size();j+=2){knots.push_back(station[j]);ref.push_back(base[j]);}
    if(knots.back()<station.back()-1e-6){knots.push_back(station.back());ref.push_back(base.back());}
    const int n=knots.size();if(n<4)return c;
    std::vector<double> times;
    try {times=prepared.time_dependent?arrival_times(heading_hint.empty()?base:heading_hint,(heading_hint.empty()?base.front():heading_hint.front()),speed,1.6):std::vector<double>(base.size(),0.);}
    catch(const std::exception& e){c.reason=e.what();return c;}
    std::vector<Occupancy> obs;std::vector<bool> blocks;
    const double velocity=std::max(cap-.15,std::abs(speed));
    const double horizon=std::max(3.,velocity*velocity/(2*.6)+.5*velocity+1.);
    for(const auto& row:prepared.objects) {
      const auto& o=row.raw;Point lo,hi;bounds(o,source+.30,0,lo,hi);
      Point gap=(lo-base.front().xy()).cwiseMax(base.front().xy()-hi).cwiseMax(Point::Zero());
      if(gap.norm()>horizon)continue;
      bool blocking=false;
      for(size_t j=0;j<base.size() && station[j]<=horizon;++j)
        if(overlap_occupancy(base[j],o,lo,hi,clearance)){blocking=true;break;}
      obs.push_back(o);blocks.push_back(blocking);
    }
    const double solve_limit=max_offset+(heading_hint.empty()?.12:0.);
    Eigen::VectorXd lower=Eigen::VectorXd::Constant(n,-solve_limit+.003),upper=Eigen::VectorXd::Constant(n,solve_limit-.003);
    for(int i=0;i<n;++i) {
      Point t(std::cos(ref[i].yaw),std::sin(ref[i].yaw)),normal(-t.y(),t.x());
      if(knots[i]>horizon+.4)continue;
      for(size_t k=0;k<obs.size();++k) {
        Point lo,hi,fl,fh;bounds(obs[k],source+.30,0,lo,hi);
        // Same receding CV/uncertainty model as path validation. It is a
        // conservative union of now and anticipated arrival, not identity.
        const double arrival=times[std::min(size_t(2*i),times.size()-1)];
        const double limit=(obs[k].has_moved || obs[k].velocity.norm()>.08)?1.6:1.2;
        bounds(obs[k],source+.30,arrival<=limit?arrival:0.,fl,fh);
        // A moving obstacle occupies its predicted region at arrival, not
        // every position between now and arrival. Current geometry is still
        // checked independently by validation and the live controller guard.
        if(obs[k].has_moved || obs[k].velocity.norm()>.08){lo=fl;hi=fh;}
        else {lo=lo.cwiseMin(fl);hi=hi.cwiseMax(fh);}
        // Intersect all four separating-axis inequalities for normal
        // translations of the oriented ego body. This is the exact forbidden
        // d interval at this s and heading, rather than an axis-aligned
        // longitudinal/lateral bounding box that rounds every corner outward.
        double angle=heading_hint.empty()?ref[i].yaw:heading_hint[std::min(size_t(2*i),heading_hint.size()-1)].yaw;
        Point et(std::cos(angle),std::sin(angle)),en(-et.y(),et.x());
        const double gap=clearance+extra,hx=.29+gap/2,hy=.155+gap/2;
        Point delta=(lo+hi)/2-ref[i].xy(),half=(hi-lo)/2+Point::Constant(gap/2);
        double forbidden_lo=-INF,forbidden_hi=INF;bool intersects=true;
        for(Point axis:{Point(1,0),Point(0,1),et,en}) {
          double radius=hx*std::abs(et.dot(axis))+hy*std::abs(en.dot(axis))+half.x()*std::abs(axis.x())+half.y()*std::abs(axis.y());
          double relative=delta.dot(axis),coefficient=normal.dot(axis);
          if(std::abs(coefficient)<1e-8){if(std::abs(relative)>radius){intersects=false;break;}continue;}
          double low=(relative-radius)/coefficient,high=(relative+radius)/coefficient;
          if(low>high)std::swap(low,high);
          forbidden_lo=std::max(forbidden_lo,low);forbidden_hi=std::min(forbidden_hi,high);
        }
        if(!intersects || forbidden_lo>forbidden_hi)continue;
        const int pass=blocks[k]?side:((forbidden_lo+forbidden_hi)>=0?-1:1);
        if(pass>0)lower[i]=std::max(lower[i],forbidden_hi+.002);
        else upper[i]=std::min(upper[i],forbidden_lo-.002);
      }
    }
    // Keep smoothing from needlessly increasing the passing excursion. This
    // cap comes from this scene's required edge, not a fixed offset lattice.
    // Return monotonically to the reference side: do not create a second
    // opposite-side wiggle just to reduce the quadratic jerk cost.
    if(side<0)upper=upper.cwiseMin(Eigen::VectorXd::Constant(n,std::max(0.,start)+.002));
    else lower=lower.cwiseMax(Eigen::VectorXd::Constant(n,std::min(0.,start)-.002));
    const double edge=side<0?std::min(0.,upper.minCoeff()):std::max(0.,lower.maxCoeff());
    if(side<0)lower=lower.cwiseMax(Eigen::VectorXd::Constant(n,std::min(edge-.035,start-.003)));
    else upper=upper.cwiseMin(Eigen::VectorXd::Constant(n,std::max(edge+.035,start+.003)));
    c.terms["corridor_required_edge_m"]=edge;
    auto pin=[&](int i,double value){if(value<lower[i]-1e-8 || value>upper[i]+1e-8)return false;lower[i]=upper[i]=value;return true;};
    if(!pin(0,start) || !pin(n-1,0) || !pin(n-2,0))return c;
    Eigen::MatrixXd h=Eigen::MatrixXd::Identity(n,n);
    auto derivative=[&](int order,double weight) {
      for(int i=0;i+order<n;++i) {
        const double step=(knots[i+order]-knots[i])/order;
        Eigen::VectorXd row=Eigen::VectorXd::Zero(n);
        if(order==1){row[i]=-1;row[i+1]=1;}
        if(order==2){row[i]=1;row[i+1]=-2;row[i+2]=1;}
        if(order==3){row[i]=-1;row[i+1]=3;row[i+2]=-3;row[i+3]=1;}
        row/=std::pow(step,order);h.noalias()+=weight*(row*row.transpose());
      }
    };
    derivative(1,.12);derivative(2,.08);derivative(3,smooth);
    Eigen::VectorXd linear=Eigen::VectorXd::Zero(n),boundary=Eigen::VectorXd::Zero(n);
    boundary[0]=-1;boundary[1]=1;
    double step=knots[1]-knots[0],weight=.16/std::pow(step,4);
    h.noalias()+=weight*(boundary*boundary.transpose());linear-=weight*slope*step*boundary;
    Eigen::VectorXd values;int iterations=0;
    if(!corridor_box_qp(h,linear,lower,upper,values,iterations))return c;
    CorridorSpline spline(knots,values,slope);
    double peak=0,integral=0,jerk=0;bool within_offset=true;
    for(size_t i=0;i<base.size();++i) {
      auto q=spline.at(station[i]);
      if(!std::isfinite(q[0]+q[1]+q[2]))return c;
      if(std::abs(q[0])>max_offset+1e-6)within_offset=false;
      c.path.push_back({base[i].x-std::sin(base[i].yaw)*q[0],base[i].y+std::cos(base[i].yaw)*q[0],base[i].yaw});
      if(std::abs(q[0])>std::abs(peak)){peak=q[0];c.transition=station[i];}
      if(i){integral+=q[0]*q[0]*(station[i]-station[i-1]);jerk+=q[2]*q[2]*(station[i]-station[i-1]);}
    }
    for(size_t i=0;i<c.path.size();++i) {
      size_t lo=i?i-1:i,hi=std::min(i+1,c.path.size()-1);Point delta=c.path[hi].xy()-c.path[lo].xy();
      c.path[i].yaw=std::atan2(delta.y(),delta.x());
    }
    c.offset=peak;c.ret=station.back()-c.transition;c.jerk=jerk;planner.distances(c,cloud);c.k=curvature(c.path);
    double dense_curvature=0;
    for(size_t j=0;j+2<c.path.size();++j) {
      Point u=c.path[j+1].xy()-c.path[j].xy(),v=c.path[j+2].xy()-c.path[j].xy();
      double denominator=u.norm()*v.norm()*(v-u).norm();
      if(denominator>1e-9)dense_curvature=std::max(dense_curvature,2*std::abs(u.x()*v.y()-u.y()*v.x())/denominator);
    }
    c.terms["dense_maximum_curvature_1pm"]=dense_curvature;
    c.valid=within_offset && dense_curvature<=std::tan(.27)/.25+.03 &&c.k<=std::max(std::tan(.27)/.25+.03,curvature(base)+.03) &&
      std::abs(wrap(c.path.front().yaw-base.front().yaw)-std::atan(slope))<.10 &&
      std::abs(wrap(c.path.back().yaw-base.back().yaw))<.10;
    c.reason=c.valid?"generic_adaptive_corridor":(!within_offset?"generic_corridor_offset_limit":"generic_corridor_curvature_or_heading");
    c.score=integral+.35*c.k+.003*jerk+1.5*std::abs(peak-planner.offset);
    c.terms["planner_geometry"]="obstacle_relative_frenet_corridor";
    c.terms["corridor_side"]=side;c.terms["corridor_qp_iterations"]=iterations;
    c.terms["corridor_extra_reserve_m"]=extra;c.terms["corridor_smoothness"]=smooth;
    c.terms["lateral_squared_integral"]=integral;
    c.terms["corridor_stations"]=Json::Value(Json::arrayValue);
    for(int i=0;i<n;++i){Json::Value q(Json::arrayValue);q.append(knots[i]);q.append(lower[i]);q.append(upper[i]);q.append(values[i]);c.terms["corridor_stations"].append(q);}
    return c;
  }
  std::vector<double> arrival_times(const Path& path,Pose pose,double speed,double horizon=INF,double reference_cap=INF) const {
    Geometry geometry(path,pose,map_speed_feasibility?&a:nullptr,running_floor);
    std::vector<double> times(path.size(),0.);double elapsed=0.,velocity=std::max(0.,std::abs(speed));
    for(size_t i=1;i<path.size();++i) {
      if(elapsed>horizon){times[i]=elapsed;continue;}
      double distance=geometry.arc[i]-std::max(geometry.arc[i-1],geometry.nearest_s);
      if(distance<=0)continue;
      // Nominal bounded-acceleration schedule, not an exact actuator/MPC
      // rollout. Revalidated every scan; actual controller prediction remains
      // an independent admission check.
      double target=std::min(reference_cap,geometry.target(geometry.arc[i],1e6,cap-.15));
      if((path.back().xy()-a.route.back().xy()).norm()<.02) {
        double remaining=std::max(0.,geometry.arc.back()-geometry.arc[i]-.25);
        double terminal=std::max(running_floor,std::sqrt(std::pow(.55*.30,2)+2*.55*remaining)-.55*.30);
        target=std::min(target,terminal);
      }
      double next=target>=velocity?std::min(target,std::sqrt(velocity*velocity+2.*distance))
                                  :std::max(target,std::sqrt(std::max(0.,velocity*velocity-1.1*distance)));
      elapsed+=2*distance/std::max(.05,velocity+next);velocity=next;times[i]=elapsed;
    }
    return times;
  }
  Candidate plan(Planner& planner,Pose pose,double source,double speed,bool allow_search=true) {
    return plan_occupancies(planner,pose,source,speed,snapshot(),preferred_gap,allow_search);
  }
  Candidate plan_occupancies(Planner& planner,Pose pose,double source,double speed,const Json::Value& input_occupancy,double clearance=.14,bool allow_search=true,double search_deadline=-1) {
    Json::Value scene_occupancy=input_occupancy;
    if(stationary_scene && scene_occupancy.isArray())for(auto& o:scene_occupancy){o["vx"]=0.;o["vy"]=0.;o["has_moved"]=false;o["fixed_world"]=true;}
    const Json::Value& occupancy=stationary_scene?scene_occupancy:input_occupancy;
    if(search_deadline<0)search_deadline=mono()+.20;
    const double stage_deadline=clearance>.080001?std::min(search_deadline,mono()+.10):search_deadline;
    if(!occupancy.isArray() || occupancy.size()>128)throw std::runtime_error("occupancy_missing");
    planner.nearest(pose);planner.candidates.clear();
    Path base(a.route.begin()+planner.progress,a.route.end());
    std::vector<double> s{0};for(size_t i=1;i<base.size();++i)s.push_back(s.back()+(base[i].xy()-base[i-1].xy()).norm());
    if(base.size()<2){Candidate c;c.valid=true;c.reason="generic_terminal";c.path=base;return c;}
    Pose b=base.front();double start=(pose.xy()-b.xy()).dot(Point(-std::sin(b.yaw),std::cos(b.yaw)));
    double slope=std::tan(clip(wrap(pose.yaw-b.yaw),-.6,.6));
    // All measured points, including nonconfirmed and small clusters, constrain
    // geometry. Predictive body checks below carry unseen extent and motion.
    std::vector<Point> cloud;
    const auto prepared=prepare_observations(occupancy,source+.30);
    bool has_moving_occupancy=false;for(const auto& row:prepared.objects)has_moving_occupancy|=!row.is_static;
    // Receding local obstacle horizon, longer than a conservative complete
    // stop plus transport and replanning allowance. All occupancies remain
    // available to the independent controller prediction/coast guard.
    const double planning_speed=std::max(cap-.15,std::abs(speed));
    const double local_distance=std::max(3.0,planning_speed*planning_speed/(2*.6)+.5*planning_speed+1.0);
    for(const auto& row:prepared.objects) {
      const auto& o=row.raw;Point lo,hi;bounds(o,source,0,lo,hi);
      Point gap=(lo-pose.xy()).cwiseMax(pose.xy()-hi).cwiseMax(Point::Zero());
      if(gap.norm()<=local_distance && (o.hazard || (!o.has_moved && o.velocity.norm()<=.08)))
        for(Point p:{lo,hi,Point(lo.x(),hi.y()),Point(hi.x(),lo.y()),Point((lo+hi)/2)})cloud.push_back(p);

    }
    auto validate=[&](Candidate c,bool held=false) {
      const double required_gap=held?.08:clearance;
      if(!c.valid)return c;
      try {
        const double reference_cap=c.terms.get("following_speed_cap_mps",2.).asDouble();
        struct Sample {Pose pose;double along,f;size_t index,nearest;};
        std::vector<Sample> samples;samples.reserve(4*c.path.size());
        double distance=0;size_t nearest=0;double best=INF;
        for(size_t j=0;j<c.path.size();++j)if((c.path[j].xy()-pose.xy()).norm()<best){best=(c.path[j].xy()-pose.xy()).norm();nearest=j;}
        if(best>.18)throw std::runtime_error("generic_path_deviation");
        for(size_t j=nearest;j<c.path.size();++j) {
          Pose previous=j>nearest?c.path[j-1]:c.path[j];
          const double segment=(c.path[j].xy()-previous.xy()).norm();
          const int steps=std::max(1,int(std::ceil(segment/.02)));
          for(int k=0;k<=steps;++k) {
            const double f=double(k)/steps;
            Pose q{previous.x+f*(c.path[j].x-previous.x),previous.y+f*(c.path[j].y-previous.y),previous.yaw+f*wrap(c.path[j].yaw-previous.yaw)};
            const double along=distance+f*segment;
            const double reserve=body_map_reserve(a,q);
            if(reserve<0)throw std::runtime_error("generic_full_body_map");
            if(!held && along>=.4 && reserve<.06)throw std::runtime_error("generic_tracking_map_reserve");
            if(along<=local_distance)check_prepared_obstacles(q,prepared,0,required_gap,SceneGroup::Static);
            samples.push_back({q,along,f,j,nearest});
          }
          distance+=segment;
        }
        const auto times=arrival_times(c.path,pose,speed,1.6,reference_cap);
        for(const auto& sample:samples) {
          size_t j=sample.index;const double t=j>sample.nearest?times[j-1]+sample.f*(times[j]-times[j-1]):0.;
          if(t<=1.2)check_prepared_obstacles(sample.pose,prepared,t,required_gap,SceneGroup::All,sample.along<=local_distance);
          else if(t<=1.6)check_prepared_obstacles(sample.pose,prepared,t,required_gap,SceneGroup::Moving);
        }
      }catch(const std::exception& e){c.valid=false;c.reason=e.what();
        if(c.reason=="generic_predicted_occupancy_conflict")c.terms["collision_diagnostic"]=collision_diagnostic;
      }
      c.terms["generic_required_body_gap_m"]=required_gap;
      c.terms["generic_held_path"]=held;
      return c;
    };
    Candidate best;best.reason="generic_no_feasible_path";
    double follow_cap=INF,pass_required=0.;
    const double cruise=cap-.15;
    Point forward_axis(std::cos(b.yaw),std::sin(b.yaw));
    for(const auto& row:prepared.objects){
      const auto& o=row.raw;if(o.hazard || !o.has_moved)continue;
      Point delta=.5*(o.lo+o.hi)-pose.xy();
      double forward=delta.dot(forward_axis),lateral=std::abs(delta.dot(Point(-forward_axis.y(),forward_axis.x())));
      double lead_speed=o.velocity.dot(forward_axis);
      if(forward<=0 || forward>local_distance || lateral>.55 || lead_speed<.45)continue;
      double required=cruise*(forward+.60)/std::max(.05,cruise-lead_speed)+1.8+cruise*cruise/(2*.6)+.30*cruise;
      if(s.back()<required){follow_cap=std::min(follow_cap,std::max(running_floor,lead_speed));pass_required=std::max(pass_required,required);}
    }
    // An open route also needs room to return AND stop. If that complete
    // maneuver cannot fit, establish following early, while braking is still
    // feasible; do not wait for the controller's collision guard to intervene.
    if(std::isfinite(follow_cap) && !incumbent_terms.isMember("following_speed_cap_mps")){
      for(double length:{2.4,1.8,1.2}){
        auto c=planner.make(0,length,1.8,start,slope,0,base,s,cloud,-INF);
        c.terms["following_speed_cap_mps"]=follow_cap;
        c.terms["complete_pass_required_m"]=pass_required;
        c.terms["route_remaining_m"]=s.back();
        c.terms["planner_geometry"]="reference_following_insufficient_pass_runway";
        c=validate(c);planner.candidates.push_back(c);
        if(c.valid){best=c;break;}
      }
    }
    if(!best.valid && !incumbent.empty()) {
      Candidate c;c.valid=true;c.reason="generic_revalidated_path";c.path=incumbent;c.offset=incumbent_offset;c.transition=incumbent_entry;c.ret=incumbent_return;c.score=incumbent_score;c.terms=incumbent_terms;
      // Trim passed segments before validation; never bypass a new obstacle.
      size_t near=0;double d=INF;for(size_t k=0;k<c.path.size();++k)if((c.path[k].xy()-pose.xy()).norm()<d){near=k;d=(c.path[k].xy()-pose.xy()).norm();}
      c.path.erase(c.path.begin(),c.path.begin()+near);planner.distances(c,cloud);c.k=curvature(c.path);c=validate(c,true);planner.candidates.push_back(c);
      if(c.valid && c.path.size()>=2)best=c;
    }
    // Retain a valid selected path to prevent moving the departure/return
    // points every scan. Replan either side only when it becomes invalid.
    if(!best.valid && allow_search) {
      auto follow=planner.make(0,1.8,1.8,start,slope,0,base,s,cloud,-INF);
      follow=validate(follow);planner.candidates.push_back(follow);
      if(follow.valid)best=follow;
      // Smooth feasible seeds are computed from obstacle edges, never a
      // fixed lateral-offset list. They also supply feasible alternatives when
      // a convex heading linearization fails in a close-start configuration.
      // Full-body overlap defines the trailing release, including ego rear.
      double release=-INF,first_blocking=INF,right_edge=0,left_edge=0;bool blocking=false;
      // This is a timing heuristic on the ORIGINAL reference frame. Detour
      // offset is not a reference-tracking error; actual candidate tracking
      // and swept-body checks still use the real pose below.
      const auto reference_times=prepared.time_dependent?arrival_times(base,base.front(),speed,1.6):std::vector<double>(base.size(),0.);
      for(const auto& row:prepared.objects) {
        const auto& o=row.raw;Point measured_lo,measured_hi;bounds(o,source+.30,0,measured_lo,measured_hi);
        const double horizon=(o.has_moved || o.velocity.norm()>.08)?1.6:1.2;
        for(size_t j=0;j<base.size() && s[j]<=local_distance;++j) {
          Point lo=measured_lo,hi=measured_hi;
          if(reference_times[j]<=horizon) {
            Point future_lo,future_hi;
            // Include the same predictive support used by validation, plus a
            // small arrival-time allowance for the longer outbound path.
            bounds(o,source+.30,std::min(horizon,reference_times[j]+.20),future_lo,future_hi);
            lo=lo.cwiseMin(future_lo);hi=hi.cwiseMax(future_hi);
          }
          if(overlap_occupancy(base[j],o,lo,hi,clearance)) {
            blocking=true;release=std::max(release,s[j]);first_blocking=std::min(first_blocking,s[j]);
            Point normal(-std::sin(base[j].yaw),std::cos(base[j].yaw));
            for(Point corner:{lo,hi,Point(lo.x(),hi.y()),Point(hi.x(),lo.y())}) {
              double edge=(corner-base[j].xy()).dot(normal);
              right_edge=std::min(right_edge,edge-.155-clearance);
              left_edge=std::max(left_edge,edge+.155+clearance);
            }
          }
        }
      }
      const double seed_deadline=!has_moving_occupancy?stage_deadline:std::min(stage_deadline,mono()+.35*std::max(0.,stage_deadline-mono()));
      std::vector<double> seed_offsets;
      const std::array<int,2> side_order=std::abs(start)>.10?(start>0?std::array<int,2>{{1,-1}}:std::array<int,2>{{-1,1}}):(std::abs(right_edge)<=std::abs(left_edge)?std::array<int,2>{{-1,1}}:std::array<int,2>{{1,-1}});
      if(blocking && allow_envelope_seeds)for(int side:side_order)for(double reserve:{seed_reserve,seed_reserve+.04,seed_reserve+.08,seed_reserve+.13}) {
        if(best.valid || mono()>=seed_deadline)break;
        const double edge=side<0?right_edge:left_edge;
        if(std::abs(edge)>max_offset)continue;
        const double d=side*std::min(max_offset,std::abs(edge)+reserve);
        if(std::any_of(seed_offsets.begin(),seed_offsets.end(),[&](double previous){return std::abs(previous-d)<1e-6;}))continue;
        seed_offsets.push_back(d);
        const double approach=clip(first_blocking-.15,.9,1.8);
        std::vector<double> entries{approach};
        for(double length:{1.8,1.5,2.1})
          if(std::abs(length-approach)>.01)entries.push_back(length);
        for(double entry:entries)for(double back:{1.8,2.1}) {
          if(best.valid || mono()>=seed_deadline)break;
          // Finish the outbound transition just before the first swept-body
          // overlap, rather than departing from ego as soon as an object is seen.
          // Preserve initial offset while settling heading during the wait.
          const double delay=std::max(0.,first_blocking-.15-entry);
          if(std::max(delay+entry,release+.10)+back>s.back())continue;
          auto c=planner.make(d,entry,back,start,slope,0,base,s,cloud,release-.15,delay);
          // The envelope seed is merely a candidate; dense curvature and
          // exact swept-body checks are identical to the optimized paths.
          double dense=0;
          for(size_t j=0;j+2<c.path.size();++j) {
            Point u=c.path[j+1].xy()-c.path[j].xy(),v=c.path[j+2].xy()-c.path[j].xy();
            double den=u.norm()*v.norm()*(v-u).norm();
            if(den>1e-9)dense=std::max(dense,2*std::abs(u.x()*v.y()-u.y()*v.x())/den);
          }
          if(dense>std::tan(.27)/.25+.03){c.valid=false;c.reason="generic_corridor_curvature_or_heading";}
          c.terms["planner_geometry"]="obstacle_relative_envelope_seed";
          c.terms["corridor_required_edge_m"]=edge;c.terms["corridor_extra_reserve_m"]=reserve;
          c.terms["dense_maximum_curvature_1pm"]=dense;c.terms["return_start_s"]=std::max(delay+entry,release+.10);
          c.terms["departure_s"]=delay;c.terms["first_blocking_s"]=first_blocking;
          double area=0;
          for(size_t j=1;j<c.path.size();++j) {
            Point normal(-std::sin(base[j].yaw),std::cos(base[j].yaw));
            double offset=(c.path[j].xy()-base[j].xy()).dot(normal);area+=offset*offset*(s[j]-s[j-1]);
          }
          c.score=area+.35*c.k+.003*c.jerk+1.5*std::abs(d-planner.offset);
          c=validate(c);planner.candidates.push_back(c);
          if(c.valid && (!best.valid || c.score<best.score))best=c;
        }
      }
      // Both homotopies are solved against measured obstacle envelopes.
      // Smoothness/reserve change feasibility, never prescribe the pass d.
      if(!best.valid)for(int side:side_order)for(double smooth:{.003,.012,.04})for(double extra:{0.,.025}) {
        if(mono()>=stage_deadline)break;
        auto c=corridor_candidate(planner,base,s,cloud,prepared,source,speed,start,slope,clearance,side,smooth,extra);
        // Two sequential convex refinements account for the yaw caused by
        // the previous smooth solution. Only a fresh exact body check can
        // accept an iterate; convergence alone never grants motion.
        for(int refinement=0;refinement<3;++refinement) {
          Path hint=c.path;c=validate(c);planner.candidates.push_back(c);
          if(c.valid && (!best.valid || c.score<best.score))best=c;
          if(hint.empty() || refinement==2 || mono()>=stage_deadline)break;
          c=corridor_candidate(planner,base,s,cloud,prepared,source,speed,start,slope,clearance,side,smooth,extra,hint);
          c.terms["corridor_heading_refinement"]=refinement+1;
        }
      }
    }
    // A short/narrow section may admit following but no complete pass-and-
    // return. Try a lower-speed return to the reference, not an ever-wider
    // mandatory pass. This is generic longitudinal following, with no ID lock.
    if(!best.valid && allow_search && mono()<search_deadline){
      double following=INF;Point tangent(std::cos(b.yaw),std::sin(b.yaw));
      for(const auto& row:prepared.objects){
        const auto& o=row.raw;if(o.hazard || !o.has_moved)continue;
        Point delta=.5*(o.lo+o.hi)-pose.xy();double forward=delta.dot(tangent);
        double longitudinal=o.velocity.dot(tangent);
        if(forward>0 && forward<local_distance && longitudinal>=.45)
          following=std::min(following,std::max(running_floor,longitudinal));
      }
      if(std::isfinite(following))for(double length:{2.4,1.8,1.2}){
        auto c=planner.make(0,length,1.8,start,slope,0,base,s,cloud,-INF);
        c.terms["following_speed_cap_mps"]=following;
        c.terms["planner_geometry"]="reference_return_with_dynamic_following";
        c=validate(c);planner.candidates.push_back(c);
        if(c.valid){c.reason="generic_following_fallback";best=c;break;}
      }
    }
    if(!best.valid && allow_search && clearance>.080001)
      return plan_occupancies(planner,pose,source,speed,occupancy,.08,true,search_deadline);
    best.terms["search_budget_exhausted"]=!best.valid && mono()>=search_deadline;
    if(!best.valid && mono()>=search_deadline)best.reason="generic_search_budget_exhausted";
    best.terms["arrival_time_policy"]="shared_controller_geometry_speed_limits";
    if(best.valid) {
      if(incumbent.empty() || std::abs(best.offset-incumbent_offset)>1e-6)++planner.switches;
      incumbent=best.path;incumbent_offset=best.offset;incumbent_entry=best.transition;incumbent_return=best.ret;incumbent_score=best.score;incumbent_terms=best.terms;
      planner.offset=best.offset;planner.mode=best.terms.isMember("following_speed_cap_mps")?"FOLLOWING_OBSTACLE":(std::abs(best.offset)>.01?"AVOIDING":"FOLLOWING");
      planner.switch_reason=best.reason;best.reason="generic_validated";
    } else {planner.mode="BLOCKED";incumbent.clear();}
    return best;
  }
};
} // namespace frenet
