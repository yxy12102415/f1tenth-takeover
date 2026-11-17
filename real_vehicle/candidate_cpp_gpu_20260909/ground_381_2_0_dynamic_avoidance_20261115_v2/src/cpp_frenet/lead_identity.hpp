#pragma once
#include "tracked_occupancy.hpp"
namespace frenet {
// Controlled scene: only the continuously associated lead may move. All raw
// geometry stays in the obstacle stream. Identity loss alone never commands a
// stop, and unmatched support near the last lead remains conservative unknown.
class LeadIdentity {
 const Assets& a;
 struct Projection {double s,d;Point t;};
 struct Candidate {std::vector<int> indices;Point center;Projection p;double error,cost;std::string view;};
 struct Sample {double stamp,s,d;};
 std::vector<std::pair<std::string,std::vector<Point>>> templates;
 bool active=false,locked=false,aligned=true;
 double length=.23,width=.30,min_ahead=.6,max_ahead=3.3,max_d=.65,shape_limit=.06;
 double stamp=-INF,s=0,d=0,v=0,vd=0,pending_stamp=-INF,pending_seen=-INF,last_input=-INF;
 int pending_count=0;Point pending_center=Point::Zero();
 std::deque<Sample> history;Json::Value diagnostic;
 Projection project(Point q) const {
  double best=INF;Projection p{};
  for(size_t j=1;j<a.route.size();++j){Point e=a.route[j].xy()-a.route[j-1].xy();double f=clip((q-a.route[j-1].xy()).dot(e)/e.squaredNorm(),0.,1.);Point foot=a.route[j-1].xy()+f*e;double dist=(q-foot).squaredNorm();if(dist<best){best=dist;Point t=e.normalized();p={a.arc[j-1]+f*e.norm(),(q-foot).dot(Point(-t.y(),t.x())),t};}}
  return p;
 }
 Point at(double station,double offset) const {
  station=clip(station,0.,a.arc.back());size_t h=std::min(a.route.size()-1,size_t(std::upper_bound(a.arc.begin(),a.arc.end(),station)-a.arc.begin()));size_t l=h?h-1:0;Point t=(a.route[h].xy()-a.route[l].xy()).normalized();double f=h==l?0.:(station-a.arc[l])/(a.arc[h]-a.arc[l]);return a.route[l].xy()+f*(a.route[h].xy()-a.route[l].xy())+offset*Point(-t.y(),t.x());
 }
 static Point low(const Json::Value& o){return {o["xmin"].asDouble(),o["ymin"].asDouble()};}
 static Point high(const Json::Value& o){return {o["xmax"].asDouble(),o["ymax"].asDouble()};}
 static std::vector<Point> principal(std::vector<Point> p){
  if(p.size()<3)return p;std::vector<bool> used(p.size());std::vector<int> best;
  for(size_t i=0;i<p.size();++i)if(!used[i]){std::vector<int> group{int(i)};used[i]=true;for(size_t k=0;k<group.size();++k)for(size_t j=0;j<p.size();++j)if(!used[j]&&(p[j]-p[group[k]]).squaredNorm()<=.085*.085){used[j]=true;group.push_back(j);}if(group.size()>best.size())best=group;}
  std::vector<Point> out;for(auto k:best)out.push_back(p[k]);return out;
 }
 std::pair<double,std::string> match(std::vector<Point> points) const {
  points=principal(std::move(points));if(points.size()<6)return {INF,""};Point mean=Point::Zero();for(auto p:points)mean+=p;mean/=points.size();for(auto& p:points)p-=mean;
  std::vector<Point> small;for(size_t i=0;i<points.size();i+=std::max(size_t(1),points.size()/48))small.push_back(points[i]);points=std::move(small);
  std::pair<double,std::string> best{INF,""};
  for(const auto& entry:templates)for(int r=0;r<16;++r){double angle=r*3.141592653589793/8,c=std::cos(angle),ss=std::sin(angle);std::vector<Point> q;for(auto p:entry.second)q.emplace_back(c*p.x()-ss*p.y(),ss*p.x()+c*p.y());std::vector<double> forward(points.size(),INF),reverse(q.size(),INF);for(size_t i=0;i<points.size();++i)for(size_t j=0;j<q.size();++j){double dist=(points[i]-q[j]).norm();forward[i]=std::min(forward[i],dist);reverse[j]=std::min(reverse[j],dist);}std::sort(forward.begin(),forward.end());std::sort(reverse.begin(),reverse.end());size_t nf=std::max(size_t(3),size_t(.8*forward.size())),nr=std::max(size_t(3),size_t(.8*reverse.size()));double total=0.;for(size_t k=0;k<nf;++k)total+=forward[k];for(size_t k=0;k<nr;++k)total+=reverse[k];double error=total/(nf+nr);if(error<best.first)best={error,entry.first};}
  return best;
 }
public:
 explicit LeadIdentity(const Assets& assets,const Json::Value& config):a(assets){
  active=config.get("enabled",false).asBool();if(!active)return;
  if(config["scene_assumption"]!="only_lead_vehicle_may_move")throw std::runtime_error("lead_scene_scope_missing");
  length=config["operator_body_length_m"].asDouble();width=config["operator_body_width_m"].asDouble();
  if(length<.1||length>.8||width<.1||width>.6)throw std::runtime_error("lead_reported_dimensions");
  min_ahead=config.get("acquisition_s_ahead_min_m",.6).asDouble();max_ahead=config.get("acquisition_s_ahead_max_m",3.3).asDouble();max_d=config.get("acquisition_abs_d_max_m",.65).asDouble();shape_limit=config.get("shape_error_limit_m",.06).asDouble();
  if(min_ahead<.3||max_ahead>4.||max_ahead<=min_ahead||max_d<=0||max_d>.8||shape_limit<=0||shape_limit>.08)throw std::runtime_error("lead_acquisition_bounds");
  if(config["templates"].size()!=8)throw std::runtime_error("lead_requires_eight_templates");
  for(const auto& row:config["templates"]){std::vector<Point> points;for(const auto& p:row["centered_xy_m"]){if(p.size()!=2||!std::isfinite(p[0].asDouble()+p[1].asDouble()))throw std::runtime_error("lead_template_schema");points.emplace_back(p[0].asDouble(),p[1].asDouble());}if(points.size()<6||points.size()>128)throw std::runtime_error("lead_template_size");templates.emplace_back(row["name"].asString(),points);}
 }
 Json::Value status()const{return diagnostic;}
 Json::Value apply(Json::Value objects,const std::vector<Point>& cloud,Pose ego,double source){
  if(!active)return objects;if(!objects.isArray()||objects.size()>128||!std::isfinite(source)||source<=last_input)throw std::runtime_error("lead_observation_time_or_capacity");last_input=source;
  const double es=project(ego.xy()).s,limit=std::hypot(std::min(.8,length+.22),std::min(.75,width+.18));
  Point expected=locked?at(s+v*std::max(0.,source-stamp),d):Point::Zero();
  std::vector<std::vector<int>> sets;std::vector<int> eligible;std::vector<bool> possible(objects.size());
  for(size_t k=0;k<objects.size();++k){const auto& o=objects[int(k)];Point lo=low(o),hi=high(o),center=.5*(lo+hi);auto p=project(center);double size=(hi-lo).maxCoeff();bool geometry=size>=.06&&size<=limit;bool gate=!locked?(p.s-es>=min_ahead&&p.s-es<=max_ahead&&std::abs(p.d)<=max_d):((center-expected).norm()<.24+.8*std::min(std::max(0.,source-stamp),.8));possible[k]=geometry&&gate;
   if(possible[k]&&!o.get("visibility_hazard",false).asBool()&&o.get("memory_state","observed")=="observed"&&o["hits"].asInt()>=4){eligible.push_back(k);sets.push_back({int(k)});}}
  for(size_t i=0;i<std::min(size_t(12),eligible.size());++i)for(size_t j=i+1;j<std::min(size_t(12),eligible.size());++j){int u=eligible[i],w=eligible[j];Point lo=low(objects[u]).cwiseMin(low(objects[w])),hi=high(objects[u]).cwiseMax(high(objects[w]));Point gap=(low(objects[u])-high(objects[w])).cwiseMax(low(objects[w])-high(objects[u])).cwiseMax(Point::Zero());if(gap.norm()<=.2&&(hi-lo).maxCoeff()<=limit)sets.push_back({u,w});}
  std::vector<Candidate> candidates;
  for(const auto& indices:sets){Point lo=low(objects[indices[0]]),hi=high(objects[indices[0]]);for(int k:indices){lo=lo.cwiseMin(low(objects[int(k)]));hi=hi.cwiseMax(high(objects[int(k)]));}std::vector<Point> support;for(auto point:cloud)if((point.array()>=(lo.array()-.02)).all()&&(point.array()<=(hi.array()+.02)).all())support.push_back(point);auto shape=match(support);if(shape.first>shape_limit)continue;Point center=.5*(lo+hi);auto p=project(center);double cost=shape.first+(locked?.35*(center-expected).norm():0.);candidates.push_back({indices,center,p,shape.first,cost,shape.second});}
  std::sort(candidates.begin(),candidates.end(),[](const Candidate& u,const Candidate& w){return u.cost<w.cost;});
  int selected=candidates.empty()?-1:0;std::string reason="no_matching_support";
  if(selected==0){reason="candidate";for(size_t k=1;k<candidates.size();++k){bool shared=false;for(auto i:candidates[0].indices)for(auto j:candidates[k].indices)shared|=i==j;if(!shared){if(candidates[k].cost-candidates[0].cost<.025){selected=-1;reason="ambiguous_candidates";}break;}}}
  if(!locked&&selected==0){const auto& c=candidates[0];if(source-pending_seen>.25||(c.center-pending_center).norm()>.18){pending_stamp=source;pending_count=1;}else ++pending_count;pending_center=c.center;pending_seen=source;if(pending_count<3||source-pending_stamp<.15){selected=-1;reason="confirming_candidate";}}
  std::vector<bool> chosen(objects.size());
  diagnostic=Json::Value();diagnostic["enabled"]=true;diagnostic["identity_loss_requests_stop"]=false;diagnostic["lead_length_m"]=length;diagnostic["lead_width_m"]=width;
  if(selected==0){const auto& c=candidates[0];history.push_back({source,c.p.s,c.p.d});while(history.size()>1&&source-history.front().stamp>.6)history.pop_front();double speed=0.,lateral=0.;if(source-history.front().stamp>=.25){double dt=source-history.front().stamp;speed=(c.p.s-history.front().s)/dt;lateral=(c.p.d-history.front().d)/dt;}aligned=std::abs(lateral)<=.15&&speed>=-.08&&speed<=1.;if(locked)speed=.65*v+.35*speed;stamp=source;s=c.p.s;d=c.p.d;v=speed;vd=lateral;locked=true;reason="associated";diagnostic["shape_error_m"]=c.error;diagnostic["matched_template"]=c.view;for(int k:c.indices){chosen[k]=true;diagnostic["component_ids"].append(objects[int(k)]["id"]);}}
  else if(locked)reason=source-stamp<=.6?"temporarily_unobserved":"identity_unresolved";
  diagnostic["reason"]=reason;diagnostic["locked"]=locked;diagnostic["route_aligned"]=aligned;
  if(locked){diagnostic["s_m"]=s;diagnostic["d_m"]=d;diagnostic["speed_mps"]=v;diagnostic["lateral_speed_mps"]=vd;diagnostic["age_s"]=source-stamp;if(aligned&&source-stamp<=.6)for(int k=0;k<=16;++k){double future=.1*k;Point p=at(s+std::max(0.,v)*(source-stamp+future),d);Json::Value row(Json::arrayValue);row.append(future);row.append(p.x());row.append(p.y());diagnostic["predicted_route_positions"].append(row);}}
  for(size_t k=0;k<objects.size();++k){auto& o=objects[int(k)];if(chosen[k]){o["semantic_role"]="lead_vehicle";o["lead_route_aligned"]=aligned;if(aligned&&o.get("has_moved",false).asBool()&&!o.get("fixed_world",false).asBool()){auto p=project(.5*(low(o)+high(o)));o["velocity_before_route_prior"].append(o["vx"]);o["velocity_before_route_prior"].append(o["vy"]);o["vx"]=std::max(0.,v)*p.t.x();o["vy"]=std::max(0.,v)*p.t.y();}continue;}
   Point center=.5*(low(o)+high(o));bool unresolved=(!locked&&possible[k])||(locked&&selected<0&&(center-at(s+std::max(0.,v)*std::min(source-stamp,.6),d)).norm()<.35+.8*std::min(source-stamp,.8));
   if(unresolved){o["semantic_role"]="unresolved_near_lead";continue;}
   o["semantic_role"]="declared_static_background";o["original_velocity"].append(o["vx"]);o["original_velocity"].append(o["vy"]);o["vx"]=0.;o["vy"]=0.;o["has_moved"]=false;o["fixed_world"]=true;o["stationary_history"]=true;
  }
  return objects;
 }
};
} // namespace frenet
