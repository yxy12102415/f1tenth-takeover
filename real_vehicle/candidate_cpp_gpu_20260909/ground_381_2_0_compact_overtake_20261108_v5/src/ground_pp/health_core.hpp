#pragma once
#include <cmath>
#include <string>
#include <jsoncpp/json/json.h>
namespace health {
struct Sample {double stamp,erpm,motor,input,fet,voltage,duty,fault;};
inline bool parse(const Json::Value&j,Sample&s){
  const char* keys[]={"stamp","erpm","motor_current","input_current","fet_temp","voltage","duty_cycle","fault"};
  double* fields[]={&s.stamp,&s.erpm,&s.motor,&s.input,&s.fet,&s.voltage,&s.duty,&s.fault};
  for(int i=0;i<8;++i){if(!j[keys[i]].isNumeric()||!std::isfinite(j[keys[i]].asDouble()))return false;*fields[i]=j[keys[i]].asDouble();}return true;
}
struct Core {
  std::string reason; bool has=false;double received=0.,stamp=0.;
  bool trip(const std::string&r){if(reason.empty())reason=r;return false;}
  bool update(const Json::Value&j,double mono,double ros){
    Sample s;
    if(!parse(j,s))return trip("missing_or_nonfinite_health");
    if(!std::isfinite(mono)||!std::isfinite(ros))return trip("invalid_clock");
    if(has){
      if(mono<received||s.stamp<=stamp)return trip("nonincreasing_health_stamp");
      if(mono-received>.50)return trip("health_receipt_gap");
    }
    if(ros-s.stamp<-.02||ros-s.stamp>.50)return trip("health_source_timeout");
    if(s.fault!=0.)return trip("vesc_fault");
    if(!(s.fet>0.&&s.fet<60.))return trip("fet_temperature_invalid_or_hot");
    if(s.voltage<10.5)return trip("low_voltage");
    if(std::abs(s.erpm)>15750.)return trip("overspeed");
    if(std::abs(s.duty)>1.)return trip("invalid_duty");
    has=true;received=mono;stamp=s.stamp;return reason.empty();
  }
  bool healthy(double mono,double ros){
    if(!reason.empty()||!has)return false;
    if(!std::isfinite(mono)||!std::isfinite(ros))return trip("invalid_clock");
    if(mono<received||mono-received>.50)return trip("health_receipt_timeout");
    if(ros-stamp<-.02||ros-stamp>.50)return trip("health_source_timeout");
    return true;
  }
};
}
