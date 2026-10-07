#pragma once
#include "health_core.hpp"
namespace health {
// Qualification only before the monitor has EVER issued a healthy permit.
// Source discovery gaps reset qualification, never a running or faulted core.
struct Startup {
  Core runtime;bool activated=false,fatal=false,cohort=false,seen=false;
  double start=0.,last_receipt=0.,last_stamp=0.;unsigned rejected=0;
  std::string failure;
  bool trip(const std::string&r){fatal=true;if(failure.empty())failure=r;return false;}
  bool update(const Json::Value&j,double mono,double ros){
    if(fatal)return false;
    if(activated)return runtime.update(j,mono,ros);
    Sample s;if(!parse(j,s))return trip("missing_or_nonfinite_health");
    if(!std::isfinite(mono)||!std::isfinite(ros))return trip("invalid_clock");
    // Validate numerical health even when source freshness is not qualified.
    Json::Value numeric=j;numeric["stamp"]=ros;Core check;
    if(!check.update(numeric,mono,ros))return trip(check.reason);
    bool increasing=!seen||(mono>=last_receipt&&s.stamp>last_stamp);
    bool recent=ros-s.stamp>=-.02&&ros-s.stamp<=.50;
    bool gap=!seen||mono-last_receipt<=.50;
    if(!increasing||!recent||!gap){cohort=false;++rejected;}
    if(!increasing||!recent)return false;
    seen=true;last_receipt=mono;last_stamp=s.stamp;
    if(!cohort){cohort=true;start=mono;}
    if(mono-start<1.0)return false;
    // This is the sole initialization of the runtime state machine.
    activated=true;return runtime.update(j,mono,ros);
  }
  bool healthy(double mono,double ros){
    if(fatal)return false;
    if(activated)return runtime.healthy(mono,ros);
    if(seen&&(mono<last_receipt||mono-last_receipt>.50||ros-last_stamp<-.02||ros-last_stamp>.50))cohort=false;
    return false;
  }
  std::string reason()const{return fatal?failure:activated?runtime.reason.empty()?"healthy":runtime.reason:"qualifying_fresh_health";}
};
}
