#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
namespace frenet {
struct SpeedTransition {
  std::array<double,25> upper{};
  std::array<double,24> acceleration{};
  bool active=false;
  unsigned recovery_stages=0;
};
// A lower preview limit is a reachable braking objective, not an impossible
// instantaneous x[command_speed] bound. Absolute speed and physical guards
// remain independent. The generated model uses 50 ms; its future jerk bounds
// are 0.15 m/s^2 per step, with 4*actual_dt on the applied first input.
inline SpeedTransition reachable_speed_transition(double command,double previous_a,
    double requested,double absolute,double floor,double actual_dt) {
  if(!std::isfinite(command+previous_a+requested+absolute+floor+actual_dt) ||
     floor<0 || requested<floor || requested>absolute || absolute>2.000001 ||
     command<floor-1e-6 || command>absolute+1e-6 || previous_a<-.800001 || previous_a>1.000001 ||
     actual_dt<=0 || actual_dt>.050001)
    throw std::runtime_error("speed_transition_input");
  SpeedTransition result;result.upper[0]=command;double v=command,a=previous_a;
  auto release_minimum=[](double velocity,double acceleration) {
    velocity+=.05*acceleration;double minimum=velocity;
    for(int n=0;n<8 && acceleration<0;++n){acceleration=std::min(0.,acceleration+.15);velocity+=.05*acceleration;minimum=std::min(minimum,velocity);}
    return minimum;
  };
  for(unsigned stage=0;stage<24;++stage) {
    double change=stage?.15:4*actual_dt;
    double low=std::max(-.8,a-change),high=std::min(1.,a+change);
    // Preserve a feasible return to the running floor. Blind maximum braking
    // can otherwise arrive at the floor with negative acceleration that cannot
    // be removed under the next jerk bound.
    if(release_minimum(v,high)<floor-1e-9)throw std::runtime_error("speed_transition_floor_unreachable");
    if(release_minimum(v,low)<floor) {
      for(int n=0;n<45;++n){double mid=.5*(low+high);if(release_minimum(v,mid)<floor)low=mid;else high=mid;}
      low=high;
    }
    a=low;v+=.05*a;
    if(v>absolute+1e-6)throw std::runtime_error("speed_transition_absolute_unreachable");
    result.acceleration[stage]=a;
    result.upper[stage+1]=std::min(absolute,std::max(requested,v+1e-6));
    if(v>requested+1e-6){result.active=true;result.recovery_stages=stage+1;}
  }
  return result;
}
} // namespace frenet
