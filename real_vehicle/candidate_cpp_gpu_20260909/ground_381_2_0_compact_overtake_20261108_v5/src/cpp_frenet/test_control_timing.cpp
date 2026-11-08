#include "control_timing.hpp"
#include <cassert>
#include <cstring>
#include <limits>
#include <iostream>
int main() {
  frenet::MpcTimingBudget p;
  // Recorded successful solves surrounding the first real-run stop.
  for(double ms: {12.758164, 9.159621, 46.271098, 30.024440, 51.732624})
    assert(p.observe(ms/1000.)==nullptr);
  p.reset();
  assert(!p.observe(.060));assert(!p.observe(.060));
  assert(std::strcmp(p.observe(.060),"joint_control_overload")==0);
  p.reset();assert(p.consecutive()==0);
  assert(!p.observe(.075));
  assert(std::strcmp(p.observe(.075001),"joint_control_deadline")==0);
  p.reset();assert(!p.observe(.046));assert(!p.observe(.045));assert(p.consecutive()==0);
  assert(std::strcmp(p.observe(std::numeric_limits<double>::quiet_NaN()),"joint_control_timing_invalid")==0);
  std::cout << "Recorded jitter accepted; sustained/hard overruns rejected; reset and boundaries passed\n";
}
