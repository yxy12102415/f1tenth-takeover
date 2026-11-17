#pragma once
#include "core.hpp"
namespace frenet {
// Retain a stationary, validated path. Every observation still revalidates its
// geometry. Refresh the initial anchor only after actual position/heading
// changes, not because a wall-clock timer expired.
class PrearmPathRefresh {
  bool anchored=false;Pose anchor;
public:
  bool update(bool enabled,double speed,Pose pose) {
    if(enabled || std::abs(speed)>=.03)return false;
    if(!std::isfinite(speed+pose.x+pose.y+pose.yaw))throw std::runtime_error("prearm_pose_invalid");
    if(!anchored || (pose.xy()-anchor.xy()).norm()>.03 || std::abs(wrap(pose.yaw-anchor.yaw))>.05) {
      anchor=pose;anchored=true;return true;
    }
    return false;
  }
};
} // namespace frenet
