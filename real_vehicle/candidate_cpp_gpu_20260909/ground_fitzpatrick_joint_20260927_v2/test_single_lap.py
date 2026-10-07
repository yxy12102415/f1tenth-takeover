"""Offline lap identity, start/end ambiguity and 90s window tests."""
import json
import subprocess
from pathlib import Path
from stand_window import StandWindow
ROOT=Path(__file__).resolve().parent
source=r'''
#include "src/cpp_frenet/core.hpp"
#include <cassert>
int main(int argc,char**argv) {
  frenet::Assets a(argv[1]);assert(a.closed_single_lap);
  frenet::Planner planner(a);frenet::Progress lap;
  lap.update(planner.nearest(a.route.front()),a);
  assert(lap.distance==0 && planner.progress==0);
  assert(frenet::endpoint_match(a,a.route.front())); // physical overlap, not completion
  planner.nearest(a.route.back());assert(planner.progress==0);
  for(int i=4;i<int(a.route.size());i+=4) {
    int n=planner.nearest(a.route[i]);lap.update(n,a);
    assert(n==i && std::abs(lap.distance-a.arc[i])<1e-8);
  }
  lap.update(planner.nearest(a.route.back()),a);
  assert(std::abs(lap.distance-a.arc.back())<1e-8);
  assert(planner.nearest(a.route.front())==int(a.route.size())-1);
  frenet::Progress bad;bool rejected=false;
  try {bad.update(100,a);}catch(const std::runtime_error&){rejected=true;}
  assert(rejected);
}
'''
(ROOT/'run_logs/lap_contract.cpp').write_text(source)
subprocess.run(['g++','-std=c++14','-O2','-I.','-I/usr/include/eigen3',
 'run_logs/lap_contract.cpp','-ljsoncpp','-o','run_logs/lap_contract'],cwd=ROOT,check=True)
subprocess.run([str(ROOT/'run_logs/lap_contract'),str(ROOT/'runtime_assets.json')],check=True)
w=StandWindow();w.joy(False,0.)
for i in range(4501):
 now=i*.02;w.health(True,now);w.permit(True,now);w.joy(True,now);w.command(1.,0.,now)
 v,_,why=w.evaluate(now)
 if now<90:assert not why and v>0.,(now,why)
 else:assert v==0 and why=='route_time_limit'
report=dict(passed=True,start_not_mistaken_for_finish=True,progress_never_wraps=True,
 incorrect_initial_progress_rejected=True,motion_window_s=90.,time_limit_stop_verified=True)
(ROOT/'run_logs/single_lap_contract.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report))
