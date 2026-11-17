"""Native occupancy regressions; no ROS or hardware."""
from pathlib import Path
import json,subprocess,hashlib
ROOT=Path(__file__).resolve().parent

def main():
 r=subprocess.run([str(ROOT/'bin/test_dynamic_memory'),str(ROOT/'runtime_assets.json')],capture_output=True,text=True,check=True)
 report=json.loads(r.stdout)
 child=subprocess.Popen([str(ROOT/'bin/generic_probe'),str(ROOT/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
 try:
  samples=[]
  for k in range(60):
   # A stationary front face becomes a visible side as ego viewpoint changes.
   # Its near edge stays fixed: this must not invent forward target motion.
   far=1.65+.30*min(1,k/30)
   points=[[1.65,-.15],[1.65,.15],[far,-.15],[far,.15]]
   req=dict(stamp=100+k*.05,poses=[[-.0047675,.0098282,.0188502]],clusters=[dict(id=k%7,points=points)],allow_search=False,free_range=.3,cap=2.15,max_offset=.65)
   child.stdin.write(json.dumps(req)+'\n');child.stdin.flush();v=json.loads(child.stdout.readline());samples.extend(v.get('occupancies',[]))
  assert samples and max(abs(v['vx']) for v in samples)<.03,'shape revelation creates false velocity'
  assert not any(v['has_moved'] for v in samples),'shape revelation creates false moving classification'
  report['stationary_side_revelation']=True
 finally:
  child.stdin.close();child.wait(timeout=5)
 child=subprocess.Popen([str(ROOT/'bin/generic_probe'),str(ROOT/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
 try:
  phase=[]
  for k in range(65):
   dt=k*.05;reveal=min(.25,max(0.,dt-1.)*.25);x=1.5+.5*dt
   points=[[x-reveal,-.15],[x-reveal,.15],[x+.10,-.15],[x+.10,.15]]
   req=dict(stamp=200+dt,poses=[[-.0047675,.0098282,.0188502]],clusters=[dict(id=k%7,points=points)],allow_search=False,free_range=.3,cap=2.15,max_offset=.65)
   child.stdin.write(json.dumps(req)+'\n');child.stdin.flush();v=json.loads(child.stdout.readline())
   if 1.2<dt<2.5:phase.extend(o['vx'] for o in v.get('occupancies',[]) if o['memory_state']=='observed')
  assert phase and min(phase)>.4,'moving side revelation corrupts established velocity'
  report['moving_side_revelation']=True
 finally:
  child.stdin.close();child.wait(timeout=5)
 report['physical_motion']=False
 report['source_hashes']={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in [ROOT/'src/cpp_frenet/test_dynamic_memory.cpp',ROOT/'src/cpp_frenet/tracked_occupancy.hpp',ROOT/'src/cpp_frenet/core.hpp',ROOT/'src/cpp_frenet/generic_avoidance.hpp']}
 report['binary_hashes']={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in [ROOT/'bin/generic_probe',ROOT/'bin/test_dynamic_memory']}
 dest=ROOT/'reports/dynamic_preparation/native_memory.json';dest.parent.mkdir(parents=True,exist_ok=True);dest.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
