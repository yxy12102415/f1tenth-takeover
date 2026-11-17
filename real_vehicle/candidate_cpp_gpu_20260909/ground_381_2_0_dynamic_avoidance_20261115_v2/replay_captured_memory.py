"""Replay real stationary LiDAR frames through native bounded memory; no actuation."""
from pathlib import Path
import json,subprocess,hashlib
R=Path(__file__).resolve().parent
OLD=R.parent/'ground_381_2_0_generic_avoidance_20261111_v1'
source=OLD/'reports/live_scan_capture/scans.json'
scans=json.loads(source.read_text());pose=json.loads((R/'stationary_memory_fixture_20261115.json').read_text())['pose']
p=subprocess.Popen([str(R/'bin/generic_probe'),str(R/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
counts=[];last=None;velocities=[]
try:
 for k in range(300):
  req=dict(scans[k%len(scans)],stamp=1000+k*.2,poses=[pose],speed=0.,allow_search=k==299,cap=2.15,max_offset=.65,map_speed_feasibility=True)
  p.stdin.write(json.dumps(req)+'\n');p.stdin.flush();last=json.loads(p.stdout.readline());occ=last.get('occupancies',[]);counts.append(len(occ));velocities.extend((o['vx']**2+o['vy']**2)**.5 for o in occ)
finally:p.stdin.close();p.wait(timeout=5)
r=dict(passed=max(counts)<=128,physical_motion=False,scope=__doc__,captured_frames=len(scans),cycled_frames=300,synthetic_elapsed_s=60.,pose_assumption=pose,pose_is_surveyed_ground_truth=False,source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),max_occupancies=max(counts),final_occupancies=counts[-1],max_estimated_speed_mps=max(velocities,default=0),final_path_valid=last.get('valid'),final_path_reason=last.get('reason'))
(R/'reports/dynamic_preparation/captured_memory_replay.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
