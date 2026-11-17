"""Offline bounded-memory stress with captured stationary real URG scans.
Uses the prior stationary PF median, not surveyed ground truth. No ROS/drivers.
"""
import json,statistics,subprocess,time,collections
from pathlib import Path
r=Path(__file__).resolve().parent
scans=json.loads((r/'reports/live_scan_capture/scans.json').read_text())
prior=r/'run_logs/20261111_153304-closed-frenet-ground-zero/tracking_trace.json'
pf=json.loads(prior.read_text())['poses'][-200:]
pose=[statistics.median(p[k] for p in pf) for k in ['x','y','yaw']]
p=subprocess.Popen([str(r/'bin/generic_probe'),str(r/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
records=[]
try:
 for k in range(300):
  scan=scans[(k*8)%len(scans)]
  payload=dict(scan,stamp=1000+k*.2,poses=[pose],speed=0.)
  t=time.monotonic();p.stdin.write(json.dumps(payload)+'\n');p.stdin.flush()
  line=p.stdout.readline();assert line,'probe exited';out=json.loads(line)
  records.append(dict(frame=k,valid=out.get('valid',False),reason=out.get('reason'),occupancy_count=len(out.get('occupancies',[])),ego_removed=out.get('perception',{}).get('ego_body_returns_removed'),compute_ms=(time.monotonic()-t)*1000))
  if k%50==0:print(records[-1],flush=True)
  if not out.get('valid'):(r/'reports/live_scan_capture/last_rejected.json').write_text(json.dumps(dict(input=payload,result=out),indent=2)+'\n')
finally:p.stdin.close();p.wait(timeout=5)
report=dict(physical_motion=False,scope=__doc__,pose_estimate=pose,captured_scan_count=len(scans),cycled_scan_frames=300,synthetic_track_time_s=60.,reasons=dict(collections.Counter(v['reason'] for v in records)),max_occupancies=max(v['occupancy_count'] for v in records),max_compute_ms=max(v['compute_ms'] for v in records),last_5s_valid=all(v['valid'] for v in records[-25:]),ego_returns_removed_max=max(v['ego_removed'] or 0 for v in records))
report['passed']=report['last_5s_valid'] and report['max_occupancies']<=128
(r/'reports/live_scan_capture/replay.json').write_text(json.dumps(report,indent=2)+'\n');(r/'reports/live_scan_capture/replay_frames.json').write_text(json.dumps(records)+'\n')
print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed'] else 1)
