import json,subprocess
from pathlib import Path
R=Path(__file__).resolve().parents[1];snap=json.loads((R/'review/current_placement_failed_snapshot.json').read_text());v=snap['controller'];case=json.loads((R/'review/current_placement_long_budget.json').read_text());p=subprocess.Popen([str(R/'bin/generic_probe'),str(R/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
def ask(req):
 p.stdin.write(json.dumps(req)+'\n');p.stdin.flush();return json.loads(p.stdout.readline())
pose=[snap['pose'][k] for k in ['x','y','yaw']];hits=[]
for c in [case['candidates'][1],case['candidates'][19]]:
 times=ask(dict(stamp=v['source_scan_stamp'],poses=[pose],arrival_path=c['path'],speed=0,arrival_horizon=1.6,map_speed_feasibility=True))['times']
 for o in v['occupancies']:
  # First locate any measured-support conflict, with no future process growth.
  for j,q in enumerate(c['path']):
   if times[j]>1.2:continue
   out=ask(dict(stamp=v['source_scan_stamp']+.30,poses=[pose],check_prediction=[q]*(1+int(times[j]/.05+.5)),occupancies=[o]))
   if out.get('reason')=='generic_predicted_occupancy_conflict':
    hits.append(dict(offset=c['offset_m'],path_index=j,ego=q,occupancy=o,arrival_s=times[j],mode='future_growth_isolation'));break
p.stdin.close();p.wait();(R/'review/current_conflict_objects.json').write_text(json.dumps(hits,indent=2));print(json.dumps(hits,indent=2))
