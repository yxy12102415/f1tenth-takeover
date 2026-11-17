"""Non-actuating causal constraint isolation using the preserved diagnostic probe."""
import json,subprocess,hashlib
from pathlib import Path
R=Path(__file__).resolve().parent
r=json.loads((R/'reports/mpc_wheels/records.json').read_text());plans={p['plan_id']:p for p in r['planner'] if p.get('valid')}
controls=[c for c in r['controller'] if c.get('reason') in ('ok','joint_mpc_solver_4') and c.get('control_input_snapshot',{}).get('plan_id') in plans]
results=[]
for mask in [0,1,2,4,8,15]:
 p=subprocess.Popen([str(R/'bin/diagnostic_probe'),str(R/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
 out={}
 try:
  for k,c in enumerate(controls):
   i=c['control_input_snapshot'];plan=plans[i['plan_id']]
   payload=dict(i,op='joint',type='dynamic_mpc',path=plan['path'],obstacles=plan.get('obstacle_points',[]),reset=k==0,feasibility_recovery=True,mpc_tracking_v10=json.loads((R/'compact_config.json').read_text())['mpc_tracking_v10'],diagnostic_mask=mask if k==len(controls)-1 else 0)
   p.stdin.write(json.dumps(payload)+'\n');p.stdin.flush()
   for _ in range(20):
    line=p.stdout.readline()
    try:out=json.loads(line);break
    except ValueError:continue
   if not out.get('ok'):break
 finally:p.stdin.close();p.wait(timeout=5)
 results.append(dict(mask=mask,replayed_calls=k+1,ok=out.get('ok'),error=out.get('error'),diagnostics=out.get('rejected_diagnostics',out.get('diag'))))
 print(mask,k+1,out.get('ok'),out.get('error'),flush=True)
(R/'reports/qp_constraint_isolation.json').write_text(json.dumps(dict(scope='Diagnostic probe only, constraints never changed in a vehicle binary',probe_sha256=hashlib.sha256((R/'bin/diagnostic_probe').read_bytes()).hexdigest(),results=results),indent=2)+'\n')
