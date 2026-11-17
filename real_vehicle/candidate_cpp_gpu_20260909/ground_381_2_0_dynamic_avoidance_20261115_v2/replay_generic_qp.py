"""Replay actual saved synthetic controller inputs; never touches ROS/drivers."""
import json
from pathlib import Path
from validate_pid_tuning import Native
ROOT=Path(__file__).resolve().parent
r=json.loads((ROOT/'reports/mpc_wheels/records.json').read_text())
plans={p['plan_id']:p for p in r['planner'] if p.get('valid')}
n=Native(ROOT);count=0;resets=0;failure=None
try:
    for c in r['controller']:
        if c.get('reason') not in ('ok','joint_mpc_solver_4'):continue
        i=c.get('control_input_snapshot',{})
        if not i or i.get('plan_id') not in plans:continue
        p=plans[i['plan_id']]
        payload=dict(i,op='joint',type='dynamic_mpc',path=p['path'],obstacles=p.get('obstacle_points',[]),reset=count==0,feasibility_recovery=True,mpc_tracking_v10=json.loads((ROOT/'compact_config.json').read_text())['mpc_tracking_v10'])
        count+=1
        try:
            n.proc.stdin.write(json.dumps(payload)+'\n');n.proc.stdin.flush()
            for _ in range(20):
                line=n.proc.stdout.readline()
                try:out=json.loads(line);break
                except ValueError:continue
            if not out.get('ok'):
                failure=out;break
        except RuntimeError as e:
            failure={'error':str(e),'call':count,'input':i};break
        if 'speed' not in out:failure=out;break
        resets+=bool(out.get('diag',{}).get('qp_memory_reset_retry'))
finally:n.close()
report=dict(replayed_calls=count,qp_memory_resets=resets,passed=failure is None,failure=failure,scope='Saved synthetic MPC wheel-case inputs, not physical data')
(ROOT/'reports/qp_input_replay.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed'] else 1)
