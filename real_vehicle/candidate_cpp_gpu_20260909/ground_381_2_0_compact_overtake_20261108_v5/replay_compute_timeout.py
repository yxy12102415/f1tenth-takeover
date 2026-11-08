"""Replay real controller inputs through the first timeout, without hardware.

This reuses recorded poses/measurements (not a closed-loop plant), the original
assets and local-path snapshots. It checks solver continuity on real inputs;
separate injected ROS tests cover deterministic wall-time overruns.
"""
import json
import os
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT.parent/'ground_381_2_0_extended_dmpc_20261107_v2'
LOG=SOURCE/'run_logs/20261107_124102-closed-frenet-ground-attempt'
data=json.loads((LOG/'tracking_trace.json').read_text())['controller']
env=os.environ.copy();env['LD_LIBRARY_PATH']=str(ROOT/'lib')+':'+env.get('LD_LIBRARY_PATH','')
proc=subprocess.Popen([str(ROOT/'bin/core_probe'),str(SOURCE/'runtime_assets.json')],
    stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,env=env)
paths={};rows=[];reached=False
try:
    for item in data:
        if 'local_path_snapshot' in item:paths[item['plan_id']]=item['local_path_snapshot']
        v=item.get('control_input_snapshot')
        if not v:continue
        diag=item.get('tracker_diagnostics') or item.get('rejected_control',{}).get('diagnostics') or {}
        if diag.get('longitudinal_controller')!='joint_dynamic_mpc':continue
        args={k:v[k] for k in ('pose','speed','lateral','yaw_rate','remaining','cap','dt')}
        args.update(op='joint',type='dynamic_mpc',reset=not rows,path=paths[v['plan_id']],obstacles=item['obstacle_points'])
        proc.stdin.write(json.dumps(args)+'\n');proc.stdin.flush()
        for _ in range(20):
            line=proc.stdout.readline()
            if not line:raise RuntimeError('native replay exited')
            try:result=json.loads(line);break
            except ValueError:continue
        assert result['ok'],result
        rd=result['diag']
        rows.append(dict(received_mono=item['received_mono'],original_reason=item['reason'],
                         original_compute_ms=diag['controller_compute_ms'],
                         replay_speed=result['speed'],replay_steering=result['angle'],
                         replay_compute_ms=rd['controller_compute_ms'],
                         equality_residual=rd.get('equality_residual'),inequality_residual=rd.get('inequality_residual')))
        if item['reason']=='joint_control_deadline':reached=True;break
    assert reached and len(rows)>20, 'did not replay the original failure'
    assert rows[-1]['replay_speed']>.8,'command unexpectedly reset'
    report=dict(passed=True,physical_motion=False,recorded_input_replay=True,
                source=str(LOG),samples=len(rows),original_failed_compute_ms=rows[-1]['original_compute_ms'],
                replay_final_command_mps=rows[-1]['replay_speed'],max_replay_compute_ms=max(r['replay_compute_ms'] for r in rows))
    print(json.dumps(report),flush=True)
    (ROOT/'run_logs/recorded_compute_replay.json').write_text(json.dumps(dict(summary=report,rows=rows),indent=2)+'\n')
finally:
    proc.stdin.close();proc.wait(timeout=5)
