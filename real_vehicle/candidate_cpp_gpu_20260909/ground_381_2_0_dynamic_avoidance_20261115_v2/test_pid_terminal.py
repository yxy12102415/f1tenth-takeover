"""Endpoint geometry and stop-distance regressions using the native controller."""
import json,math
import numpy as np
from validate_pid_tuning import ROOT,Native
assets=json.loads((ROOT/'runtime_assets.json').read_text());p=np.asarray(assets['route']);last=p[-1];t=np.array([math.cos(last[2]),math.sin(last[2])]);normal=np.array([-t[1],t[0]]);xy=last[:2]-.06*t+.03*normal;pose=[*xy,last[2]+.02]
n=Native(ROOT)
try:
    outputs=[]
    for i in range(8):
        r=n.ask(dict(op='track',type='pid',reset=i==0,path=p[-25:].tolist(),pose=pose,speed=.2,dt=.05))
        outputs.append(r['angle'])
    assert max(map(abs,outputs))<.10,outputs
    base=dict(op='joint',type='pid',reset=True,path=p[-25:].tolist(),pose=pose,speed=.6,lateral=0.,yaw_rate=0.,dt=.05,obstacles=[])
    stale=n.ask(dict(base,remaining=.30));fresh=n.ask(dict(base,remaining=.06))
    assert stale['diag']['control_remaining_m']<.07,stale
    assert abs(stale['diag']['speed_target_mps']-fresh['diag']['speed_target_mps'])<.01
    assert stale['diag']['terminal_acceleration_feedforward_mps2']==0
    assert stale['diag']['speed_target_mps']==.5
    result=dict(passed=True,terminal_steering_max_rad=max(map(abs,outputs)),stale_planner_remaining_m=.30,projected_control_remaining_m=stale['diag']['control_remaining_m'],terminal_speed_target_mps=stale['diag']['speed_target_mps'])
finally:n.close()
(ROOT/'run_logs/pid_terminal_contract.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
