"""Native control contract checks; no ROS and no hardware."""
import json
from pathlib import Path
from validate_joint_rollout import Native, ROOT

assets=json.loads((ROOT/'runtime_assets.json').read_text())
pose=assets['route'][0]
reports=[]
for kind in ('pid','dynamic_mpc'):
    n=Native()
    try:
        plan=n.ask(dict(op='plan',reset=True,pose=pose,obstacles=[],horizon=5.84))
        base=dict(op='joint',type=kind,path=plan['path'],pose=pose,speed=0.,lateral=0.,yaw_rate=0.,remaining=5.84,dt=.05,obstacles=[])
        previous=0.;prev_a=0.;prev_delta=0.;commands=[]
        for i in range(100):
            r=n.ask(dict(base,reset=i==0,dt=.041 if i%3==0 else .05))
            a=r['diag']['acceleration_command_mps2'];delta=r['angle']
            assert 0<=r['speed']<=2.0 and abs(delta)<=.26901
            assert abs(r['speed']-previous)<=.0501
            assert -.80001<=a<=1.00001 and abs(a-prev_a)<=.2001
            assert abs(delta-prev_delta)<=.0551
            previous=r['speed'];prev_a=a;prev_delta=delta;commands.append(r['speed'])
        assert max(commands)>1.90,('zero_feedback_cannot_build_launch_command',kind,commands)
        r=n.ask(dict(base,reset=True));assert r['speed']<=.0101
        rejected=[]
        for name,changes in [('invalid_dt',{'dt':0.}),('invalid_speed',{'speed':2.5}),('invalid_yaw_rate',{'yaw_rate':9.}),('obstacle_at_vehicle',{'obstacles':[[pose[0],pose[1]]]})]:
            try:n.ask(dict(base,reset=True,**changes))
            except RuntimeError:rejected.append(name)
            else:raise AssertionError(('must_reject',kind,name))
        reports.append(dict(controller=kind,passed=True,zero_speed_launch_command_max=max(commands),invalid_inputs_rejected=rejected))
    finally:n.close()
(ROOT/'run_logs/joint_contract.json').write_text(json.dumps(reports,indent=2)+'\n')
print(json.dumps(reports,indent=2))
