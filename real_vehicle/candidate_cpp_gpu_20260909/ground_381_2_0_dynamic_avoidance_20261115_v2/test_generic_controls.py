"""Non-actuating native control contract at this release's commissioning cap."""
import json
import numpy as np
from pathlib import Path
from validate_pid_tuning import Native
ROOT=Path(__file__).resolve().parent
a=json.loads((ROOT/'runtime_assets.json').read_text());route=np.array(a['route'])
arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(route[:,:2],axis=0),axis=1))]
reports=[]
for tracker,cap in [('pid',1.25),('dynamic_mpc',1.4)]:
    n=Native(ROOT);commands=[];floor=False;coast=False
    def call(pos,v,reset=False):
        pose=[float(np.interp(pos,arc,route[:,j])) for j in range(3)]
        r=n.ask(dict(op='joint',type=tracker,cap=cap,reset=reset,path=route.tolist(),pose=pose,speed=v,lateral=0,yaw_rate=0,remaining=float(arc[-1]-pos),dt=.05,obstacles=[],feasibility_recovery=True))
        assert 'speed' in r,r
        return r
    try:
        # Rest feedback deliberately remains zero: bounded launch and no
        # low-speed band after initial floor attainment must still hold.
        for i in range(80):
            r=call(.05,0,i==0);commands.append(r['speed'])
        assert max(commands)>.5 and max(commands)<=1.250001,(tracker,max(commands))
        assert max(np.diff(commands))<=.100001
        for pos in np.linspace(.05,arc[-1]-.12,220):
            r=call(float(pos),0);commands.append(r['speed'])
            coast=coast or r['diag'].get('terminal_coasting',False)
        assert coast and commands[-1]==0,(tracker,r)
        for pos in [arc[-1]-.4,arc[-1]-.2]:assert call(float(pos),0)['speed']==0
        for v in commands:
            if v>=.5:floor=True
            assert not(floor and 0<v<.5-1e-5),(tracker,v)
        reports.append(dict(tracker=tracker,passed=True,command_cap_mps=max(commands),maximum_step_mps=float(max(np.diff(commands))),terminal_zero_latched=True))
    finally:n.close()
from stand_window import StandWindow
w=StandWindow();w.joy(False,0.);floor_seen=False;actual=[]
for i in range(150):
    now=i*.02;w.health(True,now);w.permit(True,now);w.joy(True,now)
    w.command(1. if i<100 else .5,0.,now)
    v,_,why=w.evaluate(now);actual.append(v)
    if v>=.5:floor_seen=True
    assert not(floor_seen and 0<v<.5-1e-5)
assert actual[0]<=.040001 and actual[-1]==.5
w.command(0,0,3.);assert w.evaluate(3.)[0]==0
w.health(True,3.02);w.permit(True,3.02);w.joy(True,3.02);w.command(.5,0,3.02)
assert w.evaluate(3.02)[0]==0
(ROOT/'reports/control_contract.json').write_text(json.dumps(reports,indent=2)+'\n')
print(json.dumps(reports,indent=2))
