import json
import math
from pathlib import Path
import subprocess
import time
import numpy as np
from baseline_pf_pp.core import PurePursuitCore
from f1tenth_safety_migration.route_mpc_core import load_open_route

ROOT=Path(__file__).resolve().parent
ROUTE=ROOT.parent/'routes/manual_route_20260904_215501_controller_ready.csv'
BIN=ROOT/'install_v1/candidate_pp/lib/candidate_pp/core_probe'

def main():
    route=load_open_route(str(ROUTE));errors=[];count=0
    for scenario in ('route','cross_track','heading','wrong_start'):
        states=[]
        for p in route:
            q=p.copy()
            if scenario=='cross_track':q[:2]+=[-.28*math.sin(q[2]),.28*math.cos(q[2])]
            elif scenario=='heading':q[2]+=.8
            elif scenario=='wrong_start':q[0]+=2.
            states.append(q)
        cpp=subprocess.run([str(BIN),str(ROUTE)],input=''.join('%0.17g %0.17g %0.17g 0 .05\n'%tuple(q) for q in states),text=True,capture_output=True,check=True)
        py=PurePursuitCore(route)
        for q,line in zip(states,cpp.stdout.splitlines()):
            vals=line.split(',');d=py.step(*q,0.)
            assert bool(int(vals[2]))==d.valid and bool(int(vals[3]))==d.complete,(scenario,line,d)
            assert int(vals[4])==d.progress_index and vals[5]==d.reason,(scenario,line,d)
            errors.append(max(abs(float(vals[0])-d.speed),abs(float(vals[1])-d.steering)))
            count+=1
        assert len(cpp.stdout.splitlines())==len(states)
    assert max(errors)<1.e-10,max(errors)
    result={'passed':True,'comparisons':count,'max_command_error':max(errors),'route_rows':len(route)}
    (ROOT/'core_comparison.json').write_text(json.dumps(result,indent=2)+'\n');print(result)
if __name__=='__main__':main()
