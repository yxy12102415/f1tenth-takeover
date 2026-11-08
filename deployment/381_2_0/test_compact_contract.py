#!/usr/bin/env python3
"""Non-actuating native tests of short-course admission and occupied bounds."""
import json,math,subprocess,os
from pathlib import Path
import numpy as np
D=Path(__file__).resolve().parent
RELEASE=Path(os.environ.get('COMPACT_RELEASE','/home/f1tenth/f1tenth_real_migration_ws_20260829/candidate_cpp_gpu_20260909/ground_381_2_0_compact_overtake_20261107_v1'))
assets=json.loads((RELEASE/'runtime_assets.json').read_text())
origin=np.array(assets['route'][0]);c,s=math.cos(origin[2]),math.sin(origin[2])
def pose(x,y=0,h=0):return [origin[0]+c*x-s*y,origin[1]+s*x+c*y,origin[2]+h]
def track(center=1.6,side='rear',ident=1):
    x=center+(.15 if side=='front' else -.15)
    return {'id':ident,'points':[pose(x,y)[:2] for y in [-.09,0,.09]]}
class Probe:
    def __init__(self):self.p=subprocess.Popen([str(D/'build/checks/compact_probe'),str(RELEASE/'runtime_assets.json'),str(RELEASE/'compact_config.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
    def ask(self,q):
        self.p.stdin.write(json.dumps(q)+'\n');self.p.stdin.flush();return json.loads(self.p.stdout.readline())
    def close(self):self.p.stdin.close();assert self.p.wait(timeout=5)==0
    def prime(self,center=1.6):
        for k in range(7):r=self.ask(dict(stamp=100+k*.2,pose=[pose(0)],tracks=[track(center)]))
        return r
def main():
    results=[]
    def record(name,ok,detail):
        results.append(dict(name=name,passed=bool(ok),detail=detail));print(json.dumps(results[-1]),flush=True)
    p=Probe();r=p.prime();record('full_maneuver_admission',r.get('ok'),r)
    if not r.get('ok'):
        p.close();(D/'reports/compact_contract.json').write_text(json.dumps(results,indent=2)+'\n');raise SystemExit(1)
    lead=r['lead_bounds']
    r=p.ask(dict(stamp=101.4,pose=[pose(0)],tracks=[track()],enable=True));record('enable_after_admission',r.get('ok'),r)
    r=p.ask(dict(stamp=101.8,pose=[pose(.1)],tracks=[],enable=True));record('target_loss_stops',r.get('error')=='compact_target_lost',r)
    r=p.ask(dict(stamp=102.,pose=[pose(.1)],tracks=[track()],enable=True));record('loss_cannot_auto_resume',r.get('error')=='compact_latched_failure',r);p.close()
    p=Probe();r=p.prime(center=.8);record('original_short_gap_refused',not r.get('ok'),r);p.close()
    p=Probe();r=p.ask(dict(stamp=100.,pose=[pose(0)],tracks=[track()],enable=True));record('no_enable_before_admission',r.get('error')=='compact_not_admitted',r);p.close()
    p=Probe()
    separated=[dict(id=i+1,points=[pose(1.45,y+dy)[:2] for dy in [-.015,0,.015]]) for i,y in enumerate([-.18,.18])]
    r=p.ask(dict(stamp=100.,pose=[pose(0)],tracks=separated));record('ambiguous_target_refused',r.get('error')=='compact_target_ambiguous',r);p.close()
    p=Probe();p.prime();r=p.ask(dict(stamp=101.4,pose=[pose(0)],tracks=[track(),track(1.6,ident=2)]));record('compatible_body_fragments_grouped',r.get('ok'),r);p.close()
    p=Probe();p.prime();p.ask(dict(stamp=101.4,pose=[pose(0)],tracks=[track()],enable=True))
    r=p.ask(dict(stamp=102.,pose=[pose(2.9,-.5)],tracks=[],enable=True))
    record('rear_blind_sector_keeps_old_source',r.get('ok') and r.get('lead_bounds',{}).get('stamp')==101.4 and r.get('lead_bounds',{}).get('passed'),r)
    if r.get('ok'):
        r=p.ask(dict(op='check',lead=r['lead_bounds'],prediction=[pose(2.6)],now=104.))
        record('rear_prediction_still_rejects_reachable_collision',r.get('error')=='compact_dynamic_body_conflict',r)
    p.close()
    p=Probe()
    for name,pred,now,reason in [
      ('near_body_collision',[pose(1.5)],101.3,'compact_dynamic_body_conflict'),
      ('source_stale',[pose(0)],102.,'compact_target_stale'),
      ('future_stamp',[pose(0)],100.,'compact_target_stale'),
      ('clear_lateral',[pose(1.5,-.65)],101.3,None),
      ('between_samples_collision',[pose(.7),pose(2.3)],101.3,'compact_dynamic_body_conflict')]:
        r=p.ask(dict(op='check',lead=lead,prediction=pred,now=now));record(name,r.get('ok') if reason is None else r.get('error')==reason,r)
    p.close()
    (D/'reports/compact_contract.json').write_text(json.dumps(results,indent=2)+'\n')
    raise SystemExit(0 if all(r['passed'] for r in results) else 1)
if __name__=='__main__':main()
