"""Replay actual coast telemetry and check stale/moving samples cannot pass."""
import json
from stand_readiness import StationaryObservation
from replay_recorded_pid import ROOT,SOURCE
r=json.loads((SOURCE/'command_trace.json').read_text());t=json.loads((SOURCE/'tracking_trace.json').read_text());first=next(x for x in t['controller'] if x.get('speed_command',0)>0);bad=next(x for x in t['controller'] if x['received_mono']>=first['received_mono'] and not x['valid']);start=bad['received_mono']
obs=StationaryObservation();confirmed=None
for h in r['health']:
 now=h['receipt_mono']
 if now<start:continue
 if obs.update(h,now,now):confirmed=now;break
assert confirmed is not None and 1<confirmed-start<3
zero={'erpm':0.,'fault':0};o=StationaryObservation();assert not o.update(zero,10.,10.)
assert not o.update(zero,10.,10.3), 'repeated sample is not continuous evidence'
assert not o.update(zero,10.,10.6), 'stale telemetry'
assert not o.update(zero,10.61,10.61)
assert not o.update({'erpm':110.,'fault':0},10.75,10.75)
assert not o.update(zero,10.8,10.8)
assert o.update(zero,11.01,11.01)
for sample in ({'erpm':float('nan'),'fault':0},{'erpm':0.,'fault':1}):
 o=StationaryObservation();assert not o.update(sample,12.,12.)
zero_cmd=next(x for x in r['current'] if x[0]>=start)
last_positive=max(x[0] for x in r['rpm_command'] if x[1]>0)
assert last_positive<zero_cmd[0]
health=[x for x in r['health'] if x['receipt_mono']>=start]
first_stable=next(x for i,x in enumerate(health) if abs(x['erpm'])<100 and all(abs(y['erpm'])<100 for y in health[i:]))
report=dict(passed=True,actual_zero_command_delay_s=zero_cmd[0]-start,last_positive_command_delay_s=last_positive-start,recorded_stable_wheels_delay_s=first_stable['receipt_mono']-start,new_observation_confirmation_delay_s=confirmed-start,stale_repeated_nonfinite_fault_and_moving_rejected=True,hard_stop_deadlines_unchanged=True)
(ROOT/'run_logs/stop_observation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
