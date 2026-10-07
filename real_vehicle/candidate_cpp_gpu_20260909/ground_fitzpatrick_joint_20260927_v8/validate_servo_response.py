"""Independent plant response uncertainty, not actuator access."""
import json
from validate_pid_tuning import rollout,ROOT
reports=[]
for name,lag,servo,gain in [('fast',.05,.10,1.),('slow',.25,.30,1.),('steering_gain',.2,.2,1.10)]:
 result,trace=rollout(ROOT,False,lag=lag,servo_lag=servo,steering_gain=gain,kind='dynamic_mpc')
 result['case']=name;reports.append(result)
 (ROOT/'run_logs'/('servo_response_'+name+'.json')).write_text(json.dumps(dict(summary=result,trace=trace))+'\n');print(json.dumps(result),flush=True)
(ROOT/'run_logs/servo_response_summary.json').write_text(json.dumps(reports,indent=2)+'\n')
assert all(r['passed'] for r in reports)
