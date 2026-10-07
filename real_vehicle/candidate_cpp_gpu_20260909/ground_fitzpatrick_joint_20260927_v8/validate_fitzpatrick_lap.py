"""Native full-lap closed-loop model checks, no ROS or hardware."""
import json
from pathlib import Path
from validate_pid_tuning import rollout,ROOT
reports=[]
for kind in ('pid','dynamic_mpc'):
    for name,lag,servo,offset,heading in [('nominal',.20,.20,0.,0.),('offset',.25,.20,.02,.02)]:
        result,trace=rollout(ROOT,False,lag=lag,servo_lag=servo,offset=offset,heading_offset=heading,kind=kind)
        result.update(controller=kind,case=name,
            max_command_mps=max((r['command'] for r in trace),default=0),
            maximum_cross_track_error_m=max((abs(r['diag'].get('cross_track_error_m',0.)) for r in trace),default=0),
            last_progress_m=max((r['progress'] for r in trace),default=0))
        assert result['max_command_mps']<=2.00001
        if result['passed']:assert result['last_progress_m']>44.
        reports.append(result)
        (ROOT/'run_logs'/f'fitzpatrick_lap_{kind}_{name}.json').write_text(json.dumps(dict(summary=result,trace=trace))+'\n')
        print(json.dumps(result),flush=True)
(ROOT/'run_logs/fitzpatrick_lap_models.json').write_text(json.dumps(reports,indent=2)+'\n')
if not all(r['passed'] for r in reports):raise SystemExit(1)
